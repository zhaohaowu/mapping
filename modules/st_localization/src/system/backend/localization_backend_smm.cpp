/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#include "system/backend/localization_backend_smm.hpp"

#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "common/utility.hpp"
#include "eval/evaluator_localziation.hpp"
#include "localization/common/log.hpp"
#include "msf/msf_factor/msf_factor.hpp"
#include "msf/msf_fusion/msf_fusion.hpp"
#include "system/localization_dead_reckoning.hpp"
#include "system/localization_frontend.hpp"
#include "system/localization_visualizer.hpp"

namespace senseAD {
namespace localization {

void LocalizationBackendSMM::SetLocalMapData(
    uint64_t timestamp, const std::shared_ptr<RoadStructure>& local_map_data) {
  std::lock_guard<std::mutex> lock(local_map_data_mutex_);
  local_map_data_list_.push_back(std::make_pair(timestamp, local_map_data));
}

NavStatus LocalizationBackendSMM::GetCurrentLocatorStatus() const {
  // check whether backend location has huge time delay
  std::lock_guard<std::mutex> lock(conf_mutex_);
  if (conf_list_.empty()) return NavStatus::FATAL_ACCURACY;
  auto loc_frontend = loc_frontend_.lock();
  double delay_ts = loc_frontend->GetFrontendTime() * 1.0e-9 -
                    conf_list_.back().first * 1.0e-9;
  if (delay_ts > 2.0) return NavStatus::FATAL_ACCURACY;

  return smm_status_;
}

adLocStatus_t LocalizationBackendSMM::InitLocator() {
  locator_ = std::make_shared<smm::SemanticMMLocator>();
  if (nullptr == locator_) {
    LC_LERROR(SYSTEM) << "Failed to create SMM locator.";
    return LOC_ALLOC_MEMORY_FAILED;
  }
  if (locator_->Init(param_) != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to init SMM locator.";
    return LOC_LOCALIZATION_ERROR;
  }
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationBackendSMM::InitLocEvaluator() {
  std::string setting_type = "SMM";
  evaluator_ = std::make_shared<EvaluatorLocalization>();
  if (nullptr == evaluator_) {
    LC_LERROR(SYSTEM) << "Failed to create localization SMM evaluator.";
    return LOC_ALLOC_MEMORY_FAILED;
  }
  if (evaluator_->Init(param_.ci_param.results_save_dir, setting_type,
                       param_.ci_param.testcase_id) != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to init localization SMM evaluator.";
    return LOC_LOCALIZATION_ERROR;
  }
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationBackendSMM::InitDataBuffer() {
  InitCapacity(param_.smm_param.enable_camera_names);
  {
    std::lock_guard<std::mutex> lock(local_map_data_mutex_);
    local_map_data_list_.set_capacity(kMapDataWindowSize_);
  }

  {
    std::lock_guard<std::mutex> lock(conf_mutex_);
    conf_list_.set_capacity(kConfDataWindowSize);
  }
  return LOC_SUCCESS;
}

void LocalizationBackendSMM::SetThreadName() {
  pthread_setname_np(thread_->native_handle(), "LC_back_smm");
}

adLocStatus_t LocalizationBackendSMM::ResetSubData() {
  ClearData();
  {
    std::lock_guard<std::mutex> lock(local_map_data_mutex_);
    local_map_data_list_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(conf_mutex_);
    conf_list_.clear();
  }
  InitDataBuffer();
  smm_status_ = NavStatus::FATAL_ACCURACY;
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationBackendSMM::SwitchOriginSubProc() {
  // reinit for evaluator related
  if (param_.ci_param.enable_evaluation) {
    static uint64_t origin_id = 1;
    std::string tmp = "_" + std::to_string(origin_id);
    ++origin_id;
    std::string setting_type = "SMM" + tmp;
    evaluator_->Init(param_.ci_param.results_save_dir, setting_type,
                     param_.ci_param.testcase_id);
  }
  return locator_->SwitchOriginProc();
}

adLocStatus_t LocalizationBackendSMM::LocatorProcess() {
  // get semantic percepted data from cached buffer
  std::vector<PerceptData::Ptr> synced_datas;
  if (!SyncPerceptDatas(&synced_datas)) return LOC_LOCALIZATION_ERROR;

  auto loc_frontend = loc_frontend_.lock();
  auto initializer = loc_frontend->GetInitilizer();

  // check whether MM need relocalization mode
  bool is_reloc_mode = initializer->GetInitStage() != InitStage::INIT_DONE;

  // search predicted current backend time's pose
  std::vector<std::tuple<PerceptData::Ptr, NavState, OdomState>>
      synced_pp_datas;
  synced_pp_datas.reserve(synced_datas.size());
  for (const auto& data : synced_datas) {
    NavState nav_state;
    if (is_reloc_mode) {
      auto status =
          initializer->SearchInitState(data->timestamp_ns, &nav_state);
    } else {
      // search predicted current backend time's pose, avoid using
      // frontend query pose API, as has postprocessed for smoothness
      if (msf_fusion_) {
        msf_fusion_->SearchNominalState(data->timestamp_ns, &nav_state);
      }
    }

    OdomState odom_state;
    if (param_.smm_param.enable_temporal_fusion &&
        loc_dr_->QueryOdomPoseByTime(data->timestamp_ns, &odom_state) !=
            LOC_SUCCESS) {
      return LOC_LOCALIZATION_ERROR;
    }
    synced_pp_datas.emplace_back(data, nav_state, odom_state);
  }
  if (synced_pp_datas.empty() || synced_pp_datas.size() != synced_datas.size())
    return LOC_LOCALIZATION_ERROR;

  // set gt and gnss state for mm visualization
  if (loc_visualizer_) {
    NavState gt_state, rtk_state;
    if (LOC_SUCCESS != loc_frontend->GetGtState(&gt_state, &rtk_state))
      return LOC_LOCALIZATION_ERROR;
    locator_->SetGtWithTime(gt_state);
    locator_->SetGNSSWithTime(rtk_state);
  }

  // update local HDMap for smm at about 1HZ
  NavState nav_state = std::get<1>(synced_pp_datas.front());
  {
    std::lock_guard<std::mutex> guard(local_map_data_mutex_);
    if (!local_map_data_list_.empty()) {
      auto road_structure = local_map_data_list_.back().second;
      locator_->SetLocalMap(road_structure, nav_state);
      local_map_data_list_.clear();
    }
  }

  // start semantic map matching process
  adLocStatus_t locator_status =
      locator_->Process(synced_pp_datas, is_reloc_mode);
  // for mm visualization
  if (loc_visualizer_) {
    cv::Mat mm_vis_image = locator_->GetMMVisImage();
    loc_visualizer_->SetMMVisImage(mm_vis_image);
  }
  if (locator_status != LOC_SUCCESS) {
    LC_LDEBUG(SYSTEM) << "Semantic map matching error.";
    return LOC_LOCALIZATION_ERROR;
  }

  // get processed state, and convert to observation factor
  double locator_confidence;
  locator_->GetState(&nav_state, &locator_confidence);
  if (evaluator_) {
    evaluator_->WriteResult(nav_state.timestamp * kNanoSecToSec,
                            nav_state.pose.matrix());
  }

  if (is_reloc_mode) {
    // set refined state for initializer moudle as is reloc mode
    initializer->AddNavState(nav_state.timestamp, nav_state);
    return LOC_SUCCESS;
  }

  // update smm localization status
  UpdateNavStatus(nav_state.timestamp, locator_confidence);

  Eigen::Matrix<double, 7, 1> obs;
  obs.segment<3>(0) = nav_state.pose.translation();
  obs.segment<4>(3) =
      Utility::EigenQtoHamiltonVec(nav_state.pose.so3().unit_quaternion());

  auto front_locator_type =
      LocatorTypeFromString(param_.common_param.front_locator_type);
  std::shared_ptr<msf::MSFFactor> msf_factor_smm =
      msf::MSFFactorFactory::CreateMSFFactor(front_locator_type, SMM);
  if (msf_factor_smm == nullptr) {
    LC_LERROR(SYSTEM) << "unsupported fusion mode !!!";
    return LOC_LOCALIZATION_ERROR;
  }
  msf_factor_smm->SetObservationState(obs);
  msf_factor_smm->SetObservationCov(nav_state.pose_cov);
  msf_factor_smm->SetSource(nav_state.state_source);
  // add smm observation factor into queue
  if (msf_fusion_ && msf_fusion_->IsStarted()) {
    msf_fusion_->AddFactor(nav_state.timestamp, msf_factor_smm);
  }

  return LOC_SUCCESS;
}

bool LocalizationBackendSMM::SyncPerceptDatas(
    std::vector<std::shared_ptr<PerceptData>>* percept_datas) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  size_t camera_size = param_.smm_param.enable_camera_names.size();
  if (data_deques_.size() != camera_size) return false;

  for (const auto& item : data_deques_) {
    if (item.second.empty()) return false;
  }

  // find the base camera timestamp
  uint64_t base_timestamp = std::numeric_limits<uint64_t>::max();
  for (const auto& item : data_deques_) {
    if (item.second.back().first < base_timestamp) {
      base_timestamp = item.second.back().first;
    }
  }

  std::map<std::string, size_t> match_indexs;
  std::vector<PerceptData::Ptr> synced_datas;
  synced_datas.reserve(data_deques_.size());
  for (const auto& item : data_deques_) {
    const auto& cam_name = item.first;
    const auto& data_deque = item.second;

    size_t match_idx = 0;
    int64_t min_timegap = std::numeric_limits<int64_t>::max();
    for (size_t i = 0; i < data_deque.size(); ++i) {
      int64_t timegap = std::abs(static_cast<int64_t>(data_deque[i].first) -
                                 static_cast<int64_t>(base_timestamp));
      if (timegap < min_timegap) {
        match_idx = i;
        min_timegap = timegap;
      }
    }
    // gap check, sync percept data failed if gap beyond 80ms
    if (min_timegap > 8e7) return false;

    match_indexs[cam_name] = match_idx;
    synced_datas.emplace_back(data_deque[match_idx].second);
  }
  for (auto& item : data_deques_) {
    auto& data_deque = item.second;
    data_deque.erase(data_deque.begin(),
                     data_deque.begin() + match_indexs[item.first] + 1);
  }

  int valid_frame_cnt = param_.smm_param.enable_camera_names.size();
  for (const auto& data : synced_datas) {
    if (data->Empty()) --valid_frame_cnt;
  }
  if (valid_frame_cnt == 0) return false;

  *percept_datas = std::move(synced_datas);
  return true;
}

void LocalizationBackendSMM::UpdateNavStatus(uint64_t timestamp,
                                             double confidence) {
  // buffer locator confidence datas
  {
    std::lock_guard<std::mutex> lock(conf_mutex_);
    conf_list_.push_back(std::make_pair(timestamp, confidence));
    if (conf_list_.size() < kConfDataWindowSize) {
      smm_status_ = NavStatus::FATAL_ACCURACY;
      return;
    }
  }
  double rate_gap = 0.0;
  {
    // check backend location frame rate (SMM: 10HZ)
    std::lock_guard<std::mutex> lock(conf_mutex_);
    double time_gap =
        (conf_list_.back().first * 1.0e-9 - conf_list_.front().first * 1.0e-9);
    rate_gap = std::fabs((conf_list_.size() - 1) / time_gap - 8.);
  }
  // check location confidence mean and std, output final status
  std::vector<double> conf_datas;
  {
    std::lock_guard<std::mutex> lock(conf_mutex_);
    conf_datas.reserve(conf_list_.size());
    for (const auto& it : conf_list_) conf_datas.emplace_back(it.second);
  }
  std::sort(conf_datas.begin(), conf_datas.end(), std::less<double>());
  static constexpr size_t kD = 3;
  double conf_mean = 0.0, conf_std = 0.0;
  for (size_t i = kD; i < kConfDataWindowSize - kD; ++i)
    conf_mean += conf_datas[i];
  conf_mean /= (conf_datas.size() - 2 * kD);
  for (size_t i = kD; i < kConfDataWindowSize - kD; ++i)
    conf_std += std::pow(conf_datas[i] - conf_mean, 2.0);
  conf_std = std::sqrt(conf_std / (conf_datas.size() - 2 * kD));

  std::string reason;
  if (rate_gap < 3.5 && conf_mean > 0.7 && conf_std < 0.2) {
    smm_status_ = NavStatus::HIGH_ACCURACY;
  } else if (rate_gap < 5.0 && conf_mean > 0.5 && conf_std < 0.4) {
    smm_status_ = NavStatus::MID_ACCURACY;
  } else if (rate_gap < 7.0 && conf_mean > 0.3 && conf_std < 0.6) {
    if (rate_gap >= 5.0) reason += ", rate gap is larger than 5.0";
    if (conf_mean <= 0.5) reason += ", conf mean is smaller than 0.5";
    if (conf_std >= 0.4) reason += ", conf std is larger than 0.4";
    smm_status_ = NavStatus::LOW_ACCURACY;
  } else {
    if (rate_gap >= 7.0) reason += ", rate gap is larger than 7.0";
    if (conf_mean <= 0.3) reason += ", conf mean is smaller than 0.3";
    if (conf_std >= 0.6) reason += ", conf std is larger than 0.6";
    smm_status_ = NavStatus::FATAL_ACCURACY;
  }
  if (smm_status_ == NavStatus::LOW_ACCURACY ||
      smm_status_ == NavStatus::FATAL_ACCURACY) {
    LC_LERROR_EVERY_SEC(SYSTEM, 1)
        << "SMM confidence: " << GetNavStatusStr(smm_status_) << reason;
  }
}

}  // namespace localization
}  // namespace senseAD
