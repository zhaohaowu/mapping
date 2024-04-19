/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 */

#include "msf/msf_fusion/msf_fusion_imu/msf_fusion_imu.hpp"

#include <limits>
#include <map>
#include <string>
#include <utility>

#include "base_locator/base_locator.hpp"
#include "common/msf_common.hpp"
#include "common/utility.hpp"
#include "imu/imu_locator.hpp"
#include "localization/common/log.hpp"
#include "msf/msf_core/msf_core_factory.hpp"

namespace senseAD {
namespace localization {
namespace msf {

MSFFusionIMU::MSFFusionIMU(std::shared_ptr<BaseLocator> locator,
                           const LocalizationParam& param)
    : MSFFusion(), param_(param) {
  locator_ = std::dynamic_pointer_cast<IMULocator>(locator);
  sins_output_ = locator_->GetSINS();

  if (param_.msf_param.enable_multi_filters) {
    summary_strategy_ = SummaryStrategy::summary;
  } else {
    summary_strategy_ = SummaryStrategy::specific;
  }
  if (param_.common_param.ins_device == "CHNAV") gnss_frequency_ = 5;
  back_locator_type_ =
      FindLocatorTypeFromString(param_.common_param.back_locator_type);
}

MSFFusionIMU::~MSFFusionIMU() {}

adLocStatus_t MSFFusionIMU::Init() {
  CreateMainFilter();
  if (param_.msf_param.enable_multi_filters) CreateLocalFilters();
  return LOC_SUCCESS;
}

adLocStatus_t MSFFusionIMU::AddData(uint64_t time_ns,
                                    const VectorXd& data_reading) {
  std::lock_guard<std::mutex> lock(msf_core_mutex_);
  if (is_first_add_data_) {
    main_filter_->SetSINS(sins_output_);
    for (auto& filter : local_filters_) filter->SetSINS(sins_output_);
    is_first_add_data_ = false;
    return LOC_SUCCESS;
  }

  // check whether need to restart terrible filters
  bool need_restart = main_filter_->GetFilterStatus() == FilterStatus::TERRIBLE;
  for (const auto& filter : local_filters_) {
    need_restart =
        need_restart || filter->GetFilterStatus() == FilterStatus::TERRIBLE;
  }

  std::shared_ptr<SINS> sins_ref = nullptr;
  StateMeta state_ref;
  if (need_restart) {
    if (main_filter_->GetFilterStatus() != FilterStatus::TERRIBLE) {
      sins_ref = main_filter_->GetSINS();
      state_ref = main_filter_->GetNewestState();
    } else {
      for (const auto& filter : local_filters_) {
        if (filter->GetFilterStatus() != FilterStatus::GOOD) continue;
        sins_ref = filter->GetSINS();
        state_ref = filter->GetNewestState();
        break;
      }
    }
    if (sins_ref == nullptr) {
      need_restart = false;  // avoid restart, continue self-filter
      LC_LERROR(MSF) << "main and local filters are all in terrible condition.";
    }
  }

  // restart main and local filters
  bool main_filter_restart = false;
  if (need_restart &&
      main_filter_->GetFilterStatus() == FilterStatus::TERRIBLE) {
    main_filter_->Restart();
    main_filter_->SetSINS(sins_ref);
    main_filter_->InsertState(state_ref);
    main_filter_restart = true;
    LC_LINFO(MSF) << "main filter restart.";
  }
  for (size_t i = 0; i < local_filters_.size(); ++i) {
    if (need_restart &&
        local_filters_[i]->GetFilterStatus() == FilterStatus::TERRIBLE) {
      local_filters_[i]->Restart();
      local_filters_[i]->SetSINS(sins_ref);
      local_filters_[i]->InsertState(state_ref);
      LC_LINFO(MSF) << "local filter restart, index: " << i;
    }
  }

  // for main filter, check if do sins integration
  if (main_filter_restart) {
    main_filter_->AddData(time_ns, data_reading, true);
  } else {
    // copy sins state from imu locator to main filter to save computation
    main_filter_->SetSINS(sins_output_);
    main_filter_->AddData(time_ns, data_reading, false);
  }

  // for each local filter, do sins integration
  for (size_t i = 0; i < local_filters_.size(); ++i) {
    local_filters_[i]->AddData(time_ns, data_reading, true);
  }

  // update latest frontend timestamp for factor state update
  latest_frontend_ts_ = time_ns;

  return LOC_SUCCESS;
}

adLocStatus_t MSFFusionIMU::GetLatestMaxErrorState(
    Eigen::Vector4d* error_state) {
  // TODO(zhaoming): consider other filters?
  return main_filter_->GetLatestMaxErrorState(error_state);
}

adLocStatus_t MSFFusionIMU::SearchNominalState(uint64_t time_ns,
                                               NavState* nav_state) {
  std::lock_guard<std::mutex> lock(msf_core_mutex_);
  // TODO(zhaoming): how to get fusion nominal state?
  return main_filter_->SearchNominalState(time_ns, nav_state);
}

adLocStatus_t MSFFusionIMU::SwitchOriginSubProc() {
  if (LOC_SUCCESS != main_filter_->SwitchOriginProc()) {
    LC_LERROR(MSF) << "main filter switch origin process failed.";
    return LOC_LOCALIZATION_ERROR;
  }
  for (auto& filter : local_filters_) {
    if (LOC_SUCCESS != filter->SwitchOriginProc()) {
      LC_LERROR(MSF) << "local filter switch origin process failed.";
      return LOC_LOCALIZATION_ERROR;
    }
  }
  return LOC_SUCCESS;
}

adLocStatus_t MSFFusionIMU::StateUpdate(
    uint64_t timestamp, const std::shared_ptr<MSFFactor>& obs_factor) {
  std::lock_guard<std::mutex> lock(msf_core_mutex_);
  // state update for all filters
  main_filter_->StateUpdate(timestamp, obs_factor);
  for (auto& filter : local_filters_) {
    filter->StateUpdate(timestamp, obs_factor);
  }

  std::vector<StateMeta> newest_states;
  newest_states.reserve(local_filters_.size() + 1);
  newest_states.emplace_back(main_filter_->GetNewestState());
  for (auto& filter : local_filters_) {
    newest_states.emplace_back(filter->GetNewestState());
  }

  // summary all filters' fused results
  StateMeta output_state;
  float64_t min_cov_norm = std::numeric_limits<double>::max();
  switch (summary_strategy_) {
    case SummaryStrategy::specific:
      // designate the output filter (default is the main)
      sins_output_->SetCoreState(newest_states.front().core_state);
      sins_output_->SetPMat(newest_states.front().P_cov);
      break;
    case SummaryStrategy::minCov:
      // using the filter results with the min cov estimate
      for (const auto& state : newest_states) {
        if (state.P_cov.diagonal().norm() < min_cov_norm) {
          min_cov_norm = state.P_cov.diagonal().norm();
          output_state = state;
        }
      }
      sins_output_->SetCoreState(output_state.core_state);
      sins_output_->SetPMat(output_state.P_cov);
      break;
    case SummaryStrategy::summary:
      // all filters fusion to get global optimization estimation
      output_state = InformationFusion(newest_states);
      sins_output_->SetCoreState(output_state.core_state);
      sins_output_->SetPMat(output_state.P_cov);
      break;
    default:
      break;
  }

  return LOC_SUCCESS;
}

adLocStatus_t MSFFusionIMU::CreateMainFilter() {
  // main filter init
  main_filter_ =
      msf::MSFCoreFactory::CreateMSFCore<MSFCoreIMU>(IMU, locator_, param_);
  std::map<STATE_SOURCE, uint64_t> obs_srcs({{MOCK, -1}, {CAN, 10}});
  if (back_locator_type_[GNSS]) {
    obs_srcs.insert({GNSS, gnss_frequency_});
    obs_srcs.insert({HEADING, -1});
  }
  if (back_locator_type_[SMM]) obs_srcs.insert({SMM, 10});
  main_filter_->AddSource(obs_srcs);

  return LOC_SUCCESS;
}

adLocStatus_t MSFFusionIMU::CreateLocalFilters() {
  // main filter init
  if (back_locator_type_[GNSS]) {  // IMU_GNSS
    CreateLocalIMUGNSSBaseFilter();
    CreateLocalIMUGNSSDQFilter();
  } else if (back_locator_type_[GNSS] &&
             back_locator_type_[SMM]) {  // IMU_GNSS_SMM
    CreateLocalIMUGNSSBaseFilter();
    CreateLocalIMUGNSSDQFilter();
  } else if (back_locator_type_[GNSS]) {  // IMU_GNSS
    CreateLocalIMUGNSSDQFilter();
  } else if (back_locator_type_[SMM]) {  // IMU_SMM
    CreateLocalIMUSMMDQFilter();
  } else {
    LC_LERROR(MSF)
        << "Create local filters fialed, unsupported backend setting.";
    return LOC_LOCALIZATION_ERROR;
  }

  return LOC_SUCCESS;
}

adLocStatus_t MSFFusionIMU::CreateLocalFilter(
    const std::map<STATE_SOURCE, uint64_t>& obs_srcs, bool is_main_filter,
    bool different_Q) {
  std::shared_ptr<MSFCoreIMU> filter =
      msf::MSFCoreFactory::CreateMSFCore<MSFCoreIMU>(
          IMU, locator_, param_, is_main_filter, different_Q);
  filter->AddSource(obs_srcs);
  local_filters_.emplace_back(filter);

  return LOC_SUCCESS;
}

adLocStatus_t MSFFusionIMU::CreateLocalIMUGNSSBaseFilter() {
  return CreateLocalFilter({{GNSS, gnss_frequency_}, {MOCK, -1}, {CAN, 10}},
                           false);
}

adLocStatus_t MSFFusionIMU::CreateLocalIMUGNSSDQFilter() {
  return CreateLocalFilter({{GNSS, gnss_frequency_}, {MOCK, -1}, {CAN, 10}},
                           false, true);
}

adLocStatus_t MSFFusionIMU::CreateLocalIMUSMMFilter() {
  return CreateLocalFilter({{SMM, 10}, {MOCK, -1}, {CAN, 10}}, false);
}

adLocStatus_t MSFFusionIMU::CreateLocalIMUSMMDQFilter() {
  return CreateLocalFilter({{SMM, 10}, {MOCK, -1}, {CAN, 10}}, false, true);
}

Eigen::Matrix4d MSFFusionIMU::ConvertNavStatePose(const StateMeta& state) {
  // Get PA from sins
  const IMUCoreState& core_state = state.core_state;
  Vector3d sins_p = core_state.segment<3>(kiStatePosition);
  Quaterniond sins_a =
      Utility::HamiltonVecToEigenQ(core_state.segment<4>(kiStateAttitude));

  SE3d state_enu_vehicle;
  SINS::SINSPAtoGlobalSE3(sins_p, sins_a, &state_enu_vehicle);

  return state_enu_vehicle.matrix();
}

MSFCoreIMU::StateMeta MSFFusionIMU::InformationFusion(
    const std::vector<StateMeta>& states_newest) {
  std::map<float64_t, size_t> filter_cov_map;
  std::map<float64_t, size_t> filter_weight_map;
  FilterStatus filter_status;
  for (size_t index = 0; index < states_newest.size(); ++index) {
    if (index == 0) {
      filter_status = main_filter_->GetFilterStatus();
    } else {
      filter_status = local_filters_[index - 1]->GetFilterStatus();
    }
    if ((filter_status == FilterStatus::GOOD) ||
        (filter_status == FilterStatus::NORMAL)) {
      float64_t cov_norm = states_newest[index].P_cov.diagonal().norm();
      filter_cov_map.insert(std::pair<float64_t, size_t>(cov_norm, index));
    }
  }

  float64_t min_cov_norm = filter_cov_map.begin()->first;
  typename std::map<float64_t, size_t>::iterator iter;
  float64_t weight_sum = 0.0;
  for (iter = filter_cov_map.begin(); iter != filter_cov_map.end(); ++iter) {
    if (iter->first > min_cov_norm * 100) {
      continue;
    }
    float64_t cov_norm_inv = 1.0 / iter->first;
    weight_sum += cov_norm_inv;
    filter_weight_map.insert(
        std::pair<float64_t, size_t>(cov_norm_inv, iter->second));
  }

  iter = filter_weight_map.end();
  --iter;
  float64_t max_weight = iter->first;
  if ((filter_weight_map.size() >= 3) && (max_weight / weight_sum > 0.5)) {
    // limit the minimum covariance filter's weight to 0.5 for stability
    float64_t dec_weight = 2 * max_weight - weight_sum;
    weight_sum -= dec_weight;
    max_weight -= dec_weight;
    size_t index = iter->second;
    filter_weight_map.erase(iter);
    filter_weight_map.insert(std::pair<float64_t, size_t>(max_weight, index));
  }

  IMUCoreState core_state_fusion = IMUCoreState::Zero();
  IMUPMat P_fusion = IMUPMat::Zero();
  std::stringstream ss;
  ss << "filter weight: ";
  for (iter = filter_weight_map.begin(); iter != filter_weight_map.end();
       ++iter) {
    const StateMeta& state = states_newest[iter->second];
    float64_t w = iter->first / weight_sum;
    core_state_fusion += w * state.core_state;
    P_fusion += w * state.P_cov;
    ss << "(" << iter->second << ", " << w << ") ";
  }
  LC_LDEBUG(MSF) << ss.str();

  // carefully deal with multiple quaterion interpolation
  iter = filter_weight_map.begin();
  Eigen::Vector4d base_q_vec =
      states_newest[iter->second].core_state.segment<4>(kiStateAttitude);
  Quaterniond base_q = Utility::HamiltonVecToEigenQ(base_q_vec);
  float64_t base_q_weight = iter->first / weight_sum;
  typename std::map<float64_t, size_t>::iterator iter_next = iter;
  ++iter_next;
  while (iter_next != filter_weight_map.end()) {
    float64_t next_q_weight = iter_next->first / weight_sum;
    float64_t factor = base_q_weight / (base_q_weight + next_q_weight);
    Eigen::Vector4d q_vec =
        states_newest[iter_next->second].core_state.segment<4>(kiStateAttitude);
    Quaterniond q = Utility::HamiltonVecToEigenQ(q_vec);
    base_q = q.slerp(factor, base_q);
    base_q.normalize();
    base_q_weight = base_q_weight + next_q_weight;
    ++iter_next;
  }

  core_state_fusion.segment<4>(kiStateAttitude) =
      Utility::EigenQtoHamiltonVec(base_q);

  StateMeta output_state;
  output_state.t_ns = states_newest.front().t_ns;
  output_state.imu_data = states_newest.front().imu_data;
  output_state.core_state = core_state_fusion;
  output_state.P_cov = P_fusion;

  return output_state;
}

}  // namespace msf
}  // namespace localization
}  // namespace senseAD
