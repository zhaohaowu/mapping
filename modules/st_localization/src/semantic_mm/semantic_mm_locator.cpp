/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */
#include "semantic_mm/semantic_mm_locator.hpp"

#include <algorithm>

#include "ad_time/ad_time.hpp"
#include "common/coordinate_converter.hpp"
#include "common/msf_common.hpp"
#include "common/transform_config.hpp"
#include "common/utility.hpp"
#include "eval/evaluator_smm.hpp"
#include "localization/common/log.hpp"
#include "semantic_mm/common/configure.hpp"
#include "semantic_mm/frame/frame.hpp"
#include "semantic_mm/frame/frame_package.hpp"
#include "semantic_mm/map/map_manager.hpp"
#include "semantic_mm/matching/matching_manager.hpp"
#include "semantic_mm/optim/optim_manager.hpp"
#include "semantic_mm/tracking/tracking_manager.hpp"
#include "semantic_mm/visual/visualization.hpp"

namespace senseAD {
namespace localization {
namespace smm {

adLocStatus_t SemanticMMLocator::Init(const LocalizationParam& param) {
  auto status = Configure::GetInstance()->Init(param);
  if (status != LOC_SUCCESS) {
    LC_LERROR(SMM) << "failed to init smm configure.";
    return status;
  }
  // init map manager
  map_manager_.reset(new MapManager(param));
  if (!map_manager_) {
    LC_LERROR(SMM) << "failed to create MapManager.";
    return LOC_NULL_PTR;
  }

  // init matching manager
  matching_manager_.reset(new MatchingManager());
  if (!matching_manager_) {
    LC_LERROR(SMM) << "failed to create MatchingManager.";
    return LOC_NULL_PTR;
  }

  // init tracking manager
  if (param.smm_param.enable_temporal_fusion) {
    tracking_manager_.reset(new TrackingManager(param));
    if (!tracking_manager_) {
      LC_LERROR(SMM) << "failed to create TrackingManager.";
      return LOC_NULL_PTR;
    }
  }

  // init visualization
  if (param.visual_param.enable_display) {
    visualizer_.reset(new Visualization);
    if (!visualizer_) {
      LC_LERROR(SMM) << "failed to create Visualizer.";
      return LOC_NULL_PTR;
    }
  }

  // init evaluator
  if (param.ci_param.enable_evaluation) {
    evaluator_ = std::make_shared<EvaluatorSMM>();
    if (!evaluator_) {
      LC_LERROR(SMM) << "failed to create evaluator.";
      return LOC_NULL_PTR;
    }
    if (evaluator_->Init(param.ci_param.results_save_dir, "SMM_EVAL",
                         param.ci_param.testcase_id) != LOC_SUCCESS) {
      LC_LERROR(SMM) << "Failed to init smm evaluator";
      return LOC_LOCALIZATION_ERROR;
    }
    evaluator_->WriteHeader(param.smm_param.enable_camera_names);
  }

  optim_buffer_.set_capacity(optim_buffer_size_);
  optim_pose_buffer_.set_capacity(optim_buffer_size_);

  LC_LDEBUG(SMM) << "Create SemanticMMLocator done";
  return LOC_SUCCESS;
}

adLocStatus_t SemanticMMLocator::SetState(const NavState& car_state) {
  init_pose_state_ = car_state;
  pose_state_ = init_pose_state_;
  return LOC_SUCCESS;
}

adLocStatus_t SemanticMMLocator::GetState(NavState* car_state,
                                          double* confidence) {
  if (nullptr == car_state) {
    LC_LERROR(SMM) << "input nullptr.";
    return LOC_NULL_PTR;
  }
  *car_state = pose_state_;
  // output smm localization confidence
  if (confidence) {
    SE3d T_init_refine = init_pose_state_.pose.inverse() * pose_state_.pose;
    double lateral_update = T_init_refine.translation()(1);
    double factor = -3.0 * (std::pow(std::fabs(lateral_update), 2.0) +
                            pose_state_.pose_cov(1, 1));
    *confidence = std::exp(factor);
  }

  return LOC_SUCCESS;
}

adLocStatus_t SemanticMMLocator::SwitchOriginProc() {
  return map_manager_->SwitchOriginProc();
}

adLocStatus_t SemanticMMLocator::SetLocalMap(
    const RoadStructure::Ptr& road_structure, const NavState& car_state) {
  if (car_state.timestamp == 0) return LOC_LOCALIZATION_ERROR;
  TicToc timer;
  auto status = map_manager_->UpdateMap(road_structure, car_state.pose);
  map_process_cost_ = timer.toc();
  return status;
}

adLocStatus_t SemanticMMLocator::Process(
    const std::vector<std::tuple<PerceptData::Ptr, NavState, OdomState>>&
        percept_data,
    bool is_reloc_mode) {
  // check timstamp valid
  uint64_t curr_timestamp = std::get<0>(percept_data.front())->timestamp_ns;
  LC_LDEBUG(SMM) << "SMM process frame: " << curr_timestamp << "/"
                 << Time::ToString(curr_timestamp);
  if (curr_timestamp < main_timestamp_) {
    LC_LERROR(SMM) << "percept timestamp disorder.";
    return LOC_INVALID_PARAM;
  }
  main_timestamp_ = curr_timestamp;

  // check percept data empty
  if (percept_data.empty()) {
    LC_LDEBUG(SMM) << "input percept data invalid.";
    return LOC_INVALID_PARAM;
  }

  // check invalid nav state
  auto nav_state_ts = std::get<1>(percept_data.front()).timestamp;
  if (nav_state_ts == 0) {
    LC_LDEBUG(SMM) << "input navstate invalid, force into reloc mode";
    is_reloc_mode = true;
  }

  // check if dr origin reset
  OdomState curr_odom_state = std::get<2>(percept_data.front());
  id_t curr_odom_origin_id = static_cast<id_t>(curr_odom_state.origin_id);
  if (last_odom_origin_id_ != -1 &&
      curr_odom_origin_id != last_odom_origin_id_) {
    // reset tracking manager for processing in new origin
    if (tracking_manager_) {
      LC_LDEBUG(SMM) << "reset SMM TrackingManager";
      auto lp = Configure::GetInstance()->GetLocalizationParam();
      tracking_manager_.reset(new TrackingManager(lp));
      if (!tracking_manager_) {
        LC_LERROR(SMM) << "failed to create TrackingManager.";
        return LOC_NULL_PTR;
      }
    }
  }
  last_odom_origin_id_ = curr_odom_origin_id;

  // construct frame and frame package
  std::vector<Frame::Ptr> frames;
  frames.reserve(percept_data.size());
  for (const auto& item : percept_data) {
    const std::string& camera_name = std::get<0>(item)->camera_name;
    NavState nav_state = std::get<1>(item);
    TransformNavStateSE3ToSE2(std::get<1>(item), &nav_state);
    OdomState odom_state = std::get<2>(item);
    TransformOdomStateSE3ToSE2(std::get<2>(item), &odom_state);
    Frame::Ptr frame =
        std::make_shared<Frame>(std::get<0>(item)->timestamp_ns, camera_name,
                                nav_state, odom_state, std::get<0>(item));
    frames.emplace_back(frame);
  }

  FramePackage::Ptr frame_package = std::make_shared<FramePackage>(frames);
  init_pose_state_ = frames.front()->GetNavState();
  pose_state_ = init_pose_state_;
  frame_package->SetInitPoseState(init_pose_state_.pose);

  // check if need relocal
  if (is_reloc_mode) {
    frame_package->SetRelocalizationMode(is_reloc_mode);
  } else {
    CheckRelocalizationMode(frame_package);
  }

  // main SMM locator process
  TicToc timer;
  adLocStatus_t main_status;
  if (tracking_manager_) {
    main_status = ProcessMainTemporalFusion(frame_package);
  } else {
    main_status = ProcessMainSingleFrame(frame_package);
  }
  frame_package->SetSMMStatus(main_status == LOC_SUCCESS);
  double process_time_ms = timer.toc();

  // evaluation
  if (evaluator_) {
    auto eval_data =
        ConstructEvalData(frame_package, main_status, process_time_ms);
    evaluator_->WriteResult(main_timestamp_ * kNanoSecToSec, eval_data);
  }

  // visualize
  if (visualizer_) {
    SetGTDataForVisual(main_timestamp_, &frame_package);
    mm_vis_image_ =
        visualizer_->Draw(frame_package, map_manager_, tracking_manager_);
  }

  if (main_status == LOC_SUCCESS) {
    pose_state_.state_source = SMM;
    pose_state_.timestamp = main_timestamp_;
    pose_state_.pose = frame_package->GetPoseState();
    pose_state_.pose_cov = frame_package->GetPoseCov();

    // buffer optim result
    SE3d optim_delta = init_pose_state_.pose.inverse() * pose_state_.pose;
    Eigen::Vector2d optim_lat;
    optim_lat(0) = optim_delta.translation()(1);           // lateral
    optim_lat(1) = std::sqrt(pose_state_.pose_cov(1, 1));  // lateral std
    optim_buffer_.push_back(std::make_pair(main_timestamp_, optim_lat));
    optim_pose_buffer_.push_back(
        std::make_pair(main_timestamp_, pose_state_.pose));

    LC_LDEBUG(SMM) << "SMM Localization success.";
  } else {
    LC_LDEBUG(SMM) << "SMM Localization failed.";
  }

  return main_status;
}

void SemanticMMLocator::CheckRelocalizationMode(
    const std::shared_ptr<FramePackage>& frame_package) {
  if (optim_buffer_.size() < optim_buffer_size_) return;
  if (optim_buffer_.size() != optim_pose_buffer_.size()) return;

  // get max SMM timegap and lateral optim amplitude
  double max_timegap = 0;
  double max_timegap_trans = 0;
  double max_optim_lat = std::fabs(optim_buffer_.front().second(0));
  double mean_optim_lat = std::fabs(optim_buffer_.front().second(0));
  double max_optim_lat_std = optim_buffer_.front().second(1);
  for (size_t i = 1; i < optim_buffer_.size(); ++i) {
    uint64_t ts_last = optim_buffer_.at(i - 1).first;
    uint64_t ts_curr = optim_buffer_.at(i).first;
    const auto& optim_pose_last = optim_pose_buffer_.at(i - 1).second;
    const auto& optim_pose_curr = optim_pose_buffer_.at(i).second;
    double timegap =
        (static_cast<double>(ts_curr) - static_cast<double>(ts_last)) / 1e9;
    double trans =
        (optim_pose_last.inverse() * optim_pose_curr).translation().norm();
    if (timegap > max_timegap) {
      max_timegap = timegap;
      max_timegap_trans = trans;
    }

    double optim_lat = std::fabs(optim_buffer_.at(i).second(0));
    double optim_lat_std = optim_buffer_.at(i).second(1);
    if (optim_lat > max_optim_lat) {
      max_optim_lat = optim_lat;
      max_optim_lat_std = optim_lat_std;
    }
    mean_optim_lat += optim_lat;
  }
  mean_optim_lat /= optim_buffer_.size();

  double curr_timegap = (static_cast<double>(main_timestamp_) -
                         static_cast<double>(optim_buffer_.back().first)) /
                        1e9;
  double curr_trans = (optim_pose_buffer_.back().second.inverse() *
                       frame_package->GetInitPoseState())
                          .translation()
                          .norm();
  if (curr_timegap > max_timegap) {
    max_timegap = curr_timegap;
    max_timegap_trans = curr_trans;
  }

  // get main state local covariance
  auto nav_state = frame_package->GetFrames().front()->GetNavState();
  Eigen::Matrix3d rot = nav_state.pose.rotationMatrix();
  Eigen::Matrix3d P = nav_state.pose_cov.topLeftCorner(3, 3);
  Eigen::Matrix3d local_P = rot.transpose() * P * rot;
  double msf_lat_std = std::sqrt(local_P(1, 1));
  double gnss_bias_std = std::sqrt(init_pose_state_.gnss_bias_cov);

  // adapt std threshold with gnss bias std
  double lat_std_thres = 0.2;
  auto msf_param = Configure::GetInstance()->GetLocalizationParam().msf_param;
  if (msf_param.enable_gnss_bias_estimate) {
    lat_std_thres = gnss_bias_std + 0.1;
    lat_std_thres = std::min(lat_std_thres, 0.6);
    lat_std_thres = std::max(lat_std_thres, 0.1);
  }

  LC_LDEBUG(SMM) << "check reloc: "
                 << "|max_timagap: " << max_timegap
                 << ", trans: " << max_timegap_trans
                 << "|max_optim_lat: " << max_optim_lat
                 << ", lat_std:" << max_optim_lat_std
                 << "|mean_optim_lat: " << mean_optim_lat
                 << "|msf_lat_std:  " << msf_lat_std
                 << ", gnss_bias_std: " << gnss_bias_std;

  auto smm_param = Configure::GetInstance()->GetLocalizationSMMParam();
  if (smm_param.enable_strict_reloc_check) {
    // 1.reloc if lost frame more than 1s
    if (max_timegap >= 1.0 && max_timegap_trans > 10.0) {
      frame_package->SetRelocalizationMode(true);
      in_mismatch_reloc_ = true;
      return;
    }

    // reloc if optim lateral is larger than 0.8m
    if (max_optim_lat > 0.8 && max_optim_lat_std < 0.75) {
      frame_package->SetRelocalizationMode(true);
      in_mismatch_reloc_ = true;
      return;
    }

    // keep reloc if optim lateral is still > 0.3m
    if (in_mismatch_reloc_ && mean_optim_lat > 0.3) {
      frame_package->SetRelocalizationMode(true);
      in_mismatch_reloc_ = true;
      return;
    }

    // 2. otherwise, smm in good condition, return to loc mode
    frame_package->SetRelocalizationMode(false);
    in_mismatch_reloc_ = false;

  } else {
    // TODO(xxx): consider SMM tracking quality

    // 1. reloc if msf in bad condition
    if (msf_lat_std > lat_std_thres) {
      frame_package->SetRelocalizationMode(true);
      return;
    }

    // 2. otherwise, localization
    frame_package->SetRelocalizationMode(false);
  }
  return;
}

adLocStatus_t SemanticMMLocator::ProcessMainSingleFrame(
    const std::shared_ptr<FramePackage>& frame_package) {
  std::vector<std::pair<std::string, double>> time_cost;
  time_cost.emplace_back("MapProc", map_process_cost_);

  TicToc t_prep;
  // 1. update local map
  std::pair<double, double> lateral_range{-20.0, 20.0};
  std::pair<double, double> longitudinal_range{0.0, 60.0};
  if (init_pose_state_.timestamp != 0) {
    map_manager_->UpdateLocalMap(init_pose_state_.pose,
                                 init_pose_state_.pose_cov, lateral_range,
                                 longitudinal_range);
  } else {
    map_manager_->ClearLocalMap();
  }

  // 2. preprocess frame package
  auto prep_status = frame_package->PreProcess();
  if (prep_status != LOC_SUCCESS) {
    LC_LDEBUG(SMM) << "frame package preprocess failed.";
    return prep_status;
  }

  // 3. object-level matching of frame package and map
  auto match_status =
      matching_manager_->ProcessMatchingFrmpkg(frame_package, map_manager_);
  if (match_status != LOC_SUCCESS) {
    LC_LDEBUG(SMM) << "object-level frame to map matching failed.";
    return match_status;
  }
  time_cost.emplace_back("Prep&Match", t_prep.toc());

  // adapt homography and resample BV points
  auto smm_param = Configure::GetInstance()->GetLocalizationSMMParam();
  if (smm_param.enable_calib_homography &&
      !frame_package->GetRelocalizationMode()) {
    TicToc t_homo;
    auto opt_status =
        OptimManager::GetInstance()->OptimizeHomographyOnlyLaneLineInSpace(
            frame_package, map_manager_, matching_manager_);
    if (opt_status != LOC_SUCCESS) {
      LC_LDEBUG(SMM) << "SMM calib homography failed.";
    }
    time_cost.emplace_back("OptimHomo", t_homo.toc());
  }

  // 4. pose refine given percept-to-map matching
  TicToc t_pose;
  auto opt_status =
      OptimManager::GetInstance()->OptimizePoseOnlyLaneLineInSpace(
          frame_package, map_manager_, matching_manager_, nullptr);
  if (opt_status != LOC_SUCCESS) {
    LC_LDEBUG(SMM) << "SMM optimize pose failed.";
    return opt_status;
  }
  time_cost.emplace_back("OptimPose", t_pose.toc());
  frame_package->SetModuleTimeCost(time_cost);

  return LOC_SUCCESS;
}

adLocStatus_t SemanticMMLocator::ProcessMainTemporalFusion(
    const std::shared_ptr<FramePackage>& frame_package) {
  std::vector<std::pair<std::string, double>> time_cost;
  time_cost.emplace_back("MapProc", map_process_cost_);

  TicToc t_prep;
  // 1. update local map
  std::pair<double, double> lateral_range{-20.0, 20.0};
  std::pair<double, double> longitudinal_range{-20, 40.0};
  if (init_pose_state_.timestamp != 0) {
    map_manager_->UpdateLocalMap(init_pose_state_.pose,
                                 init_pose_state_.pose_cov, lateral_range,
                                 longitudinal_range);
  } else {
    map_manager_->ClearLocalMap();
  }

  // 2. preprocess frame package
  auto prep_status = frame_package->PreProcess();
  if (prep_status != LOC_SUCCESS) {
    LC_LDEBUG(SMM) << "frame package preprocess failed.";
    return prep_status;
  }
  time_cost.emplace_back("Prep", t_prep.toc());

  // 3. object-level matching of frame and map, for map-aided tracking and
  // homography optimization
  bool relocalization_mode = frame_package->GetRelocalizationMode();
  if (!relocalization_mode) {
    TicToc t_match;
    auto match_status =
        matching_manager_->ProcessMatchingFrmpkg(frame_package, map_manager_);
    if (match_status != LOC_SUCCESS) {
      LC_LDEBUG(SMM) << "object-level frame to map matching failed.";
      return match_status;
    }
    time_cost.emplace_back("FrameMatch", t_match.toc());

    // adapt homography and resample BV points
    auto smm_param = Configure::GetInstance()->GetLocalizationSMMParam();
    if (smm_param.enable_calib_homography) {
      TicToc t_homo;
      auto opt_status =
          OptimManager::GetInstance()->OptimizeHomographyOnlyLaneLineInSpace(
              frame_package, map_manager_, matching_manager_);
      if (opt_status != LOC_SUCCESS) {
        LC_LDEBUG(SMM) << "SMM calib homography failed.";
      }
      time_cost.emplace_back("OptimHomo", t_homo.toc());
    }
  }

  // 4. temporal tracking and fusion
  TicToc t_track;
  auto track_status = tracking_manager_->ProcessTracking(
      frame_package, map_manager_, matching_manager_);
  if (track_status != LOC_SUCCESS) {
    LC_LDEBUG(SMM) << "tracking and temporal fusion failed.";
    return track_status;
  }
  time_cost.emplace_back("Track", t_track.toc());

  TicToc t_match;
  auto match_status = matching_manager_->ProcessMatchingTracker(
      frame_package, tracking_manager_, map_manager_);
  if (match_status != LOC_SUCCESS) {
    LC_LDEBUG(SMM) << "tracker to map matching failed.";
    return match_status;
  }
  time_cost.emplace_back("TrackerMatch", t_match.toc());

  // 5. pose refine given percept-to-map matching
  TicToc t_pose;
  auto opt_status =
      OptimManager::GetInstance()->OptimizePoseOnlyLaneLineInSpace(
          frame_package, map_manager_, matching_manager_, tracking_manager_);
  if (opt_status != LOC_SUCCESS) {
    LC_LDEBUG(SMM) << "SMM optimize pose failed.";
    return opt_status;
  }
  time_cost.emplace_back("OptimPose", t_pose.toc());
  frame_package->SetModuleTimeCost(time_cost);

  return LOC_SUCCESS;
}

SMMEvalData SemanticMMLocator::ConstructEvalData(
    const std::shared_ptr<FramePackage>& frame_package,
    adLocStatus_t smm_status, double time_cost) {
  SMMEvalData eval_data;
  SE3d Twv = frame_package->GetInitPoseState();
  SE3d Tvw = Twv.inverse();
  eval_data.input_pose = Twv.matrix();
  eval_data.refined_pose = eval_data.input_pose;
  // check data quality
  frame_package->CheckPerceptionData(&eval_data);
  map_manager_->CheckMapData(Tvw, &eval_data);
  if (smm_status == LOC_SUCCESS) {
    eval_data.process_time_ms = time_cost;
    eval_data.refined_pose = frame_package->GetPoseState().matrix();
    eval_data.pose_cov = frame_package->GetPoseCov();
    eval_data.is_smm_success = frame_package->GetSMMStatus();
  }
  return eval_data;
}

void SemanticMMLocator::SetGTDataForVisual(
    uint64_t timestamp, std::shared_ptr<FramePackage>* frame_package) {
  NavState gt_state;
  auto status = QueryPoseByTime(main_timestamp_, gt_state_buffer_, &gt_state);
  if (status == LOC_SUCCESS) (*frame_package)->SetGTState(gt_state);
  NavState gnss_state = gnss_state_buffer_.back();
  gnss_state.pose = TransformGNSSPose(gnss_state.pose, init_pose_state_.pose);
  (*frame_package)->SetGnssState(gnss_state);
}

cv::Mat SemanticMMLocator::GetMMVisImage() const { return mm_vis_image_; }

adLocStatus_t SemanticMMLocator::SetGtWithTime(const NavState& nav_state) {
  if (gt_state_buffer_.size() <= 1 ||
      nav_state.timestamp - gt_state_buffer_.back().timestamp > 1e5) {
    gt_state_buffer_.emplace_back(nav_state);
  }
  if (gt_state_buffer_.size() > 5) {
    gt_state_buffer_.pop_front();
  }
  return LOC_SUCCESS;
}

adLocStatus_t SemanticMMLocator::SetGNSSWithTime(const NavState& nav_state) {
  if (gnss_state_buffer_.size() <= 1 ||
      nav_state.timestamp - gnss_state_buffer_.back().timestamp > 1e5) {
    gnss_state_buffer_.emplace_back(nav_state);
  }
  if (gnss_state_buffer_.size() > 5) {
    gnss_state_buffer_.pop_front();
  }
  return LOC_SUCCESS;
}

adLocStatus_t SemanticMMLocator::QueryPoseByTime(
    uint64_t query_timestamp_ns, const std::deque<NavState>& state_buffer,
    NavState* output_state) {
  if (nullptr == output_state) return LOC_NULL_PTR;
  if (state_buffer.size() < 2) {
    return LOC_INVALID;
  } else if (state_buffer.back().timestamp + 5e8 < query_timestamp_ns) {
    return LOC_TIME_AHEAD;
  } else if (state_buffer.front().timestamp > query_timestamp_ns + 5e8) {
    return LOC_TIME_DELAY;
  }

  // search two closet state
  int left_index = 0;
  int right_index = static_cast<int>(state_buffer.size() - 1);
  while (right_index - left_index > 1) {
    int mid_index = left_index + (right_index - left_index) / 2;
    uint64_t mid_timestamp = state_buffer[mid_index].timestamp;
    if (mid_timestamp > query_timestamp_ns) {
      right_index = mid_index;
    } else {
      left_index = mid_index;
    }
  }

  uint64_t left_timestamp = state_buffer[left_index].timestamp;
  uint64_t right_timestamp = state_buffer[right_index].timestamp;
  if (left_timestamp == right_timestamp) {
    return LOC_INVALID;
  }
  if (query_timestamp_ns < left_timestamp) {
    // forward extrapolate
    double factor = 1.0 *
                    (static_cast<double>(query_timestamp_ns) -
                     static_cast<double>(right_timestamp)) /
                    (static_cast<double>(left_timestamp) -
                     static_cast<double>(right_timestamp));
    NavStateInterp(state_buffer[right_index], state_buffer[left_index], factor,
                   output_state);
  } else {
    // backward extrapolate or interpolate
    double factor = 1.0 *
                    (static_cast<double>(query_timestamp_ns) -
                     static_cast<double>(left_timestamp)) /
                    (static_cast<double>(right_timestamp) -
                     static_cast<double>(left_timestamp));
    NavStateInterp(state_buffer[left_index], state_buffer[right_index], factor,
                   output_state);
  }

  return LOC_SUCCESS;
}

bool SemanticMMLocator::NavStateInterp(const NavState& s_ns,
                                       const NavState& e_ns, double factor,
                                       NavState* ns) {
  double timestamp = static_cast<double>(s_ns.timestamp) +
                     (static_cast<double>(e_ns.timestamp) -
                      static_cast<double>(s_ns.timestamp)) *
                         factor;
  ns->timestamp = static_cast<uint64_t>(timestamp);
  // ns->pose = SE3d::SE3Interpolation(s_ns.pose, e_ns.pose, factor); old Sophus
  ns->pose = Utility::SE3interpolate(s_ns.pose, e_ns.pose, factor);
  ns->linear_speed =
      s_ns.linear_speed + (e_ns.linear_speed - s_ns.linear_speed) * factor;
  ns->linear_acceleration =
      s_ns.linear_acceleration +
      (e_ns.linear_acceleration - s_ns.linear_acceleration) * factor;
  ns->angular_speed =
      s_ns.angular_speed + (e_ns.angular_speed - s_ns.angular_speed) * factor;
  ns->nav_status = e_ns.nav_status;
  // TODO(wangxiaofeng) How about covariance ?
  ns->pose_cov = e_ns.pose_cov;

  return true;
}

SE3d SemanticMMLocator::TransformGNSSPose(const SE3d& gnss_pose_lla,
                                          const SE3d& Twv) {
  // 1. transform gnss pose units
  Eigen::Matrix4d gnss_pose = gnss_pose_lla.matrix();

  // 2. lever arm compensation
  // llarm from FRD to IMU(RFU) frame
  Eigen::Vector3d llarm_RFU, llarm_FRD;
  llarm_FRD = TransformConfig::GetLeverArm();
  llarm_RFU << llarm_FRD(1), llarm_FRD(0), -llarm_FRD(2);
  Eigen::Matrix4d Tvb_matrix = TransformConfig::GetTvb().matrix();
  Eigen::Vector3d llarm_offset =
      Twv.so3().matrix() *
      (Tvb_matrix.block<3, 3>(0, 0) * llarm_RFU + Tvb_matrix.block<3, 1>(0, 3));
  gnss_pose(0, 3) = gnss_pose(0, 3) - llarm_offset[0];
  gnss_pose(1, 3) = gnss_pose(1, 3) - llarm_offset[1];
  gnss_pose(2, 3) = gnss_pose(2, 3) - llarm_offset[2];

  return SE3d(gnss_pose.block<3, 3>(0, 0), gnss_pose.block<3, 1>(0, 3));
}

void SemanticMMLocator::TransformNavStateSE3ToSE2(const NavState& nav_state_se3,
                                                  NavState* nav_state_se2) {
  // NOTE: z is not zeroed out, as map altitude is estimated from navstate
  Eigen::Vector3d t = nav_state_se3.pose.translation();
  // Eigen::Vector3d ypr = nav_state_se3.pose.so3().toYPR(); old Sophus
  Eigen::Vector3d ypr =
      nav_state_se3.pose.so3().unit_quaternion().toRotationMatrix().eulerAngles(
          2, 1, 0);
  double max_roll_pitch = 10.0 * M_PI / 180.0;
  // set zero only when roll pitch are abnormal
  if (std::fabs(ypr(1)) > max_roll_pitch ||
      std::fabs(ypr(2)) > max_roll_pitch) {
    ypr(1) = 0;
    ypr(2) = 0;
  }
  *nav_state_se2 = nav_state_se3;
  // nav_state_se2->pose = SE3d(SO3d(ypr), t); old Sophus
  nav_state_se2->pose = SE3d(SO3d::exp(ypr), t);
}

void SemanticMMLocator::TransformOdomStateSE3ToSE2(
    const OdomState& odom_state_se3, OdomState* odom_state_se2) {
  Eigen::Vector3d t = odom_state_se3.pose.translation();
  // Eigen::Vector3d ypr = odom_state_se3.pose.so3().toYPR(); old Suphus
  Eigen::Vector3d ypr = odom_state_se3.pose.so3()
                            .unit_quaternion()
                            .toRotationMatrix()
                            .eulerAngles(2, 1, 0);
  // zero out roll pitch z directly
  t(2) = 0;
  ypr(1) = 0;
  ypr(2) = 0;
  *odom_state_se2 = odom_state_se3;
  // odom_state_se2->pose = SE3d(SO3d(ypr), t); old Sophus
  odom_state_se2->pose = SE3d(SO3d::exp(ypr), t);
}

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
