/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */
#include "semantic_mm/tracking/tracking_manager.hpp"

#include <memory>

#include "common/utility.hpp"
#include "localization/common/log.hpp"
#include "localization/data_type/semantic_type.hpp"
#include "semantic_mm/common/camera_model.hpp"
#include "semantic_mm/common/configure.hpp"
#include "semantic_mm/frame/frame.hpp"
#include "semantic_mm/frame/frame_package.hpp"
#include "semantic_mm/map/map_manager.hpp"
#include "semantic_mm/matching/matching_manager.hpp"
#include "semantic_mm/tracking/database/element_lane_line_db.hpp"
#include "semantic_mm/tracking/database/element_pole_db.hpp"
#include "semantic_mm/tracking/database/element_traffic_sign_db.hpp"

namespace senseAD {
namespace localization {
namespace smm {

TrackingManager::TrackingManager(const LocalizationParam& param)
    : param_(param) {
  // init database for each camera frame
  const auto& enable_camera_names = param.smm_param.enable_camera_names;
  for (const auto& cam : enable_camera_names) {
    // get params
    CameraModel::Ptr camera_model;
    Configure::GetInstance()->GetCameraModel(cam, &camera_model);
    SE3d T_veh_cam;
    Configure::GetInstance()->GetCameraExtrinsic(cam, &T_veh_cam);

    laneline_dbs_[cam] = std::make_shared<ElementLaneLineDB>();
    if (param.smm_param.enable_percept_semantic) {
      sign_dbs_[cam] =
          std::make_shared<ElementTrafficSignDB>(camera_model, T_veh_cam);
      pole_dbs_[cam] = std::make_shared<ElementPoleDB>(camera_model, T_veh_cam);
    }
  }
}

adLocStatus_t TrackingManager::ProcessTracking(
    const std::shared_ptr<FramePackage>& frame_package,
    const std::shared_ptr<MapManager>& map_manager,
    const std::shared_ptr<MatchingManager>& matching_manager) {
  // reset match indices
  for (auto& match : match_indices_) {
    match.second->Reset();
  }

  const auto& frames = frame_package->GetFrames();
  if (frames.empty()) {
    LC_LDEBUG(TRACKING) << "frames size is empty.";
    return LOC_INVALID;
  }
  if (map_manager == nullptr || matching_manager == nullptr) {
    LC_LDEBUG(TRACKING) << "nullptr of instance module";
    return LOC_NULL_PTR;
  }

  // get map elements
  const auto& map_lanelines = map_manager->GetLocalLaneLines();
  const auto& map_signs = map_manager->GetLocalTrafficSigns();
  const auto& map_poles = map_manager->GetLocalPoles();

  // tracking for each frame
  fused_lanelines_.clear();
  fused_signs_.clear();
  fused_poles_.clear();
  bool relocalization_mode = frame_package->GetRelocalizationMode();
  for (size_t i = 0; i < frames.size(); ++i) {
    // get frame data
    const auto& frame = frames[i];
    uint64_t timestamp = frame->GetTimestamp();
    const auto& camera_name = frame->GetCameraName();
    const auto& percept_data = frame->GetPerceptionData();
    const auto& frame_pose = frame->GetOdomState().pose;
    auto match_index = frame->GetMatchIndex();
    // match indices could be empty
    std::vector<std::pair<id_t, id_t>> ll_match;
    std::vector<std::pair<id_t, id_t>> ts_match;
    std::vector<std::pair<id_t, id_t>> p_match;
    if (match_index) {
      match_index->GetSemanticMatch(SemanticType::LaneLine, &ll_match);
      match_index->GetSemanticMatch(SemanticType::TrafficSign, &ts_match);
      match_index->GetSemanticMatch(SemanticType::Pole, &p_match);
    }

    // tracking for lanelines
    adLocStatus_t track_status;
    track_status =
        TrackingLaneLine(camera_name, frame_pose, percept_data->lane_lines,
                         map_lanelines, ll_match, relocalization_mode);
    if (track_status != LOC_SUCCESS) {
      LC_LDEBUG(TRACKING) << "tracking laneline failed, frame name: "
                          << camera_name;
    }
    if (!param_.smm_param.enable_percept_semantic) continue;

    // tracking for traffic signs
    track_status = TrackingTrafficSign(timestamp, camera_name, frame_pose,
                                       percept_data->traffic_signs, map_signs,
                                       ts_match, relocalization_mode);
    if (track_status != LOC_SUCCESS) {
      LC_LDEBUG(TRACKING) << "tracking traffic sign failed, frame name: "
                          << camera_name;
    }

    // tracking for poles
    track_status =
        TrackingPole(timestamp, camera_name, frame_pose, percept_data->poles,
                     map_poles, p_match, relocalization_mode);
    if (track_status != LOC_SUCCESS) {
      LC_LDEBUG(TRACKING) << "tracking pole failed, frame name: "
                          << camera_name;
    }
  }

  return LOC_SUCCESS;
}

adLocStatus_t TrackingManager::TrackingLaneLine(
    const std::string& camera_name, const SE3d& frame_pose,
    const std::unordered_map<id_t, PerceptLaneLine>& percept_data,
    const LaneLine::PtrUMap& map_data,
    const std::vector<std::pair<id_t, id_t>>& match_indices,
    bool relocalization_mode) {
  if (!laneline_dbs_.count(camera_name)) {
    LC_LDEBUG(TRACKING) << "no laneline db found for " << camera_name;
    return LOC_INVALID;
  }

  auto line_db = laneline_dbs_.at(camera_name);
  LC_LDEBUG(TRACKING) << camera_name << ", live line tracker number "
                      << line_db->GetTrackerNumber();
  line_db->SetPoseState(frame_pose);
  line_db->SetPerceptionData(percept_data);
  line_db->SetMapData(map_data);
  line_db->SetMatchingIndecies(match_indices);

  auto status = line_db->TrackingAndFusion(!relocalization_mode);
  if (status != LOC_SUCCESS) {
    fused_lanelines_[camera_name] = {};
    return status;
  }
  fused_lanelines_[camera_name] = line_db->GetFusedPerceptionData();
  return LOC_SUCCESS;
}

adLocStatus_t TrackingManager::TrackingTrafficSign(
    const uint64_t timestamp, const std::string& camera_name,
    const SE3d& frame_pose,
    const std::unordered_map<id_t, PerceptTrafficSign>& percept_data,
    const TrafficSign::PtrUMap& map_data,
    const std::vector<std::pair<id_t, id_t>>& match_indices,
    bool relocalization_mode) {
  if (!sign_dbs_.count(camera_name)) {
    LC_LDEBUG(TRACKING) << "no traffic sign db found for " << camera_name;
    return LOC_INVALID;
  }

  auto sign_db = sign_dbs_.at(camera_name);
  LC_LDEBUG(TRACKING) << camera_name << ", live sign tracker number "
                      << sign_db->GetTrackerNumber();
  sign_db->SetTimestamp(timestamp);
  sign_db->SetPoseState(frame_pose);
  sign_db->SetPerceptionData(percept_data);
  sign_db->SetMapData(map_data);
  sign_db->SetMatchingIndecies(match_indices);

  auto status = sign_db->TrackingAndFusion(!relocalization_mode);
  if (status != LOC_SUCCESS) {
    fused_signs_[camera_name] = {};
    return status;
  }
  fused_signs_[camera_name] = sign_db->GetFusedPerceptionData();
  return LOC_SUCCESS;
}

adLocStatus_t TrackingManager::TrackingPole(
    const uint64_t timestamp, const std::string& camera_name,
    const SE3d& frame_pose,
    const std::unordered_map<id_t, PerceptPole>& percept_data,
    const Pole::PtrUMap& map_data,
    const std::vector<std::pair<id_t, id_t>>& match_indices,
    bool relocalization_mode) {
  if (!pole_dbs_.count(camera_name)) {
    LC_LDEBUG(TRACKING) << "no traffic sign db found for " << camera_name;
    return LOC_INVALID;
  }

  auto pole_db = pole_dbs_.at(camera_name);
  LC_LDEBUG(TRACKING) << camera_name << ", live pole tracker number "
                      << pole_db->GetTrackerNumber();
  pole_db->SetTimestamp(timestamp);
  pole_db->SetPoseState(frame_pose);
  pole_db->SetPerceptionData(percept_data);
  pole_db->SetMapData(map_data);
  pole_db->SetMatchingIndecies(match_indices);

  auto status = pole_db->TrackingAndFusion(!relocalization_mode);
  if (status != LOC_SUCCESS) {
    fused_poles_[camera_name] = {};
    return status;
  }
  fused_poles_[camera_name] = pole_db->GetFusedPerceptionData();
  return LOC_SUCCESS;
}

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
