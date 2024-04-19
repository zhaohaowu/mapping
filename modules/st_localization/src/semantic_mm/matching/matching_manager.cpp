/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */
#include "semantic_mm/matching/matching_manager.hpp"

#include "common/utility.hpp"
#include "localization/common/log.hpp"
#include "localization/data_type/semantic_type.hpp"
#include "semantic_mm/base/lane_line.hpp"
#include "semantic_mm/common/configure.hpp"
#include "semantic_mm/frame/frame.hpp"
#include "semantic_mm/frame/frame_package.hpp"
#include "semantic_mm/map/map_manager.hpp"
#include "semantic_mm/matching/matcher/matcher_base.hpp"
#include "semantic_mm/matching/matcher/matcher_bbox.hpp"
#include "semantic_mm/matching/matcher/matcher_lane_line.hpp"
#include "semantic_mm/matching/matcher/matcher_multi_semantic_grid.hpp"
#include "semantic_mm/matching/matcher/matcher_pole.hpp"
#include "semantic_mm/matching/matcher/matcher_traffic_sign.hpp"
#include "semantic_mm/tracking/tracking_manager.hpp"

namespace senseAD {
namespace localization {
namespace smm {

MatchingManager::MatchingManager() {
  grid_match_.reset(new MatcherMultiSemanticGrid());
}

adLocStatus_t MatchingManager::ProcessMatchingFrmpkg(
    const std::shared_ptr<FramePackage>& frame_package,
    const std::shared_ptr<MapManager>& map_manager) {
  if (frame_package == nullptr || map_manager == nullptr) {
    LC_LERROR(OPTIM) << "nullptr of instance module";
    return LOC_NULL_PTR;
  }
  const auto& frames = frame_package->GetFrames();
  if (frames.empty()) {
    LC_LDEBUG(MATCHING) << "frames size is empty.";
    return LOC_INVALID;
  }
  bool relocalization_mode = frame_package->GetRelocalizationMode();

  // get map elements
  const auto& map_lanelines = map_manager->GetLocalLaneLines();

  // matching for each frame
  for (size_t i = 0; i < frames.size(); ++i) {
    const auto& frame = frames[i];
    const auto& camera_name = frame->GetCameraName();
    MatchIndex::Ptr match_index = std::make_shared<MatchIndex>();

    ///////////////////// lane line map matching /////////////////////
    std::vector<std::pair<id_t, id_t>> ll_match;
    auto status = MatchingLaneLine(
        frame->GetNavState().pose, frame->GetPerceptionData()->lane_lines,
        map_lanelines, relocalization_mode, &ll_match);
    if (status != LOC_SUCCESS) {
      LC_LDEBUG(MATCHING) << "matching laneline failed, frame name: "
                          << camera_name;
    } else {
      match_index->AddSemanticMatch(SemanticType::LaneLine, ll_match);
    }

    // TODO(xxx): map matching of other semantic objects...

    frame->SetMatchIndex(match_index);
  }

  return LOC_SUCCESS;
}

adLocStatus_t MatchingManager::ProcessMatchingTracker(
    const std::shared_ptr<FramePackage>& frame_package,
    const std::shared_ptr<TrackingManager>& tracking_manager,
    const std::shared_ptr<MapManager>& map_manager) {
  if (frame_package == nullptr || tracking_manager == nullptr ||
      map_manager == nullptr) {
    LC_LERROR(MATCHING) << "nullptr of instance module";
    return LOC_NULL_PTR;
  }

  // check if relocalization matching
  bool relocalization_mode = frame_package->GetRelocalizationMode();
  if (relocalization_mode) {
    // auto status =
    //     MatchingTrackerRelocalizationMode(tracking_manager, map_manager);
    auto smm_param = Configure::GetInstance()->GetLocalizationSMMParam();
    auto status = smm_param.use_reloc_new_version
                      ? TrackerRelocalizationObjectMatch(
                            frame_package, tracking_manager, map_manager)
                      : TrackerRelocalizationGridMatch(
                            frame_package, tracking_manager, map_manager);
    if (status != LOC_SUCCESS) {
      LC_LDEBUG(MATCHING) << "tracker based relocalization match failed.";
      return status;
    }
  } else {
    auto status =
        MatchingTrackerLocalizationMode(tracking_manager, map_manager);
    if (status != LOC_SUCCESS) {
      LC_LDEBUG(MATCHING) << "tracker based localization match failed.";
      return status;
    }
  }

  return LOC_SUCCESS;
}

adLocStatus_t MatchingManager::MatchingTrackerLocalizationMode(
    const std::shared_ptr<TrackingManager>& tracking_manager,
    const std::shared_ptr<MapManager>& map_manager) {
  auto smm_param = Configure::GetInstance()->GetLocalizationSMMParam();
  auto enable_camera_names = smm_param.enable_camera_names;

  // get map data
  const auto& map_lanelines = map_manager->GetLocalLaneLines();
  const auto& map_poles = map_manager->GetLocalPoles();
  const auto& map_signs = map_manager->GetLocalTrafficSigns();

  // get tracker data
  const auto& multi_frame_fused_lanelines =
      tracking_manager->GetFusedLaneLines();
  const auto& multi_frame_fused_poles = tracking_manager->GetFusedPoles();
  const auto& multi_frame_fused_signs = tracking_manager->GetFusedSigns();

  for (const auto& camera_name : enable_camera_names) {
    MatchIndex::Ptr match_index = std::make_shared<MatchIndex>();

    ///////////////////// lane line map matching /////////////////////
    if (!multi_frame_fused_lanelines.count(camera_name)) {
      LC_LDEBUG(MATCHING) << "no tracked lanelines for camera " << camera_name;
    } else {
      // tracking in map-aided mode, tracker id is already matched map id
      std::vector<std::pair<id_t, id_t>> ll_match;
      auto fused_lanelines = multi_frame_fused_lanelines.at(camera_name);
      for (const auto& item : fused_lanelines) {
        id_t id = item.first;
        if (!map_lanelines.count(id)) {
          LC_LDEBUG(MATCHING) << "why not exist map line " << id;
          continue;
        }
        ll_match.emplace_back(id, id);
      }
      match_index->AddSemanticMatch(SemanticType::LaneLine, ll_match);
    }

    // only use pole and sign perception from fov120 camera
    if (camera_name.find("120") != std::string::npos) {
      ///////////////////// pole map matching /////////////////////
      if (!multi_frame_fused_poles.count(camera_name)) {
        LC_LDEBUG(MATCHING) << "no tracked poles for camera " << camera_name;
      } else {
        const auto& fused_poles = multi_frame_fused_poles.at(camera_name);
        std::vector<std::pair<id_t, id_t>> match_indices;
        MatchingPole(fused_poles, map_poles, false, &match_indices);
        if (!match_indices.empty()) {
          match_index->AddSemanticMatch(SemanticType::Pole, match_indices);
        }
      }

      ///////////////// traffic sign map matching ///////////////////////
      if (!multi_frame_fused_signs.count(camera_name)) {
        LC_LDEBUG(MATCHING)
            << "no tracked traffic signs for camera " << camera_name;
      } else {
        const auto& fused_signs = multi_frame_fused_signs.at(camera_name);
        std::vector<std::pair<id_t, id_t>> match_indices;
        MatchingTrafficSign(fused_signs, map_signs, false, &match_indices);
        if (!match_indices.empty()) {
          match_index->AddSemanticMatch(SemanticType::TrafficSign,
                                        match_indices);
        }
      }
    }

    // TODO(fy): support extended semantics...

    // set match pairs
    tracking_manager->SetMatchIndex(camera_name, match_index);
  }

  return LOC_SUCCESS;
}

adLocStatus_t MatchingManager::MatchingTrackerRelocalizationMode(
    const std::shared_ptr<TrackingManager>& tracking_manager,
    const std::shared_ptr<MapManager>& map_manager) {
  // relocalization based only lane lines
  const auto& map_lanelines = map_manager->GetLocalLaneLines();
  const auto& multi_frame_fused_lanelines =
      tracking_manager->GetFusedLaneLines();
  for (const auto& frame : multi_frame_fused_lanelines) {
    const auto& camera_name = frame.first;
    const auto& fused_lanelines = frame.second;

    std::vector<std::pair<id_t, id_t>> match_indices;
    // TODO(fy): save copy
    std::unordered_map<id_t, PerceptLaneLine> tracker_lines_copy;
    for (const auto& item : fused_lanelines) {
      tracker_lines_copy.insert({item.first, *item.second});
    }
    auto status = MatchingLaneLine(SE3d(), tracker_lines_copy, map_lanelines,
                                   true, &match_indices);
    if (status != LOC_SUCCESS) {
      LC_LDEBUG(MATCHING) << "matching laneline failed, frame name: "
                          << camera_name;
      continue;
    }

    // set match pairs
    MatchIndex::Ptr match_index = std::make_shared<MatchIndex>();
    match_index->AddSemanticMatch(SemanticType::LaneLine, match_indices);
    tracking_manager->SetMatchIndex(camera_name, match_index);
  }

  return LOC_SUCCESS;
}

adLocStatus_t MatchingManager::TrackerRelocalizationGridMatch(
    const std::shared_ptr<FramePackage>& frame_package,
    const std::shared_ptr<TrackingManager>& tracking_manager,
    const std::shared_ptr<MapManager>& map_manager) {
  LC_LDEBUG(MATCHING) << "start smm relocalitzation...";
  auto status = grid_match_->ProcessGridMatch(frame_package, tracking_manager,
                                              map_manager);
  if (status == LOC_SUCCESS) {
    LC_LDEBUG(MATCHING) << "smm relocalization success!";
  } else {
    LC_LDEBUG(MATCHING) << "lane-level reloc failed, try road-level reloc...";
    status = grid_match_->ProcessGridMatchRoadLevelAccuracy();
    if (status == LOC_SUCCESS)
      LC_LDEBUG(MATCHING) << "smm road-level relocalization success!";
  }

  frame_package->SetRelocalDistributions(
      grid_match_->GetRelocalDistributions());
  if (status == LOC_SUCCESS) {
    tracking_manager->SetMatchIndex(grid_match_->GetMatchIndex());
    frame_package->SetInitialPoseCorrection(grid_match_->GetPoseCorrection());
  }
  return status;
}

adLocStatus_t MatchingManager::TrackerRelocalizationObjectMatch(
    const std::shared_ptr<FramePackage>& frame_package,
    const std::shared_ptr<TrackingManager>& tracking_manager,
    const std::shared_ptr<MapManager>& map_manager) {
  LC_LDEBUG(MATCHING) << "start smm relocalization object match...";
  auto status = grid_match_->ProcessRelocTrackerMatch(
      frame_package, tracking_manager, map_manager);
  frame_package->SetRelocalDistributions(
      grid_match_->GetRelocalDistributions());
  if (status == LOC_SUCCESS) {
    tracking_manager->SetMatchIndex(grid_match_->GetMatchIndex());
    frame_package->SetInitialPoseCorrection(grid_match_->GetPoseCorrection());
  }
  return status;
}

//////////////// map matching algorithm for base data structure ////////////////

adLocStatus_t MatchingManager::MatchingLaneLine(
    const SE3d& pose,
    const std::unordered_map<id_t, PerceptLaneLine>& percept_data,
    const LaneLine::PtrUMap& map_data, bool relocalization_mode,
    std::vector<std::pair<id_t, id_t>>* matching_indices) {
  if (!matching_indices) {
    LC_LDEBUG(MATCHING) << "matching_indices is nullptr.";
    return LOC_NULL_PTR;
  }

  // matching instance
  MatcherLaneLine::Ptr matcher = std::make_shared<MatcherLaneLine>();
  matcher->SetPoseState(pose);
  matcher->SetPerceptionData(percept_data);
  matcher->SetMapData(map_data);

  auto status = matcher->Matching(relocalization_mode);
  if (status != LOC_SUCCESS) return status;

  *matching_indices = matcher->GetMatchingIndices();

  return LOC_SUCCESS;
}

adLocStatus_t MatchingManager::MatchingPole(
    const std::unordered_map<id_t, std::shared_ptr<PerceptPole>>& percept_data,
    const std::unordered_map<id_t, std::shared_ptr<Pole>>& map_data,
    bool relocalization_mode,
    std::vector<std::pair<id_t, id_t>>* matching_indices) {
  if (!matching_indices) {
    LC_LDEBUG(MATCHING) << "matching_indices is nullptr.";
    return LOC_NULL_PTR;
  }

  std::unordered_map<id_t, PerceptPole> converted_percept_data;
  converted_percept_data.reserve(percept_data.size());
  for (const auto& item : percept_data) {
    converted_percept_data.insert({item.first, *(item.second)});
  }

  MatcherPole::Ptr matcher = std::make_shared<MatcherPole>();
  matcher->SetPerceptionData(converted_percept_data);
  matcher->SetMapData(map_data);

  auto status = matcher->Matching(relocalization_mode);
  if (status != LOC_SUCCESS) return status;

  *matching_indices = matcher->GetMatchingIndices();

  return LOC_SUCCESS;
}

adLocStatus_t MatchingManager::MatchingTrafficSign(
    const std::unordered_map<id_t, std::shared_ptr<PerceptTrafficSign>>&
        percept_data,
    const std::unordered_map<id_t, std::shared_ptr<TrafficSign>>& map_data,
    bool relocalization_mode,
    std::vector<std::pair<id_t, id_t>>* matching_indices) {
  if (!matching_indices) {
    LC_LDEBUG(MATCHING) << "matching_indices is nullptr.";
    return LOC_NULL_PTR;
  }

  std::unordered_map<id_t, PerceptTrafficSign> converted_percept_data;
  converted_percept_data.reserve(percept_data.size());
  for (const auto& item : percept_data) {
    converted_percept_data.insert({item.first, *(item.second)});
  }

  MatcherTrafficSign::Ptr matcher = std::make_shared<MatcherTrafficSign>();
  matcher->SetPerceptionData(converted_percept_data);
  matcher->SetMapData(map_data);

  auto status = matcher->Matching(relocalization_mode);
  if (status != LOC_SUCCESS) return status;

  *matching_indices = matcher->GetMatchingIndices();

  return LOC_SUCCESS;
}

adLocStatus_t MatchingManager::MatchingTrafficSignBbox(
    const uint64_t timestamp, const SE3d& pose,
    const std::unordered_map<id_t, PerceptTrafficSign>& percept_data,
    const std::unordered_map<id_t, std::shared_ptr<PerceptTrafficSign>>&
        tracking_data,
    const DepthTracker::PtrUMap& trackers,
    const std::shared_ptr<CameraModel>& camera_model,
    std::unordered_map<id_t, id_t>* matching_indices) {
  if (!matching_indices) {
    LC_LDEBUG(MATCHING) << "matching_indices is nullptr.";
    return LOC_NULL_PTR;
  }

  static const double min_depth = 8.0;
  static const double max_depth = 120.0;
  static const double search_line_extend = 30;  // pixel
  static const double iou_thre = 0.2;

  std::unordered_map<id_t, BoundingBox2D> percept_bboxes;
  std::unordered_map<id_t, SE3d> percept_poses;
  for (const auto& item : percept_data) {
    percept_bboxes.insert({item.first, item.second.rect});
    percept_poses.insert({item.first, pose});
  }

  std::unordered_map<id_t, BoundingBox2D> tracker_bboxes;
  std::unordered_map<id_t, SE3d> tracker_poses;
  std::unordered_map<id_t, Point3D_t> tracker_3d_centers;
  std::unordered_map<id_t, Point2D_t> tracker_predict_move_flow;
  for (const auto& item : tracking_data) {
    const auto tracker_iter = trackers.find(item.first);
    if (tracker_iter == trackers.end()) continue;
    tracker_bboxes.insert({item.first, item.second->rect});
    SE3d tracker_pose = tracker_iter->second->GetLastObservPose();
    tracker_poses.insert({item.first, tracker_pose});
    if (tracker_iter->second->GetCenterValid()) {
      Point3D_t center3d = tracker_iter->second->GetFusedCenter(tracker_pose);
      tracker_3d_centers.insert({item.first, center3d});
    }

    uint64_t tracker_timestamp = tracker_iter->second->GetTimestamp();
    double delta_time_ms = timestamp * 1e-6 - tracker_timestamp * 1e-6;
    if (delta_time_ms < 500.0 &&
        std::fabs(item.second->move_velocity.x) > 1e-8 &&
        std::fabs(item.second->move_velocity.y) > 1e-8) {
      Point2D_t move_flow =
          item.second->move_velocity * (delta_time_ms / 100.0);
      tracker_predict_move_flow.insert({item.first, move_flow});
    }
  }

  MatcherBBox::Ptr matcher = std::make_shared<MatcherBBox>();
  matcher->SetCameraModel(camera_model);
  matcher->SetQueryBox(percept_bboxes);
  matcher->SetReferenceBox(tracker_bboxes);
  matcher->SetQueryPose(percept_poses);
  matcher->SetReferencePose(tracker_poses);
  matcher->SetRefBBox3DCenter(tracker_3d_centers);
  matcher->SetRefBBoxMoveFlow(tracker_predict_move_flow);

  matcher->EpipolarIOUMatch(min_depth, max_depth, iou_thre, search_line_extend,
                            true, matching_indices);

  return LOC_SUCCESS;
}

adLocStatus_t MatchingManager::MatchingPoleBbox(
    const uint64_t timestamp, const SE3d& pose,
    const std::unordered_map<id_t, PerceptPole>& percept_data,
    const std::unordered_map<id_t, std::shared_ptr<PerceptPole>>& tracking_data,
    const DepthTracker::PtrUMap& trackers,
    const std::shared_ptr<CameraModel>& camera_model,
    std::unordered_map<id_t, id_t>* matching_indices) {
  if (!matching_indices) {
    LC_LDEBUG(MATCHING) << "matching_indices is nullptr.";
    return LOC_NULL_PTR;
  }

  static const double min_depth = 8.0;
  static const double max_depth = 120.0;
  static const double search_line_extend = 30;  // pixel
  static const double iou_thre = 0.3;

  std::unordered_map<id_t, BoundingBox2D> percept_bboxes;
  std::unordered_map<id_t, SE3d> percept_poses;
  for (const auto& item : percept_data) {
    percept_bboxes.insert({item.first, item.second.rect});
    percept_poses.insert({item.first, pose});
  }

  std::unordered_map<id_t, BoundingBox2D> tracker_bboxes;
  std::unordered_map<id_t, SE3d> tracker_poses;
  std::unordered_map<id_t, Point3D_t> tracker_3d_centers;
  std::unordered_map<id_t, Point2D_t> tracker_predict_move_flow;
  for (const auto& item : tracking_data) {
    const auto tracker_iter = trackers.find(item.first);
    if (tracker_iter == trackers.end()) continue;
    tracker_bboxes.insert({item.first, item.second->rect});
    SE3d tracker_pose = tracker_iter->second->GetLastObservPose();
    tracker_poses.insert({item.first, tracker_pose});
    if (tracker_iter->second->GetCenterValid()) {
      Point3D_t center3d = tracker_iter->second->GetFusedCenter(tracker_pose);
      tracker_3d_centers.insert({item.first, center3d});
    }

    uint64_t tracker_timestamp = tracker_iter->second->GetTimestamp();
    double delta_time_ms = timestamp * 1e-6 - tracker_timestamp * 1e-6;
    if (delta_time_ms < 500.0 &&
        std::fabs(item.second->move_velocity.x) > 1e-8 &&
        std::fabs(item.second->move_velocity.y) > 1e-8) {
      Point2D_t move_flow =
          item.second->move_velocity * (delta_time_ms / 100.0);
      tracker_predict_move_flow.insert({item.first, move_flow});
    }
  }

  MatcherBBox::Ptr matcher = std::make_shared<MatcherBBox>();
  matcher->SetCameraModel(camera_model);
  matcher->SetQueryBox(percept_bboxes);
  matcher->SetReferenceBox(tracker_bboxes);
  matcher->SetQueryPose(percept_poses);
  matcher->SetReferencePose(tracker_poses);
  matcher->SetRefBBox3DCenter(tracker_3d_centers);
  matcher->SetRefBBoxMoveFlow(tracker_predict_move_flow);

  matcher->EpipolarIOUMatch(min_depth, max_depth, iou_thre, search_line_extend,
                            true, matching_indices);

  return LOC_SUCCESS;
}

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
