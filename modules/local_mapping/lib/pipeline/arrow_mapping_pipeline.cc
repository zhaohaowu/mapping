// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: simple_lane_tracker_pipeline.cc
// @brief: associate history lane_track to current detected lane　object

#include <math.h>

#include <limits>
#include <memory>
#include <utility>

#include "modules/local_mapping/lib/datalogger/pose_manager.h"
#include "modules/local_mapping/lib/pipeline/arrow_mapping_pipeline.h"
#include "perception-lib/lib/config_manager/config_manager.h"
#include "perception-lib/lib/io/file_util.h"
#include "perception-lib/lib/io/protobuf_util.h"
#include "perception-lib/lib/location_manager/location_manager.h"
namespace hozon {
namespace mp {
namespace lm {

bool ArrowMappingPipeline::Init() {
  arrow_matcher_ = std::make_unique<ArrowMatcher>();
  arrow_gate_keeper_ = std::make_unique<ArrowGatekeeper>();
  arrow_gate_keeper_->Init();
  return true;
}

bool ArrowMappingPipeline::Process(
    const ProcessOption& option, MeasurementFrameConstPtr measurement_frame_ptr,
    LocalMapFramePtr localmap_frame_ptr) {
  // PERF_FUNCTION();
  // PERF_BLOCK_START();
  const auto& measurement_arrows =
      &measurement_frame_ptr->road_arrows_ptr->arrows;

  const auto& track_arrows = &localmap_frame_ptr->road_arrows_ptr->arrows;
  track_arrows->clear();

  // 1. 观测停止线过滤模块
  const std::vector<ArrowPtr> detect_measurements_ptr =
      CleanMeasureData(measurement_arrows);

  // 2. 观测和跟踪数据坐标系对齐
  UpdateTracks();

  // 3. 关联匹配
  AssociationResult association_result;
  MatcherOptions matcher_options;
  matcher_options.timestamp = measurement_frame_ptr->header.timestamp;
  arrow_matcher_->Associate(matcher_options, detect_measurements_ptr,
                            arrow_trackers_, &association_result);

  for (auto& assign : association_result.assignments) {
    auto target_idx = std::get<0>(assign);
    auto detect_idx = std::get<1>(assign);
    HLOG_DEBUG << "assignments target_idx: " << target_idx
               << " detect_idx:" << detect_idx;
  }

  for (auto& unassign_track : association_result.unassigned_tracks) {
    HLOG_DEBUG << "unassign_track idx : " << unassign_track;
  }

  for (auto& unassign_object : association_result.unsigned_objects) {
    HLOG_DEBUG << "unassign_object idx: " << unassign_object;
  }

  // 4. 更新匹配上的跟踪线
  HLOG_DEBUG << "start do UpdateAssignedTracks...";
  UpdateAssignedTracks(option, detect_measurements_ptr, association_result);
  // PERF_BLOCK_END("lane_UpdateAssignedTracks");
  HLOG_DEBUG << "end do UpdateAssignedTracks...";
  // 5. 更新未匹配上的跟踪线

  HLOG_DEBUG << "start do UpdateUnassignedTracks...";
  UpdateUnassignedTracks(option, detect_measurements_ptr, association_result);
  HLOG_DEBUG << "end do UpdateUnassignedTracks...";
  // PERF_BLOCK_END("lane_UpdateUnassignedTracks");

  // 6. 创建新的跟踪器
  CreateNewTracks(option, detect_measurements_ptr, association_result);
  // PERF_BLOCK_END("lane_CreateNewTracks");

  // 7. 路面箭头必要的后处理操作
  MergeTracks();

  LimitTracksNum();
  // 8. 准出模块
  CollectOutputObjects(track_arrows);

  return true;
}

std::string ArrowMappingPipeline::Name() const {
  return "ArrowMappingPipeline";
}

std::vector<ArrowPtr> ArrowMappingPipeline::CleanMeasureData(
    const std::vector<ArrowPtr>* measurement_datas) {
  std::vector<ArrowPtr> output_measurement_datas;
  output_measurement_datas.clear();

  for (const auto& measure_data : *measurement_datas) {
    if (!CommonUtil::IsConvex(measure_data->vehicle_points) ||
        measure_data->length < measure_data->width) {
      continue;
    }
    output_measurement_datas.push_back(measure_data);
  }

  return output_measurement_datas;
}

void ArrowMappingPipeline::UpdateAssignedTracks(
    const ProcessOption& tracker_option,
    const std::vector<ArrowPtr>& detected_arrows,
    const AssociationResult& association_result) {
  auto& assignments = association_result.assignments;
  size_t track_index = 0;
  size_t detect_index = 0;
  for (size_t i = 0; i < assignments.size(); ++i) {
    track_index = std::get<0>(assignments[i]);
    detect_index = std::get<1>(assignments[i]);
    arrow_trackers_[track_index]->UpdateWithDetectedObject(
        tracker_option, detected_arrows.at(detect_index));
  }
}

void ArrowMappingPipeline::UpdateUnassignedTracks(
    const ProcessOption& tracker_option,
    const std::vector<ArrowPtr>& detected_arrows,
    const AssociationResult& association_result) {
  auto& unassigned_track_indexs = association_result.unassigned_tracks;
  size_t track_index = 0;
  for (size_t i = 0; i < unassigned_track_indexs.size(); ++i) {
    track_index = unassigned_track_indexs[i];
    HLOG_DEBUG << "start into UpdateWithoutDetectedLaneLine";
    arrow_trackers_[track_index]->UpdateWithoutDetectedObject(tracker_option);
    HLOG_DEBUG << "end into UpdateWithoutDetectedLaneLine";
  }
  RemoveLostTracks();
}

void ArrowMappingPipeline::CreateNewTracks(
    const ProcessOption& option, const std::vector<ArrowPtr>& detected_arrows,
    const AssociationResult& association_result) {
  auto& unsigned_objects = association_result.unsigned_objects;
  size_t detect_index = 0;
  for (size_t i = 0; i < unsigned_objects.size(); ++i) {
    detect_index = unsigned_objects[i];
    // Init LaneTarget
    ArrowTargetPtr arrow_target_ptr = std::make_shared<ArrowTarget>();

    arrow_target_ptr->Init(option, detected_arrows.at(detect_index));

    ArrowTrackerPtr arrow_tracker;
    arrow_tracker.reset(new ArrowTracker);
    if (arrow_tracker->Init(option, arrow_target_ptr)) {
      arrow_trackers_.emplace_back(arrow_tracker);
    }
  }
}

void ArrowMappingPipeline::RemoveLostTracks() {
  size_t track_count = 0;
  for (size_t i = 0; i < arrow_trackers_.size(); ++i) {
    const auto& track_id = arrow_trackers_[i]->GetConstTarget()->Id();
    if (arrow_trackers_[i]->GetConstTarget()->IsDie()) {
      continue;
    }
    if (track_count == i) {
      track_count++;
      continue;
    }

    if (arrow_trackers_[i]
            ->GetConstTarget()
            ->GetConstTrackedObject()
            ->center_point.x() < -80.0) {
      continue;
    }

    arrow_trackers_[track_count++] = arrow_trackers_[i];
  }
  arrow_trackers_.resize(track_count);
}

void ArrowMappingPipeline::UpdateTracks() {
  std::vector<ArrowTargetPtr> targets = GetAllTarget();

  const Eigen::Affine3d T_cur_last_ = POSE_MANAGER->GetDeltaPose();

  // 更新箭头的车系点、中点和heading。
  for (auto& target : targets) {
    auto& vehicle_points = target->GetTrackedObject()->vehicle_points;
    for (auto& point : vehicle_points) {
      point = T_cur_last_ * point;
    }

    auto& center_point = target->GetTrackedObject()->center_point;
    center_point = T_cur_last_ * center_point;

    Eigen::Matrix3d R_C_L = T_cur_last_.rotation();
    auto& heading = target->GetTrackedObject()->heading;
    Eigen::Matrix3d R_L_S;
    R_L_S << cos(heading), -sin(heading), 0, sin(heading), cos(heading), 0, 0,
        0, 1;
    Eigen::Matrix3d R_C_S = R_C_L * R_L_S;
    heading = atan2(R_C_S(1, 0), R_C_S(0, 0));
  }
}

void ArrowMappingPipeline::MergeTracks() {
  int tracker_nums = static_cast<int>(arrow_trackers_.size());
  if (tracker_nums < 2) {
    return;
  }

  std::vector<int> remove_trackers;
  remove_trackers.clear();

  for (int i = 0; i < tracker_nums - 1; ++i) {
    const auto& target_1 = arrow_trackers_[i]->GetConstTarget();
    if (!target_1->IsTracked()) {
      continue;
    }
    for (int j = i + 1; j < tracker_nums; ++j) {
      const auto& target_2 = arrow_trackers_[j]->GetConstTarget();
      if (!target_2->IsTracked()) {
        continue;
      }

      Eigen::Vector3d arrow_direction =
          target_1->GetConstTrackedObject()->vehicle_points[0] -
          target_1->GetConstTrackedObject()->vehicle_points[1];
      Eigen::Vector3d arrow_vertical_direction(-arrow_direction.y(),
                                               arrow_direction.x(), 0.0);
      Eigen::Vector3d arrow_distance =
          target_2->GetConstTrackedObject()->center_point -
          target_1->GetConstTrackedObject()->center_point;
      double x_dis =
          fabs(arrow_direction.dot(arrow_distance) / arrow_direction.norm());
      double y_dis = fabs(arrow_vertical_direction.dot(arrow_distance) /
                          arrow_vertical_direction.norm());
      if (x_dis > 10 || y_dis > 2.5) {
        continue;
      }
      target_1->GetConstTrackedObject()->confidence =
          (target_1->GetConstTrackedObject()->confidence +
           target_2->GetConstTrackedObject()->confidence) /
          2;
      target_1->GetConstTrackedObject()->center_point =
          0.5 * target_1->GetConstTrackedObject()->center_point +
          0.5 * target_2->GetConstTrackedObject()->center_point;

      target_1->GetConstTrackedObject()->length =
          0.5 * target_1->GetConstTrackedObject()->length +
          0.5 * target_2->GetConstTrackedObject()->length;

      target_1->GetConstTrackedObject()->width =
          0.5 * target_1->GetConstTrackedObject()->width +
          0.5 * target_2->GetConstTrackedObject()->width;

      target_1->GetConstTrackedObject()->heading =
          0.5 * target_1->GetConstTrackedObject()->heading +
          0.5 * target_2->GetConstTrackedObject()->heading;

      Eigen::Vector3d l = {target_1->GetConstTrackedObject()->length / 2 *
                               cos(target_1->GetConstTrackedObject()->heading),
                           target_1->GetConstTrackedObject()->length / 2 *
                               sin(target_1->GetConstTrackedObject()->heading),
                           0};
      Eigen::Vector3d w = {-target_1->GetConstTrackedObject()->width / 2 *
                               sin(target_1->GetConstTrackedObject()->heading),
                           target_1->GetConstTrackedObject()->width / 2 *
                               cos(target_1->GetConstTrackedObject()->heading),
                           0};
      Eigen::Vector3d point_0 =
          target_1->GetConstTrackedObject()->center_point + l + w;
      Eigen::Vector3d point_1 =
          target_1->GetConstTrackedObject()->center_point - l + w;
      Eigen::Vector3d point_2 =
          target_1->GetConstTrackedObject()->center_point - l - w;
      Eigen::Vector3d point_3 =
          target_1->GetConstTrackedObject()->center_point + l - w;
      std::vector<Eigen::Vector3d> new_vehicle_points = {point_0, point_1,
                                                         point_2, point_3};
      target_1->GetConstTrackedObject()->vehicle_points = new_vehicle_points;
      remove_trackers.push_back(target_2->Id());
      break;
    }
  }

  arrow_trackers_.erase(
      std::remove_if(arrow_trackers_.begin(), arrow_trackers_.end(),
                     [&](ArrowTrackerPtr& track) {
                       return std::count(remove_trackers.begin(),
                                         remove_trackers.end(),
                                         track->GetConstTarget()->Id());
                     }),
      arrow_trackers_.end());
}

bool ArrowMappingPipeline::CompareTrackTime(const ArrowTrackerPtr& d1,
                                            const ArrowTrackerPtr& d2) {
  return d1->GetConstTarget()->GetLastestTrackedTimestamp() >
         d2->GetConstTarget()->GetLastestTrackedTimestamp();
}

void ArrowMappingPipeline::LimitTracksNum() {
  std::sort(arrow_trackers_.begin(), arrow_trackers_.end(), CompareTrackTime);
  if (arrow_trackers_.size() > limit_max_tracker_nums_) {
    arrow_trackers_.resize(limit_max_tracker_nums_);
  }
  std::sort(arrow_trackers_.begin(), arrow_trackers_.end(),
            [](const ArrowTrackerPtr& d1, const ArrowTrackerPtr& d2) {
              return d1->GetConstTarget()->Id() < d2->GetConstTarget()->Id();
            });
}

void ArrowMappingPipeline::CollectOutputObjects(
    std::vector<ArrowPtr>* tracked_arrows) {
  for (size_t i = 0; i < arrow_trackers_.size(); ++i) {
    HLOG_DEBUG << "########CollectOutputObjects############";
    ArrowTargetConstPtr arrow_target = arrow_trackers_[i]->GetConstTarget();
    HLOG_DEBUG << "Buffer ArrowTarget " << arrow_target->ToStr();
    ArrowPtr output_object =
        std::make_shared<Arrow>(*arrow_target->GetConstTrackedObject());
    if (!arrow_gate_keeper_->AbleToOutput(arrow_trackers_[i]->GetConstTarget(),
                                          GetAllConstTarget())) {
      HLOG_DEBUG << "ArrowTrack NOT OUTPUT!";
      HLOG_DEBUG << "MinningTimestamp: "
                 << std::to_string(output_object->latest_tracked_time) << " "
                 << "Buffer ArrowTarget "
                 << "TrackStatus: " << arrow_target->ToStr() << " "
                 << "ArrowTrack NOT OUTPUT!";
      continue;
    }
    tracked_arrows->push_back(output_object);
  }
}

std::vector<ArrowTargetConstPtr> ArrowMappingPipeline::GetAllConstTarget() {
  arrow_targets_.clear();
  for (size_t i = 0; i < arrow_trackers_.size(); ++i) {
    ArrowTargetConstPtr arrow_target = arrow_trackers_[i]->GetConstTarget();
    arrow_targets_.push_back(arrow_target);
  }
  return arrow_targets_;
}

std::vector<ArrowTargetPtr> ArrowMappingPipeline::GetAllTarget() {
  std::vector<ArrowTargetPtr> arrow_targets;
  arrow_targets.clear();
  for (size_t i = 0; i < arrow_trackers_.size(); ++i) {
    ArrowTargetPtr arrow_target = arrow_trackers_[i]->GetTarget();
    arrow_targets.push_back(arrow_target);
  }
  return arrow_targets;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
