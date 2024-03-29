// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: simple_lane_tracker_pipeline.cc
// @brief: associate history lane_track to current detected lane　object

#include <math.h>

#include <limits>
#include <memory>
#include <utility>

#include "base/scene/zebracrossing.h"
#include "modules/local_mapping/lib/datalogger/pose_manager.h"
#include "modules/local_mapping/lib/pipeline/zebracrossing_mapping_pipeline.h"
#include "perception-lib/lib/config_manager/config_manager.h"
#include "perception-lib/lib/io/file_util.h"
#include "perception-lib/lib/io/protobuf_util.h"
#include "perception-lib/lib/location_manager/location_manager.h"
namespace hozon {
namespace mp {
namespace lm {

bool ZebraCrossingMappingPipeline::Init() {
  zebracrossing_matcher_ = std::make_unique<ZebraCrossingMatcher>();
  zebracrossing_gate_keeper_ = std::make_unique<ZebraCrossingGatekeeper>();
  zebracrossing_gate_keeper_->Init();
  return true;
}

bool ZebraCrossingMappingPipeline::Process(
    const ProcessOption& option, MeasurementFrameConstPtr measurement_frame_ptr,
    LocalMapFramePtr localmap_frame_ptr) {
  // PERF_FUNCTION();
  // PERF_BLOCK_START();

  const auto& measurement_zebracrossings =
      &measurement_frame_ptr->zebra_crossings_ptr->zebra_crossings;

  const auto& track_zebracrossings =
      &localmap_frame_ptr->zebra_crossings_ptr->zebra_crossings;
  track_zebracrossings->clear();

  // 1. 观测停止线过滤模块
  const std::vector<ZebraCrossingPtr> detect_measurements_ptr =
      CleanMeasureData(measurement_zebracrossings);

  // 2. 观测和跟踪数据坐标系对齐
  UpdateTracks();

  // 3. 关联匹配
  AssociationResult point_association_result;
  MatcherOptions matcher_options;
  matcher_options.timestamp = measurement_frame_ptr->header.timestamp;
  zebracrossing_matcher_->Associate(matcher_options, detect_measurements_ptr,
                                    zebracrossing_trackers_,
                                    &point_association_result);

  // 4. 更新匹配上的跟踪线

  UpdateAssignedTracks(option, detect_measurements_ptr,
                       point_association_result);
  // PERF_BLOCK_END("lane_UpdateAssignedTracks");
  // 5. 更新未匹配上的跟踪线

  UpdateUnassignedTracks(option, detect_measurements_ptr,
                         point_association_result);
  // PERF_BLOCK_END("lane_UpdateUnassignedTracks");

  // 6. 创建新的跟踪器
  CreateNewTracks(option, detect_measurements_ptr, point_association_result);
  // PERF_BLOCK_END("lane_CreateNewTracks");

  // 7. 斑马线后处理
  MergeTracks();

  LimitTracksNum();
  // 8. 准出模块
  HLOG_DEBUG << "start do collect output zebracrossing objects...";
  CollectOutputObjects(track_zebracrossings);
  HLOG_DEBUG << "start do collect output zebracrossing objects...";
  HLOG_DEBUG << "collect output objects num: " << track_zebracrossings->size();

  return true;
}

std::string ZebraCrossingMappingPipeline::Name() const {
  return "ZebraCrossingMappingPipeline";
}

std::vector<ZebraCrossingPtr> ZebraCrossingMappingPipeline::CleanMeasureData(
    const std::vector<ZebraCrossingPtr>* measurement_datas) {
  std::vector<ZebraCrossingPtr> output_measurement_datas;
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

void ZebraCrossingMappingPipeline::UpdateAssignedTracks(
    const ProcessOption& tracker_option,
    const std::vector<ZebraCrossingPtr>& detected_zebracrossings,
    const AssociationResult& association_result) {
  HLOG_DEBUG << "start do update AssignedTracks...";
  auto& assignments = association_result.assignments;
  size_t track_index = 0;
  size_t detect_index = 0;

  for (size_t i = 0; i < assignments.size(); ++i) {
    track_index = std::get<0>(assignments[i]);
    detect_index = std::get<1>(assignments[i]);
    zebracrossing_trackers_[track_index]->UpdateWithDetectedObject(
        tracker_option, detected_zebracrossings.at(detect_index));
  }
  HLOG_DEBUG << "finish do update AssignedTracks...";
}

void ZebraCrossingMappingPipeline::UpdateUnassignedTracks(
    const ProcessOption& tracker_option,
    const std::vector<ZebraCrossingPtr>& detected_zebracrossings,
    const AssociationResult& association_result) {
  HLOG_DEBUG << "start do UpdateUnassignedTracks...";
  auto& unassigned_track_indexs = association_result.unassigned_tracks;
  size_t track_index = 0;
  for (size_t i = 0; i < unassigned_track_indexs.size(); ++i) {
    track_index = unassigned_track_indexs[i];
    HLOG_DEBUG << "start into UpdateWithoutDetectedLaneLine";
    zebracrossing_trackers_[track_index]->UpdateWithoutDetectedObject(
        tracker_option);
    HLOG_DEBUG << "end into UpdateWithoutDetectedLaneLine";
  }
  RemoveLostTracks();
  HLOG_DEBUG << "finish do UpdateUnassignedTracks...";
}

void ZebraCrossingMappingPipeline::CreateNewTracks(
    const ProcessOption& option,
    const std::vector<ZebraCrossingPtr>& detected_zebracrossings,
    const AssociationResult& association_result) {
  auto& unsigned_objects = association_result.unsigned_objects;
  size_t detect_index = 0;
  for (size_t i = 0; i < unsigned_objects.size(); ++i) {
    detect_index = unsigned_objects[i];
    // Init LaneTarget
    ZebraCrossingTargetPtr zebracrossing_target_ptr =
        std::make_shared<ZebraCrossingTarget>();
    zebracrossing_target_ptr->Init(option,
                                   detected_zebracrossings.at(detect_index));

    ZebraCrossingTrackerPtr zebracrossing_tracker;
    zebracrossing_tracker.reset(new ZebraCrossingTracker);
    if (zebracrossing_tracker->Init(option, zebracrossing_target_ptr)) {
      zebracrossing_trackers_.emplace_back(zebracrossing_tracker);
    }
  }
}

void ZebraCrossingMappingPipeline::RemoveLostTracks() {
  size_t track_count = 0;
  for (size_t i = 0; i < zebracrossing_trackers_.size(); ++i) {
    const auto& track_id = zebracrossing_trackers_[i]->GetConstTarget()->Id();
    if (zebracrossing_trackers_[i]->GetConstTarget()->IsDie()) {
      continue;
    }

    if (zebracrossing_trackers_[i]
            ->GetConstTarget()
            ->GetConstTrackedObject()
            ->center_point.x() < -80.0) {
      continue;
    }

    if (track_count == i) {
      track_count++;
      continue;
    }
    zebracrossing_trackers_[track_count++] = zebracrossing_trackers_[i];
  }
  zebracrossing_trackers_.resize(track_count);
}

void ZebraCrossingMappingPipeline::UpdateTracks() {
  std::vector<ZebraCrossingTargetPtr> targets = GetAllTarget();

  const Eigen::Affine3d T_cur_last_ = POSE_MANAGER->GetDeltaPose();

  // 更新斑马线的车系点、中点和heading。
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

void ZebraCrossingMappingPipeline::CollectOutputObjects(
    std::vector<ZebraCrossingPtr>* tracked_zebracrossings) {
  for (size_t i = 0; i < zebracrossing_trackers_.size(); ++i) {
    HLOG_DEBUG << "########CollectOutputObjects############";
    ZebraCrossingTargetConstPtr zebracrossing_target =
        zebracrossing_trackers_[i]->GetConstTarget();
    HLOG_DEBUG << "Buffer ArrowTarget " << zebracrossing_target->ToStr();
    ZebraCrossingPtr output_object = std::make_shared<ZebraCrossing>(
        *zebracrossing_target->GetConstTrackedObject());
    if (!zebracrossing_gate_keeper_->AbleToOutput(
            zebracrossing_trackers_[i]->GetConstTarget(),
            GetAllConstTarget())) {
      HLOG_DEBUG << "ArrowTrack NOT OUTPUT!";
      HLOG_DEBUG << "MinningTimestamp: "
                 << std::to_string(output_object->latest_tracked_time) << " "
                 << "Buffer ArrowTarget "
                 << "TrackStatus: " << zebracrossing_target->ToStr() << " "
                 << "ArrowTrack NOT OUTPUT!";
      continue;
    }
    tracked_zebracrossings->push_back(output_object);
  }
}

bool ZebraCrossingMappingPipeline::CompareTrackTime(
    const ZebraCrossingTrackerPtr& d1, const ZebraCrossingTrackerPtr& d2) {
  return d1->GetConstTarget()->GetLastestTrackedTimestamp() >
         d2->GetConstTarget()->GetLastestTrackedTimestamp();
}

void ZebraCrossingMappingPipeline::LimitTracksNum() {
  std::sort(zebracrossing_trackers_.begin(), zebracrossing_trackers_.end(),
            CompareTrackTime);
  if (zebracrossing_trackers_.size() > limit_max_tracker_nums_) {
    zebracrossing_trackers_.resize(limit_max_tracker_nums_);
  }
  std::sort(
      zebracrossing_trackers_.begin(), zebracrossing_trackers_.end(),
      [](const ZebraCrossingTrackerPtr& d1, const ZebraCrossingTrackerPtr& d2) {
        return d1->GetConstTarget()->Id() < d2->GetConstTarget()->Id();
      });
}

void ZebraCrossingMappingPipeline::MergeTracks() {
  int tracker_nums = zebracrossing_trackers_.size();
  if (tracker_nums < 2) return;

  std::vector<int> remove_trackers;
  remove_trackers.clear();

  for (int i = 0; i < tracker_nums - 1; ++i) {
    const auto& target_1 = zebracrossing_trackers_[i]->GetConstTarget();
    if (!target_1->IsTracked()) {
      continue;
    }
    for (int j = i + 1; j < tracker_nums; ++j) {
      const auto& target_2 = zebracrossing_trackers_[j]->GetConstTarget();
      if (!target_2->IsTracked()) {
        continue;
      }
      double distance = CommonUtil::CalTwoPointsDis(
          target_1->GetConstTrackedObject()->center_point,
          target_2->GetConstTrackedObject()->center_point);
      if (distance > 10) {
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

      Eigen::Vector3d l = {-target_1->GetConstTrackedObject()->length / 2 *
                               sin(target_1->GetConstTrackedObject()->heading),
                           target_1->GetConstTrackedObject()->length / 2 *
                               cos(target_1->GetConstTrackedObject()->heading),
                           0};
      Eigen::Vector3d w = {target_1->GetConstTrackedObject()->width / 2 *
                               cos(target_1->GetConstTrackedObject()->heading),
                           target_1->GetConstTrackedObject()->width / 2 *
                               sin(target_1->GetConstTrackedObject()->heading),
                           0};
      Eigen::Vector3d point_0 =
          target_1->GetConstTrackedObject()->center_point + l + w;
      Eigen::Vector3d point_1 =
          target_1->GetConstTrackedObject()->center_point + l - w;
      Eigen::Vector3d point_2 =
          target_1->GetConstTrackedObject()->center_point - l - w;
      Eigen::Vector3d point_3 =
          target_1->GetConstTrackedObject()->center_point - l + w;
      std::vector<Eigen::Vector3d> new_vehicle_points = {point_0, point_1,
                                                         point_2, point_3};
      target_1->GetConstTrackedObject()->vehicle_points = new_vehicle_points;
      remove_trackers.push_back(target_2->Id());
      break;
    }
  }

  zebracrossing_trackers_.erase(
      std::remove_if(
          zebracrossing_trackers_.begin(), zebracrossing_trackers_.end(),
          [&](ZebraCrossingTrackerPtr& track) {
            return std::count(remove_trackers.begin(), remove_trackers.end(),
                              track->GetConstTarget()->Id());
          }),
      zebracrossing_trackers_.end());
}

std::vector<ZebraCrossingTargetConstPtr>
ZebraCrossingMappingPipeline::GetAllConstTarget() {
  zebracrossing_targets_.clear();
  for (size_t i = 0; i < zebracrossing_trackers_.size(); ++i) {
    ZebraCrossingTargetConstPtr zebracrossing_target =
        zebracrossing_trackers_[i]->GetConstTarget();
    zebracrossing_targets_.push_back(zebracrossing_target);
  }
  return zebracrossing_targets_;
}

std::vector<ZebraCrossingTargetPtr>
ZebraCrossingMappingPipeline::GetAllTarget() {
  std::vector<ZebraCrossingTargetPtr> zebracrossing_targets;
  zebracrossing_targets.clear();
  for (size_t i = 0; i < zebracrossing_trackers_.size(); ++i) {
    ZebraCrossingTargetPtr zebracrossing_target =
        zebracrossing_trackers_[i]->GetTarget();
    zebracrossing_targets.push_back(zebracrossing_target);
  }
  return zebracrossing_targets;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
