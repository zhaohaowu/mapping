// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: simple_lane_tracker_pipeline.cc
// @brief: associate history lane_track to current detected lane　object

#include <math.h>

#include <limits>
#include <memory>
#include <utility>

#include "modules/local_mapping/lib/datalogger/pose_manager.h"
#include "modules/local_mapping/lib/pipeline/stopline_mapping_pipeline.h"
#include "perception-lib/lib/config_manager/config_manager.h"
#include "perception-lib/lib/io/file_util.h"
#include "perception-lib/lib/io/protobuf_util.h"
#include "perception-lib/lib/location_manager/location_manager.h"
namespace hozon {
namespace mp {
namespace lm {

bool StopLineMappingPipeline::Init() {
  stopline_matcher_ = std::make_unique<StopLineMatcher>();

  stopline_gate_keeper_ = std::make_unique<StopLineGatekeeper>();
  stopline_gate_keeper_->Init();
  return true;
}

bool StopLineMappingPipeline::Process(
    const ProcessOption& option, MeasurementFrameConstPtr measurement_frame_ptr,
    LocalMapFramePtr localmap_frame_ptr) {
  // PERF_FUNCTION();
  // PERF_BLOCK_START();

  const auto& track_stoplines = &localmap_frame_ptr->stop_lines_ptr->stoplines;
  track_stoplines->clear();

  const auto& measurement_stoplines =
      &measurement_frame_ptr->stop_lines_ptr->stoplines;

  // 1. 观测停止线过滤模块
  HLOG_DEBUG << "start clean stopline measurement data...";
  const std::vector<StopLinePtr> detect_measurements_ptr =
      CleanMeasureData(measurement_stoplines);
  HLOG_DEBUG << "finish clean stopline measurement data...";

  HLOG_DEBUG << "[TEST] output measurement stoplines: "
             << detect_measurements_ptr.size();
  // 2. 观测和跟踪数据坐标系对齐
  UpdateTracks();

  // 3. 关联匹配
  AssociationResult association_result;
  MatcherOptions matcher_options;
  matcher_options.timestamp = measurement_frame_ptr->header.timestamp;
  stopline_matcher_->Associate(matcher_options, detect_measurements_ptr,
                               stopline_trackers_, &association_result);

  // for (auto& assign : association_result.assignments) {
  //   auto target_idx = std::get<0>(assign);
  //   auto detect_idx = std::get<1>(assign);
  //   HLOG_DEBUG << "assignments target_idx: " << target_idx
  //              << " detect_idx:" << detect_idx;
  // }

  // for (auto& unassign_track : association_result.unassigned_tracks) {
  //   HLOG_DEBUG << "unassign_track idx : " << unassign_track;
  // }

  // for (auto& unassign_object : association_result.unsigned_objects) {
  //   HLOG_DEBUG << "unassign_object idx: " << unassign_object;
  // }

  // 4. 更新匹配上的跟踪线
  UpdateAssignedTracks(option, detect_measurements_ptr, association_result);
  // PERF_BLOCK_END("lane_UpdateAssignedTracks");
  // 5. 更新未匹配上的跟踪线
  UpdateUnassignedTracks(option, detect_measurements_ptr, association_result);
  // PERF_BLOCK_END("lane_UpdateUnassignedTracks");

  // 6. 创建新的跟踪器

  CreateNewTracks(option, detect_measurements_ptr, association_result);
  // PERF_BLOCK_END("lane_CreateNewTracks");

  // 7. 停止线所必要的后处理操作
  MergeTracks();

  LimitTracksNum();
  // 8. 准出模块
  HLOG_DEBUG << "start do collect output objects...";
  CollectOutputObjects(track_stoplines);
  HLOG_DEBUG << "start do collect output objects...";
  HLOG_DEBUG << "collect output objects num: " << track_stoplines->size();

  return true;
}

std::string StopLineMappingPipeline::Name() const {
  return "StopLineMappingPipeline";
}

std::vector<StopLinePtr> StopLineMappingPipeline::CleanMeasureData(
    const std::vector<StopLinePtr>* measurement_datas) {
  std::vector<StopLinePtr> output_measurement_datas;
  output_measurement_datas.clear();

  for (const auto& measure_data : *measurement_datas) {
    if (fabs(measure_data->left_point.y() - measure_data->right_point.y()) <
            3 ||
        fabs(measure_data->left_point.x() - measure_data->right_point.x()) >
            2) {
      continue;
    }
    output_measurement_datas.push_back(measure_data);
  }

  return output_measurement_datas;
}

void StopLineMappingPipeline::UpdateAssignedTracks(
    const ProcessOption& option,
    const std::vector<StopLinePtr>& detected_stoplines,
    const AssociationResult& association_result) {
  HLOG_DEBUG << "start do track assgin update...";
  auto& assignments = association_result.assignments;
  size_t track_index = 0;
  size_t detect_index = 0;

  ProcessOption tracker_option;
  // tracker_option.novatel2world_pose = novatel2world_pose_;
  for (size_t i = 0; i < assignments.size(); ++i) {
    track_index = std::get<0>(assignments[i]);
    detect_index = std::get<1>(assignments[i]);
    HLOG_DEBUG << "start do tracker assigin update...";
    stopline_trackers_[track_index]->UpdateWithDetectedObject(
        tracker_option, detected_stoplines[detect_index]);
    HLOG_DEBUG << "finish do tracker assigin update...";
  }
  HLOG_DEBUG << "finish do track assgin update...";
}

void StopLineMappingPipeline::UpdateUnassignedTracks(
    const ProcessOption& tracker_option,
    const std::vector<StopLinePtr>& detected_stoplines,
    const AssociationResult& association_result) {
  HLOG_DEBUG << "start do UpdateUnassignedTracks...";
  auto& unassigned_track_indexs = association_result.unassigned_tracks;
  size_t track_index = 0;
  // tracker_option.novatel2world_pose = novatel2world_pose_;
  for (size_t i = 0; i < unassigned_track_indexs.size(); ++i) {
    track_index = unassigned_track_indexs[i];
    HLOG_DEBUG << "start do tracker unassigin update...";
    stopline_trackers_[track_index]->UpdateWithoutDetectedObject(
        tracker_option);
    HLOG_DEBUG << "finish do tracker unassigin update...";
  }
  RemoveLostTracks();
  HLOG_DEBUG << "finish do UpdateUnassignedTracks...";
}

void StopLineMappingPipeline::CreateNewTracks(
    const ProcessOption& tracker_option,
    const std::vector<StopLinePtr>& detected_lanelines,
    const AssociationResult& association_result) {
  HLOG_DEBUG << "start do create a new track...";
  auto& unsigned_objects = association_result.unsigned_objects;
  size_t detect_index = 0;
  for (size_t i = 0; i < unsigned_objects.size(); ++i) {
    detect_index = unsigned_objects[i];
    // Init LaneTarget
    StopLineTargetPtr lane_target_ptr = std::make_shared<StopLineTarget>();
    lane_target_ptr->Init(tracker_option, detected_lanelines[detect_index]);

    StopLineTrackerPtr stopline_tracker;
    stopline_tracker.reset(new StopLineTracker);
    // tracker_init_option_.novatel2world_pose = novatel2world_pose_;
    if (stopline_tracker->Init(tracker_option, lane_target_ptr)) {
      stopline_trackers_.emplace_back(stopline_tracker);
    }
  }
  HLOG_DEBUG << "finish do create a new track...";
}

void StopLineMappingPipeline::RemoveLostTracks() {
  size_t track_count = 0;
  for (size_t i = 0; i < stopline_trackers_.size(); ++i) {
    const auto& track_id = stopline_trackers_[i]->GetConstTarget()->Id();
    if (stopline_trackers_[i]->GetConstTarget()->IsDie()) {
      continue;
    }

    if (stopline_trackers_[i]
            ->GetConstTarget()
            ->GetConstTrackedObject()
            ->center_point.x() < -80.0) {
      continue;
    }
    if (track_count == i) {
      track_count++;
      continue;
    }
    stopline_trackers_[track_count++] = stopline_trackers_[i];
  }
  stopline_trackers_.resize(track_count);
}

bool StopLineMappingPipeline::CompareTrackTime(const StopLineTrackerPtr& d1,
                                               const StopLineTrackerPtr& d2) {
  return d1->GetConstTarget()->GetLastestTrackedTimestamp() >
         d2->GetConstTarget()->GetLastestTrackedTimestamp();
}

void StopLineMappingPipeline::LimitTracksNum() {
  std::sort(stopline_trackers_.begin(), stopline_trackers_.end(),
            CompareTrackTime);
  if (stopline_trackers_.size() > limit_max_tracker_nums_) {
    stopline_trackers_.resize(limit_max_tracker_nums_);
  }
  std::sort(
      stopline_trackers_.begin(), stopline_trackers_.end(),
      [](const StopLineTrackerPtr& d1, const StopLineTrackerPtr& d2) {
        return d1->GetConstTarget()->Id() < d2->GetConstTarget()->Id();
      });
}

void StopLineMappingPipeline::MergeTracks() {
  int tracker_nums = stopline_trackers_.size();
  if (tracker_nums < 2) return;

  std::vector<int> remove_trackers;
  remove_trackers.clear();

  for (int i = 0; i < tracker_nums - 1; ++i) {
    const auto& target_1 = stopline_trackers_[i]->GetConstTarget();
    if (!target_1->IsTracked()) {
      continue;
    }
    for (int j = i + 1; j < tracker_nums; ++j) {
      const auto& target_2 = stopline_trackers_[j]->GetConstTarget();
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

      target_1->GetConstTrackedObject()->heading =
          0.5 * target_1->GetConstTrackedObject()->heading +
          0.5 * target_2->GetConstTrackedObject()->heading;

      Eigen::Vector3d temp_left_point = {
          target_1->GetConstTrackedObject()->center_point.x() +
              target_1->GetConstTrackedObject()->length / 2.0 *
                  cos(target_1->GetConstTrackedObject()->heading),
          target_1->GetConstTrackedObject()->center_point.y() +
              target_1->GetConstTrackedObject()->length / 2.0 *
                  sin(target_1->GetConstTrackedObject()->heading),
          0};
      Eigen::Vector3d temp_right_point = {
          target_1->GetConstTrackedObject()->center_point.x() -
              target_1->GetConstTrackedObject()->length / 2.0 *
                  cos(target_1->GetConstTrackedObject()->heading),
          target_1->GetConstTrackedObject()->center_point.y() -
              target_1->GetConstTrackedObject()->length / 2.0 *
                  sin(target_1->GetConstTrackedObject()->heading),
          0};
      target_1->GetConstTrackedObject()->left_point = temp_left_point;
      target_1->GetConstTrackedObject()->right_point = temp_right_point;

      remove_trackers.push_back(target_2->Id());
      break;
    }
  }

  stopline_trackers_.erase(
      std::remove_if(stopline_trackers_.begin(), stopline_trackers_.end(),
                     [&](StopLineTrackerPtr& track) {
                       return std::count(remove_trackers.begin(),
                                         remove_trackers.end(),
                                         track->GetConstTarget()->Id());
                     }),
      stopline_trackers_.end());
}

void StopLineMappingPipeline::CollectOutputObjects(
    std::vector<StopLinePtr>* tracked_stopline) {
  for (size_t i = 0; i < stopline_trackers_.size(); ++i) {
    HLOG_DEBUG << "########CollectOutputObjects############";
    StopLineTargetConstPtr stopline_target =
        stopline_trackers_[i]->GetConstTarget();
    HLOG_DEBUG << "Buffer ArrowTarget " << stopline_target->ToStr();
    StopLinePtr output_object =
        std::make_shared<StopLine>(*stopline_target->GetConstTrackedObject());
    if (!stopline_gate_keeper_->AbleToOutput(
            stopline_trackers_[i]->GetConstTarget(), GetAllConstTarget())) {
      HLOG_DEBUG << "ArrowTrack NOT OUTPUT!";
      HLOG_DEBUG << "MinningTimestamp: "
                 << std::to_string(output_object->latest_tracked_time) << " "
                 << "Buffer ArrowTarget "
                 << "TrackStatus: " << stopline_target->ToStr() << " "
                 << "ArrowTrack NOT OUTPUT!";
      continue;
    }
    tracked_stopline->push_back(output_object);
  }
}

void StopLineMappingPipeline::UpdateTracks() {
  std::vector<StopLineTargetPtr> targets = GetAllTarget();

  const Eigen::Affine3d T_cur_last_ = POSE_MANAGER->GetDeltaPose();

  // 更新停止线的左端点、右端点、中点、heading。
  for (auto& target : targets) {
    auto& left_point = target->GetTrackedObject()->left_point;
    left_point = T_cur_last_ * left_point;

    auto& right_point = target->GetTrackedObject()->right_point;
    right_point = T_cur_last_ * right_point;

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

std::vector<StopLineTargetConstPtr>
StopLineMappingPipeline::GetAllConstTarget() {
  stopline_targets_.clear();
  for (size_t i = 0; i < stopline_trackers_.size(); ++i) {
    StopLineTargetConstPtr stopline_target =
        stopline_trackers_[i]->GetConstTarget();
    stopline_targets_.push_back(stopline_target);
  }
  return stopline_targets_;
}

std::vector<StopLineTargetPtr> StopLineMappingPipeline::GetAllTarget() {
  std::vector<StopLineTargetPtr> stopline_targets;
  stopline_targets.clear();
  for (size_t i = 0; i < stopline_trackers_.size(); ++i) {
    StopLineTargetPtr stopline_target = stopline_trackers_[i]->GetTarget();
    stopline_targets.push_back(stopline_target);
  }
  return stopline_targets;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
