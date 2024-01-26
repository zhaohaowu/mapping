// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: simple_lane_tracker_pipeline.cc
// @brief: associate history lane_track to current detected lane　object

#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/pipeline/roadedge_point_tracker_pipeline.h"

#include <math.h>

#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/data_fusion/simple_roadedge_tracker.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/datalogger/load_data_singleton.h"
#include "modules/laneline_postprocess/lib/laneline/utils/lane_utils.h"
// #include "perception-common/common/performance/perf_util.h"
#include "perception-lib/lib/config_manager/config_manager.h"
#include "perception-lib/lib/io/file_util.h"
#include "perception-lib/lib/io/protobuf_util.h"
#include "perception-lib/lib/location_manager/location_manager.h"
namespace hozon {
namespace mp {
namespace environment {
using base::LaneLine;
using base::LaneLinePoint;
using base::LaneLinePosition;
using base::LaneLinePtr;
using base::Location;
using lib::FileUtil;
using lib::LocationManager;
using lib::ParseProtobufFromFile;

bool RoadEdgePointFilterTrackerPipeline::Init(
    const ProcessInitOption &options) {
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig *model_config = nullptr;
  if (!config_manager->GetModelConfig(Name(), &model_config)) {
    HLOG_ERROR << "Parse config failed! Name: " << Name();
    return false;
  }

  const std::string work_root = config_manager->work_root();
  std::string config_file;

  if (!model_config->get_value("config_file", &config_file)) {
    HLOG_ERROR << "Get root path failed!";
    return false;
  }

  LanePostProcessParam postprocess_param;
  config_file = lib::FileUtil::GetAbsolutePath(work_root, config_file);
  CHECK(ParseProtobufFromFile<LanePostProcessParam>(config_file,
                                                    &lane_post_process_param_))
      << "Read config failed: " << config_file;
  const LaneTrackerPipelineParam &lane_tracker_pipeline_param =
      lane_post_process_param_.lane_tracker_pipeline_param();
  HLOG_DEBUG << "load lane_tracker_pipeline parameters from " << config_file
             << " \nParams: " << lane_tracker_pipeline_param.DebugString();

  target_init_option_.lane_target_param =
      lane_tracker_pipeline_param.lane_target_param();
  tracker_init_option_.lane_track_filter_param =
      lane_tracker_pipeline_param.lane_track_filter_param();

  // init matcher
  point_matcher_.reset(new PointMatcher());
  PointMatcherInitOptions point_matcher_init_options;
  point_matcher_init_options.lane_match_param =
      lane_tracker_pipeline_param.lane_match_param();
  point_matcher_->Init(point_matcher_init_options);

  // init lane_gate_keeper
  point_lane_gate_keeper_.reset(new PointLaneGatekeeper());
  PointLaneGatekeeperInitOptions lane_gate_keeper_init_opition;
  lane_gate_keeper_init_opition.lane_gate_keeper_param =
      lane_tracker_pipeline_param.lane_gate_keeper_param();
  point_lane_gate_keeper_->Init(lane_gate_keeper_init_opition);

  lane_targets_.reserve(100);
  lane_trackers_.clear();
  return true;
}

void RoadEdgePointFilterTrackerPipeline::TransMeasurementVehicle2Local(
    std::vector<base::RoadEdgeMeasurementPtr> *detect_measurements) {
  for (auto &detect_measurement : *detect_measurements) {
    auto &points = detect_measurement->point_set;
    TransVehiclePoint2Local(&points);
  }

  return;
}

void RoadEdgePointFilterTrackerPipeline::TransTrackerLocal2Vehicle(
    std::vector<base::RoadEdgePtr> *tracked_outputs) {
  for (auto &tracked_output : *tracked_outputs) {
    tracked_output->confidence = 1.0;
    auto &points = tracked_output->point_set;
    TransLocalPoint2Vehicle(&points);
  }
  return;
}

bool RoadEdgePointFilterTrackerPipeline::Track(
    const ProcessOption &options,
    base::RoadEdgesMeasurementConstPtr detect_measurements_msg,
    const base::RoadEdgesPtr &track_outputs) {
  // PERF_FUNCTION();
  // PERF_BLOCK_START();
  const auto &track_roadedges = &track_outputs->road_edges;
  track_roadedges->clear();

  // 1. 维护最近两帧的pose信息
  HLOG_DEBUG << "start do UpdateDrPose...";
  UpdateDrPose();
  HLOG_DEBUG << "end do UpdateDrPose...";

  // 2. 观测车道线转换到local系

  std::vector<base::RoadEdgeMeasurementPtr> vec_measurements =
      detect_measurements_msg->road_edges;
  const auto &detect_measurements = &vec_measurements;

  HLOG_DEBUG << "start do TransMeasurementVehicle2Local...";
  TransMeasurementVehicle2Local(detect_measurements);
  HLOG_DEBUG << "end do TransMeasurementVehicle2Local...";
  // PERF_BLOCK_END("lane_Preprocess");

  // 3. 观测线和跟踪线关联匹配
  HLOG_DEBUG << "start do Associate...";
  PointAssociationResult point_association_result;
  PointMatcherOptions matcher_options;
  matcher_options.timestamp = options.timestamp;
  point_matcher_->Associate(matcher_options, detect_measurements,
                            lane_trackers_, &point_association_result);
  HLOG_DEBUG << "detection nums: " << detect_measurements->size()
             << "tracker nums: " << lane_trackers_.size();
  for (auto &assign : point_association_result.assignments) {
    auto target_idx = std::get<0>(assign);
    auto detect_idx = std::get<1>(assign);
    HLOG_DEBUG << "assignments target_idx: " << target_idx
               << " detect_idx:" << detect_idx;
  }

  // 4. update matched lane_tracks
  HLOG_DEBUG << "start do UpdateAssignedTracks...";
  UpdateAssignedTracks(detect_measurements, point_association_result);
  // PERF_BLOCK_END("lane_UpdateAssignedTracks");
  HLOG_DEBUG << "end do UpdateAssignedTracks...";
  // 5. update unmatched lane_tracks

  HLOG_DEBUG << "start do UpdateUnassignedTracks...";
  UpdateUnassignedTracks(detect_measurements, point_association_result);
  HLOG_DEBUG << "end do UpdateUnassignedTracks...";
  // PERF_BLOCK_END("lane_UpdateUnassignedTracks");

  // 6. created new lane_tracks for unmatched detected_lanes
  CreateNewTracks(detect_measurements, point_association_result);
  // PERF_BLOCK_END("lane_CreateNewTracks");

  // 7. 主车道线后处理优化，异常检测
  PostProcess();
  // PERF_BLOCK_END("lane_PostProcess");

  // 8. 输出最终结果
  CollectOutputObjects(track_roadedges);
  HLOG_DEBUG << "output track laneline size ============"
             << track_roadedges->size();
  // PERF_BLOCK_END("lane_CollectOutputObjects");

  // 9. 把跟踪点local系转换到vehicle系
  TransTrackerLocal2Vehicle(track_roadedges);
  // PERF_BLOCK_END("RoadEdgePointFilterTrackerPipeline");

  return true;
}

void RoadEdgePointFilterTrackerPipeline::PostProcess() {
  // set lane pose atrribute
  // SetPoseAttribute();
}

void RoadEdgePointFilterTrackerPipeline::UpdateAssignedTracks(
    std::vector<base::RoadEdgeMeasurementPtr> *detected_lanelines,
    const PointAssociationResult &association_result) {
  auto &assignments = association_result.assignments;
  size_t track_index = 0;
  size_t detect_index = 0;

  SimpleRoadEdgeTrackerOptions tracker_option;
  tracker_option.novatel2world_pose = novatel2world_pose_;
  for (size_t i = 0; i < assignments.size(); ++i) {
    track_index = std::get<0>(assignments[i]);
    detect_index = std::get<1>(assignments[i]);
    lane_trackers_[track_index]->UpdateWithDetectedLaneLine(
        tracker_option, detected_lanelines->at(detect_index));
  }
  return;
}

void RoadEdgePointFilterTrackerPipeline::UpdateUnassignedTracks(
    const std::vector<base::RoadEdgeMeasurementPtr> *detected_lanelines,
    const PointAssociationResult &association_result) {
  auto &unassigned_track_indexs = association_result.unassigned_tracks;
  size_t track_index = 0;
  SimpleRoadEdgeTrackerOptions tracker_option;
  tracker_option.novatel2world_pose = novatel2world_pose_;
  for (size_t i = 0; i < unassigned_track_indexs.size(); ++i) {
    track_index = unassigned_track_indexs[i];
    HLOG_DEBUG << "start into UpdateWithoutDetectedLaneLine";
    lane_trackers_[track_index]->UpdateWithoutDetectedLaneLine(tracker_option);
    HLOG_DEBUG << "end into UpdateWithoutDetectedLaneLine";
  }
  RemoveLostTracks();

  return;
}

void RoadEdgePointFilterTrackerPipeline::CreateNewTracks(
    const std::vector<base::RoadEdgeMeasurementPtr> *detected_lanelines,
    const PointAssociationResult &association_result) {
  auto &unsigned_objects = association_result.unsigned_objects;
  size_t detect_index = 0;
  for (size_t i = 0; i < unsigned_objects.size(); ++i) {
    detect_index = unsigned_objects[i];
    // Init LaneTarget
    RoadEdgeTargetPtr lane_target_ptr = std::make_shared<RoadEdgeTarget>();
    lane_target_ptr->Init(target_init_option_,
                          detected_lanelines->at(detect_index));

    SimpleRoadEdgeTrackerPtr lane_tracker;
    lane_tracker.reset(new SimpleRoadEdgeTracker);
    tracker_init_option_.novatel2world_pose = novatel2world_pose_;
    if (lane_tracker->Init(tracker_init_option_, lane_target_ptr)) {
      lane_trackers_.emplace_back(lane_tracker);
    }
  }
  return;
}

void RoadEdgePointFilterTrackerPipeline::RemoveLostTracks() {
  size_t track_count = 0;
  for (size_t i = 0; i < lane_trackers_.size(); ++i) {
    if (lane_trackers_[i]->GetConstLaneTarget()->IsDie()) {
      HLOG_DEBUG << "LaneTarget "
                 << lane_trackers_[i]->GetConstLaneTarget()->Id()
                 << " is lost!";
      continue;
    }
    if (track_count == i) {
      track_count++;
      continue;
    }
    lane_trackers_[track_count++] = lane_trackers_[i];
  }
  lane_trackers_.resize(track_count);
  return;
}

void RoadEdgePointFilterTrackerPipeline::CollectOutputObjects(
    std::vector<base::RoadEdgePtr> *tracked_lanelines) {
  PointLaneGatekeeperOptions options;
  for (size_t i = 0; i < lane_trackers_.size(); ++i) {
    HLOG_DEBUG << "########CollectOutputObjects############";
    RoadEdgeTargetConstPtr lane_target =
        lane_trackers_[i]->GetConstLaneTarget();
    HLOG_DEBUG << "Buffer LaneTarget " << lane_target->ToStr();
    base::RoadEdgePtr output_lane_object = std::make_shared<base::RoadEdge>(
        *lane_target->GetConstTrackedLaneLine());
    if (!point_lane_gate_keeper_->AbleToOutput(
            options, lane_trackers_[i]->GetConstLaneTarget(),
            GetAllLaneTarget())) {
      HLOG_DEBUG << "LaneTrack NOT OUTPUT!";
      HLOG_DEBUG << "MinningTimestamp: "
                 << std::to_string(output_lane_object->latest_tracked_time)
                 << " "
                 << "Buffer LaneTarget "
                 << "TrackStatus: " << lane_target->ToStr() << " "
                 << "LaneTrack NOT OUTPUT!";
      continue;
    }
    // HLOG_DEBUG << "MinningTimestamp: "
    //            << std::to_string(output_lane_object->latest_tracked_time) <<
    //            " "
    //            << "Buffer LaneTarget "
    //            << "TrackStatus: " << lane_target->ToStr();

    tracked_lanelines->push_back(output_lane_object);
  }
  return;
}

std::vector<RoadEdgeTargetConstPtr>
RoadEdgePointFilterTrackerPipeline::GetAllLaneTarget() {
  lane_targets_.clear();
  for (size_t i = 0; i < lane_trackers_.size(); ++i) {
    RoadEdgeTargetConstPtr lane_target =
        lane_trackers_[i]->GetConstLaneTarget();
    lane_targets_.push_back(lane_target);
  }
  return lane_targets_;
}

std::string RoadEdgePointFilterTrackerPipeline::Name() const {
  return "RoadEdgePointFilterTrackerPipeline";
}

}  // namespace environment
}  // namespace mp
}  // namespace hozon
