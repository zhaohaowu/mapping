// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: simple_lane_tracker_pipeline.cc
// @brief: associate history lane_track to current detected lane　object

#include "modules/local_mapping/lib/laneline/tracker/bev_lane_tracker/pipeline/lane_point_tracker_pipeline.h"
#include <math.h>

#include <limits>
#include <utility>

#include "modules/local_mapping/lib/laneline/tracker/bev_lane_tracker/datalogger/load_data_singleton.h"
#include "modules/local_mapping/lib/laneline/utils/lane_utils.h"
// #include "perception-common/common/performance/perf_util.h"
#include "perception-lib/lib/config_manager/config_manager.h"
#include "perception-lib/lib/io/file_util.h"
#include "perception-lib/lib/io/protobuf_util.h"
#include "perception-lib/lib/location_manager/location_manager.h"
namespace hozon {
namespace mp {
namespace environment {

bool LanePointFilterTrackerPipeline::Init(const ProcessInitOption& options) {
  auto config_manager = perception_lib::ConfigManager::Instance();
  const perception_lib::ModelConfig* model_config = nullptr;
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
  config_file =
      perception_lib::FileUtil::GetAbsolutePath(work_root, config_file);
  CHECK(perception_lib::ParseProtobufFromFile<LanePostProcessParam>(
      config_file, &lane_post_process_param_))
      << "Read config failed: " << config_file;
  const LaneTrackerPipelineParam& lane_tracker_pipeline_param =
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

  main_road_left_line_ = std::make_shared<LaneLine>();
  main_road_right_line_ = std::make_shared<LaneLine>();

  lane_targets_.reserve(100);
  lane_trackers_.clear();
  return true;
}

void LanePointFilterTrackerPipeline::TransMeasurementVehicle2Local(
    std::vector<perception_base::LaneLineMeasurementPtr>* detect_measurements) {
  for (auto& detect_measurement : *detect_measurements) {
    auto& points = detect_measurement->point_set;
    TransVehiclePoint2Local(&points);
  }
  return;
}

void LanePointFilterTrackerPipeline::TransTrackerLocal2Vehicle(
    std::vector<perception_base::LaneLinePtr>* tracked_outputs) {
  for (auto& tracked_output : *tracked_outputs) {
    tracked_output->geo_confidence = 1.0;
    auto& points = tracked_output->point_set;
    TransLocalPoint2Vehicle(&points);
  }
  return;
}

bool LanePointFilterTrackerPipeline::Track(
    const ProcessOption& options,
    perception_base::LaneLinesMeasurementConstPtr detect_measurements_info,
    const perception_base::LaneLinesPtr& track_lanelines_info) {
  // PERF_FUNCTION();
  // PERF_BLOCK_START();

  const auto& track_lanelines = &track_lanelines_info->lanelines;
  track_lanelines->clear();

  // 1. 维护最近两帧的pose信息
  HLOG_DEBUG << "start do UpdateDrPose...";
  UpdateDrPose();
  HLOG_DEBUG << "end do UpdateDrPose...";
  std::vector<perception_base::LaneLineMeasurementPtr> vec_measurements =
      detect_measurements_info->lanelines;
  const auto& detect_measurements = &vec_measurements;
  // for (auto &cross_point : detect_measurements_info->crosspoints) {
  //   std::cout << "cross_point id:" << cross_point->id;
  //   std::cout << "cross_point type:" << int(cross_point->type);
  //   std::cout << "cross_point loc:" << cross_point->point.vehicle_point.x <<
  //   ","
  //             << cross_point->point.vehicle_point.y << ","
  //             << cross_point->point.vehicle_point.z;
  // }

  // 2. 观测车道线转换到local系
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
  for (auto& assign : point_association_result.assignments) {
    auto target_idx = std::get<0>(assign);
    auto detect_idx = std::get<1>(assign);
    HLOG_DEBUG << "assignments target_idx: " << target_idx
               << " detect_idx:" << detect_idx;
  }

  for (auto& unassign_track : point_association_result.unassigned_tracks) {
    HLOG_DEBUG << "unassign_track idx : " << unassign_track;
  }

  for (auto& unassign_object : point_association_result.unsigned_objects) {
    HLOG_DEBUG << "unassign_object idx: " << unassign_object;
  }

  HLOG_DEBUG << "end do Associate...";
  // PERF_BLOCK_END("lane_LaneAssociate");

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

  // 7. 输出准出结果
  CollectOutputObjects(track_lanelines);
  HLOG_DEBUG << "output track laneline size ============"
             << track_lanelines->size();
  // PERF_BLOCK_END("lane_CollectOutputObjects");

  // 8. 为车道线设置POS
  PostProcess(track_lanelines);
  // PERF_BLOCK_END("lane_PostProcess");

  // 9. 把跟踪点local系转换到vehicle系
  TransTrackerLocal2Vehicle(track_lanelines);

  // PERF_BLOCK_END("LanePointFilterTrackerPipeline");

  // if (lane_post_process_param_.static_strategy_param().debug_option()) {
  //   print_debug_info();
  // }

  return true;
}

void LanePointFilterTrackerPipeline::PostProcess(
    std::vector<perception_base::LaneLinePtr>* tracked_lanelines) {
  // set lane pose atrribute
  SetPoseAttribute(tracked_lanelines);

  // SetMainRoadQuality(tracked_lanelines);
}

void LanePointFilterTrackerPipeline::UpdateAssignedTracks(
    std::vector<perception_base::LaneLineMeasurementPtr>* detected_lanelines,
    const PointAssociationResult& association_result) {
  auto& assignments = association_result.assignments;
  size_t track_index = 0;
  size_t detect_index = 0;

  SimpleLaneTrackerOptions tracker_option;
  tracker_option.novatel2world_pose = novatel2world_pose_;
  for (size_t i = 0; i < assignments.size(); ++i) {
    track_index = std::get<0>(assignments[i]);
    detect_index = std::get<1>(assignments[i]);
    // HLOG_DEBUG << "point_match_debug, trackId: " <<
    // lane_trackers_[track_index]->GetConstLaneTarget()->Id()
    //           << ", detectId: " << detected_lanelines->at(detect_index)->id;
    lane_trackers_[track_index]->UpdateWithDetectedLaneLine(
        tracker_option, detected_lanelines->at(detect_index));
  }
  return;
}

void LanePointFilterTrackerPipeline::UpdateUnassignedTracks(
    const std::vector<perception_base::LaneLineMeasurementPtr>*
        detected_lanelines,
    const PointAssociationResult& association_result) {
  auto& unassigned_track_indexs = association_result.unassigned_tracks;
  size_t track_index = 0;
  SimpleLaneTrackerOptions tracker_option;
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

void LanePointFilterTrackerPipeline::CreateNewTracks(
    const std::vector<perception_base::LaneLineMeasurementPtr>*
        detected_lanelines,
    const PointAssociationResult& association_result) {
  auto& unsigned_objects = association_result.unsigned_objects;
  size_t detect_index = 0;
  for (size_t i = 0; i < unsigned_objects.size(); ++i) {
    detect_index = unsigned_objects[i];
    // Init LaneTarget
    LaneTargetPtr lane_target_ptr = std::make_shared<LaneTarget>();
    lane_target_ptr->Init(target_init_option_,
                          detected_lanelines->at(detect_index));

    SimpleLaneTrackerPtr lane_tracker;
    lane_tracker.reset(new SimpleLaneTracker);
    tracker_init_option_.novatel2world_pose = novatel2world_pose_;
    if (lane_tracker->Init(tracker_init_option_, lane_target_ptr)) {
      lane_trackers_.emplace_back(lane_tracker);
    }
  }
  return;
}

void LanePointFilterTrackerPipeline::RemoveLostTracks() {
  size_t track_count = 0;
  for (size_t i = 0; i < lane_trackers_.size(); ++i) {
    const auto& track_id = lane_trackers_[i]->GetConstLaneTarget()->Id();
    if (lane_trackers_[i]->GetConstLaneTarget()->IsDie()) {
      HLOG_DEBUG << "LaneTarget " << track_id << " is lost!";
      if (lane_pos_map.count(track_id) != 0) {
        lane_pos_map.erase(track_id);
      }
      if (lane_d_map.count(track_id) != 0) {
        lane_d_map.erase(track_id);
      }

      continue;
    }
    if (lane_result_pos_map.count(track_id) != 0) {
      lane_result_pos_map.erase(track_id);
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

void LanePointFilterTrackerPipeline::SetPoseAttribute(
    std::vector<perception_base::LaneLinePtr>* tracked_lanelines) {
  const auto& lane_tracker_pipeline_param =
      lane_post_process_param_.lane_tracker_pipeline_param();
  float ref_min =
      lane_tracker_pipeline_param.lane_pose_setter_param().ref_min();
  float ref_length =
      lane_tracker_pipeline_param.lane_pose_setter_param().ref_length();
  int sample_num =
      lane_tracker_pipeline_param.lane_pose_setter_param().sample_point_num();

  PointLaneGatekeeperOptions options;
  // HLOG_DEBUG << "lane_nums:" << tracked_lanelines->size();
  // SetLanePosition(ref_min, ref_length, sample_num, *tracked_lanelines);

  std::vector<bool> far_lanes_flag = CheckLanesDistance(tracked_lanelines);
  bool disappear_flag =
      IsMainRoadAboutDisappear(tracked_lanelines, far_lanes_flag);
  // 解决既有近处又有远处车道线但是都在一端赋值错误的case
  revise_lanes_flag(tracked_lanelines, far_lanes_flag);

  if (disappear_flag) {
    // 如果主车道线较短，即将消失，则针对远处车道线进行位置赋值,
    // 近端赋值为OTHER.
    SetLanePosition(ref_min, ref_length, sample_num, *tracked_lanelines,
                    far_lanes_flag, &lane_d_map, &lane_result_pos_map, true);
  } else {
    // 如果主车道线长度一般，则针对近处车道线进行位置赋值, 远端赋值为OTHER.
    SetLanePosition(ref_min, ref_length, sample_num, *tracked_lanelines,
                    far_lanes_flag, &lane_d_map, &lane_result_pos_map, false);
  }
  // 判断是否稳定，稳定时再判断当pos改变时是否要修改pos
  bool stable_flag = CheckStableFlag(tracked_lanelines, disappear_flag);
  bool stable_pos_flag = CheckStablePosFlag(tracked_lanelines);
  RevisePose(tracked_lanelines, stable_flag, stable_pos_flag);
}

void LanePointFilterTrackerPipeline::revise_lanes_flag(
    const std::vector<perception_base::LaneLinePtr>* tracked_lanelines,
    std::vector<bool>& far_lanes_flag) {
  std::vector<perception_base::LaneLinePtr> near_lanelines;
  float near_start = std::numeric_limits<float>::max(),
        near_end = std::numeric_limits<float>::max();
  for (int i = 0; i < far_lanes_flag.size(); ++i) {
    if (!far_lanes_flag[i]) {
      auto laneline = tracked_lanelines->at(i);
      near_lanelines.emplace_back(laneline);
      near_start =
          std::min(laneline->point_set.front().vehicle_point.x, near_start);
      near_end = std::min(laneline->point_set.back().vehicle_point.x, near_end);
    }
  }
  for (int i = 0; i < far_lanes_flag.size(); ++i) {
    if (far_lanes_flag[i]) {
      const auto& laneline = tracked_lanelines->at(i);
      float overlap_start =
          std::max(near_start, laneline->point_set.front().vehicle_point.x);
      float overlap_end =
          std::min(near_end, laneline->point_set.back().vehicle_point.x);
      if (overlap_end - overlap_start > 10.0) {
        far_lanes_flag[i] = false;
      }
    }
  }
  return;
}

bool LanePointFilterTrackerPipeline::CheckStableFlag(
    const std::vector<perception_base::LaneLinePtr>* tracked_lanelines,
    bool disappear_flag) {
  // 判断是否稳定前行
  bool stable_flag = false;
  int count_lane = 0;
  for (int i = 0; i < tracked_lanelines->size(); ++i) {
    LaneLinePtr lane_target = (*tracked_lanelines)[i];
    auto iter_d = lane_d_map.find(lane_target->id);
    if (iter_d != lane_d_map.end()) {
      auto& d_error = std::get<1>(iter_d->second);
      if (d_error < d_change_threshold_) {
        count_lane++;
      }
    }
    if (count_lane >= 2 && disappear_flag == false) {
      stable_flag = true;
      break;
    }
  }
  return stable_flag;
}

bool LanePointFilterTrackerPipeline::CheckEgoPose(LaneLinePosition pose) {
  return ((pose == LaneLinePosition::EGO_LEFT) ||
          (pose == LaneLinePosition::EGO_RIGHT));
}

bool LanePointFilterTrackerPipeline::CheckStablePosFlag(
    const std::vector<perception_base::LaneLinePtr>* tracked_lanelines) {
  bool stable_pos_flag = false;
  for (int i = 0; i < tracked_lanelines->size(); ++i) {
    LaneLinePtr lane_target = (*tracked_lanelines)[i];
    HLOG_DEBUG << "Position filter output:" << int(lane_target->position);
    auto iter_pos = lane_pos_map.find(lane_target->id);
    if (iter_pos != lane_pos_map.end()) {
      const auto& last_pos = iter_pos->second;
      if (last_pos == LaneLinePosition::OTHER &&
          CheckEgoPose(lane_target->position)) {
        // 过路口场景
        stable_pos_flag = false;
        break;
      } else if (std::abs(static_cast<int>(lane_target->position) -
                          static_cast<int>(last_pos)) == 2) {
        // 变道场景
        stable_pos_flag = false;
        break;
      } else if (std::abs(static_cast<int>(lane_target->position)) <
                 std::abs(static_cast<int>(last_pos))) {
        if (CheckEgoPose(lane_target->position)) {
          // 维持上一帧pos不变
          stable_pos_flag = true;
        }
      } else {
        continue;
      }
    }
  }
  return stable_pos_flag;
}
// todo: 若外侧新建线会出现-2, -2的情况；
void LanePointFilterTrackerPipeline::RevisePose(
    const std::vector<perception_base::LaneLinePtr>* tracked_lanelines,
    bool stable_flag, bool stable_pos_flag) {
  std::set<int> pos_set;
  pos_set.clear();
  int valid_flag = 0;
  for (int i = 0; i < tracked_lanelines->size(); ++i) {
    LaneLinePtr lane_target = (*tracked_lanelines)[i];
    auto iter_pos = lane_pos_map.find(lane_target->id);
    if (iter_pos != lane_pos_map.end()) {
      if (stable_flag && stable_pos_flag) {
        lane_target->position = iter_pos->second;
      } else {
        iter_pos->second = lane_target->position;
      }
    } else {
      lane_pos_map.emplace(
          std::make_pair(lane_target->id, lane_target->position));
    }
  }
  if (stable_flag && stable_pos_flag) {
    pos_stable_count_++;
  } else {
    pos_stable_count_ = 0;
  }

  for (int i = 0; i < tracked_lanelines->size(); ++i) {
    LaneLinePtr lane_target = (*tracked_lanelines)[i];
    HLOG_DEBUG << "Position filter output result:"
               << int(lane_target->position);
    if (abs(static_cast<int>(lane_target->position)) < 4) {
      pos_set.insert(static_cast<int>(lane_target->position));
      valid_flag++;
    }
  }
  if (pos_set.size() != valid_flag) {
    for (int i = 0; i < tracked_lanelines->size(); ++i) {
      LaneLinePtr lane_target = (*tracked_lanelines)[i];
      auto iter_pos = lane_result_pos_map.find(lane_target->id);
      if (iter_pos != lane_result_pos_map.end()) {
        lane_target->position = iter_pos->second;
      }
    }
  }
}

bool LanePointFilterTrackerPipeline::IsMainRoadAboutDisappear(
    const std::vector<perception_base::LaneLinePtr>* tracked_lanelines,
    const std::vector<bool>& far_line_index) {
  // 获取近处的车道线
  std::vector<perception_base::LaneLinePtr> near_lanelines;
  for (int i = 0; i < far_line_index.size(); ++i) {
    if (!far_line_index[i]) {
      auto laneline = tracked_lanelines->at(i);
      near_lanelines.push_back(laneline);
    }
  }

  const auto& lane_tracker_pipeline_param =
      lane_post_process_param_.lane_tracker_pipeline_param();
  float ref_min =
      lane_tracker_pipeline_param.lane_pose_setter_param().ref_min();
  float ref_length =
      lane_tracker_pipeline_param.lane_pose_setter_param().ref_length();
  int sample_num =
      lane_tracker_pipeline_param.lane_pose_setter_param().sample_point_num();

  // 近处的车道线进行位置赋值，并找到主车道线
  SetLanePosition(ref_min, ref_length, sample_num, near_lanelines);
  float ego_left_loc, ego_right_loc = std::numeric_limits<float>::min();
  for (auto& laneline : near_lanelines) {
    if (laneline->position == LaneLinePosition::EGO_LEFT) {
      ego_left_loc = laneline->point_set.back().vehicle_point.x;
    } else if (laneline->position == LaneLinePosition::EGO_RIGHT) {
      ego_right_loc = laneline->point_set.back().vehicle_point.x;
    } else {
      continue;
    }
  }
  // 解决远端没有线，但是近端小于5m的case
  if (near_lanelines.size() == far_line_index.size()) {
    return false;
  }
  if (ego_left_loc < dist_separation_point_ &&
      ego_right_loc < dist_separation_point_) {
    // 如果主车道线都丢失， 会拿远端的车道线
    // 如果主车道线丢失其中一根，另一根远端点小于5米，拿远端的车道线。
    // 如果主车道线都存在， 远短点都小于5米， 拿远端的车道线。
    return true;
  }

  return false;
}

std::vector<bool> LanePointFilterTrackerPipeline::CheckLanesDistance(
    const std::vector<perception_base::LaneLinePtr>* tracked_lanelines) {
  std::vector<bool> is_far_laneline(tracked_lanelines->size(), false);
  for (int i = 0; i < tracked_lanelines->size(); ++i) {
    auto vehicle_min =
        tracked_lanelines->at(i)->point_set.front().vehicle_point.x;
    auto vehicle_max =
        tracked_lanelines->at(i)->point_set.back().vehicle_point.x;
    if (vehicle_max <= dist_separation_point_) {
      is_far_laneline[i] = false;
    } else if ((vehicle_max > dist_separation_point_) &&
               (vehicle_min <= dist_separation_point_)) {
      is_far_laneline[i] = false;
    } else {
      is_far_laneline[i] = true;
    }
  }
  return is_far_laneline;
}

void LanePointFilterTrackerPipeline::GetMainRoadLaneLine(
    LaneLinePtr left_laneline, LaneLinePtr right_laneline) {
  PointLaneGatekeeperOptions options;
  for (int i = 0; i < lane_trackers_.size(); ++i) {
    LaneTargetConstPtr lane_target = lane_trackers_[i]->GetConstLaneTarget();
    if (point_lane_gate_keeper_->AbleToOutput(
            options, lane_trackers_[i]->GetConstLaneTarget(),
            GetAllLaneTarget())) {
      auto& position = lane_target->GetConstTrackedLaneLine()->position;
      if (position == LaneLinePosition::EGO_LEFT) {
        left_laneline = lane_target->GetConstTrackedLaneLine();
      } else if (position == LaneLinePosition::EGO_RIGHT) {
        right_laneline = lane_target->GetConstTrackedLaneLine();
      }
    }
  }
}
// void LanePointFilterTrackerPipeline::SetMainRoadQuality() {
//   LaneLinePtr left_laneline = nullptr;
//   LaneLinePtr right_laneline = nullptr;
//   GetMainRoadLaneLine(left_laneline, right_laneline);

//   bool is_left_short =
//       (left_laneline == nullptr ||
//        laneline_quality_evaluator_->EvaluateSingleLaneline(left_laneline) ==
//            LaneLineQualityStatus::ERROR_LENGTH);
//   bool is_right_short =
//       (right_laneline == nullptr ||
//        laneline_quality_evaluator_->EvaluateSingleLaneline(right_laneline) ==
//            LaneLineQualityStatus::ERROR_LENGTH);

//   // compute left_main_laneline quality
//   LaneLineQualityStatus main_road_left_quality =
//   LaneLineQualityStatus::NORMAL; if (is_last_main_road_left_exist_ &&
//   left_laneline != nullptr) {
//     LaneLineCurve transform_curve;
//     bool status = TransformLaneLineCurveInNovatel(
//         main_road_left_line_->vehicle_curve,
//         novatel2world_pose_.inverse() * last_novatel2world_pose_,
//         &transform_curve);
//     if (status) {
//       // HLOG_DEBUG << "Left_lane, before transform:"
//       //            <<
//       GetLaneLineCurveInfo(main_road_left_line_->vehicle_curve)
//       //            << ", after :" << GetLaneLineCurveInfo(transform_curve);
//       main_road_left_line_->vehicle_curve = transform_curve;
//       main_road_left_quality =
//           laneline_quality_evaluator_->EvaluateTwoLanelines(
//               left_laneline, main_road_left_line_);
//       HLOG_DEBUG << "Main road left lane quality for pnc staus! "
//                  << kLaneLineQualityStausToString.at(main_road_left_quality);
//     }
//   }

//   // compute right_main_laneline quality
//   LaneLineQualityStatus main_road_right_quality =
//   LaneLineQualityStatus::NORMAL; if (is_last_main_road_right_exist_ &&
//   right_laneline != nullptr) {
//     LaneLineCurve transform_curve;
//     bool status = TransformLaneLineCurveInNovatel(
//         main_road_right_line_->vehicle_curve,
//         novatel2world_pose_.inverse() * last_novatel2world_pose_,
//         &transform_curve);
//     if (status) {
//       // HLOG_DEBUG << "Right_lane, before transform:"
//       // << GetLaneLineCurveInfo(main_road_right_line_->vehicle_curve)
//       // << ", after :"
//       // << GetLaneLineCurveInfo(transform_curve);
//       main_road_right_line_->vehicle_curve = transform_curve;
//       main_road_right_quality =
//           laneline_quality_evaluator_->EvaluateTwoLanelines(
//               right_laneline, main_road_right_line_);
//       HLOG_DEBUG << "Main road right lane quality for pnc staus! "
//                  <<
//                  kLaneLineQualityStausToString.at(main_road_right_quality);
//     }
//   }

//   // bool is_right_low_quality =
//   //     (main_road_right_quality == LaneLineQualityStatus::ERROR_JUMP ||
//   //     main_road_right_quality == LaneLineQualityStatus::ERROR_POSITION);
//   // if (is_right_low_quality) {
//   //   HLOG_DEBUG << "[Single LaneLine] Main road right lane low0 quality! "
//   //   << kLaneLineQualityStausToString.at(main_road_right_quality);
//   //   right_laneline->quality = LaneLineQuality::LOW0;
//   // }

//   // bool is_left_low_quality =
//   //     (main_road_left_quality == LaneLineQualityStatus::ERROR_JUMP ||
//   //     main_road_left_quality == LaneLineQualityStatus::ERROR_POSITION);
//   // if (is_left_low_quality) {
//   //   HLOG_DEBUG << "[Single LaneLine] Main road left lane low0 quality! "
//   //   << kLaneLineQualityStausToString.at(main_road_left_quality);
//   //   left_laneline->quality = LaneLineQuality::LOW0;
//   // }

//   if ((is_left_short && is_right_short) ||
//       (!is_left_short && !is_right_short)) {
//     // both long or both short, no need to set main road quality
//   } else if (is_left_short) {
//     // left_lane is short, right_lane is long, set right_lane quality
//     bool is_right_low_quality =
//         (main_road_right_quality == LaneLineQualityStatus::ERROR_JUMP ||
//          main_road_right_quality == LaneLineQualityStatus::ERROR_POSITION);
//     if (is_right_low_quality) {
//       HLOG_DEBUG << "[Single LaneLine] Main road right lane low0 quality! "
//                  <<
//                  kLaneLineQualityStausToString.at(main_road_right_quality);
//       right_laneline->quality = LaneLineQuality::LOW0;
//     }
//   } else {
//     // right_lane is short, left_lane is long ,set left_lane quality
//     bool is_left_low_quality =
//         (main_road_left_quality == LaneLineQualityStatus::ERROR_JUMP ||
//          main_road_left_quality == LaneLineQualityStatus::ERROR_POSITION);
//     if (is_left_low_quality) {
//       HLOG_DEBUG << "[Single LaneLine] Main road left lane low0 quality! "
//                  << kLaneLineQualityStausToString.at(main_road_left_quality);
//       left_laneline->quality = LaneLineQuality::LOW0;
//     }
//   }

//   if (left_laneline != nullptr) {
//     is_last_main_road_left_exist_ = true;
//     *main_road_left_line_ = *left_laneline;
//   } else {
//     is_last_main_road_left_exist_ = false;
//   }

//   if (right_laneline != nullptr) {
//     is_last_main_road_right_exist_ = true;
//     *main_road_right_line_ = *right_laneline;
//   } else {
//     is_last_main_road_right_exist_ = false;
//   }
// }

void LanePointFilterTrackerPipeline::CollectOutputObjects(
    std::vector<LaneLinePtr>* tracked_lanelines) {
  PointLaneGatekeeperOptions options;
  for (size_t i = 0; i < lane_trackers_.size(); ++i) {
    HLOG_DEBUG << "########CollectOutputObjects############";
    LaneTargetConstPtr lane_target = lane_trackers_[i]->GetConstLaneTarget();
    HLOG_DEBUG << "Buffer LaneTarget " << lane_target->ToStr();
    LaneLinePtr output_lane_object =
        std::make_shared<LaneLine>(*lane_target->GetConstTrackedLaneLine());
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

std::vector<LaneTargetConstPtr>
LanePointFilterTrackerPipeline::GetAllLaneTarget() {
  lane_targets_.clear();
  for (size_t i = 0; i < lane_trackers_.size(); ++i) {
    LaneTargetConstPtr lane_target = lane_trackers_[i]->GetConstLaneTarget();
    lane_targets_.push_back(lane_target);
  }
  return lane_targets_;
}

std::string LanePointFilterTrackerPipeline::Name() const {
  return "LanePointFilterTrackerPipeline";
}

void LanePointFilterTrackerPipeline::print_debug_info() {
  // const auto lane_debug = lane_debug::LaneDebugDataSingleton::Instance();
  // auto cur_ts = lane_debug->post_process_debug_data_ptr_->current_timestamp;
  // auto &filter_module_debug_info =
  //     lane_debug->post_process_debug_data_ptr_->tracker_debug_data
  //         ->filter_module_debug;

  // auto &current_pose =
  //     InputDataSingleton::Instance()->dr_data_buffer_.back()->pose;

  // // vehicle_pt = novatel2world_pose_.inverse() * local_pt;

  // for (auto &track_info : filter_module_debug_info) {
  //   HLOG_DEBUG << "track_id" << track_info->track_id;
  // }

  // for (auto &track_info : filter_module_debug_info) {
  //   for (auto &point : track_info->measurement_points) {
  //     // auto vehicle_point = current_pose.inverse() *
  //     // Eigen::Vector3d{point.local_point.x, point.local_point.y,
  //     // point.local_point.z};

  //     HLOG_DEBUG << " [lane_debug] "
  //               << " [filter:] "
  //               << " [current_ts:] " << std::to_string(cur_ts)
  //               << " [measurement point:] "
  //               << " [track_id] " << track_info->track_id << " [detect_id] "
  //               << track_info->detect_id << " [x] " << point.local_point.x
  //               << ","
  //               << " [y] " << point.local_point.y << ","
  //               << " [z] " << point.local_point.z;
  //   }
  // }

  // for (auto &track_info : filter_module_debug_info) {
  //   for (auto &point : track_info->update_points) {
  //     HLOG_DEBUG << " [lane_debug] "
  //               << " [filter:] "
  //               << " [current_ts:] " << std::to_string(cur_ts)
  //               << " [update point:] "
  //               << " [track_id] " << track_info->track_id << " [detect_id] "
  //               << track_info->detect_id << " [x] " << point.local_point.x
  //               << ","
  //               << " [y] " << point.local_point.y << ","
  //               << " [z] " << point.local_point.z;
  //   }
  // }

  // for (auto &track_info : filter_module_debug_info) {
  //   for (auto &point : track_info->predict_points) {
  //     HLOG_DEBUG << " [lane_debug] "
  //               << " [filter:] "
  //               << " [current_ts:] " << std::to_string(cur_ts)
  //               << " [predict point:] "
  //               << " [track_id] " << track_info->track_id << " [detect_id] "
  //               << track_info->detect_id << " [x] " << point.local_point.x
  //               << ","
  //               << " [y] " << point.local_point.y << ","
  //               << " [z] " << point.local_point.z;
  //   }
  // }

  // for (auto &track_info : filter_module_debug_info) {
  //   for (auto &point : track_info->tracked_points) {
  //     HLOG_DEBUG << " [lane_debug] "
  //               << " [filter:] "
  //               << " [current_ts:] " << std::to_string(cur_ts)
  //               << " [tracked point:] "
  //               << " [track_id] " << track_info->track_id << " [detect_id] "
  //               << track_info->detect_id << " [x] " << point.local_point.x
  //               << ","
  //               << " [y] " << point.local_point.y << ","
  //               << " [z] " << point.local_point.z;
  //   }
  // }

  return;
}

// Register plugin.
// REGISTER_LANE_TRACKER(LanePointFilterTrackerPipeline);

}  // namespace environment
}  // namespace mp
}  // namespace hozon
