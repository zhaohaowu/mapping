// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: simple_lane_tracker_pipeline.cc
// @brief: associate history lane_track to current detected lane　object

#include <math.h>

#include <unordered_set>

#include "modules/local_mapping/lib/datalogger/pose_manager.h"
#include "modules/local_mapping/lib/pipeline/roadedge_mapping_pipeline.h"
#include "modules/local_mapping/utils/lane_utils.h"
#include "modules/util/include/util/orin_trigger_manager.h"
#include "perception-lib/lib/config_manager/config_manager.h"
#include "perception-lib/lib/io/file_util.h"
#include "perception-lib/lib/io/protobuf_util.h"
#include "perception-lib/lib/location_manager/location_manager.h"

namespace hozon {
namespace mp {
namespace lm {

bool RoadEdgeMappingPipeline::Init() {
  roadedge_matcher_ = std::make_unique<RoadEdgeMatcher>();
  MatcherInitOptions matcher_init_options;
  CHECK(roadedge_matcher_->Init(matcher_init_options));

  roadedge_gate_keeper_ = std::make_unique<RoadEdgeGatekeeper>();
  roadedge_gate_keeper_->Init();
  roadedge_position_manager_ = std::make_unique<RoadEdgePositionManager>();
  roadedge_position_manager_->Init();
  post_roadedge_position_manager_ =
      std::make_unique<PostRoadEdgePositionManager>();
  post_roadedge_position_manager_->Init();
  roadedge_targets_.reserve(100);
  roadedge_trackers_.clear();

  return true;
}
void RoadEdgeMappingPipeline::PostProcess(RoadEdgesPtr tracked_roadedges) {
  // set roadedge pose atrribute
  if (tracked_roadedges->road_edges.empty()) {
    return;
  }
  roadedge_position_manager_->Process(tracked_roadedges);
  post_roadedge_position_manager_->Process(tracked_roadedges);
}
bool RoadEdgeMappingPipeline::Process(
    const ProcessOption& option, MeasurementFrameConstPtr measurement_frame_ptr,
    LocalMapFramePtr localmap_frame_ptr) {
  auto& tracked_roadedges = localmap_frame_ptr->road_edges_ptr;
  tracked_roadedges->road_edges.clear();
  // 1. track更新
  UpdateTracks();

  // 2. 观测线过滤模块
  std::vector<RoadEdgePtr>& measurements_roadedges =
      measurement_frame_ptr->road_edges_ptr->road_edges;

  std::vector<RoadEdgePtr> detect_measurements =
      CleanMeasureData(&measurements_roadedges);
  // 2. 观测车道线转换到local系
  HLOG_DEBUG << "start sync roadedge TransMeasurementVehicle2Local...";
  TransMeasurementVehicle2Local(&detect_measurements);
  HLOG_DEBUG << "finish sync roadedge TransMeasurementVehicle2Local...";
  // 3. 观测线和跟踪线关联匹配
  HLOG_DEBUG << "start do roadedge associate...";
  AssociationResult point_association_result;
  MatcherOptions matcher_options;
  matcher_options.timestamp = measurement_frame_ptr->header.timestamp;
  roadedge_matcher_->Associate(matcher_options, detect_measurements,
                               roadedge_trackers_, &point_association_result);

  for (auto& assign : point_association_result.assignments) {
    auto target_idx = std::get<0>(assign);
    auto detect_idx = std::get<1>(assign);
    HLOG_DEBUG << "assignments target_idx: " << target_idx
               << " detect_idx:" << detect_idx;
  }

  HLOG_DEBUG << "end do roadedge associate...";

  // 4. update matched lane_tracks
  UpdateAssignedTracks(option, detect_measurements, point_association_result);
  // 5. update unmatched lane_tracks

  UpdateUnassignedTracks(option, detect_measurements, point_association_result);

  // 6. created new lane_tracks for unmatched detected_lanes
  CreateNewTracks(option, detect_measurements, point_association_result);
  MergeTracks(&roadedge_trackers_);

  LimitTracksNum();
  // 7. 输出准出结果
  CollectOutputObjects(tracked_roadedges);
  // 8. postprocess
  PostProcess(tracked_roadedges);
  // 9. catmallrom拟合路边沿
  CatmullRomFit(tracked_roadedges);

// 当tracker中存在nan值时进行触发
#ifdef ISORIN
  if (roadedge_nanvalue_trigger()) {
    CheckTriggerRoadedgeNan();
  }
#endif

  return true;
}

std::string RoadEdgeMappingPipeline::Name() const {
  return "RoadEdgeMappingPipeline";
}
bool CheckReverseFlag(const RoadEdgeTargetPtr& target) {
  // 取首尾5个点求向量
  const auto& line_pts = target->GetTrackedObject()->vehicle_points;
  int pt_size = static_cast<int>(line_pts.size());
  if (pt_size < 10) {
    return false;
  }
  const Eigen::Vector3d unit_vec(1.0, 0.0, 0.0);
  const int count_num = 5;
  double avg_cos_theta = 0.0;
  for (int i = 0; i < count_num; ++i) {
    const auto& first_pt = line_pts[i];
    const auto& second_pt = line_pts[pt_size - i - 1];
    auto direct_vec = second_pt - first_pt;
    avg_cos_theta +=
        direct_vec.dot(unit_vec) / (direct_vec.norm() + 0.001) / count_num;
  }
  bool reverse_flag = avg_cos_theta < -0.5;
  HLOG_DEBUG << "CheckReverseFlag: " << target->GetConstTrackedObject()->id
             << ", first_pt x: " << line_pts.front().x()
             << ", y: " << line_pts.front().y()
             << ", back_pt x: " << line_pts.back().x()
             << ", y: " << line_pts.back().y()
             << ", avg_cos_theta: " << avg_cos_theta
             << ", reverse_flag: " << reverse_flag;
  return reverse_flag;
}
void RoadEdgeMappingPipeline::UpdateTracks() {
  std::vector<RoadEdgeTargetPtr> targets = GetAllTarget();

  const Eigen::Affine3d T_cur_last_ = POSE_MANAGER->GetDeltaPose();

  reverse_x_y_flag_ = false;
  for (auto& target : targets) {
    auto& vehicle_points = target->GetTrackedObject()->vehicle_points;
    for (auto& point : vehicle_points) {
      point = T_cur_last_ * point;
    }
    if (CheckReverseFlag(target)) {
      std::reverse(vehicle_points.begin(), vehicle_points.end());
      reverse_x_y_flag_ = true;
    }
  }
}

// tracker 合并策略
void RoadEdgeMappingPipeline::MergeTracks(
    std::vector<RoadEdgeTrackerPtr>* trackers) {
  if (trackers->size() < 2) {
    return;
  }
  std::unordered_set<int> remove_index;
  for (int i = 0; i < trackers->size() - 1; ++i) {
    const auto& left_line = trackers->at(i)->GetConstTarget();
    if (!left_line->IsTracked()) {
      continue;
    }
    for (int j = i + 1; j < trackers->size(); ++j) {
      const auto& right_line = trackers->at(j)->GetConstTarget();
      if (!right_line->IsTracked()) {
        continue;
      }
      double over_lay_ratio =
          GetOverLayRatioBetweenTwoLane(left_line->GetConstTrackedObject(),
                                        right_line->GetConstTrackedObject());
      double over_lay_y_ratio =
          GetOverLayYRatioBetweenTwoLane(left_line->GetConstTrackedObject(),
                                         right_line->GetConstTrackedObject());
      float avg_dist = GetDistBetweenTwoLane(
          left_line->GetConstTrackedObject()->vehicle_points,
          right_line->GetConstTrackedObject()->vehicle_points);
      double time_diff = left_line->GetLastestTrackedTimestamp() -
                         right_line->GetLastestTrackedTimestamp();
      bool over_lay_y_condition = reverse_x_y_flag_ && over_lay_y_ratio > 0.7;
      HLOG_DEBUG << "roadedge MergeTracks: id " << left_line->Id() << ", id "
                 << right_line->Id() << ", avg_dist: " << avg_dist
                 << ", overlay_ratio: " << over_lay_ratio
                 << ", overlay_y_ratio: " << over_lay_y_ratio
                 << ", reverse_x_y_flag_: " << reverse_x_y_flag_
                 << ", time_diff: " << time_diff;
      // 两条Tracker重合度很高
      // 根据线的质量来做删除,需要根据case专门抽一个评估函数
      if ((over_lay_ratio > 0.7 || over_lay_y_condition) && avg_dist < 1.5) {
        if (over_lay_y_condition) {
          // 转弯场景，xy轴互换了
          const auto& pts1 = left_line->GetConstTrackedObject()->vehicle_points;
          auto end_y_1 =
              std::max_element(pts1.begin(), pts1.end(),
                               [](const auto& left, const auto& right) {
                                 return left.y() < right.y();
                               });
          const auto& pts2 =
              right_line->GetConstTrackedObject()->vehicle_points;
          auto end_y_2 =
              std::max_element(pts2.begin(), pts2.end(),
                               [](const auto& left, const auto& right) {
                                 return left.y() < right.y();
                               });
          auto abs_error = std::abs(end_y_2->y() - end_y_1->y());
          // 端点相差不大时保留跟踪久的
          if (abs_error < 2.0) {
            if (left_line->Count() > right_line->Count()) {
              remove_index.insert(right_line->Id());
            } else {
              remove_index.insert(left_line->Id());
            }
          }
        } else {
          // 两条tracker时间差超过2帧保存最新的
          if (std::abs(time_diff) > 0.2) {
            if (time_diff > 0) {
              remove_index.insert(right_line->Id());
            } else {
              remove_index.insert(left_line->Id());
            }
          } else {
            // 时间差不超过2帧保存跟踪时间长的
            if (left_line->Count() > right_line->Count()) {
              remove_index.insert(right_line->Id());
            } else {
              remove_index.insert(left_line->Id());
            }
          }
        }
      } else {
        // 待根据case补充删除策略
        continue;
      }
    }
  }
  trackers->erase(std::remove_if(trackers->begin(), trackers->end(),
                                 [&](const auto& tracker) {
                                   return remove_index.count(
                                              tracker->GetConstTarget()->Id()) >
                                          0;
                                 }),
                  trackers->end());
}

std::vector<RoadEdgePtr> RoadEdgeMappingPipeline::CleanMeasureData(
    const std::vector<RoadEdgePtr>* measurement_datas) {
  std::vector<RoadEdgePtr> output_measurement_datas;
  output_measurement_datas.clear();

  for (const auto& measure_data : *measurement_datas) {
    output_measurement_datas.push_back(measure_data);
  }

  return output_measurement_datas;
}

void RoadEdgeMappingPipeline::UpdateAssignedTracks(
    const ProcessOption& tracker_option,
    const std::vector<RoadEdgePtr>& detect_measurements,
    const AssociationResult& association_result) {
  const auto& assignments = association_result.assignments;
  size_t track_index = 0;
  size_t detect_index = 0;
  // todo
  for (const auto& assignment : assignments) {
    track_index = std::get<0>(assignment);
    detect_index = std::get<1>(assignment);

    roadedge_trackers_[track_index]->UpdateWithDetectedObject(
        tracker_option, detect_measurements[detect_index]);
  }
}

void RoadEdgeMappingPipeline::UpdateUnassignedTracks(
    const ProcessOption& tracker_option,
    const std::vector<RoadEdgePtr>& detect_measurements,
    const AssociationResult& association_result) {
  const auto& unassigned_track_indexs = association_result.unassigned_tracks;
  size_t track_index = 0;
  // todo
  for (const auto& index : unassigned_track_indexs) {
    roadedge_trackers_[index]->UpdateWithoutDetectedObject(tracker_option);
  }
  RemoveLostTracks();
}

void RoadEdgeMappingPipeline::CreateNewTracks(
    const ProcessOption& tracker_option,
    const std::vector<RoadEdgePtr>& detect_measurements,
    const AssociationResult& association_result) {
  const auto& unsigned_objects = association_result.unsigned_objects;
  size_t detect_index = 0;
  for (const auto& detect_index : unsigned_objects) {
    // Init RoadEdgeTarget
    RoadEdgeTargetPtr roadedge_target_ptr = std::make_shared<RoadEdgeTarget>();
    roadedge_target_ptr->Init(tracker_option,
                              detect_measurements[detect_index]);

    RoadEdgeTrackerPtr roadedge_tracker = std::make_shared<RoadEdgeTracker>();
    // tracker_init_option_.novatel2world_pose = novatel2world_pose_;
    if (roadedge_tracker->Init(tracker_option, roadedge_target_ptr)) {
      roadedge_trackers_.emplace_back(roadedge_tracker);
    }
  }
}

void RoadEdgeMappingPipeline::RemoveLostTracks() {
  size_t track_count = 0;
  for (size_t i = 0; i < roadedge_trackers_.size(); ++i) {
    const auto& track_id = roadedge_trackers_[i]->GetConstTarget()->Id();
    if (roadedge_trackers_[i]->GetConstTarget()->IsDie()) {
      continue;
    }
    if (roadedge_trackers_[i]
            ->GetConstTarget()
            ->GetConstTrackedObject()
            ->vehicle_points.back()
            .x() < -80.0) {
      continue;
    }
    if (roadedge_trackers_[i]
            ->GetConstTarget()
            ->GetConstTrackedObject()
            ->vehicle_points.size() < 5) {
      continue;
    }

    if (CheckBadTrack(roadedge_trackers_[i])) {
      continue;
    }

    if (track_count == i) {
      track_count++;
      continue;
    }
    roadedge_trackers_[track_count++] = roadedge_trackers_[i];
  }
  roadedge_trackers_.resize(track_count);
}

bool RoadEdgeMappingPipeline::CompareTrackTime(const RoadEdgeTrackerPtr& d1,
                                               const RoadEdgeTrackerPtr& d2) {
  return d1->GetConstTarget()->GetLastestTrackedTimestamp() >
         d2->GetConstTarget()->GetLastestTrackedTimestamp();
}

void RoadEdgeMappingPipeline::LimitTracksNum() {
  roadedge_trackers_.erase(
      std::remove_if(
          roadedge_trackers_.begin(), roadedge_trackers_.end(),
          [&](const auto& tracker) { return CheckBadTrack(tracker); }),
      roadedge_trackers_.end());
  std::sort(roadedge_trackers_.begin(), roadedge_trackers_.end(),
            CompareTrackTime);
  if (roadedge_trackers_.size() > limit_max_tracker_nums_) {
    roadedge_trackers_.resize(limit_max_tracker_nums_);
  }
}

void RoadEdgeMappingPipeline::CollectOutputObjects(
    RoadEdgesPtr tracked_roadedges) {
  for (const auto& roadedge_tracker : roadedge_trackers_) {
    HLOG_DEBUG << "########CollectOutputObjects############";
    RoadEdgeTargetConstPtr roadedge_target = roadedge_tracker->GetConstTarget();
    HLOG_DEBUG << "Buffer RoadEdgeTarget " << roadedge_target->ToStr();
    RoadEdgePtr output_roadedge_object =
        std::make_shared<RoadEdge>(*roadedge_target->GetConstTrackedObject());
    if (!roadedge_gate_keeper_->AbleToOutput(roadedge_tracker->GetConstTarget(),
                                             GetAllConstTarget())) {
      HLOG_DEBUG << "RoadedgeTarget TrackStatus: " << roadedge_target->ToStr()
                 << ", RoadedgeTrack NOT OUTPUT!";
      continue;
    }
    output_roadedge_object->send_postlane = roadedge_target->SendPostLane();
    tracked_roadedges->road_edges.push_back(output_roadedge_object);
  }
  HLOG_DEBUG << "RoadedgeTrack output size: "
             << tracked_roadedges->road_edges.size();
}

void RoadEdgeMappingPipeline::CatmullRomFit(RoadEdgesPtr tracked_roadedges) {
  auto euclidean_distance = [](const Eigen::Vector3d& p0,
                               const Eigen::Vector3d& p1) {
    return static_cast<int>(round(std::sqrt(std::pow(p0.x() - p1.x(), 2) +
                                            std::pow(p0.y() - p1.y(), 2))));
  };
  for (auto& roadedgeptr : tracked_roadedges->road_edges) {
    if (roadedgeptr->vehicle_points.size() < 4) {
      continue;
    }
    std::vector<Eigen::Vector3d> fit_points;
    int gap_front = euclidean_distance(roadedgeptr->vehicle_points[0],
                                       roadedgeptr->vehicle_points[1]);
    if (gap_front == 0) {
      fit_points.emplace_back(roadedgeptr->vehicle_points[0]);
    }
    for (int i = 0; i < gap_front; i++) {
      fit_points.emplace_back(roadedgeptr->vehicle_points[0] * (gap_front - i) /
                                  gap_front +
                              roadedgeptr->vehicle_points[1] * i / gap_front);
    }  // 首尾点线性插值
    for (int i = 0;
         i < static_cast<int>(roadedgeptr->vehicle_points.size()) - 3; i++) {
      std::vector<Eigen::Vector3d> points{roadedgeptr->vehicle_points[i],
                                          roadedgeptr->vehicle_points[i + 1],
                                          roadedgeptr->vehicle_points[i + 2],
                                          roadedgeptr->vehicle_points[i + 3]};
      // 一次拿四个点
      int gap_mid = euclidean_distance(roadedgeptr->vehicle_points[i + 1],
                                       roadedgeptr->vehicle_points[i + 2]);
      // 计算中间俩点的距离，四舍五入
      CommonUtil::CatmullRom(points, &fit_points, gap_mid);
    }
    int gap_back = euclidean_distance(
        roadedgeptr->vehicle_points[roadedgeptr->vehicle_points.size() - 2],
        roadedgeptr->vehicle_points[roadedgeptr->vehicle_points.size() - 1]);
    if (gap_back == 0) {
      fit_points.emplace_back(
          roadedgeptr->vehicle_points[roadedgeptr->vehicle_points.size() - 2]);
      fit_points.emplace_back(
          roadedgeptr->vehicle_points[roadedgeptr->vehicle_points.size() - 1]);
    }
    for (int i = 0; i < gap_back; i++) {
      fit_points.emplace_back(
          roadedgeptr->vehicle_points[roadedgeptr->vehicle_points.size() - 2] *
              (gap_back - i) / gap_back +
          roadedgeptr->vehicle_points[roadedgeptr->vehicle_points.size() - 1] *
              i / gap_back);
    }

    roadedgeptr->vehicle_points = fit_points;
  }
}

std::vector<RoadEdgeTargetConstPtr>
RoadEdgeMappingPipeline::GetAllConstTarget() {
  roadedge_targets_.clear();
  for (size_t i = 0; i < roadedge_trackers_.size(); ++i) {
    RoadEdgeTargetConstPtr lane_target =
        roadedge_trackers_[i]->GetConstTarget();
    roadedge_targets_.push_back(lane_target);
  }
  return roadedge_targets_;
}

std::vector<RoadEdgeTargetPtr> RoadEdgeMappingPipeline::GetAllTarget() {
  std::vector<RoadEdgeTargetPtr> roadedge_targets;
  roadedge_targets.clear();
  for (size_t i = 0; i < roadedge_trackers_.size(); ++i) {
    RoadEdgeTargetPtr roadedge_target = roadedge_trackers_[i]->GetTarget();
    roadedge_targets.push_back(roadedge_target);
  }
  return roadedge_targets;
}

bool RoadEdgeMappingPipeline::CheckBadTrack(
    const RoadEdgeTrackerPtr& roadedge_track) {
  auto roadedge_data =
      roadedge_track->GetConstTarget()->GetConstTrackedObject();
  for (auto& point : roadedge_data->vehicle_points) {
    if (std::isnan(point.x()) || std::isnan(point.y()) ||
        std::isnan(point.z())) {
      HLOG_ERROR << "track_id:" << roadedge_data->id
                 << ", nan data in roadedge tracker...";
      return true;
    }
  }
  return false;
}

bool RoadEdgeMappingPipeline::roadedge_nanvalue_trigger() {
  for (const auto& tracker : roadedge_trackers_) {
    const auto& object_data =
        tracker->GetConstTarget()->GetConstTrackedObject();
    for (const auto& point : object_data->vehicle_points) {
      if (std::isnan(point.x()) || std::isnan(point.y()) ||
          std::isnan(point.z())) {
        return true;
      }
    }
  }
  return false;
}

void RoadEdgeMappingPipeline::CheckTriggerRoadedgeNan() {
  static bool enable_15 = true;
  static double last_time_15 = -1;
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
      tc = std::chrono::time_point_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now());
  double curr_time =
      static_cast<double>(tc.time_since_epoch().count()) * 1.0e-9;
  if (enable_15) {
    // mapping trigger 路沿线nan值
    HLOG_WARN << "Roadedge Nan, Start to trigger dc 1015";
    GLOBAL_DC_TRIGGER.TriggerCollect(1015);
    enable_15 = false;
    last_time_15 = curr_time;
  }
  enable_15 = (curr_time - last_time_15) > 600;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
