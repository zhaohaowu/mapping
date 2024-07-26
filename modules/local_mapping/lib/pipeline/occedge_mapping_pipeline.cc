// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: simple_lane_tracker_pipeline.cc
// @brief: associate history lane_track to current detected lane　object

#include "modules/local_mapping/lib/pipeline/occedge_mapping_pipeline.h"
#include <math.h>

#include <unordered_set>

#include "modules/local_mapping/lib/datalogger/pose_manager.h"
#include "modules/local_mapping/utils/lane_utils.h"
#include "modules/util/include/util/orin_trigger_manager.h"
#include "perception-lib/lib/config_manager/config_manager.h"
#include "perception-lib/lib/io/file_util.h"
#include "perception-lib/lib/io/protobuf_util.h"
#include "perception-lib/lib/location_manager/location_manager.h"

namespace hozon {
namespace mp {
namespace lm {

bool OccEdgeMappingPipeline::Init() {
  occedge_matcher_ = std::make_unique<OccEdgeMatcher>();
  MatcherInitOptions matcher_init_options;
  CHECK(occedge_matcher_->Init(matcher_init_options));

  occedge_gate_keeper_ = std::make_unique<OccEdgeGatekeeper>();
  occedge_gate_keeper_->Init();

  occedge_targets_.reserve(100);
  occedge_trackers_.clear();

  return true;
}

bool OccEdgeMappingPipeline::Process(
    const ProcessOption& option, MeasurementFrameConstPtr measurement_frame_ptr,
    LocalMapFramePtr localmap_frame_ptr) {
  // 需要修改
  HLOG_DEBUG << "OccEdgeMappingPipeline time_stamp: "
             << measurement_frame_ptr->header.timestamp;
  auto& tracked_occedges = localmap_frame_ptr->occ_edges_ptr;
  tracked_occedges->occ_edges.clear();
  // PERF_BLOCK_START();
  // // 1. track更新
  // UpdateTracks();

  // 2. 观测线过滤模块
  std::vector<OccEdgePtr>& measurements_occedges =
      measurement_frame_ptr->occ_edges_ptr->occ_edges;

  // 点的过滤和清除
  std::vector<OccEdgePtr> filter_measurements =
      CleanMeasureData(&measurements_occedges);
  if (filter_measurements.empty()) {
    return true;
  }
  // 3. 插值点
  InterpolatePoint(filter_measurements);
  // 4. 拟合曲线
  std::vector<OccEdgePtr> detect_measurements =
      CurveFitMeasureData(&filter_measurements);
  // // 3. 观测线和跟踪线关联匹配
  // HLOG_DEBUG << "start do occedge associate...";
  // AssociationResult point_association_result;
  // MatcherOptions matcher_options;
  // matcher_options.timestamp = measurement_frame_ptr->header.timestamp;
  // occedge_matcher_->Associate(matcher_options, detect_measurements,
  //                             occedge_trackers_, &point_association_result);
  // HLOG_DEBUG << "end do occedge associate...";
  // // 4. update matched lane_tracks
  // UpdateAssignedTracks(option, detect_measurements,
  // point_association_result);
  // // 5. update unmatched lane_tracks

  // UpdateUnassignedTracks(option, detect_measurements,
  // point_association_result);

  // 6. created new lane_tracks for unmatched detected_lanes
  CreateNewTracks(option, detect_measurements);
  // // MergeTracks(&occedge_trackers_);

  LimitTracksNum();
  // 7. 输出准出结果
  CollectOutputObjects(tracked_occedges);
  // 8. catmallrom拟合路边沿
  // CatmullRomFit(tracked_occedges);
  // InterpolatePoint(tracked_occedges->occ_edges);
  // PERF_BLOCK_END("occedge_mapping_pipeline_process_end");
  return true;
}

std::string OccEdgeMappingPipeline::Name() const {
  return "OccEdgeMappingPipeline";
}

void OccEdgeMappingPipeline::UpdateTracks() {
  std::vector<OccEdgeTargetPtr> targets = GetAllTarget();

  const Eigen::Affine3d T_cur_last_ = POSE_MANAGER->GetDeltaPose();

  for (auto& target : targets) {
    auto& vehicle_points = target->GetTrackedObject()->vehicle_points;
    auto& vehicle_curve = target->GetTrackedObject()->vehicle_curve;
    // 点转换投影
    for (auto& point : vehicle_points) {
      point = T_cur_last_ * point;
    }
    // 系数投影
    LaneLineCurve transform_curve;
    bool status = TransformLaneLineCurveInNovatelPolyfit(
        vehicle_curve, T_cur_last_, &transform_curve);
    if (status) {
      vehicle_curve = transform_curve;
    } else {
      HLOG_ERROR << "Tracker Id: " << target->Id()
                 << ", transform curve error !!!";
    }
    if (vehicle_points.front().x() > vehicle_points.back().x()) {
      std::reverse(vehicle_points.begin(), vehicle_points.end());
    }
  }
}

// tracker 合并策略
void OccEdgeMappingPipeline::MergeTracks(
    std::vector<OccEdgeTrackerPtr>* trackers) {
  if (trackers->size() < 2) {
    return;
  }
  std::unordered_set<int> remove_index;
  for (int i = 0; i < static_cast<int>(trackers->size() - 1); ++i) {
    const auto& left_line = trackers->at(i)->GetConstTarget();
    if (!left_line->IsTracked()) {
      continue;
    }
    for (int j = i + 1; j < static_cast<int>(trackers->size()); ++j) {
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
      HLOG_DEBUG << "occedge MergeTracks: id " << left_line->Id() << ", id "
                 << right_line->Id() << ", avg_dist: " << avg_dist
                 << ", overlay_ratio: " << over_lay_ratio
                 << ", overlay_y_ratio: " << over_lay_y_ratio
                 << ", time_diff: " << time_diff;
      // 两条Tracker重合度很高
      // 根据线的质量来做删除,需要根据case专门抽一个评估函数
      if ((over_lay_ratio > 0.7 || over_lay_y_ratio > 0.7) && avg_dist < 1.5) {
        if (over_lay_y_ratio > 0.7) {
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

double GetDistBetweenTwoEdge(const std::vector<Eigen::Vector3d>& point_set1,
                             const std::vector<Eigen::Vector3d>& point_set2) {
  CurveFitter curve_fitter(1);
  curve_fitter.RandomPointsPolyFitProcess(point_set1);
  double a_x = 0.0;
  double a_y = curve_fitter.evalueValue(a_x);
  double b_x = 10.0;
  double b_y = curve_fitter.evalueValue(b_x);
  Eigen::Vector3d A(a_x, a_y, 0.0);
  Eigen::Vector3d B(b_x, b_y, 0.0);
  double dist_sum = 0.0;
  for (const auto& pt : point_set2) {
    auto dist = GetDistPointLane(pt, A, B);
    dist_sum += dist;
  }
  return dist_sum / point_set2.size();
}

bool CompareOccLine(const std::vector<Eigen::Vector3d>& left_pts,
                    const std::vector<Eigen::Vector3d>& right_pts,
                    const double& dist_threshold) {
  auto left_max_x = std::max_element(
      left_pts.begin(), left_pts.end(),
      [](const auto& left, const auto& right) { return left.x() < right.x(); });
  const Eigen::Vector3d& left_max_pt = *left_max_x;

  auto right_min_x = std::min_element(
      right_pts.begin(), right_pts.end(),
      [](const auto& left, const auto& right) { return left.x() < right.x(); });
  const Eigen::Vector3d& right_min_pt = *right_min_x;
  if (left_max_pt.x() < right_min_pt.x()) {
    if ((left_max_pt - right_min_pt).norm() < dist_threshold) {
      return true;
    }
  }
  return false;
}

std::vector<OccEdgePtr> OccEdgeMappingPipeline::CleanMeasureData(
    const std::vector<OccEdgePtr>* measurement_datas) {
  std::unordered_map<int, std::vector<OccEdgePtr>> occedge_map;
  std::vector<OccEdgePtr> measurement_data;
  CurveFitter curve_fitter(3);
  const double length_threshold = 10.0;
  for (const auto& measure_edge : *measurement_datas) {
    auto measure_data = std::make_shared<OccEdge>(*measure_edge);
    if (measure_data->vehicle_points.size() < 4) {
      continue;
    }
    // 长度小于10m
    if (GetLength(measure_data->vehicle_points) < length_threshold) {
      continue;
    }
    // 类型过滤
    if (measure_data->type == OccEdgeType::CONE_EDGE) {
      continue;
    }
    measurement_data.push_back(measure_data);
  }
  std::vector<OccEdgePtr> output_measurement_data;
  const double dist_threshold = 4.0;
  // 距离近的两根路沿合并
  for (int i = 0; i < static_cast<int>(measurement_data.size()); ++i) {
    if (measurement_data.at(i)->low_quality) {
      continue;
    }
    auto& left_pts = measurement_data.at(i)->vehicle_points;
    for (int j = i + 1; j < static_cast<int>(measurement_data.size()); ++j) {
      auto& right_pts = measurement_data.at(j)->vehicle_points;
      auto distance = GetDistBetweenTwoEdge(left_pts, right_pts);
      if (distance < 1.5) {
        if (CompareOccLine(left_pts, right_pts, dist_threshold)) {
          left_pts.insert(left_pts.end(), right_pts.begin(), right_pts.end());
          measurement_data.at(j)->low_quality = true;
          continue;
        }
        if (CompareOccLine(right_pts, left_pts, dist_threshold)) {
          right_pts.insert(right_pts.end(), left_pts.begin(), left_pts.end());
          measurement_data.at(i)->low_quality = true;
          continue;
        }
      }
    }
    if (!measurement_data.at(i)->low_quality) {
      output_measurement_data.push_back(measurement_data.at(i));
    }
  }
  return output_measurement_data;
}

std::vector<OccEdgePtr> OccEdgeMappingPipeline::CurveFitMeasureData(
    const std::vector<OccEdgePtr>* measurement_datas) {
  std::unordered_map<int, std::vector<OccEdgePtr>> occedge_map;
  std::vector<OccEdgePtr> output_measurement_data;
  CurveFitter curve_fitter(3);
  for (const auto& measure_data : *measurement_datas) {
    if (!curve_fitter.RandomPointsPolyFitProcess(
            measure_data->vehicle_points)) {
      continue;
    }
    measure_data->vehicle_curve.min = curve_fitter.x_min;
    measure_data->vehicle_curve.max = curve_fitter.x_max;
    measure_data->vehicle_curve.coeffs = curve_fitter.params;
    HLOG_DEBUG << "CleanMeasureData: " << curve_fitter.x_min << ", "
               << curve_fitter.x_max << ", " << curve_fitter.params[0];
    std::sort(measure_data->vehicle_points.begin(),
              measure_data->vehicle_points.end(),
              [=](const auto& left, const auto& right) {
                return left.x() < right.x();
              });
    output_measurement_data.push_back(measure_data);
  }
  return output_measurement_data;
}

void OccEdgeMappingPipeline::UpdateAssignedTracks(
    const ProcessOption& tracker_option,
    const std::vector<OccEdgePtr>& detect_measurements,
    const AssociationResult& association_result) {
  const auto& assignments = association_result.assignments;
  size_t track_index = 0;
  size_t detect_index = 0;
  // todo
  for (const auto& assignment : assignments) {
    track_index = std::get<0>(assignment);
    detect_index = std::get<1>(assignment);
    HLOG_DEBUG << "assignments trackId: "
               << occedge_trackers_[track_index]->GetConstTarget()->Id()
               << ", detectId: " << detect_measurements[detect_index]->id;
    occedge_trackers_[track_index]->UpdateWithDetectedObject(
        tracker_option, detect_measurements[detect_index]);
  }
}

void OccEdgeMappingPipeline::UpdateUnassignedTracks(
    const ProcessOption& tracker_option,
    const std::vector<OccEdgePtr>& detect_measurements,
    const AssociationResult& association_result) {
  const auto& unassigned_track_indexs = association_result.unassigned_tracks;
  size_t track_index = 0;
  // todo
  for (const auto& index : unassigned_track_indexs) {
    occedge_trackers_[index]->UpdateWithoutDetectedObject(tracker_option);
  }
  RemoveLostTracks();
}

void OccEdgeMappingPipeline::CreateNewTracks(
    const ProcessOption& tracker_option,
    const std::vector<OccEdgePtr>& detect_measurements,
    const AssociationResult& association_result) {
  const auto& unsigned_objects = association_result.unsigned_objects;
  size_t detect_index = 0;
  for (const auto& detect_index : unsigned_objects) {
    // Init OccEdgeTarget
    OccEdgeTargetPtr occedge_target_ptr = std::make_shared<OccEdgeTarget>();
    occedge_target_ptr->Init(tracker_option, detect_measurements[detect_index]);

    OccEdgeTrackerPtr occedge_tracker = std::make_shared<OccEdgeTracker>();
    // tracker_init_option_.novatel2world_pose = novatel2world_pose_;
    if (occedge_tracker->Init(tracker_option, occedge_target_ptr)) {
      occedge_trackers_.emplace_back(occedge_tracker);
    }
  }
}

void OccEdgeMappingPipeline::CreateNewTracks(
    const ProcessOption& tracker_option,
    const std::vector<OccEdgePtr>& detect_measurements) {
  for (const auto& detect_measurement : detect_measurements) {
    // Init OccEdgeTarget
    OccEdgeTargetPtr occedge_target_ptr = std::make_shared<OccEdgeTarget>();
    occedge_target_ptr->Init(tracker_option, detect_measurement);

    OccEdgeTrackerPtr occedge_tracker = std::make_shared<OccEdgeTracker>();
    // tracker_init_option_.novatel2world_pose = novatel2world_pose_;
    if (occedge_tracker->Init(tracker_option, occedge_target_ptr)) {
      occedge_trackers_.emplace_back(occedge_tracker);
    }
  }
}

void OccEdgeMappingPipeline::RemoveLostTracks() {
  size_t track_count = 0;
  for (size_t i = 0; i < occedge_trackers_.size(); ++i) {
    const auto& track_id = occedge_trackers_[i]->GetConstTarget()->Id();
    if (occedge_trackers_[i]->GetConstTarget()->IsDie()) {
      continue;
    }
    if (occedge_trackers_[i]
            ->GetConstTarget()
            ->GetConstTrackedObject()
            ->vehicle_points.back()
            .x() < -80.0) {
      continue;
    }
    if (occedge_trackers_[i]
            ->GetConstTarget()
            ->GetConstTrackedObject()
            ->vehicle_points.size() < 5) {
      continue;
    }

    if (CheckBadTrack(occedge_trackers_[i])) {
      continue;
    }

    if (track_count == i) {
      track_count++;
      continue;
    }
    occedge_trackers_[track_count++] = occedge_trackers_[i];
  }
  occedge_trackers_.resize(track_count);
}

bool OccEdgeMappingPipeline::CompareTrackTime(const OccEdgeTrackerPtr& d1,
                                              const OccEdgeTrackerPtr& d2) {
  return d1->GetConstTarget()->GetLastestTrackedTimestamp() >
         d2->GetConstTarget()->GetLastestTrackedTimestamp();
}

void OccEdgeMappingPipeline::LimitTracksNum() {
  occedge_trackers_.erase(
      std::remove_if(
          occedge_trackers_.begin(), occedge_trackers_.end(),
          [&](const auto& tracker) { return CheckBadTrack(tracker); }),
      occedge_trackers_.end());
  std::sort(occedge_trackers_.begin(), occedge_trackers_.end(),
            CompareTrackTime);
  if (occedge_trackers_.size() > limit_max_tracker_nums_) {
    occedge_trackers_.resize(limit_max_tracker_nums_);
  }
}

void OccEdgeMappingPipeline::CollectOutputObjects(
    OccEdgesPtr tracked_occedges) {
  for (const auto& occedge_tracker : occedge_trackers_) {
    HLOG_DEBUG << "########CollectOutputObjects############";
    OccEdgeTargetConstPtr occedge_target = occedge_tracker->GetConstTarget();
    HLOG_DEBUG << "Buffer OccEdgeTarget " << occedge_target->ToStr();
    OccEdgePtr output_occedge_object =
        std::make_shared<OccEdge>(*occedge_target->GetConstTrackedObject());
    output_occedge_object->detect_id = occedge_target->occ_detect_id;
    if (!occedge_gate_keeper_->AbleToOutput(occedge_tracker->GetConstTarget(),
                                            GetAllConstTarget())) {
      HLOG_ERROR << "OccedgeTarget TrackStatus trackId: "
                 << occedge_target->Id() << ", " << occedge_target->ToStr()
                 << ", OccedgeTrack NOT OUTPUT!";
      continue;
    }
    tracked_occedges->occ_edges.push_back(output_occedge_object);
  }
  // 透传下游，清空trackers
  occedge_trackers_.clear();
}

void OccEdgeMappingPipeline::CatmullRomFit(OccEdgesPtr tracked_occedges) {
  auto euclidean_distance = [](const Eigen::Vector3d& p0,
                               const Eigen::Vector3d& p1) {
    return static_cast<int>(round(std::sqrt(std::pow(p0.x() - p1.x(), 2) +
                                            std::pow(p0.y() - p1.y(), 2))));
  };
  for (auto& occedgeptr : tracked_occedges->occ_edges) {
    if (occedgeptr->vehicle_points.size() < 4) {
      continue;
    }
    std::vector<Eigen::Vector3d> fit_points;
    int gap_front = euclidean_distance(occedgeptr->vehicle_points[0],
                                       occedgeptr->vehicle_points[1]);
    if (gap_front == 0) {
      fit_points.emplace_back(occedgeptr->vehicle_points[0]);
    }
    for (int i = 0; i < gap_front; i++) {
      fit_points.emplace_back(occedgeptr->vehicle_points[0] * (gap_front - i) /
                                  gap_front +
                              occedgeptr->vehicle_points[1] * i / gap_front);
    }  // 首尾点线性插值
    for (int i = 0; i < static_cast<int>(occedgeptr->vehicle_points.size()) - 3;
         i++) {
      std::vector<Eigen::Vector3d> points{
          occedgeptr->vehicle_points[i], occedgeptr->vehicle_points[i + 1],
          occedgeptr->vehicle_points[i + 2], occedgeptr->vehicle_points[i + 3]};
      // 一次拿四个点
      int gap_mid = euclidean_distance(occedgeptr->vehicle_points[i + 1],
                                       occedgeptr->vehicle_points[i + 2]);
      // 计算中间俩点的距离，四舍五入
      CommonUtil::CatmullRom(points, &fit_points, gap_mid);
    }
    int gap_back = euclidean_distance(
        occedgeptr->vehicle_points[occedgeptr->vehicle_points.size() - 2],
        occedgeptr->vehicle_points[occedgeptr->vehicle_points.size() - 1]);
    if (gap_back == 0) {
      fit_points.emplace_back(
          occedgeptr->vehicle_points[occedgeptr->vehicle_points.size() - 2]);
      fit_points.emplace_back(
          occedgeptr->vehicle_points[occedgeptr->vehicle_points.size() - 1]);
    }
    for (int i = 0; i < gap_back; i++) {
      fit_points.emplace_back(
          occedgeptr->vehicle_points[occedgeptr->vehicle_points.size() - 2] *
              (gap_back - i) / gap_back +
          occedgeptr->vehicle_points[occedgeptr->vehicle_points.size() - 1] *
              i / gap_back);
    }

    occedgeptr->vehicle_points = fit_points;
  }
}

void OccEdgeMappingPipeline::InterpolatePoint(
    std::vector<OccEdgePtr> detect_measurements) {
  for (auto& occedge_ptr : detect_measurements) {
    const auto& pts = occedge_ptr->vehicle_points;
    // 找拐点
    bool split_flag = false;
    auto fix_iter = std::min_element(pts.begin(), pts.end(),
                                     [](const auto& left, const auto& right) {
                                       return left.norm() < right.norm();
                                     });
    int fix_pos = static_cast<int>(std::distance(pts.begin(), fix_iter));
    double left_y_val = std::accumulate(
        pts.begin(), fix_iter, 0.0, [](double left, const auto& right) {
          return std::abs(left) + std::abs(right.y());
        });
    left_y_val /= (fix_pos + 1);
    auto left_y_max = std::max_element(
        pts.begin(), fix_iter, [](const auto& left, const auto& right) {
          return std::abs(left.y()) < std::abs(right.y());
        });
    double right_y_val = std::accumulate(
        fix_iter, pts.end(), 0.0, [](double left, const auto& right) {
          return std::abs(left) + std::abs(right.y());
        });
    right_y_val /= (std::distance(fix_iter, pts.end()) + 1);
    auto right_y_max = std::max_element(
        fix_iter, pts.end(), [](const auto& left, const auto& right) {
          return std::abs(left.y()) < std::abs(right.y());
        });
    int range_begin = 0;
    int range_end = static_cast<int>(pts.size()) - 1;
    // 求平均y决定选哪边插值
    split_flag = (fix_iter != pts.begin() && fix_iter != pts.end()) &&
                 (std::abs(right_y_max->y() - left_y_max->y()) > 2.5);
    if (split_flag) {
      if (left_y_val < right_y_val) {
        range_begin = 0;
        range_end = fix_pos;
      } else {
        range_begin = fix_pos;
        range_end = static_cast<int>(pts.size()) - 1;
      }
    }
    HLOG_DEBUG << "occedge_id: " << occedge_ptr->detect_id
               << ", split_flag: " << split_flag << ", fix_pos: " << fix_pos
               << ", fix_point x: " << fix_iter->x() << ", y: " << fix_iter->y()
               << ", left_y_val: " << left_y_val
               << ", left_y_max: " << left_y_max->y()
               << ", right_y_val: " << right_y_val
               << ", right_y_max: " << right_y_max->y()
               << ", range_begin: " << range_begin
               << ", range_end: " << range_end;
    const double interpolate_step = 1.0;
    std::vector<Eigen::Vector3d> new_pts;
    for (int i = 0; i < static_cast<int>(pts.size()) - 1; ++i) {
      new_pts.emplace_back(pts[i]);
      // 包络上两个点x方向距离小于1m则跳过
      double length = std::abs(pts[i + 1].x() - pts[i].x());
      if (length <= interpolate_step) {
        continue;
      }
      // 插值范围内才插值
      if (i >= range_begin && i < range_end) {
        // 1m一个点开始插值
        int interpolate_pts = static_cast<int>(length / interpolate_step);
        for (int j = 1; j < interpolate_pts; ++j) {
          double weight = (j * interpolate_step) / length;
          new_pts.emplace_back((1 - weight) * pts[i] + weight * pts[i + 1]);
        }
      }
    }
    new_pts.emplace_back(pts[pts.size() - 1]);
    occedge_ptr->vehicle_points = new_pts;
  }
}

std::vector<OccEdgeTargetConstPtr> OccEdgeMappingPipeline::GetAllConstTarget() {
  occedge_targets_.clear();
  for (size_t i = 0; i < occedge_trackers_.size(); ++i) {
    OccEdgeTargetConstPtr occedge_target =
        occedge_trackers_[i]->GetConstTarget();
    occedge_targets_.push_back(occedge_target);
  }
  return occedge_targets_;
}

std::vector<OccEdgeTargetPtr> OccEdgeMappingPipeline::GetAllTarget() {
  std::vector<OccEdgeTargetPtr> occedge_targets;
  occedge_targets.clear();
  for (size_t i = 0; i < occedge_trackers_.size(); ++i) {
    OccEdgeTargetPtr occedge_target = occedge_trackers_[i]->GetTarget();
    occedge_targets.push_back(occedge_target);
  }
  return occedge_targets;
}

bool OccEdgeMappingPipeline::CheckBadTrack(
    const OccEdgeTrackerPtr& occedge_track) {
  auto occedge_data = occedge_track->GetConstTarget()->GetConstTrackedObject();
  for (auto& point : occedge_data->vehicle_points) {
    if (std::isnan(point.x()) || std::isnan(point.y()) ||
        std::isnan(point.z())) {
      HLOG_ERROR << "track_id:" << occedge_data->id
                 << ", nan data in occedge tracker...";
      return true;
    }
  }
  return false;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
