// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: simple_lane_tracker_pipeline.cc
// @brief: associate history lane_track to current detected lane　object

#include "modules/local_mapping/lib/pipeline/laneline_mapping_pipeline.h"

#include <cmath>
#include <limits>
#include <memory>
#include <unordered_set>
#include <utility>

#include "depend/common/util/perf_util.h"
#include "lib/interface/base_init_options.h"
#include "modules/local_mapping/lib/association/base_struct.h"
#include "modules/local_mapping/lib/datalogger/map_manager.h"
#include "modules/local_mapping/lib/datalogger/pose_manager.h"
#include "modules/local_mapping/utils/common.h"
#include "modules/local_mapping/utils/lane_utils.h"
#include "modules/util/include/util/orin_trigger_manager.h"
#include "perception-lib/lib/config_manager/config_manager.h"
#include "perception-lib/lib/io/file_util.h"
#include "perception-lib/lib/io/protobuf_util.h"
#include "perception-lib/lib/location_manager/location_manager.h"

namespace hozon {
namespace mp {
namespace lm {
bool LaneLineMappingPipeline::Init() {
  laneline_matcher_ = std::make_unique<LaneLineMatcher>();
  MatcherInitOptions matcher_init_options;
  CHECK(laneline_matcher_->Init(matcher_init_options));

  lane_gate_keeper_ = std::make_unique<LaneGatekeeper>();
  lane_gate_keeper_->Init();

  lane_position_manager_ = std::make_unique<LanePositionManager>();
  lane_position_manager_->Init();
  map_lane_position_manager_ = std::make_unique<MappingPositionManager>();
  CHECK(map_lane_position_manager_->Init());
  mapping_remove_manager_ = std::make_unique<MappingRemoveManager>();
  CHECK(mapping_remove_manager_->Init());
  laneline_merge_tracker_ = std::make_unique<LaneLineMergeTrack>();
  lane_targets_.reserve(100);
  lane_trackers_.clear();

  lane_meas_filter_ = std::make_unique<LaneMeasurementFilter>();
  lane_meas_filter_->Init();

  return true;
}

void LaneLineMappingPipeline::AssginPosition(
    LaneLinesPtr localmap_laneline_ptr) {
  if (localmap_laneline_ptr->lanelines.empty()) {
    return;
  }
  // set lane pose atrribute
  map_lane_position_manager_->Process(localmap_laneline_ptr);
  lane_position_manager_->Process(localmap_laneline_ptr);
}
void LaneLineMappingPipeline::DeleteOutlierLaneLines(
    std::vector<LaneTrackerPtr>* trackers) {
  if (trackers->size() < 2) {
    return;
  }
  // 删除异常车道线
  mapping_remove_manager_->Process(trackers);
}
bool LaneLineMappingPipeline::Process(
    const ProcessOption& option, MeasurementFrameConstPtr measurement_frame_ptr,
    LocalMapFramePtr localmap_frame_ptr) {
  HLOG_DEBUG << " cam timestamp:"
             << std::to_string(measurement_frame_ptr->header.timestamp);
  // PERF_BLOCK_START();
  auto& tracked_lanes = localmap_frame_ptr->lane_lines_ptr;
  tracked_lanes->lanelines.clear();
  // 1. track更新
  UpdateTracks();

  // 2. 观测线过滤模块
  const std::vector<LaneLinePtr>& orin_measurements =
      measurement_frame_ptr->lane_lines_ptr->lanelines;
  std::vector<LaneLinePtr> detect_measurements;
  detect_measurements.clear();
  HLOG_DEBUG << "before laneline anomy filter:" << orin_measurements.size();
  lane_meas_filter_->Process(orin_measurements, &detect_measurements);
  HLOG_DEBUG << "after laneline anomy filter:" << detect_measurements.size();

  // 3. 观测车道线转换到local系
  HLOG_DEBUG << "start do sync laneline measurement vehicle points to world "
                "points...";

  TransMeasurementVehicle2Local(&detect_measurements);
  HLOG_DEBUG << "finish do sync laneline measurement vehicle points to world "
                "points...";

  for (auto& laneline : detect_measurements) {
    HLOG_DEBUG << "cvt laneline vpoint num :"
               << laneline->vehicle_points.size();
    HLOG_DEBUG << "cvt laneline wpoint num :" << laneline->world_points.size();
  }

  // 4. 观测线和跟踪线关联匹配
  HLOG_DEBUG << "start do Associate...";
  AssociationResult point_association_result;
  MatcherOptions matcher_options;
  matcher_options.timestamp = option.timestamp;
  laneline_matcher_->Associate(matcher_options, detect_measurements,
                               lane_trackers_, &point_association_result);
  HLOG_DEBUG << "point_match_debug detection nums: "
             << detect_measurements.size()
             << "tracker nums: " << lane_trackers_.size();
  for (auto& assign : point_association_result.assignments) {
    auto target_idx = std::get<0>(assign);
    auto detect_idx = std::get<1>(assign);
    HLOG_DEBUG << "point_match_debug laneline_aso assignments target_idx: "
               << target_idx << " detect_idx:" << detect_idx;
  }

  for (auto& unassign_track : point_association_result.unassigned_tracks) {
    HLOG_DEBUG << "point_match_debug laneline_aso unassign_track idx : "
               << unassign_track;
  }

  for (auto& unassign_object : point_association_result.unsigned_objects) {
    HLOG_DEBUG << "point_match_debug laneline_aso unassign_object idx: "
               << unassign_object;
  }

  HLOG_DEBUG << "laneline_aso end do Associate...";
  // PERF_BLOCK_END("laneline_LaneAssociate");

  // 5. update matched lane_tracks

  UpdateAssignedTracks(option, detect_measurements, point_association_result);
  // PERF_BLOCK_END("laneline_UpdateAssignedTracks");
  // 6. update unmatched lane_tracks
  UpdateUnassignedTracks(option, detect_measurements, point_association_result);
  // PERF_BLOCK_END("laneline_UpdateUnassignedTracks");
  // 7. created new lane_tracks for unmatched detected_lanes
  CreateNewTracks(option, detect_measurements, point_association_result);
  // PERF_BLOCK_END("laneline_CreateNewTracks");
  // 8. 脑补情况删除
  lane_meas_filter_->SetLostTrackerTruncation(&detect_measurements,
                                              &lane_trackers_);
  HLOG_DEBUG << "lane tracker size:" << lane_trackers_.size();
  laneline_merge_tracker_->MergeTracks(&lane_trackers_);
  HLOG_DEBUG << "after mergeTracks lane tracker size:" << lane_trackers_.size();
  // 9 过滤异常车道线
  DeleteOutlierLaneLines(&lane_trackers_);
  // 端点优化
  SmoothEndPt();
  LimitTracksNum();
  // todo 曲线平滑，检查每个点的导数，突变点，用周围的点拟合点填充该点
  CollectOutputObjects(tracked_lanes);
  HLOG_DEBUG << "output track laneline size ============"
             << lane_trackers_.size();
  // PERF_BLOCK_END("laneline_CollectOutputObjects");

  // 10.车道线设置POS
  AssginPosition(tracked_lanes);
  // for (const auto& lane : tracked_lanes->lanelines) {
  //   HLOG_INFO << "cam2 lane id:" << lane->id
  //             << " lane pos:" << static_cast<int>(lane->position);
  // }
  // PERF_BLOCK_END("laneline_PostProcess");

  // 11. 输出点插值到1米一个
  CatmullRomFit(tracked_lanes);

  // 11. 删除相交的线中某一条的一部分
  AdjustIntersectionLines(tracked_lanes);

// 当tracker中存在nan值时进行触发
#ifdef ISORIN
  if (laneline_nanvalue_trigger()) {
    CheckTriggerLanelineNan();
  }
#endif

  return true;
}

std::string LaneLineMappingPipeline::Name() const {
  return "LaneLineMappingPipeline";
}

void LaneLineMappingPipeline::UpdateTracks() {
  std::vector<LaneTargetPtr> targets = GetAllTarget();

  const Eigen::Affine3d T_cur_last_ = POSE_MANAGER->GetDeltaPose();

  for (auto& target : targets) {
    auto& vehicle_points = target->GetTrackedObject()->vehicle_points;
    for (auto& point : vehicle_points) {
      point = T_cur_last_ * point;
    }
    if (vehicle_points.front().x() > vehicle_points.back().x()) {
      std::reverse(vehicle_points.begin(), vehicle_points.end());
    }
  }
}

void LaneLineMappingPipeline::UpdateAssignedTracks(
    const ProcessOption& tracker_option,
    const std::vector<LaneLinePtr>& detect_measurements,
    const AssociationResult& association_result) {
  const auto& assignments = association_result.assignments;
  size_t track_index = 0;
  size_t detect_index = 0;

  // todo
  for (const auto& assignment : assignments) {
    track_index = std::get<0>(assignment);
    detect_index = std::get<1>(assignment);
    lane_trackers_[track_index]->UpdateWithDetectedObject(
        tracker_option, detect_measurements[detect_index]);
  }
}

void LaneLineMappingPipeline::UpdateUnassignedTracks(
    const ProcessOption& tracker_option,
    const std::vector<LaneLinePtr>& detect_measurements,
    const AssociationResult& association_result) {
  const auto& unassigned_track_indexs = association_result.unassigned_tracks;
  // size_t track_index = 0;
  // todo
  for (const auto& index : unassigned_track_indexs) {
    lane_trackers_[index]->UpdateWithoutDetectedObject(tracker_option);
  }
  RemoveLostTracks();
}

void LaneLineMappingPipeline::CreateNewTracks(
    const ProcessOption& init_option,
    const std::vector<LaneLinePtr>& detect_measurements,
    const AssociationResult& association_result) {
  const auto& unsigned_objects = association_result.unsigned_objects;
  size_t detect_index = 0;
  for (const auto& detect_index : unsigned_objects) {
    // Init LaneTarget
    LaneTargetPtr lane_target_ptr = std::make_shared<LaneTarget>();
    lane_target_ptr->Init(init_option, detect_measurements[detect_index]);

    LaneTrackerPtr lane_tracker = std::make_shared<LaneTracker>();
    if (lane_tracker->Init(init_option, lane_target_ptr)) {
      lane_trackers_.emplace_back(lane_tracker);
    }
  }
}

void LaneLineMappingPipeline::RemoveLostTracks() {
  size_t track_count = 0;
  for (size_t i = 0; i < lane_trackers_.size(); ++i) {
    const auto& track_id = lane_trackers_[i]->GetConstTarget()->Id();
    if (lane_trackers_[i]->GetConstTarget()->IsDie()) {
      continue;
    }
    if (lane_trackers_[i]
            ->GetConstTarget()
            ->GetConstTrackedObject()
            ->vehicle_points.back()
            .x() < -80.0) {
      continue;
    }
    if (lane_trackers_[i]
            ->GetConstTarget()
            ->GetConstTrackedObject()
            ->vehicle_points.size() < 5) {
      continue;
    }

    if (CheckBadTrack(lane_trackers_[i])) {
      continue;
    }

    // 判断前面是否存在track应被过滤的情况
    if (track_count == i) {
      track_count++;
      continue;
    }
    lane_trackers_[track_count++] = lane_trackers_[i];
  }
  lane_trackers_.resize(track_count);
}

bool LaneLineMappingPipeline::CompareTrackTime(const LaneTrackerPtr& d1,
                                               const LaneTrackerPtr& d2) {
  return d1->GetConstTarget()->GetLastestTrackedTimestamp() >
         d2->GetConstTarget()->GetLastestTrackedTimestamp();
}

void LaneLineMappingPipeline::LimitTracksNum() {
  lane_trackers_.erase(
      std::remove_if(
          lane_trackers_.begin(), lane_trackers_.end(),
          [&](const auto& tracker) { return CheckBadTrack(tracker); }),
      lane_trackers_.end());
  std::sort(lane_trackers_.begin(), lane_trackers_.end(), CompareTrackTime);
  if (lane_trackers_.size() > limit_max_tracker_nums_) {
    lane_trackers_.resize(limit_max_tracker_nums_);
  }
}

std::vector<LaneTargetConstPtr> LaneLineMappingPipeline::GetAllConstTarget() {
  lane_targets_.clear();
  for (size_t i = 0; i < lane_trackers_.size(); ++i) {
    LaneTargetConstPtr lane_target = lane_trackers_[i]->GetConstTarget();
    lane_targets_.push_back(lane_target);
  }
  return lane_targets_;
}

std::vector<LaneTargetPtr> LaneLineMappingPipeline::GetAllTarget() {
  std::vector<LaneTargetPtr> laneline_targets;
  laneline_targets.clear();
  for (size_t i = 0; i < lane_trackers_.size(); ++i) {
    LaneTargetPtr laneline_target = lane_trackers_[i]->GetTarget();
    laneline_targets.push_back(laneline_target);
  }
  return laneline_targets;
}

void LaneLineMappingPipeline::CatmullRomFit(LaneLinesPtr tracked_lanelines) {
  auto euclidean_distance = [](const Eigen::Vector3d& p0,
                               const Eigen::Vector3d& p1) {
    return static_cast<int>(round(std::sqrt(std::pow(p0.x() - p1.x(), 2) +
                                            std::pow(p0.y() - p1.y(), 2))));
  };
  for (auto& lanelineptr : tracked_lanelines->lanelines) {
    if (lanelineptr->vehicle_points.size() < 4) {
      continue;
    }
    std::vector<Eigen::Vector3d> fit_points;
    int gap_front = euclidean_distance(lanelineptr->vehicle_points[0],
                                       lanelineptr->vehicle_points[1]);
    if (gap_front == 0) {
      fit_points.emplace_back(lanelineptr->vehicle_points[0]);
    }
    for (int i = 0; i < gap_front; i++) {
      fit_points.emplace_back(lanelineptr->vehicle_points[0] * (gap_front - i) /
                                  gap_front +
                              lanelineptr->vehicle_points[1] * i / gap_front);
    }  // 首尾点线性插值
    for (int i = 0;
         i < static_cast<int>(lanelineptr->vehicle_points.size()) - 3; i++) {
      std::vector<Eigen::Vector3d> points{lanelineptr->vehicle_points[i],
                                          lanelineptr->vehicle_points[i + 1],
                                          lanelineptr->vehicle_points[i + 2],
                                          lanelineptr->vehicle_points[i + 3]};
      // 一次拿四个点
      int gap_mid = euclidean_distance(lanelineptr->vehicle_points[i + 1],
                                       lanelineptr->vehicle_points[i + 2]);
      // 计算中间俩点的距离，四舍五入
      CommonUtil::CatmullRom(points, &fit_points, gap_mid);
    }
    int gap_back = euclidean_distance(
        lanelineptr->vehicle_points[lanelineptr->vehicle_points.size() - 2],
        lanelineptr->vehicle_points[lanelineptr->vehicle_points.size() - 1]);
    if (gap_back == 0) {
      fit_points.emplace_back(
          lanelineptr->vehicle_points[lanelineptr->vehicle_points.size() - 2]);
      fit_points.emplace_back(
          lanelineptr->vehicle_points[lanelineptr->vehicle_points.size() - 1]);
    }
    for (int i = 0; i < gap_back; i++) {
      fit_points.emplace_back(
          lanelineptr->vehicle_points[lanelineptr->vehicle_points.size() - 2] *
              (gap_back - i) / gap_back +
          lanelineptr->vehicle_points[lanelineptr->vehicle_points.size() - 1] *
              i / gap_back);
    }

    lanelineptr->vehicle_points = fit_points;
  }
}

void LaneLineMappingPipeline::AdjustIntersectionLines(
    LaneLinesPtr tracked_lanelines) {
  int laneline_size = static_cast<int>(tracked_lanelines->lanelines.size());
  // 至少有三条车道线才会进删线逻辑
  if (laneline_size <= 2) {
    return;
  }
  for (int i = 0; i < laneline_size - 1; i++) {
    auto& laneline_i_ptr = tracked_lanelines->lanelines[i];
    if (laneline_i_ptr->vehicle_points.size() < 2) {
      continue;
    }
    for (int j = i + 1; j < laneline_size; j++) {
      auto& laneline_j_ptr = tracked_lanelines->lanelines[j];
      if (laneline_j_ptr->vehicle_points.size() < 2) {
        continue;
      }
      Eigen::Vector3d intersect_pt;  // 交点
      // 判断两线段是否相交，如果相交，计算交点
      if (LaneLineIntersection(laneline_i_ptr->vehicle_points,
                               laneline_j_ptr->vehicle_points, &intersect_pt)) {
        // 根据heading与主车道线的heading差大小判断删哪个线
        LaneLinePtr left_lane_line = std::make_shared<LaneLine>();
        LaneLinePtr right_lane_line = std::make_shared<LaneLine>();

        for (const auto& laneline_ptr : tracked_lanelines->lanelines) {
          if (laneline_ptr->position == LaneLinePosition::EGO_LEFT) {
            left_lane_line = laneline_ptr;
          } else if (laneline_ptr->position == LaneLinePosition::EGO_RIGHT) {
            right_lane_line = laneline_ptr;
          }
        }
        double left_lane_line_heading = 0;
        double right_lane_line_heading = 0;
        if (!left_lane_line->vehicle_points.empty()) {
          if (laneline_i_ptr->position != LaneLinePosition::EGO_LEFT &&
              laneline_j_ptr->position != LaneLinePosition::EGO_LEFT) {
            left_lane_line_heading =
                CommonUtil::CalMeanLineHeading(left_lane_line->vehicle_points);
          }
        }
        if (!right_lane_line->vehicle_points.empty()) {
          if (laneline_i_ptr->position != LaneLinePosition::EGO_RIGHT &&
              laneline_j_ptr->position != LaneLinePosition::EGO_RIGHT) {
            right_lane_line_heading =
                CommonUtil::CalMeanLineHeading(right_lane_line->vehicle_points);
          }
        }
        double main_laneline_heading =
            (left_lane_line_heading + right_lane_line_heading) / 2;
        double i_heading =
            CommonUtil::CalMeanLineHeading(laneline_i_ptr->vehicle_points);
        double j_heading =
            CommonUtil::CalMeanLineHeading(laneline_j_ptr->vehicle_points);
        int laneline_delete_index =
            (fabs(CommonUtil::NormalizeAngle(i_heading -
                                             main_laneline_heading))) >
                    (fabs(CommonUtil::NormalizeAngle(j_heading -
                                                     main_laneline_heading)))
                ? i
                : j;

        size_t point_delete_index = 0;
        while (tracked_lanelines->lanelines[laneline_delete_index]
                       ->vehicle_points[point_delete_index + 1]
                       .x() < intersect_pt.x() &&
               point_delete_index <
                   tracked_lanelines->lanelines[laneline_delete_index]
                           ->vehicle_points.size() -
                       2) {
          point_delete_index++;
        }
        // 删除目标车道线被交点分成的两段线中较短的那段
        DeleteLaneLineShortPart(
            point_delete_index,
            &(tracked_lanelines->lanelines[laneline_delete_index]
                  ->vehicle_points));
        HLOG_WARN << "delete some intersection laneline";
        HLOG_WARN << "intersect_pt.x()" << intersect_pt.x();
        HLOG_WARN << "point_delete_index: " << point_delete_index;
        // 继续判断这两条线是否相交，防止出现多个交点
        j--;
      }
    }
  }
}

void LaneLineMappingPipeline::DeleteLaneLineShortPart(
    size_t point_delete_index, std::vector<Eigen::Vector3d>* vehicle_points) {
  double length1 =
      (vehicle_points->back() - vehicle_points->at(point_delete_index + 1))
          .norm();
  double length2 =
      (vehicle_points->front() - vehicle_points->at(point_delete_index)).norm();
  std::vector<Eigen::Vector3d> new_points;
  auto& points = *vehicle_points;
  if (length1 < length2) {
    new_points.assign(
        points.begin(),
        points.begin() + static_cast<int64>(point_delete_index + 1));
    *vehicle_points = new_points;
  } else {
    new_points.assign(
        points.begin() + static_cast<int64>(point_delete_index + 1),
        points.end());
    *vehicle_points = new_points;
  }
}

void LaneLineMappingPipeline::CollectOutputObjects(
    LaneLinesPtr tracked_lanelines) {
  for (const auto& lane_tracker : lane_trackers_) {
    HLOG_DEBUG << "########CollectOutputObjects############";
    LaneTargetConstPtr lane_target = lane_tracker->GetConstTarget();
    HLOG_DEBUG << "Buffer LaneTarget " << lane_target->ToStr();
    LaneLinePtr output_lane_object =
        std::make_shared<LaneLine>(*lane_target->GetConstTrackedObject());
    if (!lane_gate_keeper_->AbleToOutput(lane_tracker->GetConstTarget(),
                                         GetAllConstTarget())) {
      HLOG_DEBUG << "LaneTrack NOT OUTPUT!";
      HLOG_DEBUG << "MinningTimestamp: "
                 << std::to_string(output_lane_object->latest_tracked_time)
                 << " " << "Buffer LaneTarget "
                 << "TrackStatus: " << lane_target->ToStr() << " "
                 << "LaneTrack NOT OUTPUT!";
      // HLOG_INFO << "use_debug_mode_" << use_debug_mode_;
      continue;
    }
    // output_lane_object->after_intersection =
    //     output_lane_object->vehicle_points.front().x() >
    //     min_intersection_x_;
    // 临时对点删除操作，解决脑补case
    auto& pts = output_lane_object->vehicle_points;
    pts.erase(std::remove_if(pts.begin(), pts.end(),
                             [&](const auto& pt) {
                               return pt.x() > lane_target->GetTruncation();
                             }),
              pts.end());
    if (pts.empty()) {
      continue;
    }
    // 供map_lane使用
    output_lane_object->id = lane_target->Id();
    output_lane_object->send_postlane = lane_target->SendPostLane();
    output_lane_object->deleted_track_ids = lane_target->GetDeletedTrackIds();
    HLOG_DEBUG << "send_postline:" << output_lane_object->send_postlane
               << "id_:" << output_lane_object->id;
    tracked_lanelines->lanelines.push_back(output_lane_object);
  }
}

void LaneLineMappingPipeline::SmoothEndPt() {
  // 用二次曲线来做点的拟合和平滑
  CurveFitter curve_fitter(2);
  for (const auto& lane_tracker : lane_trackers_) {
    auto lane = lane_tracker->GetTarget()->GetTrackedObject();
    auto& pts = lane->vehicle_points;
    // 只对处于图中间线的端点做平滑
    if (pts.front().x() > -50 && pts.size() > 10) {
      std::vector<Eigen::Vector3d> points(pts.begin(), pts.begin() + 6);
      if (!curve_fitter.PolyFitProcess(points)) {
        continue;
      }
      for (int i = 0; i < 3 && i < pts.size(); ++i) {
        double y = curve_fitter.evalueValue(pts[i].x());
        double abs_error = std::abs(y - pts[i].y());
        if (abs_error < 1.5) {
          pts[i][1] = y;
        }
      }
    }
  }
}

bool LaneLineMappingPipeline::CheckBadTrack(
    const LaneTrackerPtr& laneline_track) {
  auto laneline_data =
      laneline_track->GetConstTarget()->GetConstTrackedObject();
  for (auto& point : laneline_data->vehicle_points) {
    if (std::isnan(point.x()) || std::isnan(point.y()) ||
        std::isnan(point.z())) {
      HLOG_ERROR << "track_id:" << laneline_data->id
                 << ", nan data in laneline tracker...";
      return true;
    }
  }
  return false;
}

bool LaneLineMappingPipeline::laneline_nanvalue_trigger() {
  for (const auto& tracker : lane_trackers_) {
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

void LaneLineMappingPipeline::CheckTriggerLanelineNan() {
  static bool enable_16 = true;
  static double last_time_16 = -1;
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
      tc = std::chrono::time_point_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now());
  double curr_time =
      static_cast<double>(tc.time_since_epoch().count()) * 1.0e-9;
  if (enable_16) {
    // mapping trigger 车道线nan值
    HLOG_WARN << "Laneline Nan, Start to trigger dc 1016";
    GLOBAL_DC_TRIGGER.TriggerCollect(1016);
    enable_16 = false;
    last_time_16 = curr_time;
  }
  enable_16 = (curr_time - last_time_16) > 600;
}

// Register plugin.
// REGISTER_LANE_TRACKER(LaneLineMappingPipeline);

}  // namespace lm
}  // namespace mp
}  // namespace hozon
