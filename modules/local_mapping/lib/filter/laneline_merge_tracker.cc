// Copyright 2020 Hozon Inc. All Rights Reserved.
// @file: laneline_merge_tracker.cc
// @brief: merge tracker

#include "modules/local_mapping/lib/filter/laneline_merge_tracker.h"

#include <algorithm>
#include <unordered_set>
#include <utility>
#include <vector>

#include "modules/local_mapping/utils/curve_fitter.h"
#include "modules/local_mapping/utils/lane_utils.h"

namespace hozon {
namespace mp {
namespace lm {
// 把右往左合并
void LaneLineMergeTrack::MergeTrackPoints(
    const LaneTargetConstPtr& left_target,
    const LaneTargetConstPtr& right_target) {
  // 先删掉超过最新的后处理跟踪点
  auto right_pts = right_target->GetConstTrackedObject()->vehicle_points;
  auto& left_pts = left_target->GetConstTrackedObject()->vehicle_points;
  if (left_pts.size() < 5) {
    return;
  }
  // 取left_pts 前面4个点和right_pts最后2个点拟合二次曲线筛选right_pts
  CurveFitter curve_fitter(2);
  std::vector<Eigen::Vector3d> points(left_pts.begin(), left_pts.begin() + 4);
  right_pts.erase(std::remove_if(right_pts.begin(), right_pts.end(),
                                 [&](const auto& pt) {
                                   return pt.x() >= left_pts.front().x() - 1.0;
                                 }),
                  right_pts.end());
  double begin_left_x = left_pts.front().x();
  // 从后往前遍历找最近的两个点加入拟合, 衔接处用拟合点保证平滑性
  for (int i = static_cast<int>(right_pts.size()) - 1, count = 0;
       i >= 0 && count < 2; i--) {
    if (right_pts[i].x() < begin_left_x - 4.0) {
      points.emplace_back(right_pts[i]);
      count++;
    }
  }
  if (!curve_fitter.PolyFitProcess(points)) {
    return;
  }
  std::vector<Eigen::Vector3d> insert_pts;
  insert_pts.reserve(right_pts.size());
  Eigen::Vector3d end_pt;
  for (const auto& pt : right_pts) {
    if (std::abs(pt.x() - begin_left_x) < 4.0) {
      auto y = curve_fitter.evalueValue(pt.x());
      if (std::abs(y - pt.y()) < 2.0) {
        insert_pts.emplace_back(pt.x(), y, 0.0);
        continue;
      }
    }
    insert_pts.emplace_back(pt);
  }
  // 插入最新的后处理跟踪点
  left_pts.insert(left_pts.begin(), insert_pts.begin(), insert_pts.end());
}
// 选择merge后的车道线类型，merge前后虚实不同则相信是实线
void LaneLineMergeTrack::SetLaneLineType(const LaneTargetPtr& curr_line,
                                         const LaneTargetPtr& deleted_line) {
  auto curr_line_type = curr_line->GetTrackedObject()->type;
  auto deleted_line_type = deleted_line->GetTrackedObject()->type;
  // 判断都是虚线
  bool bothDashed =
      (curr_line_type == LaneLineType::DASHED_LINE ||
       curr_line_type == LaneLineType::SHORT_DASHED_LINE ||
       curr_line_type == LaneLineType::DOUBLE_DASHED_LINE ||
       curr_line_type == LaneLineType::LEFT_SOLID_RIGHT_DASHED ||
       curr_line_type == LaneLineType::FISHBONE_DASHED_LINE) &&
      (deleted_line_type == LaneLineType::DASHED_LINE ||
       deleted_line_type == LaneLineType::SHORT_DASHED_LINE ||
       deleted_line_type == LaneLineType::DOUBLE_DASHED_LINE ||
       deleted_line_type == LaneLineType::LEFT_SOLID_RIGHT_DASHED ||
       deleted_line_type == LaneLineType::FISHBONE_DASHED_LINE);
  // 判断都是实线
  bool bothSolid =
      (curr_line_type == LaneLineType::SOLID_LINE ||
       curr_line_type == LaneLineType::DOUBLE_SOLID_LINE ||
       curr_line_type == LaneLineType::RIGHT_SOLID_LEFT_DASHED ||
       curr_line_type == LaneLineType::FISHBONE) &&
      (deleted_line_type == LaneLineType::SOLID_LINE ||
       deleted_line_type == LaneLineType::DOUBLE_SOLID_LINE ||
       deleted_line_type == LaneLineType::RIGHT_SOLID_LEFT_DASHED ||
       deleted_line_type == LaneLineType::FISHBONE);

  // 如果一条为虚线另一条为实线，则将两者都设为实线类型
  if (!bothDashed && !bothSolid) {
    // 找出实线类型并赋值给两者
    curr_line->GetTrackedObject()->type =
        (curr_line_type == LaneLineType::SOLID_LINE ||
         curr_line_type == LaneLineType::DOUBLE_SOLID_LINE ||
         curr_line_type == LaneLineType::RIGHT_SOLID_LEFT_DASHED ||
         curr_line_type == LaneLineType::FISHBONE)
            ? curr_line_type
            : deleted_line_type;
    deleted_line->GetTrackedObject()->type =
        curr_line->GetTrackedObject()->type;  // 保证两者类型一致
  }
}
// 设置跟踪id对
void LaneLineMergeTrack::SetTrackIdPair(const LaneTargetPtr& curr_line,
                                        const LaneTargetPtr& delete_line) {
  if (line_id_pairs_.find(curr_line->Id()) == line_id_pairs_.end()) {
    line_id_pairs_[curr_line->Id()] = curr_line->Id();
  }

  double time_diff = curr_line->GetMatureTrackedTimestamp() -
                     delete_line->GetMatureTrackedTimestamp();
  if (time_diff > 0) {
    line_id_pairs_[curr_line->Id()] = delete_line->Id();
  }
}
// 两条Tracker重合度很高
bool LaneLineMergeTrack::MergeOverlayStrategy(const LaneTargetPtr& left_line,
                                              const LaneTargetPtr& right_line) {
  if (left_line->GetConstTrackedObject()->vehicle_points.back().x() < -10 ||
      right_line->GetConstTrackedObject()->vehicle_points.back().x() < -10) {
    return true;
  }
  // 如果其中之一是分流和合流线，则不进行merge（待加）；
  double over_lay_ratio = GetOverLayRatioBetweenTwoLane(
      left_line->GetConstTrackedObject(), right_line->GetConstTrackedObject());
  float avg_dist = GetDistBetweenTwoLane(
      left_line->GetConstTrackedObject()->vehicle_points,
      right_line->GetConstTrackedObject()->vehicle_points);
  double time_diff = left_line->GetLastestTrackedTimestamp() -
                     right_line->GetLastestTrackedTimestamp();
  double thresh_width = 1.0;
  std::pair<double, double> overlay_length_out = GetOverLayLengthBetweenTwoLane(
      left_line->GetConstTrackedObject()->vehicle_points,
      right_line->GetConstTrackedObject()->vehicle_points, thresh_width);
  double avg_overlap_dist = overlay_length_out.first;
  double avg_overlap_under_thresh_length = overlay_length_out.second;
  float line_interval = 0;
  double left_last_x =
      left_line->GetConstTrackedObject()->vehicle_points.back().x();
  double right_last_x =
      right_line->GetConstTrackedObject()->vehicle_points.back().x();

  double delta_y =
      left_last_x >= right_last_x
          ? std::abs(
                left_line->GetConstTrackedObject()->vehicle_points.front().y() -
                right_line->GetConstTrackedObject()->vehicle_points.back().y())
          : std::abs(
                right_line->GetConstTrackedObject()
                    ->vehicle_points.front()
                    .y() -
                left_line->GetConstTrackedObject()->vehicle_points.back().y());
  if (left_line->GetConstTrackedObject()->vehicle_points.back().x() >
      right_line->GetConstTrackedObject()->vehicle_points.back().x()) {
    line_interval =
        right_line->GetConstTrackedObject()->vehicle_points.back().x() -
        left_line->GetConstTrackedObject()->vehicle_points.front().x();
  } else {
    line_interval =
        left_line->GetConstTrackedObject()->vehicle_points.back().x() -
        right_line->GetConstTrackedObject()->vehicle_points.front().x();
  }
  HLOG_DEBUG << "laneline MergeTracks: id " << left_line->Id() << ", id "
             << right_line->Id() << ", over_lay_ratio: " << over_lay_ratio
             << ", avg_dist: " << avg_dist << ", time_diff: " << time_diff
             << " ,line_interval:" << line_interval
             << " ,avg_overlap_dist:" << avg_overlap_dist
             << " ,delta_y:" << delta_y;
  // 根据线的质量来做删除,需要根据case专门抽一个评估函数
  if (over_lay_ratio > 0.7 && avg_dist < 1.0 ||
      over_lay_ratio > 0.2 && avg_dist < 0.5) {
    // 两条tracker时间差超过2帧保存最新的
    if (std::abs(time_diff) > 0.2) {
      if (time_diff > 0) {
        right_line->SetDeleteFlag(true);
        left_line->SetDeletedTrackIds(*right_line);
        SetLaneLineType(left_line, right_line);
        MergeTrackPoints(left_line, right_line);
        SetTrackIdPair(left_line, right_line);
      } else {
        left_line->SetDeleteFlag(true);
        right_line->SetDeletedTrackIds(*left_line);
        SetLaneLineType(right_line, left_line);
        MergeTrackPoints(right_line, left_line);
        SetTrackIdPair(right_line, left_line);
      }
    } else {
      // 时间差不超过2帧保存跟踪时间长的
      // 如果一条远端点大于另一条的远端点30m，不再选一条，而是直接merge点在一起
      if (left_line->GetConstTrackedObject()->vehicle_points.back().x() -
              right_line->GetConstTrackedObject()->vehicle_points.back().x() >
          30) {
        right_line->SetDeleteFlag(true);
        left_line->SetDeletedTrackIds(*right_line);
        SetLaneLineType(left_line, right_line);
        MergeTrackPoints(left_line, right_line);
        SetTrackIdPair(left_line, right_line);
      } else if (right_line->GetConstTrackedObject()
                         ->vehicle_points.back()
                         .x() -
                     left_line->GetConstTrackedObject()
                         ->vehicle_points.back()
                         .x() >
                 30) {
        left_line->SetDeleteFlag(true);
        right_line->SetDeletedTrackIds(*left_line);
        SetLaneLineType(right_line, left_line);
        MergeTrackPoints(right_line, left_line);
        SetTrackIdPair(right_line, left_line);
      } else {
        if (left_line->Count() > right_line->Count()) {
          right_line->SetDeleteFlag(true);
          left_line->SetDeletedTrackIds(*right_line);
          SetLaneLineType(left_line, right_line);
          SetTrackIdPair(left_line, right_line);
        } else {
          left_line->SetDeleteFlag(true);
          right_line->SetDeletedTrackIds(*left_line);
          SetLaneLineType(right_line, left_line);
          SetTrackIdPair(right_line, left_line);
        }
      }
    }
    return true;
  } else if (over_lay_ratio <= 0.2 &&
             (line_interval > -4 && avg_dist < 4 &&
              ((over_lay_ratio > 0 && avg_overlap_dist < 1) ||
               (over_lay_ratio == 0 && avg_dist < 1 && delta_y < 1)))) {
    if (left_line->GetConstTrackedObject()->vehicle_points.back().x() >
        right_line->GetConstTrackedObject()->vehicle_points.back().x()) {
      right_line->SetDeleteFlag(true);
      left_line->SetDeletedTrackIds(*right_line);
      SetLaneLineType(left_line, right_line);
      MergeTrackPoints(left_line, right_line);
      SetTrackIdPair(left_line, right_line);
    } else {
      left_line->SetDeleteFlag(true);
      right_line->SetDeletedTrackIds(*left_line);
      SetLaneLineType(right_line, left_line);
      MergeTrackPoints(right_line, left_line);
      SetTrackIdPair(right_line, left_line);
    }
    return true;
  }
  return false;
}

// tracker 交叉合并策略

// tracker 重叠区域很近及分叉线的合并策略
bool LaneLineMergeTrack::MergeOverlayCrossStrategy(
    const LaneTargetPtr& left_line, const LaneTargetPtr& right_line) {
  // 尾端点在车身后10m的不merge
  if (left_line->GetConstTrackedObject()->vehicle_points.back().x() < -10 ||
      right_line->GetConstTrackedObject()->vehicle_points.back().x() < -10) {
    return true;
  }
  // 如果其中之一是分流和合流线，则不进行merge（待加）；
  double thresh_width = 1.0;
  double over_lay_ratio = GetOverLayRatioBetweenTwoLane(
      left_line->GetConstTrackedObject(), right_line->GetConstTrackedObject());
  float avg_dist = GetDistBetweenTwoLane(
      left_line->GetConstTrackedObject()->vehicle_points,
      right_line->GetConstTrackedObject()->vehicle_points);
  std::pair<double, double> overlay_length_out = GetOverLayLengthBetweenTwoLane(
      left_line->GetConstTrackedObject()->vehicle_points,
      right_line->GetConstTrackedObject()->vehicle_points, thresh_width);

  double avg_overlap_dist = overlay_length_out.first;
  double avg_overlap_under_thresh_length = overlay_length_out.second;
  double time_diff = left_line->GetLastestTrackedTimestamp() -
                     right_line->GetLastestTrackedTimestamp();

  // 前提两条线的车辆系下的点已经从近到远排好序
  float overlay_min =
      std::max(left_line->GetConstTrackedObject()->vehicle_points.front().x(),
               right_line->GetConstTrackedObject()->vehicle_points.front().x());
  float overlay_max =
      std::min(left_line->GetConstTrackedObject()->vehicle_points.back().x(),
               right_line->GetConstTrackedObject()->vehicle_points.back().x());
  double over_lay_length = overlay_max - overlay_min;
  double over_lay_length_ratio =
      avg_overlap_under_thresh_length / (over_lay_length + 0.00001);
  HLOG_DEBUG << "MergeOverlayCrossStrategy laneline MergeTracks: id "
             << left_line->Id() << ", id " << right_line->Id()
             << ", over_lay_ratio: " << over_lay_ratio
             << ", avg_dist: " << avg_dist << ", time_diff: " << time_diff
             << ", overlay_min_length:" << avg_overlap_under_thresh_length
             << " ,over_lay_length:" << over_lay_length
             << " ,over_lay_length_ratio:" << over_lay_length_ratio
             << ", avg_overlap_dist:" << avg_overlap_dist;
  // 根据线的质量来做删除,需要根据case专门抽一个评估函数
  // 如果两条线有超过10m的重叠区域（需要过滤合流分流），则认为要合并为一条
  if (avg_overlap_under_thresh_length > 10 && over_lay_ratio > 0 &&
      avg_overlap_dist < 1.0) {
    // merge在一起
    if (left_line->GetConstTrackedObject()->vehicle_points.back().x() >
        right_line->GetConstTrackedObject()->vehicle_points.back().x()) {
      right_line->SetDeleteFlag(true);
      left_line->SetDeletedTrackIds(*right_line);
      SetLaneLineType(left_line, right_line);
      MergeTrackPoints(left_line, right_line);
      SetTrackIdPair(left_line, right_line);

    } else {
      left_line->SetDeleteFlag(true);
      right_line->SetDeletedTrackIds(*left_line);
      SetLaneLineType(right_line, left_line);
      MergeTrackPoints(right_line, left_line);
      SetTrackIdPair(right_line, left_line);
    }
    return true;
  }
  return false;
}

// tracker 合并策略
void LaneLineMergeTrack::MergeTracks(std::vector<LaneTrackerPtr>* trackers) {
  line_id_pairs_.clear();
  if (trackers->size() < 2) {
    return;
  }
  for (int i = 0; i < trackers->size() - 1; ++i) {
    const auto& left_line = trackers->at(i)->GetTarget();
    if (!left_line->IsTracked() || left_line->GetDeleteFlag()) {
      continue;
    }
    for (int j = i + 1; j < trackers->size(); ++j) {
      const auto& right_line = trackers->at(j)->GetTarget();
      if (!right_line->IsTracked() || right_line->GetDeleteFlag()) {
        continue;
      }
      // 路口前后的不merge
      // if (left_line->GetConstTrackedObject()->after_intersection !=
      //     right_line->GetConstTrackedObject()->after_intersection) {
      //   HLOG_DEBUG << "left id:" << left_line->GetConstTrackedObject()->id
      //              << " ,right id:" <<
      //              right_line->GetConstTrackedObject()->id;
      //   continue;
      // }
      if (IsForkConvergelike(left_line, right_line)) {
        continue;
      }
      if (MergeOverlayStrategy(left_line, right_line)) {
        continue;
      }
      if (MergeOverlayCrossStrategy(left_line, right_line)) {
        continue;
      }
    }
  }
  trackers->erase(
      std::remove_if(trackers->begin(), trackers->end(),
                     [&](const auto& tracker) {
                       return tracker->GetConstTarget()->GetDeleteFlag();
                     }),
      trackers->end());
  for (auto& tracker : *trackers) {
    if (line_id_pairs_.find(tracker->GetTarget()->Id()) ==
        line_id_pairs_.end()) {
      continue;
    } else {
      tracker->GetTarget()->SetId(line_id_pairs_[tracker->GetTarget()->Id()]);
    }
  }
}
}  // namespace lm
}  // namespace mp
}  // namespace hozon
