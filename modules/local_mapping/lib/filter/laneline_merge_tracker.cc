// Copyright 2020 Hozon Inc. All Rights Reserved.
// @file: laneline_merge_tracker.cc
// @brief: merge tracker

#include "modules/local_mapping/lib/filter/laneline_merge_tracker.h"

#include <algorithm>
#include <unordered_set>
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
      insert_pts.emplace_back(pt.x(), y, 0.0);
      continue;
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
  // 如果其中之一是分流和合流线，则不进行merge（待加）；
  double over_lay_ratio = GetOverLayRatioBetweenTwoLane(
      left_line->GetConstTrackedObject(), right_line->GetConstTrackedObject());
  float avg_dist = GetDistBetweenTwoLane(
      left_line->GetConstTrackedObject()->vehicle_points,
      right_line->GetConstTrackedObject()->vehicle_points);
  double time_diff = left_line->GetLastestTrackedTimestamp() -
                     right_line->GetLastestTrackedTimestamp();

  float line_interval = 0;
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
             << " ,line_interval:" << line_interval;
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
    return true;
  } else if (over_lay_ratio <= 0.2 && (line_interval > -4 && avg_dist < 1)) {
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
  }
  return false;
}

// tracker 交叉合并策略

// tracker 重叠区域很近及分叉线的合并策略
bool LaneLineMergeTrack::MergeOverlayCrossStrategy(
    const LaneTargetPtr& left_line, const LaneTargetPtr& right_line) {
  // 如果其中之一是分流和合流线，则不进行merge（待加）；
  double thresh_width = 1.0;
  double over_lay_ratio = GetOverLayRatioBetweenTwoLane(
      left_line->GetConstTrackedObject(), right_line->GetConstTrackedObject());
  float avg_dist = GetDistBetweenTwoLane(
      left_line->GetConstTrackedObject()->vehicle_points,
      right_line->GetConstTrackedObject()->vehicle_points);
  double overlay_min_length = GetOverLayLengthBetweenTwoLane(
      left_line->GetConstTrackedObject()->vehicle_points,
      right_line->GetConstTrackedObject()->vehicle_points, thresh_width);
  double time_diff = left_line->GetLastestTrackedTimestamp() -
                     right_line->GetLastestTrackedTimestamp();
  HLOG_DEBUG << "laneline MergeTracks: id " << left_line->Id() << ", id "
             << right_line->Id() << ", over_lay_ratio: " << over_lay_ratio
             << ", avg_dist: " << avg_dist << ", time_diff: " << time_diff
             << ", overlay_min_length:" << overlay_min_length;
  // 根据线的质量来做删除,需要根据case专门抽一个评估函数
  // 如果两条线有超过10m的重叠区域（需要过滤合流分流），则认为要合并为一条
  if (overlay_min_length > 10 && over_lay_ratio > 0) {
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
    return true;
  }
  return false;
}

bool LaneLineMergeTrack::IsForkConvergelike(
    const LaneTargetConstPtr& left_line, const LaneTargetConstPtr& right_line) {
  const std::vector<Eigen::Vector3d>& point_set1 =
      left_line->GetConstTrackedObject()->vehicle_points;

  const std::vector<Eigen::Vector3d>& point_set2 =
      right_line->GetConstTrackedObject()->vehicle_points;

  if (point_set1.empty() || point_set2.empty()) {
    return false;
  }

  // 短线的长度必须小于40米， 否则不认为是分合流线场景
  float line1_length = GetLength(point_set1);
  float line2_length = GetLength(point_set2);
  HLOG_DEBUG << "line1_length:" << line1_length;
  float short_line_length = std::min(line1_length, line2_length);
  if (short_line_length >= 40) {
    return false;
  }

  std::vector<double> dist_list;
  std::vector<Eigen::Vector3d> point_list;
  dist_list.clear();
  point_list.clear();
  // 前提两条线的车辆系下的点已经从近到远排好序
  float overlay_min = std::max(point_set1.front().x(), point_set2.front().x());
  float overlay_max = std::min(point_set1.back().x(), point_set2.back().x());

  std::vector<Eigen::Vector3d> overlay_point_set1;
  std::vector<Eigen::Vector3d> overlay_point_set2;
  overlay_point_set1.clear();
  overlay_point_set2.clear();

  for (const auto& point : point_set1) {
    if (point.x() < overlay_min || point.x() > overlay_max) {
      continue;
    }
    overlay_point_set1.push_back(point);
  }

  for (const auto& point : point_set2) {
    if (point.x() < overlay_min || point.x() > overlay_max) {
      continue;
    }
    overlay_point_set2.push_back(point);
  }

  if (overlay_point_set1.size() < 2 || overlay_point_set2.size() < 2) {
    return false;
  }

  Eigen::Vector3d A, B, C;
  for (int i = 0, j = 0;
       i < overlay_point_set1.size() && j < overlay_point_set2.size(); ++i) {
    A = overlay_point_set1[i];
    if (A.x() <= overlay_point_set2[0].x()) {
      B = overlay_point_set2[0];
      C = overlay_point_set2[1];
    } else if (A.x() >= overlay_point_set2.back().x()) {
      B = overlay_point_set2[overlay_point_set2.size() - 2];
      C = overlay_point_set2[overlay_point_set2.size() - 1];
    } else if (A.x() < overlay_point_set2[j].x()) {
      B = overlay_point_set2[j - 1];
      C = overlay_point_set2[j];
    } else {
      ++j;
      --i;
      continue;
    }
    double dist = GetDistPointLane(A, B, C);
    dist_list.push_back(dist);
    point_list.push_back(A);
  }

  int order_times = 0;
  int reorder_times = 0;
  int ambiguous_times = 0;
  float equal_length = 0.0;
  int bins = 0;
  for (int i = 0, j = 0; i < point_list.size() && j < point_list.size();) {
    double point_dis = (point_list[i].head(2) - point_list[j].head(2)).norm();
    if (point_dis > 4.0) {
      HLOG_DEBUG << "TEST i:" << i << ",j:" << j << ", point_dis:" << point_dis
                 << ",dis:" << dist_list[j] - dist_list[i];
      bins++;
      if (dist_list[j] - dist_list[i] > 0.15) {
        order_times++;
      } else if (dist_list[j] - dist_list[i] < -0.15) {
        reorder_times++;
      } else if (abs(dist_list[j] - dist_list[i]) < 0.03) {
        ambiguous_times++;
      } else if (abs(dist_list[j] - dist_list[i]) < 0.08) {
        // 横向距离在8cm内认为是重叠区域
        equal_length += point_dis;
      }
      i = j;
      j++;
    } else {
      j++;
    }
  }

  HLOG_DEBUG << "[IsForkConvergelike], bins:" << bins << "order_rate:"
             << 1.0 * order_times / (bins - ambiguous_times + 0.001)
             << "reorder_rate:"
             << 1.0 * reorder_times / (bins - ambiguous_times + 0.001)
             << "same pos length:" << equal_length;

  if (bins <= 2) {
    return false;
  }

  // 以下是判定两条线是否是分合流场景还是一条线存在误检的情况，重叠区域小于20m
  if ((1.0 * order_times / (bins - ambiguous_times + 0.001) > 0.65 ||
       1.0 * reorder_times / (bins - ambiguous_times + 0.001) > 0.65) &&
      equal_length < 15) {
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
