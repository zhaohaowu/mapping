// Copyright 2020 Hozon Inc. All Rights Reserved.
// @file: laneline_merge_tracker.cc
// @brief: merge tracker

#include "modules/local_mapping/lib/filter/laneline_merge_tracker.h"

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
  curve_fitter.PolyFitProcess(points);
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

// 两条Tracker重合度很高
bool LaneLineMergeTrack::MergeOverlayStrategy(
    const LaneTargetConstPtr& left_line, const LaneTargetConstPtr& right_line) {
  double over_lay_ratio = GetOverLayRatioBetweenTwoLane(
      left_line->GetConstTrackedObject(), right_line->GetConstTrackedObject());
  float avg_dist = GetDistBetweenTwoLane(
      left_line->GetConstTrackedObject()->vehicle_points,
      right_line->GetConstTrackedObject()->vehicle_points);
  double time_diff = left_line->GetLastestTrackedTimestamp() -
                     right_line->GetLastestTrackedTimestamp();
  HLOG_DEBUG << "laneline MergeTracks: id " << left_line->Id() << ", id "
             << right_line->Id() << ", over_lay_ratio: " << over_lay_ratio
             << ", avg_dist: " << avg_dist << ", time_diff: " << time_diff;
  // 根据线的质量来做删除,需要根据case专门抽一个评估函数
  if (over_lay_ratio > 0.7 && avg_dist < 1.0) {
    // 两条tracker时间差超过2帧保存最新的
    if (std::abs(time_diff) > 0.2) {
      if (time_diff > 0) {
        remove_index_.insert(right_line->Id());
        MergeTrackPoints(left_line, right_line);
      } else {
        remove_index_.insert(left_line->Id());
        MergeTrackPoints(right_line, left_line);
      }
    } else {
      // 时间差不超过2帧保存跟踪时间长的
      if (left_line->Count() > right_line->Count()) {
        remove_index_.insert(right_line->Id());
      } else {
        remove_index_.insert(left_line->Id());
      }
    }
    return true;
  }
  return false;
}

// tracker 交叉合并策略

// tracker 重叠区域很近合并策略

// tracker 合并策略
void LaneLineMergeTrack::MergeTracks(std::vector<LaneTrackerPtr>* trackers) {
  if (trackers->size() < 2) {
    return;
  }
  remove_index_.clear();
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
      MergeOverlayStrategy(left_line, right_line);
    }
  }
  trackers->erase(std::remove_if(trackers->begin(), trackers->end(),
                                 [&](const auto& tracker) {
                                   return remove_index_.count(
                                              tracker->GetConstTarget()->Id()) >
                                          0;
                                 }),
                  trackers->end());
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
