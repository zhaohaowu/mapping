// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Hozon (hozon@hozon.com)
// @file: mapping_position_manager.h
// @brief: mapping_position for local map

#pragma once
#include <float.h>

#include <functional>
#include <map>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "modules/local_mapping/base/scene/laneline.h"
#include "modules/local_mapping/base/scene/localmap.h"
#include "modules/local_mapping/lib/target/base_target.h"
#include "modules/local_mapping/lib/tracker/laneline_tracker.h"
namespace hozon {
namespace mp {
namespace lm {

class MappingRemoveManager {
 public:
  bool Init();
  // 主处理函数
  void Process(std::vector<LaneTrackerPtr>* trackers);
  // 是否为参考线
  bool isReference(const LaneTargetConstPtr& target,
                   std::vector<LaneTrackerPtr>* trackers);
  // 分车道线为路口前后
  void SetIntersection(std::vector<LaneTrackerPtr>* trackers);
  // 是否车前的路口对面的线
  bool isOnOpposite(const LaneLinePtr& laneline);
  // 选点函数
  void SelectPoints(const LaneLinePtr& laneline,
                    std::vector<Eigen::Vector3d>* point_set, int min_x,
                    int max_x);
  // 获取两条线的一定范围内的平均距离
  float GetAvgDistBetweenTwoLane(const LaneLinePtr& laneline_left,
                                 const LaneLinePtr& laneline_right);
  // 跟踪状态统计函数
  void TrackedStatic(const LaneTargetConstPtr& target, int* true_count,
                     int* max_continue_count);
  // 待删除线之间的博弈删除函数
  bool DeleteLaneisTooNear(const LaneTrackerPtr& lanetarget,
                           std::vector<LaneTrackerPtr>* trackers);
  // 寻找和参考线构成车道的待删除线
  std::map<int, std::vector<LaneTrackerPtr>> GenerateLanesFromRefLines(
      const LaneTrackerPtr& ref_track,
      const std::vector<LaneTrackerPtr>* remove_schedule_lanes);
  bool IsForkConvergelike(const LaneTargetConstPtr& left_line,
                          const LaneTargetConstPtr& right_line);
  bool DeleteLinebetweenRefLane(const LaneTrackerPtr& lanetarget,
                                std::vector<LaneTrackerPtr>* trackers);
  float GetAvgDeltaBetweenTwoLane(const LaneLinePtr& line_delete,
                                  const LaneLinePtr& line_ref);

 private:
  // 需要删除的tracker标记
  std::unordered_set<int> remove_index_;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
