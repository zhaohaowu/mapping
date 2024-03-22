// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: lane_point_filter.h
// @brief: filter 3d points

#pragma once
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "base/scene/laneline.h"
#include "boost/circular_buffer.hpp"
#include "modules/local_mapping/base/scene/stopline.h"
#include "modules/local_mapping/lib/target/base_target.h"
namespace hozon {
namespace mp {
namespace lm {

struct Point {
  double x;
  double y;
};

class StopLinePointFilter {
 public:
  explicit StopLinePointFilter(StopLineTargetPtr stopline_target) {
    target_ref_ = stopline_target;
  }

  bool Init();

  void UpdateWithMeasurement(const StopLinePtr& measurement);

  void UpdateCenterPoint(const StopLinePtr& measurement);

  void OptiCenterPoint();

  void UpdateLength(const StopLinePtr& measurement);

  void UpdateHeading(const StopLinePtr& measurement);

  void UpdateVehiclePoints();

  void UpdateWithoutMeasurement();

  bool IsAbnormalPose(const Eigen::Affine3d& novatel2world_pose);

  void Reset();

  bool CheckStableMeasureState();

 private:
  std::map<std::string, LaneLinePtr> GetAllEgolines();

  std::map<std::string, LaneLinePtr> SelectEgolines(
      const std::map<std::string, LaneLinePtr>& all_ego_lines);

  bool IsPointInsideBox(const Point& point, const ZebraCrossing& zebracross);

  void MoveOutsideBox(Point* point, const ZebraCrossing& box);

  bool is_stable_state_ = false;

  StopLineTargetPtr target_ref_;

  // 维护历史10帧的观测数据用于做滤波
  boost::circular_buffer<StopLinePtr> history_measure_stoplines_;
  int history_measure_size_ = 10;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
