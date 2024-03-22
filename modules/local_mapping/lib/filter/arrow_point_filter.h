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
#include "boost/circular_buffer.hpp"
#include "modules/local_mapping/base/scene/arrow.h"
#include "modules/local_mapping/lib/target/base_target.h"
namespace hozon {
namespace mp {
namespace lm {

class ArrowPointFilter {
 public:
  explicit ArrowPointFilter(ArrowTargetPtr arrow_target) {
    target_ref_ = arrow_target;
  }

  bool Init();

  void UpdateWithMeasurement(const ArrowPtr& measurement);

  void UpdateCenterPoint(const ArrowPtr& measurement);

  void UpdateLength(const ArrowPtr& measurement);

  void UpdateWidth(const ArrowPtr& measurement);

  void UpdateHeading(const ArrowPtr& measurement);

  void UpdateVehiclePoints();

  void UpdateWithoutMeasurement();

  bool IsAbnormalPose(const Eigen::Affine3d& novatel2world_pose);

  void Reset();

  bool CheckStableMeasureState();

 private:
  void UpdateResult();

 private:
  bool is_stable_state_ = false;

  ArrowTargetPtr target_ref_;

  boost::circular_buffer<ArrowPtr> history_measure_arrows_;
  int history_measure_size_ = 10;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
