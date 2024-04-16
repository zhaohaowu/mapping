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
#include "modules/local_mapping/base/scene/zebracrossing.h"
#include "modules/local_mapping/lib/target/base_target.h"
namespace hozon {
namespace mp {
namespace lm {

class ZebraCrossingPointFilter {
 public:
  explicit ZebraCrossingPointFilter(
      ZebraCrossingTargetPtr zebracrossing_target) {
    target_ref_ = zebracrossing_target;
  }

  bool Init();

  void UpdateWithMeasurement(const ZebraCrossingPtr& measurement);

  void UpdateCenterPoint(const ZebraCrossingPtr& measurement);

  void UpdateLength(const ZebraCrossingPtr& measurement);

  void UpdateWidth(const ZebraCrossingPtr& measurement);

  void UpdateHeading(const ZebraCrossingPtr& measurement);

  void UpdateVehiclePoints();

  void UpdateWithoutMeasurement();

  bool IsAbnormalPose(const Eigen::Affine3d& novatel2world_pose);

  void Reset();

  bool CheckCenterPointStableMeasureState();

  bool CheckLengthStableMeasureState();

  bool CheckWidthStableMeasureState();

  bool CheckHeadingStableMeasureState();

 private:
  void UpdateResult();

 private:
  bool center_point_is_stable_state_ = false;

  bool length_is_stable_state_ = false;

  bool width_is_stable_state_ = false;

  bool heading_is_stable_state_ = false;

  ZebraCrossingTargetPtr target_ref_;

  boost::circular_buffer<ZebraCrossingPtr> history_measure_zebracrossings_;
  int history_measure_size_ = 10;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
