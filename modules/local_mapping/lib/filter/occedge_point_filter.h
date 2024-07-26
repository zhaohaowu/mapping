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

#include "boost/circular_buffer.hpp"
#include "modules/local_mapping/lib/filter/point_manager.h"
#include "modules/local_mapping/lib/interface/base_init_options.h"
#include "modules/local_mapping/lib/target/base_target.h"
#include "modules/local_mapping/utils/curve_fitter.h"

namespace hozon {
namespace mp {
namespace lm {

class OccEdgePointFilter {
 public:
  explicit OccEdgePointFilter(OccEdgeTargetPtr occedge_target)
      : target_ref_(std::move(occedge_target)), curve_fit_(3) {}
  virtual ~OccEdgePointFilter() = default;
  OccEdgePointFilter(const OccEdgePointFilter&) = delete;
  OccEdgePointFilter& operator=(const OccEdgePointFilter&) = delete;

  bool Init(const FilterInitOption& init_options);

  void UpdateWithMeasurement(const FilterOption& filter_options,
                             const OccEdgePtr& measurement);

  void UpdateWithoutMeasurement(const FilterOption& filter_options);

  bool IsAbnormalPose(const Eigen::Affine3d& novatel2world_pose);

  void Reset();

 private:
  void UpdateObservePoints();
  void UpdateHistoryPoints();
  void UpdateStage(const OccEdgePtr& measurement_line);
  int LocateBinIndexByLinearTime(const float& locate_value) const;

  void UpdateResult(bool match_flag);

 private:
  boost::circular_buffer<OccEdgePtr> measures_;
  OccEdgeTargetPtr target_ref_;
  CurveFitter curve_fit_;
  std::vector<std::pair<float, float>> ref_bin_range_table_;
  std::vector<int> count_for_each_bin_;
  std::vector<Eigen::Vector3d> total_observe_points_;
  const float min_near_distance_ = -10.0F;
  const float max_far_distance_ = 150.0F;
  const float middle_near_distance_ = 20.0F;
  const float far_sample_bin_width_ = 2.0F;
  const float near_sample_bin_width_ = 1.0F;
  const int min_point_num_each_bin_ = 1;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
