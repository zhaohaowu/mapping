/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Fang Yu <fangyu1@senseauto.com>
 */

#pragma once

#include <algorithm>
#include <list>
#include <unordered_map>
#include <utility>

#include "localization/data_type/base.hpp"
#include "localization/data_type/percept_data.hpp"
#include "localization/data_type/smart_ptr.hpp"
#include "semantic_mm/common/voxel_index.hpp"
#include "semantic_mm/tracking/tracker/base_tracker.hpp"

namespace senseAD {
namespace localization {
namespace smm {

// class holds the lane line tracker
class LaneLineTracker : public BaseTracker {
 public:
  // laneline bv point struct
  struct LaneLineBVPoint {
    LaneLineBVPoint() = default;
    LaneLineBVPoint(const Eigen::Vector2d& point, const Eigen::Matrix2d& cov)
        : point(point), cov(cov) {
      observ_cnt = 1;
    }
    void AddPoint(const Eigen::Vector2d& point_obs,
                  const Eigen::Matrix2d& cov_obs, int obs_cnt = 1) {
      observ_cnt += obs_cnt;
      double w = 1.0 / (std::max(cov.diagonal().norm(), 1e-4));
      double w_obs = 1.0 / (std::max(cov_obs.diagonal().norm(), 1e-4));
      double w_sum = w + w_obs;
      point = (w * point + w_obs * point_obs) / w_sum;
      cov = (w * cov + w_obs * cov_obs) / w_sum;
    }
    void AddPoint(const LaneLineBVPoint& other) {
      AddPoint(other.point, other.cov, other.observ_cnt);
    }

    Eigen::Vector2d point = Eigen::Vector2d::Zero();
    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
    size_t observ_cnt{0};
  };

 public:
  DEFINE_SMART_PTR(LaneLineTracker)
  DEFINE_PTR_CONTAINER(LaneLineTracker)

  LaneLineTracker();
  ~LaneLineTracker() override {}

  // @brief: update by one lane line observation
  void Update(const PerceptLaneLine& observation, const SE3d& frame_pose);
  void Update(const LaneLineTracker::Ptr& other_tracker);

  // @brief: check if tracker in BEV given query pose
  bool IsTrackerInBEV(const SE3d& Twv) const;

  // @brief: get recent observed lanline points
  PerceptLaneLine GetRecentLaneLine(const SE3d& Twv, size_t point_num = 20);

  // @brief: get fused laneline
  PerceptLaneLine::Ptr GetFusedLaneLine(const SE3d& Twv, bool get_all_points);

 private:
  SE3d Twv_query_;                             // query vehicle pose
  PerceptLaneLine::Ptr fused_laneline_local_;  // fused local laneline

  // voxel related params
  static constexpr size_t voxel_capacity_ = 150;  // maximum voxel number
  static constexpr double voxel_size_ = 0.25;     // voxel size
  static constexpr double voxel_size_nms_ =
      1.0;  // voxel size for non-maximum suppression
  VoxelIndexConverter::Ptr voxel_idx_converter_;  // voxel index converter
  using VoxelKeyVal = std::pair<VoxelIndex, LaneLineBVPoint>;
  std::unordered_map<VoxelIndex, typename std::list<VoxelKeyVal>::iterator,
                     VoxelIndexHash>
      voxel_laneline_;                  // map voxel index to bv point
  std::list<VoxelKeyVal> voxel_cache_;  // LRU cache of bv points

  size_t update_cnt_ = 0;
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
