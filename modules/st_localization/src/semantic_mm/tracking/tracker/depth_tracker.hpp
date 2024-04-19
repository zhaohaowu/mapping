/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Fang Yu <fangyu1@senseauto.com>
 */

#pragma once

#include <vector>

#include "localization/data_type/base.hpp"
#include "localization/data_type/percept_data.hpp"
#include "localization/data_type/smart_ptr.hpp"
#include "semantic_mm/common/math_tools.hpp"
#include "semantic_mm/common/voxel_index.hpp"
#include "semantic_mm/tracking/tracker/base_tracker.hpp"

namespace senseAD {
namespace localization {
namespace smm {

class DepthTracker : public BaseTracker {
 public:
  DEFINE_SMART_PTR(DepthTracker)
  DEFINE_PTR_CONTAINER(DepthTracker)

  DepthTracker() : BaseTracker(next_id_++) {}
  ~DepthTracker() override {}

  void SetTimestamp(uint64_t timestamp) { last_update_timestamp_ = timestamp; }

  uint64_t GetTimestamp() const { return last_update_timestamp_; }

  // @brief: update by one bbox observation
  void Update(const Point3D_t& observation, const SE3d& frame_pose) {
    last_observation_pose_ = frame_pose;
    if (!CheckAddObservation(observation, frame_pose)) return;
    observation_rays_.emplace_back(observation);
    observation_poses_.emplace_back(frame_pose);
    projection_poses_.emplace_back(frame_pose.inverse());
    UpdateCenter();
  }

  void Update(const DepthTracker::Ptr& other_tracker) {
    uint64_t other_update_timestamp = other_tracker->GetTimestamp();
    if (last_update_timestamp_ < other_update_timestamp) {
      const auto& observation_rays = other_tracker->GetObservations();
      const auto& observation_poses = other_tracker->GetObservPoses();
      const auto& projection_poses = other_tracker->GetProjectionPoses();
      observation_rays_.insert(observation_rays_.end(),
                               observation_rays.begin(),
                               observation_rays.end());
      observation_poses_.insert(observation_poses_.end(),
                                observation_poses.begin(),
                                observation_poses.end());
      projection_poses_.insert(projection_poses_.end(),
                               projection_poses.begin(),
                               projection_poses.end());
      last_update_timestamp_ = other_update_timestamp;
      last_observation_pose_ = other_tracker->GetLastObservPose();
    } else {
      auto observation_rays = other_tracker->GetObservations();
      auto observation_poses = other_tracker->GetObservPoses();
      auto projection_poses = other_tracker->GetProjectionPoses();
      observation_rays_.swap(observation_rays);
      observation_poses_.swap(observation_poses);
      projection_poses_.swap(projection_poses);
      observation_rays_.insert(observation_rays_.end(),
                               observation_rays.begin(),
                               observation_rays.end());
      observation_poses_.insert(observation_poses_.end(),
                                observation_poses.begin(),
                                observation_poses.end());
      projection_poses_.insert(projection_poses_.end(),
                               projection_poses.begin(),
                               projection_poses.end());
    }

    UpdateCenter();
  }

  // @brief: check if tracker in BEV given query pose
  bool IsTrackerInBEV(const SE3d& Twc) const {
    // NOTE: Twc is RDF camera pose
    if (center_valid_) {
      auto center_local = Twc.inverse() * center_;
      if (center_local.z > -30.0) return true;
    } else {
      double z_trans_dist =
          (Twc.inverse() * last_observation_pose_).translation()(2);
      if (z_trans_dist > -100.0) return true;
    }
    return false;
  }

  bool GetCenterValid() const { return center_valid_; }

  // @brief: get fused bbox center in given frame
  Point3D_t GetFusedCenter(const SE3d& Twv) { return Twv.inverse() * center_; }

  // @brief: get observation count
  size_t GetObservationCount() const { return observation_rays_.size(); }

  const std::vector<Point3D_t>& GetObservations() const {
    return observation_rays_;
  }

  const std::vector<SE3d>& GetProjectionPoses() { return projection_poses_; }

  SE3d GetLastObservPose() { return last_observation_pose_; }

  const std::vector<SE3d>& GetObservPoses() { return observation_poses_; }

 private:
  void UpdateCenter() {
    Point3D_t new_center;
    if (TriangulatePoint(observation_rays_, projection_poses_, &new_center)) {
      center_valid_ = true;
      center_ = new_center;
    }
  }

  bool CheckAddObservation(const Point3D_t& observation,
                           const SE3d& frame_pose) const {
    if (observation_rays_.empty()) return true;

    const Point3D_t& last_observation = observation_rays_.back();
    const SE3d& last_frame_pose = observation_poses_.back();

    double cos_parallax = last_observation.Dot(observation) /
                          (last_observation.Norm() * observation.Norm());
    if (cos_parallax < 0.9998) return true;  // parallax > 1.4deg
    return false;
  }

 private:
  uint64_t last_update_timestamp_ = 0;
  SE3d last_observation_pose_;  // used for tracking
  std::vector<Point3D_t> observation_rays_;
  std::vector<SE3d> observation_poses_;
  std::vector<SE3d> projection_poses_;
  Point3D_t center_;
  bool center_valid_ = false;
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
