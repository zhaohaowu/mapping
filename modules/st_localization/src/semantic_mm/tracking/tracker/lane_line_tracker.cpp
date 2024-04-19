/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Fang Yu <fangyu1@senseauto.com>
 */

#include "semantic_mm/tracking/tracker/lane_line_tracker.hpp"

#include <algorithm>
#include <vector>

#include "semantic_mm/common/math_tools.hpp"

namespace senseAD {
namespace localization {
namespace smm {

LaneLineTracker::LaneLineTracker() : BaseTracker(next_id_++) {
  voxel_idx_converter_.reset(new VoxelIndexConverter(voxel_size_, voxel_size_));

  fused_laneline_local_.reset(new PerceptLaneLine);
}

void LaneLineTracker::Update(const PerceptLaneLine& observation,
                             const SE3d& frame_pose) {
  // update with newest observation
  fused_laneline_local_->line_width = observation.line_width;
  fused_laneline_local_->color = observation.color;
  fused_laneline_local_->line_type = observation.line_type;
  fused_laneline_local_->line_style = observation.line_style;
  fused_laneline_local_->linked_id = observation.linked_id;

  const std::vector<Point3D_t>& bv_points = observation.processed_bv_points;
  const std::vector<Eigen::Matrix2d>& bv_point_covs =
      observation.processed_bv_point_covs;

  // add all points
  for (size_t i = 0; i < bv_points.size(); ++i) {
    Point3D_t p3d_world = frame_pose * bv_points[i];
    const Eigen::Matrix2d& cov = bv_point_covs[i];
    Eigen::Vector2d pt(p3d_world.x, p3d_world.y);
    auto index = voxel_idx_converter_->PointEigenToVoxelIndex(pt);

    auto iter = voxel_laneline_.find(index);
    if (iter == voxel_laneline_.end()) {
      // create new voxel
      LaneLineBVPoint ll_point(pt, cov);
      voxel_cache_.push_front({index, ll_point});
      voxel_laneline_.insert({index, voxel_cache_.begin()});

      // delete least recently used voxel
      if (voxel_cache_.size() > voxel_capacity_) {
        voxel_laneline_.erase(voxel_cache_.back().first);
        voxel_cache_.pop_back();
      }
    } else {
      // add observation point and move node to front
      iter->second->second.AddPoint(pt, cov);
      voxel_cache_.splice(voxel_cache_.begin(), voxel_cache_, iter->second);
      voxel_laneline_[index] = voxel_cache_.begin();
    }
  }

  ++update_cnt_;
}

void LaneLineTracker::Update(const LaneLineTracker::Ptr& other_tracker) {
  // update with other tracker
  fused_laneline_local_->line_width =
      other_tracker->fused_laneline_local_->line_width;
  fused_laneline_local_->color = other_tracker->fused_laneline_local_->color;
  fused_laneline_local_->line_type =
      other_tracker->fused_laneline_local_->line_type;
  fused_laneline_local_->line_style =
      other_tracker->fused_laneline_local_->line_style;
  fused_laneline_local_->linked_id =
      other_tracker->fused_laneline_local_->linked_id;

  // reverse iteraton as point at begin is most recently observed
  for (auto r_iter = other_tracker->voxel_cache_.rbegin();
       r_iter != other_tracker->voxel_cache_.rend(); r_iter++) {
    const LaneLineBVPoint& other_bv_point = r_iter->second;
    auto index = r_iter->first;
    auto iter = voxel_laneline_.find(index);
    if (iter == voxel_laneline_.end()) {
      // create new voxel
      voxel_cache_.push_front({index, other_bv_point});
      voxel_laneline_.insert({index, voxel_cache_.begin()});

      // delete least recently used voxel
      if (voxel_cache_.size() > voxel_capacity_) {
        voxel_laneline_.erase(voxel_cache_.back().first);
        voxel_cache_.pop_back();
      }
    } else {
      // add observation point and move node to front
      iter->second->second.AddPoint(other_bv_point);
      voxel_cache_.splice(voxel_cache_.begin(), voxel_cache_, iter->second);
      voxel_laneline_[index] = voxel_cache_.begin();
    }
  }

  update_cnt_ += other_tracker->update_cnt_;
}

bool LaneLineTracker::IsTrackerInBEV(const SE3d& Twv) const {
  const auto& latest_insert_point = voxel_cache_.begin()->second.point;
  Point3D_t pt_e;
  pt_e.x = latest_insert_point(0);
  pt_e.y = latest_insert_point(1);
  pt_e.z = Twv.translation()(2);
  Point3D_t pt_e_local = Twv.inverse() * pt_e;
  if (pt_e_local.x < -20.0) return false;
  return true;
}

PerceptLaneLine LaneLineTracker::GetRecentLaneLine(const SE3d& Twv,
                                                   size_t point_num) {
  PerceptLaneLine ll_recent;
  ll_recent.line_width = fused_laneline_local_->line_width;
  ll_recent.color = fused_laneline_local_->color;
  ll_recent.line_type = fused_laneline_local_->line_type;
  ll_recent.line_style = fused_laneline_local_->line_style;
  ll_recent.linked_id = fused_laneline_local_->linked_id;

  SE3d Tvw = Twv.inverse();
  double z = Twv.translation()(2);
  ll_recent.processed_bv_points.reserve(point_num);
  ll_recent.processed_bv_point_covs.reserve(point_num);
  for (const auto& index_pt_pair : voxel_cache_) {
    if (ll_recent.processed_bv_points.size() >= point_num) break;
    const auto& bv_point = index_pt_pair.second;
    Point3D_t point_local =
        Tvw * Point3D_t(bv_point.point(0), bv_point.point(1), z);
    if (point_local.x < -20.0) continue;
    ll_recent.processed_bv_points.emplace_back(point_local);
    ll_recent.processed_bv_point_covs.emplace_back(bv_point.cov);
  }

  SortPointAndCovFromNearToFar(&ll_recent.processed_bv_points,
                               &ll_recent.processed_bv_point_covs);

  return ll_recent;
}

PerceptLaneLine::Ptr LaneLineTracker::GetFusedLaneLine(const SE3d& Twv,
                                                       bool get_all_points) {
  // TODO(fy): save computation if tracker not updated
  // prepare frame pose
  Twv_query_ = Twv;
  SE3d Tvw = Twv.inverse();
  double z = Twv.translation()(2);

  // get local points
  auto& fused_bv_pts = fused_laneline_local_->processed_bv_points;
  auto& fused_bv_pt_covs = fused_laneline_local_->processed_bv_point_covs;
  fused_bv_pts.clear();
  fused_bv_pt_covs.clear();
  fused_bv_pts.reserve(voxel_laneline_.size());
  fused_bv_pt_covs.reserve(voxel_laneline_.size());

  // get all points
  if (get_all_points) {
    for (const auto& index_pt_pair : voxel_cache_) {
      const auto& bv_point = index_pt_pair.second;
      Point3D_t point_local =
          Tvw * Point3D_t(bv_point.point(0), bv_point.point(1), z);
      if (point_local.x < -20.0) continue;
      fused_bv_pts.emplace_back(point_local);
      fused_bv_pt_covs.emplace_back(bv_point.cov);
    }
    return fused_laneline_local_;
  }

  // get stable points during tracking
  using StablePoints =
      std::unordered_map<VoxelIndex, LaneLineBVPoint, VoxelIndexHash>;
  StablePoints stable_local_points;
  // non maximum suppression using larger voxel size
  VoxelIndexConverter voxel_index_converter(voxel_size_nms_);
  int min_observ_num = 2;
  for (const auto& index_pt_pair : voxel_cache_) {
    LaneLineBVPoint bv_point = index_pt_pair.second;
    if (bv_point.observ_cnt < min_observ_num) continue;
    Eigen::Vector3d point_local =
        Tvw * Eigen::Vector3d(bv_point.point(0), bv_point.point(1), z);
    if (point_local(0) < -20.0) continue;
    bv_point.point = point_local.head(2);

    auto index = voxel_index_converter.PointEigenToVoxelIndex(bv_point.point);
    auto iter = stable_local_points.find(index);
    if (iter == stable_local_points.end()) {
      stable_local_points.insert({index, bv_point});
    } else {
      // insert point if has smaller covariance
      double cov_original = iter->second.cov.diagonal().norm();
      double cov_other = bv_point.cov.diagonal().norm();
      if (cov_other < cov_original) {
        iter->second.point = bv_point.point;
        iter->second.cov = bv_point.cov;
        iter->second.observ_cnt = bv_point.observ_cnt;
      }
    }
  }

  // size_t max_stable_point_num = 50;
  // if (stable_local_points.size() > max_stable_point_num) {
  //     // get mostly observed stable points
  //     std::vector<StablePoints::iterator> sorted_stable_points;
  //     sorted_stable_points.reserve(stable_local_points.size());
  //     for (auto iter = stable_local_points.begin();
  //          iter != stable_local_points.end(); ++iter) {
  //         sorted_stable_points.emplace_back(iter);
  //     }
  //     std::sort(sorted_stable_points.begin(), sorted_stable_points.end(),
  //               [](const StablePoints::iterator& iter_i,
  //                  const StablePoints::iterator& iter_j) {
  //                   return iter_i->second.observ_cnt >
  //                          iter_j->second.observ_cnt;
  //               });

  //     for (const auto& item : sorted_stable_points) {
  //         auto pt = item->second.point;
  //         double cov_factor = item->second.observ_cnt == 1 ? 2.0 : 1.0;
  //         fused_bv_pts.emplace_back(Point3D_t(pt(0), pt(1), 0));
  //         fused_bv_pt_covs.emplace_back(cov_factor * item->second.cov);
  //         if (fused_bv_pts.size() >= max_stable_point_num) break;
  //     }
  // } else {
  //     for (const auto& item : stable_local_points) {
  //         auto pt = item.second.point;
  //         double cov_factor = item.second.observ_cnt == 1 ? 2.0 : 1.0;
  //         fused_bv_pts.emplace_back(Point3D_t(pt(0), pt(1), 0));
  //         fused_bv_pt_covs.emplace_back(cov_factor * item.second.cov);
  //     }
  // }

  for (const auto& item : stable_local_points) {
    auto pt = item.second.point;
    fused_bv_pts.emplace_back(pt(0), pt(1), 0);
    fused_bv_pt_covs.emplace_back(item.second.cov);
  }

  SortPointAndCovFromNearToFar(&fused_laneline_local_->processed_bv_points,
                               &fused_laneline_local_->processed_bv_point_covs);

  return fused_laneline_local_;
}

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
