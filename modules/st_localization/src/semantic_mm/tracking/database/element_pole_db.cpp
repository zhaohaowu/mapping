/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Fang Yu <fangyu1@senseauto.com>
 */

#include "semantic_mm/tracking/database/element_pole_db.hpp"

#include <algorithm>
#include <set>

#include "semantic_mm/common/configure.hpp"
#include "semantic_mm/matching/matching_manager.hpp"

namespace senseAD {
namespace localization {
namespace smm {

ElementPoleDB::ElementPoleDB(const CameraModel::Ptr& camera_model,
                             const SE3d& T_veh_cam)
    : camera_model_(camera_model), T_veh_cam_(T_veh_cam) {}

void ElementPoleDB::Reset() {
  // reset input/output data
  timestamp_ = 0;
  frame_pose_ = SE3d(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  percept_data_.clear();
  map_data_.clear();
  std::vector<std::pair<id_t, id_t>>().swap(matching_indices_);
  fused_local_percept_data_.clear();

  // reset trackers
  pole_trackers_.clear();

  // reset tracking related data
  id_mapping_.clear();
  id_mapping_inverse_.clear();
  tracked_pole_data_.clear();
}

adLocStatus_t ElementPoleDB::TrackingAndFusion(bool map_aided) {
  SE3d T_w_cam = frame_pose_ * T_veh_cam_;

  auto& smm_param = Configure::GetInstance()->GetLocalizationSMMParam();
  std::unordered_map<id_t, id_t> matched_ids;
  if (smm_param.track_by_percept_tracker_id) {
    // use tracking results from perception
    for (const auto& item : percept_data_) {
      id_t p_id = item.first;
      auto iter = id_mapping_.find(p_id);
      if (iter != id_mapping_.end()) {
        id_t tracker_id = iter->second;
        matched_ids.insert({p_id, tracker_id});
      }
    }
  } else {
    MatchingManager::MatchingPoleBbox(timestamp_, T_w_cam, percept_data_,
                                      tracked_pole_data_, pole_trackers_,
                                      camera_model_, &matched_ids);
  }

  // update matched tracker or create new tracker
  for (const auto& item : percept_data_) {
    id_t p_id = item.first;
    auto bbox = item.second.rect;
    Point3D_t view_ray;
    camera_model_->Pixel2Camera(bbox.center, &view_ray);
    view_ray = view_ray / view_ray.Norm();

    auto iter = matched_ids.find(p_id);
    if (iter != matched_ids.end()) {
      id_t tracker_id = iter->second;
      auto tracker_iter = pole_trackers_.find(tracker_id);
      if (tracker_iter == pole_trackers_.end()) {
        LC_LERROR(SMM) << "why not exist tracker " << tracker_id;
        continue;
      }
      tracker_iter->second->SetTimestamp(timestamp_);
      tracker_iter->second->Update(view_ray, T_w_cam);
      UpdateTrackedPerceptData(tracker_id, item.second);
    } else {
      auto new_tracker = std::make_shared<DepthTracker>();
      new_tracker->SetTimestamp(timestamp_);
      new_tracker->Update(view_ray, T_w_cam);
      id_t new_tracker_id = new_tracker->GetId();
      LC_LDEBUG(TRACKING) << "new tracker pole " << new_tracker_id;
      pole_trackers_.insert(std::make_pair(new_tracker_id, new_tracker));
      tracked_pole_data_.insert(
          {new_tracker_id, std::make_shared<PerceptPole>(item.second)});
      if (smm_param.track_by_percept_tracker_id) {
        id_mapping_.insert({p_id, new_tracker_id});
        id_mapping_inverse_.insert({new_tracker_id, p_id});
      }
    }
  }

  BetweenTrackerFusion();

  // delete too old trackers
  for (auto iter = pole_trackers_.begin(); iter != pole_trackers_.end();) {
    if (!iter->second->IsTrackerInBEV(T_w_cam)) {
      id_t tracker_id = iter->first;
      tracked_pole_data_.erase(tracker_id);
      if (smm_param.track_by_percept_tracker_id) {
        id_t pid = id_mapping_inverse_[iter->first];
        id_mapping_.erase(pid);
        id_mapping_inverse_.erase(iter->first);
      }
      pole_trackers_.erase(iter++);
    } else {
      iter++;
    }
  }

  auto status = GetStablePoles();
  if (status != LOC_SUCCESS) {
    LC_LERROR(SMM) << "get stable poles failed!";
    return status;
  }
  return LOC_SUCCESS;
}

adLocStatus_t ElementPoleDB::GetStablePoles() {
  fused_local_percept_data_.clear();
  for (auto iter = pole_trackers_.begin(); iter != pole_trackers_.end();
       iter++) {
    if (!iter->second->GetCenterValid()) continue;
    // TODO(xxx): fuse other attribute
    PerceptPole::Ptr fused_pole = std::make_shared<PerceptPole>();
    fused_pole->processed_center = iter->second->GetFusedCenter(frame_pose_);
    // TODO(xx): estimate the covariance
    fused_pole->processed_center_cov = Eigen::Matrix3d::Identity();
    fused_pole->processed_center_cov(1, 1) = 0.25;
    fused_pole->observ_cnt = iter->second->GetObservationCount();
    LC_LDEBUG(TRACKING) << "pole " << iter->first << " observ_cnt "
                        << fused_pole->observ_cnt;
    fused_local_percept_data_.insert({iter->first, fused_pole});
  }
  if (fused_local_percept_data_.empty()) return LOC_LOCALIZATION_ERROR;
  return LOC_SUCCESS;
}

adLocStatus_t ElementPoleDB::UpdateTrackedPerceptData(
    const id_t tracker_id, const PerceptPole& new_percept) {
  auto tracker_iter = pole_trackers_.find(tracker_id);
  if (tracker_iter == pole_trackers_.end()) {
    LC_LERROR(SMM) << "no pole tracker " << tracker_id;
    return LOC_INVALID;
  }
  auto tracked_pole_iter = tracked_pole_data_.find(tracker_id);
  if (tracked_pole_iter == tracked_pole_data_.end()) {
    LC_LERROR(SMM) << "no tracked pole " << tracker_id;
    return LOC_INVALID;
  }

  auto& smm_param = Configure::GetInstance()->GetLocalizationSMMParam();
  if (!smm_param.track_by_percept_tracker_id) {
    // calculate moving velocity in image
    uint64_t last_update_timestamp = tracker_iter->second->GetTimestamp();
    double delta_time_ms = timestamp_ * 1e-6 - last_update_timestamp * 1e-6;
    if (delta_time_ms > 0.0 && delta_time_ms < 500.0) {
      Point2D_t curr_move_v =
          (new_percept.rect.center - tracked_pole_iter->second->rect.center) /
          (delta_time_ms / 100.0);
      Point2D_t last_move_v = tracked_pole_iter->second->move_velocity;
      tracked_pole_iter->second->move_velocity =
          curr_move_v * 0.7 + last_move_v * 0.3;
    } else {
      // get a large time gap, need recalculate the velocity
      tracked_pole_iter->second->move_velocity = Point2D_t(0.0, 0.0);
    }
  }

  // update basic info
  tracked_pole_iter->second->id = new_percept.id;
  tracked_pole_iter->second->confidence = new_percept.confidence;
  tracked_pole_iter->second->rect = new_percept.rect;
  tracked_pole_iter->second->type = new_percept.type;

  return LOC_SUCCESS;
}

adLocStatus_t ElementPoleDB::BetweenTrackerFusion() {
  if (pole_trackers_.size() < 2) return LOC_SUCCESS;

  std::vector<id_t> all_tracker_ids;
  all_tracker_ids.reserve(pole_trackers_.size());
  for (const auto& tracker : pole_trackers_) {
    all_tracker_ids.emplace_back(tracker.first);
  }

  std::unordered_map<id_t, std::vector<id_t>> to_fuse_pairs;
  std::set<id_t> used_ids;
  for (int i = 0; i < all_tracker_ids.size(); ++i) {
    id_t curr_id = all_tracker_ids[i];
    if (used_ids.find(curr_id) != used_ids.end()) continue;

    const auto& curr_tracker = pole_trackers_[curr_id];
    if (!curr_tracker->GetCenterValid()) continue;
    Point3D_t curr_center = curr_tracker->GetFusedCenter(frame_pose_);
    std::vector<id_t> matched_ids;
    for (int j = i + 1; j < all_tracker_ids.size(); ++j) {
      id_t target_id = all_tracker_ids[j];
      if (used_ids.find(target_id) != used_ids.end()) continue;

      const auto& target_tracker = pole_trackers_[target_id];
      if (!target_tracker->GetCenterValid()) continue;
      Point3D_t target_center = target_tracker->GetFusedCenter(frame_pose_);
      if ((target_center - curr_center).Norm2D() < 5.0) {
        SE3d pose_curr = curr_tracker->GetLastObservPose();
        SE3d pose_tar = target_tracker->GetLastObservPose();
        SE3d T_tar_curr = pose_tar.inverse() * pose_curr;
        double depth_in_curr = curr_center.z;
        const auto& bbox_curr = tracked_pole_data_[curr_id]->rect;
        const auto& bbox_tar = tracked_pole_data_[target_id]->rect;
        Point2D_t lt_pt(bbox_curr.center.x - bbox_curr.width / 2.0,
                        bbox_curr.center.y - bbox_curr.length / 2.0);
        Point2D_t rb_pt(bbox_curr.center.x + bbox_curr.width / 2.0,
                        bbox_curr.center.y + bbox_curr.length / 2.0);
        Point3D_t lt_view_ray;
        camera_model_->Pixel2Camera(lt_pt, &lt_view_ray);
        Point3D_t rb_view_ray;
        camera_model_->Pixel2Camera(rb_pt, &rb_view_ray);
        Point3D_t lt_view_proj = T_tar_curr * (lt_view_ray * depth_in_curr);
        Point3D_t rb_view_proj = T_tar_curr * (rb_view_ray * depth_in_curr);
        Point2D_t lt_pt_proj, rb_pt_proj;
        camera_model_->Camera2Pixel(lt_view_proj, &lt_pt_proj);
        camera_model_->Camera2Pixel(rb_view_proj, &rb_pt_proj);
        double bbox_width_proj = std::fabs(rb_pt_proj.x - lt_pt_proj.x);
        double bbox_length_proj = rb_pt_proj.y - lt_pt_proj.y;
        if (bbox_length_proj < 0.0) continue;
        BoundingBox2D bbox_proj;
        bbox_proj.center = Point2D_t(0.5 * (rb_pt_proj.x + lt_pt_proj.x),
                                     0.5 * (rb_pt_proj.y + lt_pt_proj.y));
        bbox_proj.width = bbox_width_proj;
        bbox_proj.length = bbox_length_proj;
        double iou = BoundingBoxIOU(bbox_proj, bbox_tar);
        if (iou > 0.8) {
          matched_ids.emplace_back(target_id);
          used_ids.insert(target_id);
        }
      }
    }
    if (!matched_ids.empty()) {
      matched_ids.emplace_back(curr_id);
      std::sort(matched_ids.begin(), matched_ids.end());
      to_fuse_pairs.insert(std::make_pair(matched_ids[0], matched_ids));
    }
  }

  auto& smm_param = Configure::GetInstance()->GetLocalizationSMMParam();

  // do fusion
  for (const auto& item : to_fuse_pairs) {
    id_t main_tracker_id = item.first;
    auto& main_data = tracked_pole_data_[main_tracker_id];
    main_data = tracked_pole_data_[item.second.back()];

    auto& main_tracker = pole_trackers_[main_tracker_id];
    for (const auto& to_fuse_id : item.second) {
      if (smm_param.track_by_percept_tracker_id) {
        id_t p_id = id_mapping_inverse_[to_fuse_id];
        id_mapping_inverse_.erase(to_fuse_id);
        id_mapping_.erase(p_id);
      }
      if (to_fuse_id == main_tracker_id) continue;
      main_tracker->Update(pole_trackers_[to_fuse_id]);
      pole_trackers_.erase(to_fuse_id);
      tracked_pole_data_.erase(to_fuse_id);
    }

    if (smm_param.track_by_percept_tracker_id) {
      id_mapping_inverse_[main_tracker_id] = main_data->id;
      id_mapping_[main_data->id] = main_tracker_id;
    }
  }

  return LOC_SUCCESS;
}

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
