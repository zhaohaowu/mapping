/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Fang Yu <fangyu1@senseauto.com>
 */

#include "semantic_mm/tracking/database/element_lane_line_db.hpp"

#include <algorithm>
#include <memory>
#include <vector>

#include "common/utility.hpp"
#include "semantic_mm/common/configure.hpp"

namespace senseAD {
namespace localization {
namespace smm {

void ElementLaneLineDB::Reset() {
  // reset input/output data
  frame_pose_ = SE3d(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  percept_data_.clear();
  map_data_.clear();
  std::vector<std::pair<id_t, id_t>>().swap(matching_indices_);
  fused_local_percept_data_.clear();

  // reset trackers
  ll_trackers_.clear();
}

adLocStatus_t ElementLaneLineDB::TrackingAndFusion(bool map_aided) {
  // as map id changes, rematch lost tracker to map firstly
  if (map_aided) RematchLostTrackerToMap();

  // check tracking status switch
  static bool map_adided_last_time = false;
  if (map_adided_last_time && !map_aided) {
    // now tracker ids are associated to map, need to be reassigned for
    // non-map-aided tracking
    LaneLineTracker::PtrUMap new_ll_trackers;
    for (auto item : ll_trackers_) {
      // get new tracker id
      auto new_tracker = std::make_shared<LaneLineTracker>();
      id_t new_tracker_id = new_tracker->GetId();
      LaneLineTracker::Ptr tracker = item.second;
      tracker->SetId(new_tracker_id);
      new_ll_trackers.insert({new_tracker_id, tracker});
    }
    ll_trackers_ = new_ll_trackers;
  }
  map_adided_last_time = map_aided;

  // add frame when translation large enough
  double delta_trans =
      (last_frame_pose_.inverse() * frame_pose_).translation().norm();
  if (delta_trans > 0.7) {
    std::unordered_set<id_t> new_tracker_ids;
    adLocStatus_t status;
    if (map_aided) {
      status = MapAidedTrackAndFusion(&new_tracker_ids);
    } else {
      status = TrackAndFusion(&new_tracker_ids);
    }
    if (status != LOC_SUCCESS) {
      LC_LDEBUG(SMM) << "laneline tracking failed!";
      return LOC_LOCALIZATION_ERROR;
    }

    // NOTE: map lines are splited according to line style, therefore not
    // merge trackers in map-aided mode
    if (!map_aided) BetweenTrackerFusion(new_tracker_ids);
    last_frame_pose_ = frame_pose_;
  }

  adLocStatus_t status = GetStableLaneLines(map_aided);
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(SMM) << "get stable lanelines failed!";
    return LOC_LOCALIZATION_ERROR;
  }
  return LOC_SUCCESS;
}

adLocStatus_t ElementLaneLineDB::MapAidedTrackAndFusion(
    std::unordered_set<id_t>* new_tracker_ids) {
  if (new_tracker_ids == nullptr) {
    return LOC_NULL_PTR;
  }
  new_tracker_ids->clear();

  // for observations matched to map
  for (const auto& match_pair : matching_indices_) {
    // get observation
    id_t percept_id = match_pair.first;
    if (!percept_data_.count(percept_id)) {
      LC_LDEBUG(SMM) << "why not exist percept laneline " << percept_id;
      continue;
    }
    const PerceptLaneLine& percept_ll = percept_data_.at(percept_id);

    // get tracker, tracker id is associated to map laneline id
    id_t tracker_id = match_pair.second;
    LaneLineTracker::Ptr ll_tracker;
    auto iter = ll_trackers_.find(tracker_id);
    if (iter != ll_trackers_.end()) {
      // add observation to tracker
      ll_tracker = iter->second;
      ll_tracker->Update(percept_ll, frame_pose_);
      LC_LDEBUG(SMM) << "update tracker " << tracker_id;
    } else {
      // create new trakcer
      ll_tracker = std::make_shared<LaneLineTracker>();
      ll_tracker->SetId(tracker_id);
      ll_tracker->Update(percept_ll, frame_pose_);
      ll_trackers_.insert(std::make_pair(tracker_id, ll_tracker));
      new_tracker_ids->insert(tracker_id);
      LC_LDEBUG(SMM) << "create tracker " << tracker_id;
    }
  }

  // delete tracker outside of BEV
  for (auto iter = ll_trackers_.begin(); iter != ll_trackers_.end();) {
    if (!iter->second->IsTrackerInBEV(frame_pose_)) {
      LC_LDEBUG(SMM) << "delete tracker " << iter->first;
      ll_trackers_.erase(iter++);
    } else {
      iter++;
    }
  }
  return LOC_SUCCESS;
}

adLocStatus_t ElementLaneLineDB::TrackAndFusion(
    std::unordered_set<id_t>* new_tracker_ids) {
  if (new_tracker_ids == nullptr) {
    return LOC_NULL_PTR;
  }
  new_tracker_ids->clear();

  // transform and sample tracker points to current frame
  LaneLine::PtrUMap tracked_lanelines;
  double sample_grid_size = 4.0;
  for (const auto& id_tracker_pair : ll_trackers_) {
    id_t id = id_tracker_pair.first;
    auto ll = id_tracker_pair.second->GetFusedLaneLine(frame_pose_, true);
    if (ll->processed_bv_points.empty()) continue;

    // sample points from trackers
    const auto& points = ll->processed_bv_points;
    const auto& covs = ll->processed_bv_point_covs;
    auto sample_idxs = VoxelDownSample2D(points, covs, sample_grid_size);
    std::vector<Point3D_t> sample_pts;
    sample_pts.reserve(sample_idxs.size());
    for (auto idx : sample_idxs) {
      const auto& sample_pt = points[idx];
      if (sample_pt.x > 0) sample_pts.emplace_back(sample_pt);
    }
    // sort from near to far
    std::sort(sample_pts.begin(), sample_pts.end(),
              [](const Point3D_t& a, const Point3D_t& b) { return a.x < b.x; });

    // convert PerctpLaneLine to LaneLine data structure
    auto line_seg = std::make_shared<LineSegment>();
    line_seg->SetPoints(std::move(sample_pts));
    line_seg->SetLineType(ll->line_type);
    line_seg->SetLineStyle(ll->line_style);
    std::vector<LineSegment::Ptr> line_segs{line_seg};
    auto map_ll = std::make_shared<LaneLine>(id, line_segs);
    tracked_lanelines.insert({id, map_ll});
  }

  // match frame to trackers
  std::vector<std::pair<id_t, id_t>> temporal_matching_indices;
  auto status = MatchingManager::MatchingLaneLine(SE3d(), percept_data_,
                                                  tracked_lanelines, false,
                                                  &temporal_matching_indices);

  std::unordered_map<id_t, id_t> matched_ids;
  for (const auto& pair : temporal_matching_indices) {
    matched_ids.insert({pair.first, pair.second});
  }

  for (const auto& item : percept_data_) {
    id_t percept_id = item.first;
    const PerceptLaneLine& percept_ll = item.second;

    auto match_iter = matched_ids.find(percept_id);
    if (match_iter != matched_ids.end()) {
      // get tracker
      id_t tracker_id = match_iter->second;
      auto track_iter = ll_trackers_.find(tracker_id);
      if (track_iter == ll_trackers_.end()) {
        LC_LDEBUG(SMM) << "why not exist tracker " << tracker_id;
        continue;
      }
      // add observation to tracker
      track_iter->second->Update(percept_ll, frame_pose_);
    } else {
      // too short percept line not considered
      if (percept_ll.processed_bv_points.empty()) continue;
      double line_length = (percept_ll.processed_bv_points.front() -
                            percept_ll.processed_bv_points.back())
                               .Norm();
      if (line_length < 10.0) continue;

      auto ll_tracker = std::make_shared<LaneLineTracker>();
      ll_tracker->Update(percept_ll, frame_pose_);
      id_t new_tracker_id = ll_tracker->GetId();
      ll_trackers_.insert(std::make_pair(new_tracker_id, ll_tracker));
      new_tracker_ids->insert(new_tracker_id);

      LC_LDEBUG(SMM) << "new tracker line " << new_tracker_id << ", percept id "
                     << percept_id << ", pt num "
                     << percept_ll.processed_bv_points.size();
    }
  }

  // delete tracker outside of BEV
  for (auto iter = ll_trackers_.begin(); iter != ll_trackers_.end();) {
    if (!iter->second->IsTrackerInBEV(frame_pose_)) {
      LC_LDEBUG(SMM) << "delete tracker " << iter->first;
      ll_trackers_.erase(iter++);
    } else {
      iter++;
    }
  }
  return LOC_SUCCESS;
}

adLocStatus_t ElementLaneLineDB::BetweenTrackerFusion(
    const std::unordered_set<id_t>& new_tracker_ids) {
  if (new_tracker_ids.empty()) {
    return LOC_SUCCESS;
  }

  // get new and old lanelines
  std::unordered_map<id_t, PerceptLaneLine> new_lanelines;
  LaneLine::PtrUMap old_lanelines;
  for (auto id_tracker_pair : ll_trackers_) {
    auto id = id_tracker_pair.first;
    if (new_tracker_ids.count(id)) {
      PerceptLaneLine recent_new_ll =
          ll_trackers_.at(id)->GetRecentLaneLine(frame_pose_, 20);
      if (recent_new_ll.processed_bv_points.size() < 10) continue;

      new_lanelines.insert({id, recent_new_ll});
    } else {
      PerceptLaneLine recent_old_ll =
          ll_trackers_.at(id)->GetRecentLaneLine(frame_pose_, 20);
      if (recent_old_ll.processed_bv_points.size() < 10) continue;

      // convert PerctpLaneLine to LaneLine data structure
      auto line_seg = std::make_shared<LineSegment>();
      line_seg->SetPoints(std::move(recent_old_ll.processed_bv_points));
      line_seg->SetLineType(recent_old_ll.line_type);
      line_seg->SetLineStyle(recent_old_ll.line_style);
      std::vector<LineSegment::Ptr> line_segs{line_seg};
      auto map_ll = std::make_shared<LaneLine>(id, line_segs);

      old_lanelines.insert({id, map_ll});
    }
  }

  // rematch old trackers to new trackers
  std::vector<std::pair<id_t, id_t>> new_old_match_indices;
  auto status = MatchingManager::MatchingLaneLine(
      SE3d(), new_lanelines, old_lanelines, false, &new_old_match_indices);
  if (status != LOC_SUCCESS) return LOC_SUCCESS;

  for (const auto& match_pair : new_old_match_indices) {
    id_t new_id = match_pair.first;
    id_t old_id = match_pair.second;
    if (!new_lanelines.count(new_id) || !old_lanelines.count(old_id)) {
      continue;
    }
    // avoid mismatch
    double dist = Line2LineDistance2D(
        new_lanelines.at(new_id).processed_bv_points,
        old_lanelines.at(old_id)->GetLineSegments().front()->GetPoints());
    if (dist > 1.0) continue;

    if (!ll_trackers_.count(old_id)) {
      LC_LDEBUG(SMM) << "why not exist tracker old_id " << old_id;
      continue;
    }
    auto old_tracker = ll_trackers_.at(old_id);
    ll_trackers_.erase(old_id);
    if (!ll_trackers_.count(new_id)) {
      LC_LDEBUG(SMM) << "why not exist tracker new_id " << new_id;
      ll_trackers_[new_id] = old_tracker;
      old_tracker->SetId(new_id);
      continue;
    }
    auto new_tracker = ll_trackers_.at(new_id);
    old_tracker->Update(new_tracker);
    ll_trackers_[new_id] = old_tracker;
    old_tracker->SetId(new_id);
  }
  return LOC_SUCCESS;
}

adLocStatus_t ElementLaneLineDB::RematchLostTrackerToMap() {
  id_t max_id = 0;
  for (const auto& map_line : map_data_) {
    max_id = std::max(max_id, map_line.first);
  }
  for (const auto& tracker : ll_trackers_) {
    max_id = std::max(max_id, tracker.first);
  }

  // get lost trackers
  std::unordered_map<id_t, PerceptLaneLine> lost_lanelines;
  std::unordered_map<id_t, LaneLineTracker::Ptr> lost_trackers;
  for (auto iter = ll_trackers_.begin(); iter != ll_trackers_.end();) {
    id_t id = iter->first;
    LaneLineTracker::Ptr tracker = iter->second;
    PerceptLaneLine ll = tracker->GetRecentLaneLine(frame_pose_, 20);
    if (ll.processed_bv_points.size() < 10) {
      iter++;
      continue;
    }

    // tracker with non-matched id
    auto map_iter = map_data_.find(id);
    if (map_iter == map_data_.end()) {
      lost_lanelines.insert({id, ll});
      lost_trackers.insert({id, tracker});
      ll_trackers_.erase(iter++);
      LC_LDEBUG(SMM) << "tracked line lost " << id;
      continue;
    }
    auto map_line_seg = map_iter->second->GetLineSegments().front();

    // tracker with inconsitent match
    bool semantic_inconsist = ll.line_type != map_line_seg->GetLineType();
    bool geometry_inconsist = false;
    if (!semantic_inconsist) {
      int valid_pt_num = 0;
      double mah_distance = Line2LineManhattanDistance2D(
          ll.processed_bv_points, ll.processed_bv_point_covs,
          map_line_seg->GetPoints(), &valid_pt_num);
      double valid_ratio =
          static_cast<double>(valid_pt_num) / ll.processed_bv_points.size();
      // small overlap or large distance indicate geometrical
      // inconsistency
      geometry_inconsist =
          valid_pt_num < 5 || valid_ratio < 0.3 || mah_distance > 5.99;
    }

    if (semantic_inconsist || geometry_inconsist) {
      id_t new_id = ++max_id;
      lost_lanelines.insert({new_id, ll});
      lost_trackers.insert({new_id, tracker});
      ll_trackers_.erase(iter++);
      LC_LDEBUG(SMM) << "tracked line inconsist with map " << id
                     << ", assign new id " << new_id;
      continue;
    }
    iter++;
  }
  if (lost_lanelines.empty()) return LOC_SUCCESS;

  // rematch lost trackers to map
  std::vector<std::pair<id_t, id_t>> rematch_indices;
  auto status = MatchingManager::MatchingLaneLine(
      SE3d(), lost_lanelines, map_data_, false, &rematch_indices);
  if (status != LOC_SUCCESS) return LOC_SUCCESS;

  // merge between trackers
  for (const auto& match_pair : rematch_indices) {
    id_t id_lost = match_pair.first;
    id_t id_map = match_pair.second;
    if (!lost_lanelines.count(id_lost) || !map_data_.count(id_map)) {
      continue;
    }
    LC_LDEBUG(SMM) << "merge btw lost line " << id_lost << " -> map " << id_map;

    // avoid mismatch
    const auto& ll_lost_pts = lost_lanelines.at(id_lost).processed_bv_points;
    const auto& ll_map_pts =
        map_data_.at(id_map)->GetLineSegments().front()->GetPoints();
    double dist = Line2LineDistance2D(ll_lost_pts, ll_map_pts);
    if (dist > 1.0) {
      LC_LDEBUG(SMM) << "large euclidean distance!";
      continue;
    }

    if (!lost_trackers.count(id_lost)) {
      LC_LDEBUG(SMM) << "why not has id_lost " << id_lost;
      continue;
    }
    auto lost_tracker = lost_trackers.at(id_lost);
    if (!ll_trackers_.count(id_map)) {
      // rematched map laneline is not associated to any tracker
      ll_trackers_.insert({id_map, lost_tracker});
      lost_tracker->SetId(id_map);
    } else {
      // merge map-associated tracker to lost tracker
      auto map_associated_tracker = ll_trackers_.at(id_map);
      lost_tracker->Update(map_associated_tracker);
      ll_trackers_[id_map] = lost_tracker;
      lost_tracker->SetId(id_map);
    }
  }
  return LOC_SUCCESS;
}

adLocStatus_t ElementLaneLineDB::GetStableLaneLines(bool map_aided) {
  fused_local_percept_data_.clear();
  int mature_line_num = 0;
  for (const auto& id_tracker_pair : ll_trackers_) {
    auto tracked_ll =
        id_tracker_pair.second->GetFusedLaneLine(frame_pose_, false);
    if (tracked_ll->processed_bv_points.size() < 3) continue;
    if (tracked_ll->processed_bv_points.size() >= 8) ++mature_line_num;
    fused_local_percept_data_.insert(
        std::make_pair(id_tracker_pair.first, tracked_ll));
  }

  // append recent laneline points if mature lines too few
  // only in non-map-aided mode (relocalization mode)
  if (!map_aided &&
      mature_line_num < static_cast<int>(percept_data_.size()) - 2) {
    fused_local_percept_data_.clear();
    for (const auto& id_tracker_pair : ll_trackers_) {
      auto tracked_ll =
          id_tracker_pair.second->GetFusedLaneLine(frame_pose_, false);
      auto& fused_pts = tracked_ll->processed_bv_points;
      auto& fused_pt_covs = tracked_ll->processed_bv_point_covs;
      // append recent laneline points if fused points too few
      if (fused_pts.size() < 8) {
        auto recent_ll = id_tracker_pair.second->GetRecentLaneLine(frame_pose_);
        const auto& recent_pts = recent_ll.processed_bv_points;
        const auto& recent_pt_covs = recent_ll.processed_bv_point_covs;
        size_t append_num = 10;
        append_num = std::min(recent_pts.size(), append_num);
        fused_pts.insert(fused_pts.end(), recent_pts.begin(),
                         recent_pts.begin() + append_num);
        fused_pt_covs.insert(fused_pt_covs.end(), recent_pt_covs.begin(),
                             recent_pt_covs.begin() + append_num);
        SortPointAndCovFromNearToFar(&fused_pts, &fused_pt_covs);
      }
      if (tracked_ll->processed_bv_points.size() < 3) continue;

      fused_local_percept_data_.insert(
          std::make_pair(id_tracker_pair.first, tracked_ll));
    }
  }

  if (fused_local_percept_data_.empty()) return LOC_LOCALIZATION_ERROR;
  return LOC_SUCCESS;
}

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
