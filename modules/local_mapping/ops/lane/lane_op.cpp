/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-09-05
 *****************************************************************************/
#include "modules/local_mapping/ops/lane/lane_op.h"

namespace hozon {
namespace mp {
namespace lm {

void LaneOp::Init(BipartiteAssocParams params) {
  BipartiteLaneAssocOptions bipar_lane_assoc_options(params);
  bipar_lane_assoc_.reset(new BipartiteLaneAssoc(bipar_lane_assoc_options));
}

void LaneOp::Match(std::shared_ptr<const Lanes> cur_lanes,
                   std::shared_ptr<LocalMap> local_map,
                   std::shared_ptr<std::vector<LaneMatchInfo>> match_info) {
  HLOG_INFO << "GetMatches";

  match_info->clear();
  if (local_map->local_map_lane_.size() == 0) {
    for (auto cur_lane : cur_lanes->lanes_) {
      LaneMatchInfo match;
      match.frame_lane_ = std::make_shared<Lane>(cur_lane);
      match.update_type_ = ObjUpdateType::ADD_NEW;
      match_info->emplace_back(match);
    }
    return;
  }

  for (auto cur_lane : cur_lanes->lanes_) {
    LaneMatchInfo match;
    match.frame_lane_ = std::make_shared<Lane>(cur_lane);
    bool match_success = false;
    for (auto map_lane : local_map->local_map_lane_) {
      if (cur_lane.track_id_ == map_lane.track_id_) {
        match.map_lane_ = std::make_shared<LocalMapLane>(map_lane);
        match_success = true;
      }
    }
    if (match_success) {
      match.update_type_ = ObjUpdateType::MERGE_OLD;
    } else {
      match.update_type_ = ObjUpdateType::ADD_NEW;
    }
    match_info->emplace_back(match);
  }
}

void LaneOp::Match(std::shared_ptr<const Lanes> cur_lanes,
                   std::shared_ptr<LocalMap> local_map,
                   std::shared_ptr<std::vector<LaneMatchInfo>> match_info,
                   bool use_bipartite_assoc_match,
                   const double& sample_interval) {
  match_info->clear();
  if (local_map->local_map_lane_.size() == 0) {
    for (auto cur_lane : cur_lanes->lanes_) {
      LaneMatchInfo match;
      match.frame_lane_ = std::make_shared<Lane>(cur_lane);
      match.update_type_ = ObjUpdateType::ADD_NEW;
      match_info->emplace_back(match);
    }
    return;
  }

  std::vector<std::vector<Eigen::Vector3d>> lanes_points;
  for (size_t i = 0; i < cur_lanes->lanes_.size(); ++i) {
    std::vector<Eigen::Vector3d> pts;
    CommonUtil::SampleLanePoints(std::make_shared<Lane>(cur_lanes->lanes_[i]),
                                 &pts, sample_interval);
    lanes_points.emplace_back(pts);
  }

  std::unordered_map<int, int> map_det_lm;
  if (use_bipartite_assoc_match) {
    map_det_lm =
        bipar_lane_assoc_->Process(lanes_points, &local_map->local_map_lane_);
  } else {
    lane_assoc_.reset(new LaneAssoc(lane_assoc_options_));
    map_det_lm = lane_assoc_->Process(lanes_points, local_map->local_map_lane_);
  }

  for (size_t i = 0; i < cur_lanes->lanes_.size(); ++i) {
    if (!use_bipartite_assoc_match) {
      if (lane_assoc_->delete_det_lines_.find(i) !=
          lane_assoc_->delete_det_lines_.end())
        continue;
    }
    LaneMatchInfo match;
    match.frame_lane_ = std::make_shared<Lane>(cur_lanes->lanes_[i]);
    if (map_det_lm.find(i) != map_det_lm.end()) {
      match.map_lane_ = std::make_shared<LocalMapLane>(
          local_map->local_map_lane_[map_det_lm[i]]);
      match.update_type_ = ObjUpdateType::MERGE_OLD;
      // HLOG_ERROR << "merge det " << i << " with lm " << map_det_lm[i];
    } else {
      match.update_type_ = ObjUpdateType::ADD_NEW;
    }
    match_info->emplace_back(match);
  }
}

void LaneOp::FilterCurve(std::shared_ptr<LocalMap> local_map,
                         const Lane& cur_lane, const LocalMapLane& map_lane,
                         const double& sample_interval) {
  std::vector<Eigen::Vector3d> new_pts;
  double delta_y1 = 0;
  double delta_y2 = 0;
  for (auto p_map : map_lane.points_) {
    if (p_map.x() < cur_lane.x_start_vrf_) {
      new_pts.emplace_back(p_map);
    } else if (p_map.x() >= cur_lane.x_start_vrf_ &&
               p_map.x() <= cur_lane.x_end_vrf_) {
      Eigen::Vector3d p_lane = Eigen::Vector3d::Identity();
      p_lane.x() = p_map.x();
      p_lane.y() = cur_lane.lane_fit_a_ * pow(p_lane.x(), 3) +
                   cur_lane.lane_fit_b_ * pow(p_lane.x(), 2) +
                   cur_lane.lane_fit_c_ * p_lane.x() + cur_lane.lane_fit_d_;
      // double update_y = 0.8 * p_map.y() + 0.2 * p_lane.y();
      double update_y = 0 * p_map.y() + 1.0 * p_lane.y();
      Eigen::Vector3d tmp(p_map.x(), update_y, 0);
      new_pts.emplace_back(tmp);
      // delta_y1 = 0.6 * delta_y1 + (p_map.y() - update_y) * 0.4;
      // delta_y2 = 0.6 * delta_y2 + (p_lane.y() - update_y) * 0.4;
    } else if (p_map.x() > cur_lane.x_end_vrf_) {
      Eigen::Vector3d tmp(p_map.x(), p_map.y() - delta_y1, 0);
      new_pts.emplace_back(tmp);
    }
  }
  double map_x_end = map_lane.points_[map_lane.points_.size() - 1].x();
  if (map_x_end < cur_lane.x_end_vrf_) {
    Eigen::Vector3d p_lane = Eigen::Vector3d::Identity();
    for (double x = map_x_end + 0.8; x <= cur_lane.x_end_vrf_;
         x += sample_interval) {
      p_lane.x() = x;
      p_lane.y() = cur_lane.lane_fit_a_ * pow(p_lane.x(), 3) +
                   cur_lane.lane_fit_b_ * pow(p_lane.x(), 2) +
                   cur_lane.lane_fit_c_ * p_lane.x() + cur_lane.lane_fit_d_;
      Eigen::Vector3d tmp(p_lane.x(), p_lane.y() - delta_y2, 0);
      new_pts.emplace_back(tmp);
    }
  }
  for (auto& lane : local_map->local_map_lane_) {
    if (lane.track_id_ == map_lane.track_id_) {
      lane.points_ = new_pts;
      return;
    }
  }
  // std::shared_ptr<LaneCubicSpline> filtered_lane_func =
  //     std::make_shared<LaneCubicSpline>();
  // if (filter_map_.find(cur_lane->track_id_) != filter_map_.end()) {
  //   HLOG_INFO << "aaaa";
  //   filter_map_[cur_lane->track_id_]->SetCurLanePose(cur_lane_pose_);
  //   filter_map_[cur_lane->track_id_]->LaneProcess(cur_lane,
  //   filtered_lane_func); filtered_lane_func->start_point_x_ =
  //   cur_lane->x_start_vrf_; filtered_lane_func->end_point_x_ =
  //   cur_lane->x_end_vrf_; CommonUtil::SampleCurvePts(*filtered_lane_func,
  //   new_pts, sample_interval);
  // } else {
  //   HLOG_INFO << "bbbb";
  //   filter_map_[cur_lane->track_id_] = std::make_shared<LaneFilter>();
  //   filter_map_[cur_lane->track_id_]->SetCurLanePose(cur_lane_pose_);
  //   filter_map_[cur_lane->track_id_]->Init(cur_lane);
  // }
}

bool LaneOp::SetCurLanePose(const double time) {
  ConstDrDataPtr dr_ptr = GetDrPoseForTime(time);
  if (dr_ptr == nullptr) {
    HLOG_INFO << "dr_ptr is empty";
    return false;
  }

  Eigen::Quaterniond quat(dr_ptr->quaternion.w(), dr_ptr->quaternion.x(),
                          dr_ptr->quaternion.y(), dr_ptr->quaternion.z());
  Eigen::Matrix3d rotation = quat.toRotationMatrix();
  double theta = atan2(rotation(1, 0), rotation(0, 0));
  cur_lane_pose_ << dr_ptr->pose.x(), dr_ptr->pose.y(), theta;
  HLOG_INFO << "Set Cur Lane Pose: " << cur_lane_pose_.x() << ","
            << cur_lane_pose_.y() << "," << cur_lane_pose_.z();

  return true;
}

void LaneOp::SetLastLanePose() {
  last_lane_pose_ = cur_lane_pose_;

  for (auto it = filter_map_.begin(); it != filter_map_.end(); ++it) {
    it->second->SetLastLanePose(last_lane_pose_);
  }

  HLOG_INFO << "Set last Lane Pose: " << last_lane_pose_.x() << ","
            << last_lane_pose_.y() << "," << last_lane_pose_.z();
}

ConstDrDataPtr LaneOp::GetDrPoseForTime(const double timestamp) {
  auto& local_data = LocalDataSingleton::GetInstance();

  ::std::list<::std::pair<double, ConstDrDataPtr>> dr_list;
  local_data.dr_data_buffer_.get_all_messages(&dr_list);

  if (dr_list.size() <= 1) {
    HLOG_ERROR << "too few dr data, can't interpolate";
    return nullptr;
  }

  auto iter = dr_list.rbegin();
  for (; iter != dr_list.rend(); iter++) {
    if (iter->first < timestamp) {
      // HLOG_ERROR << "Dr time: " << std::setprecision(20) << iter->first;
      break;
    }
  }

  if (iter == dr_list.rbegin()) {
    if (timestamp - dr_list.back().second->timestamp > 0.2) {
      HLOG_ERROR << "Dr delay";
      return nullptr;
    }

    DrDataPtr dr_ptr = std::make_shared<DrData>();
    ConstDrDataPtr latest_data_ptr = dr_list.back().second;

    double delta_t = timestamp - latest_data_ptr->timestamp;
    dr_ptr->timestamp = timestamp;
    dr_ptr->pose =
        latest_data_ptr->pose +
        latest_data_ptr->quaternion * (latest_data_ptr->local_vel * delta_t);
    dr_ptr->quaternion = latest_data_ptr->quaternion;
    Eigen::Vector3d delta_ang = latest_data_ptr->local_omg * delta_t;
    if (delta_ang.norm() > 1e-12) {
      dr_ptr->quaternion =
          dr_ptr->quaternion * Eigen::Quaterniond(Eigen::AngleAxisd(
                                   delta_ang.norm(), delta_ang.normalized()));
      dr_ptr->quaternion = dr_ptr->quaternion.normalized();
    }

    dr_ptr->local_vel = latest_data_ptr->local_vel;
    dr_ptr->local_omg = latest_data_ptr->local_omg;

    return dr_ptr;
  }

  if (iter == dr_list.rend()) {
    HLOG_ERROR << "Dr lost";
    return nullptr;
  }

  auto pre = iter;
  auto next = --iter;
  double ratio = (timestamp - pre->first) / (next->first - pre->first);
  auto dr_pose_state = std::make_shared<DrData>(
      pre->second->Interpolate(ratio, *(next->second), timestamp));
  return dr_pose_state;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
