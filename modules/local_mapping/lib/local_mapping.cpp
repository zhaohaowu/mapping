/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-08-31
 *****************************************************************************/
#include "modules/local_mapping/lib/local_mapping.h"

#include <list>
#include <utility>

namespace hozon {
namespace mp {
namespace lm {

LMapApp::LMapApp(const std::string& config_file) {
  YAML::Node config = YAML::LoadFile(config_file);
  use_rviz = config["use_rviz"].as<bool>();
  laneOp_ = std::make_shared<LaneOp>();
  mmgr_ = std::make_shared<MapManager>();
  map_lanes_ = std::make_shared<std::vector<LocalMapLane>>();
  latest_location_ = std::make_shared<Location>();
  latest_dr_ = std::make_shared<Location>();
  latest_lanes_ = std::make_shared<Lanes>();
  lane_matches_ = std::make_shared<std::vector<LaneMatchInfo>>();
  new_lane_pts_ = std::make_shared<std::vector<Eigen::Vector3d>>();
  map_init_timestamp_ = 0;
  init_T_ = Eigen::Matrix4d::Identity();
  lasted_T_ = Eigen::Matrix4d::Identity();
  if (use_rviz) {
    util::RvizAgent::Instance().Init("ipc:///tmp/rviz_agent_local_map");
  }
  mmgr_->Init();

  auto& local_data = LocalDataSingleton::GetInstance();
  local_data.Init();
}

void LMapApp::OnLocation(
    const std::shared_ptr<const adsfi_proto::hz_Adsfi::AlgLocation>& msg) {
  DataConvert::SetLocation(*msg, latest_dr_);
}

void LMapApp::OnDr(
    const std::shared_ptr<const adsfi_proto::hz_Adsfi::AlgLocation>& msg) {
  DataConvert::SetDr(*msg, latest_location_);
  if (map_init_timestamp_ < 1e-5) {
    map_init_timestamp_ = latest_location_->timestamp_;
    init_T_.block<3, 3>(0, 0) =
        latest_location_->quaternion_.toRotationMatrix();
    init_T_.block<3, 1>(0, 3) = latest_location_->position_;
  }
  lasted_T_.block<3, 3>(0, 0) =
      latest_location_->quaternion_.toRotationMatrix();
  lasted_T_.block<3, 1>(0, 3) = latest_location_->position_;
  T_V_W_ = init_T_.inverse() * lasted_T_;

  Eigen::Vector3d trans_pose(T_V_W_(0, 3), T_V_W_(1, 3), T_V_W_(2, 3));
  Eigen::Quaterniond trans_quat(T_V_W_.block<3, 3>(0, 0));

  auto& local_data = LocalDataSingleton::GetInstance();
  // HLOG_INFO << "Dr buffer size: " <<
  // local_data.dr_data_buffer_.buffer_size(); HLOG_INFO << "Dr timestamp: " <<
  // std::setprecision(20)
  //           << latest_location_->timestamp_;
  if (local_data.dr_data_buffer_.is_empty() ||
      local_data.dr_data_buffer_.back()->timestamp <
          latest_location_->timestamp_) {
    DrDataPtr dr_data_ptr = std::make_shared<DrData>();
    dr_data_ptr->timestamp = latest_location_->timestamp_;
    dr_data_ptr->pose = trans_pose;
    dr_data_ptr->quaternion = trans_quat;
    dr_data_ptr->local_omg = latest_location_->angular_vrf_;
    dr_data_ptr->local_vel = latest_location_->linear_vrf_;
    local_data.dr_data_buffer_.push_new_message(latest_location_->timestamp_,
                                                dr_data_ptr);
  } else {
    // HLOG_ERROR << "Dr timestamp error";
  }

  if (use_rviz) {
    auto sec = msg->header().timestamp().sec();
    auto nsec = msg->header().timestamp().nsec();
    CommonUtil::PubOdom(T_V_W_, sec, nsec, "/localmap/odom");
    CommonUtil::PubTf(T_V_W_, sec, nsec, "/localmap/tf");
    CommonUtil::PubPath(T_V_W_, sec, nsec, "/localmap/path");
  }
}

void LMapApp::OnLaneLine(
    const std::shared_ptr<
        const adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray>& msg) {
  HLOG_INFO << "OnLaneLine";
  localmap_mutex_.lock();
  auto sec = msg->header().timestamp().sec();
  auto nsec = msg->header().timestamp().nsec();

  DataConvert::SetLaneLine(*msg, latest_lanes_);
  if (use_rviz) {
    CommonUtil::PubPercepPoints(T_V_W_, latest_lanes_, sec, nsec,
                                "/localmap/percep_points");
  }
  map_lanes_.reset(new std::vector<LocalMapLane>());
  mmgr_->GetLanes(map_lanes_);
  ConstDrDataPtr lane_pose = GetDrPoseForTime(latest_lanes_->timestamp_);
  // match current frame lanes to map lanes
  laneOp_->Match(lane_pose, latest_lanes_, map_lanes_, lane_matches_);
  for (auto match : *lane_matches_) {
    if (match.update_type_ == ObjUpdateType::ADD_NEW) {
      CommonUtil::SampleLanePoints(match.frame_lane_, T_V_W_, new_lane_pts_);
      mmgr_->AppendNewLanePoints(*new_lane_pts_);
    } else if (match.update_type_ == ObjUpdateType::MERGE_OLD) {
      laneOp_->FilterCurve(match.map_lane_, match.frame_lane_, new_lane_pts_);
      mmgr_->DeleteLanePoints(match.map_lane_->track_id_, T_V_W_.inverse(),
                              match.frame_lane_->x_start_vrf_);
      auto world_pts = std::make_shared<std::vector<Eigen::Vector3d>>();
      CommonUtil::PointsToWorld(new_lane_pts_, T_V_W_, world_pts);
      mmgr_->AppendOldLanePoints(match.map_lane_->track_id_, *world_pts);
    }
    new_lane_pts_->clear();
  }
  CommonUtil::CubicCurve(&mmgr_->local_map_);
  if (use_rviz) {
    CommonUtil::PubMapPoints(mmgr_->local_map_, sec, nsec,
                             "/localmap/map_points");
  }
  lane_matches_->clear();
  latest_lanes_->front_lanes_.clear();
  latest_lanes_->rear_lanes_.clear();
  localmap_mutex_.unlock();
}
void LMapApp::OnRoadEdge(
    const std::shared_ptr<
        const adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray>& msg) {}

bool LMapApp::FetchLocalMap(
    std::shared_ptr<hozon::mapping::LocalMap> local_map) {
  Eigen::Vector3d p_v = T_V_W_.block<3, 1>(0, 3);
  localmap_mutex_.lock();
  mmgr_->CutLocalMap(150, 150, p_v);
  for (size_t i = 0; i < mmgr_->local_map_.local_map_lane_.size(); ++i) {
    auto lane = local_map->add_lanes();
    for (size_t j = 0;
         j < mmgr_->local_map_.local_map_lane_[i].lane_param_.size(); ++j) {
      lane->mutable_lane_param()->add_cubic_curve_set()->set_c0(
          mmgr_->local_map_.local_map_lane_[i].lane_param_[j].c0_);
      lane->mutable_lane_param()->add_cubic_curve_set()->set_c1(
          mmgr_->local_map_.local_map_lane_[i].lane_param_[j].c1_);
      lane->mutable_lane_param()->add_cubic_curve_set()->set_c2(
          mmgr_->local_map_.local_map_lane_[i].lane_param_[j].c2_);
      lane->mutable_lane_param()->add_cubic_curve_set()->set_c3(
          mmgr_->local_map_.local_map_lane_[i].lane_param_[j].c3_);
      lane->mutable_lane_param()->add_cubic_curve_set()->set_start_point_x(
          mmgr_->local_map_.local_map_lane_[i].lane_param_[j].start_point_x_);
      lane->mutable_lane_param()->add_cubic_curve_set()->set_end_point_x(
          mmgr_->local_map_.local_map_lane_[i].lane_param_[j].end_point_x_);
    }
  }
  localmap_mutex_.unlock();
  return true;
}

ConstDrDataPtr LMapApp::GetDrPoseForTime(double timestamp) {
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
      HLOG_ERROR << "Dr time: " << std::setprecision(20) << iter->first;
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
