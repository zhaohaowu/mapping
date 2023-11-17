/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-08-31
 *****************************************************************************/
#include "modules/local_mapping/local_mapping.h"

// NOLINTBEGIN
DEFINE_uint32(zone, 51, "time zone");
// NOLINTEND

namespace hozon {
namespace mp {
namespace lm {

LMapApp::LMapApp(const std::string& config_file) {
  YAML::Node config = YAML::LoadFile(config_file);
  use_perception_match_ = config["use_perception_match"].as<bool>();
  use_bipartite_assoc_match_ = config["use_bipartite_assoc_match"].as<bool>();
  use_rviz_ = config["use_rviz"].as<bool>();
  compute_error = config["compute_error"].as<bool>();
  sample_interval_ = config["sample_interval"].as<double>();
  BipartiteAssocParams params;
  params.min_match_point_num = config["min_match_point_num"].as<int>();
  params.min_overlap_point_num = config["min_overlap_point_num"].as<int>();
  params.same_group_max_dist = config["same_group_max_dist"].as<double>();
  params.match_score_threshold = config["match_score_threshold"].as<double>();
  laneOp_ = std::make_shared<LaneOp>();
  laneOp_->Init(params);
  mmgr_ = std::make_shared<MapManager>();
  laneline_inited_ = false;
  dr_inited_ = false;
  // load hdmap
  provider_ = std::make_shared<PriorProvider>();
  provider_->Init(config_file);
  auto map = provider_->GetPrior();
  hdmap_ = std::make_shared<hozon::hdmap::HDMap>();
  hdmap_->LoadMapFromProto(*map);
  mmgr_->Init();
  local_map_ = std::make_shared<LocalMap>();

  auto& local_data = LocalDataSingleton::GetInstance();
  local_data.Init();
}

void LMapApp::OnLocation(
    const std::shared_ptr<const hozon::localization::Localization>& msg) {
  std::shared_ptr<Location> latest_location = std::make_shared<Location>();
  DataConvert::SetLocation(*msg, latest_location);
}

void LMapApp::OnDr(
    const std::shared_ptr<const hozon::dead_reckoning::DeadReckoning>& msg) {
  std::shared_ptr<Location> latest_dr = std::make_shared<Location>();
  DataConvert::SetDr(*msg, latest_dr);
  static bool dr_flag = true;
  static Sophus::SE3d init_T;
  if (dr_flag) {
    dr_flag = false;
    init_T = Sophus::SE3d(latest_dr->quaternion_, latest_dr->position_);
  }
  Sophus::SE3d latest_T =
      Sophus::SE3d(latest_dr->quaternion_, latest_dr->position_);
  Sophus::SE3d T_W_V = init_T.inverse() * latest_T;
  Eigen::Vector3d trans_pose = T_W_V.translation();
  Eigen::Quaterniond trans_quat = T_W_V.so3().unit_quaternion();
  auto& local_data = LocalDataSingleton::GetInstance();
  // HLOG_ERROR << "Dr buffer size: " <<
  // local_data.dr_data_buffer_.buffer_size();

  // HLOG_ERROR << "Dr timestamp: " << std::setprecision(20)
  //            << latest_dr->timestamp_;
  if (local_data.dr_data_buffer_.is_empty() ||
      local_data.dr_data_buffer_.back()->timestamp < latest_dr->timestamp_) {
    DrDataPtr dr_data_ptr = std::make_shared<DrData>();
    dr_data_ptr->timestamp = latest_dr->timestamp_;
    dr_data_ptr->pose = trans_pose;
    dr_data_ptr->quaternion = trans_quat;
    dr_data_ptr->local_omg = latest_dr->angular_vrf_;
    dr_data_ptr->local_vel = latest_dr->linear_vrf_;
    local_data.dr_data_buffer_.push_new_message(latest_dr->timestamp_,
                                                dr_data_ptr);
  } else {
    // HLOG_ERROR << "Dr timestamp error";
  }
  dr_inited_ = true;
}

void LMapApp::OnIns(
    const std::shared_ptr<const hozon::localization::HafNodeInfo>& msg) {
  // Eigen::Vector3d p_G_V(msg->pos_gcj02().x(), msg->pos_gcj02().y(),
  //                       msg->pos_gcj02().z());
  // Eigen::Quaterniond q_G_V(msg->quaternion().w(), msg->quaternion().x(),
  //                          msg->quaternion().y(), msg->quaternion().z());
  // Sophus::SE3d T_G_V = Sophus::SE3d(q_G_V, p_G_V);
  // static int ins_n = 0;
  // ins_n++;
  // if (use_rviz_ && RVIZ_AGENT.Ok() && ins_n == 100) {
  //   ins_n = 0;
  //   if (!dr_inited_) return;
  //   auto sec = static_cast<uint64_t>(msg->header().gnss_stamp());
  //   auto nsec = static_cast<uint64_t>((msg->header().gnss_stamp() - sec) *
  //   1e9); if (compute_error) {
  //     std::vector<Eigen::Vector3d> hq_pts;
  //     T_W_V_mutex_.lock();
  //     CommonUtil::PubHdMapPoints(T_G_V, T_W_V_, hdmap_, sec, nsec,
  //                                "/localmap/hq_map_points", &hq_pts);
  //     T_W_V_mutex_.unlock();
  //     if (hq_pts.size() && local_map_->local_map_lane_.size()) {
  //       float error = loss_.Process(hq_pts, local_map_->local_map_lane_);
  //     }
  //   } else {
  //     T_W_V_mutex_.lock();
  //     CommonUtil::PubHdMapPoints(T_G_V, T_W_V_, hdmap_, sec, nsec,
  //                                "/localmap/hq_map_points");
  //     T_W_V_mutex_.unlock();
  //   }
  // }
}

void LMapApp::OnLaneLine(
    const std::shared_ptr<const hozon::perception::TransportElement>& msg) {
  auto start = std::chrono::high_resolution_clock::now();
  // HLOG_ERROR << "LaneLine timestamp: " << std::setprecision(20)
  //            << msg->header().gnss_stamp();
  ConstDrDataPtr lane_pose = GetDrPoseForTime(msg->header().gnss_stamp());
  if (lane_pose == nullptr) {
    HLOG_ERROR << "lane_pose is nullptr";
    return;
  }
  std::shared_ptr<Lanes> latest_lanes = std::make_shared<Lanes>();
  DataConvert::SetLaneLine(*msg, latest_lanes);
  Sophus::SE3d T_W_V = Sophus::SE3d(lane_pose->quaternion, lane_pose->pose);
  T_W_V_mutex_.lock();
  T_W_V_ = T_W_V;
  T_W_V_mutex_.unlock();
  static Sophus::SE3d last_T_W_V;
  Sophus::SE3d T_C_L = T_W_V.inverse() * last_T_W_V;
  last_T_W_V = T_W_V;
  mmgr_->UpdateLane(local_map_, T_C_L);
  local_map_->timestamp = latest_lanes->timestamp_;
  std::shared_ptr<std::vector<LaneMatchInfo>> lane_matches =
      std::make_shared<std::vector<LaneMatchInfo>>();
  if (use_perception_match_) {
    laneOp_->Match(latest_lanes, local_map_, lane_matches);
  } else {
    laneOp_->Match(latest_lanes, local_map_, lane_matches,
                   use_bipartite_assoc_match_, sample_interval_);
  }
  for (auto& lane : local_map_->local_map_lane_) {
    lane.need_fit_ = false;
    for (auto match : *lane_matches) {
      if (match.map_lane_ == nullptr) continue;
      if (match.map_lane_->track_id_ == lane.track_id_) {
        lane.need_fit_ = true;
        break;
      }
    }
  }
  for (auto match : *lane_matches) {
    if (match.update_type_ == ObjUpdateType::ADD_NEW) {
      if (use_perception_match_) {
        mmgr_->AddNewLane(local_map_, *match.frame_lane_,
                          match.frame_lane_->track_id_);
      } else {
        mmgr_->AddNewLane(local_map_, *match.frame_lane_, sample_interval_);
      }
    } else if (match.update_type_ == ObjUpdateType::MERGE_OLD) {
      mmgr_->MergeOldLane(local_map_, *match.frame_lane_, *match.map_lane_,
                          sample_interval_);
    }
  }
  // for (auto& lane : local_map_->local_map_lane_) {
  //   if (lane.need_fit_ == false) {
  //     lane.lanepos_ = LanePositionType::OTHER;
  //   }
  // }
  // CommonUtil::CubicCurve(local_map, sample_interval_);
  CommonUtil::CatmullRom(local_map_, sample_interval_);
  mmgr_->CutLocalMap(local_map_, 150, 150);
  localmap_mutex_.lock();
  local_map_tmp_ = *local_map_;
  localmap_mutex_.unlock();
  if (use_rviz_ && RVIZ_AGENT.Ok()) {
    auto sec = static_cast<uint64_t>(msg->header().gnss_stamp());
    auto nsec = static_cast<uint64_t>((msg->header().gnss_stamp() - sec) * 1e9);
    CommonUtil::PubPercepPoints(T_W_V, latest_lanes, sec, nsec,
                                "/localmap/percep_points", sample_interval_);
    CommonUtil::PubMapPoints(*local_map_, sec, nsec, "/localmap/map_points",
                             T_W_V);
    CommonUtil::PubMapPointsMarker(*local_map_, sec, nsec,
                                   "/localmap/map_points/marker", T_W_V);
    CommonUtil::PubOriMapPoints(*local_map_, sec, nsec,
                                "/localmap/ori_map_points", T_W_V);
    CommonUtil::PubOdom(T_W_V, sec, nsec, "/localmap/odom");
    CommonUtil::PubTf(T_W_V, sec, nsec, "/localmap/tf");
    CommonUtil::PubPath(T_W_V, sec, nsec, "/localmap/path");
  }
  laneline_inited_ = true;
  auto end = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  HLOG_ERROR << "totle time: " << duration.count() << " ms";
}
void LMapApp::OnRoadEdge(
    const std::shared_ptr<const hozon::perception::TransportElement>& msg) {}

void LMapApp::OnImage(
    const std::shared_ptr<const hozon::soc::CompressedImage>& msg) {
  if (use_rviz_ && RVIZ_AGENT.Ok()) {
    CommonUtil::PubImage("/localmap/image", msg);
  }
}

bool LMapApp::FetchLocalMap(
    std::shared_ptr<hozon::mapping::LocalMap> local_map) {
  if (!dr_inited_) {
    HLOG_INFO << "==========dr_inited_ failed=======";
    return false;
  }
  if (!laneline_inited_) {
    HLOG_INFO << "==========laneline_inited_ failed=======";
    return false;
  }
  localmap_mutex_.lock();
  auto local_map_msg = local_map_tmp_;
  localmap_mutex_.unlock();
  static int seq = 0;
  local_map->mutable_header()->set_seq(seq++);
  static double init_timestamp = local_map_msg.timestamp;
  local_map->set_init_timestamp(init_timestamp);
  local_map->mutable_header()->set_gnss_stamp(local_map_msg.timestamp);
  local_map->mutable_header()->set_publish_stamp(local_map_msg.timestamp);
  for (size_t i = 0; i < local_map_msg.local_map_lane_.size(); ++i) {
    auto lane = local_map->add_lanes();
    hozon::mapping::LanePositionType lanepos;
    DataConvert::ConvertInnerLanePos(local_map_msg.local_map_lane_[i].lanepos_,
                                     &lanepos);
    lane->set_lanepos(lanepos);
    hozon::mapping::LaneType lanetype;
    DataConvert::ConvertInnerLaneType(
        local_map_msg.local_map_lane_[i].lanetype_, &lanetype);
    lane->set_lanetype(lanetype);
    lane->set_track_id(local_map_msg.local_map_lane_[i].track_id_);
    for (auto& point : local_map_msg.local_map_lane_[i].fit_points_) {
      auto p = lane->add_points();
      p->set_x(point.x());
      p->set_y(point.y());
      p->set_z(point.z());
    }
  }
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
      // HLOG_ERROR << "Dr time: " << std::setprecision(20) << iter->first;
      break;
    }
  }

  if (iter == dr_list.rbegin()) {
    if (timestamp - dr_list.back().second->timestamp > 0.2) {
      HLOG_ERROR << "Dr delay:" << timestamp - dr_list.back().second->timestamp;
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
