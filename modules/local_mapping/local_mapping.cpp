/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-08-31
 *****************************************************************************/
#include "modules/local_mapping/local_mapping.h"

DEFINE_uint32(zone, 51, "time zone");

namespace hozon {
namespace mp {
namespace lm {

LMapApp::LMapApp(const std::string& config_file) {
  YAML::Node config = YAML::LoadFile(config_file);
  use_perception_match_ = config["use_perception_match"].as<bool>();
  use_bipartite_assoc_match_ = config["use_bipartite_assoc_match"].as<bool>();
  use_rviz_ = config["use_rviz"].as<bool>();
  compute_error = config["compute_error"].as<bool>();
  sample_interval = config["sample_interval"].as<double>();
  BipartiteAssocParams params;
  params.min_match_point_num = config["min_match_point_num"].as<int>();
  params.min_overlap_point_num = config["min_overlap_point_num"].as<int>();
  params.same_group_max_dist = config["same_group_max_dist"].as<double>();
  params.match_score_threshold = config["match_score_threshold"].as<double>();
  laneOp_ = std::make_shared<LaneOp>();
  laneOp_->Init(params);
  mmgr_ = std::make_shared<MapManager>();
  init_T_ = Sophus::SE3d(Eigen::Matrix<double, 4, 4>::Identity());
  lasted_T_ = Sophus::SE3d(Eigen::Matrix<double, 4, 4>::Identity());
  T_G_V_ = Sophus::SE3d(Eigen::Matrix<double, 4, 4>::Identity());
  T_W_V_ = Sophus::SE3d(Eigen::Matrix<double, 4, 4>::Identity());
  laneline_inited_ = false;
  dr_inited_ = false;
  // load hdmap
  provider_ = std::make_shared<PriorProvider>();
  provider_->Init(config_file);
  auto map = provider_->GetPrior();
  hdmap_ = std::make_shared<hozon::hdmap::HDMap>();
  hdmap_->LoadMapFromProto(*map);
  mmgr_->Init();

  auto& local_data = LocalDataSingleton::GetInstance();
  local_data.Init();
}

void LMapApp::OnLocation(
    const std::shared_ptr<const hozon::localization::Localization>& msg) {
  std::shared_ptr<Location> latest_location_ = std::make_shared<Location>();
  DataConvert::SetLocation(*msg, latest_location_);
}

void LMapApp::OnDr(
    const std::shared_ptr<const hozon::dead_reckoning::DeadReckoning>& msg) {
  std::shared_ptr<Location> latest_dr_ = std::make_shared<Location>();
  DataConvert::SetDr(*msg, latest_dr_);
  static double map_init_timestamp_ = 0;
  if (map_init_timestamp_ < 1e-5) {
    map_init_timestamp_ = latest_dr_->timestamp_;
    init_T_ = Sophus::SE3d(latest_dr_->quaternion_, latest_dr_->position_);
  }
  lasted_T_ = Sophus::SE3d(latest_dr_->quaternion_, latest_dr_->position_);
  Sophus::SE3d T_W_V = init_T_.inverse() * lasted_T_;

  Eigen::Vector3d trans_pose = T_W_V.translation();
  Eigen::Quaterniond trans_quat = T_W_V.so3().unit_quaternion();

  auto& local_data = LocalDataSingleton::GetInstance();
  // HLOG_ERROR << "Dr buffer size: " <<
  // local_data.dr_data_buffer_.buffer_size(); HLOG_ERROR << "Dr timestamp: " <<
  // std::setprecision(20)
  //           << latest_dr_->timestamp_;
  if (local_data.dr_data_buffer_.is_empty() ||
      local_data.dr_data_buffer_.back()->timestamp < latest_dr_->timestamp_) {
    DrDataPtr dr_data_ptr = std::make_shared<DrData>();
    dr_data_ptr->timestamp = latest_dr_->timestamp_;
    dr_data_ptr->pose = trans_pose;
    dr_data_ptr->quaternion = trans_quat;
    dr_data_ptr->local_omg = latest_dr_->angular_vrf_;
    dr_data_ptr->local_vel = latest_dr_->linear_vrf_;
    local_data.dr_data_buffer_.push_new_message(latest_dr_->timestamp_,
                                                dr_data_ptr);
  } else {
    // HLOG_ERROR << "Dr timestamp error";
  }

  if (use_rviz_) {
    auto sec = static_cast<uint64_t>(msg->header().gnss_stamp());
    auto nsec = static_cast<uint64_t>((msg->header().gnss_stamp() - sec) * 1e9);
    CommonUtil::PubOdom(T_W_V_, sec, nsec, "/localmap/odom");
    CommonUtil::PubTf(T_W_V_, sec, nsec, "/localmap/tf");
    CommonUtil::PubPath(T_W_V_, sec, nsec, "/localmap/path");
  }
  dr_inited_ = true;
}

void LMapApp::OnIns(
    const std::shared_ptr<const hozon::localization::HafNodeInfo>& msg) {
  Eigen::Vector3d p_G_V(msg->pos_gcj02().x(), msg->pos_gcj02().y(),
                        msg->pos_gcj02().z());
  Eigen::Quaterniond q_G_V(msg->quaternion().w(), msg->quaternion().x(),
                           msg->quaternion().y(), msg->quaternion().z());
  T_G_V_ = Sophus::SE3d(q_G_V, p_G_V);
  static int ins_n = 0;
  ins_n++;
  if (use_rviz_ && ins_n == 100) {
    ins_n = 0;
    if (!dr_inited_) return;
    auto sec = static_cast<uint64_t>(msg->header().gnss_stamp());
    auto nsec = static_cast<uint64_t>((msg->header().gnss_stamp() - sec) * 1e9);

    if (compute_error) {
      std::vector<Eigen::Vector3d> hq_pts;
      CommonUtil::PubHdMapPoints(T_G_V_, T_W_V_, hdmap_, sec, nsec,
                                 "/localmap/hq_map_points", &hq_pts);
      if (hq_pts.size() && mmgr_->local_map_.local_map_lane_.size()) {
        float error = loss_.Process(hq_pts, mmgr_->local_map_.local_map_lane_);
      }
    } else {
      CommonUtil::PubHdMapPoints(T_G_V_, T_W_V_, hdmap_, sec, nsec,
                                 "/localmap/hq_map_points");
    }
  }
}

void LMapApp::OnLaneLine(
    const std::shared_ptr<const hozon::perception::TransportElement>& msg) {
  auto start = std::chrono::high_resolution_clock::now();
  if (!dr_inited_) {
    HLOG_ERROR << "dr_inited_ is failed";
    return;
  }
  mmgr_->UpdateTimestamp(msg->header().gnss_stamp());
  auto sec = static_cast<uint64_t>(msg->header().gnss_stamp());
  auto nsec = static_cast<uint64_t>((msg->header().gnss_stamp() - sec) * 1e9);
  std::shared_ptr<Lanes> latest_lanes = std::make_shared<Lanes>();
  DataConvert::SetLaneLine(*msg, latest_lanes);
  ConstDrDataPtr lane_pose = GetDrPoseForTime(latest_lanes->timestamp_);
  if (lane_pose == nullptr) {
    HLOG_ERROR << "lane_pose is nullptr";
    return;
  }
  Sophus::SE3d T_W_V = Sophus::SE3d(lane_pose->quaternion, lane_pose->pose);
  T_W_V_ = T_W_V;
  static Sophus::SE3d last_T_W_V =
      Sophus::SE3d(Eigen::Matrix<double, 4, 4>::Identity());
  Sophus::SE3d T_C_L = T_W_V.inverse() * last_T_W_V;
  last_T_W_V = T_W_V;
  auto start2 = std::chrono::high_resolution_clock::now();
  mmgr_->UpdateLane(T_C_L);
  auto end2 = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start2);
  // HLOG_ERROR << "update time: " << duration.count() << " ms";
  std::shared_ptr<std::vector<LocalMapLane>> map_lanes =
      std::make_shared<std::vector<LocalMapLane>>();
  mmgr_->GetLanes(map_lanes);
  // match current frame lanes to map lanes
  auto start3 = std::chrono::high_resolution_clock::now();
  std::shared_ptr<std::vector<LaneMatchInfo>> lane_matches =
      std::make_shared<std::vector<LaneMatchInfo>>();
  if (use_perception_match_) {
    laneOp_->Match(latest_lanes, map_lanes, lane_matches);
  } else {
    laneOp_->Match(latest_lanes, map_lanes, lane_matches,
                   use_bipartite_assoc_match_, sample_interval);
  }
  auto end3 = std::chrono::high_resolution_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(end3 - start3);
  // HLOG_ERROR << "match time: " << duration.count() << " ms";
  for (auto& map_lane : mmgr_->local_map_.local_map_lane_) {
    map_lane.need_fit_ = false;
    for (auto match : *lane_matches) {
      if (match.map_lane_ == nullptr) continue;
      if (match.map_lane_->track_id_ == map_lane.track_id_) {
        map_lane.need_fit_ = true;
        break;
      }
    }
  }
  auto start4 = std::chrono::high_resolution_clock::now();
  for (auto match : *lane_matches) {
    if (match.update_type_ == ObjUpdateType::ADD_NEW) {
      std::vector<Eigen::Vector3d> new_lane_pts;
      CommonUtil::SampleLanePoints(match.frame_lane_, &new_lane_pts,
                                   sample_interval);
      if (use_perception_match_) {
        mmgr_->CreateNewLane(new_lane_pts, match.frame_lane_->lane_id_);
        mmgr_->SetLaneProperty(match.frame_lane_->lane_id_, match.frame_lane_);
      } else {
        double lane_id = mmgr_->CreateNewLane(new_lane_pts);
        mmgr_->SetLaneProperty(lane_id, match.frame_lane_);
      }
    } else if (match.update_type_ == ObjUpdateType::MERGE_OLD) {
      int lane_start_x = 0;
      std::vector<Eigen::Vector3d> new_lane_pts;
      // laneOp_->FilterCurve(match.frame_lane_, &new_lane_pts, &lane_start_x,
      //                      sample_interval);
      double x, y;
      for (x = match.frame_lane_->x_start_vrf_;
           x <= match.frame_lane_->x_end_vrf_; x += sample_interval) {
        y = CommonUtil::f(
            x, match.frame_lane_->lane_fit_d_, match.frame_lane_->lane_fit_c_,
            match.frame_lane_->lane_fit_b_, match.frame_lane_->lane_fit_a_);
        Eigen::Vector3d pt(x, y, 0);
        new_lane_pts.emplace_back(pt);
      }
      if (new_lane_pts.size() == 0) {
        continue;
      }
      mmgr_->DeleteLanePoints(match.map_lane_->track_id_,
                              match.frame_lane_->x_start_vrf_);
      auto world_pts = std::make_shared<std::vector<Eigen::Vector3d>>();
      mmgr_->AppendOldLanePoints(match.map_lane_->track_id_, new_lane_pts);
      mmgr_->SetLaneProperty(match.map_lane_->track_id_, match.frame_lane_);
    }
  }
  auto end4 = std::chrono::high_resolution_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(end4 - start4);
  // HLOG_ERROR << "map lane deal time: " << duration.count() << " ms";
  auto start5 = std::chrono::high_resolution_clock::now();
  // CommonUtil::CubicCurve(&mmgr_->local_map_, sample_interval);
  CommonUtil::CatmullRoom(&mmgr_->local_map_, sample_interval);
  mmgr_->CutLocalMap(150, 150);
  localmap_mutex_.lock();
  local_map_tmp_ = mmgr_->local_map_;
  localmap_mutex_.unlock();
  auto end5 = std::chrono::high_resolution_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(end5 - start5);
  // HLOG_ERROR << "fit && cut time: " << duration.count() << " ms";
  auto start6 = std::chrono::high_resolution_clock::now();
  if (use_rviz_) {
    CommonUtil::PubPercepPoints(T_W_V_, latest_lanes, sec, nsec,
                                "/localmap/percep_points", sample_interval);
    CommonUtil::PubMapPoints(mmgr_->local_map_, sec, nsec,
                             "/localmap/map_points", T_W_V_);
    CommonUtil::PubMapPointsMarker(mmgr_->local_map_, sec, nsec,
                                   "/localmap/map_points/marker", T_W_V_);
    CommonUtil::PubOriMapPoints(mmgr_->local_map_, sec, nsec,
                                "/localmap/ori_map_points", T_W_V_);
  }
  auto end6 = std::chrono::high_resolution_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(end6 - start6);
  HLOG_ERROR << "rviz time: " << duration.count() << " ms";
  laneline_inited_ = 1;
  auto end = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  HLOG_ERROR << "totle time: " << duration.count() << " ms";
}
void LMapApp::OnRoadEdge(
    const std::shared_ptr<const hozon::perception::TransportElement>& msg) {
  // if (!dr_inited_) return;
  // auto sec_edge = static_cast<uint64_t>(msg->header().gnss_stamp());
  // auto nsec_edge =
  //     static_cast<uint64_t>((msg->header().gnss_stamp() - sec_edge) * 1e9);
  // std::shared_ptr<Lanes> latest_edges_ = std::make_shared<Lanes>();
  // DataConvert::SetEdgeLine(*msg, latest_edges_);
  // ConstDrDataPtr lane_pose_edge =
  // GetDrPoseForTime(latest_edges_->timestamp_); if (lane_pose_edge == nullptr)
  // {
  //   return;
  // }
  // Sophus::SE3d T_W_V_edge =
  //     Sophus::SE3d(lane_pose_edge->quaternion, lane_pose_edge->pose);
  // static Sophus::SE3d last_T_W_V_edge =
  //     Sophus::SE3d(Eigen::Matrix<double, 4, 4>::Identity());
  // Sophus::SE3d T_C_L_edge = T_W_V_edge.inverse() * last_T_W_V_edge;
  // last_T_W_V_edge = T_W_V_edge;
  // mmgr_->UpdateEdge(T_C_L_edge);
  // if (use_rviz_) {
  //   CommonUtil::PubEdgePoints(T_W_V_edge, latest_edges_, sec_edge, nsec_edge,
  //                             "/localmap/edge_points", sample_interval);
  // }
}

void LMapApp::OnImage(
    const std::shared_ptr<const hozon::soc::CompressedImage>& msg) {
  if (use_rviz_) {
    auto start = std::chrono::high_resolution_clock::now();
    // HLOG_ERROR << "rviz time: " << duration.count() << " ms";
    CommonUtil::PubImage("/localmap/image", msg);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    HLOG_ERROR << "===" << duration.count() << "ms";
  }
}

bool LMapApp::FetchLocalMap(
    std::shared_ptr<hozon::mapping::LocalMap> local_map) {
  auto start = std::chrono::high_resolution_clock::now();
  if (!dr_inited_) {
    HLOG_INFO << "==========dr_inited_ failed=======";
    return false;
  }
  if (!laneline_inited_) {
    HLOG_INFO << "==========laneline_inited_ failed=======";
    return false;
  }
  localmap_mutex_.lock();
  auto local_map_ = local_map_tmp_;
  localmap_mutex_.unlock();
  static int seq = 0;
  local_map->mutable_header()->set_seq(seq++);
  static double init_timestamp = local_map_.timestamp;
  local_map->set_init_timestamp(init_timestamp);
  local_map->mutable_header()->set_gnss_stamp(local_map_.timestamp);
  local_map->mutable_header()->set_publish_stamp(local_map_.timestamp);
  for (size_t i = 0; i < local_map_.local_map_lane_.size(); ++i) {
    auto lane = local_map->add_lanes();
    hozon::mapping::LanePositionType proto_lane_type;
    DataConvert::ConvertInnerLanePoseType(
        local_map_.local_map_lane_[i].pos_type_, &proto_lane_type);
    lane->set_lanepos(proto_lane_type);
    lane->set_track_id(local_map_.local_map_lane_[i].track_id_);
    for (auto& point : local_map_.local_map_lane_[i].fit_points_) {
      auto p = lane->add_points();
      p->set_x(point.x());
      p->set_y(point.y());
      p->set_z(point.z());
    }
  }
  auto end = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  // HLOG_ERROR << "pub time: " << duration.count() << " ms";
  return true;
}

bool LMapApp::FetchLocalMapLocation(
    std::shared_ptr<hozon::localization::Localization> local_map_location) {
  if (!dr_inited_) return false;
  if (!laneline_inited_) return false;
  local_map_location->mutable_header()->set_gnss_stamp(mmgr_->GetTimestamp());
  local_map_location->mutable_pose()->mutable_position()->set_x(
      T_W_V_.translation().x());
  local_map_location->mutable_pose()->mutable_position()->set_y(
      T_W_V_.translation().y());
  local_map_location->mutable_pose()->mutable_position()->set_z(
      T_W_V_.translation().z());
  Eigen::Quaterniond q(T_W_V_.so3().matrix());
  local_map_location->mutable_pose()->mutable_quaternion()->set_w(q.w());
  local_map_location->mutable_pose()->mutable_quaternion()->set_x(q.x());
  local_map_location->mutable_pose()->mutable_quaternion()->set_y(q.y());
  local_map_location->mutable_pose()->mutable_quaternion()->set_z(q.z());
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
