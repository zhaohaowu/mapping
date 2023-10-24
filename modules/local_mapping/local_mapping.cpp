/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-08-31
 *****************************************************************************/
#include "modules/local_mapping/local_mapping.h"

namespace hozon {
namespace mp {
namespace lm {

LMapApp::LMapApp(const std::string& config_file) {
  YAML::Node config = YAML::LoadFile(config_file);
  use_point_tracking_ = config["use_point_tracking"].as<bool>();
  use_perception_match_ = config["use_perception_match"].as<bool>();
  use_horizon_assoc_match_ = config["use_horizon_assoc_match"].as<bool>();
  use_rviz_ = config["use_rviz"].as<bool>();
  compute_error = config["compute_error"].as<bool>();
  sample_interval_ = config["sample_interval"].as<double>();
  laneOp_ = std::make_shared<LaneOp>();
  mmgr_ = std::make_shared<MapManager>();
  // load hdmap
  provider_ = std::make_shared<PriorProvider>();
  provider_->Init(config_file);
  auto map = provider_->GetPrior();
  // hdmap_ = std::make_shared<hozon::hdmap::HDMap>();
  // hdmap_->LoadMapFromProto(*map);
  auto& local_data = LocalDataSingleton::GetInstance();
}

void LMapApp::OnLocation(
    const std::shared_ptr<const hozon::localization::Localization>& msg) {
  Location latest_location;
  DataConvert::SetLocation(*msg, &latest_location);
}

void LMapApp::OnDr(
    const std::shared_ptr<const hozon::dead_reckoning::DeadReckoning>& msg) {
  Location latest_dr;
  DataConvert::SetDr(*msg, &latest_dr);
  static bool dr_flag = true;
  static Sophus::SE3d init_T;
  if (dr_flag) {
    dr_flag = false;
  }
  Sophus::SE3d latest_T =
      Sophus::SE3d(latest_dr.quaternion_, latest_dr.position_);
  Sophus::SE3d T_W_V = init_T.inverse() * latest_T;
  Eigen::Vector3d trans_pose = T_W_V.translation();
  Eigen::Quaterniond trans_quat = T_W_V.so3().unit_quaternion();
  auto& local_data = LocalDataSingleton::GetInstance();
  // HLOG_ERROR << "Dr buffer size: " <<
  // local_data.dr_buffer().buffer_size();

  // HLOG_ERROR << "Dr timestamp: " << std::setprecision(20)
  //            << latest_dr->timestamp_;
  if (local_data.dr_buffer().is_empty() ||
      local_data.dr_buffer().back()->timestamp < latest_dr.timestamp_) {
    DrDataPtr dr_data_ptr = std::make_shared<DrData>();
    dr_data_ptr->timestamp = latest_dr.timestamp_;
    dr_data_ptr->pose = trans_pose;
    dr_data_ptr->quaternion = trans_quat;
    dr_data_ptr->local_omg = latest_dr.angular_vrf_;
    dr_data_ptr->local_vel = latest_dr.linear_vrf_;
    local_data.dr_buffer().push_new_message(latest_dr.timestamp_, dr_data_ptr);
  } else {
    // HLOG_ERROR << "Dr timestamp error";
  }
  dr_inited_ = true;
}

void LMapApp::OnIns(
    const std::shared_ptr<const hozon::localization::HafNodeInfo>& msg) {
  Eigen::Vector3d p_G_V(msg->pos_gcj02().x(), msg->pos_gcj02().y(),
                        msg->pos_gcj02().z());
  Eigen::Quaterniond q_G_V(msg->quaternion().w(), msg->quaternion().x(),
                           msg->quaternion().y(), msg->quaternion().z());
  Sophus::SE3d T_G_V = Sophus::SE3d(q_G_V, p_G_V);
  static int ins_n = 0;
  ins_n++;
  if (use_rviz_ && RVIZ_AGENT.Ok() && ins_n == 100 && hdmap_ != nullptr) {
    ins_n = 0;
    if (!dr_inited_) {
      return;
    }
    auto sec = static_cast<uint64_t>(msg->header().gnss_stamp());
    auto nsec = static_cast<uint64_t>(
        (msg->header().gnss_stamp() - static_cast<double>(sec)) * 1e9);
    if (compute_error) {
      std::vector<Eigen::Vector3d> hq_pts;
      T_W_V_mutex_.lock();
      CommonUtil::PubHdMapPoints(T_G_V, T_W_V_, hdmap_, sec, nsec,
                                 "/localmap/hq_map_points", &hq_pts);
      T_W_V_mutex_.unlock();
      localmap_mutex_.lock();
      if (!hq_pts.empty() && !local_map_tmp_.lane_lines_.empty()) {
        // float error = Loss::Process(hq_pts, local_map_tmp_.lane_lines_);
      }
      localmap_mutex_.unlock();
    } else {
      T_W_V_mutex_.lock();
      CommonUtil::PubHdMapPoints(T_G_V, T_W_V_, hdmap_, sec, nsec,
                                 "/localmap/hq_map_points");
      T_W_V_mutex_.unlock();
    }
  }
}

void LMapApp::OnLaneLine(
    const std::shared_ptr<const hozon::perception::TransportElement>& msg) {
  // auto start = std::chrono::high_resolution_clock::now();
  std::shared_ptr<Perception> latest_lane_lines =
      std::make_shared<Perception>();
  if (use_point_tracking_) {
    DataConvert::SetLaneLine(*msg, latest_lane_lines.get());
  } else {
    DataConvert::SetLaneLine(*msg, latest_lane_lines.get(), sample_interval_);
  }
  // HLOG_ERROR << "LaneLine timestamp: " << std::setprecision(20)
  //            << latest_lane_lines->timestamp_;
  ConstDrDataPtr lane_line_pose =
      laneOp_->GetDrPoseForTime(latest_lane_lines->timestamp_);
  if (lane_line_pose == nullptr) {
    HLOG_ERROR << "lane_line_pose is nullptr";
    return;
  }
  Sophus::SE3d T_W_V =
      Sophus::SE3d(lane_line_pose->quaternion, lane_line_pose->pose);
  T_W_V_mutex_.lock();
  T_W_V_ = T_W_V;
  T_W_V_mutex_.unlock();
  static Sophus::SE3d last_T_W_V;
  Sophus::SE3d T_C_L = T_W_V.inverse() * last_T_W_V;
  last_T_W_V = T_W_V;
  mmgr_->UpdateLaneLine(local_map_.get(), T_C_L);
  local_map_->timestamp = latest_lane_lines->timestamp_;
  std::vector<LaneMatchInfo> lane_line_matches;
  if (use_perception_match_) {
    laneOp_->Match(*latest_lane_lines, *local_map_, &lane_line_matches);
  } else {
    laneOp_->Match(*latest_lane_lines, *local_map_, &lane_line_matches,
                   use_horizon_assoc_match_);
  }
  for (auto& lane_line : local_map_->lane_lines_) {
    lane_line.need_fit_ = false;
    for (const auto& match : lane_line_matches) {
      if (match.map_lane_line_ == nullptr) {
        continue;
      }
      if (match.map_lane_line_->track_id_ == lane_line.track_id_) {
        lane_line.need_fit_ = true;
        break;
      }
    }
  }
  for (const auto& match : lane_line_matches) {
    if (match.update_type_ == ObjUpdateType::ADD_NEW) {
      mmgr_->AddNewLaneLine(local_map_.get(), *match.frame_lane_line_,
                            use_perception_match_);
    } else if (match.update_type_ == ObjUpdateType::MERGE_OLD) {
      mmgr_->MergeOldLaneLine(local_map_.get(), *match.frame_lane_line_,
                              *match.map_lane_line_);
    }
  }
  if (!use_perception_match_) {
    laneOp_->MergeMapLeftRight(local_map_.get());
    laneOp_->MergeMapFrontBack(local_map_.get());
  }
  for (auto& lane_line : local_map_->lane_lines_) {
    if (!lane_line.need_fit_) {
      lane_line.lanepos_ = LanePositionType::OTHER;
    }
  }
  CommonUtil::FitLocalMap(local_map_.get());
  mmgr_->CutLocalMap(local_map_.get(), 150, 150);
  // add Lane
  mmgr_->UpdateLane(local_map_.get(), *latest_lane_lines);
  localmap_mutex_.lock();
  local_map_tmp_ = *local_map_;
  localmap_mutex_.unlock();
  // auto start2 = std::chrono::high_resolution_clock::now();
  if (use_rviz_ && RVIZ_AGENT.Ok()) {
    auto sec = static_cast<uint64_t>(latest_lane_lines->timestamp_);
    auto nsec = static_cast<uint64_t>(
        (latest_lane_lines->timestamp_ - static_cast<double>(sec)) * 1e9);
    CommonUtil::PubPercepPoints(T_W_V, *latest_lane_lines, sec, nsec,
                                "/localmap/percep_points");
    CommonUtil::PubMapPoints(*local_map_, sec, nsec, "/localmap/map_points",
                             T_W_V);
    CommonUtil::PubMapPointsMarker(*local_map_, sec, nsec,
                                   "/localmap/map_points/marker", T_W_V);
    CommonUtil::PubPercepPointsMarker(T_W_V, *latest_lane_lines, sec, nsec,
                                      "/localmap/percep_points/marker");
    CommonUtil::PubOriMapPoints(*local_map_, sec, nsec,
                                "/localmap/ori_map_points", T_W_V);
    CommonUtil::PubMapControlPoints(*local_map_, sec, nsec,
                                    "/localmap/map_control_points", T_W_V);
    CommonUtil::PubMapLane(*local_map_, sec, nsec, "/localmap/lane", T_W_V);
    CommonUtil::PubOdom(T_W_V, sec, nsec, "/localmap/odom");
    CommonUtil::PubTf(T_W_V, sec, nsec, "/localmap/tf");
    CommonUtil::PubPath(T_W_V, sec, nsec, "/localmap/path");
  }
  // auto end2 = std::chrono::high_resolution_clock::now();
  // auto duration2 =
  //     std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start2);
  // HLOG_ERROR << "rviz time: " << duration2.count() << " ms";
  laneline_inited_ = true;
  // auto end = std::chrono::high_resolution_clock::now();
  // auto duration =
  //     std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  // HLOG_ERROR << "totle time: " << duration.count() << " ms";
}
void LMapApp::OnRoadEdge(
    const std::shared_ptr<const hozon::perception::TransportElement>& msg) {}

void LMapApp::OnImage(
    const std::shared_ptr<const hozon::soc::CompressedImage>& msg) const {
  if (use_rviz_ && RVIZ_AGENT.Ok()) {
    CommonUtil::PubImage("/localmap/image", msg);
  }
}

bool LMapApp::FetchLocalMap(
    const std::shared_ptr<hozon::mapping::LocalMap>& local_map) {
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
  for (const auto& lane_line_msg : local_map_msg.lane_lines_) {
    hozon::mapping::LaneType lanetype =
        hozon::mapping::LaneType::LaneType_UNKNOWN;
    DataConvert::ConvertInnerLaneType(lane_line_msg.lanetype_, &lanetype);
    hozon::mapping::LanePositionType lanepos =
        hozon::mapping::LanePositionType::LanePositionType_OTHER;
    DataConvert::ConvertInnerLanePos(lane_line_msg.lanepos_, &lanepos);
    auto* lane_line = local_map->add_lane_lines();
    lane_line->set_track_id(lane_line_msg.track_id_);
    lane_line->set_lanetype(lanetype);
    lane_line->set_lanepos(lanepos);
    for (auto point_msg : lane_line_msg.fit_points_) {
      auto* point = lane_line->add_points();
      point->set_x(point_msg.x());
      point->set_y(point_msg.y());
      point->set_z(point_msg.z());
    }
  }
  for (const auto& lane_msg : local_map_msg.lanes_) {
    hozon::mapping::LaneType left_lanetype =
        hozon::mapping::LaneType::LaneType_UNKNOWN;
    DataConvert::ConvertInnerLaneType(lane_msg.left_line_.lanetype_,
                                      &left_lanetype);
    hozon::mapping::LanePositionType left_lanepos =
        hozon::mapping::LanePositionType::LanePositionType_OTHER;
    DataConvert::ConvertInnerLanePos(lane_msg.left_line_.lanepos_,
                                     &left_lanepos);
    hozon::mapping::LaneType right_lanetype =
        hozon::mapping::LaneType::LaneType_UNKNOWN;
    DataConvert::ConvertInnerLaneType(lane_msg.right_line_.lanetype_,
                                      &right_lanetype);
    hozon::mapping::LanePositionType right_lanepos =
        hozon::mapping::LanePositionType::LanePositionType_OTHER;
    DataConvert::ConvertInnerLanePos(lane_msg.right_line_.lanepos_,
                                     &right_lanepos);
    auto* lane = local_map->add_lanes();
    lane->set_lane_id(lane_msg.lane_id_);
    lane->set_width(lane_msg.width_);
    lane->mutable_left_line()->set_track_id(lane_msg.left_line_.track_id_);
    lane->mutable_left_line()->set_lanetype(left_lanetype);
    lane->mutable_left_line()->set_lanepos(left_lanepos);
    for (auto point_msg : lane_msg.left_line_.points_) {
      auto* point = lane->mutable_left_line()->add_points();
      point->set_x(point_msg.x());
      point->set_y(point_msg.y());
      point->set_z(point_msg.z());
    }
    lane->mutable_right_line()->set_track_id(lane_msg.right_line_.track_id_);
    lane->mutable_right_line()->set_lanetype(right_lanetype);
    lane->mutable_right_line()->set_lanepos(right_lanepos);
    for (auto point_msg : lane_msg.right_line_.points_) {
      auto* point = lane->mutable_right_line()->add_points();
      point->set_x(point_msg.x());
      point->set_y(point_msg.y());
      point->set_z(point_msg.z());
    }
    for (auto point_msg : lane_msg.center_line_.points_) {
      auto* point = lane->mutable_center_line()->add_points();
      point->set_x(point_msg.x());
      point->set_y(point_msg.y());
      point->set_z(point_msg.z());
    }
    lane->set_left_lane_id(lane_msg.left_lane_id_);
    lane->set_right_lane_id(lane_msg.right_lane_id_);
  }
  return true;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
