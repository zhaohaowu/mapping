/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-08-31
 *****************************************************************************/
#include "modules/local_mapping/local_mapping.h"

namespace hozon {
namespace mp {
namespace lm {

LMapApp::LMapApp(const std::string& mapping_path,
                 const std::string& config_file) {
  YAML::Node config = YAML::LoadFile(config_file);
  use_point_tracking_ = config["use_point_tracking"].as<bool>();
  use_perception_match_ = config["use_perception_match"].as<bool>();
  use_horizon_assoc_match_ = config["use_horizon_assoc_match"].as<bool>();
  map_file_ = config["map_file"].as<std::string>();
  use_rviz_ = config["use_rviz"].as<bool>();
  local_map_ptr_ = std::make_shared<LocalMap>();
  laneOp_ = std::make_shared<LaneOp>();
  mmgr_ = std::make_shared<MapManager>();
  // load city hdmap
  provider_ = std::make_shared<PriorProvider>();
  provider_->Init(mapping_path, config_file);
  auto map = provider_->GetPrior();
  if (map) {
    hdmap_ = std::make_shared<hozon::hdmap::HDMap>();
    hdmap_->LoadMapFromProto(*map);
  }
  if (use_rviz_ && RVIZ_AGENT.Ok()) {
    stop_rviz_thread_.store(false);
    rviz_thread_ = std::thread(&LMapApp::RvizFunc, this);
  }
}

LMapApp::~LMapApp() {
  if (use_rviz_ && RVIZ_AGENT.Ok() && rviz_thread_.joinable()) {
    stop_rviz_thread_.store(true);
    rviz_thread_.join();
  }
}

void LMapApp::RvizFunc() {
  while (!stop_rviz_thread_.load()) {
    T_mutex_.lock();
    Sophus::SE3d T_W_V = T_W_V_;
    Sophus::SE3d T_G_V = T_G_V_;
    T_mutex_.unlock();
    localmap_mutex_.lock();
    LocalMap local_map = local_map_output_;
    localmap_mutex_.unlock();
    perception_mutex_.lock();
    Perception perception = perception_;
    perception_mutex_.unlock();
    auto sec = static_cast<uint64_t>(perception.timestamp_);
    auto nsec = static_cast<uint64_t>(
        (perception.timestamp_ - static_cast<double>(sec)) * 1e9);
    static int rviz_count = 0;
    rviz_count++;
    if (rviz_count >= 10 && ins_inited_) {
      rviz_count = 0;
      if (hdmap_) {
        CommonUtil::PubHdMapPoints(T_G_V, T_W_V, hdmap_, sec, nsec,
                                   "/localmap/hd_map_points");
      }
      CommonUtil::PubHqMapPoints(T_G_V, T_W_V, sec, nsec,
                                 "/localmap/hq_map_points");
    }
    CommonUtil::PubPercepPoints(T_W_V, perception, sec, nsec,
                                "/localmap/percep_points");
    CommonUtil::PubMapPoints(local_map, sec, nsec, "/localmap/map_points",
                             T_W_V);
    CommonUtil::PubMapPointsMarker(local_map, sec, nsec,
                                   "/localmap/map_points/marker", T_W_V);
    CommonUtil::PubPercepPointsMarker(T_W_V, perception, sec, nsec,
                                      "/localmap/percep_points/marker");
    CommonUtil::PubOriMapPoints(local_map, sec, nsec,
                                "/localmap/ori_map_points", T_W_V);
    CommonUtil::PubMapControlPoints(local_map, sec, nsec,
                                    "/localmap/map_control_points", T_W_V);
    CommonUtil::PubLane(local_map, sec, nsec, "/localmap/lane", T_W_V);
    CommonUtil::PubMapLane(local_map, sec, nsec, "/localmap/map_lane", T_W_V);
    CommonUtil::PubOdom(T_W_V, sec, nsec, "/localmap/odom");
    CommonUtil::PubTf(T_W_V, sec, nsec, "/localmap/tf");
    CommonUtil::PubPath(T_W_V, sec, nsec, "/localmap/path");
    usleep(100 * 1e3);
  }
}

void LMapApp::OnLocalization(
    const std::shared_ptr<const hozon::localization::Localization>& msg) {
  std::shared_ptr<Localization> latest_localization =
      std::make_shared<Localization>();
  DataConvert::SetLocalization(*msg, latest_localization.get());
  Sophus::SE3d T_W_V_localization = Sophus::SE3d(
      latest_localization->quaternion_, latest_localization->position_);
  Eigen::Vector3d trans_pose = T_W_V_localization.translation();
  Eigen::Quaterniond trans_quat = T_W_V_localization.so3().unit_quaternion();
  auto& local_data = LocalDataSingleton::GetInstance();
  // HLOG_ERROR << "latest_localization buffer size: "
  //            << local_data.dr_buffer().buffer_size();

  // HLOG_ERROR << "latest_localization timestamp: " << std::setprecision(20)
  //            << latest_localization->timestamp_;
  if (local_data.dr_buffer().is_empty() ||
      local_data.dr_buffer().back()->timestamp <
          latest_localization->timestamp_) {
    DrDataPtr dr_data_ptr = std::make_shared<DrData>();
    dr_data_ptr->timestamp = latest_localization->timestamp_;
    dr_data_ptr->pose = trans_pose;
    dr_data_ptr->quaternion = trans_quat;
    dr_data_ptr->local_omg = latest_localization->angular_vrf_;
    dr_data_ptr->local_vel = latest_localization->linear_vrf_;
    local_data.dr_buffer().push_new_message(latest_localization->timestamp_,
                                            dr_data_ptr);
  } else {
    // HLOG_ERROR << "Dr timestamp error";
  }
  localization_inited_ = true;
}

void LMapApp::OnDr(
    const std::shared_ptr<const hozon::dead_reckoning::DeadReckoning>& msg) {
  std::shared_ptr<Localization> latest_dr = std::make_shared<Localization>();
  DataConvert::SetDr(*msg, latest_dr.get());
  static bool dr_flag = true;
  static Sophus::SE3d init_T_inverse;
  if (dr_flag) {
    dr_flag = false;
    Sophus::SE3d init_T =
        Sophus::SE3d(latest_dr->quaternion_, latest_dr->position_);
    init_T_inverse = init_T.inverse();
  }
  Sophus::SE3d latest_T =
      Sophus::SE3d(latest_dr->quaternion_, latest_dr->position_);
  Sophus::SE3d T_W_V_dr = init_T_inverse * latest_T;
  Eigen::Vector3d trans_pose = T_W_V_dr.translation();
  Eigen::Quaterniond trans_quat = T_W_V_dr.so3().unit_quaternion();
  auto& local_data = LocalDataSingleton::GetInstance();
  // HLOG_ERROR << "Dr buffer size: " <<
  // local_data.dr_buffer().buffer_size();

  // HLOG_ERROR << "Dr timestamp: " << std::setprecision(20)
  //            << latest_dr->timestamp_;
  if (local_data.dr_buffer().is_empty() ||
      local_data.dr_buffer().back()->timestamp < latest_dr->timestamp_) {
    DrDataPtr dr_data_ptr = std::make_shared<DrData>();
    dr_data_ptr->timestamp = latest_dr->timestamp_;
    dr_data_ptr->pose = trans_pose;
    dr_data_ptr->quaternion = trans_quat;
    dr_data_ptr->local_omg = latest_dr->angular_vrf_;
    dr_data_ptr->local_vel = latest_dr->linear_vrf_;
    local_data.dr_buffer().push_new_message(latest_dr->timestamp_, dr_data_ptr);
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
  T_mutex_.lock();
  T_G_V_ = T_G_V;
  T_mutex_.unlock();
  ins_inited_ = true;
}

void LMapApp::OnLaneLine(
    const std::shared_ptr<const hozon::perception::TransportElement>& msg) {
  // auto start = std::chrono::high_resolution_clock::now();
  static Sophus::SE3d last_T_W_V;
  std::shared_ptr<Perception> perception = std::make_shared<Perception>();
  if (use_point_tracking_) {
    DataConvert::SetLaneLinePoint(*msg, perception.get());
  } else {
    DataConvert::SetLaneLine(*msg, perception.get());
  }
  // HLOG_ERROR << "LaneLine timestamp: " << std::setprecision(20)
  //            << perception->timestamp_;
  ConstDrDataPtr lane_line_pose =
      laneOp_->GetDrPoseForTime(perception->timestamp_);
  if (lane_line_pose == nullptr) {
    HLOG_ERROR << "lane_line_pose is nullptr";
    return;
  }
  Sophus::SE3d T_W_V =
      Sophus::SE3d(lane_line_pose->quaternion, lane_line_pose->pose);
  Sophus::SE3d T_C_L = T_W_V.inverse() * last_T_W_V;
  last_T_W_V = T_W_V;
  mmgr_->UpdateLaneLine(local_map_ptr_.get(), T_C_L);
  local_map_ptr_->timestamp = perception->timestamp_;
  std::vector<LaneMatchInfo> lane_line_matches;
  if (use_perception_match_) {
    laneOp_->Match(*perception, *local_map_ptr_, &lane_line_matches);
  } else {
    laneOp_->Match(*perception, *local_map_ptr_, &lane_line_matches,
                   use_horizon_assoc_match_);
  }
  for (auto& lane_line : local_map_ptr_->lane_lines_) {
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
      mmgr_->AddNewLaneLine(local_map_ptr_.get(), *match.frame_lane_line_,
                            use_perception_match_);
    } else if (match.update_type_ == ObjUpdateType::MERGE_OLD) {
      mmgr_->MergeOldLaneLine(local_map_ptr_.get(), *match.frame_lane_line_,
                              *match.map_lane_line_);
    }
  }
  laneOp_->MergeMapLeftRight(local_map_ptr_.get());
  laneOp_->MergeMapFrontBack(local_map_ptr_.get());
  for (auto& lane_line : local_map_ptr_->lane_lines_) {
    if (!lane_line.need_fit_) {
      if (!lane_line.ismature_) {
        lane_line.edge_laneline_count_--;
      }
      lane_line.lanepos_ = LanePositionType::OTHER;
    }
  }
  CommonUtil::FitLocalMap(local_map_ptr_.get());
  mmgr_->CutLocalMap(local_map_ptr_.get(), 150, 150);
  // add Lane
  mmgr_->UpdateLanepos(local_map_ptr_.get());
  mmgr_->UpdateLaneByPerception(local_map_ptr_.get(), *perception);
  mmgr_->UpdateLaneByLocalmap(local_map_ptr_.get());
  localmap_mutex_.lock();
  local_map_output_ = *local_map_ptr_;
  localmap_mutex_.unlock();
  perception_mutex_.lock();
  perception_ = *perception;
  perception_mutex_.unlock();
  T_mutex_.lock();
  T_W_V_ = T_W_V;
  T_mutex_.unlock();
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
  if (!localization_inited_) {
    HLOG_INFO << "==========localization_inited_ failed=======";
    return false;
  }
  if (!laneline_inited_) {
    HLOG_INFO << "==========laneline_inited_ failed=======";
    return false;
  }

  localmap_mutex_.lock();
  auto local_map_msg = local_map_output_;
  localmap_mutex_.unlock();
  static int seq = 0;
  local_map->mutable_header()->set_seq(seq++);
  static double init_timestamp = local_map_msg.timestamp;
  local_map->set_init_timestamp(init_timestamp);
  local_map->mutable_header()->set_gnss_stamp(local_map_msg.timestamp);
  local_map->mutable_header()->set_publish_stamp(local_map_msg.timestamp);
  local_map->mutable_header()->set_data_stamp(local_map_msg.timestamp);
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

bool LMapApp::FetchLocalMap(
    const std::shared_ptr<hozon::hdmap::Map>& local_map) {
  if (!localization_inited_) {
    HLOG_INFO << "==========localization_inited_ failed=======";
    return false;
  }
  if (!laneline_inited_) {
    HLOG_INFO << "==========laneline_inited_ failed=======";
    return false;
  }

  localmap_mutex_.lock();
  auto local_map_msg = local_map_output_;
  localmap_mutex_.unlock();
  static int seq = 0;
  local_map->mutable_header()->mutable_header()->set_seq(seq++);
  local_map->mutable_header()->mutable_header()->set_data_stamp(
      local_map_msg.timestamp);
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
    auto* lane = local_map->add_lane();
    lane->mutable_id()->set_id(std::to_string(lane_msg.lane_id_));
    auto* left_id = lane->add_left_neighbor_forward_lane_id();
    left_id->set_id(std::to_string(lane_msg.left_lane_id_));
    auto* right_id = lane->add_right_neighbor_forward_lane_id();
    right_id->set_id(std::to_string(lane_msg.right_lane_id_));
    auto* segment = lane->mutable_central_curve()->add_segment();
    auto* point = segment->mutable_line_segment()->add_point();
    // lane->mutable_left_line()->set_track_id(lane_msg.left_line_.track_id_);
    // lane->mutable_left_line()->set_lanetype(left_lanetype);
    // lane->mutable_left_line()->set_lanepos(left_lanepos);
    // for (auto point_msg : lane_msg.left_line_.points_) {
    //   auto* point = lane->mutable_left_line()->add_points();
    //   point->set_x(point_msg.x());
    //   point->set_y(point_msg.y());
    //   point->set_z(point_msg.z());
    // }
    // lane->mutable_right_line()->set_track_id(lane_msg.right_line_.track_id_);
    // lane->mutable_right_line()->set_lanetype(right_lanetype);
    // lane->mutable_right_line()->set_lanepos(right_lanepos);
    // for (auto point_msg : lane_msg.right_line_.points_) {
    //   auto* point = lane->mutable_right_line()->add_points();
    //   point->set_x(point_msg.x());
    //   point->set_y(point_msg.y());
    //   point->set_z(point_msg.z());
    // }
    // for (auto point_msg : lane_msg.center_line_.points_) {
    //   auto* point = lane->mutable_center_line()->add_points();
    //   point->set_x(point_msg.x());
    //   point->set_y(point_msg.y());
    //   point->set_z(point_msg.z());
    // }
    // lane->set_left_lane_id(lane_msg.left_lane_id_);
    // lane->set_right_lane_id(lane_msg.right_lane_id_);
  }
  return true;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
