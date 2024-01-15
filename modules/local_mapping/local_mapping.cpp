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
    CommonUtil::PubPerLaneLine(T_W_V, perception.lane_lines_, sec, nsec,
                               "/localmap/per_lane_line");
    CommonUtil::PubPerEdgeLine(T_W_V, perception.edge_lines_, sec, nsec,
                               "/localmap/per_edge_line");
    CommonUtil::PubPerStopLine(T_W_V, perception.stop_lines_, sec, nsec,
                               "/localmap/per_stop_line");
    CommonUtil::PubPerArrow(T_W_V, perception.arrows_, sec, nsec,
                            "/localmap/per_arrow");
    CommonUtil::PubPerZebraCrossing(T_W_V, perception.zebra_crossings_, sec,
                                    nsec, "/localmap/per_zebra_crossing");
    CommonUtil::PubMapLaneLine(T_W_V, local_map.lane_lines_, sec, nsec,
                               "/localmap/map_lane_line");
    CommonUtil::PubMapEdgeLine(T_W_V, local_map.edge_lines_, sec, nsec,
                               "/localmap/map_edge_line");
    CommonUtil::PubMapStopLine(T_W_V, local_map.stop_lines_, sec, nsec,
                               "/localmap/map_stop_line");
    CommonUtil::PubMapArrow(T_W_V, local_map.arrows_, sec, nsec,
                            "/localmap/map_arrow");
    CommonUtil::PubMapZebraCrossing(T_W_V, local_map.zebra_crossings_, sec,
                                    nsec, "/localmap/map_zebra_crossing");
    CommonUtil::PubMapLaneLineMarker(T_W_V, local_map.lane_lines_, sec, nsec,
                                     "/localmap/map_lane_line/marker");
    CommonUtil::PubImmatureMapLaneLine(T_W_V, local_map.lane_lines_, sec, nsec,
                                       "/localmap/immature_map_lane_line");
    CommonUtil::PubImmatureMapLaneLineMarker(
        T_W_V, local_map.lane_lines_, sec, nsec,
        "/localmap/immature_map_lane_line/marker");
    CommonUtil::PubMapLaneLineControl(T_W_V, local_map, sec, nsec,
                                      "/localmap/map_lane_line/control");
    CommonUtil::PubMapLane(T_W_V, local_map, sec, nsec, "/localmap/map_lane");
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
  HLOG_INFO << "latest_localization buffer size: "
            << local_data.dr_buffer().buffer_size();
  HLOG_INFO << "latest_localization timestamp: " << SET_PRECISION(20)
            << latest_localization->timestamp_;
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
    HLOG_ERROR << "localization timestamp error";
  }
  if (!localization_inited_) {
    localization_inited_ = true;
  }
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
  if (!ins_inited_) {
    ins_inited_ = true;
  }
}

void LMapApp::OnPerception(
    const std::shared_ptr<const hozon::perception::TransportElement>& msg) {
  util::TicToc global_tic;
  global_tic.Tic();
  std::shared_ptr<Perception> perception = std::make_shared<Perception>();
  DataConvert::SetPerception(*msg, perception.get());
  HLOG_INFO << "perception->timestamp: " << SET_PRECISION(20)
            << perception->timestamp_;
  ConstDrDataPtr perception_pose =
      laneOp_->GetDrPoseForTime(perception->timestamp_);
  if (perception_pose == nullptr) {
    HLOG_ERROR << "perception_pose is nullptr";
    return;
  }
  static Sophus::SE3d last_T_W_V;
  Sophus::SE3d T_W_V =
      Sophus::SE3d(perception_pose->quaternion, perception_pose->pose);
  Sophus::SE3d T_C_L = T_W_V.inverse() * last_T_W_V;
  last_T_W_V = T_W_V;
  mmgr_->UpdatePerception(local_map_ptr_.get(), T_C_L);
  local_map_ptr_->timestamp = perception->timestamp_;
  std::vector<LaneLineMatchInfo> lane_line_matches;
  std::vector<EdgeLineMatchInfo> edge_line_matches;
  std::vector<StopLineMatchInfo> stop_line_matches;
  std::vector<ArrowMatchInfo> arrow_matches;
  std::vector<ZebraCrossingMatchInfo> zebra_crossing_matches;
  laneOp_->MatchLaneLine(perception->lane_lines_, local_map_ptr_->lane_lines_,
                         &lane_line_matches);
  laneOp_->MatchEdgeLine(perception->edge_lines_, local_map_ptr_->edge_lines_,
                         &edge_line_matches);
  laneOp_->MatchStopLine(perception->stop_lines_, local_map_ptr_->stop_lines_,
                         &stop_line_matches);
  laneOp_->MatchArrow(perception->arrows_, local_map_ptr_->arrows_,
                      &arrow_matches);
  laneOp_->MatchZebraCrossing(perception->zebra_crossings_,
                              local_map_ptr_->zebra_crossings_,
                              &zebra_crossing_matches);
  for (const auto& match : lane_line_matches) {
    if (match.update_type_ == ObjUpdateType::ADD_NEW) {
      mmgr_->AddNewLaneLine(local_map_ptr_.get(), match.per_lane_line_);
    } else if (match.update_type_ == ObjUpdateType::MERGE_OLD) {
      mmgr_->MergeOldLaneLine(local_map_ptr_.get(), match.per_lane_line_,
                              match.map_lane_line_);
    }
  }
  for (const auto& match : edge_line_matches) {
    if (match.update_type_ == ObjUpdateType::ADD_NEW) {
      mmgr_->AddNewEdgeLine(local_map_ptr_.get(), match.per_edge_line_);
    } else if (match.update_type_ == ObjUpdateType::MERGE_OLD) {
      mmgr_->MergeOldEdgeLine(local_map_ptr_.get(), match.per_edge_line_,
                              match.map_edge_line_);
    }
  }
  for (const auto& match : stop_line_matches) {
    if (match.update_type_ == ObjUpdateType::ADD_NEW) {
      mmgr_->AddNewStopLine(local_map_ptr_.get(), match.per_stop_line_);
    } else if (match.update_type_ == ObjUpdateType::MERGE_OLD) {
      mmgr_->MergeOldStopLine(local_map_ptr_.get(), match.per_stop_line_,
                              match.map_stop_line_);
    }
  }
  for (const auto& match : arrow_matches) {
    if (match.update_type_ == ObjUpdateType::ADD_NEW) {
      mmgr_->AddNewArrow(local_map_ptr_.get(), match.per_arrow_);
    } else if (match.update_type_ == ObjUpdateType::MERGE_OLD) {
      mmgr_->MergeOldArrow(local_map_ptr_.get(), match.per_arrow_,
                           match.map_arrow_);
    }
  }
  for (const auto& match : zebra_crossing_matches) {
    if (match.update_type_ == ObjUpdateType::ADD_NEW) {
      mmgr_->AddNewZebraCrossing(local_map_ptr_.get(),
                                 match.per_zebra_crossing_);
    } else if (match.update_type_ == ObjUpdateType::MERGE_OLD) {
      mmgr_->MergeOldZebraCrossing(local_map_ptr_.get(),
                                   match.per_zebra_crossing_,
                                   match.map_zebra_crossing_);
    }
  }
  for (auto& lane_line : local_map_ptr_->lane_lines_) {
    MapManager::MergeMapLeftRight(local_map_ptr_.get(), &lane_line);
    MapManager::MergeMapFrontBack(local_map_ptr_.get(), &lane_line);
  }
  mmgr_->UpdateLanepos(local_map_ptr_.get());
  // mmgr_->MapMerge(local_map_ptr_.get());
  CommonUtil::FitLaneLines(&local_map_ptr_->lane_lines_);
  CommonUtil::FitEdgeLines(&local_map_ptr_->edge_lines_);
  mmgr_->CutLocalMap(local_map_ptr_.get(), 150, 150);
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
  if (!perception_inited_) {
    perception_inited_ = true;
  }
  HLOG_INFO << "on perception cost " << global_tic.Toc() << "ms";
}

bool LMapApp::FetchLocalMap(
    const std::shared_ptr<hozon::mapping::LocalMap>& local_map) {
  if (!localization_inited_) {
    HLOG_ERROR << "==========localization_inited_ failed=======";
    return false;
  }
  if (!perception_inited_) {
    HLOG_ERROR << "==========perception_inited_ failed=======";
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
    if (!lane_line_msg.ismature_) {
      continue;
    }
    hozon::mapping::LaneType lanetype =
        hozon::mapping::LaneType::LaneType_UNKNOWN;
    DataConvert::ConvertInnerLaneType(lane_line_msg.lanetype_, &lanetype);
    hozon::mapping::LanePositionType lanepos =
        hozon::mapping::LanePositionType::LanePositionType_OTHER;
    DataConvert::ConvertInnerLanePos(lane_line_msg.lanepos_, &lanepos);
    // hozon::mapping::Color color = hozon::mapping::Color::UNKNOWN;
    // DataConvert::ConvertInnerColor(lane_line_msg.color_, &color);
    auto* lane_line = local_map->add_lane_lines();
    lane_line->set_track_id(lane_line_msg.track_id_);
    lane_line->set_lanetype(lanetype);
    lane_line->set_lanepos(lanepos);
    // lane_line->set_color(color);
    for (auto point_msg : lane_line_msg.fit_points_) {
      auto* point = lane_line->add_points();
      point->set_x(point_msg.x());
      point->set_y(point_msg.y());
      point->set_z(point_msg.z());
    }
  }
  for (const auto& edge_line_msg : local_map_msg.edge_lines_) {
    if (!edge_line_msg.ismature_) {
      continue;
    }
    auto* edge_line = local_map->add_edge_lines();
    edge_line->set_track_id(edge_line_msg.track_id_);
    hozon::mapping::LanePositionType lanepos =
        hozon::mapping::LanePositionType::LanePositionType_OTHER;
    DataConvert::ConvertInnerLanePos(edge_line_msg.lanepos_, &lanepos);
    edge_line->set_lanepos(lanepos);
    for (auto point_msg : edge_line_msg.fit_points_) {
      auto* point = edge_line->add_points();
      point->set_x(point_msg.x());
      point->set_y(point_msg.y());
      point->set_z(point_msg.z());
    }
  }
  for (const auto& lane_msg : local_map_msg.map_lanes_) {
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
  for (const auto& stop_line_msg : local_map_msg.stop_lines_) {
    if (!stop_line_msg.ismature_) {
      continue;
    }
    auto* stop_line = local_map->add_stop_lines();
    stop_line->mutable_left_point()->set_x(stop_line_msg.left_point_.x());
    stop_line->mutable_left_point()->set_y(stop_line_msg.left_point_.y());
    stop_line->mutable_left_point()->set_z(0);
    stop_line->mutable_right_point()->set_x(stop_line_msg.right_point_.x());
    stop_line->mutable_right_point()->set_y(stop_line_msg.right_point_.y());
    stop_line->mutable_right_point()->set_z(0);
    stop_line->set_track_id(stop_line_msg.track_id_);
  }
  for (const auto& arrow_msg : local_map_msg.arrows_) {
    if (!arrow_msg.ismature_ ||
        !CommonUtil::IsConvex(arrow_msg.points_.points_)) {
      continue;
    }
    auto* arrow = local_map->add_arrows();
    arrow->set_track_id(arrow_msg.track_id_);
    arrow->set_heading(0);
    for (const auto& point_msg : arrow_msg.points_.points_) {
      auto* point = arrow->mutable_points()->add_point();
      point->set_x(point_msg.x());
      point->set_y(point_msg.y());
      point->set_z(point_msg.z());
    }
    // hozon::hdmap::ArrowData::Type arrowtype =
    //     hozon::hdmap::ArrowData::Type::ArrowData_Type_UNKNOWN_TURN;
    // DataConvert::ConvertInnerArrowType(arrow_msg.type_, &arrowtype);
    // arrow->set_arrow_type(arrowtype);
  }
  for (const auto& zebra_crossing_msg : local_map_msg.zebra_crossings_) {
    if (!zebra_crossing_msg.ismature_ ||
        !CommonUtil::IsConvex(zebra_crossing_msg.points_.points_)) {
      continue;
    }
    auto* zebra_crossing = local_map->add_cross_walks();
    zebra_crossing->set_track_id(zebra_crossing_msg.track_id_);
    for (const auto& point_msg : zebra_crossing_msg.points_.points_) {
      auto* point = zebra_crossing->mutable_points()->add_point();
      point->set_x(point_msg.x());
      point->set_y(point_msg.y());
      point->set_z(point_msg.z());
    }
  }
  return true;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
