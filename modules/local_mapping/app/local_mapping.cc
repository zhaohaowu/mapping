/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-08-31
 *****************************************************************************/
#include "modules/local_mapping/app/local_mapping.h"

#include <memory>
#include <string>
#include <utility>

// #include "Eigen/src/Core/Matrix.h"
#include "modules/local_mapping/base/location/dr.h"
#include "modules/local_mapping/lib/datalogger/map_manager.h"
#include "modules/local_mapping/lib/datalogger/pose_manager.h"
#include "perception-base/base/utils/log.h"
#include "perception-lib/lib/config_manager/config_manager.h"
#include "perception-lib/lib/io/file_util.h"

namespace hozon {
namespace mp {
namespace lm {

namespace perception_lib = hozon::perception::lib;

bool LocalMapApp::Init() {
  // localmap管理器初始化
  mmgr_ptr_ = std::make_shared<MapWorker>();
  CHECK(mmgr_ptr_->Init());
  HLOG_DEBUG << "map manager init successful...";

  // 配置rviz可视化
  const auto& config_manager = perception_lib::ConfigManager::Instance();
  const perception_lib::ModelConfig* model_config = nullptr;
  if (!config_manager->GetModelConfig(Name(), &model_config)) {
    HLOG_ERROR << "Parse config failed! Name: " << Name();
  }

  model_config->get_value("use_rviz", &use_rviz_);
  if (use_rviz_) {
    HLOG_DEBUG << "Start RvizAgent!!!";
    std::string rviz_addr;
    model_config->get_value("rviz_addr", &rviz_addr);
    int ret = RVIZ_AGENT.Init(rviz_addr);
    if (ret < 0) {
      HLOG_ERROR << "RvizAgent start failed";
    }
  }
  if (use_rviz_ && RVIZ_AGENT.Ok()) {
    stop_rviz_thread_ = false;
    rviz_thread_ = std::thread(&LocalMapApp::RvizFunc, this);
  }
  return true;
}

void LocalMapApp::OnLocalization(
    const LocationConstPtr& localization_frame_ptr) {
  // 用一个单例来存储一段时间内的dr原始定位数据。
  if (!POSE_MANAGER->PushDrData(localization_frame_ptr)) {
    HLOG_ERROR << "localization timestamp error";
  }
}

void LocalMapApp::OnIns(const InsDataConstPtr& ins_msg_ptr) {
  // 获取全局定位数据
  globle_transfor_mat_ = ins_msg_ptr->pose;
}

void LocalMapApp::OnPerception(
    const MeasurementFrameConstPtr& measurement_frame_ptr) {
  // localmap 局部建图的处理过程
  DrDataConstPtr perception_pose = POSE_MANAGER->GetDrPoseByTimeStamp(
      measurement_frame_ptr->header.timestamp);
  if (perception_pose == nullptr) {
    HLOG_ERROR << "perception time is:"
               << std::to_string(measurement_frame_ptr->header.timestamp);
    HLOG_ERROR << "perception_pose is nullptr";
    return;
  }
  POSE_MANAGER->PushLocalDrData(measurement_frame_ptr->header.timestamp,
                                perception_pose);
  mmgr_ptr_->Process(measurement_frame_ptr);
  if (use_rviz_ && RVIZ_AGENT.Ok()) {
    rviz_mutex_.lock();
    T_W_V_ = perception_pose->pose;
    perception_ = *measurement_frame_ptr;
    rviz_mutex_.unlock();
  }
}

LocalMapApp::~LocalMapApp() {
  if (use_rviz_ && RVIZ_AGENT.Ok() && rviz_thread_.joinable()) {
    stop_rviz_thread_ = true;
    rviz_thread_.join();
  }
}

void LocalMapApp::RvizFunc() {
  while (!stop_rviz_thread_) {
    rviz_mutex_.lock();
    Eigen::Affine3d T_W_V = T_W_V_;
    MeasurementFrame perception = perception_;
    LocalMapFramePtr local_map = nullptr;
    MANAGER_RVIZ_GET_MAP(local_map);
    rviz_mutex_.unlock();
    auto sec = static_cast<uint64_t>(perception.header.timestamp);
    auto nsec = static_cast<uint64_t>(
        (perception.header.timestamp - static_cast<double>(sec)) * 1e9);
    if (perception.lane_lines_ptr == nullptr ||
        perception.road_edges_ptr == nullptr ||
        local_map->lane_lines_ptr == nullptr ||
        local_map->road_edges_ptr == nullptr) {
      continue;
    }
    // 感知车道线
    std::vector<LaneLine> per_lane_lines;
    for (const auto& lane_line : perception.lane_lines_ptr->lanelines) {
      per_lane_lines.emplace_back(*lane_line);
    }
    RvizUtil::PubPerLaneLine(T_W_V, per_lane_lines, sec, nsec,
                             "/localmap/per_lane_line");
    // 感知路沿
    std::vector<RoadEdge> per_road_edges;
    for (const auto& road_edge : perception.road_edges_ptr->road_edges) {
      per_road_edges.emplace_back(*road_edge);
    }
    RvizUtil::PubPerRoadEdge(T_W_V, per_road_edges, sec, nsec,
                             "/localmap/per_road_edge");
    // 感知停止线
    std::vector<StopLine> per_stop_lines;
    for (const auto& stop_line : perception.stop_lines_ptr->stoplines) {
      per_stop_lines.emplace_back(*stop_line);
    }
    RvizUtil::PubPerStopLine(T_W_V, per_stop_lines, sec, nsec,
                             "/localmap/per_stop_line");
    // 感知箭头
    std::vector<Arrow> per_arrows;
    for (const auto& arrow : perception.road_arrows_ptr->arrows) {
      per_arrows.emplace_back(*arrow);
    }
    RvizUtil::PubPerArrow(T_W_V, per_arrows, sec, nsec, "/localmap/per_arrow");
    // 感知斑马线
    std::vector<ZebraCrossing> per_zebra_crossings;
    for (const auto& zebra_crossing :
         perception.zebra_crossings_ptr->zebra_crossings) {
      per_zebra_crossings.emplace_back(*zebra_crossing);
    }
    RvizUtil::PubPerZebraCrossing(T_W_V, per_zebra_crossings, sec, nsec,
                                  "/localmap/per_zebra_crossing");
    // 地图车道线
    std::vector<LaneLine> map_lane_lines;
    for (const auto& lane_line : local_map->lane_lines_ptr->lanelines) {
      map_lane_lines.emplace_back(*lane_line);
    }
    RvizUtil::PubMapLaneLine(T_W_V, map_lane_lines, sec, nsec,
                             "/localmap/map_lane_line");
    RvizUtil::PubMapLaneLineMarker(T_W_V, map_lane_lines, sec, nsec,
                                   "/localmap/map_lane_line_marker");
    RvizUtil::PubImmatureMapLaneLine(T_W_V, map_lane_lines, sec, nsec,
                                     "/localmap/immature_map_lane_line");
    RvizUtil::PubImmatureMapLaneLineMarker(
        T_W_V, map_lane_lines, sec, nsec,
        "/localmap/immature_map_lane_line_marker");
    // 地图路沿
    std::vector<RoadEdge> map_road_edges;
    for (const auto& road_edge : local_map->road_edges_ptr->road_edges) {
      map_road_edges.emplace_back(*road_edge);
    }
    RvizUtil::PubMapRoadEdge(T_W_V, map_road_edges, sec, nsec,
                             "/localmap/map_road_edge");
    RvizUtil::PubMapRoadEdgeMarker(T_W_V, map_road_edges, sec, nsec,
                                   "/localmap/map_road_edge_marker");
    RvizUtil::PubImmatureMapRoadEdge(T_W_V, map_road_edges, sec, nsec,
                                     "/localmap/immature_map_road_edge");
    RvizUtil::PubImmatureMapRoadEdgeMarker(
        T_W_V, map_road_edges, sec, nsec,
        "/localmap/immature_map_road_edge_marker");
    // 地图停止线
    std::vector<StopLine> map_stop_lines;
    for (const auto& stop_line : local_map->stop_lines_ptr->stoplines) {
      map_stop_lines.emplace_back(*stop_line);
    }
    RvizUtil::PubMapStopLine(T_W_V, map_stop_lines, sec, nsec,
                             "/localmap/map_stop_line");
    RvizUtil::PubImmatureMapStopLine(T_W_V, map_stop_lines, sec, nsec,
                                     "/localmap/immature_map_stop_line");
    RvizUtil::PubImmatureMapStopLineMarker(
        T_W_V, map_stop_lines, sec, nsec,
        "/localmap/immature_map_stop_line_marker");
    // 地图箭头
    std::vector<Arrow> map_arrows;
    for (const auto& arrow : local_map->road_arrows_ptr->arrows) {
      map_arrows.emplace_back(*arrow);
    }
    RvizUtil::PubMapArrow(T_W_V, map_arrows, sec, nsec, "/localmap/map_arrow");
    RvizUtil::PubImmatureMapArrow(T_W_V, map_arrows, sec, nsec,
                                  "/localmap/immature_map_arrow");
    RvizUtil::PubImmatureMapArrowMarker(T_W_V, map_arrows, sec, nsec,
                                        "/localmap/immature_map_arrow_marker");
    // 地图斑马线
    std::vector<ZebraCrossing> map_zebra_crossings;
    for (const auto& zebra_crossing :
         local_map->zebra_crossings_ptr->zebra_crossings) {
      map_zebra_crossings.emplace_back(*zebra_crossing);
    }
    RvizUtil::PubMapZebraCrossing(T_W_V, map_zebra_crossings, sec, nsec,
                                  "/localmap/map_zebra_crossing");
    RvizUtil::PubImmatureMapZebraCrossing(
        T_W_V, map_zebra_crossings, sec, nsec,
        "/localmap/immature_map_zebra_crossing");
    RvizUtil::PubImmatureMapZebraCrossingMarker(
        T_W_V, map_zebra_crossings, sec, nsec,
        "/localmap/immature_map_zebra_crossing_marker");
    RvizUtil::PubOdom(T_W_V, sec, nsec, "/localmap/odom");
    RvizUtil::PubTf(T_W_V, sec, nsec, "/localmap/tf");
    RvizUtil::PubPath(T_W_V, sec, nsec, "/localmap/path");
    usleep(100 * 1e3);
  }
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
