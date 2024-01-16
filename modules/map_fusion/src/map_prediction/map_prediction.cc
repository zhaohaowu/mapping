/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_prediction.cc
 *   author     ： zhangshuo
 *   date       ： 2023.09
 ******************************************************************************/

#include "map_fusion/map_prediction/map_prediction.h"
#include <bits/types/clock_t.h>
#include <gflags/gflags.h>
#include <pthread.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <future>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "boost/thread/exceptions.hpp"
#include "common/configs/config_gflags.h"
#include "common/math/line_segment2d.h"
#include "common/math/vec2d.h"
#include "common/time/clock.h"
#include "common/time/time.h"
#include "common/utm_projection/coordinate_convertor.h"
#include "map/hdmap/hdmap.h"
#include "map/hdmap/hdmap_common.h"
#include "map_fusion/map_fusion.h"
#include "map_fusion/map_service/map_table.h"
#include "modules/util/include/util/geo.h"

// #include "util/log.h"

#include "map_fusion/map_service/global_hd_map.h"
#include "opencv2/core/matx.hpp"
#include "proto/map/map.pb.h"
#include "proto/map/map_id.pb.h"
#include "proto/map/map_lane.pb.h"
#include "proto/map/map_road.pb.h"
#include "util/mapping_log.h"
#include "util/rate.h"
#include "util/rviz_agent/rviz_agent.h"
#include "util/tic_toc.h"

// NOLINTBEGIN
// DEFINE_bool(pred_run, true, "pred thread run");
// DEFINE_uint32(pred_thread_interval, 100, "pred thread interval ms");
DEFINE_bool(viz_odom_map_in_local, true,
            "whether publish viz msgs of odometry and map in local frame");
DEFINE_string(viz_topic_odom_in_local, "/mf/pred/odom_local",
              "viz topic of odometry in local frame");
DEFINE_string(viz_topic_map_in_local, "/mf/pred/map_local",
              "viz topic of map in local frame");
DEFINE_bool(output_hd_map, false,
            "Whether to output a map with a range of 300m");
// NOLINTEND

namespace hozon {
namespace mp {
namespace mf {

using Vec2d = common::math::Vec2d;

void SplitSecs(double secs, uint32_t* sec, uint32_t* nsec) {
  if (sec == nullptr || nsec == nullptr) {
    return;
  }
  if (secs < 0) {
    return;
  }
  auto s = static_cast<uint32_t>(secs);
  auto ns = static_cast<uint32_t>((secs - s) * 1e9);
  *sec = s;
  *nsec = ns;
}

int MapPrediction::Init() {
  topo_map_ = std::make_shared<hozon::hdmap::Map>();
  hq_map_server_ = std::make_shared<hozon::hdmap::HDMap>();
  local_msg_ = std::make_shared<hozon::hdmap::Map>();
  local_msg_->Clear();

  // is_pred_proc_stop_ = !FLAGS_pred_run;
  // pred_proc_ = std::async(&MapPrediction::Prediction, this);
  return 0;
}

void MapPrediction::Stop() {}

void MapPrediction::OnInsNodeInfo(
    const std::shared_ptr<hozon::localization::HafNodeInfo>& msg) {
  Eigen::Vector3d gcj02(msg->pos_gcj02().x(), msg->pos_gcj02().y(),
                        msg->pos_gcj02().z());
  uint32_t zone_u = std::floor(msg->pos_gcj02().y() / 6.0 + 31);
  int zone = static_cast<int>(zone_u);
  double x = msg->pos_gcj02().x();
  double y = msg->pos_gcj02().y();
  hozon::common::coordinate_convertor::GCS2UTM(zone, &y, &x);
  OnLocationInGlobal(zone, y, x);
}

void MapPrediction::OnLocalization(
    const std::shared_ptr<hozon::localization::Localization>& msg) {
  // 提取全局定位
  // gcj02
  Eigen::Vector3d pos_global(msg->pose().gcj02().x(), msg->pose().gcj02().y(),
                             msg->pose().gcj02().z());
  uint32_t zone_t = msg->pose().utm_zone_01();
  int zone = static_cast<int>(zone_t);
  OnLocationInGlobal(zone, msg->pose().pos_utm_01().x(),
                     msg->pose().pos_utm_01().y());

  stamp_loc_ = msg->header().gnss_stamp();
  pos_local_ << msg->pose_local().position().x(),
      msg->pose_local().position().y(), msg->pose_local().position().z();
  auto yaw = msg->pose_local().local_heading() * M_PI / 180.;
  // auto roll = msg->pose().euler_angles_local().x();
  // auto pitch = msg->pose().euler_angles_local().y();
  quat_local_ = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(0., Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(0., Eigen::Vector3d::UnitY());

  if (FLAGS_output_hd_map) {
    local_enu_center_ = pos_global;
    local_enu_center_flag_ = false;
  }

  if (local_enu_center_flag_) {
    HLOG_WARN
        << "local_enu_center_ not init yet, not update T_local_enu_to_local_";
    return;
  }

  Eigen::Isometry3d T_veh_to_local;
  T_veh_to_local.setIdentity();
  T_veh_to_local.rotate(quat_local_);
  T_veh_to_local.pretranslate(pos_local_);

  Eigen::Vector3d pos_local_enu =
      util::Geo::Gcj02ToEnu(pos_global, local_enu_center_);

  yaw = msg->pose().euler_angles().z();
  // roll = msg->pose().euler_angles().x();
  // pitch = msg->pose().euler_angles().y();
  Eigen::Quaterniond quat_enu =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0., Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(0., Eigen::Vector3d::UnitY());

  Eigen::Isometry3d T_veh_to_local_enu;
  T_veh_to_local_enu.setIdentity();
  T_veh_to_local_enu.rotate(quat_enu);
  T_veh_to_local_enu.pretranslate(pos_local_enu);

  T_local_enu_to_local_.setIdentity();
  T_local_enu_to_local_ = T_veh_to_local * T_veh_to_local_enu.inverse();
}

void MapPrediction::OnLocationInGlobal(int utm_zone_id, double utm_x,
                                       double utm_y) {
  std::lock_guard<std::mutex> lock(mtx_);
  location_utm_.x() = utm_x;
  location_utm_.y() = utm_y;
  utm_zone_ = utm_zone_id;
}

void MapPrediction::OnHqMap(const std::shared_ptr<hozon::hdmap::Map>& hqmap) {
  // std::lock_guard<std::mutex> lock(mtx_);
  hqmap_road_edge.clear();
  if (!hqmap) {
    return;
  }
  int ret = hq_map_server_->LoadMapFromProto(*hqmap);

  if (ret != 0 || hq_map_server_->Empty()) {
    return;
  }
}

void MapPrediction::OnTopoMap(
    const std::shared_ptr<hozon::hdmap::Map>& msg,
    const std::tuple<std::unordered_map<std::string, LaneInfo>,
                     std::unordered_map<std::string, RoadInfo>>& map_info) {
  // std::lock_guard<std::mutex> lock(mtx_);
  hq_map_ = std::make_shared<hozon::hdmap::Map>();
  if (!msg) {
    HLOG_ERROR << "nullptr topo map";
    return;
  }

  if (msg->lane().empty()) {
    HLOG_ERROR << "no OnTopoMap info!";
    return;
  }

  hozon::common::PointENU utm_pos;
  // uint32_t utm_zone = 0;
  {
    std::lock_guard<std::mutex> lock(mtx_);

    local_msg_ = std::make_shared<hozon::hdmap::Map>();
    local_msg_->Clear();
    local_msg_->CopyFrom(*msg);
    utm_pos.set_x(location_utm_.x());
    utm_pos.set_y(location_utm_.y());
    utm_pos.set_z(0);
    // utm_zone = utm_zone_;
  }

  //! 1. 根据当前位置找到300m所有lane
  //! ids，同步创建lane_id_hash和section_id_hash; lane_id_hash: {
  //!   "lane_id": {
  //!     enum FarSide {far_left, far_right, mid},
  //!     left_line: {pt, pt}
  //!     right_line: {pt, pt}
  //!     section_id
  //!     left_lane_ids: {lane_id, lane_id}
  //!     right_lane_ids: {lane_id, lane_id}
  //!     prev_lane_ids: {lane_id, lane_id}
  //!     next_lane_ids: {lane_id, lane_id}
  //!   }
  //! }
  //! section_id_hash: {
  //!   "section_id": {
  //!     lane_ids: {lane_id, lane_id}
  //!     left_boundary: {pt, pt}
  //!     right_boundary: {pt, pt}
  //! }
  //! }
  //! 2.
  //! 遍历topo_msg里每条lane，把lane_id存到topo_lane_id_vector里，
  // ！ 并且从lane_id_hash里找到对应section
  //! id，
  //!    存到topo_section_vector;
  //! 3. 遍历topo_msg里每条lane，判断next是否为空，如果为空把lane_id存到
  //! end_lane_id_vector，并且把所属section id存到end_section_vector;
  //! 4. 遍历topo_msg里每条lane，如果当前lane的next为空，从lane_id_hash找
  //!    到这个lane的next，递归所有next，把next所属section
  //!    id存到add_section_vector；
  //!
  {
    std::lock_guard<std::mutex> lock(mtx_);
    util::TicToc tic;
    util::TicToc local_tic;

    Clear();

    lane_table_ = std::get<0>(map_info);
    // 更新站心
    if (local_enu_center_flag_) {
      local_enu_center_ = lane_table_.begin()->second.ref_point;
      local_enu_center_flag_ = false;
    }
    road_table_ = std::get<1>(map_info);
    // all_section_ids_ = std::get<2>(map_info);

    // 可视化hd lane id
    // viz_map_.VizHDLaneID(lane_table_);

    // 可视化hd lane id
    // viz_map_.VizHDLaneID(lanes_in_range, local_enu_center_, utm_zone_);

    if (FLAGS_map_service_mode == 0) {
      auto ret = GLOBAL_HD_MAP->GetLocalMap(utm_pos, {300, 300}, hq_map_.get());
      if (ret != 0) {
        HLOG_ERROR << "GetLocalMap failed";
        return;
      }

      hq_map_->mutable_crosswalk()->Clear();
      hq_map_->mutable_stop_sign()->Clear();
      hq_map_->mutable_signal()->Clear();
      hq_map_->mutable_yield()->Clear();
      hq_map_->mutable_overlap()->Clear();
      hq_map_->mutable_clear_area()->Clear();
      hq_map_->mutable_speed_bump()->Clear();
      hq_map_->mutable_parking_space()->Clear();
      hq_map_->mutable_pnc_junction()->Clear();
      hq_map_->mutable_rsu()->Clear();
      hq_map_->mutable_arraw()->Clear();
      hq_map_->mutable_marker()->Clear();
      hq_map_->mutable_prohibited_area()->Clear();
      hq_map_->mutable_free_area()->Clear();
      hq_map_->mutable_pillar()->Clear();
      hq_map_->mutable_gate()->Clear();
      hq_map_->mutable_parking()->Clear();
      hq_map_->mutable_stop_line()->Clear();

      for (auto& hq_lane : *hq_map_->mutable_lane()) {
        for (auto& seg : *hq_lane.mutable_left_boundary()
                              ->mutable_curve()
                              ->mutable_segment()) {
          seg.mutable_line_segment()->clear_point();
        }
        for (auto& seg : *hq_lane.mutable_right_boundary()
                              ->mutable_curve()
                              ->mutable_segment()) {
          seg.mutable_line_segment()->clear_point();
        }
        for (auto& seg : *hq_lane.mutable_central_curve()->mutable_segment()) {
          seg.mutable_line_segment()->clear_point();
        }
      }
    } else if (FLAGS_map_service_mode == 1) {
      GLOBAL_HD_MAP->GetMapWithoutLaneGeometry(hq_map_.get());
    }

    hq_map_->mutable_header()->CopyFrom(msg->header());
    hq_map_->mutable_header()->mutable_id()->clear();
    HLOG_INFO << "pred OnTopoMap GetMapWithoutLaneGeometry cost "
              << local_tic.Toc();
    local_tic.Tic();

    // CreatLaneTable(lanes_in_range);
    // CreatRoadTable(roads_in_range);

    // 求没条lane的tan_theta
    // for (const auto& it : lanes_in_range) {
    //   // 计算变道引导线的tan theta
    //   CalculateTanTheta(it->lane().id().id());
    // }

    CreatIdVector(local_msg_);
    CreatAddIdVector(end_lane_ids_);

    HLOG_INFO << "pred OnTopoMap hash " << local_tic.Toc();
    auto cost = tic.Toc();
    HLOG_INFO << "pred OnTopoMap construct hash cost " << cost;

    HLOG_INFO << "far_table_.size() " << far_table_.size();
    HLOG_INFO << "lane_table_.size() " << lane_table_.size();
    HLOG_INFO << "road_table_.size() " << road_table_.size();
    HLOG_INFO << "topo_lane_ids_.size() " << topo_lane_ids_.size();
    HLOG_INFO << "topo_section_ids_.size() " << topo_section_ids_.size();
    HLOG_INFO << "end_lane_ids_.size() " << end_lane_ids_.size();
    HLOG_INFO << "end_section_ids_.size() " << end_section_ids_.size();
    HLOG_INFO << "add_lane_ids_.size() " << add_lane_ids_.size();
    HLOG_INFO << "add_section_ids_.size() " << add_section_ids_.size();
  }
}

std::shared_ptr<hozon::hdmap::Map> MapPrediction::GetHdMap(
    bool need_update_global_hd, hozon::routing::RoutingResponse* routing) {
  if (!need_update_global_hd) {
    return hq_map_;
  }

  if (!GLOBAL_HD_MAP) {
    HLOG_ERROR << "nullptr hq_map_server_";
    return nullptr;
  }

  hozon::common::PointENU utm_pos;
  utm_pos.set_x(location_utm_.x());
  utm_pos.set_y(location_utm_.y());
  utm_pos.set_z(0);

  double nearest_s = 0.;
  double nearest_l = 0.;
  hozon::hdmap::LaneInfoConstPtr lane_ptr = nullptr;

  int ret =
      GLOBAL_HD_MAP->GetNearestLane(utm_pos, &lane_ptr, &nearest_s, &nearest_l);

  if (ret != 0 || lane_ptr == nullptr) {
    HLOG_ERROR << "get nearest lane failed";
    hq_map_ = nullptr;
    return hq_map_;
  }

  hq_map_ = std::make_shared<hozon::hdmap::Map>();

  std::vector<std::vector<std::string>> routing_lanes;
  for (const auto& road_it : routing->road()) {
    std::vector<std::string> routing_lane;
    for (const auto& passage_it : road_it.passage()) {
      for (const auto& lane_it : passage_it.segment()) {
        routing_lane.emplace_back(lane_it.id());
      }
    }
    routing_lanes.emplace_back(routing_lane);
  }

  if (routing_lanes.empty()) {
    HLOG_ERROR << "get rouitng message failed";
    return nullptr;
  }

  int row = -1;
  // 查找车辆所在lane的位置
  for (int i = 0; i < routing_lanes.size(); ++i) {
    for (int j = 0; j < routing_lanes[i].size(); ++j) {
      if (routing_lanes[i][j] == lane_ptr->lane().id().id()) {
        row = i;
        break;
      }
    }
    if (row != -1) {
      break;
    }
  }

  if (row == -1) {
    HLOG_ERROR << "find vehicle lane failed";
    return nullptr;
  }

  double lane_length_forward = 0.;
  double lane_length_backward = 0.;

  int row_max = -1;
  int row_min = 0;

  for (int i = row; i < routing_lanes.size(); ++i) {
    hozon::hdmap::Id lane_id;
    auto size = routing_lanes[i].size();
    lane_id.set_id(routing_lanes[i][size - 1]);
    auto lane_ptr = GLOBAL_HD_MAP->GetLaneById(lane_id);
    if (lane_ptr != nullptr) {
      lane_length_forward = lane_length_forward + lane_ptr->lane().length();
    }
    row_max = i;
    if (lane_length_forward >= 300) {
      break;
    }
  }

  for (int i = row - 1; i >= 0; --i) {
    hozon::hdmap::Id lane_id;
    auto size = routing_lanes[i].size();
    lane_id.set_id(routing_lanes[i][size - 1]);
    auto lane_ptr = GLOBAL_HD_MAP->GetLaneById(lane_id);
    if (lane_ptr != nullptr) {
      lane_length_backward = lane_length_backward + lane_ptr->lane().length();
    }
    row_min = i;
    if (lane_length_backward >= 100) {
      break;
    }
  }

  for (int i = row_min; i <= row_max; ++i) {
    for (int j = 0; j < routing_lanes[i].size(); ++j) {
      hozon::hdmap::Id lane_id;
      lane_id.set_id(routing_lanes[i][j]);
      auto lane_ptr = GLOBAL_HD_MAP->GetLaneById(lane_id);
      if (lane_ptr != nullptr) {
        hq_map_->add_lane()->CopyFrom(lane_ptr->lane());
      }
    }
  }

  // for (int i = 0; i < row_min; ++i) {
  //   for (int j = 0; j < routing_lanes[i].size(); ++j) {
  //     hozon::hdmap::Id lane_id;
  //     lane_id.set_id(routing_lanes[i][j]);
  //     auto lane_ptr = GLOBAL_HD_MAP->GetLaneById(lane_id);
  //     if (lane_ptr != nullptr) {
  //       AppendRoutingLane(lane_ptr);
  //     }
  //   }
  // }

  for (int i = row_max + 1; i < routing_lanes.size(); ++i) {
    for (int j = 0; j < routing_lanes[i].size(); ++j) {
      hozon::hdmap::Id lane_id;
      lane_id.set_id(routing_lanes[i][j]);
      auto lane_ptr = GLOBAL_HD_MAP->GetLaneById(lane_id);
      if (lane_ptr != nullptr) {
        AppendRoutingLane(lane_ptr);
      }
    }
  }

  // 获取roads
  std::vector<hozon::hdmap::RoadInfoConstPtr> roads;
  ret = GLOBAL_HD_MAP->GetRoads(utm_pos, 300, &roads);
  if (ret == 0) {
    for (const auto& road : roads) {
      hq_map_->add_road()->CopyFrom(road->road());
    }
  } else {
    HLOG_DEBUG << "get roads failed";
    return nullptr;
  }

  if (!hq_map_) {
    HLOG_ERROR << "nullptr hq_map_";
    return nullptr;
  }
  if (local_enu_center_flag_) {
    HLOG_ERROR << "init_pose_ not inited";
    return nullptr;
  }
  HDMapLaneToLocal();
  RoutingPointToLocal(routing);
  // viz_map_.VizLocalMapLaneLine(hq_map_);
  // viz_map_.VizLaneID(hq_map_);

  // for (const auto& lane : hq_map_->lane()) {
  //   std::vector<Vec2d> cent_points;

  //   cent_points.clear();
  //   for (const auto& lane_seg : lane.central_curve().segment()) {
  //     for (const auto& point : lane_seg.line_segment().point()) {
  //       Vec2d cent_point;
  //       cent_point.set_x(point.x());
  //       cent_point.set_y(point.y());
  //       cent_points.emplace_back(cent_point);
  //     }
  //   }

  //   viz_map_.VizCenterLane(cent_points);
  // }
  return hq_map_;
}

void MapPrediction::Clear() {
  far_table_.clear();
  lane_table_.clear();
  road_table_.clear();
  topo_lane_ids_.clear();
  topo_section_ids_.clear();
  end_lane_ids_.clear();
  end_section_ids_.clear();
  add_lane_ids_.clear();
  add_section_ids_.clear();
  end_prev_ids_.clear();
  all_section_ids_.clear();
}

void MapPrediction::CreatIdVector(
    const std::shared_ptr<hozon::hdmap::Map>& local_msg_) {
  // 创建id存储器
  for (const auto& it : local_msg_->lane()) {
    auto lane_id = it.id().id();
    topo_lane_ids_.emplace_back(lane_id);
    if (lane_table_.find(lane_id) == lane_table_.end()) {
      HLOG_ERROR << "lane in local_msg not found in lane_table";
      continue;
    }
    const auto& local_lane = lane_table_[lane_id];
    topo_section_ids_.insert(local_lane.section_id);

    CreatendIdVector(it, &lane_id, local_lane);
  }
}

void MapPrediction::CreatendIdVector(const hozon::hdmap::Lane& it,
                                     std::string* lane_id,
                                     const LaneInfo& local_lane) {
  // 创建end_lane_id和end_section_id表
  if (!it.successor_id().empty()) {
    return;
  }
  end_lane_ids_.emplace_back(*lane_id);
  end_section_ids_.insert(local_lane.section_id);

  // 判断当前lane是否都有左右边线
  auto left_seg = it.left_boundary().curve().segment_size();
  auto right_seg = it.right_boundary().curve().segment_size();
  while ((left_seg == 0 && right_seg != 0) ||
         (left_seg != 0 && right_seg == 0)) {
    if (lane_table_.find(*lane_id) == lane_table_.end() ||
        lane_table_.at(*lane_id).prev_lane_ids.empty()) {
      break;
    }
    const auto& prev_id = lane_table_.at(*lane_id).prev_lane_ids[0];
    if (lane_table_.find(prev_id) == lane_table_.end()) {
      break;
    }
    end_lane_ids_.emplace_back(prev_id);
    end_section_ids_.insert(lane_table_.at(prev_id).section_id);
    if (left_seg == 0) {
      end_prev_ids_.insert_or_assign(prev_id, 0);
    } else {
      end_prev_ids_.insert_or_assign(prev_id, 1);
    }

    int count = 0;
    for (const auto& itt : local_msg_->lane()) {
      if (itt.id().id() == prev_id) {
        left_seg = itt.left_boundary().curve().segment_size();
        right_seg = itt.right_boundary().curve().segment_size();
        *lane_id = itt.id().id();
        break;
      }
      count += 1;
    }

    if (count == local_msg_->lane_size()) {
      break;
    }
  }
}

void MapPrediction::CreatAddIdVector(
    const std::vector<std::string>& end_lane_ids_) {
  // 创建AddIdVector
  for (const auto& it : end_lane_ids_) {
    std::string curr_lane_id = it;
    while (lane_table_.find(curr_lane_id) != lane_table_.end()) {
      const auto& local_lane = lane_table_[curr_lane_id];
      if (local_lane.next_lane_ids.empty()) {
        break;
      }
      std::string first = local_lane.next_lane_ids.front();
      std::string last = local_lane.next_lane_ids.back();
      add_lane_ids_.insert(first);
      if (std::find(add_section_ids_.begin(), add_section_ids_.end(),
                    local_lane.section_id) == add_section_ids_.end()) {
        add_section_ids_.emplace_back(local_lane.section_id);
      }
      curr_lane_id = first;
    }
  }

  for (const auto& it : end_lane_ids_) {
    std::string curr_lane_id = it;
    while (lane_table_.find(curr_lane_id) != lane_table_.end()) {
      const auto& local_lane = lane_table_[curr_lane_id];
      if (local_lane.next_lane_ids.empty()) {
        break;
      }
      std::string last = local_lane.next_lane_ids.back();
      add_lane_ids_.insert(last);
      if (std::find(add_section_ids_.begin(), add_section_ids_.end(),
                    local_lane.section_id) == add_section_ids_.end()) {
        add_section_ids_.emplace_back(local_lane.section_id);
      }
      curr_lane_id = last;
    }
  }
}

void MapPrediction::FusionLocalAndMap() {
  // fusion local map lane and hdmap lane
  FusionLanePoint();

  // for (auto& hq_road : *local_msg_->mutable_road()) {
  //   for (auto& sec : *hq_road.mutable_section()) {
  //     if (sec.lane_id().empty()) {
  //       continue;
  //     }
  //     auto lane_id = sec.lane_id(0);
  //     if (lane_table_.find(lane_id.id()) == lane_table_.end()) {
  //       continue;
  //     }
  //     auto section_id = lane_table_.at(lane_id.id()).section_id;
  //     auto road_id = lane_table_.at(lane_id.id()).road_id;
  //     if (road_table_.find(road_id) == road_table_.end()) {
  //       continue;
  //     }
  //     auto sections = road_table_.at(road_id);
  //     auto left_boundary = sections.section_ids.at(section_id).left_boundary;
  //     auto right_boundary =
  //     sections.section_ids.at(section_id).right_boundary;

  //     for (auto& edge :
  //          *sec.mutable_boundary()->mutable_outer_polygon()->mutable_edge())
  //          {
  //       FusionRoadEdgePoint(&edge, left_boundary, right_boundary);
  //     }
  //   }
  // }
}

void MapPrediction::FusionLanePoint() {
  // fusion lane line point
  for (auto& it : *local_msg_->mutable_lane()) {
    if (lane_table_.find(it.id().id()) == lane_table_.end()) {
      continue;
    }
    auto left_points = lane_table_.at(it.id().id()).left_line;
    for (auto& left_seg :
         *it.mutable_left_boundary()->mutable_curve()->mutable_segment()) {
      for (auto& pt : *left_seg.mutable_line_segment()->mutable_point()) {
        Eigen::Vector3d P(pt.x(), pt.y(), pt.z());
        Eigen::Vector3d C;
        bool flag = false;
        ComputePerpendicularPoint(P, left_points, &C, &flag);
        if (!flag) {
          FusionStartOrEndLeftPoint(P, it.id().id(), &C, &flag);
          if (!flag) {
            continue;
          }
        }
        // 对原始点和新点进行加权平均
        auto new_point = (P + C) / 2.0;
        pt.set_x(new_point.x());
        pt.set_y(new_point.y());
        pt.set_z(new_point.z());
      }
    }

    auto right_points = lane_table_.at(it.id().id()).right_line;
    for (auto& right_seg :
         *it.mutable_right_boundary()->mutable_curve()->mutable_segment()) {
      for (auto& pt : *right_seg.mutable_line_segment()->mutable_point()) {
        Eigen::Vector3d P(pt.x(), pt.y(), pt.z());
        Eigen::Vector3d C;
        bool flag = false;
        ComputePerpendicularPoint(P, right_points, &C, &flag);
        if (!flag) {
          FusionStartOrEndRightPoint(P, it.id().id(), &C, &flag);
          if (!flag) {
            continue;
          }
        }
        // 对原始点和新点进行加权平均
        auto new_point = (P + C) / 2.0;
        pt.set_x(new_point.x());
        pt.set_y(new_point.y());
        pt.set_z(new_point.z());
      }
    }
  }
}

void MapPrediction::FusionStartOrEndLeftPoint(const Eigen::Vector3d& P,
                                              const std::string& id,
                                              Eigen::Vector3d* C, bool* flag) {
  auto next_ids = lane_table_.at(id).next_lane_ids;
  if (next_ids.empty()) {
    return;
  }
  auto next_id = next_ids[0];
  if (lane_table_.find(next_id) == lane_table_.end()) {
    return;
  }
  auto next_left_points = lane_table_.at(next_id).left_line;
  ComputePerpendicularPoint(P, next_left_points, C, flag);

  auto prev_ids = lane_table_.at(id).prev_lane_ids;
  if (prev_ids.empty()) {
    return;
  }
  auto prev_id = prev_ids[0];
  if (lane_table_.find(prev_id) == lane_table_.end()) {
    return;
  }
  auto prev_left_points = lane_table_.at(prev_id).left_line;
  ComputePerpendicularPoint(P, prev_left_points, C, flag);
}

void MapPrediction::FusionStartOrEndRightPoint(const Eigen::Vector3d& P,
                                               const std::string& id,
                                               Eigen::Vector3d* C, bool* flag) {
  auto next_ids = lane_table_.at(id).next_lane_ids;
  if (next_ids.empty()) {
    return;
  }
  auto next_id = next_ids[0];
  if (lane_table_.find(next_id) == lane_table_.end()) {
    return;
  }
  auto next_right_points = lane_table_.at(next_id).right_line;
  ComputePerpendicularPoint(P, next_right_points, C, flag);

  auto prev_ids = lane_table_.at(id).prev_lane_ids;
  if (prev_ids.empty()) {
    return;
  }
  auto prev_id = prev_ids[0];
  if (lane_table_.find(prev_id) == lane_table_.end()) {
    return;
  }
  auto prev_right_points = lane_table_.at(prev_id).right_line;
  ComputePerpendicularPoint(P, prev_right_points, C, flag);
}

#if 0
void MapPrediction::FusionRoadEdgePoint(
    hozon::hdmap::BoundaryEdge* edge,
    const std::vector<Eigen::Vector3d>& left_boundary,
    const std::vector<Eigen::Vector3d>& right_boundary) {
  // fusion road edge point
  if (edge->type() == 2) {
    for (auto& seg : *edge->mutable_curve()->mutable_segment()) {
      for (auto& pt : *seg.mutable_line_segment()->mutable_point()) {
        Eigen::Vector3d P(pt.x(), pt.y(), pt.z());
        Eigen::Vector3d C;
        bool flag = false;
        ComputePerpendicularPoint(P, left_boundary, &C, &flag);
        if (!flag) {
          continue;
        }
        // 对原始点和新点进行加权平均
        auto new_point = (P + C) / 2.0;
        pt.set_x(new_point.x());
        pt.set_y(new_point.y());
        pt.set_z(new_point.z());
      }
    }
  }
  if (edge->type() == 3) {
    for (auto& seg : *edge->mutable_curve()->mutable_segment()) {
      for (auto& pt : *seg.mutable_line_segment()->mutable_point()) {
        Eigen::Vector3d P(pt.x(), pt.y(), pt.z());
        Eigen::Vector3d C;
        bool flag = false;
        ComputePerpendicularPoint(P, right_boundary, &C, &flag);
        if (!flag) {
          continue;
        }
        // 对原始点和新点进行加权平均
        auto new_point = (P + C) / 2.0;
        pt.set_x(new_point.x());
        pt.set_y(new_point.y());
        pt.set_z(new_point.z());
      }
    }
  }
}
#endif

void MapPrediction::ComputePerpendicularPoint(
    const Eigen::Vector3d& P, const std::vector<Eigen::Vector3d>& left_points,
    Eigen::Vector3d* C, bool* flag) {
  // 计算垂点
  for (size_t i = 1; i < left_points.size(); i++) {
    const auto& A = left_points[i - 1];
    const auto& B = left_points[i];
    const auto& AB = B - A;
    const auto& AP = P - A;
    double ABLengthSquared = AB.squaredNorm();
    if (ABLengthSquared == 0) {
      continue;
    }
    double t = AB.dot(AP) / ABLengthSquared;
    if (t < 0 || t > 1) {
      continue;
    }
    t = std::max(0.0, std::min(t, 1.0));
    *C = A + t * AB;  // 点到线段的最近点
    *flag = true;
    break;
  }
}

void MapPrediction::CompleteLaneline(
    const std::vector<std::string>& end_lane_ids_,
    const std::set<std::string>& end_section_id,
    const std::unordered_map<std::string, RoadInfo>& road_table) {
  if (end_lane_ids_.empty() || end_section_id.empty() || road_table.empty()) {
    return;
  }
  for (const auto& sec : end_section_id) {
    std::vector<std::string> com_id;
    std::string road_id;
    GetCorresId(&com_id, &road_id, sec);

    if (com_id.empty()) {
      continue;
    }

    if (road_table.find(road_id) == road_table.end()) {
      HLOG_ERROR << "end lane road not found in road_table";
      continue;
    }

    const auto& hash_table = road_table.at(road_id).section_ids;
    if (hash_table.empty()) {
      HLOG_ERROR << "empty section id";
      continue;
    }

    // std::vector<std::vector<Eigen::Vector3d>> left_boundary =
    //     hash_table.at(sec).left_boundary;
    // std::vector<std::vector<Eigen::Vector3d>> right_boundary =
    //     hash_table.at(sec).right_boundary;
    // std::vector<Eigen::Vector3d> road_boundary =
    //     hash_table.at(sec).road_boundary;
    // 拟合线
    // std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>
    //     complete_lines;
    // uint32_t lane_num = hash_table.at(sec).lane_id.size();
    // FitPredLaneLine(road_boundary, hash_table.at(sec).lane_id,
    // complete_lines); FitAheadLaneLine(left_boundary, right_boundary,
    // complete_lines, lane_num);

    // viz_map_.VizAddAheadLaneLine(complete_lines);
    // 延伸
    ExpansionLaneLine(com_id, hash_table.at(sec).lane_id);
  }
}

void MapPrediction::GetCorresId(std::vector<std::string>* com_id,
                                std::string* road_id, const std::string& sec) {
  for (const auto& end_lane_id : end_lane_ids_) {
    if (lane_table_.find(end_lane_id) == lane_table_.end()) {
      HLOG_ERROR << "end lane id not found in lane_table";
      continue;
    }
    if (lane_table_[end_lane_id].section_id != sec) {
      continue;
    }
    com_id->emplace_back(end_lane_id);
    *road_id = lane_table_[end_lane_id].road_id;
  }
}

void MapPrediction::ExpansionLaneLine(const std::vector<std::string>& com_id,
                                      const std::vector<std::string>& sec_id) {
  // 开始对当前的车道进行延伸
  if (com_id.empty() || sec_id.empty()) {
    return;
  }
  // 将填充的车道线保存并可视化出来
  std::vector<std::vector<Eigen::Vector3d>> compan_lines;
  for (auto& lane : *local_msg_->mutable_lane()) {
    for (const auto& end_id : com_id) {
      if (end_id != lane.id().id()) {
        continue;
      }
      // 定义全局标识符
      uint32_t com_fit = 2;
      JudgeDiection(&com_fit, end_id);

      if ((com_fit == 0 || com_fit == 2) &&
          !lane.left_boundary().curve().segment().empty()) {
        // 左边界
        // 临时加了一个判断条件
        // if (lane.left_boundary().curve().segment().empty()) {
        //   continue;
        // }
        ExpansionLeft(&lane);
      }

      if ((com_fit == 1 || com_fit == 2) &&
          !lane.right_boundary().curve().segment().empty()) {
        // 右边界
        // 临时加了一个判断条件
        // if (!lane.right_neighbor_forward_lane_id().empty()) {
        //   continue;
        // }
        // if (lane.right_boundary().curve().segment().empty()) {
        //   continue;
        // }
        ExpansionRight(&lane);
      }
    }
  }
  // 可视化延伸的车道线
  // viz_map_.VizCompanLane(compan_lines);
}

void MapPrediction::JudgeDiection(uint32_t* com_fit,
                                  const std::string& end_id) {
  // 判断左右延伸方向
  if (end_prev_ids_.find(end_id) != end_prev_ids_.end() &&
      end_prev_ids_.at(end_id) == 0) {
    // 只延伸左侧
    *com_fit = 0;
  } else if (end_prev_ids_.find(end_id) != end_prev_ids_.end() &&
             end_prev_ids_.at(end_id) == 1) {
    // 只延伸右侧
    *com_fit = 1;
  } else {
    // 左右都延伸
    *com_fit = 2;
  }
}

void MapPrediction::ExpansionLeft(hdmap::Lane* lane) {
  // 延伸左
  int seg_size = lane->left_boundary().curve().segment().size();
  int point_size = lane->left_boundary()
                       .curve()
                       .segment()[seg_size - 1]
                       .line_segment()
                       .point()
                       .size();
  // 临时加了一个判断条件
  if (seg_size == 0 || point_size == 0) {
    return;
  }
  // 存储左边线的所有点
  std::vector<Eigen::Vector3d> com_left_line;
  for (const auto& seg : lane->left_boundary().curve().segment()) {
    for (const auto& point : seg.line_segment().point()) {
      Eigen::Vector3d point_enu(point.x(), point.y(), point.z());
      com_left_line.emplace_back(point_enu);
    }
  }
  // 左边界最后一个点
  Eigen::Vector3d left_end_point(com_left_line.back().x(),
                                 com_left_line.back().y(),
                                 com_left_line.back().z());
  // 左边界最后一个点到新拟合车道线的投影点的距离
  double min_distance_left = std::numeric_limits<double>::max();
  uint32_t flag_count_left = 0;

  // 地图预测线
  if (lane_table_.find(lane->id().id()) == lane_table_.end()) {
    return;
  }
  auto line_points = lane_table_.at(lane->id().id()).pred_left_line;
  if (line_points.empty()) {
    return;
  }

  PointToLineDist(left_end_point, &flag_count_left, &min_distance_left,
                  line_points);
  // 判断是否满足距离要求
  if (min_distance_left == std::numeric_limits<double>::max()) {
    return;
  }

  // STD!!!!
  auto seg_num = lane->left_boundary().curve().segment_size();
  // 临时加入判断条件
  if (seg_num == 0 || flag_count_left == 0) {
    return;
  }
  std::vector<Eigen::Vector3d> interp_points;

  if (line_points.size() - flag_count_left <= 2) {
    // 直接连接complete_lines中的最后一个点
    const auto& complete_line_size = line_points.size();
    // 去除最后一个点
    if (com_left_line.size() > 1) {
      auto* ptt = lane->mutable_left_boundary()
                      ->mutable_curve()
                      ->mutable_segment(seg_size - 1)
                      ->mutable_line_segment()
                      ->mutable_point(point_size - 1);
      ptt->Clear();
      ptt->set_x(line_points.back().x());
      ptt->set_y(line_points.back().y());
      ptt->set_z(line_points.back().z());
    } else {
      auto* pt = lane->mutable_left_boundary()
                     ->mutable_curve()
                     ->mutable_segment(seg_size - 1)
                     ->mutable_line_segment()
                     ->add_point();
      pt->set_x(line_points.back().x());
      pt->set_y(line_points.back().y());
      pt->set_z(line_points.back().z());
    }
  } else {
    if (com_left_line.size() <= 2) {
      for (size_t i = flag_count_left + 2; i < line_points.size(); ++i) {
        auto* pt = lane->mutable_left_boundary()
                       ->mutable_curve()
                       ->mutable_segment(seg_size - 1)
                       ->mutable_line_segment()
                       ->add_point();
        pt->set_x(line_points[i].x());
        pt->set_y(line_points[i].y());
        pt->set_z(line_points[i].z());
      }
    } else {
      const auto& com_left_size = com_left_line.size();
      Eigen::Vector3d local_first = com_left_line[com_left_size - 3];
      Eigen::Vector3d local_second = com_left_line[com_left_size - 2];
      Eigen::Vector3d com_first = line_points[flag_count_left + 1];
      Eigen::Vector3d com_second = line_points[flag_count_left + 2];
      interp_points.emplace_back(local_first);
      interp_points.emplace_back(local_second);
      interp_points.emplace_back(com_first);
      interp_points.emplace_back(com_second);
      std::vector<Eigen::Vector3d> cat_points;
      // cat_points.emplace_back(local_first);
      CatmullRoom(interp_points, &cat_points);
      // cat_points.emplace_back(com_second);
      // 去除最后一个点
      auto* ptt = lane->mutable_left_boundary()
                      ->mutable_curve()
                      ->mutable_segment(seg_size - 1)
                      ->mutable_line_segment()
                      ->mutable_point(point_size - 1);
      ptt->Clear();
      ptt->set_x(cat_points[0].x());
      ptt->set_y(cat_points[0].y());
      ptt->set_z(cat_points[0].z());

      for (size_t i = flag_count_left + 2; i < line_points.size(); ++i) {
        auto* pt = lane->mutable_left_boundary()
                       ->mutable_curve()
                       ->mutable_segment(seg_size - 1)
                       ->mutable_line_segment()
                       ->add_point();
        pt->set_x(line_points[i].x());
        pt->set_y(line_points[i].y());
        pt->set_z(line_points[i].z());
      }
    }
  }
}

void MapPrediction::ExpansionRight(hdmap::Lane* lane) {
  // 延伸右
  int seg_size_ = lane->right_boundary().curve().segment().size();
  int point_size_ = lane->right_boundary()
                        .curve()
                        .segment(seg_size_ - 1)
                        .line_segment()
                        .point()
                        .size();
  // 临时加了一个判断条件
  if (seg_size_ == 0 || point_size_ == 0) {
    return;
  }
  // 存储右边线的所有点
  std::vector<Eigen::Vector3d> com_right_line;
  for (const auto& seg : lane->right_boundary().curve().segment()) {
    for (const auto& point : seg.line_segment().point()) {
      Eigen::Vector3d point_enu(point.x(), point.y(), point.z());
      com_right_line.emplace_back(point_enu);
    }
  }
  Eigen::Vector3d right_end_point(com_right_line.back().x(),
                                  com_right_line.back().y(),
                                  com_right_line.back().z());
  // 左边界最后一个点到新拟合车道线的投影点的距离
  double min_distance_right = std::numeric_limits<double>::max();
  uint32_t flag_count_right = 0;

  // 地图预测线
  if (lane_table_.find(lane->id().id()) == lane_table_.end()) {
    return;
  }
  auto line_points = lane_table_.at(lane->id().id()).pred_right_line;
  if (line_points.empty()) {
    return;
  }

  // uint32_t count_right = 1;  // 记录从第几个点开始扩充
  // PointToLineDist(right_end_point, complete_lines, right_line_id,
  //                 &flag_count_right, &min_distance_right, lane->id().id(),
  //                 1);
  PointToLineDist(right_end_point, &flag_count_right, &min_distance_right,
                  line_points);
  // 判断是否满足距离要求
  if (min_distance_right == std::numeric_limits<double>::max()) {
    return;
  }

  auto seg_num_right = lane->right_boundary().curve().segment_size();
  // 临时加入判断条件
  if (seg_num_right == 0 || flag_count_right == 0) {
    return;
  }
  std::vector<Eigen::Vector3d> interp_points_right;

  if (line_points.size() - flag_count_right <= 2) {
    // 直接连接complete_lines中的最后一个点
    if (com_right_line.size() > 1) {
      const auto& complete_line_size = line_points.size();
      auto* ptt = lane->mutable_right_boundary()
                      ->mutable_curve()
                      ->mutable_segment(seg_size_ - 1)
                      ->mutable_line_segment()
                      ->mutable_point(point_size_ - 1);
      ptt->Clear();
      ptt->set_x(line_points.back().x());
      ptt->set_y(line_points.back().y());
      ptt->set_z(line_points.back().z());
    } else {
      auto* pt = lane->mutable_right_boundary()
                     ->mutable_curve()
                     ->mutable_segment(seg_size_ - 1)
                     ->mutable_line_segment()
                     ->add_point();
      pt->set_x(line_points.back().x());
      pt->set_y(line_points.back().y());
      pt->set_z(line_points.back().z());
    }
  } else {
    if (com_right_line.size() <= 2) {
      for (size_t i = flag_count_right + 2; i < line_points.size(); ++i) {
        auto* pt = lane->mutable_right_boundary()
                       ->mutable_curve()
                       ->mutable_segment(seg_size_ - 1)
                       ->mutable_line_segment()
                       ->add_point();
        pt->set_x(line_points[i].x());
        pt->set_y(line_points[i].y());
        pt->set_z(line_points[i].z());
      }
    } else {
      const auto& com_right_size = com_right_line.size();
      Eigen::Vector3d local_first = com_right_line[com_right_size - 3];
      Eigen::Vector3d local_second = com_right_line[com_right_size - 2];
      Eigen::Vector3d com_first = line_points[flag_count_right + 1];
      Eigen::Vector3d com_second = line_points[flag_count_right + 2];
      interp_points_right.emplace_back(local_first);
      interp_points_right.emplace_back(local_second);
      interp_points_right.emplace_back(com_first);
      interp_points_right.emplace_back(com_second);
      std::vector<Eigen::Vector3d> cat_points;
      // cat_points.emplace_back(local_first);
      CatmullRoom(interp_points_right, &cat_points);
      // cat_points.emplace_back(com_second);

      auto* ptt = lane->mutable_right_boundary()
                      ->mutable_curve()
                      ->mutable_segment(seg_size_ - 1)
                      ->mutable_line_segment()
                      ->mutable_point(point_size_ - 1);
      ptt->Clear();
      ptt->set_x(cat_points[0].x());
      ptt->set_y(cat_points[0].y());
      ptt->set_z(cat_points[0].z());

      for (size_t i = flag_count_right + 2; i < line_points.size(); ++i) {
        auto* pt = lane->mutable_right_boundary()
                       ->mutable_curve()
                       ->mutable_segment(seg_size_ - 1)
                       ->mutable_line_segment()
                       ->add_point();
        pt->set_x(line_points[i].x());
        pt->set_y(line_points[i].y());
        pt->set_z(line_points[i].z());
      }
    }
  }
}

void MapPrediction::PointToLineDist(
    const Eigen::Vector3d& end_point, uint32_t* index, double* min_distance,
    const std::vector<Eigen::Vector3d>& line_points) {
  // point to line distance
  uint32_t count = 1;  // 记录从第几个点开始扩充
  for (size_t i = 1; i < line_points.size(); ++i) {
    const auto& A = line_points[i - 1];
    const auto& B = line_points[i];
    const auto& P = end_point;

    // 计算A点和B点的方向向量
    Eigen::Vector3d AB = B - A;
    Eigen::Vector3d AP = P - A;
    // 计算AB模长
    double ABLengthSquared = AB.squaredNorm();
    if (ABLengthSquared == 0) {
      continue;
    }
    double t = AB.dot(AP) / ABLengthSquared;
    count = i;
    if (t < 0 || t > 1) {
      continue;
    }

    t = std::max(0.0, std::min(t, 1.0));
    Eigen::Vector3d C = A + t * AB;  // 点到线段的最近点
    double dis = (P - C).norm();
    if (dis < *min_distance) {  //&& dis < 3.75
      *min_distance = dis;
      *index = count;
    }
    break;
  }
}

void MapPrediction::CatmullRoom(
    const std::vector<Eigen::Vector3d>& compan_point,
    std::vector<Eigen::Vector3d>* cat_points) {
  // 拟合-->重采样
  if (compan_point.size() < 4) {
    return;
  }
  auto func = [](double p0, double p1, double p2, double p3, double t) {
    double t2 = t * t;
    double t3 = t2 * t;
    double b1 = 0.5 * (-t3 + 2 * t2 - t);
    double b2 = 0.5 * (3 * t3 - 5 * t2 + 2);
    double b3 = 0.5 * (-3 * t3 + 4 * t2 + t);
    double b4 = 0.5 * (t3 - t2);
    return (p0 * b1 + p1 * b2 + p2 * b3 + p3 * b4);
  };
  for (size_t i = 1; i < compan_point.size() - 2; ++i) {
    for (int t = 5; t < 10; t += 5) {
      double px =
          func(compan_point[i - 1].x(), compan_point[i].x(),
               compan_point[i + 1].x(), compan_point[i + 2].x(), t * 0.1);
      double py =
          func(compan_point[i - 1].y(), compan_point[i].y(),
               compan_point[i + 1].y(), compan_point[i + 2].y(), t * 0.1);
      Eigen::Vector3d point(px, py, 0);
      cat_points->emplace_back(point);
    }
  }
}

Eigen::Vector3d MapPrediction::UtmPtToLocalEnu(
    const hozon::common::PointENU& point_utm) {
  double x = point_utm.x();
  double y = point_utm.y();
  hozon::common::coordinate_convertor::UTM2GCS(utm_zone_, &x, &y);
  Eigen::Vector3d point_gcj(y, x, 0);
  Eigen::Vector3d point_enu =
      util::Geo::Gcj02ToEnu(point_gcj, local_enu_center_);
  return point_enu;
}

Eigen::Vector3d MapPrediction::GcjPtToLocalEnu(
    const hozon::common::PointENU& point_gcj) {
  Eigen::Vector3d pt_gcj(point_gcj.y(), point_gcj.x(), 0);
  Eigen::Vector3d point_enu = util::Geo::Gcj02ToEnu(pt_gcj, local_enu_center_);
  return point_enu;
}

std::shared_ptr<hozon::hdmap::Map> MapPrediction::GetPredictionMap() {
  std::lock_guard<std::mutex> lock_map(mtx_);
  //  return local_msg_;
  return hq_map_;
}

void MapPrediction::AddResTopo() {
  if (!hq_map_) {
    HLOG_ERROR << "hq_map_ not ready";
    return;
  }
  // 清除三公里的全部车道线/中心线点
  util::TicToc tic;
  // 将local_msg_中的车道线点加入对应的hq_map中
  for (auto& hq_lane : *hq_map_->mutable_lane()) {
    for (const auto& local_lane : local_msg_->lane()) {
      if (hq_lane.id().id() == local_lane.id().id()) {
        // 加入左边线点
        auto left_seg_size = local_lane.left_boundary().curve().segment_size();
        if (left_seg_size == 0) {
          continue;
        }
        hq_lane.mutable_left_boundary()
            ->mutable_curve()
            ->mutable_segment(0)
            ->mutable_line_segment()
            ->mutable_point()
            ->CopyFrom(local_lane.left_boundary()
                           .curve()
                           .segment(0)
                           .line_segment()
                           .point());
        // 加入右边线点
        auto right_seg_size =
            local_lane.right_boundary().curve().segment_size();
        if (right_seg_size == 0) {
          continue;
        }
        hq_lane.mutable_right_boundary()
            ->mutable_curve()
            ->mutable_segment(0)
            ->mutable_line_segment()
            ->mutable_point()
            ->CopyFrom(local_lane.right_boundary()
                           .curve()
                           .segment(0)
                           .line_segment()
                           .point());
        // 加入中心线点
        auto cen_seg_size = local_lane.central_curve().segment_size();
        if (cen_seg_size == 0) {
          continue;
        }
        hq_lane.mutable_central_curve()
            ->mutable_segment(0)
            ->mutable_line_segment()
            ->mutable_point()
            ->CopyFrom(
                local_lane.central_curve().segment(0).line_segment().point());
      }
    }
  }

  AddArrawStopLine();
  AddCrossWalk();

  HLOG_ERROR << "pred AddResTopo insert pts " << tic.Toc();
  tic.Tic();

  ConvertToLocal();
  HLOG_ERROR << "pred ConvertToLocal cost " << tic.Toc();
  tic.Tic();

  //  hozon::hdmap::Header raw_header;
  //  raw_header.CopyFrom(local_msg_->header());
  local_msg_->Clear();
  //  local_msg_->CopyFrom(*hq_map);
  //  local_msg_->mutable_header()->CopyFrom(raw_header);

  HLOG_ERROR << "pred clear local_msg_ cost " << tic.Toc();
}

void MapPrediction::AddArrawStopLine() {
  if (local_msg_->arraw_size() != 0) {
    hq_map_->mutable_arraw()->CopyFrom(local_msg_->arraw());
  }

  if (local_msg_->stop_line_size() != 0) {
    hq_map_->mutable_stop_line()->CopyFrom(local_msg_->stop_line());
  }
}

void MapPrediction::AddCrossWalk() {
  if (local_msg_->crosswalk_size() != 0) {
    hq_map_->mutable_crosswalk()->CopyFrom(local_msg_->crosswalk());
  }
}

void MapPrediction::ConvertToLocal() {
  if (!hq_map_) {
    HLOG_ERROR << "nullptr hq_map_";
    return;
  }

  if (local_enu_center_flag_) {
    HLOG_ERROR << "init_pose_ not inited";
    return;
  }

  FusionMapLaneToLocal();
  RoadToLocal();
  ArrawStopLineToLocal();
  CrossWalkToLocal();
}

void MapPrediction::FusionMapLaneToLocal() {
  // local_enu to local
  for (auto& hq_lane : *hq_map_->mutable_lane()) {
    for (auto& seg : *hq_lane.mutable_central_curve()->mutable_segment()) {
      for (auto& pt : *seg.mutable_line_segment()->mutable_point()) {
        Eigen::Vector3d pt_local(pt.x(), pt.y(), pt.z());
        pt_local = T_local_enu_to_local_ * pt_local;
        pt.set_x(pt_local.x());
        pt.set_y(pt_local.y());
        pt.set_z(0);
      }
    }

    for (auto& seg :
         *hq_lane.mutable_left_boundary()->mutable_curve()->mutable_segment()) {
      for (auto& pt : *seg.mutable_line_segment()->mutable_point()) {
        Eigen::Vector3d pt_local(pt.x(), pt.y(), pt.z());
        pt_local = T_local_enu_to_local_ * pt_local;
        pt.set_x(pt_local.x());
        pt.set_y(pt_local.y());
        pt.set_z(0);
      }
    }

    for (auto& seg : *hq_lane.mutable_right_boundary()
                          ->mutable_curve()
                          ->mutable_segment()) {
      for (auto& pt : *seg.mutable_line_segment()->mutable_point()) {
        Eigen::Vector3d pt_local(pt.x(), pt.y(), pt.z());
        pt_local = T_local_enu_to_local_ * pt_local;
        pt.set_x(pt_local.x());
        pt.set_y(pt_local.y());
        pt.set_z(0);
      }
    }
  }
}

void MapPrediction::HDMapLaneToLocal() {
  // gcj02 to local
  for (auto& hq_lane : *hq_map_->mutable_lane()) {
    for (auto& seg : *hq_lane.mutable_central_curve()->mutable_segment()) {
      (*seg.mutable_line_segment()->mutable_point()).Clear();
      for (auto& pt : *seg.mutable_line_segment()->mutable_original_point()) {
        Eigen::Vector3d pt_local = GcjPtToLocalEnu(pt);
        pt_local = T_local_enu_to_local_ * pt_local;
        auto* point = (*seg.mutable_line_segment()).add_point();
        point->set_x(pt_local.x());
        point->set_y(pt_local.y());
        point->set_z(0);
      }
    }

    for (auto& seg :
         *hq_lane.mutable_left_boundary()->mutable_curve()->mutable_segment()) {
      (*seg.mutable_line_segment()->mutable_point()).Clear();
      for (auto& pt : *seg.mutable_line_segment()->mutable_original_point()) {
        Eigen::Vector3d pt_local = GcjPtToLocalEnu(pt);
        pt_local = T_local_enu_to_local_ * pt_local;
        auto* point = (*seg.mutable_line_segment()).add_point();
        point->set_x(pt_local.x());
        point->set_y(pt_local.y());
        point->set_z(0);
      }
    }

    for (auto& seg : *hq_lane.mutable_right_boundary()
                          ->mutable_curve()
                          ->mutable_segment()) {
      (*seg.mutable_line_segment()->mutable_point()).Clear();
      for (auto& pt : *seg.mutable_line_segment()->mutable_original_point()) {
        Eigen::Vector3d pt_local = GcjPtToLocalEnu(pt);
        pt_local = T_local_enu_to_local_ * pt_local;
        auto* point = (*seg.mutable_line_segment()).add_point();
        point->set_x(pt_local.x());
        point->set_y(pt_local.y());
        point->set_z(0);
      }
    }
  }
}

void MapPrediction::ArrawStopLineToLocal() {
  for (auto& hq_arraw : *hq_map_->mutable_arraw()) {
    for (auto& pt : *hq_arraw.mutable_shape()->mutable_point()) {
      Eigen::Vector3d pt_local(pt.x(), pt.y(), pt.z());
      pt_local = T_local_enu_to_local_ * pt_local;
      pt.set_x(pt_local.x());
      pt.set_y(pt_local.y());
      pt.set_z(0);
    }

    // 将center point从local enu转到local系
    auto& pt_center = *hq_arraw.mutable_center_point();
    Eigen::Vector3d pt_local_center(pt_center.x(), pt_center.y(), 0);
    pt_local_center = T_local_enu_to_local_ * pt_local_center;
    pt_center.set_x(pt_local_center.x());
    pt_center.set_y(pt_local_center.y());

    // 将heading从local_enu系转到local系
    double enu_heading = hq_arraw.heading();
    Eigen::Quaterniond quat_arrow_in_local_enu(
        Eigen::AngleAxisd(enu_heading, Eigen::Vector3d::UnitZ()));

    Eigen::Quaterniond quat_enu_to_local(T_local_enu_to_local_.rotation());

    Eigen::Quaterniond quat_arrow_in_local =
        quat_enu_to_local * quat_arrow_in_local_enu;
    Eigen::Vector3d euler =
        quat_arrow_in_local.toRotationMatrix().eulerAngles(2, 0, 1);
    hq_arraw.clear_heading();
    hq_arraw.set_heading(euler[0]);
  }

  for (auto& hq_line : *hq_map_->mutable_stop_line()) {
    for (auto& pt : *hq_line.mutable_shape()->mutable_point()) {
      Eigen::Vector3d pt_local(pt.x(), pt.y(), pt.z());
      pt_local = T_local_enu_to_local_ * pt_local;
      pt.set_x(pt_local.x());
      pt.set_y(pt_local.y());
      pt.set_z(0);
    }
  }
}

void MapPrediction::RoutingPointToLocal(
    hozon::routing::RoutingResponse* routing) {
  for (auto way_point_it =
           (*routing->mutable_routing_request()->mutable_waypoint()).begin();
       way_point_it !=
       (*routing->mutable_routing_request()->mutable_waypoint()).end();) {
    if ((*way_point_it).type() == hozon::routing::LaneWaypointType::NORMAL) {
      way_point_it = (*routing->mutable_routing_request()->mutable_waypoint())
                         .erase(way_point_it);
    } else {
      auto pose = (*way_point_it).pose();
      Eigen::Vector3d pt_local = UtmPtToLocalEnu(pose);
      pt_local = T_local_enu_to_local_ * pt_local;
      // 转到local系下面
      (*way_point_it).mutable_pose()->Clear();
      (*way_point_it).mutable_pose()->set_x(pt_local.x());
      (*way_point_it).mutable_pose()->set_y(pt_local.y());
      (*way_point_it).mutable_pose()->set_z(0.0);
      ++way_point_it;
    }
  }
}

void MapPrediction::AppendRoutingLane(
    const hozon::hdmap::LaneInfoConstPtr& lane_ptr) {
  if (lane_ptr == nullptr) return;
  auto* new_lane = hq_map_->add_lane();
  new_lane->mutable_id()->CopyFrom(lane_ptr->lane().id());
  // central_curve
  for (const auto& seg : lane_ptr->lane().central_curve().segment()) {
    auto* new_seg = new_lane->mutable_central_curve()->add_segment();
    new_seg->set_s(seg.s());
    new_seg->mutable_start_position()->CopyFrom(seg.start_position());
    new_seg->set_heading(seg.heading());
    new_seg->set_length(seg.length());
  }

  // left_boundary
  for (const auto& seg : lane_ptr->lane().left_boundary().curve().segment()) {
    auto* new_seg =
        new_lane->mutable_left_boundary()->mutable_curve()->add_segment();
    new_seg->set_s(seg.s());
    new_seg->mutable_start_position()->CopyFrom(seg.start_position());
    new_seg->set_heading(seg.heading());
    new_seg->set_length(seg.length());
  }
  new_lane->mutable_left_boundary()->set_length(
      lane_ptr->lane().left_boundary().length());
  new_lane->mutable_left_boundary()->set_virtual_(
      lane_ptr->lane().left_boundary().virtual_());
  new_lane->mutable_left_boundary()->mutable_boundary_type()->CopyFrom(
      lane_ptr->lane().left_boundary().boundary_type());
  new_lane->mutable_left_boundary()->mutable_id()->CopyFrom(
      lane_ptr->lane().left_boundary().id());

  // right_boundary
  for (const auto& seg : lane_ptr->lane().right_boundary().curve().segment()) {
    auto* new_seg =
        new_lane->mutable_right_boundary()->mutable_curve()->add_segment();
    new_seg->set_s(seg.s());
    new_seg->mutable_start_position()->CopyFrom(seg.start_position());
    new_seg->set_heading(seg.heading());
    new_seg->set_length(seg.length());
  }

  new_lane->mutable_right_boundary()->set_length(
      lane_ptr->lane().right_boundary().length());
  new_lane->mutable_right_boundary()->set_virtual_(
      lane_ptr->lane().right_boundary().virtual_());
  new_lane->mutable_right_boundary()->mutable_boundary_type()->CopyFrom(
      lane_ptr->lane().right_boundary().boundary_type());
  new_lane->mutable_right_boundary()->mutable_id()->CopyFrom(
      lane_ptr->lane().right_boundary().id());

  new_lane->set_length(lane_ptr->lane().length());
  new_lane->set_speed_limit(lane_ptr->lane().speed_limit());
  new_lane->mutable_overlap_id()->CopyFrom(lane_ptr->lane().overlap_id());
  new_lane->mutable_predecessor_id()->CopyFrom(
      lane_ptr->lane().predecessor_id());
  new_lane->mutable_successor_id()->CopyFrom(lane_ptr->lane().successor_id());
  new_lane->mutable_left_neighbor_forward_lane_id()->CopyFrom(
      lane_ptr->lane().left_neighbor_forward_lane_id());
  new_lane->mutable_right_neighbor_forward_lane_id()->CopyFrom(
      lane_ptr->lane().right_neighbor_forward_lane_id());
  new_lane->set_type(lane_ptr->lane().type());
  new_lane->set_turn(lane_ptr->lane().turn());
  new_lane->mutable_left_neighbor_reverse_lane_id()->CopyFrom(
      lane_ptr->lane().left_neighbor_reverse_lane_id());
  new_lane->mutable_right_neighbor_reverse_lane_id()->CopyFrom(
      lane_ptr->lane().right_neighbor_reverse_lane_id());
  new_lane->mutable_junction_id()->CopyFrom(lane_ptr->lane().junction_id());
  new_lane->mutable_left_sample()->CopyFrom(lane_ptr->lane().left_sample());
  new_lane->mutable_right_sample()->CopyFrom(lane_ptr->lane().right_sample());
  new_lane->set_direction(lane_ptr->lane().direction());
  new_lane->mutable_left_road_sample()->CopyFrom(
      lane_ptr->lane().left_road_sample());
  new_lane->mutable_right_road_sample()->CopyFrom(
      lane_ptr->lane().right_road_sample());
  new_lane->mutable_self_reverse_lane_id()->CopyFrom(
      lane_ptr->lane().self_reverse_lane_id());
  new_lane->set_lane_transition(lane_ptr->lane().lane_transition());
  new_lane->mutable_map_lane_type()->CopyFrom(lane_ptr->lane().map_lane_type());
  new_lane->mutable_gcs_lane_point()->CopyFrom(
      lane_ptr->lane().gcs_lane_point());
  new_lane->mutable_extra_left_boundary()->CopyFrom(
      lane_ptr->lane().extra_left_boundary());
  new_lane->mutable_extra_right_boundary()->CopyFrom(
      lane_ptr->lane().extra_right_boundary());
}

void MapPrediction::RoadToLocal() {
  // road to local
  for (auto& hq_road : *hq_map_->mutable_road()) {
    for (auto& sec : *hq_road.mutable_section()) {
      for (auto& edge :
           *sec.mutable_boundary()->mutable_outer_polygon()->mutable_edge()) {
        DeelEdge(&edge);
      }
    }
  }
}

void MapPrediction::DeelEdge(hozon::hdmap::BoundaryEdge* edge) {
  for (auto& seg : *edge->mutable_curve()->mutable_segment()) {
    (*seg.mutable_line_segment()->mutable_point()).Clear();
    for (auto& pt : *seg.mutable_line_segment()->mutable_original_point()) {
      Eigen::Vector3d pt_local = GcjPtToLocalEnu(pt);
      pt_local = T_local_enu_to_local_ * pt_local;
      auto* point = (*seg.mutable_line_segment()).add_point();
      point->set_x(pt_local.x());
      point->set_y(pt_local.y());
      point->set_z(0);
    }
  }
}

void MapPrediction::CrossWalkToLocal() {
  for (auto& hq_cross_walk : *hq_map_->mutable_crosswalk()) {
    for (auto& pt : *hq_cross_walk.mutable_polygon()->mutable_point()) {
      Eigen::Vector3d pt_local(pt.x(), pt.y(), pt.z());
      pt_local = T_local_enu_to_local_ * pt_local;
      pt.set_x(pt_local.x());
      pt.set_y(pt_local.y());
      pt.set_z(0);
    }
  }
}

void MapPrediction::FitLaneCenterline() {
  // 拟合车道中心线
  if (!local_msg_) {
    HLOG_ERROR << "!no local msg";
    return;
  }
  if (local_msg_->lane().empty()) {
    HLOG_ERROR << "!no local msg lane";
    return;
  }
  for (auto& lane : *local_msg_->mutable_lane()) {
    std::vector<Vec2d> left_point;
    std::vector<Vec2d> right_point;
    // 存储左右边线
    StoreLaneline(lane, &left_point, &right_point);
    std::vector<Vec2d> cent_points;
    if (left_point.empty() || right_point.empty()) {
      continue;
    }
    cent_points.clear();
    common::math::GenerateCenterPoint(left_point, right_point, &cent_points);

    viz_map_.VizCenterLane(cent_points);

    // 此时获得了中心线上的点,塞入local_msg_中
    InsertCenterPoint(&lane, cent_points);
  }
}

void MapPrediction::InsertCenterPoint(hozon::hdmap::Lane* lane,
                                      const std::vector<Vec2d>& cent_points) {
  if (cent_points.empty()) {
    return;
  }

  auto* seg = lane->mutable_central_curve()->add_segment();
  for (const auto& cen_point : cent_points) {
    if (std::isnan(cen_point.x()) || std::isnan(cen_point.y())) {
      HLOG_ERROR << "x or y isnan!";
      continue;
    }
    auto* pt = seg->mutable_line_segment()->add_point();
    pt->set_x(cen_point.x());
    pt->set_y(cen_point.y());
    pt->set_z(0.);
  }
}

void MapPrediction::StoreLaneline(const hozon::hdmap::Lane& lane,
                                  std::vector<Vec2d>* left_point,
                                  std::vector<Vec2d>* right_point) {
  // 存储左
  for (const auto& seg : lane.left_boundary().curve().segment()) {
    for (const auto& point : seg.line_segment().point()) {
      Vec2d point_enu(point.x(), point.y());
      if (!left_point->empty() &&
          std::find(left_point->begin(), left_point->end(), point_enu) !=
              left_point->end()) {
        continue;
      }
      left_point->emplace_back(point_enu);
    }
  }
  // 存储右
  for (const auto& seg : lane.right_boundary().curve().segment()) {
    for (const auto& point : seg.line_segment().point()) {
      Vec2d point_enu(point.x(), point.y());
      if (!right_point->empty() &&
          std::find(right_point->begin(), right_point->end(), point_enu) !=
              right_point->end()) {
        continue;
      }
      right_point->emplace_back(point_enu);
    }
  }
}

void MapPrediction::CheckLocalLane(std::set<std::string>* side_miss_ids) {
  // 检查原始的local_map中线是否完整(先暂时判断左右是否缺失)
  for (auto& lane : *local_msg_->mutable_lane()) {
    auto lane_id = lane.id().id();
    if (lane_table_.find(lane_id) == lane_table_.end()) {
      continue;
    }
    auto left_seg_size = lane.left_boundary().curve().segment_size();
    auto right_seg_size = lane.right_boundary().curve().segment_size();
    auto left_points = lane_table_.at(lane_id).pred_left_line;
    auto* left_seg =
        lane.mutable_left_boundary()->mutable_curve()->add_segment();
    if (left_seg_size == 0) {
      for (const auto& point : left_points) {
        auto* pt = left_seg->mutable_line_segment()->add_point();
        pt->set_x(point.x());
        pt->set_y(point.y());
        pt->set_z(point.z());
      }
    }

    auto right_points = lane_table_.at(lane_id).pred_right_line;
    auto* right_seg =
        lane.mutable_right_boundary()->mutable_curve()->add_segment();
    if (right_seg_size == 0) {
      for (const auto& point : right_points) {
        auto* pt = right_seg->mutable_line_segment()->add_point();
        pt->set_x(point.x());
        pt->set_y(point.y());
        pt->set_z(point.z());
      }
    }

    auto sec_id = lane_table_.at(lane_id).section_id;
    auto road_id = lane_table_.at(lane_id).road_id;
    if (road_table_.find(road_id) == road_table_.end()) {
      continue;
    }
    auto sec_lane_ids = road_table_.at(road_id).section_ids.at(sec_id).lane_id;
    for (const auto& it : sec_lane_ids) {
      if (std::find(topo_lane_ids_.begin(), topo_lane_ids_.end(), it) !=
          topo_lane_ids_.end()) {
        continue;
      }
      side_miss_ids->insert(it);
    }
  }
}

void MapPrediction::PredLeftRight(const std::set<std::string>& side_miss_ids) {
  // 预测左右缺失
  for (const auto& it : side_miss_ids) {
    if (lane_table_.find(it) == lane_table_.end()) {
      continue;
    }
    auto left_points = lane_table_.at(it).pred_left_line;
    auto right_points = lane_table_.at(it).pred_right_line;
    // 创建新的lane
    auto* new_lane = local_msg_->add_lane();
    new_lane->mutable_id()->set_id(it);
    auto* left_seg =
        new_lane->mutable_left_boundary()->mutable_curve()->add_segment();
    for (const auto& point : left_points) {
      auto* pt = left_seg->mutable_line_segment()->add_point();
      pt->set_x(point.x());
      pt->set_y(point.y());
      pt->set_z(point.z());
    }

    auto* right_seg =
        new_lane->mutable_right_boundary()->mutable_curve()->add_segment();
    for (const auto& point : right_points) {
      auto* pt = right_seg->mutable_line_segment()->add_point();
      pt->set_x(point.x());
      pt->set_y(point.y());
      pt->set_z(point.z());
    }
    topo_lane_ids_.emplace_back(it);
  }
}

void MapPrediction::PredAheadLanes() {
  // 预测前方缺失车道线
  if (add_lane_ids_.empty()) {
    return;
  }
  for (const auto& add_id : add_lane_ids_) {
    if (lane_table_.find(add_id) == lane_table_.end()) {
      continue;
    }
    auto sec_id = lane_table_.at(add_id).section_id;
    auto road_id = lane_table_.at(add_id).road_id;
    if (road_table_.find(road_id) == road_table_.end()) {
      continue;
    }
    auto sec_lane_ids = road_table_.at(road_id).section_ids.at(sec_id).lane_id;
    for (const auto& it : sec_lane_ids) {
      if (std::find(topo_lane_ids_.begin(), topo_lane_ids_.end(), it) !=
          topo_lane_ids_.end()) {
        continue;
      }
      if (lane_table_.find(it) == lane_table_.end()) {
        continue;
      }
      auto left_points = lane_table_.at(it).pred_left_line;
      auto right_points = lane_table_.at(it).pred_right_line;
      // 创建新的lane
      auto* new_lane = local_msg_->add_lane();
      new_lane->mutable_id()->set_id(it);
      auto* left_seg =
          new_lane->mutable_left_boundary()->mutable_curve()->add_segment();
      for (const auto& point : left_points) {
        auto* pt = left_seg->mutable_line_segment()->add_point();
        pt->set_x(point.x());
        pt->set_y(point.y());
        pt->set_z(point.z());
      }

      auto* right_seg =
          new_lane->mutable_right_boundary()->mutable_curve()->add_segment();
      for (const auto& point : right_points) {
        auto* pt = right_seg->mutable_line_segment()->add_point();
        pt->set_x(point.x());
        pt->set_y(point.y());
        pt->set_z(point.z());
      }
      topo_lane_ids_.emplace_back(it);
    }
  }
}

#if 0
void MapPrediction::PredLocalRoads() {
  // 预测当前local_msg_中缺失的road
  for (auto& road : *local_msg_->mutable_road()) {
    for (auto& sec : *road.mutable_section()) {
      for (auto& edge :
           *sec.mutable_boundary()->mutable_outer_polygon()->mutable_edge()) {
        if (edge.type() == 2) {
          // left boundary
          AddLeftRoadBoundary(&edge, road.id().id(), sec.id().id());
        }

        if (edge.type() == 3) {
          // right boundary
          AddRightRoadBoundary(&edge, road.id().id(), sec.id().id());
        }
      }
    }
  }
}
#endif

#if 0
void MapPrediction::AddLeftRoadBoundary(hozon::hdmap::BoundaryEdge* edge,
                                        const std::string& road_id,
                                        const std::string& sec_id) {
  // add left boundary
  auto seg_size = edge->curve().segment().size();
  if (seg_size != 0) {
    return;
  }
  // seg_size == 0
  if (road_table_.find(road_id) == road_table_.end()) {
    return;
  }
  auto section = road_table_.at(road_id).section_ids;
  auto left_boundary = section.at(sec_id).left_boundary;
  auto new_seg = *edge->mutable_curve()->add_segment();
  for (const auto& pt : left_boundary) {
    auto ptt = *new_seg.mutable_line_segment()->add_point();
    ptt.set_x(pt.x());
    ptt.set_y(pt.y());
    ptt.set_z(pt.z());
  }
}
#endif

#if 0
void MapPrediction::AddRightRoadBoundary(hozon::hdmap::BoundaryEdge* edge,
                                         const std::string& road_id,
                                         const std::string& sec_id) {
  // add right boundary
  auto seg_size = edge->curve().segment().size();
  if (seg_size != 0) {
    return;
  }
  // seg_size == 0
  if (road_table_.find(road_id) == road_table_.end()) {
    return;
  }
  auto section = road_table_.at(road_id).section_ids;
  auto right_boundary = section.at(sec_id).right_boundary;
  auto new_seg = *edge->mutable_curve()->add_segment();
  for (const auto& pt : right_boundary) {
    auto ptt = *new_seg.mutable_line_segment()->add_point();
    ptt.set_x(pt.x());
    ptt.set_y(pt.y());
    ptt.set_z(pt.z());
  }
}
#endif

#if 0
void MapPrediction::PredAheadRoads() {
  // 预测前方道路边界
  if (add_lane_ids_.empty()) {
    return;
  }

  for (const auto& it : add_lane_ids_) {
    if (lane_table_.find(it) == lane_table_.end()) {
      continue;
    }
    auto road_id = lane_table_.at(it).road_id;
    auto section_id = lane_table_.at(it).section_id;
    if (road_table_.find(road_id) == road_table_.end()) {
      continue;
    }
    auto section = road_table_.at(road_id).section_ids;
    if (std::find(topo_section_ids_.begin(), topo_section_ids_.end(),
                  section_id) != topo_section_ids_.end()) {
      continue;
    }
    auto left_boundary = section.at(section_id).left_boundary;
    auto right_boundary = section.at(section_id).right_boundary;
    // local_msg_中添加新的boundary元素
    bool flag = false;
    AddAheadEdgeWithSameId(left_boundary, right_boundary, road_id, section_id,
                           section.at(section_id).lane_id, &flag);
    if (flag) {
      continue;
    }
    AddAheadEdgeWithNewId(left_boundary, right_boundary, road_id, section_id,
                          section.at(section_id).lane_id);
  }
}
#endif

#if 0
void MapPrediction::AddAheadEdgeWithSameId(
    const std::vector<Eigen::Vector3d>& left_boundary,
    const std::vector<Eigen::Vector3d>& right_boundary,
    const std::string& road_id, const std::string& section_id,
    const std::vector<std::string>& lane_ids, bool* flag) {
  // has same road id
  for (auto& road : *local_msg_->mutable_road()) {
    if (road.id().id() != road_id) {
      continue;
    }
    auto new_section = *road.add_section();
    new_section.mutable_id()->set_id(section_id);
    for (const auto& id : lane_ids) {
      new_section.add_lane_id()->set_id(id);
    }
    auto new_edge_left =
        *new_section.mutable_boundary()->mutable_outer_polygon()->add_edge();
    hozon::hdmap::BoundaryEdge_Type valuel =
        static_cast<hozon::hdmap::BoundaryEdge_Type>(2);
    new_edge_left.set_type(valuel);
    auto seg_l = *new_edge_left.mutable_curve()->add_segment();
    for (const auto& it : left_boundary) {
      auto pt = *seg_l.mutable_line_segment()->add_point();
      pt.set_x(it.x());
      pt.set_y(it.y());
      pt.set_z(it.z());
    }

    auto new_edge_right =
        *new_section.mutable_boundary()->mutable_outer_polygon()->add_edge();
    hozon::hdmap::BoundaryEdge_Type valuer =
        static_cast<hozon::hdmap::BoundaryEdge_Type>(3);
    new_edge_left.set_type(valuer);
    auto seg_r = *new_edge_right.mutable_curve()->add_segment();
    for (const auto& it : right_boundary) {
      auto pt = *seg_r.mutable_line_segment()->add_point();
      pt.set_x(it.x());
      pt.set_y(it.y());
      pt.set_z(it.z());
    }
    *flag = true;
    topo_section_ids_.insert(section_id);
    break;
  }
}
#endif

#if 0
void MapPrediction::AddAheadEdgeWithNewId(
    const std::vector<Eigen::Vector3d>& left_boundary,
    const std::vector<Eigen::Vector3d>& right_boundary,
    const std::string& road_id, const std::string& section_id,
    const std::vector<std::string>& lane_ids) {
  auto new_road = *local_msg_->add_road();
  new_road.mutable_id()->set_id(road_id);
  auto new_section = *new_road.add_section();
  new_section.mutable_id()->set_id(section_id);
  for (const auto& id : lane_ids) {
    new_section.add_lane_id()->set_id(id);
  }
  auto new_edge_left =
      *new_section.mutable_boundary()->mutable_outer_polygon()->add_edge();
  hozon::hdmap::BoundaryEdge_Type valuel =
      static_cast<hozon::hdmap::BoundaryEdge_Type>(2);
  new_edge_left.set_type(valuel);
  auto seg_l = *new_edge_left.mutable_curve()->add_segment();
  for (const auto& it : left_boundary) {
    auto pt = *seg_l.mutable_line_segment()->add_point();
    pt.set_x(it.x());
    pt.set_y(it.y());
    pt.set_z(it.z());
  }

  auto new_edge_right =
      *new_section.mutable_boundary()->mutable_outer_polygon()->add_edge();
  hozon::hdmap::BoundaryEdge_Type valuer =
      static_cast<hozon::hdmap::BoundaryEdge_Type>(3);
  new_edge_left.set_type(valuer);
  auto seg_r = *new_edge_right.mutable_curve()->add_segment();
  for (const auto& it : right_boundary) {
    auto pt = *seg_r.mutable_line_segment()->add_point();
    pt.set_x(it.x());
    pt.set_y(it.y());
    pt.set_z(it.z());
  }
  topo_section_ids_.insert(section_id);
}
#endif

void MapPrediction::Prediction() {
  //    std::lock_guard<std::mutex> lock(mtx_);
  util::TicToc global_tic;
  util::TicToc local_tic;
  // 对原始local_msg_中的车道线元素与地图中的车道线元素进行融合
  FusionLocalAndMap();

  CompleteLaneline(end_lane_ids_, end_section_ids_, road_table_);
  HLOG_INFO << "pred Prediction CompleteLaneline cost " << local_tic.Toc();
  local_tic.Tic();

  std::set<std::string> side_miss_ids;
  CheckLocalLane(&side_miss_ids);
  HLOG_INFO << "pred Prediction CheckLocalLane cost " << local_tic.Toc();
  local_tic.Tic();

  PredLeftRight(side_miss_ids);
  HLOG_INFO << "pred Prediction PredLeftRight cost " << local_tic.Toc();
  local_tic.Tic();

  PredAheadLanes();
  HLOG_INFO << "pred Prediction PredAheadLanes cost " << local_tic.Toc();

  // PredLocalRoads();
  // HLOG_INFO << "pred Prediction PredLocalRoads cost " << local_tic.Toc();

  // PredAheadRoads();
  // HLOG_INFO << "pred Prediction PredAheadRoads cost " << local_tic.Toc();

  local_tic.Tic();
  FitLaneCenterline();
  HLOG_INFO << "pred Prediction FitLaneCenterline cost " << local_tic.Toc();

  local_tic.Tic();
  viz_map_.VizLocalMapLaneLine(local_msg_);
  viz_map_.VizLaneID(local_msg_);

  AddResTopo();
  HLOG_INFO << "pred Prediction AddResTopo cost " << local_tic.Toc();
  local_tic.Tic();

  VizLocAndHqMap();
  HLOG_INFO << "pred Prediction VizLocAndHqMap cost " << local_tic.Toc();
  local_tic.Tic();

  auto global_cost = global_tic.Toc();
  HLOG_INFO << "pred Prediction cost " << global_cost;
}

void MapPrediction::VizLocAndHqMap() {
  if (!FLAGS_viz_odom_map_in_local) {
    return;
  }

  if (!RVIZ_AGENT.Ok()) {
    HLOG_ERROR << "RvizAgent not ok";
    return;
  }

  if (!RVIZ_AGENT.Registered(FLAGS_viz_topic_odom_in_local)) {
    int ret = RVIZ_AGENT.Register<adsfi_proto::viz::Odometry>(
        FLAGS_viz_topic_odom_in_local);
    if (ret < 0) {
      HLOG_ERROR << "register " << FLAGS_viz_topic_odom_in_local << " failed";
      return;
    }
  }
  if (!RVIZ_AGENT.Registered(FLAGS_viz_topic_map_in_local)) {
    int ret = RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(
        FLAGS_viz_topic_map_in_local);
    if (ret < 0) {
      HLOG_ERROR << "register " << FLAGS_viz_topic_map_in_local << " failed";
      return;
    }
  }

  const double kMarkerLifetime = 0.1;
  // uint32_t lifetime_sec = 0;
  // uint32_t lifetime_nsec = 0;
  SplitSecs(kMarkerLifetime, &lifetime_sec_, &lifetime_nsec_);

  adsfi_proto::viz::Odometry odom;
  odom.mutable_header()->set_frameid(kLocalFrameId_);
  // uint32_t sec = 0;
  // uint32_t nsec = 0;
  SplitSecs(stamp_loc_, &sec_, &nsec_);
  odom.mutable_header()->mutable_timestamp()->set_sec(sec_);
  odom.mutable_header()->mutable_timestamp()->set_nsec(nsec_);
  odom.mutable_pose()->mutable_pose()->mutable_position()->set_x(
      pos_local_.x());
  odom.mutable_pose()->mutable_pose()->mutable_position()->set_y(
      pos_local_.y());
  odom.mutable_pose()->mutable_pose()->mutable_position()->set_z(
      pos_local_.z());
  odom.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(
      quat_local_.w());
  odom.mutable_pose()->mutable_pose()->mutable_orientation()->set_x(
      quat_local_.x());
  odom.mutable_pose()->mutable_pose()->mutable_orientation()->set_y(
      quat_local_.y());
  odom.mutable_pose()->mutable_pose()->mutable_orientation()->set_z(
      quat_local_.z());

  RVIZ_AGENT.Publish(FLAGS_viz_topic_odom_in_local, odom);

  if (!hq_map_) {
    HLOG_ERROR << "nullptr hq_maq_";
    return;
  }

  adsfi_proto::viz::MarkerArray ma;
  DealHqLine(&ma);

  if (!ma.markers().empty()) {
    RVIZ_AGENT.Publish(FLAGS_viz_topic_map_in_local, ma);
  }
}

void MapPrediction::DealHqLine(adsfi_proto::viz::MarkerArray* ma) {
  for (const auto& hq_lane : hq_map_->lane()) {
    std::string ns = hq_lane.id().id();
    std::vector<Eigen::Vector3d> center_pts;
    for (const auto& seg : hq_lane.central_curve().segment()) {
      for (const auto& pt : seg.line_segment().point()) {
        Eigen::Vector3d cpt(pt.x(), pt.y(), pt.z());
        center_pts.emplace_back(cpt);
      }
    }

    std::vector<Eigen::Vector3d> left_pts;
    for (const auto& seg : hq_lane.left_boundary().curve().segment()) {
      for (const auto& pt : seg.line_segment().point()) {
        Eigen::Vector3d lpt(pt.x(), pt.y(), pt.z());
        left_pts.emplace_back(lpt);
      }
    }
    std::vector<Eigen::Vector3d> right_pts;
    for (const auto& seg : hq_lane.right_boundary().curve().segment()) {
      for (const auto& pt : seg.line_segment().point()) {
        Eigen::Vector3d rpt(pt.x(), pt.y(), pt.z());
        right_pts.emplace_back(rpt);
      }
    }

    LineToMarker(center_pts, left_pts, right_pts, ma, ns);
  }
}

void MapPrediction::LineToMarker(const std::vector<Eigen::Vector3d>& center_pts,
                                 const std::vector<Eigen::Vector3d>& left_pts,
                                 const std::vector<Eigen::Vector3d>& right_pts,
                                 adsfi_proto::viz::MarkerArray* ma,
                                 const std::string& ns) {
  // 可视化
  if (!center_pts.empty()) {
    auto* marker = ma->add_markers();
    marker->mutable_header()->set_frameid(kLocalFrameId_);
    marker->mutable_header()->mutable_timestamp()->set_sec(sec_);
    marker->mutable_header()->mutable_timestamp()->set_nsec(nsec_);
    std::string line_ns = ns + "/center_line";
    marker->set_ns(line_ns);
    marker->set_id(0);
    marker->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
    marker->mutable_pose()->mutable_orientation()->set_x(0.);
    marker->mutable_pose()->mutable_orientation()->set_y(0.);
    marker->mutable_pose()->mutable_orientation()->set_z(0.);
    marker->mutable_pose()->mutable_orientation()->set_w(1.);
    marker->mutable_scale()->set_x(0.3);
    marker->mutable_lifetime()->set_sec(lifetime_sec_);
    marker->mutable_lifetime()->set_nsec(lifetime_nsec_);
    marker->mutable_color()->set_a(0.8);
    marker->mutable_color()->set_r(1.0);
    marker->mutable_color()->set_g(1.0);
    marker->mutable_color()->set_b(1.0);
    marker->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);

    for (const auto& pt : center_pts) {
      auto* mpt = marker->add_points();
      mpt->set_x(pt.x());
      mpt->set_y(pt.y());
      mpt->set_z(pt.z());
    }
  }

  if (!left_pts.empty()) {
    auto* marker = ma->add_markers();
    marker->mutable_header()->set_frameid(kLocalFrameId_);
    marker->mutable_header()->mutable_timestamp()->set_sec(sec_);
    marker->mutable_header()->mutable_timestamp()->set_nsec(nsec_);
    std::string line_ns = ns + "/left_line";
    marker->set_ns(line_ns);
    marker->set_id(0);
    marker->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
    marker->mutable_pose()->mutable_orientation()->set_x(0.);
    marker->mutable_pose()->mutable_orientation()->set_y(0.);
    marker->mutable_pose()->mutable_orientation()->set_z(0.);
    marker->mutable_pose()->mutable_orientation()->set_w(1.);
    marker->mutable_scale()->set_x(0.2);
    marker->mutable_lifetime()->set_sec(lifetime_sec_);
    marker->mutable_lifetime()->set_nsec(lifetime_nsec_);
    marker->mutable_color()->set_a(1.0);
    marker->mutable_color()->set_r(0.0);
    marker->mutable_color()->set_g(1.0);
    marker->mutable_color()->set_b(0.0);
    marker->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);

    for (const auto& pt : left_pts) {
      auto* mpt = marker->add_points();
      mpt->set_x(pt.x());
      mpt->set_y(pt.y());
      mpt->set_z(pt.z());
    }
  }

  if (!right_pts.empty()) {
    auto* marker = ma->add_markers();
    marker->mutable_header()->set_frameid(kLocalFrameId_);
    marker->mutable_header()->mutable_timestamp()->set_sec(sec_);
    marker->mutable_header()->mutable_timestamp()->set_nsec(nsec_);
    std::string line_ns = ns + "/right_line";
    marker->set_ns(line_ns);
    marker->set_id(0);
    marker->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
    marker->mutable_pose()->mutable_orientation()->set_x(0.);
    marker->mutable_pose()->mutable_orientation()->set_y(0.);
    marker->mutable_pose()->mutable_orientation()->set_z(0.);
    marker->mutable_pose()->mutable_orientation()->set_w(1.);
    marker->mutable_scale()->set_x(0.2);
    marker->mutable_lifetime()->set_sec(lifetime_sec_);
    marker->mutable_lifetime()->set_nsec(lifetime_nsec_);
    marker->mutable_color()->set_a(1.0);
    marker->mutable_color()->set_r(0.0);
    marker->mutable_color()->set_g(1.0);
    marker->mutable_color()->set_b(0.0);
    marker->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);

    for (const auto& pt : right_pts) {
      auto* mpt = marker->add_points();
      mpt->set_x(pt.x());
      mpt->set_y(pt.y());
      mpt->set_z(pt.z());
    }
  }
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
