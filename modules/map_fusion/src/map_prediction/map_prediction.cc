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
#include "common/math/line_segment2d.h"
#include "common/math/vec2d.h"
#include "common/time/clock.h"
#include "common/time/time.h"
#include "common/utm_projection/coordinate_convertor.h"
#include "map/hdmap/hdmap.h"
#include "map/hdmap/hdmap_common.h"
#include "map_fusion/map_fusion.h"
#include "modules/util/include/util/geo.h"

// #include "util/log.h"

#include "map_fusion/map_service/global_hd_map.h"
#include "opencv2/core/matx.hpp"
#include "proto/map/map.pb.h"
#include "proto/map/map_id.pb.h"
#include "proto/map/map_lane.pb.h"
#include "proto/map/map_road.pb.h"
#include "util/rate.h"
#include "util/rviz_agent/rviz_agent.h"
#include "util/temp_log.h"
#include "util/tic_toc.h"

// NOLINTBEGIN
// DEFINE_bool(pred_run, true, "pred thread run");
// DEFINE_uint32(pred_thread_interval, 100, "pred thread interval ms");
DEFINE_bool(viz_odom_map_in_local, false,
            "whether publish viz msgs of odometry and map in local frame");
DEFINE_string(viz_topic_odom_in_local, "/mf/pred/odom_local",
              "viz topic of odometry in local frame");
DEFINE_string(viz_topic_map_in_local, "/mf/pred/map_local",
              "viz topic of map in local frame");
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
  OnLocationInGlobal(gcj02, zone, y, x);
}

void MapPrediction::OnLocalization(
    const std::shared_ptr<hozon::localization::Localization>& msg) {
  // 提取全局定位
  Eigen::Vector3d pos_global_utm(msg->pose().pos_utm_01().x(),
                                 msg->pose().pos_utm_01().y(),
                                 msg->pose().pos_utm_01().z());

  // utm转gcj02
  double utm_x = msg->pose().pos_utm_01().x();
  double utm_y = msg->pose().pos_utm_01().y();
  uint32_t zone_t = msg->pose().utm_zone_01();
  int zone = static_cast<int>(zone_t);
  bool ret = hozon::common::coordinate_convertor::UTM2GCS(zone, &utm_x, &utm_y);
  if (!ret) {
    HLOG_ERROR << "UTM2GCS failed";
    return;
  }

  // gcj02
  Eigen::Vector3d pos_global(utm_y, utm_x, 0);
  OnLocationInGlobal(pos_global, zone, msg->pose().pos_utm_01().x(),
                     msg->pose().pos_utm_01().y());

  stamp_loc_ = msg->header().gnss_stamp();
  pos_local_ << msg->pose().local_pose().x(), msg->pose().local_pose().y(),
      msg->pose().local_pose().z();
  auto yaw = msg->pose().euler_angles_local().z();
  auto roll = msg->pose().euler_angles_local().x();
  auto pitch = msg->pose().euler_angles_local().y();
  quat_local_ = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());

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
  roll = msg->pose().euler_angles().x();
  pitch = msg->pose().euler_angles().y();
  Eigen::Quaterniond quat_enu =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());

  Eigen::Isometry3d T_veh_to_local_enu;
  T_veh_to_local_enu.setIdentity();
  T_veh_to_local_enu.rotate(quat_enu);
  T_veh_to_local_enu.pretranslate(pos_local_enu);

  T_local_enu_to_local_.setIdentity();
  T_local_enu_to_local_ = T_veh_to_local * T_veh_to_local_enu.inverse();
}

void MapPrediction::OnLocationInGlobal(const Eigen::Vector3d& pos_gcj02,
                                       uint32_t utm_zone_id, double utm_x,
                                       double utm_y) {
  std::lock_guard<std::mutex> lock(mtx_);
  location_ = pos_gcj02;
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

void MapPrediction::OnTopoMap(const std::shared_ptr<hozon::hdmap::Map>& msg) {
  // std::lock_guard<std::mutex> lock(mtx_);
  if (!msg) {
    HLOG_ERROR << "nullptr topo map";
    return;
  }

  localmap_lanelines.clear();
  if (msg->lane().empty()) {
    HLOG_ERROR << "no OnTopoMap info!";
    return;
  }

  hozon::common::PointENU utm_pos;
  // uint32_t utm_zone = 0;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (local_enu_center_flag_ && !msg->header().id().empty()) {
      bool parsed = init_pose_.ParseFromString(msg->header().id());
      if (!parsed) {
        HLOG_ERROR << "parse init pose failed";
        return;
      }
      local_enu_center_ << init_pose_.gcj02().x(), init_pose_.gcj02().y(),
          init_pose_.gcj02().z();

      HLOG_ERROR << "init_pose_:\n" << init_pose_.DebugString();

      local_enu_center_flag_ = false;
    }

    //    local_msg_ = msg;
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
    const double range = 300.;
    std::vector<hozon::hdmap::LaneInfoConstPtr> lanes_in_range;
    if (!GLOBAL_HD_MAP) {
      HLOG_ERROR << "nullptr hq_map_server_";
      return;
    }
    int ret = GLOBAL_HD_MAP->GetLanes(utm_pos, range, &lanes_in_range);
    if (ret != 0) {
      HLOG_ERROR << "get local map lanes failed";
      return;
    }
    HLOG_INFO << "pred OnTopoMap GetLanes cost " << local_tic.Toc();
    local_tic.Tic();
    std::vector<hozon::hdmap::RoadInfoConstPtr> roads_in_range;
    ret = GLOBAL_HD_MAP->GetRoads(utm_pos, range, &roads_in_range);
    if (ret != 0) {
      HLOG_ERROR << "get local map roads failed";
      return;
    }
    HLOG_INFO << "pred OnTopoMap GetRoads cost " << local_tic.Toc();
    local_tic.Tic();

    hq_map_ = std::make_shared<hozon::hdmap::Map>();
    GLOBAL_HD_MAP->GetMapWithoutLaneGeometry(hq_map_.get());
    hq_map_->mutable_header()->CopyFrom(msg->header());
    hq_map_->mutable_header()->mutable_id()->clear();
    HLOG_INFO << "pred OnTopoMap GetMapWithoutLaneGeometry cost "
              << local_tic.Toc();
    local_tic.Tic();

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

    CreatLaneTable(lanes_in_range);
    CreatRoadTable(roads_in_range);
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

void MapPrediction::CreatLaneTable(
    const std::vector<hozon::hdmap::LaneInfoConstPtr>& lanes_in_range) {
  // 创建lane_table_
  for (const auto& lane : lanes_in_range) {
    const auto& lane_id = lane->id().id();
    LocalLane local_lane;
    local_lane.lane_id = lane_id;
    local_lane.road_id = lane->road_id().id();
    local_lane.section_id = lane->section_id().id();
    all_section_ids_.emplace_back(lane->section_id().id());

    // if (lane->lane().left_neighbor_forward_lane_id().empty()) {
    //   local_lane.flag = LocalLane::FAR_LEFT;
    // } else if (lane->lane().right_neighbor_forward_lane_id().empty()) {
    //   local_lane.flag = LocalLane::FAR_RIGHT;
    // } else {
    //   local_lane.flag = LocalLane::MIDDLE;
    // }

    // 存lane左右边线
    for (const auto& it : lane->lane().left_boundary().curve().segment()) {
      for (const auto& itt : it.line_segment().point()) {
        Eigen::Vector3d pt = UtmPtToLocalEnu(itt);
        local_lane.left_line.emplace_back(pt);
      }
    }
    for (const auto& it : lane->lane().right_boundary().curve().segment()) {
      for (const auto& itt : it.line_segment().point()) {
        Eigen::Vector3d pt = UtmPtToLocalEnu(itt);
        local_lane.right_line.emplace_back(pt);
      }
    }

    for (const auto& it : lane->lane().left_neighbor_forward_lane_id()) {
      local_lane.left_lane_ids.emplace_back(it.id());
    }
    for (const auto& it : lane->lane().right_neighbor_forward_lane_id()) {
      local_lane.right_lane_ids.emplace_back(it.id());
    }
    for (const auto& it : lane->lane().predecessor_id()) {
      local_lane.prev_lane_ids.emplace_back(it.id());
    }
    for (const auto& it : lane->lane().successor_id()) {
      local_lane.next_lane_ids.emplace_back(it.id());
    }
    // 存储lane的宽度
    double length = lane->lane().length();
    double s = 0.;
    std::vector<double> lane_width;
    while (s < length) {
      double width = lane->GetWidth(s);
      lane_width.push_back(width);
      s += 0.5;
    }
    double end_width = lane->GetWidth(length);
    lane_width.push_back(end_width);
    local_lane.lane_width = lane_width;
    lane_table_.insert_or_assign(lane_id, local_lane);
  }
}

void MapPrediction::CreatRoadTable(
    const std::vector<hozon::hdmap::RoadInfoConstPtr>& roads_in_range) {
  // 创建road_table_
  for (const auto& road : roads_in_range) {
    const auto& road_id = road->id().id();
    LocalRoad local_road;
    local_road.road_id = road_id;
    for (const auto& it : road->sections()) {
      if (std::find(all_section_ids_.begin(), all_section_ids_.end(),
                    it.id().id()) == all_section_ids_.end()) {
        continue;
      }
      Boundary section;
      section.section_id = it.id().id();
      for (const auto& itt : it.lane_id()) {
        section.lane_id.emplace_back(itt.id());
      }
      // 将左二车道的左边界作为road_boundary
      // std::vector<std::vector<Eigen::Vector3d>> predict_lanelines;
      const auto& left_second_left_id = section.lane_id.back();

      if (lane_table_.find(left_second_left_id) != lane_table_.end()) {
        section.road_boundary = lane_table_.at(left_second_left_id).right_line;
      }

      // predict_lanelines.emplace_back(section.road_boundary);
      // viz_map_.VizCompanLane(predict_lanelines);

      // for (const auto& edge : it.boundary().outer_polygon().edge()) {
      //   if (edge.type() == 2) {  // left
      //     std::vector<Eigen::Vector3d> edge_point;
      //     for (const auto& seg : edge.curve().segment()) {
      //       for (const auto& point : seg.line_segment().point()) {
      //         Eigen::Vector3d point_enu = UtmPtToLocalEnu(point);
      //         edge_point.emplace_back(point_enu);
      //       }
      //     }
      //     if (!edge_point.empty()) {
      //       viz_map_.VizHqMapRoad(edge_point);
      //       section.left_boundary.emplace_back(edge_point);
      //     }
      //   }

      //   if (edge.type() == 3) {  // right
      //     std::vector<Eigen::Vector3d> edge_point;
      //     for (const auto& seg : edge.curve().segment()) {
      //       for (const auto& point : seg.line_segment().point()) {
      //         Eigen::Vector3d point_enu = UtmPtToLocalEnu(point);
      //         edge_point.emplace_back(point_enu);
      //       }
      //     }
      //     if (!edge_point.empty()) {
      //       viz_map_.VizHqMapRoad(edge_point);
      //       section.right_boundary.emplace_back(edge_point);
      //     }
      //   }
      // }
      local_road.section_ids.insert_or_assign(section.section_id, section);
    }
    road_table_.insert_or_assign(road_id, local_road);
  }
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
    CreateFarTable(it, local_lane);
  }
}

void MapPrediction::CreatendIdVector(const hozon::hdmap::Lane& it,
                                     std::string* lane_id,
                                     const LocalLane& local_lane) {
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

void MapPrediction::CreateFarTable(const hozon::hdmap::Lane& it,
                                   const LocalLane& local_lane) {
  // 创建far_table_
  const auto& left_seg = it.left_boundary().curve().segment_size();
  const auto& right_seg = it.right_boundary().curve().segment_size();
  auto road_id = local_lane.road_id;
  auto section_id = local_lane.section_id;
  if (road_table_.find(road_id) == road_table_.end()) {
    HLOG_ERROR << "road not found in road_table";
    return;
  }
  const auto& local_road = road_table_.at(road_id);
  if (local_road.section_ids.empty()) {
    HLOG_ERROR << "empty section ids";
    return;
  }
  if (local_road.section_ids.find(section_id) == local_road.section_ids.end()) {
    HLOG_ERROR << "section not found in section ids";
    return;
  }

  bool far_left = false;
  bool far_right = false;
  if (it.id().id() == local_road.section_ids.at(section_id).lane_id.back() &&
      left_seg != 0) {
    far_left = true;
  }
  if (it.id().id() == local_road.section_ids.at(section_id).lane_id.front() &&
      right_seg != 0) {
    far_right = true;
  }

  if ((it.left_neighbor_forward_lane_id().empty() &&
       !it.right_neighbor_forward_lane_id().empty() && !far_left) ||
      (!it.left_neighbor_forward_lane_id().empty() && left_seg == 0)) {
    const auto& lane_id = it.id().id();
    FarLane far_lane;
    far_lane.lane_id = lane_id;
    far_lane.flag = 0;
    CreateLaneline(it, &far_lane);
    far_table_.insert_or_assign(lane_id, far_lane);
  }
  if ((it.right_neighbor_forward_lane_id().empty() &&
       !it.left_neighbor_forward_lane_id().empty() && !far_right) ||
      (!it.right_neighbor_forward_lane_id().empty() && right_seg == 0)) {
    const auto& lane_id = it.id().id();
    FarLane far_lane;
    far_lane.lane_id = lane_id;
    far_lane.flag = 1;
    CreateLaneline(it, &far_lane);
    far_table_.insert_or_assign(lane_id, far_lane);
  }

  if (it.right_neighbor_forward_lane_id().empty() &&
      it.left_neighbor_forward_lane_id().empty()) {
    const auto& lane_id = it.id().id();
    FarLane far_lane;
    far_lane.lane_id = lane_id;
    far_lane.flag = 2;
    CreateLaneline(it, &far_lane);
    far_table_.insert_or_assign(lane_id, far_lane);
  }
}

void MapPrediction::CreateLaneline(const hozon::hdmap::Lane& it,
                                   FarLane* far_lane) {
  // 存储车道的两个边线
  for (const auto& itt : it.left_boundary().curve().segment()) {
    for (const auto& point : itt.line_segment().point()) {
      Eigen::Vector3d left_line(point.x(), point.y(), point.z());
      if (!far_lane->left_line.empty() &&
          std::find(far_lane->left_line.begin(), far_lane->left_line.end(),
                    left_line) != far_lane->left_line.end()) {
        continue;
      }
      far_lane->left_line.emplace_back(left_line);
    }
  }
  for (const auto& itt : it.right_boundary().curve().segment()) {
    for (const auto& point : itt.line_segment().point()) {
      Eigen::Vector3d right_line(point.x(), point.y(), point.z());
      if (!far_lane->right_line.empty() &&
          std::find(far_lane->right_line.begin(), far_lane->right_line.end(),
                    right_line) != far_lane->right_line.end()) {
        continue;
      }
      far_lane->right_line.emplace_back(right_line);
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

void MapPrediction::PredictLeftRightLaneline(
    const std::set<std::string>& topo_section_ids_,
    const std::unordered_map<std::string, LocalLane>& lane_table_,
    const std::unordered_map<std::string, LocalRoad>& road_table_,
    const std::vector<std::string>& topo_lane_ids_) {
  if (topo_section_ids_.empty() || road_table_.empty() ||
      topo_lane_ids_.empty() || lane_table_.empty()) {
    return;
  }
  for (const auto& it : far_table_) {
    if (lane_table_.find(it.first) == lane_table_.end()) {
      HLOG_ERROR << "topo lane not found in lane_table";
      continue;
    }
    // if (std::find(end_lane_ids_.begin(), end_lane_ids_.end(), it.first) !=
    //     end_lane_ids_.end()) {
    //   continue;
    // }

    const auto& local_lane = lane_table_.at(it.first);
    if (it.second.flag == 0 || it.second.flag == 2) {
      // 处理左
      BuildLeft(it, local_lane);
    }
    if (it.second.flag == 1 || it.second.flag == 2) {
      // 处理右
      BuildRight(it, local_lane);
    }
  }
}

void MapPrediction::BuildLeft(const std::pair<const std::string, FarLane>& it,
                              const LocalLane& local_lane) {
  // 弥补左边线
  // auto left_seg_size = it.second.left_line.size();
  auto road_id = local_lane.road_id;
  auto section_id = local_lane.section_id;
  const auto& local_road = road_table_.at(road_id);
  if (local_road.section_ids.empty()) {
    HLOG_ERROR << "empty section ids";
    return;
  }
  if (local_road.section_ids.find(section_id) == local_road.section_ids.end()) {
    HLOG_ERROR << "section not found in section ids";
    return;
  }
  // if (local_road.section_ids.at(section_id).lane_id.back() == it.first &&
  //     left_seg_size != 0) {
  //   continue;
  // }
  // 如果上面没有continue，则有缺失车道，计算缺失车道数量
  uint32_t lane_num = local_road.section_ids.at(section_id).lane_id.size();
  uint32_t mis_num = 0;
  bool flag = false;
  for (const auto& itt : local_road.section_ids.at(section_id).lane_id) {
    if (itt == it.first) {
      flag = true;
    }
    if (flag) {
      mis_num += 1;
    }
  }
  const auto& boundary1 = local_road.section_ids.at(section_id).left_boundary;
  const auto& boundary2 = local_road.section_ids.at(section_id).right_boundary;
  const auto& road_boundary =
      local_road.section_ids.at(section_id).road_boundary;

  // 存储右边线
  std::pair<std::string, std::vector<Eigen::Vector3d>> curr_left_line;
  if (!far_table_.at(it.first).right_line.empty()) {
    curr_left_line.first = far_table_.at(it.first).lane_id;
    curr_left_line.second = far_table_.at(it.first).right_line;
  }
  AddLeftOrRightLine(road_boundary, curr_left_line, mis_num, lane_num, 0,
                     local_road.section_ids.at(section_id).lane_id);
}

void MapPrediction::BuildRight(const std::pair<const std::string, FarLane>& it,
                               const LocalLane& local_lane) {
  // 弥补右边线
  // auto right_seg_size = it.second.right_line.size();
  auto road_id = local_lane.road_id;
  auto section_id = local_lane.section_id;
  if (road_table_.find(road_id) == road_table_.end()) {
    HLOG_ERROR << "road not found in road_table";
    return;
  }
  const auto& local_road = road_table_.at(road_id);
  if (local_road.section_ids.empty()) {
    HLOG_ERROR << "empty section ids";
    return;
  }
  if (local_road.section_ids.find(section_id) == local_road.section_ids.end()) {
    HLOG_ERROR << "section not found in section ids";
    return;
  }
  // if (local_road.section_ids.at(section_id).lane_id.front() == it.first
  // &&
  //     right_seg_size != 0) {
  //   continue;
  // }
  // 如果上面没有continue，则有缺失车道，计算缺失车道数量
  uint32_t mis_num = 0;
  uint32_t lane_num = local_road.section_ids.at(section_id).lane_id.size();
  bool flag = true;
  for (const auto& itt : local_road.section_ids.at(section_id).lane_id) {
    if (flag) {
      mis_num += 1;
    }
    if (itt == it.first) {
      flag = false;
    }
  }

  const auto& boundary1 = local_road.section_ids.at(section_id).left_boundary;
  const auto& boundary2 = local_road.section_ids.at(section_id).right_boundary;
  const auto& road_boundary =
      local_road.section_ids.at(section_id).road_boundary;

  // 存储右边线
  std::pair<std::string, std::vector<Eigen::Vector3d>> curr_right_line;
  if (!far_table_.at(it.first).left_line.empty()) {
    curr_right_line.first = far_table_.at(it.first).lane_id;
    curr_right_line.second = far_table_.at(it.first).left_line;
  }

  // AddLeftOrRightLine(boundary1, boundary2, curr_left_line, mis_num,
  //                    lane_num, 0);
  AddLeftOrRightLine(road_boundary, curr_right_line, mis_num, lane_num, 1,
                     local_road.section_ids.at(section_id).lane_id);
}

void MapPrediction::AddLeftOrRightLine(
    const std::vector<Eigen::Vector3d>& road_boundary,
    const std::pair<std::string, std::vector<Eigen::Vector3d>>& curr_line,
    const uint32_t& mis_num, const uint32_t& lane_num, const uint32_t& record,
    const std::vector<std::string>& sec_lane_id) {
  // 开始拟合车道线并赋予拓扑关系
  if (road_boundary.empty() || curr_line.second.empty()) {
    return;
  }

  // uint32_t num_miss_line = uint32_t(dist / 3);  // 判断缺失车道数量
  if (mis_num == 0 || lane_num == 0) {
    return;
  }

  std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>> predict_line;
  FitPredLaneLine(road_boundary, sec_lane_id, predict_line);
  // FitAheadLaneLine(boundary1, boundary2, predict_line, lane_num);
  if (predict_line.empty()) {
    return;
  }
  std::vector<std::vector<Eigen::Vector3d>> side_line;
  side_line.clear();
  int lane_num_i = static_cast<int>(lane_num);
  int mis_num_i = static_cast<int>(mis_num);
  if (record == 1) {
    for (int i = lane_num_i - mis_num_i + 1; i < lane_num_i + 1; ++i) {
      side_line.emplace_back(predict_line[i].second);
    }
  }
  if (record == 0) {
    for (int i = mis_num_i - 1; i >= 0; --i) {
      side_line.emplace_back(predict_line[i].second);
    }
  }
  if (side_line.empty()) {
    return;
  }
  // 对预测的左右缺失车道线进行可视化
  viz_map_.VizAddSideLaneLine(side_line);

  // 将预测的车道线添加到local_msg中，并赋予拓扑关系
  AddSideTopological(side_line, record, curr_line.first);
}

void MapPrediction::AddSideTopological(
    const std::vector<std::vector<Eigen::Vector3d>>& predict_line,
    const uint32_t& record, const std::string& curr_id) {
  // 将新的车道线添加到local_msg中，并赋予拓扑关系
  if (predict_line.empty()) {
    return;
  }
  // 添加左
  if (record == 0) {
    // 预测缺失的左边线
    AddLeft(predict_line, curr_id);
  }

  // 添加右
  if (record == 1) {
    // 预测缺失的右边线
    AddRight(predict_line, curr_id);
  }
}

void MapPrediction::AddLeft(
    const std::vector<std::vector<Eigen::Vector3d>>& predict_line,
    const std::string& curr_id) {
  // 左
  std::string id_ = curr_id;
  bool flag = true;
  for (const auto& line : predict_line) {
    HLOG_ERROR << "!mf line size " << line.size();
    // 现将当前lane的左边线补充完整
    if (id_ == curr_id && flag) {
      for (auto& lane : *local_msg_->mutable_lane()) {
        if (lane.id().id() == id_ && !id_.empty()) {
          auto* left_seg =
              lane.mutable_left_boundary()->mutable_curve()->add_segment();
          for (const auto& point : line) {
            auto* pt = left_seg->mutable_line_segment()->add_point();
            pt->set_x(point.x());
            pt->set_y(point.y());
            pt->set_z(point.z());
          }
        }
      }
      flag = false;
      continue;
    }
    AddSideTopoLeft(line, &id_);
    if (id_.empty()) {
      break;
    }
  }
}

void MapPrediction::AddRight(
    const std::vector<std::vector<Eigen::Vector3d>>& predict_line,
    const std::string& curr_id) {
  // 右
  std::string id_ = curr_id;
  bool flag = true;
  for (const auto& line : predict_line) {
    // 现将当前lane的右边线补充完整
    if (id_ == curr_id && flag) {
      for (auto& lane : *local_msg_->mutable_lane()) {
        if (lane.id().id() == id_ && !id_.empty()) {
          auto* right_seg =
              lane.mutable_right_boundary()->mutable_curve()->add_segment();
          for (const auto& point : line) {
            auto* pt = right_seg->mutable_line_segment()->add_point();
            pt->set_x(point.x());
            pt->set_y(point.y());
            pt->set_z(point.z());
          }
        }
      }
      flag = false;
      continue;
    }
    AddSideTopoRight(line, &id_);
    if (id_.empty()) {
      break;
    }
  }
}

void MapPrediction::AddSideTopoLeft(const std::vector<Eigen::Vector3d>& line,
                                    std::string* id_) {
  // 预测左
  // 从topo_map_中找取对应id的左邻和后继id
  std::string id_left_id;
  std::string id_next_id;

  // 这里要注意id_left_id/id_next_id是否为空
  if (lane_table_.find(*id_) == lane_table_.end()) {
    HLOG_ERROR << "lane not found in lane_table";
    return;
  }
  if (lane_table_.at(*id_).left_lane_ids.empty()) {
    HLOG_ERROR << "left lane id is empty!";
    return;
  }
  id_left_id = lane_table_.at(*id_).left_lane_ids.front();
  if (std::find(topo_lane_ids_.begin(), topo_lane_ids_.end(), id_left_id) !=
      topo_lane_ids_.end()) {
    HLOG_ERROR << "new lane id has in topo_lane_ids";
    for (auto& lane : *local_msg_->mutable_lane()) {
      if (lane.id().id() == *id_) {
        lane.add_left_neighbor_reverse_lane_id()->set_id(id_left_id);
      }
      if (lane.id().id() == id_left_id) {
        lane.add_right_neighbor_forward_lane_id()->set_id(*id_);
      }
    }
    return;
  }
  // 定义一个新的车道
  auto* new_lane = local_msg_->add_lane();
  // 赋予id
  new_lane->mutable_id()->set_id(id_left_id);
  topo_lane_ids_.emplace_back(id_left_id);

  if (!lane_table_.at(*id_).next_lane_ids.empty()) {
    id_next_id = lane_table_.at(*id_).next_lane_ids.front();
  }

  // new_lane赋予新几何
  auto* left_seg =
      new_lane->mutable_left_boundary()->mutable_curve()->add_segment();
  for (const auto& point : line) {
    auto* pt = left_seg->mutable_line_segment()->add_point();
    pt->set_x(point.x());
    pt->set_y(point.y());
    pt->set_z(point.z());
  }

  // 确定当前lane后继lane的左邻lane(拓扑信息暂时关闭)
  // std::string id_next_left_id;
  // if (!id_next_id.empty() &&
  //     lane_table_.find(id_next_id) != lane_table_.end()) {
  //   //!!!! TBD
  //   if (lane_table_.find(id_next_id) == lane_table_.end()) {
  //     HLOG_ERROR << "lane not found in lane_table";
  //     return;
  //   }
  //   if (!lane_table_.at(id_next_id).left_lane_ids.empty()) {
  //     id_next_left_id = lane_table_.at(id_next_id).left_lane_ids.front();
  //   }
  // }
  // new_lane的后继指向id_next_left_id
  // if (!id_next_left_id.empty()) {
  //   new_lane->add_successor_id()->set_id(id_next_left_id);
  // }

  // new_lane加入到local_msg
  for (auto& lane : *local_msg_->mutable_lane()) {
    if (lane.id().id() == *id_ && !id_->empty()) {
      new_lane->mutable_right_boundary()->CopyFrom(lane.left_boundary());
      // new_lane->add_right_neighbor_forward_lane_id()->set_id(lane.id().id());
      // lane.add_left_neighbor_forward_lane_id()->set_id(new_lane->id().id());
      break;
    }
  }
  // 完善前后继关系(拓扑信息暂时关闭)
  // for (auto& lane : *local_msg_->mutable_lane()) {
  //   if (lane.id().id() == id_next_left_id && !id_next_left_id.empty()) {
  //     lane.add_predecessor_id()->set_id(new_lane->id().id());
  //     break;
  //   }
  // }

  *id_ = id_left_id;
}

void MapPrediction::AddSideTopoRight(const std::vector<Eigen::Vector3d>& line,
                                     std::string* id_) {
  // 预测右
  // 从topo_map_中找取对应id的左邻和后继id
  std::string id_right_id;
  std::string id_next_id;

  // 这里要注意id_left_id/id_next_id是否为空
  if (lane_table_.find(*id_) == lane_table_.end()) {
    HLOG_ERROR << "lane not found in lane_table";
    return;
  }
  if (lane_table_.at(*id_).right_lane_ids.empty()) {
    HLOG_ERROR << "left lane id is empty!";
    return;
  }
  id_right_id = lane_table_.at(*id_).right_lane_ids.front();
  if (std::find(topo_lane_ids_.begin(), topo_lane_ids_.end(), id_right_id) !=
      topo_lane_ids_.end()) {
    HLOG_ERROR << "lane id has in topo_lane_ids";
    for (auto& lane : *local_msg_->mutable_lane()) {
      if (lane.id().id() == *id_) {
        lane.add_right_neighbor_reverse_lane_id()->set_id(id_right_id);
      }
      if (lane.id().id() == id_right_id) {
        lane.add_left_neighbor_forward_lane_id()->set_id(*id_);
      }
    }
    return;
  }
  // 定义一个新的车道
  auto* new_lane = local_msg_->add_lane();
  // 赋予id
  new_lane->mutable_id()->set_id(id_right_id);
  topo_lane_ids_.emplace_back(id_right_id);
  if (!lane_table_.at(*id_).next_lane_ids.empty()) {
    id_next_id = lane_table_.at(*id_).next_lane_ids.front();
  }

  // new_lane赋予新几何
  auto* right_seg =
      new_lane->mutable_right_boundary()->mutable_curve()->add_segment();
  for (const auto& point : line) {
    auto* pt = right_seg->mutable_line_segment()->add_point();
    pt->set_x(point.x());
    pt->set_y(point.y());
    pt->set_z(point.z());
  }
  // 确定当前lane后继lane的右邻lane(拓扑信息暂时关闭)
  // std::string id_next_right_id;
  // if (!id_next_id.empty()) {
  //   if (lane_table_.find(id_next_id) == lane_table_.end()) {
  //     HLOG_ERROR << "lane not found in lane_table";
  //     return;
  //   }
  //   if (!lane_table_.at(id_next_id).right_lane_ids.empty()) {
  //     id_next_right_id = lane_table_.at(id_next_id).right_lane_ids.front();
  //   }
  // }
  // new_lane的后继指向id_next_left_id
  // if (!id_next_right_id.empty()) {
  //   new_lane->add_successor_id()->set_id(id_next_right_id);
  // }

  // new_lane加入到local_msg中
  for (auto& lane : *local_msg_->mutable_lane()) {
    if (lane.id().id() == *id_ && !id_->empty()) {
      new_lane->mutable_left_boundary()->CopyFrom(lane.right_boundary());
      // new_lane->add_left_neighbor_forward_lane_id()->set_id(lane.id().id());
      // lane.add_right_neighbor_forward_lane_id()->set_id(new_lane->id().id());
      break;
    }
  }

  // 完善前后继关系(拓扑关系暂时关闭)
  // for (auto& lane : *local_msg_->mutable_lane()) {
  //   if (lane.id().id() == id_next_right_id && !id_next_right_id.empty()) {
  //     lane.add_predecessor_id()->set_id(new_lane->id().id());
  //     break;
  //   }
  // }

  *id_ = id_right_id;
}

void MapPrediction::PredictAheadLaneLine(
    const std::vector<std::string>& add_section_ids_,
    const std::unordered_map<std::string, LocalLane>& lane_table_,
    const std::unordered_map<std::string, LocalRoad>& road_table_) {
  // 用于预测前方车道线
  if (add_section_ids_.empty() || lane_table_.empty() || road_table_.empty()) {
    return;
  }
  // 这里有个假设，即add_section_id严格的按照顺序执行的
  for (const auto& sec : add_section_ids_) {
    std::string curr_lane_id;
    std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>
        predict_lanelines;
    for (const auto& it : add_lane_ids_) {
      if (lane_table_.find(it) == lane_table_.end()) {
        HLOG_ERROR << "road not found in road_table";
        continue;
      }
      if (lane_table_.at(it).section_id == sec) {
        curr_lane_id = it;
        break;
      }
    }
    if (curr_lane_id.empty()) {
      continue;
    }
    const auto& local_lane = lane_table_.at(curr_lane_id);
    auto road_id = local_lane.road_id;
    auto section_id = local_lane.section_id;
    if (road_table_.find(road_id) == road_table_.end()) {
      HLOG_ERROR << "road not found in road_table";
      continue;
    }
    const auto& local_road = road_table_.at(road_id);
    if (local_road.section_ids.empty() ||
        local_road.section_ids.find(section_id) ==
            local_road.section_ids.end()) {
      HLOG_ERROR << "empty section ids or section not found in section ids";
      continue;
    }
    // if (local_road.section_ids.find(section_id) ==
    //     local_road.section_ids.end()) {
    //   HLOG_ERROR << "section not found in section ids";
    //   continue;
    // }
    // 查找其是否包含end_lane_ids_;
    bool flag = false;
    for (const auto& it : local_road.section_ids.at(sec).lane_id) {
      if (std::find(topo_lane_ids_.begin(), topo_lane_ids_.end(), it) !=
          topo_lane_ids_.end()) {
        flag = true;
        break;
      }
    }
    if (flag) {
      continue;
    }
    std::vector<std::vector<Eigen::Vector3d>> boundary1 =
        local_road.section_ids.at(sec).left_boundary;
    std::vector<std::vector<Eigen::Vector3d>> boundary2 =
        local_road.section_ids.at(sec).right_boundary;
    std::vector<Eigen::Vector3d> road_boundary =
        local_road.section_ids.at(sec).road_boundary;

    uint32_t lane_num = local_road.section_ids.at(sec).lane_id.size();
    FitPredLaneLine(road_boundary, local_road.section_ids.at(sec).lane_id,
                    predict_lanelines);
    // FitAheadLaneLine(boundary1, boundary2, predict_lanelines, lane_num);
    // 可视化
    viz_map_.VizAddAheadLaneLine(predict_lanelines);
    // 对预测的车道继承HQ的ID并添加拓扑关系
    AheadTopological(predict_lanelines, local_road.section_ids.at(sec).lane_id);
  }
}

void MapPrediction::CompleteLaneline(
    const std::vector<std::string>& end_lane_id_,
    const std::set<std::string>& end_section_id,
    const std::unordered_map<std::string, LocalRoad>& road_table) {
  if (end_lane_id_.empty() || end_section_id.empty() || road_table.empty()) {
    return;
  }
  for (const auto& sec : end_section_id) {
    std::vector<std::string> com_id;
    std::string road_id;
    for (const auto& end_lane_id : end_lane_id_) {
      if (lane_table_.find(end_lane_id) == lane_table_.end()) {
        HLOG_ERROR << "end lane id not found in lane_table";
        continue;
      }
      if (lane_table_[end_lane_id].section_id != sec) {
        continue;
      }
      com_id.emplace_back(end_lane_id);
      road_id = lane_table_[end_lane_id].road_id;
    }

    //    for (const auto& sec_lane_id : hash_table.at(sec).lane_id) {
    //      if (std::find(end_lane_id_.begin(), end_lane_id_.end(),
    //      sec_lane_id)
    //      !=
    //          end_lane_id_.end()) {
    //        com_id.emplace_back(sec_lane_id);
    //      }
    //    }
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

    std::vector<std::vector<Eigen::Vector3d>> left_boundary =
        hash_table.at(sec).left_boundary;
    std::vector<std::vector<Eigen::Vector3d>> right_boundary =
        hash_table.at(sec).right_boundary;
    std::vector<Eigen::Vector3d> road_boundary =
        hash_table.at(sec).road_boundary;
    // 拟合线
    std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>
        complete_lines;
    uint32_t lane_num = hash_table.at(sec).lane_id.size();
    FitPredLaneLine(road_boundary, hash_table.at(sec).lane_id, complete_lines);
    // FitAheadLaneLine(left_boundary, right_boundary, complete_lines,
    // lane_num);
    viz_map_.VizAddAheadLaneLine(complete_lines);
    // 延伸
    ExpansionLaneLine(complete_lines, com_id, hash_table.at(sec).lane_id);
  }
}

void MapPrediction::ExpansionLaneLine(
    const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
        complete_lines,
    const std::vector<std::string>& com_id,
    const std::vector<std::string>& sec_id) {
  // 开始对当前的车道进行延伸
  if (complete_lines.empty() || com_id.empty() || sec_id.empty()) {
    return;
  }
  // 将填充的车道线保存并可视化出来
  std::vector<std::vector<Eigen::Vector3d>> compan_lines;
  uint32_t lane_count = 0;  // 车道计数
  for (auto& lane : *local_msg_->mutable_lane()) {
    for (const auto& end_id : com_id) {
      if (end_id != lane.id().id()) {
        continue;
      }
      // 定义全局标识符
      uint32_t com_fit = 2;
      JudgeDiection(&com_fit, end_id);
      // 求该道路的两个边界到拟合的车道线的最近距离
      int seq = 0;  // section_id从右-->左
      for (const auto& it : sec_id) {
        if (it == lane.id().id()) {
          break;
        }
        seq += 1;
      }
      // complete_lines从左-->右
      // 左
      int complete_size = static_cast<int>(complete_lines.size());
      int left_line_id = complete_size - 2 - seq;
      // 右
      int right_line_id = complete_size - 1 - seq;

      if ((com_fit == 0 || com_fit == 2) &&
          !lane.left_boundary().curve().segment().empty()) {
        // 左边界
        // 临时加了一个判断条件
        // if (lane.left_boundary().curve().segment().empty()) {
        //   continue;
        // }
        ExpansionLeft(&lane, left_line_id, complete_lines);
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
        ExpansionRight(&lane, right_line_id, complete_lines);
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

void MapPrediction::ExpansionLeft(
    hdmap::Lane* lane, const int& left_line_id,
    const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
        complete_lines) {
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

  // uint32_t count_left = 0;  // 记录从第几个点开始扩充
  PointToLineDist(left_end_point, complete_lines, left_line_id,
                  &flag_count_left, &min_distance_left);
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

  if (complete_lines[left_line_id].second.size() - flag_count_left <= 2) {
    // 直接连接complete_lines中的最后一个点
    const auto& complete_line_size = complete_lines[left_line_id].second.size();
    auto* pt = lane->mutable_left_boundary()
                   ->mutable_curve()
                   ->mutable_segment(seg_size - 1)
                   ->mutable_line_segment()
                   ->add_point();
    pt->set_x(complete_lines[left_line_id].second.back().x());
    pt->set_y(complete_lines[left_line_id].second.back().y());
    pt->set_z(complete_lines[left_line_id].second.back().z());
  } else {
    if (com_left_line.size() <= 2) {
      for (size_t i = flag_count_left + 2;
           i < complete_lines[left_line_id].second.size(); ++i) {
        auto* pt = lane->mutable_left_boundary()
                       ->mutable_curve()
                       ->mutable_segment(seg_size - 1)
                       ->mutable_line_segment()
                       ->add_point();
        pt->set_x(complete_lines[left_line_id].second[i].x());
        pt->set_y(complete_lines[left_line_id].second[i].y());
        pt->set_z(complete_lines[left_line_id].second[i].z());
      }
    } else {
      const auto& com_left_size = com_left_line.size();
      Eigen::Vector3d local_first = com_left_line[com_left_size - 3];
      Eigen::Vector3d local_second = com_left_line[com_left_size - 2];
      Eigen::Vector3d com_first =
          complete_lines[left_line_id].second[flag_count_left + 1];
      Eigen::Vector3d com_second =
          complete_lines[left_line_id].second[flag_count_left + 2];
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

      for (size_t i = flag_count_left + 2;
           i < complete_lines[left_line_id].second.size(); ++i) {
        auto* pt = lane->mutable_left_boundary()
                       ->mutable_curve()
                       ->mutable_segment(seg_size - 1)
                       ->mutable_line_segment()
                       ->add_point();
        pt->set_x(complete_lines[left_line_id].second[i].x());
        pt->set_y(complete_lines[left_line_id].second[i].y());
        pt->set_z(complete_lines[left_line_id].second[i].z());
      }
    }
  }
}

void MapPrediction::ExpansionRight(
    hdmap::Lane* lane, const int& right_line_id,
    const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
        complete_lines) {
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

  // uint32_t count_right = 1;  // 记录从第几个点开始扩充
  PointToLineDist(right_end_point, complete_lines, right_line_id,
                  &flag_count_right, &min_distance_right);
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

  if (complete_lines[right_line_id].second.size() - flag_count_right <= 2) {
    // 直接连接complete_lines中的最后一个点
    const auto& complete_line_size =
        complete_lines[right_line_id].second.size();
    auto* pt = lane->mutable_right_boundary()
                   ->mutable_curve()
                   ->mutable_segment(seg_size_ - 1)
                   ->mutable_line_segment()
                   ->add_point();
    pt->set_x(complete_lines[right_line_id].second.back().x());
    pt->set_y(complete_lines[right_line_id].second.back().y());
    pt->set_z(complete_lines[right_line_id].second.back().z());
  } else {
    if (com_right_line.size() <= 2) {
      for (size_t i = flag_count_right + 2;
           i < complete_lines[right_line_id].second.size(); ++i) {
        auto* pt = lane->mutable_right_boundary()
                       ->mutable_curve()
                       ->mutable_segment(seg_size_ - 1)
                       ->mutable_line_segment()
                       ->add_point();
        pt->set_x(complete_lines[right_line_id].second[i].x());
        pt->set_y(complete_lines[right_line_id].second[i].y());
        pt->set_z(complete_lines[right_line_id].second[i].z());
      }
    } else {
      const auto& com_right_size = com_right_line.size();
      Eigen::Vector3d local_first = com_right_line[com_right_size - 3];
      Eigen::Vector3d local_second = com_right_line[com_right_size - 2];
      Eigen::Vector3d com_first =
          complete_lines[right_line_id].second[flag_count_right + 1];
      Eigen::Vector3d com_second =
          complete_lines[right_line_id].second[flag_count_right + 2];
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

      for (size_t i = flag_count_right + 2;
           i < complete_lines[right_line_id].second.size(); ++i) {
        auto* pt = lane->mutable_right_boundary()
                       ->mutable_curve()
                       ->mutable_segment(seg_size_ - 1)
                       ->mutable_line_segment()
                       ->add_point();
        pt->set_x(complete_lines[right_line_id].second[i].x());
        pt->set_y(complete_lines[right_line_id].second[i].y());
        pt->set_z(complete_lines[right_line_id].second[i].z());
      }
    }
  }
}

void MapPrediction::PointToLineDist(
    const Eigen::Vector3d& end_point,
    const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
        complete_lines,
    const int& line_id, uint32_t* index, double* min_distance) {
  // point to line distance
  uint32_t count = 1;  // 记录从第几个点开始扩充
  for (size_t i = 1; i < complete_lines[line_id].second.size(); ++i) {
    Eigen::Vector3d A = complete_lines[line_id].second[i - 1];
    Eigen::Vector3d B = complete_lines[line_id].second[i];
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
    if (dis < *min_distance && dis < 3.75) {
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
  hozon::common::coordinate_convertor::UTM2GCS(static_cast<int>(utm_zone_), &x,
                                               &y);
  Eigen::Vector3d point_gcj(y, x, 0);
  Eigen::Vector3d point_enu =
      util::Geo::Gcj02ToEnu(point_gcj, local_enu_center_);
  return point_enu;
}

void MapPrediction::FitPredLaneLine(
    const std::vector<Eigen::Vector3d>& road_boundary,
    const std::vector<std::string>& sec_lane_id,
    std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
        predict_lanelines) {
  // 新的拟合车道线的方法
  if (road_boundary.empty() || sec_lane_id.empty()) {
    return;
  }
  std::vector<std::vector<Eigen::Vector3d>> predict_line(sec_lane_id.size() +
                                                         1);
  const auto& road_size = road_boundary.size();
  int index = 0;  // 表示第几个点的索引
  double s = 0.;
  const auto& start_point = road_boundary.front();
  for (size_t i = 1; i < road_size; ++i) {
    const auto& A = road_boundary[i - 1];
    const auto& B = road_boundary[i];
    Eigen::Vector3d AB = B - A;
    Eigen::Vector3d AB_N = AB.normalized();
    Eigen::Vector3d AB_C(-AB_N.y(), AB_N.x(), 0);
    // 根据距离求其他的点
    // 先求左侧的车道线点
    if (lane_table_.find(sec_lane_id.back()) == lane_table_.end()) {
      HLOG_ERROR << "lane id not in lane table!";
      continue;
    }
    if (i > 1) {
      s += (B - A).norm();
      index = static_cast<int>(s / 0.5);
    }
    const auto& first_width = lane_table_.at(sec_lane_id.back()).lane_width;
    // if (index > first_width.size()) {
    //   continue;
    // }
    Eigen::Vector3d first_point = A + AB_C * first_width[index];
    predict_line.at(0).push_back(first_point);
    predict_line.at(1).push_back(A);
    double width = 0.;
    if (sec_lane_id.size() < 2) {
      continue;
    }
    for (int j = static_cast<int>(sec_lane_id.size()) - 2; j >= 0; --j) {
      if (lane_table_.find(sec_lane_id[j]) == lane_table_.end()) {
        continue;
      }
      const auto& wid = lane_table_.at(sec_lane_id[j]).lane_width;
      width += wid[index];
      Eigen::Vector3d point = A - AB_C * width;
      predict_line.at(sec_lane_id.size() - j).push_back(point);
    }
  }
  // 求最后一个点的左右点
  const auto& A = road_boundary[road_size - 1];
  const auto& B = road_boundary[road_size - 2];
  Eigen::Vector3d AB = B - A;
  Eigen::Vector3d AB_N = AB.normalized();
  Eigen::Vector3d AB_C(-AB_N.y(), AB_N.x(), 0);
  // 先求右侧的车道线点
  if (lane_table_.find(sec_lane_id.back()) == lane_table_.end()) {
    HLOG_ERROR << "lane id not in lane table!";
    return;
  }
  const auto& end_width = lane_table_.at(sec_lane_id.back()).lane_width;
  Eigen::Vector3d first_point = A - AB_C * end_width.back();
  predict_line.at(0).push_back(first_point);
  predict_line.at(1).push_back(A);
  double width = 0.;
  if (sec_lane_id.size() < 2) {
    HLOG_ERROR << "sec_lane_id.size() < 2";
    return;
  }
  for (int j = static_cast<int>(sec_lane_id.size()) - 2; j >= 0; --j) {
    if (lane_table_.find(sec_lane_id[j]) == lane_table_.end()) {
      HLOG_ERROR << "lane id not in lane table!";
      continue;
    }
    const auto& wid = lane_table_.at(sec_lane_id[j]).lane_width;
    width += wid.back();
    Eigen::Vector3d point = A + AB_C * width;
    predict_line.at(sec_lane_id.size() - j).push_back(point);
  }

  if (!predict_line.empty()) {
    for (const auto& it : predict_line) {
      predict_lanelines.emplace_back(id, it);
      id -= 1;
    }
  }
}

#if 0
void MapPrediction::FitAheadLaneLine(
    const std::vector<std::vector<Eigen::Vector3d>>& boundary1,
    const std::vector<std::vector<Eigen::Vector3d>>& boundary2,
    std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
        predict_lanelines,
    const uint32_t& lane_num) {
  // 跟据boundary1和boundary2拟合车道线
  if (boundary1.empty() || boundary2.empty()) {
    return;
  }
  std::vector<Eigen::Vector3d> edge1_;
  std::vector<Eigen::Vector3d> edge2_;
  for (const auto& bound : boundary1) {
    for (const auto& point : bound) {
      if (!edge1_.empty() && point == edge1_.back()) {
        continue;
      }
      edge1_.push_back(point);
    }
  }
  // 去除重复元素
  // edge1_.erase(std::unique(edge1_.begin(), edge1_.end()), edge1_.end());
  for (const auto& bound : boundary2) {
    for (const auto& point : bound) {
      if (!edge2_.empty() && point == edge2_.back()) {
        continue;
      }
      edge2_.push_back(point);
    }
  }
  // 去除重复元素
  // edge2_.erase(std::unique(edge2_.begin(), edge2_.end()), edge2_.end());

  std::vector<std::vector<Eigen::Vector3d>> predict_line(lane_num + 1);
  // 将起点加入进去
  // if (edge1.begin()->x() == edge2_.front().x()) {
  //   // k的值是无穷
  //   for (double j = 0.0; j < num_lines + 1; ++j) {
  //     double ratio = j / num_lines;
  //     double x = ratio * (edge2_.front().x() - edge1.begin()->x()) +
  //                edge1.begin()->x();
  //     double y = edge1.begin()->y() +
  //                (edge2_.front().y() - edge1.begin()->y()) * ratio;
  //     Eigen::Vector3d new_point(x, y, 0);
  //     predict_line[j].push_back(new_point);
  //   }
  // }
  Eigen::Vector3d edge1_begin = edge1_.front();
  Eigen::Vector3d edge2_begin = edge2_.front();
  double k =
      (edge1_begin.y() - edge2_begin.y()) / (edge1_begin.x() - edge2_begin.x());
  double b = edge2_begin.y() - k * edge2_begin.x();
  for (uint32_t j = 0; j < lane_num + 1; ++j) {
    double ratio = j * 1.0 / lane_num;
    double x = ratio * (edge2_begin.x() - edge1_begin.x()) + edge1_begin.x();
    double y = k * x + b;
    if (std::isnan(x) || std::isnan(y)) {
      continue;
    }
    Eigen::Vector3d new_point(x, y, 0);
    predict_line[j].push_back(new_point);
  }

  // 将中间点加入进去
  for (const auto& bound1 : edge1_) {
    double min_dis = std::numeric_limits<double>::max();
    for (size_t i = 1; i < edge2_.size(); ++i) {
      Eigen::Vector3d A = edge2_[i - 1];
      Eigen::Vector3d B = edge2_[i];
      Eigen::Vector3d P = bound1;

      // 计算A点和B点的方向向量
      Eigen::Vector3d AB = B - A;
      Eigen::Vector3d AP = P - A;
      // 计算AB模长
      double ABLengthSquared = AB.squaredNorm();
      if (ABLengthSquared == 0) {
        continue;
      }
      double t = AB.dot(AP) / ABLengthSquared;
      if (t < 0 || t > 1) {
        continue;
      }
      t = std::max(0.0, std::min(t, 1.0));
      Eigen::Vector3d C = A + t * AB;  // 点到线段的最近点
      // double dis = (P - C).norm();
      // uint32_t num_lines = uint32_t(dis / 3.5);

      // if (P.x() == C.x()) {
      //   for (double j = 0.0; j < num_lines + 1; ++j) {
      //     double ratio = j / num_lines;
      //     double x = P.x();
      //     double y = P.y() + (C.y() - P.y()) * ratio;
      //     Eigen::Vector3d new_point(x, y, 0);
      //     predict_line[j].push_back(new_point);
      //   }
      // }
      double k = (P.y() - C.y()) / (P.x() - C.x());
      double b = C.y() - k * C.x();
      for (double j = 0; j < lane_num + 1; ++j) {
        double ratio = j * 1.0 / lane_num;
        double x = ratio * (C.x() - P.x()) + P.x();
        double y = k * x + b;
        if (std::isnan(x) || std::isnan(y)) {
          continue;
        }
        Eigen::Vector3d new_point(x, y, 0);
        if (!predict_line[j].empty() &&
            std::find(predict_line[j].begin(), predict_line[j].end(),
                      new_point) == predict_line[j].end()) {
          predict_line[j].push_back(new_point);
        }
      }
      break;
    }
  }
  // 将终点加入进去
  Eigen::Vector3d edge1_end = edge1_.back();
  Eigen::Vector3d edge2_end = edge2_.back();
  double k1 = (edge1_end.y() - edge2_end.y()) / (edge1_end.x() - edge2_end.x());
  double b1 = edge2_end.y() - k1 * edge2_end.x();
  for (uint32_t j = 0; j < lane_num + 1; ++j) {
    double ratio = j * 1.0 / lane_num;
    double x = ratio * (edge2_end.x() - edge1_end.x()) + edge1_end.x();
    double y = k1 * x + b1;
    if (std::isnan(x) || std::isnan(y)) {
      continue;
    }
    Eigen::Vector3d new_point(x, y, 0);
    if (!predict_line[j].empty() &&
        std::find(predict_line[j].begin(), predict_line[j].end(), new_point) ==
            predict_line[j].end()) {
      predict_line[j].push_back(new_point);
    }
  }

  if (!predict_line.empty()) {
    for (const auto& it : predict_line) {
      predict_lanelines.emplace_back(std::make_pair(id, it));
      id -= 1;
    }
  }
}
#endif

void MapPrediction::AheadTopological(
    const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
        predict_lanelines,
    const std::vector<std::string>& section_lane_id) {
  // 预测前方地图的拓扑关系
  // 存储预测的车道线的起点
  // std::lock_guard<std::mutex> lock(mtx_);
  if (!local_msg_ || predict_lanelines.empty() || section_lane_id.empty()) {
    return;
  }

  if (section_lane_id.size() + 1 != predict_lanelines.size()) {
    return;
  }
  /*
  根据LOG判断section_lane_id的顺序依次是从右往左(boundary2-->boundary1)
  对拓扑关系进行优化操作：
  1.从section_lane_id找到对应的车道id
  2.从topo_map_中遍历id,找到左右边界lane，0-->左边界lane, 1-->右边界lane
  3.predict_lanelines中第一根新的起点与左lane的左线和右lane的右线进行比较求关联
  4.根据3中的关联关系依次创建new_lane，并赋予id和几何和左右拓扑关系，并加入local_msg中
  5.从topo_map_中找new_lane的所有前继id，在local_msg中赋予new_lane的前继关系
  6.从local_msg中找new_lane的所有前继id，并赋予后继关系
  */
  // 从bounary2-->boundary1
  for (int i = static_cast<int>(predict_lanelines.size()) - 2; i >= 0; --i) {
    if (std::find(end_lane_ids_.begin(), end_lane_ids_.end(),
                  section_lane_id[section_lane_id.size() - i - 1]) !=
        end_lane_ids_.end()) {
      continue;
    }
    if (std::find(topo_lane_ids_.begin(), topo_lane_ids_.end(),
                  section_lane_id[section_lane_id.size() - i - 1]) !=
        topo_lane_ids_.end()) {
      continue;
    }
    // 定义新车道
    hozon::hdmap::Lane new_lane;
    // 为新车道赋予Id
    new_lane.mutable_id()->set_id(
        section_lane_id[section_lane_id.size() - i - 1]);
    if (new_lane.id().id().empty()) {
      HLOG_ERROR << "AheadTopological have empty lane id!";
      continue;
    }
    topo_lane_ids_.emplace_back(new_lane.id().id());
    // 为新车道添加几何信息
    // 左
    auto* left_seg =
        new_lane.mutable_left_boundary()->mutable_curve()->add_segment();
    for (const auto& point : predict_lanelines[i].second) {
      auto* pt = left_seg->mutable_line_segment()->add_point();
      pt->set_x(point.x());
      pt->set_y(point.y());
      pt->set_z(point.z());
    }
    // 右
    auto* right_seg =
        new_lane.mutable_right_boundary()->mutable_curve()->add_segment();
    for (const auto& point : predict_lanelines[i + 1].second) {
      auto* pt = right_seg->mutable_line_segment()->add_point();
      pt->set_x(point.x());
      pt->set_y(point.y());
      pt->set_z(point.z());
    }
    // 查询新车道的所有前继id
    // if (lane_table_.find(section_lane_id[section_lane_id.size() - i - 1]) ==
    //     lane_table_.end()) {
    //   HLOG_ERROR << "lane not found in lane_table";
    //   continue;
    // }
    // std::vector<std::string> pred;
    // if (!lane_table_.at(section_lane_id[section_lane_id.size() - i - 1])
    //          .prev_lane_ids.empty()) {
    //   for (const auto& pred_id :
    //        lane_table_.at(section_lane_id[section_lane_id.size() - i - 1])
    //            .prev_lane_ids) {
    //     new_lane.add_predecessor_id()->set_id(pred_id);
    //     pred.push_back(pred_id);
    //   }
    // }

    // 将new_lane加入到local_msg中
    local_msg_->add_lane()->CopyFrom(new_lane);
    // 将local_msg中对应的pred的后继指向new_lane
    // for (auto& local_lane : *local_msg_->mutable_lane()) {
    //   for (const auto& pd : pred) {
    //     if (local_lane.id().id() == pd) {
    //       local_lane.add_successor_id()->set_id(new_lane.id().id());
    //     }
    //   }
    //   // 左右邻
    //   if (section_lane_id.size() - i - 1 > 0 &&
    //       local_lane.id().id() ==
    //           section_lane_id[section_lane_id.size() - i - 1]) {
    //     local_lane.add_right_neighbor_forward_lane_id()->set_id(
    //         section_lane_id[section_lane_id.size() - i - 2]);
    //   }
    //   if (section_lane_id.size() - i - 1 > 0 &&
    //       local_lane.id().id() ==
    //           section_lane_id[section_lane_id.size() - i - 2]) {
    //     local_lane.add_left_neighbor_forward_lane_id()->set_id(
    //         section_lane_id[section_lane_id.size() - i - 1]);
    //   }
    // }
  }
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
        // auto left_point_size = local_lane.left_boundary()
        //                            .curve()
        //                            .segment(left_seg_size - 1)
        //                            .line_segment()
        //                            .point_size();
        // if (left_point_size == 0) {
        //   continue;
        // }
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
        // auto right_point_size = local_lane.right_boundary()
        //                             .curve()
        //                             .segment(right_seg_size - 1)
        //                             .line_segment()
        //                             .point_size();
        // if (right_point_size == 0) {
        //   continue;
        // }
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
        // auto cen_point_size = local_lane.central_curve()
        //                           .segment(cen_seg_size - 1)
        //                           .line_segment()
        //                           .point_size();
        // if (cen_point_size == 0) {
        //   continue;
        // }
        hq_lane.mutable_central_curve()
            ->mutable_segment(0)
            ->mutable_line_segment()
            ->mutable_point()
            ->CopyFrom(
                local_lane.central_curve().segment(0).line_segment().point());
      }
    }
  }
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

void MapPrediction::ConvertToLocal() {
  if (!hq_map_) {
    HLOG_ERROR << "nullptr hq_map_";
    return;
  }

  if (local_enu_center_flag_) {
    HLOG_ERROR << "init_pose_ not inited";
    return;
  }

  int zone = static_cast<int>(utm_zone_);
  LaneToLocal();
  RoadToLocal(zone);
}

void MapPrediction::LaneToLocal() {
  // local_enu to local
  for (auto& hq_lane : *hq_map_->mutable_lane()) {
    for (auto& seg : *hq_lane.mutable_central_curve()->mutable_segment()) {
      for (auto& pt : *seg.mutable_line_segment()->mutable_point()) {
        Eigen::Vector3d pt_local_enu(pt.x(), pt.y(), pt.z());
        Eigen::Vector3d pt_local = T_local_enu_to_local_ * pt_local_enu;
        pt.set_x(pt_local.x());
        pt.set_y(pt_local.y());
        pt.set_z(pt_local.z());
      }
    }

    for (auto& seg :
         *hq_lane.mutable_left_boundary()->mutable_curve()->mutable_segment()) {
      for (auto& pt : *seg.mutable_line_segment()->mutable_point()) {
        Eigen::Vector3d pt_local_enu(pt.x(), pt.y(), pt.z());
        Eigen::Vector3d pt_local = T_local_enu_to_local_ * pt_local_enu;
        pt.set_x(pt_local.x());
        pt.set_y(pt_local.y());
        pt.set_z(pt_local.z());
      }
    }

    for (auto& seg : *hq_lane.mutable_right_boundary()
                          ->mutable_curve()
                          ->mutable_segment()) {
      for (auto& pt : *seg.mutable_line_segment()->mutable_point()) {
        Eigen::Vector3d pt_local_enu(pt.x(), pt.y(), pt.z());
        Eigen::Vector3d pt_local = T_local_enu_to_local_ * pt_local_enu;
        pt.set_x(pt_local.x());
        pt.set_y(pt_local.y());
        pt.set_z(pt_local.z());
      }
    }
  }
}

void MapPrediction::RoadToLocal(const int& zone) {
  // road to local
  for (auto& hq_road : *hq_map_->mutable_road()) {
    for (auto& sec : *hq_road.mutable_section()) {
      for (auto& edge :
           *sec.mutable_boundary()->mutable_outer_polygon()->mutable_edge()) {
        for (auto& seg : *edge.mutable_curve()->mutable_segment()) {
          for (auto& pt : *seg.mutable_line_segment()->mutable_point()) {
            Eigen::Vector3d pt_utm(pt.x(), pt.y(), pt.z());
            double utm_x = pt.x();
            double utm_y = pt.y();
            bool ret = hozon::common::coordinate_convertor::UTM2GCS(
                zone, &utm_x, &utm_y);
            if (!ret) {
              HLOG_ERROR << "UTM2GCS road point failed, set DBL_MAX value";
              pt.set_x(DBL_MAX);
              pt.set_y(DBL_MAX);
              pt.set_z(DBL_MAX);
              continue;
            }

            Eigen::Vector3d pt_gcj(utm_y, utm_x, 0);
            Eigen::Vector3d pt_local_enu =
                util::Geo::Gcj02ToEnu(pt_gcj, local_enu_center_);

            Eigen::Vector3d pt_local = T_local_enu_to_local_ * pt_local_enu;
            pt.set_x(pt_local.x());
            pt.set_y(pt_local.y());
            pt.set_z(pt_local.z());
          }
        }
      }
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
    if (cent_points.empty()) {
      continue;
    }

    auto* seg = lane.mutable_central_curve()->add_segment();
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

#if 0
void MapPrediction::SmoothAlignment() {
  // 车道线平滑对齐操作
  for (auto& lane : *local_msg_->mutable_lane()) {
    // 创建插值数组
    std::vector<Eigen::Vector3d> left_interp_points;
    std::vector<Eigen::Vector3d> right_interp_points;
    // 找到当前lane的后继id
    std::vector<std::string> next_id;
    if (lane.successor_id().empty()) {
      continue;
    }
    for (const auto& it : lane.successor_id()) {
      next_id.push_back(it.id());
    }
    // 存储当前lane左车道线的最后一个点，并clear
    const auto& left_seg_size = lane.left_boundary().curve().segment_size();
    if (left_seg_size == 0) {
      continue;
    }
    const auto& left_point_size = lane.left_boundary()
                                      .curve()
                                      .segment(left_seg_size - 1)
                                      .line_segment()
                                      .point_size();
    if (left_point_size <= 1) {
      continue;
    }
    const auto& pt = lane.left_boundary()
                         .curve()
                         .segment(left_seg_size - 1)
                         .line_segment()
                         .point(left_point_size - 1);
    Eigen::Vector3d left_end_point(pt.x(), pt.y(), pt.z());
    if (left_point_size == 2) {
      // 插值一个新点
      const auto& ptt = lane.left_boundary()
                            .curve()
                            .segment(left_seg_size - 1)
                            .line_segment()
                            .point(0);
      Eigen::Vector3d left_start_point(ptt.x(), ptt.y(), ptt.z());
      Eigen::Vector3d left_new_point =
          (left_end_point + left_start_point) / 2.0;

      left_interp_points.emplace_back(left_start_point);
      left_interp_points.emplace_back(left_new_point);
    } else {
      const auto& ptt = lane.left_boundary()
                            .curve()
                            .segment(left_seg_size - 1)
                            .line_segment()
                            .point(left_point_size - 3);
      Eigen::Vector3d P1(ptt.x(), ptt.y(), ptt.z());
      const auto& pttt = lane.left_boundary()
                             .curve()
                             .segment(left_seg_size - 1)
                             .line_segment()
                             .point(left_point_size - 2);
      Eigen::Vector3d P2(pttt.x(), pttt.y(), pttt.z());
      left_interp_points.emplace_back(P1);
      left_interp_points.emplace_back(P2);
    }

    const auto& right_seg_size = lane.right_boundary().curve().segment_size();
    if (right_seg_size == 0) {
      continue;
    }
    const auto& right_point_size = lane.right_boundary()
                                       .curve()
                                       .segment(right_seg_size - 1)
                                       .line_segment()
                                       .point_size();
    if (right_point_size <= 1) {
      continue;
    }
    const auto& ptr = lane.right_boundary()
                          .curve()
                          .segment(right_seg_size - 1)
                          .line_segment()
                          .point(right_point_size - 1);
    Eigen::Vector3d right_end_point(ptr.x(), ptr.y(), ptr.z());
    if (right_point_size == 2) {
      // 插值一个新点
      const auto& ptt = lane.right_boundary()
                            .curve()
                            .segment(right_seg_size - 1)
                            .line_segment()
                            .point(0);
      Eigen::Vector3d right_start_point(ptt.x(), ptt.y(), ptt.z());
      Eigen::Vector3d right_new_point =
          (right_end_point + right_start_point) / 2.0;

      right_interp_points.emplace_back(right_start_point);
      right_interp_points.emplace_back(right_new_point);
    } else {
      const auto& ptt = lane.right_boundary()
                            .curve()
                            .segment(right_seg_size - 1)
                            .line_segment()
                            .point(right_point_size - 3);
      Eigen::Vector3d P1(ptt.x(), ptt.y(), ptt.z());
      const auto& pttt = lane.right_boundary()
                             .curve()
                             .segment(right_seg_size - 1)
                             .line_segment()
                             .point(right_point_size - 2);
      Eigen::Vector3d P2(pttt.x(), pttt.y(), pttt.z());
      right_interp_points.emplace_back(P1);
      right_interp_points.emplace_back(P2);
    }

    // 对当前lane的后继id进行处理
    for (auto& next_lane : *local_msg_->mutable_lane()) {
      for (const auto& it : next_id) {
        if (it == next_lane.id().id()) {
          // 存储左边线的起点,并clear
          const auto& next_left_seg_size =
              next_lane.left_boundary().curve().segment_size();
          if (next_left_seg_size == 0) {
            continue;
          }
          const auto& next_left_point_size = next_lane.left_boundary()
                                                 .curve()
                                                 .segment(0)
                                                 .line_segment()
                                                 .point_size();
          if (next_left_point_size <= 1) {
            continue;
          }
          const auto& pt =
              next_lane.left_boundary().curve().segment(0).line_segment().point(
                  0);
          Eigen::Vector3d next_left_start_point(pt.x(), pt.y(), pt.z());
          if (next_left_start_point == left_end_point) {
            continue;
          }
          if (next_left_point_size == 2) {
            // 插值一个新点
            const auto& ptt = next_lane.left_boundary()
                                  .curve()
                                  .segment(0)
                                  .line_segment()
                                  .point(next_left_point_size - 1);
            Eigen::Vector3d next_left_end_point(ptt.x(), ptt.y(), ptt.z());
            Eigen::Vector3d next_left_new_point =
                (next_left_start_point + next_left_end_point) / 2.0;

            left_interp_points.emplace_back(next_left_new_point);
            left_interp_points.emplace_back(next_left_end_point);
          } else {
            const auto& ptt = next_lane.left_boundary()
                                  .curve()
                                  .segment(0)
                                  .line_segment()
                                  .point(1);
            Eigen::Vector3d P1(ptt.x(), ptt.y(), ptt.z());
            const auto& pttt = next_lane.left_boundary()
                                   .curve()
                                   .segment(0)
                                   .line_segment()
                                   .point(2);
            Eigen::Vector3d P2(pttt.x(), pttt.y(), pttt.z());
            left_interp_points.emplace_back(P1);
            left_interp_points.emplace_back(P2);
          }
          // 先开始对left_interp_points进行插值处理
          std::vector<Eigen::Vector3d> interps;
          CatmullRoom(left_interp_points, &interps);
          // 清除next_lane的起点坐标，并将interps中的点插入
          auto next_wpt = next_lane.mutable_left_boundary()
                              ->mutable_curve()
                              ->mutable_segment(0)
                              ->mutable_line_segment()
                              ->mutable_point(0);
          next_wpt->Clear();
          next_wpt->set_x(interps[0].x());
          next_wpt->set_y(interps[0].y());
          next_wpt->set_z(interps[0].z());

          auto curr_wpt = lane.mutable_left_boundary()
                              ->mutable_curve()
                              ->mutable_segment(left_seg_size - 1)
                              ->mutable_line_segment()
                              ->mutable_point(left_point_size - 1);
          curr_wpt->Clear();
          curr_wpt->set_x(interps[0].x());
          curr_wpt->set_y(interps[0].y());
          curr_wpt->set_z(interps[0].z());

          left_interp_points.pop_back();
          left_interp_points.pop_back();

          // 存储右边线的起点,并clear
          const auto& next_right_seg_size =
              next_lane.right_boundary().curve().segment_size();
          if (next_right_seg_size == 0) {
            continue;
          }
          const auto& next_right_point_size = next_lane.right_boundary()
                                                  .curve()
                                                  .segment(0)
                                                  .line_segment()
                                                  .point_size();
          if (next_right_point_size <= 1) {
            continue;
          }
          const auto& ptr = next_lane.right_boundary()
                                .curve()
                                .segment(0)
                                .line_segment()
                                .point(0);
          Eigen::Vector3d next_right_start_point(ptr.x(), ptr.y(), ptr.z());
          if (next_right_start_point == right_end_point) {
            continue;
          }
          if (next_right_point_size == 2) {
            // 插值一个新点
            const auto& ptt = next_lane.right_boundary()
                                  .curve()
                                  .segment(0)
                                  .line_segment()
                                  .point(next_right_point_size - 1);
            Eigen::Vector3d next_right_end_point(ptt.x(), ptt.y(), ptt.z());
            Eigen::Vector3d next_right_new_point =
                (next_right_start_point + next_right_end_point) / 2.0;

            right_interp_points.emplace_back(next_right_new_point);
            right_interp_points.emplace_back(next_right_end_point);
          } else {
            const auto& ptt = next_lane.right_boundary()
                                  .curve()
                                  .segment(0)
                                  .line_segment()
                                  .point(1);
            Eigen::Vector3d P1(ptt.x(), ptt.y(), ptt.z());
            const auto& pttt = next_lane.right_boundary()
                                   .curve()
                                   .segment(0)
                                   .line_segment()
                                   .point(2);
            Eigen::Vector3d P2(pttt.x(), pttt.y(), pttt.z());
            right_interp_points.emplace_back(P1);
            right_interp_points.emplace_back(P2);
          }
          // 先开始对left_interp_points进行插值处理
          std::vector<Eigen::Vector3d> interpsr;
          CatmullRoom(right_interp_points, &interpsr);
          // 清除next_lane的起点坐标，并将interps中的点插入
          auto next_wptr = next_lane.mutable_right_boundary()
                               ->mutable_curve()
                               ->mutable_segment(0)
                               ->mutable_line_segment()
                               ->mutable_point(0);
          next_wptr->Clear();
          next_wptr->set_x(interpsr[0].x());
          next_wptr->set_y(interpsr[0].y());
          next_wptr->set_z(interpsr[0].z());

          auto curr_wptr = lane.mutable_right_boundary()
                               ->mutable_curve()
                               ->mutable_segment(right_seg_size - 1)
                               ->mutable_line_segment()
                               ->mutable_point(right_point_size - 1);
          curr_wptr->Clear();
          curr_wptr->set_x(interpsr[0].x());
          curr_wptr->set_y(interpsr[0].y());
          curr_wptr->set_z(interpsr[0].z());

          right_interp_points.pop_back();
          right_interp_points.pop_back();

          break;
        }
      }
    }
  }
}
#endif

void MapPrediction::Prediction() {
  //  pthread_setname_np(pthread_self(), "pred");
  //  util::Rate rate(10.);
  //  while (!is_pred_proc_stop_) {
  //    rate.Sleep();
  //    HLOG_ERROR << "!!pred Prediction try enter mtx";
  // 存取所有的车道id用于找取道路边界和预测前方车道线时使用
  //    std::lock_guard<std::mutex> lock(mtx_);
  util::TicToc global_tic;
  util::TicToc local_tic;

  CompleteLaneline(end_lane_ids_, end_section_ids_, road_table_);
  //    CompleteLaneline(end_lane_id_, end_section_id, hash_table);
  HLOG_INFO << "pred Prediction CompleteLaneline cost " << local_tic.Toc();
  local_tic.Tic();
  PredictLeftRightLaneline(topo_section_ids_, lane_table_, road_table_,
                           topo_lane_ids_);
  HLOG_INFO << "pred Prediction PredictLeftRightLaneline cost "
            << local_tic.Toc();
  local_tic.Tic();

  // 现在对预测的道路边界补充几何信息
  PredictAheadLaneLine(add_section_ids_, lane_table_, road_table_);
  HLOG_INFO << "pred Prediction PredictAheadLaneLine cost " << local_tic.Toc();
  local_tic.Tic();

  // 对local_msg_中的车道线进行平滑且连接操作
  // SmoothAlignment();
  // 对全部的local_msg_添加中心线
  FitLaneCenterline();
  HLOG_INFO << "pred Prediction FitLaneCenterline cost " << local_tic.Toc();
  local_tic.Tic();
  viz_map_.VizLocalMapLaneLine(local_msg_);
  viz_map_.VizLaneID(local_msg_);

  Eigen::Vector3d pose = location_;
  // viz_map_.VizLocalMsg(local_msg_, pose);

  // 对3公里预测之后的车道添加拓扑关系及其他元素，添加到local_msg_中
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
