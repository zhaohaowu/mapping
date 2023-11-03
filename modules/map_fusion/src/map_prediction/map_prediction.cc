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

DEFINE_bool(pred_run, true, "pred thread run");
DEFINE_uint32(pred_thread_interval, 100, "pred thread interval ms");

namespace hozon {
namespace mp {
namespace mf {

using Vec2d = common::math::Vec2d;

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
  uint32_t zone = std::floor(msg->pos_gcj02().y() / 6.0 + 31);
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
  auto zone = msg->pose().utm_zone_01();
  bool ret = hozon::common::coordinate_convertor::UTM2GCS(
      msg->pose().utm_zone_01(), &utm_x, &utm_y);
  if (!ret) {
    HLOG_ERROR << "UTM2GCS failed";
    return;
  }

  // gcj02
  Eigen::Vector3d pos_global(utm_y, utm_x, 0);
  OnLocationInGlobal(pos_global, zone, msg->pose().pos_utm_01().x(),
                     msg->pose().pos_utm_01().y());
}

void MapPrediction::OnLocationInGlobal(const Eigen::Vector3d& pos_gcj02,
                                       uint32_t utm_zone_id, double utm_x,
                                       double utm_y) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (local_enu_center_flag_) {
    local_enu_center_ << pos_gcj02;
    local_enu_center_flag_ = false;
  }
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

  // PredictAheadLaneLine(localMapLaneLines, hqMapRoadEdge, location);

  // 这里拿到了所有车道的id，可以通过lane id找到其所在的所有road id

  // 假设现在拿到的road只是左边第二个车道的左边界
  /*
  通过车道数量在vehicle道路上进行补充，得到道路的两个边界
  */
  // std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>> left_edge;
  // std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>> right_edge;
  // for (const auto& road : hqMap->road()) {
  //   for (const auto& section : road.section()) {
  //     //
  //     这里开始拿到了一条road(section.boundary().outer_polygon().edge().end())
  //     int lane_num;
  //     std::vector<Eigen::Vector3d> left;
  //     std::vector<Eigen::Vector3d> right;
  //     for (const auto& point : section.boundary()
  //                                  .outer_polygon()
  //                                  .edge()
  //                                  .end()
  //                                  ->curve()
  //                                  .segment()
  //                                  .end()
  //                                  ->line_segment()
  //                                  .point()) {
  //       // 开始遍历所有的点
  //       // 将点转到vehcle坐标系下
  //       Eigen::Vector3d point_left(point.x(), point.y() + 3.5, 0);
  //       Eigen::Vector3d point_right(point.x(), point.y() - 3.5 * (lane_num -
  //       1), 0); left.emplace_back(point_left);
  //       right.emplace_back(point_right);
  //     }
  //     left_edge.emplace_back(std::make_pair(id, left));
  //     right_edge.emplace_back(std::make_pair(id, right));
  //   }
  // }
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
  uint32_t utm_zone = 0;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    //    local_msg_ = msg;
    local_msg_ = std::make_shared<hozon::hdmap::Map>();
    local_msg_->Clear();
    local_msg_->CopyFrom(*msg);
    utm_pos.set_x(location_utm_.x());
    utm_pos.set_y(location_utm_.y());
    utm_pos.set_z(0);
    utm_zone = utm_zone_;
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

    for (const auto& road : roads_in_range) {
      const auto& road_id = road->id().id();
      LocalRoad local_road;
      local_road.road_id = road_id;
      for (const auto& it : road->sections()) {
        Boundary section;
        section.section_id = it.id().id();
        for (const auto& itt : it.lane_id()) {
          section.lane_id.emplace_back(itt.id());
        }
        for (const auto& edge : it.boundary().outer_polygon().edge()) {
          if (edge.type() == 2) {  // left
            std::vector<Eigen::Vector3d> edge_point;
            for (const auto& seg : edge.curve().segment()) {
              for (const auto& point : seg.line_segment().point()) {
                Eigen::Vector3d point_enu = UtmPtToLocalEnu(point);
                edge_point.emplace_back(point_enu);
              }
            }
            if (!edge_point.empty()) {
              section.left_boundary.emplace_back(edge_point);
            }
          }

          if (edge.type() == 3) {  // right
            std::vector<Eigen::Vector3d> edge_point;
            for (const auto& seg : edge.curve().segment()) {
              for (const auto& point : seg.line_segment().point()) {
                Eigen::Vector3d point_enu = UtmPtToLocalEnu(point);
                edge_point.emplace_back(point_enu);
              }
            }
            if (!edge_point.empty()) {
              section.right_boundary.emplace_back(edge_point);
            }
          }
        }
        local_road.section_ids.insert_or_assign(section.section_id, section);
      }
      road_table_.insert_or_assign(road_id, local_road);
    }

    for (const auto& lane : lanes_in_range) {
      const auto& lane_id = lane->id().id();
      LocalLane local_lane;
      local_lane.lane_id = lane_id;
      local_lane.road_id = lane->road_id().id();
      local_lane.section_id = lane->section_id().id();
      if (lane->lane().left_neighbor_forward_lane_id().empty()) {
        local_lane.flag = LocalLane::FAR_LEFT;
      } else if (lane->lane().right_neighbor_forward_lane_id().empty()) {
        local_lane.flag = LocalLane::FAR_RIGHT;
      } else {
        local_lane.flag = LocalLane::MIDDLE;
      }

      //      for (const auto& it :
      //      lane->lane().left_boundary().curve().segment()) {
      //        for (const auto& itt : it.line_segment().point()) {
      //          Eigen::Vector3d pt = UtmPtToLocalEnu(itt);
      //          local_lane.left_line.emplace_back(pt);
      //        }
      //      }
      //      for (const auto& it :
      //      lane->lane().right_boundary().curve().segment()) {
      //        for (const auto& itt : it.line_segment().point()) {
      //          Eigen::Vector3d pt = UtmPtToLocalEnu(itt);
      //          local_lane.right_line.emplace_back(pt);
      //        }
      //      }
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

      lane_table_.insert_or_assign(lane_id, local_lane);
    }

    std::vector<std::vector<Eigen::Vector3d>> temp_edge2;
    for (const auto& it : local_msg_->lane()) {
      const auto& lane_id = it.id().id();
      topo_lane_ids_.emplace_back(lane_id);
      if (lane_table_.find(lane_id) == lane_table_.end()) {
        HLOG_ERROR << "lane in local_msg not found in lane_table";
        continue;
      }
      const auto& local_lane = lane_table_[lane_id];
      topo_section_ids_.insert(local_lane.section_id);
      if (it.successor_id().empty()) {
        end_lane_ids_.emplace_back(lane_id);
        end_section_ids_.insert(local_lane.section_id);

        // 判断当前lane是否都有左右边线
        const auto& left_seg = it.left_boundary().curve().segment_size();
        const auto& right_seg = it.right_boundary().curve().segment_size();

        if ((left_seg == 0 || right_seg == 0) &&
            lane_table_.find(lane_id) != lane_table_.end() &&
            !lane_table_.at(lane_id).prev_lane_ids.empty()) {
          const auto& prev_id = lane_table_.at(lane_id).prev_lane_ids[0];
          end_lane_ids_.emplace_back(prev_id);
          end_section_ids_.insert(lane_table_.at(prev_id).section_id);
          if (left_seg == 0) {
            end_prev_ids_.insert_or_assign(prev_id, 0);
          } else {
            end_prev_ids_.insert_or_assign(prev_id, 1);
          }
        }
      }
      // 额外定义一个哈希表用于存储local_msg中最左和最有的lane
      if (it.left_neighbor_forward_lane_id().empty()) {
        const auto& lane_id = it.id().id();
        FarLane far_lane;
        far_lane.lane_id = lane_id;
        far_lane.flag = 0;
        for (const auto& itt : it.left_boundary().curve().segment()) {
          for (const auto& point : itt.line_segment().point()) {
            Eigen::Vector3d left_line(point.x(), point.y(), point.z());
            if (!far_lane.left_line.empty() &&
                std::find(far_lane.left_line.begin(), far_lane.left_line.end(),
                          left_line) != far_lane.left_line.end()) {
              continue;
            }
            far_lane.left_line.emplace_back(left_line);
          }
        }
        for (const auto& itt : it.right_boundary().curve().segment()) {
          for (const auto& point : itt.line_segment().point()) {
            Eigen::Vector3d right_line(point.x(), point.y(), point.z());
            if (!far_lane.right_line.empty() &&
                std::find(far_lane.right_line.begin(),
                          far_lane.right_line.end(),
                          right_line) != far_lane.right_line.end()) {
              continue;
            }
            far_lane.right_line.emplace_back(right_line);
          }
        }
        far_table_.insert_or_assign(lane_id, far_lane);
      }
      if (it.right_neighbor_forward_lane_id().empty()) {
        const auto& lane_id = it.id().id();
        FarLane far_lane;
        far_lane.lane_id = lane_id;
        far_lane.flag = 1;
        for (const auto& itt : it.left_boundary().curve().segment()) {
          for (const auto& point : itt.line_segment().point()) {
            Eigen::Vector3d left_line(point.x(), point.y(), point.z());
            if (!far_lane.left_line.empty() &&
                std::find(far_lane.left_line.begin(), far_lane.left_line.end(),
                          left_line) != far_lane.left_line.end()) {
              continue;
            }
            far_lane.left_line.emplace_back(left_line);
          }
        }
        for (const auto& itt : it.right_boundary().curve().segment()) {
          for (const auto& point : itt.line_segment().point()) {
            Eigen::Vector3d right_line(point.x(), point.y(), point.z());
            if (!far_lane.right_line.empty() &&
                std::find(far_lane.right_line.begin(),
                          far_lane.right_line.end(),
                          right_line) != far_lane.right_line.end()) {
              continue;
            }
            far_lane.right_line.emplace_back(right_line);
          }
        }
        far_table_.insert_or_assign(lane_id, far_lane);
      }
    }

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

#if 0
void MapPrediction::HashTable(
    const hozon::hdmap::RoadInfoConstPtr& road_ptr,
    const hozon::hdmap::LaneInfoConstPtr& lane_ptr,
    std::unordered_map<std::string, Boundary>* hash_table) {
  // 哈希表用于存储section id--->{left boundary, right boundary}
  if (!road_ptr || !lane_ptr) {
    return;
  }

  for (const auto& section : road_ptr->sections()) {
    if (section.id().id() == lane_ptr->section_id().id()) {
      std::vector<std::vector<Eigen::Vector3d>> boundary1;  // left
      std::vector<std::vector<Eigen::Vector3d>> boundary2;  // right
      for (const auto& edge : section.boundary().outer_polygon().edge()) {
        if (edge.type() == 2) {  // left
          std::vector<Eigen::Vector3d> edge_point;
          for (const auto& seg : edge.curve().segment()) {
            for (const auto& point : seg.line_segment().point()) {
              Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
              int zone = 51;
              double x = point_utm.x();
              double y = point_utm.y();
              hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
              Eigen::Vector3d point_gcj(y, x, 0);
              Eigen::Vector3d point_enu =
                  util::Geo::Gcj02ToEnu(point_gcj, local_enu_center_);
              edge_point.emplace_back(point_enu);
            }
          }
          if (!edge_point.empty()) {
            boundary1.emplace_back(edge_point);
          }
        }

        if (edge.type() == 3) {  // right
          std::vector<Eigen::Vector3d> edge_point;
          for (const auto& seg : edge.curve().segment()) {
            for (const auto& point : seg.line_segment().point()) {
              Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
              int zone = 51;
              double x = point_utm.x();
              double y = point_utm.y();
              hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
              Eigen::Vector3d point_gcj(y, x, 0);
              Eigen::Vector3d point_enu =
                  util::Geo::Gcj02ToEnu(point_gcj, local_enu_center_);
              edge_point.emplace_back(point_enu);
            }
          }
          if (!edge_point.empty()) {
            boundary2.emplace_back(edge_point);
          }
        }
      }
      std::vector<std::string> lane_id;
      for (const auto& id : section.lane_id()) {
        lane_id.emplace_back(id.id());
      }
      // 存储
      (*hash_table)[section.id().id()] = {lane_id, boundary1, boundary2};
    }
  }
}
#endif

#if 1
void MapPrediction::PredictLeftRightLaneline(
    const std::set<std::string>& topo_section_ids_,
    const std::unordered_map<std::string, LocalLane>& lane_table_,
    const std::unordered_map<std::string, LocalRoad>& road_table_,
    const std::vector<std::string>& topo_lane_ids_) {
  if (topo_section_ids_.empty() || road_table_.empty() ||
      topo_lane_ids_.empty() || lane_table_.empty()) {
    return;
  }
  //  std::unordered_map<std::string, LocalLane> lane_table;
  // 构建哈希表
  //  for (const auto& lane : local_msg_->lane()) {
  //    // 左空
  //    if (lane.left_neighbor_forward_lane_id().empty()) {
  //      std::vector<Eigen::Vector3d> left_line;
  //      for (const auto& seg : lane.left_boundary().curve().segment()) {
  //        for (const auto& point : seg.line_segment().point()) {
  //          Eigen::Vector3d point_enu(point.x(), point.y(), point.z());
  //          left_line.push_back(point_enu);
  //        }
  //      }
  //
  //      std::vector<Eigen::Vector3d> right_line;
  //
  //      for (const auto& seg : lane.right_boundary().curve().segment()) {
  //        for (const auto& point : seg.line_segment().point()) {
  //          Eigen::Vector3d point_enu(point.x(), point.y(), point.z());
  //          right_line.push_back(point_enu);
  //        }
  //      }
  //      lane_table[lane.id().id()] = {0, left_line, right_line};
  //    }
  //
  //    // 右空
  //    if (lane.right_neighbor_forward_lane_id().empty()) {
  //      std::vector<Eigen::Vector3d> left_line;
  //      for (const auto& seg : lane.left_boundary().curve().segment()) {
  //        for (const auto& point : seg.line_segment().point()) {
  //          Eigen::Vector3d point_enu(point.x(), point.y(), point.z());
  //          left_line.push_back(point_enu);
  //        }
  //      }
  //
  //      std::vector<Eigen::Vector3d> right_line;
  //
  //      for (const auto& seg : lane.right_boundary().curve().segment()) {
  //        for (const auto& point : seg.line_segment().point()) {
  //          Eigen::Vector3d point_enu(point.x(), point.y(), point.z());
  //          right_line.push_back(point_enu);
  //        }
  //      }
  //
  //      lane_table[lane.id().id()] = {1, left_line, right_line};
  //    }
  //  }

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
    if (it.second.flag == 0) {
      auto left_seg_size = it.second.left_line.size();
      auto road_id = local_lane.road_id;
      auto section_id = local_lane.section_id;
      if (road_table_.find(road_id) == road_table_.end()) {
        HLOG_ERROR << "road not found in road_table";
        continue;
      }
      const auto& local_road = road_table_.at(road_id);
      if (local_road.section_ids.empty()) {
        HLOG_ERROR << "empty section ids";
        continue;
      }
      if (local_road.section_ids.find(section_id) ==
          local_road.section_ids.end()) {
        HLOG_ERROR << "section not found in section ids";
        continue;
      }
      if (local_road.section_ids.at(section_id).lane_id.back() == it.first &&
          left_seg_size != 0) {
        continue;
      }
      // 如果上面没有continue，则有缺失车道，计算缺失车道数量
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
      std::vector<std::vector<Eigen::Vector3d>> boundary1 =
          local_road.section_ids.at(section_id).left_boundary;
      std::vector<Eigen::Vector3d> edge1_;
      for (const auto& bound : boundary1) {
        for (const auto& point : bound) {
          if (!edge1_.empty() && point == edge1_.back()) {
            continue;
          }
          edge1_.push_back(point);
        }
      }
      // 存储右边线
      std::pair<std::string, std::vector<Eigen::Vector3d>> curr_left_line;
      if (!far_table_.at(it.first).right_line.empty()) {
        curr_left_line.first = far_table_.at(it.first).lane_id;
        curr_left_line.second = far_table_.at(it.first).right_line;
      }
      AddLeftOrRightLine(edge1_, curr_left_line, mis_num, 0);
    } else if (it.second.flag == 1) {
      auto right_seg_size = it.second.right_line.size();
      auto road_id = local_lane.road_id;
      auto section_id = local_lane.section_id;
      if (road_table_.find(road_id) == road_table_.end()) {
        HLOG_ERROR << "road not found in road_table";
        continue;
      }
      const auto& local_road = road_table_.at(road_id);
      if (local_road.section_ids.empty()) {
        HLOG_ERROR << "empty section ids";
        continue;
      }
      if (local_road.section_ids.find(section_id) ==
          local_road.section_ids.end()) {
        HLOG_ERROR << "section not found in section ids";
        continue;
      }
      if (local_road.section_ids.at(section_id).lane_id.front() == it.first &&
          right_seg_size != 0) {
        continue;
      }
      // 如果上面没有continue，则有缺失车道，计算缺失车道数量
      uint32_t mis_num = 0;
      bool flag = true;
      for (const auto& itt : local_road.section_ids.at(section_id).lane_id) {
        if (flag) {
          mis_num += 1;
        }
        if (itt == it.first) {
          flag = false;
        }
      }
      std::vector<std::vector<Eigen::Vector3d>> boundary2 =
          local_road.section_ids.at(section_id).right_boundary;
      std::vector<Eigen::Vector3d> edge2_;
      for (const auto& bound : boundary2) {
        for (const auto& point : bound) {
          if (!edge2_.empty() && point.x() == edge2_.back().x()) {
            continue;
          }
          edge2_.push_back(point);
        }
      }
      // 存储右边线
      std::pair<std::string, std::vector<Eigen::Vector3d>> curr_right_line;
      if (!far_table_.at(it.first).left_line.empty()) {
        curr_right_line.first = far_table_.at(it.first).lane_id;
        curr_right_line.second = far_table_.at(it.first).left_line;
      }
      AddLeftOrRightLine(edge2_, curr_right_line, mis_num, 1);
    }
  }

//  for (const auto& lane : lane_table) {
//    if (lane.second.flag == 0) {
//      // 判断左
//      hozon::hdmap::Id lane_id;
//      lane_id.set_id(lane.first);
//      auto lane_ptr = GLOBAL_HD_MAP->GetLaneById(lane_id);
//      if (!lane_ptr) {
//        continue;
//      }
//      // 判断是否有缺失线
//      auto left_seg_size = lane.second.left_line.size();
//      if (hash_table.at(lane_ptr->section_id().id()).lane_id.back() ==
//              lane.first &&
//          left_seg_size != 0) {
//        // 无缺失
//        continue;
//      }
//      // 如果上面没有continue，则有缺失车道，计算缺失车道数量
//      uint32_t mis_num = 0;
//      bool flag = false;
//      for (const auto& it :
//           hash_table.at(lane_ptr->section_id().id()).lane_id) {
//        if (it == lane.first) {
//          flag = true;
//        }
//        if (flag) {
//          mis_num += 1;
//        }
//      }
//
//      // 存储左boundary
//      hash_table.at(lane_ptr->section_id().id()).lane_id.back();
//      std::vector<std::vector<Eigen::Vector3d>> boundary1 =
//          hash_table.at(lane_ptr->section_id().id()).left_boundary;
//      std::vector<Eigen::Vector3d> edge1_;
//      for (const auto& bound : boundary1) {
//        for (const auto& point : bound) {
//          edge1_.push_back(point);
//        }
//      }
//
//      // 存储右边线
//      std::pair<std::string, std::vector<Eigen::Vector3d>> curr_left_line;
//      if (!lane.second.right_line.empty()) {
//        curr_left_line.first = lane.first;
//        curr_left_line.second = lane.second.right_line;
//      }
//
//      AddLeftOrRightLine(edge1_, curr_left_line, mis_num, 0);
//    }
//
//    if (lane.second.flag == 1) {
//      // 判断右
//      hozon::hdmap::Id lane_id;
//      lane_id.set_id(lane.first);
//      auto lane_ptr = GLOBAL_HD_MAP->GetLaneById(lane_id);
//      if (!lane_ptr) {
//        continue;
//      }
//      // 判断是否有缺失线
//      auto right_seg_size = lane.second.right_line.size();
//      if (hash_table.at(lane_ptr->section_id().id()).lane_id.front() ==
//              lane.first &&
//          right_seg_size != 0) {
//        // 无缺失
//        continue;
//      }
//      // 如果上面没有continue，则有缺失车道，计算缺失车道数量
//      uint32_t mis_num = 0;
//      bool flag = true;
//      for (const auto& it :
//           hash_table.at(lane_ptr->section_id().id()).lane_id) {
//        if (flag) {
//          mis_num += 1;
//        }
//        if (it == lane.first) {
//          flag = false;
//        }
//      }
//
//      std::vector<std::vector<Eigen::Vector3d>> boundary2 =
//          hash_table.at(lane_ptr->section_id().id()).right_boundary;
//      std::vector<Eigen::Vector3d> edge2_;
//      for (const auto& bound : boundary2) {
//        for (const auto& point : bound) {
//          edge2_.push_back(point);
//        }
//      }
//
//      std::pair<std::string, std::vector<Eigen::Vector3d>> curr_right_line;
//      if (!lane.second.left_line.empty()) {
//        curr_right_line.first = lane.first;
//        curr_right_line.second = lane.second.left_line;
//      }
//
//      // double right_dist = 0.;
//      // ComputeDistLineToEdge(curr_right_line, edge2_, &right_dist);
//      AddLeftOrRightLine(edge2_, curr_right_line, mis_num, 1);
//    }
//  }
#if 0
  for (const auto& road : roads) {
    for (const auto& it : curr_road_id_) {
      if (road->id().id() == it) {
        for (const auto& road_section : road->sections()) {
          // 存储road_section中的lane_id
          // 判断当前的road_section中包含的lane是否与当前local_msg中的lane有重复
          std::vector<std::string> section_id;
          bool flag = false;
          for (const auto& it : road_section.lane_id()) {
            section_id.emplace_back(it.id());
            for (const auto& local_id : lane_id_) {
              if (local_id == it.id()) {
                flag = true;
              }
            }
          }

          if (!flag) {
            continue;
          }
          // 找取road_section中的两个道路边界
          std::vector<std::vector<Eigen::Vector3d>> boundary1;
          std::vector<std::vector<Eigen::Vector3d>> boundary2;
          for (const auto& edge :
               road_section.boundary().outer_polygon().edge()) {
            if (edge.type() == 2) {
              std::vector<Eigen::Vector3d> edge_point;
              for (const auto& seg : edge.curve().segment()) {
                for (const auto& point : seg.line_segment().point()) {
                  Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
                  int zone = 51;
                  double x = point_utm.x();
                  double y = point_utm.y();
                  hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
                  Eigen::Vector3d point_gcj(y, x, 0);
                  Eigen::Vector3d point_enu =
                      util::Geo::Gcj02ToEnu(point_gcj, local_enu_center_);
                  edge_point.emplace_back(point_enu);
                }
              }
              if (!edge_point.empty()) {
                boundary1.emplace_back(edge_point);
              }
            }

            if (edge.type() == 3) {
              std::vector<Eigen::Vector3d> edge_point;
              for (const auto& seg : edge.curve().segment()) {
                for (const auto& point : seg.line_segment().point()) {
                  Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
                  int zone = 51;
                  double x = point_utm.x();
                  double y = point_utm.y();
                  hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
                  Eigen::Vector3d point_gcj(y, x, 0);
                  Eigen::Vector3d point_enu =
                      util::Geo::Gcj02ToEnu(point_gcj, local_enu_center_);
                  edge_point.emplace_back(point_enu);
                }
              }
              if (!edge_point.empty()) {
                boundary2.emplace_back(edge_point);
              }
            }
          }
          std::vector<Eigen::Vector3d> edge1_;
          for (const auto& bound : boundary1) {
            for (const auto& point : bound) {
              edge1_.emplace_back(point);
            }
          }

          std::vector<Eigen::Vector3d> edge2_;
          for (const auto& bound : boundary2) {
            for (const auto& point : bound) {
              edge2_.emplace_back(point);
            }
          }
          // 以上存储了当前section中的两个道路边界，使用新的策略对时间进行优化
          // 接下来计算每个车道的左边线与boundary1/boundary2道路边界的匹配情况
          std::pair<std::string, std::vector<Eigen::Vector3d>> curr_left_line;
          std::pair<std::string, std::vector<Eigen::Vector3d>> curr_right_line;

          for (const auto& lane : local_msg_->lane()) {
            for (const auto& id : section_id) {
              if (id == lane.id().id() &&
                  lane.left_neighbor_forward_lane_id().empty()) {
                // 记录左边线
                std::vector<Eigen::Vector3d> left_line;
                for (const auto& seg : lane.left_boundary().curve().segment()) {
                  for (const auto& point : seg.line_segment().point()) {
                    // 此时的point是在enu系下
                    Eigen::Vector3d point_enu(point.x(), point.y(), point.z());
                    left_line.emplace_back(point_enu);
                  }
                }
                if (!left_line.empty()) {
                  curr_left_line.first = lane.id().id();
                  curr_left_line.second = left_line;
                }

                // 如果左边线是空，存储右边线
                if (left_line.empty()) {
                  // 记录右边线
                  std::vector<Eigen::Vector3d> right_line;
                  for (const auto& seg :
                       lane.right_boundary().curve().segment()) {
                    for (const auto& point : seg.line_segment().point()) {
                      // 此时的point是在enu系下
                      Eigen::Vector3d point_enu(point.x(), point.y(),
                                                point.z());
                      right_line.emplace_back(point_enu);
                    }
                  }
                  if (!right_line.empty()) {
                    curr_left_line.first = lane.id().id();
                    curr_left_line.second = right_line;
                  }
                }
              }

              if (id == lane.id().id() &&
                  lane.right_neighbor_forward_lane_id().empty()) {
                // 记录右边线
                std::vector<Eigen::Vector3d> right_line;
                for (const auto& seg :
                     lane.right_boundary().curve().segment()) {
                  for (const auto& point : seg.line_segment().point()) {
                    // 此时的point是在enu系下
                    Eigen::Vector3d point_enu(point.x(), point.y(), point.z());
                    right_line.emplace_back(point_enu);
                  }
                }
                if (!right_line.empty()) {
                  curr_right_line.first = lane.id().id();
                  curr_right_line.second = right_line;
                }

                if (right_line.empty()) {
                  // 存储左边线
                  std::vector<Eigen::Vector3d> left_line;
                  for (const auto& seg :
                       lane.left_boundary().curve().segment()) {
                    for (const auto& point : seg.line_segment().point()) {
                      // 此时的point是在enu系下
                      Eigen::Vector3d point_enu(point.x(), point.y(),
                                                point.z());
                      left_line.emplace_back(point_enu);
                    }
                  }
                  if (!left_line.empty()) {
                    curr_right_line.first = lane.id().id();
                    curr_right_line.second = left_line;
                  }
                }
              }
            }
          }
          double left_dist = 0.;
          ComputeDistLineToEdge(curr_left_line, edge1_,
                                &left_dist);  // 边-->线
          AddLeftOrRightLine(edge1_, curr_left_line, left_dist, 0);

          double right_dist = 0.;
          ComputeDistLineToEdge(curr_right_line, edge2_, &right_dist);
          AddLeftOrRightLine(edge2_, curr_right_line, right_dist, 1);
        }
      }
    }
  }
#endif
}
#endif
void MapPrediction::ComputeDistLineToEdge(
    const std::pair<std::string, std::vector<Eigen::Vector3d>>& curr_line,
    const std::vector<Eigen::Vector3d>& edge, double* dist) {
  // 分别计算每条车道边线到edge的距离
  if (curr_line.second.empty() || edge.empty()) {
    return;
  }

  double min_distance = 0;  // 存储最小距离
  uint32_t count = 0;
  for (const auto& point : curr_line.second) {
    bool flag = false;
    for (size_t i = 1; i < edge.size(); i++) {
      Eigen::Vector3d A = edge[i - 1];
      Eigen::Vector3d B = edge[i];
      Eigen::Vector3d P = point;
      // 计算A点和B点的方向向量
      Eigen::Vector3d AB = B - A;
      Eigen::Vector3d AP = P - A;
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
      // min_distance = (P - C).norm();   // 点到线段的距离
      min_distance += (P - C).norm();
      count += 1;
      flag = true;
      break;
    }
    if (flag) {
      break;
    }
  }
  double mean_distance = min_distance / count;
  *dist = mean_distance;
}

void MapPrediction::AddLeftOrRightLine(
    const std::vector<Eigen::Vector3d>& edge,
    const std::pair<std::string, std::vector<Eigen::Vector3d>>& curr_line,
    const uint32_t& mis_num, const uint32_t& record) {
  // 开始拟合车道线并赋予拓扑关系
  if (edge.empty() || curr_line.second.empty()) {
    return;
  }
  // uint32_t num_miss_line = uint32_t(dist / 3);  // 判断缺失车道数量
  if (mis_num == 0) {
    return;
  }

  std::vector<std::vector<Eigen::Vector3d>> predict_line(mis_num);
  // 加入起点
  if (curr_line.second.size() >= 2) {
    Eigen::Vector3d P = edge.front();
    Eigen::Vector3d A(curr_line.second[0]);
    Eigen::Vector3d B(curr_line.second[1]);

    Eigen::Vector3d AP = P - A;
    Eigen::Vector3d AB = B - A;
    double t = AP.dot(AB) / AB.squaredNorm();
    Eigen::Vector3d C = A + t * AB;

    double k = (P.y() - C.y()) / (P.x() - C.x());
    double b = C.y() - k * C.x();
    if (!std::isnan(k)) {
      for (uint32_t j = 0; j < mis_num; j++) {
        double ratio = j * 1.0 / mis_num;
        double x = ratio * (C.x() - P.x()) + P.x();
        double y = k * x + b;
        Eigen::Vector3d new_point(x, y, 0);
        predict_line[mis_num - j - 1].push_back(new_point);
      }
    }
  }
  // 拟合中间点
  for (const auto& line_point : curr_line.second) {
    for (size_t i = 1; i < edge.size(); ++i) {
      Eigen::Vector3d A = edge[i - 1];
      Eigen::Vector3d B = edge[i];
      Eigen::Vector3d P = line_point;
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

      double k = (P.y() - C.y()) / (P.x() - C.x());
      double b = C.y() - k * C.x();
      if (std::isnan(k)) {
        continue;
      }
      for (uint32_t j = 1; j < mis_num + 1; j++) {
        double ratio = j * 1.0 / mis_num;
        double x = ratio * (C.x() - P.x()) + P.x();
        double y = k * x + b;
        Eigen::Vector3d new_point(x, y, 0);
        predict_line[j - 1].push_back(new_point);
      }
      break;
    }
  }
  // 加入终点
  if (curr_line.second.size() >= 2) {
    double size = curr_line.second.size();
    Eigen::Vector3d P = edge.back();
    Eigen::Vector3d A(curr_line.second[size - 1]);
    Eigen::Vector3d B(curr_line.second[size - 2]);

    Eigen::Vector3d AP = P - A;
    Eigen::Vector3d AB = B - A;
    double t = AP.dot(AB) / AB.squaredNorm();
    Eigen::Vector3d C = A + t * AB;

    double k = (P.y() - C.y()) / (P.x() - C.x());
    double b = C.y() - k * C.x();
    if (!std::isnan(k)) {
      for (uint32_t j = 0; j < mis_num; j++) {
        double ratio = j * 1.0 / mis_num;
        double x = ratio * (C.x() - P.x()) + P.x();
        double y = k * x + b;
        Eigen::Vector3d new_point(x, y, 0);
        predict_line[mis_num - j - 1].push_back(new_point);
      }
    }
  }

  if (predict_line.empty()) {
    return;
  }
  // 对预测的左右缺失车道线进行可视化
  viz_map_.VizAddSideLaneLine(predict_line);

  // 将预测的车道线添加到local_msg中，并赋予拓扑关系
  AddSideTopological(predict_line, record, curr_line.first);
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
    std::string id_ = curr_id;
    bool flag = true;
    for (const auto& line : predict_line) {
      HLOG_ERROR << "!mf line size " << line.size();
      // 现将当前lane的左边线补充完整
      if (id_ == curr_id && flag) {
        for (auto& lane : *local_msg_->mutable_lane()) {
          if (lane.id().id() == id_ && !id_.empty()) {
            auto left_seg =
                lane.mutable_left_boundary()->mutable_curve()->add_segment();
            for (const auto& point : line) {
              auto pt = left_seg->mutable_line_segment()->add_point();
              pt->set_x(point.x());
              pt->set_y(point.y());
              pt->set_z(point.z());
            }
          }
        }
        flag = false;
        continue;
      }
      // 定义一个新的车道
      auto new_lane = local_msg_->add_lane();
      // 从topo_map_中找取对应id的左邻和后继id
      std::string id_left_id;
      std::string id_next_id;

      // 这里要注意id_left_id/id_next_id是否为空
      if (lane_table_.find(id_) == lane_table_.end()) {
        HLOG_ERROR << "lane not found in lane_table";
        continue;
      }
      if (!lane_table_.at(id_).left_lane_ids.empty()) {
        id_left_id = lane_table_.at(id_).left_lane_ids.front();
        // 赋予id
        new_lane->mutable_id()->set_id(id_left_id);
      }
      if (!lane_table_.at(id_).next_lane_ids.empty()) {
        id_next_id = lane_table_.at(id_).next_lane_ids.front();
      }

      //      hozon::hdmap::Id lane_id;
      //      lane_id.set_id(id_);
      //      auto lane_ptr = GLOBAL_HD_MAP->GetLaneById(lane_id);
      //      if (lane_ptr) {
      //        auto lane = lane_ptr->lane();
      //        if (!lane.left_neighbor_forward_lane_id().empty()) {
      //          id_left_id = lane.left_neighbor_forward_lane_id(0).id();
      //        }
      //        if (!lane.successor_id().empty()) {
      //          id_next_id = lane.successor_id(0).id();
      //        }
      //      }
      //      // 赋予id
      //      new_lane->mutable_id()->set_id(id_left_id);
      // new_lane赋予新几何
      auto left_seg =
          new_lane->mutable_left_boundary()->mutable_curve()->add_segment();
      for (const auto& point : line) {
        auto pt = left_seg->mutable_line_segment()->add_point();
        pt->set_x(point.x());
        pt->set_y(point.y());
        pt->set_z(point.z());
      }
      // 确定当前lane后继lane的左邻lane
      std::string id_next_left_id;
      if (!id_next_id.empty() &&
          lane_table_.find(id_next_id) != lane_table_.end()) {
        //!!!! TBD
        if (lane_table_.find(id_next_id) == lane_table_.end()) {
          HLOG_ERROR << "lane not found in lane_table";
          continue;
        }
        if (!lane_table_.at(id_next_id).left_lane_ids.empty()) {
          id_next_left_id = lane_table_.at(id_next_id).left_lane_ids.front();
        }

        // hozon::hdmap::Id lane_id;
        // lane_id.set_id(id_next_id);
        // auto lane_ptr = GLOBAL_HD_MAP->GetLaneById(lane_id);
        // if (lane_ptr) {
        //   auto lane = lane_ptr->lane();
        //   if (!lane.left_neighbor_forward_lane_id().empty()) {
        //     id_next_left_id = lane.left_neighbor_forward_lane_id(0).id();
        //   }
        // }
      }
      // new_lane的后继指向id_next_left_id
      if (!id_next_left_id.empty()) {
        new_lane->add_successor_id()->set_id(id_next_left_id);
      }

      // new_lane加入到local_msg
      for (auto& lane : *local_msg_->mutable_lane()) {
        if (lane.id().id() == id_ && !id_.empty()) {
          new_lane->mutable_right_boundary()->CopyFrom(lane.left_boundary());
          new_lane->add_right_neighbor_forward_lane_id()->set_id(
              lane.id().id());
          lane.add_left_neighbor_forward_lane_id()->set_id(new_lane->id().id());
          break;
        }
      }
      // 完善前后继关系
      for (auto& lane : *local_msg_->mutable_lane()) {
        if (lane.id().id() == id_next_left_id && !id_next_left_id.empty()) {
          lane.add_predecessor_id()->set_id(new_lane->id().id());
          break;
        }
      }

      id_ = id_left_id;
      if (id_.empty()) {
        break;
      }
    }
  }

  // 添加右
  if (record == 1) {
    // 预测缺失的右边线
    std::string id_ = curr_id;
    bool flag = true;
    for (const auto& line : predict_line) {
      // 现将当前lane的右边线补充完整
      if (id_ == curr_id && flag) {
        for (auto& lane : *local_msg_->mutable_lane()) {
          if (lane.id().id() == id_ && !id_.empty()) {
            auto right_seg =
                lane.mutable_right_boundary()->mutable_curve()->add_segment();
            for (const auto& point : line) {
              auto pt = right_seg->mutable_line_segment()->add_point();
              pt->set_x(point.x());
              pt->set_y(point.y());
              pt->set_z(point.z());
            }
          }
        }
        flag = false;
        continue;
      }
      // 定义一个新的车道
      auto new_lane = local_msg_->add_lane();
      // 从topo_map_中找取对应id的左邻和后继id
      std::string id_right_id;
      std::string id_next_id;

      // 这里要注意id_left_id/id_next_id是否为空
      if (lane_table_.find(id_) == lane_table_.end()) {
        HLOG_ERROR << "lane not found in lane_table";
        continue;
      }
      if (!lane_table_.at(id_).right_lane_ids.empty()) {
        id_right_id = lane_table_.at(id_).right_lane_ids.front();
        // 赋予id
        new_lane->mutable_id()->set_id(id_right_id);
      }
      if (!lane_table_.at(id_).next_lane_ids.empty()) {
        id_next_id = lane_table_.at(id_).next_lane_ids.front();
      }
      // hozon::hdmap::Id lane_id;
      // lane_id.set_id(id_);
      // auto lane_ptr = GLOBAL_HD_MAP->GetLaneById(lane_id);
      // if (lane_ptr) {
      //   auto lane = lane_ptr->lane();
      //   if (!lane.right_neighbor_forward_lane_id().empty()) {
      //     id_right_id = lane.right_neighbor_forward_lane_id(0).id();
      //   }
      //   if (!lane.successor_id().empty()) {
      //     id_next_id = lane.successor_id(0).id();
      //   }
      // }
      // if (id_right_id.empty()) {
      //   break;
      // }
      // new_lane赋予新id
      new_lane->mutable_id()->set_id(id_right_id);

      // new_lane赋予新几何
      auto right_seg =
          new_lane->mutable_right_boundary()->mutable_curve()->add_segment();
      for (const auto& point : line) {
        auto pt = right_seg->mutable_line_segment()->add_point();
        pt->set_x(point.x());
        pt->set_y(point.y());
        pt->set_z(point.z());
      }
      // 确定当前lane后继lane的右邻lane
      std::string id_next_right_id;
      if (!id_next_id.empty()) {
        if (lane_table_.find(id_next_id) == lane_table_.end()) {
          HLOG_ERROR << "lane not found in lane_table";
          continue;
        }
        if (!lane_table_.at(id_next_id).right_lane_ids.empty()) {
          id_next_right_id = lane_table_.at(id_next_id).right_lane_ids.front();
        }
        // hozon::hdmap::Id lane_id;
        // lane_id.set_id(id_next_id);
        // auto lane_ptr = GLOBAL_HD_MAP->GetLaneById(lane_id);
        // if (lane_ptr) {
        //   auto lane = lane_ptr->lane();
        //   if (!lane.right_neighbor_forward_lane_id().empty()) {
        //     id_next_right_id = lane.right_neighbor_forward_lane_id(0).id();
        //   }
        // }
      }
      // new_lane的后继指向id_next_left_id
      if (!id_next_right_id.empty()) {
        new_lane->add_successor_id()->set_id(id_next_right_id);
      }

      // new_lane加入到local_msg中
      for (auto& lane : *local_msg_->mutable_lane()) {
        if (lane.id().id() == id_ && !id_.empty()) {
          new_lane->mutable_left_boundary()->CopyFrom(lane.right_boundary());
          new_lane->add_left_neighbor_forward_lane_id()->set_id(lane.id().id());
          lane.add_right_neighbor_forward_lane_id()->set_id(
              new_lane->id().id());
          break;
        }
      }
      // 完善前后继关系
      for (auto& lane : *local_msg_->mutable_lane()) {
        if (lane.id().id() == id_next_right_id && !id_next_right_id.empty()) {
          lane.add_predecessor_id()->set_id(new_lane->id().id());
          break;
        }
      }

      id_ = id_right_id;
      if (id_.empty()) {
        break;
      }
    }
  }
  // 拓扑关系和几何添加完毕
}

void MapPrediction::PredictAheadLaneLine(
    const std::vector<std::string> add_section_ids_,
    const std::unordered_map<std::string, LocalLane>& lane_table_,
    const std::unordered_map<std::string, LocalRoad>& road_table_) {
  // 用于预测前方车道线
  if (add_section_ids_.empty() || lane_table_.empty() || road_table_.empty()) {
    return;
  }
  std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>
      predict_lanelines;
#if 0
  // 存储已经计算过的section
  std::vector<std::string> has_section;
  while (has_section.size() != add_section_id.size()) {
    for (const auto& lane : local_msg_->lane()) {
      if (!lane.successor_id().empty()) {
        continue;
      }
      auto lane_ptr = GLOBAL_HD_MAP->GetLaneById(lane.id());
      if (!lane_ptr || lane_ptr->lane().successor_id().empty()) {
        continue;
      }
      for (const auto& succ_id : lane_ptr->lane().successor_id()) {
        auto succ_lane_ptr = GLOBAL_HD_MAP->GetLaneById(succ_id);
        if (!succ_lane_ptr) {
          continue;
        }
        for (const auto& sec : add_section_id) {
          if (sec != succ_lane_ptr->section_id().id() ||
              std::find(has_section.begin(), has_section.end(), sec) !=
                  has_section.end()) {
            continue;
          }
          std::vector<std::vector<Eigen::Vector3d>> boundary1 =
              hash_table.at(sec).left_boundary;
          std::vector<std::vector<Eigen::Vector3d>> boundary2 =
              hash_table.at(sec).right_boundary;
          uint32_t lane_num = hash_table.at(sec).lane_id.size();
          FitAheadLaneLine(boundary1, boundary2, predict_lanelines, lane_num);
          // 可视化
          // viz_map_.VizAddAheadLaneLine(predict_lanelines);
          // 对预测的车道继承HQ的ID并添加拓扑关系
          AheadTopological(predict_lanelines, hash_table.at(sec).lane_id);
          has_section.emplace_back(sec);
        }
      }
    }
  }
#endif
  // 这里有个假设，即add_section_id严格的按照顺序执行的
  for (const auto& sec : add_section_ids_) {
    std::string curr_lane_id;
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
    // STD!!!
    const auto& local_lane = lane_table_.at(curr_lane_id);
    auto road_id = local_lane.road_id;
    auto section_id = local_lane.section_id;
    if (road_table_.find(road_id) == road_table_.end()) {
      HLOG_ERROR << "road not found in road_table";
      continue;
    }
    const auto& local_road = road_table_.at(road_id);
    if (local_road.section_ids.empty()) {
      HLOG_ERROR << "empty section ids";
      continue;
    }
    if (local_road.section_ids.find(section_id) ==
        local_road.section_ids.end()) {
      HLOG_ERROR << "section not found in section ids";
      continue;
    }
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
    uint32_t lane_num = local_road.section_ids.at(sec).lane_id.size();
    FitAheadLaneLine(boundary1, boundary2, predict_lanelines, lane_num);
    // 可视化
    // viz_map_.VizAddAheadLaneLine(predict_lanelines);
    // 对预测的车道继承HQ的ID并添加拓扑关系
    AheadTopological(predict_lanelines, local_road.section_ids.at(sec).lane_id);
  }
#if 0
  for (const auto& road : roads) {
    for (const auto& id : add_road_id_) {
      if (road->id().id() == id) {
        // 通过两侧的道路边界拟合车道线
        // bool flag_id = false;
        // int flag_sec_id;
        for (const auto& road_section : road->sections()) {
          // 判断是否从local_msg_中的末端id开始往前填充
          // bool flag_in_end = false;
          // for (const auto& lane : local_msg_->lane()) {
          //   if (lane.successor_id().empty()) {
          //     hozon::hdmap::Id lane_id;
          //     lane_id.set_id(lane.id().id());
          //     auto lane_ptr = GLOBAL_HD_MAP->GetLaneById(lane_id);
          //     if (lane_ptr) {
          //       auto lane = lane_ptr->lane();
          //       if (lane.successor_id().empty()) {
          //         continue;
          //       }
          //       for (const auto& succ_id : lane.successor_id()) {
          //         for (const auto& section_id : road_section.lane_id()) {
          //           if (section_id.id() == succ_id.id()) {
          //             flag_in_end = true;
          //             break;
          //           }
          //         }
          //       }
          //     }
          //   }
          // }

          // 判断road_section中所包含的lane_id是否与local_msg中的end_id有重合部分
          bool flag_id = false;
          for (const auto& section_id : road_section.lane_id()) {
            for (const auto& lan_id : add_lane_id_) {
              if (lan_id == section_id.id()) {
                flag_id = true;
                break;
              }
            }
          }

          if (!flag_id) {
            continue;
          }

          /*
          会用到road_section中的id,用于补拓扑关系
          */
          // std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>
          // edge_; int edge_id = 0;
          std::vector<std::vector<Eigen::Vector3d>> boundary1;
          std::vector<std::vector<Eigen::Vector3d>> boundary2;
          for (const auto& edge :
               road_section.boundary().outer_polygon().edge()) {
            if (edge.type() == 2) {
              std::vector<Eigen::Vector3d> edge_point;
              for (const auto& seg : edge.curve().segment()) {
                for (const auto& point : seg.line_segment().point()) {
                  Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
                  int zone = 51;
                  double x = point_utm.x();
                  double y = point_utm.y();
                  hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
                  Eigen::Vector3d point_gcj(y, x, 0);
                  Eigen::Vector3d point_enu =
                      util::Geo::Gcj02ToEnu(point_gcj, local_enu_center_);
                  edge_point.emplace_back(point_enu);
                }
              }
              if (!edge_point.empty()) {
                boundary1.emplace_back(edge_point);
                // edge_.emplace_back(std::make_pair(edge_id, edge_point));
                // edge_id += 1;
              }
            }

            if (edge.type() == 3) {
              std::vector<Eigen::Vector3d> edge_point;
              for (const auto& seg : edge.curve().segment()) {
                for (const auto& point : seg.line_segment().point()) {
                  Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
                  int zone = 51;
                  double x = point_utm.x();
                  double y = point_utm.y();
                  hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
                  Eigen::Vector3d point_gcj(y, x, 0);
                  Eigen::Vector3d point_enu =
                      util::Geo::Gcj02ToEnu(point_gcj, local_enu_center_);
                  edge_point.emplace_back(point_enu);
                }
              }
              if (!edge_point.empty()) {
                boundary2.emplace_back(edge_point);
                // edge_.emplace_back(std::make_pair(edge_id, edge_point));
                // edge_id += 1;
              }
            }
            // std::vector<Eigen::Vector3d> edge_point;
            // for (const auto& seg : edge.curve().segment()) {
            //   for (const auto& point : seg.line_segment().point()) {
            //     Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
            //     int zone = 51;
            //     double x = point_utm.x();
            //     double y = point_utm.y();
            //     hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
            //     Eigen::Vector3d point_gcj(y, x, 0);
            //     Eigen::Vector3d point_enu =
            //         util::Geo::Gcj02ToEnu(point_gcj, local_enu_center_);
            //     edge_point.emplace_back(point_enu);
            //   }
            // }
            // if (!edge_point.empty()) {
            //   edge_.emplace_back(std::make_pair(edge_id, edge_point));
            //   edge_id += 1;
            // }
          }
          // std::vector<std::vector<Eigen::Vector3d>> boundary1;
          // std::vector<std::vector<Eigen::Vector3d>> boundary2;
          // separateBoundaries(edge_, &boundary1, &boundary2);
          // 求boundary1和boundary2的最小投影点
          uint32_t lane_num = road_section.lane_id_size();
          FitAheadLaneLine(boundary1, boundary2, predict_lanelines, lane_num);
          // 可视化
          viz_map_.VizAddAheadLaneLine(predict_lanelines);
          // 对预测的车道继承HQ的ID并添加拓扑关系
          std::vector<std::string> section_lane_id;
          for (const auto& it : road_section.lane_id()) {
            section_lane_id.emplace_back(it.id());
          }
          AheadTopological(predict_lanelines, topo_map_, section_lane_id);
        }
      }
    }
  }
  // viz_map_.VizLaneID(local_msg_, local_enu_center_);
  // viz_map_.VizAddAheadLaneLine(predict_lanelines);
#endif
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

    // 拟合线
    std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>
        complete_lines;
    uint32_t lane_num = hash_table.at(sec).lane_id.size();
    FitAheadLaneLine(left_boundary, right_boundary, complete_lines, lane_num);
    // viz_map_.VizAddAheadLaneLine(complete_lines);
    // 延伸
    ExpansionLaneLine(complete_lines, com_id, hash_table.at(sec).lane_id);
  }
#if 0
  for (const auto& it : end_lane_id_) {
    hozon::hdmap::Id lane_id;
    lane_id.set_id(it);
    auto lane_ptr = GLOBAL_HD_MAP->GetLaneById(lane_id);
    if (!lane_ptr) {
      continue;
    }

    auto road_id = lane_ptr->road_id();
    auto road_ptr = GLOBAL_HD_MAP->GetRoadById(road_id);
  }
  // 对原车道信息进行补全
  for (const auto& road : roads) {
    for (const auto& it : end_road_id_) {
      if (road->id().id() == it) {
        // 对当前road的两个边界拟合车道线
        for (const auto& road_section : road->sections()) {
          // 存储road_section中的lane_id
          std::vector<std::string> sec_id;
          for (const auto& it : road_section.lane_id()) {
            sec_id.emplace_back(it.id());
          }
          // 是否要先判断当前road_section中所包含的lane_id是否与end_lane_id_有交集,
          // 有交集就保存
          std::vector<std::string> com_id;
          bool flag = false;
          for (const auto& sec_lane_id : sec_id) {
            if (std::find(end_lane_id_.begin(), end_lane_id_.end(),
                          sec_lane_id) != end_lane_id_.end()) {
              com_id.emplace_back(sec_lane_id);
              flag = true;
            }
          }
          if (!flag) {
            continue;
          }
          std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>
              complete_lines;
          // 找当前road_section所在的两个road
          // std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>
          // edge_; int edge_id = 0;
          std::vector<std::vector<Eigen::Vector3d>> boundary1;
          std::vector<std::vector<Eigen::Vector3d>> boundary2;
          for (const auto& edge :
               road_section.boundary().outer_polygon().edge()) {
            if (edge.type() == 2) {
              std::vector<Eigen::Vector3d> edge_point;
              for (const auto& seg : edge.curve().segment()) {
                for (const auto& point : seg.line_segment().point()) {
                  Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
                  int zone = 51;
                  double x = point_utm.x();
                  double y = point_utm.y();
                  hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
                  Eigen::Vector3d point_gcj(y, x, 0);
                  Eigen::Vector3d point_enu =
                      util::Geo::Gcj02ToEnu(point_gcj, local_enu_center_);
                  edge_point.emplace_back(point_enu);
                }
              }
              if (!edge_point.empty()) {
                boundary1.emplace_back(edge_point);
              }
            }

            if (edge.type() == 3) {
              std::vector<Eigen::Vector3d> edge_point;
              for (const auto& seg : edge.curve().segment()) {
                for (const auto& point : seg.line_segment().point()) {
                  Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
                  int zone = 51;
                  double x = point_utm.x();
                  double y = point_utm.y();
                  hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
                  Eigen::Vector3d point_gcj(y, x, 0);
                  Eigen::Vector3d point_enu =
                      util::Geo::Gcj02ToEnu(point_gcj, local_enu_center_);
                  edge_point.emplace_back(point_enu);
                }
              }
              if (!edge_point.empty()) {
                boundary2.emplace_back(edge_point);
              }
            }
          }
          // std::vector<std::vector<Eigen::Vector3d>> boundary1;
          // std::vector<std::vector<Eigen::Vector3d>> boundary2;
          // separateBoundaries(edge_, &boundary1, &boundary2);
          // 求boundary1和boundary2的最小投影点
          uint32_t lane_num = road_section.lane_id_size();
          double start_time = clock();
          FitAheadLaneLine(boundary1, boundary2, complete_lines, lane_num);
          double end_time = clock();
          // viz_map_.VizAddAheadLaneLine(complete_lines);
          // 这里是否要加入一个判断
          // 通过拟合出的车道线对当前lane进行延伸
          ExpansionLaneLine(complete_lines, com_id, sec_id);
        }
      }
    }
  }
#endif
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
      uint32_t com_fit;
      if (end_prev_ids_.find(end_id) != end_prev_ids_.end() &&
          end_prev_ids_.at(end_id) == 0) {
        // 只延伸左侧
        com_fit = 0;
      } else if (end_prev_ids_.find(end_id) != end_prev_ids_.end() &&
                 end_prev_ids_.at(end_id) == 1) {
        // 只延伸右侧
        com_fit = 1;
      } else {
        // 左右都延伸
        com_fit = 2;
      }
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
      int left_line_id = complete_lines.size() - 2 - seq;
      // 右
      int right_line_id = complete_lines.size() - 1 - seq;

      if (com_fit == 0 || com_fit == 2) {
        // 左边界
        // 临时加了一个判断条件
        if (lane.left_boundary().curve().segment().empty()) {
          continue;
        }
        double seg_size = lane.left_boundary().curve().segment().size();
        double point_size = lane.left_boundary()
                                .curve()
                                .segment()[seg_size - 1]
                                .line_segment()
                                .point()
                                .size();
        // 临时加了一个判断条件
        if (seg_size == 0 || point_size == 0) {
          continue;
        }
        // 存储左边线的所有点
        std::vector<Eigen::Vector3d> com_left_line;
        for (const auto& seg : lane.left_boundary().curve().segment()) {
          for (const auto& point : seg.line_segment().point()) {
            Eigen::Vector3d point_enu(point.x(), point.y(), point.z());
            com_left_line.emplace_back(point_enu);
          }
        }
        // 左边界最后一个点
        Eigen::Vector3d left_end_point(com_left_line.back().x(),
                                       com_left_line.back().y(),
                                       com_left_line.back().z());
        // com_left_line.pop_back();
        // 左边界最后一个点
        // Eigen::Vector3d left_end_point(lane.left_boundary()
        //                                    .curve()
        //                                    .segment()[seg_size - 1]
        //                                    .line_segment()
        //                                    .point()[point_size - 1]
        //                                    .x(),
        //                                lane.left_boundary()
        //                                    .curve()
        //                                    .segment()[seg_size - 1]
        //                                    .line_segment()
        //                                    .point()[point_size - 1]
        //                                    .y(),
        //                                lane.left_boundary()
        //                                    .curve()
        //                                    .segment()[seg_size - 1]
        //                                    .line_segment()
        //                                    .point()[point_size - 1]
        //                                    .z());
        // 左边界最后一个点到新拟合车道线的投影点的距离
        double min_distance_left = std::numeric_limits<double>::max();
        uint32_t flag_count_left = 0;

        uint32_t count_left;  // 记录从第几个点开始扩充
        for (size_t i = 1; i < complete_lines[left_line_id].second.size();
             ++i) {
          Eigen::Vector3d A = complete_lines[left_line_id].second[i - 1];
          Eigen::Vector3d B = complete_lines[left_line_id].second[i];
          Eigen::Vector3d P = left_end_point;

          // 计算A点和B点的方向向量
          Eigen::Vector3d AB = B - A;
          Eigen::Vector3d AP = P - A;
          // 计算AB模长
          double ABLengthSquared = AB.squaredNorm();
          if (ABLengthSquared == 0) {
            continue;
          }
          double t = AB.dot(AP) / ABLengthSquared;
          count_left = i;
          if (t < 0 || t > 1) {
            continue;
          }

          t = std::max(0.0, std::min(t, 1.0));
          Eigen::Vector3d C = A + t * AB;  // 点到线段的最近点
          double dis = (P - C).norm();
          if (dis < min_distance_left && dis < 3.75) {
            min_distance_left = dis;
            flag_count_left = count_left;
          }
          break;
        }
        // 判断是否满足距离要求
        if (min_distance_left == std::numeric_limits<double>::max()) {
          continue;
        }

        // STD!!!!
        auto seg_num = lane.left_boundary().curve().segment_size();
        // 临时加入判断条件
        if (seg_num == 0) {
          continue;
        }
        std::vector<Eigen::Vector3d> interp_points;
        if (complete_lines[left_line_id].second.size() - flag_count_left < 2 ||
            com_left_line.size() < 2) {
          // 无法进行插值拟合
          for (size_t i = flag_count_left;
               i < complete_lines[left_line_id].second.size(); ++i) {
            auto pt = lane.mutable_left_boundary()
                          ->mutable_curve()
                          ->mutable_segment(seg_num - 1)
                          ->mutable_line_segment()
                          ->add_point();
            pt->set_x(complete_lines[left_line_id].second[i].x());
            pt->set_y(complete_lines[left_line_id].second[i].y());
            pt->set_z(complete_lines[left_line_id].second[i].z());
          }
        } else {
          Eigen::Vector3d local_first = com_left_line[com_left_line.size() - 2];
          Eigen::Vector3d local_second =
              com_left_line[com_left_line.size() - 1];
          double com_size = complete_lines[left_line_id].second.size();
          Eigen::Vector3d com_first =
              complete_lines[left_line_id].second[com_size - 2];
          Eigen::Vector3d com_second =
              complete_lines[left_line_id].second[com_size - 1];
          interp_points.emplace_back(local_first);
          interp_points.emplace_back(local_second);
          interp_points.emplace_back(com_first);
          interp_points.emplace_back(com_second);
          std::vector<Eigen::Vector3d> cat_points;
          cat_points.emplace_back(local_first);
          CatmullRoom(interp_points, &cat_points);
          cat_points.emplace_back(com_second);
          for (const auto& it : cat_points) {
            auto pt = lane.mutable_left_boundary()
                          ->mutable_curve()
                          ->mutable_segment(seg_num - 1)
                          ->mutable_line_segment()
                          ->add_point();
            pt->set_x(it.x());
            pt->set_y(it.y());
            pt->set_z(it.z());
          }
        }

        // 满足要求，对该车道的边界点进行延伸
        // for (size_t i = flag_count_left + 1;
        //      i < complete_lines[left_line_id].second.size(); i++) {
        //   com_left_line.emplace_back(complete_lines[left_line_id].second[i]);
        // }

        // auto seg_num = lane.left_boundary().curve().segment_size();
        // // 临时加入判断条件
        // if (seg_num == 0) {
        //   continue;
        // }
        // // 清除原lane中的车道线点
        // lane.mutable_left_boundary()
        //     ->mutable_curve()
        //     ->mutable_segment(seg_num - 1)
        //     ->mutable_line_segment()
        //     ->clear_point();
        // // 对compan_lines进行重新样条采样
        // std::vector<Eigen::Vector3d> cat_points;
        // cat_points.emplace_back(com_left_line.front());
        // CatmullRoom(com_left_line, &cat_points);
        // cat_points.emplace_back(com_left_line.back());
        // for (const auto& it : cat_points) {
        //   auto pt = lane.mutable_left_boundary()
        //                 ->mutable_curve()
        //                 ->mutable_segment(seg_num - 1)
        //                 ->mutable_line_segment()
        //                 ->add_point();
        //   pt->set_x(it.x());
        //   pt->set_y(it.y());
        //   pt->set_z(it.z());
        // }
        // compan_lines.emplace_back(cat_points);
      }

      if (com_fit == 1 || com_fit == 2) {
        // 右边界
        // 临时加了一个判断条件
        // if (!lane.right_neighbor_forward_lane_id().empty()) {
        //   continue;
        // }
        if (lane.right_boundary().curve().segment().empty()) {
          continue;
        }
        double seg_size_ = lane.right_boundary().curve().segment().size();
        double point_size_ = lane.right_boundary()
                                 .curve()
                                 .segment()[seg_size_ - 1]
                                 .line_segment()
                                 .point()
                                 .size();
        // 临时加了一个判断条件
        if (seg_size_ == 0 || point_size_ == 0) {
          continue;
        }
        // 存储右边线的所有点
        std::vector<Eigen::Vector3d> com_right_line;
        for (const auto& seg : lane.right_boundary().curve().segment()) {
          for (const auto& point : seg.line_segment().point()) {
            Eigen::Vector3d point_enu(point.x(), point.y(), point.z());
            com_right_line.emplace_back(point_enu);
          }
        }
        // 右边界最后一个点
        Eigen::Vector3d right_end_point(com_right_line.back().x(),
                                        com_right_line.back().y(),
                                        com_right_line.back().z());
        // 右边界最后一个点
        // Eigen::Vector3d right_end_point(lane.right_boundary()
        //                                     .curve()
        //                                     .segment()[seg_size_ - 1]
        //                                     .line_segment()
        //                                     .point()[point_size_ - 1]
        //                                     .x(),
        //                                 lane.right_boundary()
        //                                     .curve()
        //                                     .segment()[seg_size_ - 1]
        //                                     .line_segment()
        //                                     .point()[point_size_ - 1]
        //                                     .y(),
        //                                 lane.right_boundary()
        //                                     .curve()
        //                                     .segment()[seg_size_ - 1]
        //                                     .line_segment()
        //                                     .point()[point_size_ - 1]
        //                                     .z());
        // 左边界最后一个点到新拟合车道线的投影点的距离
        double min_distance_right = std::numeric_limits<double>::max();
        uint32_t flag_count_right = 0;

        uint32_t count_right = 1;  // 记录从第几个点开始扩充
        for (size_t i = 1; i < complete_lines[right_line_id].second.size();
             ++i) {
          Eigen::Vector3d A = complete_lines[right_line_id].second[i - 1];
          Eigen::Vector3d B = complete_lines[right_line_id].second[i];
          Eigen::Vector3d P = right_end_point;

          // 计算A点和B点的方向向量
          Eigen::Vector3d AB = B - A;
          Eigen::Vector3d AP = P - A;
          // 计算AB模长
          double ABLengthSquared = AB.squaredNorm();
          if (ABLengthSquared == 0) {
            continue;
          }
          double t = AB.dot(AP) / ABLengthSquared;
          count_right = i;
          if (t < 0 || t > 1) {
            continue;
          }

          t = std::max(0.0, std::min(t, 1.0));
          Eigen::Vector3d C = A + t * AB;  // 点到线段的最近点
          double dis = (P - C).norm();
          if (dis < min_distance_right && dis < 3.75) {
            min_distance_right = dis;
            flag_count_right = count_right;
          }
          break;
        }
        // 判断是否满足距离要求
        if (min_distance_right == std::numeric_limits<double>::max()) {
          continue;
        }

        // STD!!!!
        auto seg_num_right = lane.right_boundary().curve().segment_size();
        // 临时加入判断条件
        if (seg_num_right == 0) {
          continue;
        }
        std::vector<Eigen::Vector3d> interp_points_right;
        if (complete_lines[right_line_id].second.size() - flag_count_right <
                2 ||
            com_right_line.size() < 2) {
          // 无法进行插值拟合
          for (size_t i = flag_count_right;
               i < complete_lines[right_line_id].second.size(); ++i) {
            auto pt = lane.mutable_right_boundary()
                          ->mutable_curve()
                          ->mutable_segment(seg_num_right - 1)
                          ->mutable_line_segment()
                          ->add_point();
            pt->set_x(complete_lines[right_line_id].second[i].x());
            pt->set_y(complete_lines[right_line_id].second[i].y());
            pt->set_z(complete_lines[right_line_id].second[i].z());
          }
        } else {
          Eigen::Vector3d local_first =
              com_right_line[com_right_line.size() - 2];
          Eigen::Vector3d local_second =
              com_right_line[com_right_line.size() - 1];
          double com_size = complete_lines[right_line_id].second.size();
          Eigen::Vector3d com_first =
              complete_lines[right_line_id].second[com_size - 2];
          Eigen::Vector3d com_second =
              complete_lines[right_line_id].second[com_size - 1];
          interp_points_right.emplace_back(local_first);
          interp_points_right.emplace_back(local_second);
          interp_points_right.emplace_back(com_first);
          interp_points_right.emplace_back(com_second);
          std::vector<Eigen::Vector3d> cat_points;
          cat_points.emplace_back(local_first);
          CatmullRoom(interp_points_right, &cat_points);
          cat_points.emplace_back(com_second);
          for (const auto& it : cat_points) {
            auto pt = lane.mutable_right_boundary()
                          ->mutable_curve()
                          ->mutable_segment(seg_num_right - 1)
                          ->mutable_line_segment()
                          ->add_point();
            pt->set_x(it.x());
            pt->set_y(it.y());
            pt->set_z(it.z());
          }
        }

        // 满足要求，对该车道的边界点进行延伸
        // std::vector<Eigen::Vector3d> com_line;
        // com_line.emplace_back(right_end_point);
        // for (size_t i = flag_count_right + 1;
        //      i < complete_lines[right_line_id].second.size(); i++) {
        //   com_right_line.emplace_back(complete_lines[right_line_id].second[i]);
        // }
        // auto seg_num_right = lane.right_boundary().curve().segment_size();
        // if (seg_num_right == 0) {
        //   continue;
        // }
        // // 清除原lane中的车道线点
        // lane.mutable_right_boundary()
        //     ->mutable_curve()
        //     ->mutable_segment(seg_num_right - 1)
        //     ->mutable_line_segment()
        //     ->clear_point();
        // // 对compan_lines进行重新插值采样
        // std::vector<Eigen::Vector3d> cat_points_right;
        // cat_points_right.emplace_back(com_right_line.front());
        // CatmullRoom(com_right_line, &cat_points_right);
        // cat_points_right.emplace_back(com_right_line.back());
        // for (const auto& it : cat_points_right) {
        //   auto pt = lane.mutable_right_boundary()
        //                 ->mutable_curve()
        //                 ->mutable_segment(seg_num_right - 1)
        //                 ->mutable_line_segment()
        //                 ->add_point();
        //   pt->set_x(it.x());
        //   pt->set_y(it.y());
        //   pt->set_z(it.z());
        // }
        // compan_lines.emplace_back(cat_points_right);
        // 延伸结束
      }
    }
  }
  // 可视化延伸的车道线
  // viz_map_.VizCompanLane(compan_lines);
}

void MapPrediction::CatmullRoom(
    const std::vector<Eigen::Vector3d>& campan_point,
    std::vector<Eigen::Vector3d>* cat_points) {
  // 拟合-->重采样
  if (campan_point.size() < 4) {
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
  for (size_t i = 1; i < campan_point.size() - 2; ++i) {
    for (double t = 0; t < 1; t += 0.1) {
      double px = func(campan_point[i - 1].x(), campan_point[i].x(),
                       campan_point[i + 1].x(), campan_point[i + 2].x(), t);
      double py = func(campan_point[i - 1].y(), campan_point[i].y(),
                       campan_point[i + 1].y(), campan_point[i + 2].y(), t);
      Eigen::Vector3d point(px, py, 0);
      cat_points->emplace_back(point);
    }
  }
}

void MapPrediction::separateBoundaries(
    const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>& edge_,
    std::vector<std::vector<Eigen::Vector3d>>* boundary1,
    std::vector<std::vector<Eigen::Vector3d>>* boundary2) {
  if (!boundary1 || !boundary2) {
    return;
  }

  std::vector<Eigen::Vector3d> startSegment1 = edge_.front().second;
  findConnectedSegments(startSegment1, edge_, boundary1);

  std::vector<Eigen::Vector3d> startSegment2;
  for (const auto& segment : edge_) {
    bool isSegmentInBoundary1 = false;
    for (const auto& resultSegment : *boundary1) {
      if (resultSegment == segment.second) {
        isSegmentInBoundary1 = true;
        break;
      }
    }
    if (!isSegmentInBoundary1) {
      startSegment2 = segment.second;
    }
  }
  findConnectedSegments(startSegment2, edge_, boundary2);
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

void MapPrediction::findConnectedSegments(
    const std::vector<Eigen::Vector3d>& startSegment1,
    const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>& edge_,
    std::vector<std::vector<Eigen::Vector3d>>* boundary1) {
  if (!boundary1) {
    return;
  }

  boundary1->push_back(startSegment1);

  for (const auto& segment : edge_) {
    bool isSegmentInResult = false;
    for (const auto& resultSegment : *boundary1) {
      if (resultSegment == segment.second) {
        isSegmentInResult = true;
        break;
      }
    }

    if (!isSegmentInResult &&
        (segment.second.front() == boundary1->back().back() ||
         segment.second.back() == boundary1->back().front())) {
      findConnectedSegments(segment.second, edge_, boundary1);
      break;
    }
  }
}

void MapPrediction::DetermineEdgeAssPair(
    const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
        all_edges,
    std::vector<std::pair<std::pair<uint32_t, uint32_t>, double>>&
        boundary_pairs) {
  // 求取车道之间的关联对
  for (const auto& edge : all_edges) {
    double min_distance = std::numeric_limits<double>::max();
    double ass_id;
    for (size_t i = 0; i < all_edges.size(); ++i) {
      if (all_edges[i].first == edge.first) {
        continue;
      }
      // 遍历edge中的所有点,并存储每个点到对应点的平均距离
      std::vector<double> distance;
      if (edge.second.size() == 1 && all_edges[i].second.size() == 1 &&
          uint32_t((edge.second.back() - all_edges[i].second.back()).norm() /
                   3.5) == 3) {
        double distance =
            (edge.second.back() - all_edges[i].second.back()).norm();
        std::pair<uint32_t, uint32_t> id_pair =
            std::make_pair(edge.first, all_edges[i].first);
        boundary_pairs.emplace_back(std::make_pair(id_pair, distance));
      }
      for (const auto& point : edge.second) {
        for (size_t j = 1; j < all_edges[i].second.size(); ++j) {
          Eigen::Vector3d A = all_edges[i].second[j - 1];
          Eigen::Vector3d B = all_edges[i].second[j];
          Eigen::Vector3d P = point;

          // 计算A点和B点的方向向量
          Eigen::Vector3d AB = B - A;
          Eigen::Vector3d AP = P - A;
          // 计算AB模长
          double ABLengthSquared = AB.squaredNorm();
          double t = AB.dot(AP) / ABLengthSquared;

          if (t < 0 || t > 1) {
            continue;
          }

          t = std::max(0.0, std::min(t, 1.0));
          Eigen::Vector3d C = A + t * AB;  // 点到线段的最近点
          double dis = (point - C).norm();
          distance.emplace_back(dis);
          break;
        }
      }
      if (distance.empty()) {
        continue;
      }
      double sum = std::accumulate(distance.begin(), distance.end(), 0.0);
      double mean_distance = sum / distance.size();
      if (mean_distance < min_distance && sum != 0) {
        min_distance = mean_distance;
        ass_id = all_edges[i].first;
      }
    }
    if (min_distance != std::numeric_limits<double>::max()) {
      std::pair<uint32_t, uint32_t> id_pair =
          std::make_pair(edge.first, ass_id);
      boundary_pairs.emplace_back(std::make_pair(id_pair, min_distance));
    }
  }
}

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
  for (int i = predict_lanelines.size() - 2; i >= 0; --i) {
    if (std::find(end_lane_ids_.begin(), end_lane_ids_.end(),
                  section_lane_id[section_lane_id.size() - i - 1]) !=
        end_lane_ids_.end()) {
      continue;
    }
    // 定义新车道
    hozon::hdmap::Lane new_lane;
    // 为新车道赋予Id
    new_lane.mutable_id()->set_id(
        section_lane_id[section_lane_id.size() - i - 1]);
    // 为新车道添加几何信息
    // 左
    auto left_seg =
        new_lane.mutable_left_boundary()->mutable_curve()->add_segment();
    for (const auto& point : predict_lanelines[i].second) {
      auto pt = left_seg->mutable_line_segment()->add_point();
      pt->set_x(point.x());
      pt->set_y(point.y());
      pt->set_z(point.z());
    }
    // 右
    auto right_seg =
        new_lane.mutable_right_boundary()->mutable_curve()->add_segment();
    for (const auto& point : predict_lanelines[i + 1].second) {
      auto pt = right_seg->mutable_line_segment()->add_point();
      pt->set_x(point.x());
      pt->set_y(point.y());
      pt->set_z(point.z());
    }
    // 查询新车道的所有前继id
    if (lane_table_.find(section_lane_id[section_lane_id.size() - i - 1]) ==
        lane_table_.end()) {
      HLOG_ERROR << "lane not found in lane_table";
      continue;
    }
    std::vector<std::string> pred;
    if (!lane_table_.at(section_lane_id[section_lane_id.size() - i - 1])
             .prev_lane_ids.empty()) {
      for (const auto& pred_id :
           lane_table_.at(section_lane_id[section_lane_id.size() - i - 1])
               .prev_lane_ids) {
        new_lane.add_predecessor_id()->set_id(pred_id);
        pred.push_back(pred_id);
      }
    }
    // hozon::hdmap::Id lane_id;
    // lane_id.set_id(section_lane_id[section_lane_id.size() - i - 1]);
    // auto lane_ptr = GLOBAL_HD_MAP->GetLaneById(lane_id);
    // if (lane_ptr) {
    //   auto lane = lane_ptr->lane();
    //   if (!lane.predecessor_id().empty()) {
    //     for (const auto& pred_id : lane.predecessor_id()) {
    //       new_lane.add_predecessor_id()->set_id(pred_id.id());
    //       pred.push_back(pred_id.id());
    //     }
    //   }
    // }
    // 将new_lane加入到local_msg中
    local_msg_->add_lane()->CopyFrom(new_lane);
    // 将local_msg中对应的pred的后继指向new_lane
    for (auto& local_lane : *local_msg_->mutable_lane()) {
      for (const auto& pd : pred) {
        if (local_lane.id().id() == pd) {
          local_lane.add_successor_id()->set_id(new_lane.id().id());
        }
      }
      // 左右邻
      if (section_lane_id.size() - i - 1 > 0 &&
          local_lane.id().id() ==
              section_lane_id[section_lane_id.size() - i - 1]) {
        local_lane.add_right_neighbor_forward_lane_id()->set_id(
            section_lane_id[section_lane_id.size() - i - 2]);
      }
      if (section_lane_id.size() - i - 1 > 0 &&
          local_lane.id().id() ==
              section_lane_id[section_lane_id.size() - i - 2]) {
        local_lane.add_left_neighbor_forward_lane_id()->set_id(
            section_lane_id[section_lane_id.size() - i - 1]);
      }
    }
  }

  // 补全预测车道之间的左右邻关系
  // AddAheadLeftRightTopo(local_msg_);
}

std::shared_ptr<hozon::hdmap::Map> MapPrediction::GetPredictionMap() {
  std::lock_guard<std::mutex> lock_map(mtx_);
  //  return local_msg_;
  return hq_map_;
}

void MapPrediction::AddAheadLeftRightTopo(hozon::hdmap::Map* local_msg_) {
  if (!local_msg_) {
    return;
  }

  for (auto& lane : local_msg_->lane()) {
    // 增添拓扑关系
    /*
    1 2 3
    4 5 6
    7 8 9
    */
    // 记录当前lane的左邻和后继和右邻
    std::string left_id;
    std::string successor_id;
    std::string right_id;
    if (!lane.left_neighbor_forward_lane_id().empty()) {
      left_id = lane.left_neighbor_forward_lane_id()[0].id();
    }
    if (!lane.successor_id().empty()) {
      successor_id = lane.successor_id()[0].id();
    }
    if (!lane.right_neighbor_forward_lane_id().empty()) {
      right_id = lane.right_neighbor_forward_lane_id()[0].id();
    }

    // left_id和right_id对应的车道后继id
    std::string left_successor_id_;
    std::string right_successor_id_;
    for (auto& lane_it : local_msg_->lane()) {
      if (lane_it.id().id() == left_id && !left_id.empty() &&
          !lane_it.successor_id().empty()) {
        left_successor_id_ = lane_it.successor_id()[0].id();
      }
      if (lane_it.id().id() == right_id && !right_id.empty() &&
          !lane_it.successor_id().empty()) {
        right_successor_id_ = lane_it.successor_id()[0].id();
      }
    }

    // 找到successor_id对应的车道
    int count = 0;
    for (auto& lane_it : local_msg_->lane()) {
      if (!successor_id.empty() && lane_it.id().id() == successor_id &&
          !left_successor_id_.empty()) {
        local_msg_->mutable_lane(count)
            ->add_left_neighbor_forward_lane_id()
            ->set_id(left_successor_id_);
      }
      if (!successor_id.empty() && lane_it.id().id() == successor_id &&
          !right_successor_id_.empty()) {
        local_msg_->mutable_lane(count)
            ->add_right_neighbor_forward_lane_id()
            ->set_id(right_successor_id_);
      }
      count += 1;
    }
  }
}

void MapPrediction::AddResTopo() {
  if (!hq_map_) {
    HLOG_ERROR << "hq_map_ not ready";
    return;
  }
  // 清除三公里的全部车道线/中心线点
  util::TicToc tic;
  //  std::shared_ptr<hozon::hdmap::Map> hq_map =
  //      std::make_shared<hozon::hdmap::Map>();
  //  GLOBAL_HD_MAP->GetMap(hq_map.get());
  //  HLOG_ERROR << "!!pred AddResTopo GetMap " << tic.Toc();
  //  GLOBAL_HD_MAP->GetMapWithoutLaneGeometry(hq_map.get());
  //  HLOG_ERROR << "!!pred AddResTopo GetMapWithoutLaneGeometry " << tic.Toc();
  //  tic.Tic();
  //  for (auto& lane : *hq_map->mutable_lane()) {
  //    auto seg_left =
  //        lane.mutable_left_boundary()->mutable_curve()->mutable_segment();
  //    for (auto& point : *seg_left) {
  //      point.mutable_line_segment()->clear_point();
  //    }
  //    auto seg_right =
  //        lane.mutable_right_boundary()->mutable_curve()->mutable_segment();
  //    for (auto& point : *seg_right) {
  //      point.mutable_line_segment()->clear_point();
  //    }
  //    auto seg_cen = lane.mutable_central_curve()->mutable_segment();
  //    for (auto& point : *seg_cen) {
  //      point.mutable_line_segment()->clear_point();
  //    }
  //  }
  //  HLOG_ERROR << "!!pred AddResTopo clear pts " << tic.Toc();
  //  tic.Tic();
  // 将local_msg_中的车道线点加入对应的hq_map中
  for (auto& hq_lane : *hq_map_->mutable_lane()) {
    for (const auto& local_lane : local_msg_->lane()) {
      if (hq_lane.id().id() == local_lane.id().id()) {
        // 加入左边线点
        auto left_seg_size = local_lane.left_boundary().curve().segment_size();
        if (left_seg_size == 0) {
          continue;
        }
        auto left_point_size = local_lane.left_boundary()
                                   .curve()
                                   .segment(left_seg_size - 1)
                                   .line_segment()
                                   .point_size();
        if (left_point_size == 0) {
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
        auto right_point_size = local_lane.right_boundary()
                                    .curve()
                                    .segment(right_seg_size - 1)
                                    .line_segment()
                                    .point_size();
        if (right_point_size == 0) {
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
        auto cen_point_size = local_lane.central_curve()
                                  .segment(cen_seg_size - 1)
                                  .line_segment()
                                  .point_size();
        if (cen_point_size == 0) {
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
  HLOG_ERROR << "pred AddResTopo insert pts " << tic.Toc();
  tic.Tic();

  //  hozon::hdmap::Header raw_header;
  //  raw_header.CopyFrom(local_msg_->header());
  local_msg_->Clear();
  //  local_msg_->CopyFrom(*hq_map);
  //  local_msg_->mutable_header()->CopyFrom(raw_header);

  HLOG_ERROR << "pred AddResTopo local_msg_ copyfrom " << tic.Toc();
  tic.Tic();

#if 0
  // 向3公里剩余的一段添加拓扑关系
  // 首先取local_msg_中的end_id
  std::vector<std::string> local_end_ids;
  for (const auto& lane : local_msg_->lane()) {
    if (lane.successor_id().empty()) {
      local_end_ids.emplace_back(lane.id().id());
    }
  }

  // 此时拿到了3公里剩余的全部lane_id,现将其塞入local_msg_中，并赋予拓扑关系
  bool flag = true;
  while (flag && !local_end_ids.empty()) {
    std::vector<hozon::hdmap::Lane> new_lanes;
    for (const auto& end_id : local_end_ids) {
      hozon::hdmap::Id lane_id;
      lane_id.set_id(end_id);
      auto lane_ptr = GLOBAL_HD_MAP->GetLaneById(lane_id);
      if (lane_ptr) {
        auto lane = lane_ptr->lane();
        for (const auto& succ_id : lane.successor_id()) {
          // 创建新的lane，并赋予id和拓扑
          hozon::hdmap::Lane new_lane;
          new_lane.mutable_id()->set_id(succ_id.id());
          // 前继
          new_lane.add_predecessor_id()->set_id(end_id);
          // 左右邻
          auto lane_succ_ptr = GLOBAL_HD_MAP->GetLaneById(succ_id);
          if (lane_succ_ptr) {
            auto lane_succ = lane_succ_ptr->lane();
            if (!lane_succ.right_neighbor_forward_lane_id().empty()) {
              new_lane.add_right_neighbor_forward_lane_id()->set_id(
                  lane_succ.right_neighbor_forward_lane_id(0).id());
            }
            if (!lane_succ.left_neighbor_forward_lane_id().empty()) {
              new_lane.add_left_neighbor_forward_lane_id()->set_id(
                  lane_succ.left_neighbor_forward_lane_id(0).id());
            }
          }
          // 将new_lane塞到local_msg_中
          local_msg_->add_lane()->CopyFrom(new_lane);
          // 将local_msg中对应的pred的后继指向new_lane
          int count = 0;
          for (auto& local_lane : local_msg_->lane()) {
            if (local_lane.id().id() == end_id) {
              local_msg_->mutable_lane(count)->add_successor_id()->set_id(
                  new_lane.id().id());
            }
            count += 1;
          }
          new_lanes.emplace_back(new_lane);
        }
      }
    }

    for (const auto& lane : local_msg_->lane()) {
      if (lane.successor_id().empty()) {
        local_end_ids.emplace_back(lane.id().id());
      }
    }
    if (new_lanes.empty()) {
      flag = false;
    }
  }
#endif
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
  HLOG_ERROR << "!!!local_msg_->lane() size " << local_msg_->lane_size();
  for (auto& lane : *local_msg_->mutable_lane()) {
    std::vector<Vec2d> left_point;
    std::vector<Vec2d> right_point;
    // 存储左
    for (const auto& seg : lane.left_boundary().curve().segment()) {
      for (const auto& point : seg.line_segment().point()) {
        Vec2d point_enu(point.x(), point.y());
        if (!left_point.empty() &&
            std::find(left_point.begin(), left_point.end(), point_enu) !=
                left_point.end()) {
          continue;
        }
        left_point.emplace_back(point_enu);
      }
    }
    // 存储右
    for (const auto& seg : lane.right_boundary().curve().segment()) {
      for (const auto& point : seg.line_segment().point()) {
        Vec2d point_enu(point.x(), point.y());
        if (!right_point.empty() &&
            std::find(right_point.begin(), right_point.end(), point_enu) !=
                right_point.end()) {
          continue;
        }
        right_point.emplace_back(point_enu);
      }
    }

    std::vector<Vec2d> cent_points;
    if (left_point.empty() || right_point.empty()) {
      continue;
    }
    cent_points.clear();
    common::math::GenerateCenterPoint(left_point, right_point, &cent_points);
    // 手动实现
    // std::vector<Vec2d> project_point;
    // std::vector<Vec2d> side_point;
    // if (left_point.size() >= right_point.size()) {
    //   project_point = left_point;
    //   side_point = right_point;
    // } else {
    //   project_point = right_point;
    //   side_point = left_point;
    // }
    // // 加入起点
    // if (side_point.size() >= 2) {
    //   Vec2d P = project_point.front();
    //   Vec2d A = side_point[0];
    //   Vec2d B = side_point[1];

    //   Vec2d AP = P - A;
    //   Vec2d AB = B - A;
    //   double t = AB.InnerProd(AP) / AB.LengthSquare();
    //   Vec2d C = A + t * AB;

    //   Vec2d cen_point = (P + C) / 2.0;
    //   cent_points.emplace_back(cen_point);
    // }
    // // 中间点
    // for (const auto& point : project_point) {
    //   for (size_t j = 1; j < side_point.size(); ++j) {
    //     Vec2d A = side_point[j - 1];
    //     Vec2d B = side_point[j];
    //     Vec2d P = point;

    //     // 计算A点和B点的方向向量
    //     Vec2d AB = B - A;
    //     Vec2d AP = P - A;
    //     // 计算AB模长
    //     double ABLengthSquared = AB.LengthSquare();
    //     double t = AB.InnerProd(AP) / ABLengthSquared;

    //     if (t < 0 || t > 1) {
    //       continue;
    //     }

    //     t = std::max(0.0, std::min(t, 1.0));
    //     Vec2d C = A + t * AB;  // 点到线段的最近点
    //     double dis = point.DistanceTo(C);

    //     Vec2d cen_point = (P + C) / 2.0;
    //     cent_points.emplace_back(cen_point);
    //     break;
    //   }
    // }

    // 加入终点
    // if (side_point.size() >= 2) {
    //   Vec2d P = project_point.back();
    //   double size = side_point.size();
    //   Vec2d A = side_point[size - 1];
    //   Vec2d B = side_point[size - 2];

    //   Vec2d AP = P - A;
    //   Vec2d AB = B - A;
    //   double t = AB.InnerProd(AP) / AB.LengthSquare();
    //   Vec2d C = A + t * AB;

    //   Vec2d cen_point = (P + C) / 2.0;
    //   cent_points.emplace_back(cen_point);
    // }
    // cent_points.emplace_back((left_point.back() + right_point.back()) / 2.0);
    viz_map_.VizCenterLane(cent_points);

    // 此时获得了中心线上的点,塞入local_msg_中
    if (cent_points.empty()) {
      continue;
    }

    auto seg = lane.mutable_central_curve()->add_segment();
    for (const auto& cen_point : cent_points) {
      if (std::isnan(cen_point.x()) || std::isnan(cen_point.y())) {
        HLOG_ERROR << "isnan";
        continue;
      }
      auto pt = seg->mutable_line_segment()->add_point();
      pt->set_x(cen_point.x());
      pt->set_y(cen_point.y());
      pt->set_z(0.);
    }
  }
}

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
  // 对已有的车道信息进行补全
  // viz_map_.VizHqMapRoad(roads_in_range, all_road_id_, local_enu_center_);

  // 现在对预测的道路边界补充几何信息
  PredictAheadLaneLine(add_section_ids_, lane_table_, road_table_);
  HLOG_INFO << "pred Prediction PredictAheadLaneLine cost " << local_tic.Toc();
  local_tic.Tic();
  // 对全部的local_msg_添加中心线
  FitLaneCenterline();
  HLOG_INFO << "pred Prediction FitLaneCenterline cost " << local_tic.Toc();
  local_tic.Tic();
  viz_map_.VizLocalMapLaneLine(localmap_lanelines, local_msg_);
  viz_map_.VizLaneID(local_msg_, local_enu_center_);

  // 对3公里预测之后的车道添加拓扑关系及其他元素，添加到local_msg_中
  AddResTopo();
  HLOG_INFO << "pred Prediction AddResTopo cost " << local_tic.Toc();
  local_tic.Tic();
  // 现在开始验证完整的local_msg
  Eigen::Vector3d pose = location_;
  // 这里可视化拓扑关系的情况会报索引越界的错误（记得后面要修改）
  // viz_map_.VizLocalMsg(local_msg_, pose);
  auto global_cost = global_tic.Toc();
  HLOG_INFO << "pred Prediction cost " << global_cost;
  //  }
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
