/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： topo_assignment.cc
 *   author     ： xuliang
 *   date       ： 2023.09
 ******************************************************************************/

#include "map_fusion/topo_assignment/topo_assignment.h"

#include <gflags/gflags.h>

#include <limits>

#include "depend/common/utm_projection/coordinate_convertor.h"
#include "util/temp_log.h"
namespace hozon {
namespace mp {
namespace mf {

int TopoAssignment::Init() {
  if (!RVIZ_AGENT.Ok()) {
    HLOG_WARN << "RvizAgent start failed";
  } else {
    int ret = RVIZ_AGENT.Register<adsfi_proto::viz::TransformStamped>(
        kTopicTopoAsignTf);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register " << kTopicTopoAsignTf << " failed";
    }

    ret = RVIZ_AGENT.Register<adsfi_proto::viz::Path>(kTopicTopoAsignLocation);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register " << kTopicTopoAsignLocation
                << " failed";
    }

    ret = RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(
        KTopicTopoAsignLocalMap);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register " << KTopicTopoAsignLocalMap
                << " failed";
    }

    ret = RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(
        KTopicTopoAsignHQMapRoad);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register " << KTopicTopoAsignHQMapRoad
                << " failed";
    }

    ret = RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(
        KTopicTopoAsignHQMapLane);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register " << KTopicTopoAsignHQMapLane
                << " failed";
    }

    ret = RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(
        KTopicTopoAsignTopoMapRoad);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register " << KTopicTopoAsignTopoMapRoad
                << " failed";
    }

    ret = RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(
        KTopicTopoAsignTopoMapLane);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register " << KTopicTopoAsignTopoMapLane
                << " failed";
    }

    ret = RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>("lane");
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register "
                << "lane"
                << " failed";
    }
    ret = RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>("left");
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register "
                << "left"
                << " failed";
    }
    ret = RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>("right");
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register "
                << "right"
                << " failed";
    }
  }

  topo_map_ = std::make_shared<hozon::hdmap::Map>();
  crop_map_ = std::make_shared<hozon::hdmap::Map>();
  hq_map_server_ = std::make_shared<hozon::hdmap::HDMap>();
  return 0;
}

void TopoAssignment::OnInsNodeInfo(
    const std::shared_ptr<hozon::localization::HafNodeInfo>& msg) {
  // 更新车辆ins位置

  vehicle_pose_ << msg->pos_gcj02().x(), msg->pos_gcj02().y(),
      msg->pos_gcj02().z();

  Eigen::Quaterniond q_W_V(msg->quaternion().w(), msg->quaternion().x(),
                           msg->quaternion().y(), msg->quaternion().z());
  q_W_V.normalize();
  if (!init_) {
    HLOG_INFO << "ref_point_ = pose";
    ref_point_ = vehicle_pose_;
  }
  init_ = true;

  Eigen::Vector3d enu = util::Geo::Gcj02ToEnu(vehicle_pose_, ref_point_);
  VizLocation(enu, q_W_V, msg->header().gnss_stamp());

  {
    std::lock_guard<std::mutex> lock_pose(pose_mtx_);
    ins_q_w_v_ = q_W_V;
    ins_pose_ = enu;
  }

  // 通过ins位置拿到hq中最近的车道
  double x = vehicle_pose_.x();
  double y = vehicle_pose_.y();

  hozon::common::coordinate_convertor::GCS2UTM(51, &y, &x);
  hozon::common::PointENU enupos;
  enupos.set_x(y);
  enupos.set_y(x);
  enupos.set_z(0);

  double nearest_s = 0.;
  double nearest_l = 0.;
  hozon::hdmap::LaneInfoConstPtr lane_ptr = nullptr;

  if (hq_map_server_->Empty()) {
    HLOG_ERROR << "hqmap server load map failed";
    return;
  }

  int ret =
      hq_map_server_->GetNearestLane(enupos, &lane_ptr, &nearest_s, &nearest_s);
  if (ret != 0 || lane_ptr == nullptr) {
    HLOG_ERROR << "get nearest lane failed";
    return;
  }

  {
    std::lock_guard<std::mutex> lock_map(map_mtx_);
    location_left_id_.clear();
    location_right_id_.clear();
    hozon::hdmap::Lane nearest_lane = lane_ptr->lane();
    location_left_id_.emplace_back(nearest_lane.id());
    while (nearest_lane.left_neighbor_forward_lane_id_size() > 0) {
      auto id = nearest_lane.left_neighbor_forward_lane_id()[0];
      location_left_id_.emplace_back(id);
      auto left_lane = hq_map_server_->GetLaneById(id);
      if (!left_lane) {
        break;
      }
      nearest_lane.CopyFrom(left_lane->lane());
    }
    nearest_lane.CopyFrom(lane_ptr->lane());
    while (nearest_lane.right_neighbor_forward_lane_id_size() > 0) {
      auto id = nearest_lane.right_neighbor_forward_lane_id()[0];
      location_right_id_.emplace_back(id);
      auto right_lane = hq_map_server_->GetLaneById(id);
      if (!right_lane) {
        break;
      }
      nearest_lane.CopyFrom(right_lane->lane());
    }

    location_left_id_next_.clear();
    location_right_id_next_.clear();
    if (!lane_ptr->lane().successor_id().empty()) {
      auto successor_id_size = lane_ptr->lane().successor_id().size();
      auto lane_ptr_next = hq_map_server_->GetLaneById(
          lane_ptr->lane().successor_id(successor_id_size - 1));
      if (lane_ptr_next) {
        hozon::hdmap::Lane nearest_lane = lane_ptr_next->lane();
        location_left_id_next_.emplace_back(nearest_lane.id());
        while (nearest_lane.left_neighbor_forward_lane_id_size() > 0) {
          auto id = nearest_lane.left_neighbor_forward_lane_id()[0];
          location_left_id_next_.emplace_back(id);
          auto left_lane = hq_map_server_->GetLaneById(id);
          if (!left_lane) {
            break;
          }
          nearest_lane.CopyFrom(left_lane->lane());
        }
        nearest_lane.CopyFrom(lane_ptr_next->lane());
        while (nearest_lane.right_neighbor_forward_lane_id_size() > 0) {
          auto id = nearest_lane.right_neighbor_forward_lane_id()[0];
          location_right_id_next_.emplace_back(id);
          auto right_lane = hq_map_server_->GetLaneById(id);
          if (!right_lane) {
            break;
          }
          nearest_lane.CopyFrom(right_lane->lane());
        }
      }
    }
  }

  // HLOG_ERROR << "id sizexxxxxxxxxxxxxxx:" << location_left_id_.size() << ", "
  //            << location_right_id_.size();
}

void TopoAssignment::OnHQMap(const std::shared_ptr<hozon::hdmap::Map>& msg) {
  if (!hq_map_server_) {
    HLOG_ERROR << "!!! nullptr hq_map_server_";
  }
  int ret = hq_map_server_->LoadMapFromProto(*msg);
  if (ret != 0 || hq_map_server_->Empty()) {
    HLOG_ERROR << "hqmap server load map failed";
    return;
  }
  {
    std::lock_guard<std::mutex> lock_crop_map(crop_map_mtx_);
    crop_map_->CopyFrom(*msg);
  }
  // VizHQMap(msg);
}

void TopoAssignment::OnLocalMap(
    const std::shared_ptr<hozon::mapping::LocalMap>& msg) {
  if (msg->lanes().empty()) {
    return;
  }
  // 可视化crop map
  {
    std::lock_guard<std::mutex> lock_crop_map(crop_map_mtx_);
    std::vector<adsfi_proto::viz::MarkerArray> result =
        marker_rviz_.LaneToMarker(crop_map_, ref_point_, true);
    adsfi_proto::viz::MarkerArray lane = result[0];
    adsfi_proto::viz::MarkerArray left = result[1];
    adsfi_proto::viz::MarkerArray right = result[2];

    RVIZ_AGENT.Publish("lane", lane);
    RVIZ_AGENT.Publish("left", left);
    RVIZ_AGENT.Publish("right", right);
  }

  Eigen::Isometry3d T_W_V;
  Eigen::Isometry3d T_U_V;
  {
    std::lock_guard<std::mutex> lock_pose(pose_mtx_);
    T_U_V.setIdentity();
    Eigen::Matrix3d orient = ins_q_w_v_.toRotationMatrix();
    T_U_V.rotate(orient);
    T_U_V.pretranslate(ins_pose_);
  }

  {
    std::lock_guard<std::mutex> lock_location(location_mtx_);
    T_W_V.setIdentity();
    Eigen::Matrix3d orient = local_map_q_w_v_.toRotationMatrix();
    T_W_V.rotate(orient);
    T_W_V.pretranslate(local_map_pose_);
  }

  // 更新时间信息
  cur_timestamp_ = msg->header().publish_stamp();

  Eigen::Isometry3d T_U_W = T_U_V * T_W_V.inverse();

  // VizLocalMap(msg, T_U_W);

  // 更新all_lanelines
  {
    std::lock_guard<std::mutex> lock_map(map_mtx_);
    std::map<int32_t, LaneLine> all_lanelines;
    for (const auto& lane_line_it : msg->lanes()) {
      // 过滤空的点
      if (lane_line_it.points_size() <= 0) {
        continue;
      }
      if (location_left_id_.empty() && location_right_id_.empty()) {
        continue;
      }

      // if (all_lanelines_.find(lane_line_it.track_id()) !=
      //     all_lanelines_.end()) {
      //   LaneLine lane_line = all_lanelines_[lane_line_it.track_id()];
      //   all_lanelines[lane_line_it.track_id()] = lane_line;

      //   // 更新all_lanelines里面的left_lanes和right_lanes
      //   auto start_point = lane_line_it.points(0);
      //   auto point_size = lane_line_it.points().size();
      //   auto end_point = lane_line_it.points(point_size - 1);
      //   // 需要把local map的点转到enu下面
      //   Eigen::Vector3d point_local_start(start_point.x(), start_point.y(),
      //                                     start_point.z());
      //   Eigen::Vector3d point_local_end(end_point.x(), end_point.y(),
      //                                   end_point.z());
      //   Eigen::Vector3d point_enu_start = T_U_W * point_local_start;
      //   Eigen::Vector3d point_enu_end = T_U_W * point_local_end;
      //   Eigen::Vector2d p0(point_enu_start.x(), point_enu_start.y());
      //   Eigen::Vector2d p1(point_enu_end.x(), point_enu_end.y());

      //   // 更新left_lanes第一个点
      //   std::vector<std::string> left_lanes;
      //   left_lanes = all_lanelines_[lane_line_it.track_id()].left_lanes;
      //   for (int i = 0;
      //        i < all_lanelines_[lane_line_it.track_id()].left_lanes.size();
      //        ++i) {
      //     hozon::hdmap::Id lane_id;
      //     lane_id.set_id(all_lanelines_[lane_line_it.track_id()].left_lanes[i]);
      //     auto hq_lane = hq_map_server_->GetLaneById(lane_id);
      //     if (!hq_lane) {
      //       continue;
      //     }
      //     auto hq_lane_left_points =
      //         GetLaneStartAndEndPoint(hq_lane->lane(), true);
      //     if (!hq_lane_left_points.empty()) {
      //       auto point_size = hq_lane_left_points.size();
      //       if (PerpendicularFootInSegment(hq_lane_left_points[0],
      //                                      hq_lane_left_points[point_size -
      //                                      1], p0)) {
      //         break;
      //       } else {
      //         left_lanes.erase(left_lanes.begin() + i);
      //       }
      //     }
      //   }
      //   all_lanelines[lane_line_it.track_id()].left_lanes.clear();
      //   all_lanelines[lane_line_it.track_id()].left_lanes = left_lanes;

      //   // 更新left_lanes最后一个点
      //   if (!all_lanelines_[lane_line_it.track_id()].left_lanes.empty()) {
      //     auto size =
      //     all_lanelines_[lane_line_it.track_id()].left_lanes.size();

      //     hozon::hdmap::Id lane_id;
      //     lane_id.set_id(
      //         all_lanelines_[lane_line_it.track_id()].left_lanes[size - 1]);
      //     if (hq_map_server_->GetLaneById(lane_id)) {
      //       auto hq_lane = hq_map_server_->GetLaneById(lane_id)->lane();
      //       auto hq_lane_left_points = GetLaneStartAndEndPoint(hq_lane,
      //       true);

      //       while (!hq_lane_left_points.empty() &&
      //              !PerpendicularFootInSegment(hq_lane_left_points[0],
      //                                          hq_lane_left_points[1], p1)) {
      //         if (hq_lane.successor_id().empty()) {
      //           break;
      //         }
      //         auto lane =
      //         hq_map_server_->GetLaneById(hq_lane.successor_id(0)); if
      //         (!lane) {
      //           break;
      //         }
      //         hq_lane.CopyFrom(lane->lane());
      //         hq_lane_left_points = GetLaneStartAndEndPoint(hq_lane, true);
      //         all_lanelines[lane_line_it.track_id()].left_lanes.emplace_back(
      //             hq_lane.id().id());
      //       }
      //     }
      //   }

      //   // 更新right_lanes第一点
      //   std::vector<std::string> right_lanes;
      //   right_lanes = all_lanelines_[lane_line_it.track_id()].right_lanes;
      //   for (int i = 0;
      //        i < all_lanelines_[lane_line_it.track_id()].right_lanes.size();
      //        ++i) {
      //     hozon::hdmap::Id lane_id;
      //     lane_id.set_id(
      //         all_lanelines_[lane_line_it.track_id()].right_lanes[i]);
      //     auto hq_lane = hq_map_server_->GetLaneById(lane_id);
      //     if (!hq_lane) {
      //       continue;
      //     }
      //     auto hq_lane_right_points =
      //         GetLaneStartAndEndPoint(hq_lane->lane(), false);
      //     if (!hq_lane_right_points.empty()) {
      //       auto point_size = hq_lane_right_points.size();
      //       if (PerpendicularFootInSegment(hq_lane_right_points[0],
      //                                      hq_lane_right_points[point_size -
      //                                      1], p0)) {
      //         break;
      //       } else {
      //         right_lanes.erase(right_lanes.begin() + i);
      //       }
      //     }
      //   }

      //   all_lanelines[lane_line_it.track_id()].right_lanes.clear();
      //   all_lanelines[lane_line_it.track_id()].right_lanes = right_lanes;

      //   // 更新right_lanes最后一个点
      //   if (!all_lanelines_[lane_line_it.track_id()].right_lanes.empty()) {
      //     auto size =
      //         all_lanelines_[lane_line_it.track_id()].right_lanes.size();

      //     hozon::hdmap::Id lane_id;
      //     lane_id.set_id(
      //         all_lanelines_[lane_line_it.track_id()].right_lanes[size - 1]);
      //     if (hq_map_server_->GetLaneById(lane_id)) {
      //       auto hq_lane = hq_map_server_->GetLaneById(lane_id)->lane();
      //       auto hq_lane_right_points = GetLaneStartAndEndPoint(hq_lane,
      //       false);

      //       while (!hq_lane_right_points.empty() &&
      //              !PerpendicularFootInSegment(hq_lane_right_points[0],
      //                                          hq_lane_right_points[1], p1))
      //                                          {
      //         if (hq_lane.successor_id().empty()) {
      //           break;
      //         }
      //         auto lane =
      //         hq_map_server_->GetLaneById(hq_lane.successor_id(0)); if
      //         (!lane) {
      //           break;
      //         }
      //         hq_lane.CopyFrom(lane->lane());
      //         hq_lane_right_points = GetLaneStartAndEndPoint(hq_lane, false);
      //         all_lanelines[lane_line_it.track_id()].right_lanes.emplace_back(
      //             hq_lane.id().id());
      //       }
      //     }
      //   }

      //   // 更新points放在enu坐标系下面
      //   all_lanelines[lane_line_it.track_id()].points.clear();
      //   for (const auto& line_point_it : lane_line_it.points()) {
      //     Eigen::Vector3d point_local(line_point_it.x(), line_point_it.y(),
      //                                 line_point_it.z());
      //     Eigen::Vector3d point_enu = T_U_W * point_local;
      //     Point3D lane_line_point;
      //     lane_line_point.set_x(point_enu.x());
      //     lane_line_point.set_y(point_enu.y());
      //     lane_line_point.set_z(point_enu.z());
      //     all_lanelines[lane_line_it.track_id()].points.emplace_back(
      //         lane_line_point);
      //   }

      // } else {
      // 更新一个新的LaneLine
      LaneLine lane_line;
      // 更新id
      lane_line.id = lane_line_it.track_id();

      auto start_point = lane_line_it.points(0);
      auto point_size = lane_line_it.points().size();
      auto end_point = lane_line_it.points(point_size - 1);
      // 需要把local map的点转到enu下面
      Eigen::Vector3d point_local_start(start_point.x(), start_point.y(),
                                        start_point.z());
      Eigen::Vector3d point_local_end(end_point.x(), end_point.y(),
                                      end_point.z());
      Eigen::Vector3d point_enu_start = T_U_W * point_local_start;
      Eigen::Vector3d point_enu_end = T_U_W * point_local_end;
      Eigen::Vector2d p0(point_enu_start.x(), point_enu_start.y());
      Eigen::Vector2d p1(point_enu_end.x(), point_enu_end.y());

      // HLOG_ERROR << "location_left_id size:" << location_left_id_.size();
      // for (const auto& left : location_left_id_) {
      //   HLOG_ERROR << "left idxxxxxxxxxxxxxxx: " << left.id();
      // }
      // HLOG_ERROR << "location_right_id size:" << location_right_id_.size();
      // for (const auto& right : location_right_id_) {
      //   HLOG_ERROR << "right idxxxxxxxxxxxxxxx: " << right.id();
      // }
      // HLOG_ERROR << "lane line it lanepos:" << lane_line_it.lanepos();
      // HLOG_ERROR << "lane line it tack id:" << lane_line_it.track_id();

      switch (lane_line_it.lanepos()) {
        case hozon::mapping::LanePositionType::LanePositionType_BOLLARD_LEFT:
          if (location_left_id_.size() >= 5) {
            hozon::hdmap::Id lane_id = location_left_id_[4];
            AppendLaneLine(lane_id, &lane_line, p1, true);
          } else {
            if (location_left_id_next_.size() >= 5) {
              hozon::hdmap::Id lane_id = location_left_id_next_[4];
              AppendLaneLine(lane_id, &lane_line, p1, true);
            }
            if (location_left_id_next_.size() >= 6) {
              hozon::hdmap::Id lane_id = location_left_id_next_[5];
              AppendLaneLine(lane_id, &lane_line, p1, false);
            }
          }
          if (location_left_id_.size() >= 6) {
            hozon::hdmap::Id lane_id = location_left_id_[5];
            AppendLaneLine(lane_id, &lane_line, p1, false);
          }
          break;
        case hozon::mapping::LanePositionType::LanePositionType_FOURTH_LEFT:
          if (location_left_id_.size() >= 4) {
            hozon::hdmap::Id lane_id = location_left_id_[3];
            AppendLaneLine(lane_id, &lane_line, p1, true);
          } else {
            if (location_left_id_next_.size() >= 4) {
              hozon::hdmap::Id lane_id = location_left_id_next_[3];
              AppendLaneLine(lane_id, &lane_line, p1, true);
            }
            if (location_left_id_next_.size() >= 5) {
              hozon::hdmap::Id lane_id = location_left_id_next_[4];
              AppendLaneLine(lane_id, &lane_line, p1, false);
            }
          }
          if (location_left_id_.size() >= 5) {
            hozon::hdmap::Id lane_id = location_left_id_[4];
            AppendLaneLine(lane_id, &lane_line, p1, false);
          }
          break;
        case hozon::mapping::LanePositionType::LanePositionType_THIRD_LEFT:
          if (location_left_id_.size() >= 3) {
            hozon::hdmap::Id lane_id = location_left_id_[2];
            AppendLaneLine(lane_id, &lane_line, p1, true);
          } else {
            if (location_left_id_next_.size() >= 3) {
              hozon::hdmap::Id lane_id = location_left_id_next_[2];
              AppendLaneLine(lane_id, &lane_line, p1, true);
            }
            if (location_left_id_next_.size() >= 4) {
              hozon::hdmap::Id lane_id = location_left_id_next_[3];
              AppendLaneLine(lane_id, &lane_line, p1, false);
            }
          }
          if (location_left_id_.size() >= 4) {
            hozon::hdmap::Id lane_id = location_left_id_[3];
            AppendLaneLine(lane_id, &lane_line, p1, false);
          }
          break;
        case hozon::mapping::LanePositionType::LanePositionType_ADJACENT_LEFT:
          if (location_left_id_.size() >= 2) {
            hozon::hdmap::Id lane_id = location_left_id_[1];
            AppendLaneLine(lane_id, &lane_line, p1, true);
          } else {
            if (location_left_id_next_.size() >= 2) {
              hozon::hdmap::Id lane_id = location_left_id_next_[1];
              AppendLaneLine(lane_id, &lane_line, p1, true);
            }
            if (location_left_id_next_.size() >= 3) {
              hozon::hdmap::Id lane_id = location_left_id_next_[2];
              AppendLaneLine(lane_id, &lane_line, p1, false);
            }
          }
          if (location_left_id_.size() >= 3) {
            hozon::hdmap::Id lane_id = location_left_id_[2];
            AppendLaneLine(lane_id, &lane_line, p1, false);
          }
          break;
        case hozon::mapping::LanePositionType::LanePositionType_EGO_LEFT:
          if (location_left_id_.size() >= 1) {
            hozon::hdmap::Id lane_id = location_left_id_[0];
            AppendLaneLine(lane_id, &lane_line, p1, true);
          } else {
            if (location_left_id_next_.size() >= 1) {
              hozon::hdmap::Id lane_id = location_left_id_next_[0];
              AppendLaneLine(lane_id, &lane_line, p1, true);
            }
            if (location_left_id_next_.size() >= 2) {
              hozon::hdmap::Id lane_id = location_left_id_next_[1];
              AppendLaneLine(lane_id, &lane_line, p1, false);
            }
          }
          if (location_left_id_.size() >= 2) {
            hozon::hdmap::Id lane_id = location_left_id_[1];
            AppendLaneLine(lane_id, &lane_line, p1, false);
          }
          break;
        case hozon::mapping::LanePositionType::LanePositionType_EGO_RIGHT:
          if (location_left_id_.size() >= 1) {
            hozon::hdmap::Id lane_id = location_left_id_[0];
            AppendLaneLine(lane_id, &lane_line, p1, false);
          } else {
            if (location_left_id_next_.size() >= 1) {
              hozon::hdmap::Id lane_id = location_left_id_next_[0];
              AppendLaneLine(lane_id, &lane_line, p1, false);
            }
            if (location_right_id_next_.size() >= 1) {
              hozon::hdmap::Id lane_id = location_right_id_next_[0];
              AppendLaneLine(lane_id, &lane_line, p1, true);
            }
          }
          if (location_right_id_.size() >= 1) {
            hozon::hdmap::Id lane_id = location_right_id_[0];
            AppendLaneLine(lane_id, &lane_line, p1, true);
          }
          break;
        case hozon::mapping::LanePositionType::LanePositionType_ADJACENT_RIGHT:
          if (location_right_id_.size() >= 1) {
            hozon::hdmap::Id lane_id = location_right_id_[0];
            AppendLaneLine(lane_id, &lane_line, p1, false);
          } else {
            if (location_right_id_next_.size() >= 1) {
              hozon::hdmap::Id lane_id = location_right_id_next_[0];
              AppendLaneLine(lane_id, &lane_line, p1, false);
            }
            if (location_right_id_next_.size() >= 2) {
              hozon::hdmap::Id lane_id = location_right_id_next_[1];
              AppendLaneLine(lane_id, &lane_line, p1, true);
            }
          }
          if (location_right_id_.size() >= 2) {
            hozon::hdmap::Id lane_id = location_right_id_[1];
            AppendLaneLine(lane_id, &lane_line, p1, true);
          }
          break;
        case hozon::mapping::LanePositionType::LanePositionType_THIRD_RIGHT:
          if (location_right_id_.size() >= 2) {
            hozon::hdmap::Id lane_id = location_right_id_[1];
            AppendLaneLine(lane_id, &lane_line, p1, false);
          } else {
            if (location_right_id_next_.size() >= 2) {
              hozon::hdmap::Id lane_id = location_right_id_next_[1];
              AppendLaneLine(lane_id, &lane_line, p1, false);
            }
            if (location_right_id_next_.size() >= 3) {
              hozon::hdmap::Id lane_id = location_right_id_next_[2];
              AppendLaneLine(lane_id, &lane_line, p1, true);
            }
          }
          if (location_right_id_.size() >= 3) {
            hozon::hdmap::Id lane_id = location_right_id_[2];
            AppendLaneLine(lane_id, &lane_line, p1, true);
          }
          break;
        case hozon::mapping::LanePositionType::LanePositionType_FOURTH_RIGHT:
          if (location_right_id_.size() >= 3) {
            hozon::hdmap::Id lane_id = location_right_id_[2];
            AppendLaneLine(lane_id, &lane_line, p1, false);
          } else {
            if (location_right_id_next_.size() >= 3) {
              hozon::hdmap::Id lane_id = location_right_id_next_[2];
              AppendLaneLine(lane_id, &lane_line, p1, false);
            }
            if (location_right_id_next_.size() >= 4) {
              hozon::hdmap::Id lane_id = location_right_id_next_[3];
              AppendLaneLine(lane_id, &lane_line, p1, true);
            }
          }
          if (location_right_id_.size() >= 4) {
            hozon::hdmap::Id lane_id = location_right_id_[3];
            AppendLaneLine(lane_id, &lane_line, p1, true);
          }
          break;
        case hozon::mapping::LanePositionType::LanePositionType_BOLLARD_RIGHT:
          if (location_right_id_.size() >= 4) {
            hozon::hdmap::Id lane_id = location_right_id_[3];
            AppendLaneLine(lane_id, &lane_line, p1, false);
          } else {
            if (location_right_id_next_.size() >= 4) {
              hozon::hdmap::Id lane_id = location_right_id_next_[3];
              AppendLaneLine(lane_id, &lane_line, p1, false);
            }
            if (location_right_id_next_.size() >= 5) {
              hozon::hdmap::Id lane_id = location_right_id_next_[4];
              AppendLaneLine(lane_id, &lane_line, p1, true);
            }
          }
          if (location_right_id_.size() >= 5) {
            hozon::hdmap::Id lane_id = location_right_id_[4];
            AppendLaneLine(lane_id, &lane_line, p1, true);
          }
          break;

        default:
          break;
      }

      // 更新points全部转到enu坐标系下面
      for (const auto& line_point_it : lane_line_it.points()) {
        Eigen::Vector3d point_local(line_point_it.x(), line_point_it.y(),
                                    line_point_it.z());
        Eigen::Vector3d point_enu = T_U_W * point_local;
        hozon::common::Point3D lane_line_point;
        lane_line_point.set_x(point_enu.x());
        lane_line_point.set_y(point_enu.y());
        lane_line_point.set_z(point_enu.z());
        lane_line.points.emplace_back(lane_line_point);
      }
      all_lanelines[lane_line_it.track_id()] = lane_line;
      //}
    }

    all_lanelines_.clear();
    all_lanelines_ = all_lanelines;

    // // debug信息看看all_lanelines
    // for (const auto& laneline : all_lanelines_) {
    //   HLOG_ERROR << "idxxxxxxxxxxxxxxx: " << laneline.second.id;
    //   HLOG_ERROR << "point size xxxxxxxxxxxxxxx: "
    //              << laneline.second.points.size();
    //   for (const auto& left : laneline.second.left_lanes) {
    //     HLOG_ERROR << "left idxxxxxxxxxxxxxxx: " << left << ", "
    //                << laneline.second.id;
    //   }
    //   HLOG_ERROR << "zzzzzzzzzzzzzzzzz";
    //   for (const auto& right : laneline.second.right_lanes) {
    //     HLOG_ERROR << "right idxxxxxxxxxxxxxxx: " << right << ", "
    //                << laneline.second.id;
    //   }
    // }

    // HLOG_ERROR << "YYYYYYYYYYYYYYYYYYYYYYYYY";

    // 从all_lanelines构造all_lanes
    std::map<std::string, Lane> all_lanes;
    AppendLane(all_lanelines_, &all_lanes);

    // // debug信息看看all_lanelines
    // for (const auto& lane : all_lanes) {
    //   HLOG_ERROR << "idxxxxxxxxxxxxxxx: " << lane.second.id;
    //   for (const auto& left : lane.second.left_lines) {
    //     HLOG_ERROR << "left idxxxxxxxxxxxxxxx: " << left << ", "
    //                << lane.second.id;
    //   }
    //   HLOG_ERROR << "zzzzzzzzzzzzzzzzz";
    //   for (const auto& right : lane.second.right_lines) {
    //     HLOG_ERROR << "right idxxxxxxxxxxxxxxx: " << right << ", "
    //                << lane.second.id;
    //   }
    //   for (const auto& left : lane.second.left_lanes) {
    //     HLOG_ERROR << "left_lanes idxxxxxxxxxxxxxxx: " << left << ", "
    //                << lane.second.id;
    //   }
    //   for (const auto& right : lane.second.right_lanes) {
    //     HLOG_ERROR << "right_lanes idxxxxxxxxxxxxxxx: " << right << ", "
    //                << lane.second.id;
    //   }
    //   for (const auto& prev : lane.second.prev_lanes) {
    //     HLOG_ERROR << "prev_lanes idxxxxxxxxxxxxxxx: " << prev << ", "
    //                << lane.second.id;
    //   }
    //   for (const auto& next : lane.second.next_lanes) {
    //     HLOG_ERROR << "next_lanes idxxxxxxxxxxxxxxx: " << next << ", "
    //                << lane.second.id;
    //   }
    // }

    // HLOG_ERROR << "YYYYYYYYYYYYYYYYYYYYYYYYY";

    // all_lanes构造topo map

    topo_map_ = std::make_shared<hozon::hdmap::Map>();
    AppendTopoMap(all_lanes, topo_map_);
  }

  VizTopoMap(topo_map_);
  // VizLocalMap(msg);
}

void TopoAssignment::OnLocalMapLocation(
    const std::shared_ptr<hozon::localization::Localization>& msg) {
  std::lock_guard<std::mutex> lock_location(location_mtx_);
  local_map_pose_ << msg->pose().position().x(), msg->pose().position().y(),
      msg->pose().position().z();
  local_map_q_w_v_.w() = msg->pose().quaternion().w();
  local_map_q_w_v_.x() = msg->pose().quaternion().x();
  local_map_q_w_v_.y() = msg->pose().quaternion().y();
  local_map_q_w_v_.z() = msg->pose().quaternion().z();
}

std::shared_ptr<hozon::hdmap::Map> TopoAssignment::GetTopoMap() {
  std::lock_guard<std::mutex> lock_map(map_mtx_);
  return topo_map_;
}

void TopoAssignment::AppendLaneLine(const hozon::hdmap::Id& lane_id,
                                    hozon::mp::mf::LaneLine* lane_line,
                                    const Eigen::Vector2d& p1,
                                    const bool left) {
  auto hq_lane_ptr = hq_map_server_->GetLaneById(lane_id);
  if (hq_lane_ptr) {
    auto hq_lane = hq_lane_ptr->lane();
    auto hq_lane_left_points = GetLaneStartAndEndPoint(hq_lane, true);
    if (left) {
      lane_line->left_lanes.emplace_back(hq_lane.id().id());
    } else {
      lane_line->right_lanes.emplace_back(hq_lane.id().id());
    }

    while (!hq_lane_left_points.empty() &&
           !PerpendicularFootInSegment(hq_lane_left_points[0],
                                       hq_lane_left_points[1], p1)) {
      if (hq_lane.successor_id().empty()) {
        break;
      }
      auto successor_id_size = hq_lane.successor_id().size();
      auto lane = hq_map_server_->GetLaneById(
          hq_lane.successor_id(successor_id_size - 1));
      if (!lane) {
        break;
      }
      hq_lane.CopyFrom(lane->lane());
      hq_lane_left_points = GetLaneStartAndEndPoint(hq_lane, true);
      if (left) {
        lane_line->left_lanes.emplace_back(hq_lane.id().id());
      } else {
        lane_line->right_lanes.emplace_back(hq_lane.id().id());
      }
    }
  }
}

void TopoAssignment::AppendLane(
    const std::map<int32_t, LaneLine>& all_lanelines,
    std::map<std::string, Lane>* all_lanes) {
  for (const auto& line_it : all_lanelines) {
    for (const auto& left_lane_it : line_it.second.left_lanes) {
      if (all_lanes->find(left_lane_it) == all_lanes->end()) {
        Lane lane;
        lane.id = left_lane_it;
        lane.left_lines.emplace_back(line_it.second.id);
        (*all_lanes)[left_lane_it] = lane;
      } else {
        (*all_lanes)[left_lane_it].left_lines.emplace_back(line_it.second.id);
      }
    }
    for (const auto& right_lane_it : line_it.second.right_lanes) {
      if (all_lanes->find(right_lane_it) == all_lanes->end()) {
        Lane lane;
        lane.id = right_lane_it;
        lane.right_lines.emplace_back(line_it.second.id);
        (*all_lanes)[right_lane_it] = lane;
      } else {
        (*all_lanes)[right_lane_it].right_lines.emplace_back(line_it.second.id);
      }
    }
  }

  // 再构造前后左右
  for (auto& lane_it : (*all_lanes)) {
    hozon::hdmap::Id lane_id;
    lane_id.set_id(lane_it.second.id);
    auto hq_lane = hq_map_server_->GetLaneById(lane_id);
    if (!hq_lane) {
      continue;
    }
    for (const auto& left_lane_it :
         hq_lane->lane().left_neighbor_forward_lane_id()) {
      auto left_lane_it_id = left_lane_it.id();
      if (all_lanes->find(left_lane_it_id) != all_lanes->end()) {
        lane_it.second.left_lanes.emplace_back(left_lane_it_id);
      }
    }
    for (const auto& right_lane_it :
         hq_lane->lane().right_neighbor_forward_lane_id()) {
      auto right_lane_it_id = right_lane_it.id();
      if (all_lanes->find(right_lane_it_id) != all_lanes->end()) {
        lane_it.second.right_lanes.emplace_back(right_lane_it_id);
      }
    }
    for (const auto& prev_lane_it : hq_lane->lane().predecessor_id()) {
      auto prev_lane_it_id = prev_lane_it.id();
      if (all_lanes->find(prev_lane_it_id) != all_lanes->end()) {
        lane_it.second.prev_lanes.emplace_back(prev_lane_it_id);
      }
    }
    for (const auto& next_lane_it : hq_lane->lane().successor_id()) {
      auto next_lane_it_id = next_lane_it.id();
      if (all_lanes->find(next_lane_it_id) != all_lanes->end()) {
        lane_it.second.next_lanes.emplace_back(next_lane_it_id);
      }
    }
  }
}

void TopoAssignment::AppendTopoMap(
    const std::map<std::string, Lane>& all_lanes,
    const std::shared_ptr<hozon::hdmap::Map>& topo_map) {
  for (const auto& lane_it : all_lanes) {
    // 通过lane id 拿到首尾两个点
    hozon::hdmap::Id lane_id;
    lane_id.set_id(lane_it.second.id);

    auto hq_lane = hq_map_server_->GetLaneById(lane_id);
    // 这边需要判断hq_lane是不是空指针后面加上
    if (!hq_lane) {
      continue;
    }

    // 后续需要该一下改成map里面有点再加入
    auto lane = topo_map->add_lane();
    lane->mutable_id()->set_id(lane_it.second.id);
    for (const auto& left_lane_it : lane_it.second.left_lanes) {
      lane->add_left_neighbor_forward_lane_id()->set_id(left_lane_it);
    }
    for (const auto& right_lane_it : lane_it.second.right_lanes) {
      lane->add_right_neighbor_forward_lane_id()->set_id(right_lane_it);
    }
    for (const auto& prev_lane_it : lane_it.second.prev_lanes) {
      lane->add_predecessor_id()->set_id(prev_lane_it);
    }
    for (const auto& next_lane_it : lane_it.second.next_lanes) {
      lane->add_successor_id()->set_id(next_lane_it);
    }
    // 更新topo map的左右车道线

    auto hq_lane_left_points = GetLaneStartAndEndPoint(hq_lane->lane(), true);
    auto hq_lane_right_points = GetLaneStartAndEndPoint(hq_lane->lane(), false);
    auto hq_lane_left_points_size = hq_lane_left_points.size();
    auto hq_lane_right_points_size = hq_lane_right_points.size();

    for (const auto& left_line_it : lane_it.second.left_lines) {
      // 打断给到lane_it 里面去
      bool flag_p0 = false;
      bool flag_p1 = false;
      bool flag_pt = false;
      if (all_lanelines_.find(left_line_it) != all_lanelines_.end()) {
        if (!all_lanelines_[left_line_it].points.empty()) {
          auto start_point = all_lanelines_[left_line_it].points[0];
          auto point_size = all_lanelines_[left_line_it].points.size();
          auto end_point = all_lanelines_[left_line_it].points[point_size - 1];

          Eigen::Vector2d p0(start_point.x(), start_point.y());
          Eigen::Vector2d p1(end_point.x(), end_point.y());

          if (!hq_lane_left_points.empty()) {
            flag_p0 = PerpendicularFootInSegment(
                hq_lane_left_points[0],
                hq_lane_left_points[hq_lane_left_points_size - 1], p0);
            flag_p1 = PerpendicularFootInSegment(
                hq_lane_left_points[0],
                hq_lane_left_points[hq_lane_left_points_size - 1], p1);
            flag_pt =
                PerpendicularFootInSegment(p0, p1, hq_lane_left_points[0]);
          }
          if (flag_p0 && flag_p1) {
            // 不用找index
            hozon::hdmap::LaneBoundary lane_boundary;
            auto segment = lane_boundary.mutable_curve()->add_segment();
            for (const auto& point : all_lanelines_[left_line_it].points) {
              auto topo_point = segment->mutable_line_segment()->add_point();
              topo_point->set_x(point.x());
              topo_point->set_y(point.y());
              topo_point->set_z(point.z());
            }
            lane->mutable_left_boundary()->CopyFrom(lane_boundary);
          }
          if (flag_p0 && !flag_p1) {
            // 找到后面一个点最近的index
            auto index = FindNearestPointIndex(
                hq_lane_left_points[hq_lane_left_points_size - 1],
                all_lanelines_[left_line_it].points);
            hozon::hdmap::LaneBoundary lane_boundary;
            auto segment = lane_boundary.mutable_curve()->add_segment();
            for (int i = 0; i <= index; ++i) {
              auto topo_point = segment->mutable_line_segment()->add_point();
              topo_point->set_x(all_lanelines_[left_line_it].points[i].x());
              topo_point->set_y(all_lanelines_[left_line_it].points[i].y());
              topo_point->set_z(all_lanelines_[left_line_it].points[i].z());
            }
            lane->mutable_left_boundary()->CopyFrom(lane_boundary);
          }
          if (!flag_p0 && flag_p1) {
            // 找到前面一个点最近的index
            auto index = FindNearestPointIndex(
                hq_lane_left_points[0], all_lanelines_[left_line_it].points);
            hozon::hdmap::LaneBoundary lane_boundary;
            auto segment = lane_boundary.mutable_curve()->add_segment();
            for (int i = index; i < point_size; ++i) {
              auto topo_point = segment->mutable_line_segment()->add_point();
              topo_point->set_x(all_lanelines_[left_line_it].points[i].x());
              topo_point->set_y(all_lanelines_[left_line_it].points[i].y());
              topo_point->set_z(all_lanelines_[left_line_it].points[i].z());
            }
            lane->mutable_left_boundary()->CopyFrom(lane_boundary);
          }
          if (!flag_p0 && !flag_p1 && flag_pt) {
            // 找到两个点最近的index
            auto index_start = FindNearestPointIndex(
                hq_lane_left_points[0], all_lanelines_[left_line_it].points);
            auto index_end = FindNearestPointIndex(
                hq_lane_left_points[hq_lane_left_points_size - 1],
                all_lanelines_[left_line_it].points);
            hozon::hdmap::LaneBoundary lane_boundary;
            auto segment = lane_boundary.mutable_curve()->add_segment();
            for (int i = index_start; i <= index_end; ++i) {
              auto topo_point = segment->mutable_line_segment()->add_point();
              topo_point->set_x(all_lanelines_[left_line_it].points[i].x());
              topo_point->set_y(all_lanelines_[left_line_it].points[i].y());
              topo_point->set_z(all_lanelines_[left_line_it].points[i].z());
            }
            lane->mutable_left_boundary()->CopyFrom(lane_boundary);
          }
        }
      }
    }
    for (const auto& right_line_it : lane_it.second.right_lines) {
      // 打断给到lant_it 里面去
      bool flag_p0 = false;
      bool flag_p1 = false;
      bool flag_pt = false;
      if (all_lanelines_.find(right_line_it) != all_lanelines_.end()) {
        if (!all_lanelines_[right_line_it].points.empty()) {
          auto start_point = all_lanelines_[right_line_it].points[0];
          auto point_size = all_lanelines_[right_line_it].points.size();
          auto end_point = all_lanelines_[right_line_it].points[point_size - 1];

          Eigen::Vector2d p0(start_point.x(), start_point.y());
          Eigen::Vector2d p1(end_point.x(), end_point.y());

          if (!hq_lane_right_points.empty()) {
            auto point_size = hq_lane_right_points.size();
            flag_p0 = PerpendicularFootInSegment(
                hq_lane_right_points[0],
                hq_lane_right_points[hq_lane_right_points_size - 1], p0);
            flag_p1 = PerpendicularFootInSegment(
                hq_lane_right_points[0],
                hq_lane_right_points[hq_lane_right_points_size - 1], p1);
            flag_pt =
                PerpendicularFootInSegment(p0, p1, hq_lane_right_points[0]);
          }
          if (flag_p0 && flag_p1) {
            // 不用找index
            hozon::hdmap::LaneBoundary lane_boundary;
            auto segment = lane_boundary.mutable_curve()->add_segment();
            for (const auto& point : all_lanelines_[right_line_it].points) {
              auto topo_point = segment->mutable_line_segment()->add_point();
              topo_point->set_x(point.x());
              topo_point->set_y(point.y());
              topo_point->set_z(point.z());
            }
            lane->mutable_right_boundary()->CopyFrom(lane_boundary);
          }
          if (flag_p0 && !flag_p1) {
            // 找到后面一个点最近的index
            auto index = FindNearestPointIndex(
                hq_lane_right_points[hq_lane_right_points_size - 1],
                all_lanelines_[right_line_it].points);
            hozon::hdmap::LaneBoundary lane_boundary;
            auto segment = lane_boundary.mutable_curve()->add_segment();
            for (int i = 0; i <= index; ++i) {
              auto topo_point = segment->mutable_line_segment()->add_point();
              topo_point->set_x(all_lanelines_[right_line_it].points[i].x());
              topo_point->set_y(all_lanelines_[right_line_it].points[i].y());
              topo_point->set_z(all_lanelines_[right_line_it].points[i].z());
            }
            lane->mutable_right_boundary()->CopyFrom(lane_boundary);
          }
          if (!flag_p0 && flag_p1) {
            // 找到前面一个点最近的index
            auto index = FindNearestPointIndex(
                hq_lane_right_points[0], all_lanelines_[right_line_it].points);
            hozon::hdmap::LaneBoundary lane_boundary;
            auto segment = lane_boundary.mutable_curve()->add_segment();
            for (int i = index; i < point_size; ++i) {
              auto topo_point = segment->mutable_line_segment()->add_point();
              topo_point->set_x(all_lanelines_[right_line_it].points[i].x());
              topo_point->set_y(all_lanelines_[right_line_it].points[i].y());
              topo_point->set_z(all_lanelines_[right_line_it].points[i].z());
            }
            lane->mutable_right_boundary()->CopyFrom(lane_boundary);
          }
          if (!flag_p0 && !flag_p1 && flag_pt) {
            // 找到两个点最近的index
            auto index_start = FindNearestPointIndex(
                hq_lane_right_points[0], all_lanelines_[right_line_it].points);
            auto index_end = FindNearestPointIndex(
                hq_lane_right_points[hq_lane_right_points_size - 1],
                all_lanelines_[right_line_it].points);
            hozon::hdmap::LaneBoundary lane_boundary;
            auto segment = lane_boundary.mutable_curve()->add_segment();
            for (int i = index_start; i <= index_end; ++i) {
              auto topo_point = segment->mutable_line_segment()->add_point();
              topo_point->set_x(all_lanelines_[right_line_it].points[i].x());
              topo_point->set_y(all_lanelines_[right_line_it].points[i].y());
              topo_point->set_z(all_lanelines_[right_line_it].points[i].z());
            }
            lane->mutable_right_boundary()->CopyFrom(lane_boundary);
          }
        }
      }
    }
  }
}

void TopoAssignment::VizLocalMap(
    const std::shared_ptr<hozon::mapping::LocalMap>& local_map,
    const Eigen::Isometry3d& T_U_W) {
  if (!RVIZ_AGENT.Ok()) {
    return;
  }
  adsfi_proto::viz::MarkerArray markers;
  for (const auto& i : local_map->lanes()) {
    std::vector<Eigen::Vector3d> lane_points;
    for (const auto& point : i.points()) {
      Eigen::Vector3d point_local(point.x(), point.y(), point.z());
      Eigen::Vector3d point_enu = T_U_W * point_local;
      lane_points.emplace_back(point_enu);
    }
    adsfi_proto::viz::Marker marker;
    PointsToMarker(local_map->header().publish_stamp(), lane_points, &marker,
                   0.1);
    if (!marker.points().empty()) {
      markers.add_markers()->CopyFrom(marker);
    }
    if (!lane_points.empty()) {
      adsfi_proto::viz::Marker marker_id;
      auto id = std::to_string(i.lanepos());
      LineIdToMarker(local_map->header().publish_stamp(), lane_points[0], id,
                     &marker_id);
      markers.add_markers()->CopyFrom(marker_id);
    }
  }

  RVIZ_AGENT.Publish(KTopicTopoAsignLocalMap, markers);
}

void TopoAssignment::VizLocation(const Eigen::Vector3d& pose,
                                 const Eigen::Quaterniond& q_W_V,
                                 const double stamp) {
  if (RVIZ_AGENT.Ok()) {
    static uint32_t seq = 0;
    int curr_seq = seq++;
    adsfi_proto::viz::TransformStamped geo_tf;
    geo_tf.mutable_header()->set_seq(curr_seq);
    geo_tf.mutable_header()->mutable_timestamp()->set_sec(stamp);
    geo_tf.mutable_header()->set_frameid("map");
    geo_tf.set_child_frame_id("vehicle");
    geo_tf.mutable_transform()->mutable_translation()->set_x(pose.x());
    geo_tf.mutable_transform()->mutable_translation()->set_y(pose.y());
    geo_tf.mutable_transform()->mutable_translation()->set_z(pose.z());
    geo_tf.mutable_transform()->mutable_rotation()->set_x(q_W_V.x());
    geo_tf.mutable_transform()->mutable_rotation()->set_y(q_W_V.y());
    geo_tf.mutable_transform()->mutable_rotation()->set_z(q_W_V.z());
    geo_tf.mutable_transform()->mutable_rotation()->set_w(q_W_V.w());
    RVIZ_AGENT.Publish(kTopicTopoAsignTf, geo_tf);

    auto* location_pose = location_path_.add_poses();

    location_path_.mutable_header()->set_seq(curr_seq);
    location_path_.mutable_header()->mutable_timestamp()->set_sec(stamp);
    location_path_.mutable_header()->set_frameid("map");

    location_pose->mutable_header()->set_seq(curr_seq);
    location_pose->mutable_header()->mutable_timestamp()->set_sec(stamp);
    location_pose->mutable_header()->set_frameid("map");
    location_pose->mutable_pose()->mutable_position()->set_x(pose.x());
    location_pose->mutable_pose()->mutable_position()->set_y(pose.y());
    location_pose->mutable_pose()->mutable_position()->set_z(pose.z());

    location_pose->mutable_pose()->mutable_orientation()->set_w(q_W_V.w());
    location_pose->mutable_pose()->mutable_orientation()->set_x(q_W_V.x());
    location_pose->mutable_pose()->mutable_orientation()->set_y(q_W_V.y());
    location_pose->mutable_pose()->mutable_orientation()->set_z(q_W_V.z());

    if (location_path_.poses().size() > 200) {
      location_path_.mutable_poses()->DeleteSubrange(0, 1);
    }
    RVIZ_AGENT.Publish(kTopicTopoAsignLocation, location_path_);
  }
}

void TopoAssignment::VizHQMap(const std::shared_ptr<hozon::hdmap::Map>& msg) {
  if (!RVIZ_AGENT.Ok()) {
    return;
  }
  adsfi_proto::viz::MarkerArray markers_road;

  for (const auto& hq_road : msg->road()) {
    for (const auto& hq_road_section : hq_road.section()) {
      for (const auto& hq_road_section_edge :
           hq_road_section.boundary().outer_polygon().edge()) {
        for (const auto& edge : hq_road_section_edge.curve().segment()) {
          std::vector<Eigen::Vector3d> boundary_points;
          for (const auto& point : edge.line_segment().point()) {
            // 这里点的坐标是在utm坐标系下，需要将其转到enu坐标系下
            Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
            int zone = 51;
            double x = point_utm.x();
            double y = point_utm.y();
            auto ret =
                hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
            if (ret) {
              Eigen::Vector3d point_gcj(y, x, 0);
              Eigen::Vector3d point_enu =
                  util::Geo::Gcj02ToEnu(point_gcj, ref_point_);
              boundary_points.emplace_back(point_enu);
            } else {
              HLOG_ERROR << "=== UTM2GCS failed";
            }
          }
          adsfi_proto::viz::Marker marker;
          PointsToMarker(cur_timestamp_, boundary_points, &marker, 0.9);
          if (!marker.points().empty()) {
            markers_road.add_markers()->CopyFrom(marker);
          }
        }
      }
    }
  }

  RVIZ_AGENT.Publish(KTopicTopoAsignHQMapRoad, markers_road);

  adsfi_proto::viz::MarkerArray markers_lane;

  for (const auto& hq_lane : msg->lane()) {
    for (const auto& left_points : hq_lane.left_boundary().curve().segment()) {
      std::vector<Eigen::Vector3d> lane_points;
      for (const auto& point : left_points.line_segment().point()) {
        // 这里点的坐标是在utm坐标系下，需要将其转到enu坐标系下
        Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
        int zone = 51;
        double x = point_utm.x();
        double y = point_utm.y();
        auto ret = hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
        if (ret) {
          Eigen::Vector3d point_gcj(y, x, 0);
          Eigen::Vector3d point_enu =
              util::Geo::Gcj02ToEnu(point_gcj, ref_point_);
          lane_points.emplace_back(point_enu);
        } else {
          HLOG_ERROR << "=== UTM2GCS failed";
        }
      }
      adsfi_proto::viz::Marker marker;
      PointsToMarker(cur_timestamp_, lane_points, &marker, 0.5);
      markers_lane.add_markers()->CopyFrom(marker);
    }

    for (const auto& right_points :
         hq_lane.right_boundary().curve().segment()) {
      std::vector<Eigen::Vector3d> lane_points;
      for (const auto& point : right_points.line_segment().point()) {
        // 这里点的坐标是在utm坐标系下，需要将其转到enu坐标系下
        Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
        int zone = 51;
        double x = point_utm.x();
        double y = point_utm.y();
        auto ret = hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
        if (ret) {
          Eigen::Vector3d point_gcj(y, x, 0);
          Eigen::Vector3d point_enu =
              util::Geo::Gcj02ToEnu(point_gcj, ref_point_);
          lane_points.emplace_back(point_enu);
        } else {
          HLOG_ERROR << "=== UTM2GCS failed";
        }
      }
      adsfi_proto::viz::Marker marker;
      PointsToMarker(cur_timestamp_, lane_points, &marker, 0.5);
      markers_lane.add_markers()->CopyFrom(marker);
    }
    if (!hq_lane.central_curve().segment().empty()) {
      auto point =
          hq_lane.central_curve().segment()[0].line_segment().point()[0];
      Eigen::Vector3d point_central(point.x(), point.y(), point.z());
      int zone = 51;
      double x = point_central.x();
      double y = point_central.y();
      auto ret = hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
      Eigen::Vector3d point_enu;
      if (ret) {
        Eigen::Vector3d point_gcj(y, x, 0);
        point_enu = util::Geo::Gcj02ToEnu(point_gcj, ref_point_);
      } else {
        HLOG_ERROR << "=== UTM2GCS failed";
      }
      adsfi_proto::viz::Marker marker_id;
      auto id = hq_lane.id().id();
      LineIdToMarker(cur_timestamp_, point_enu, id, &marker_id);
      markers_lane.add_markers()->CopyFrom(marker_id);
    }
  }

  RVIZ_AGENT.Publish(KTopicTopoAsignHQMapLane, markers_lane);
}

void TopoAssignment::VizTopoMap(const std::shared_ptr<hozon::hdmap::Map>& msg) {
  if (!RVIZ_AGENT.Ok()) {
    return;
  }
  adsfi_proto::viz::MarkerArray markers_road;

  for (const auto& hq_road : msg->road()) {
    for (const auto& hq_road_section : hq_road.section()) {
      for (const auto& hq_road_section_edge :
           hq_road_section.boundary().outer_polygon().edge()) {
        for (const auto& edge : hq_road_section_edge.curve().segment()) {
          std::vector<Eigen::Vector3d> boundary_points;
          for (const auto& point : edge.line_segment().point()) {
            Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
            boundary_points.emplace_back(point_utm);
          }
          adsfi_proto::viz::Marker marker;
          PointsToMarker(cur_timestamp_, boundary_points, &marker, 0.2);
          if (!marker.points().empty()) {
            markers_road.add_markers()->CopyFrom(marker);
          }
        }
      }
    }
  }

  RVIZ_AGENT.Publish(KTopicTopoAsignTopoMapRoad, markers_road);

  adsfi_proto::viz::MarkerArray markers_lane;

  for (const auto& hq_lane : msg->lane()) {
    for (const auto& left_points : hq_lane.left_boundary().curve().segment()) {
      std::vector<Eigen::Vector3d> lane_points;
      for (const auto& point : left_points.line_segment().point()) {
        Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
        lane_points.emplace_back(point_utm);
      }
      adsfi_proto::viz::Marker marker;
      PointsToMarker(cur_timestamp_, lane_points, &marker, 0.1);
      markers_lane.add_markers()->CopyFrom(marker);
    }

    for (const auto& right_points :
         hq_lane.right_boundary().curve().segment()) {
      std::vector<Eigen::Vector3d> lane_points;
      for (const auto& point : right_points.line_segment().point()) {
        Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
        lane_points.emplace_back(point_utm);
      }
      adsfi_proto::viz::Marker marker;
      PointsToMarker(cur_timestamp_, lane_points, &marker, 0.1);
      markers_lane.add_markers()->CopyFrom(marker);
    }
    if (!hq_lane.left_boundary().curve().segment().empty()) {
      if (!hq_lane.left_boundary()
               .curve()
               .segment()[0]
               .line_segment()
               .point()
               .empty()) {
        auto point = hq_lane.left_boundary()
                         .curve()
                         .segment()[0]
                         .line_segment()
                         .point()[0];
        Eigen::Vector3d point_left_boundary(point.x(), point.y(), point.z());
        adsfi_proto::viz::Marker marker_id;
        auto id = hq_lane.id().id();
        LineIdToMarker(cur_timestamp_, point_left_boundary, id, &marker_id);
        markers_lane.add_markers()->CopyFrom(marker_id);
      }
    }
  }

  RVIZ_AGENT.Publish(KTopicTopoAsignTopoMapLane, markers_lane);
}

void TopoAssignment::PointsToMarker(const double stamp,
                                    const std::vector<Eigen::Vector3d>& points,
                                    adsfi_proto::viz::Marker* marker,
                                    double color_type) {
  static int id = 0;
  marker->Clear();
  marker->mutable_header()->set_frameid("map");
  marker->mutable_header()->mutable_timestamp()->set_sec(stamp);
  marker->mutable_header()->mutable_timestamp()->set_nsec(stamp);
  marker->set_ns("ns_local_map_lane");
  marker->set_id(id++);
  marker->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
  marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
  marker->mutable_pose()->mutable_position()->set_x(0);
  marker->mutable_pose()->mutable_position()->set_y(0);
  marker->mutable_pose()->mutable_position()->set_z(0);
  marker->mutable_pose()->mutable_orientation()->set_x(0.);
  marker->mutable_pose()->mutable_orientation()->set_y(0.);
  marker->mutable_pose()->mutable_orientation()->set_z(0.);
  marker->mutable_pose()->mutable_orientation()->set_w(1.);
  marker->mutable_scale()->set_x(0.2);
  marker->mutable_scale()->set_y(0.2);
  marker->mutable_scale()->set_z(0.2);
  marker->mutable_lifetime()->set_sec(0);
  marker->mutable_lifetime()->set_nsec(150000000);

  adsfi_proto::viz::ColorRGBA color;
  color.set_a(1.0);
  color.set_r(1.0);
  color.set_g(color_type);
  color.set_b(0);

  marker->mutable_color()->CopyFrom(color);
  for (const auto& point : points) {
    auto pt = marker->add_points();
    pt->set_x(point.x());
    pt->set_y(point.y());
    pt->set_z(point.z());
  }

  if (marker->points().empty()) {
    HLOG_WARN << "empty lane line";
  }
}

void TopoAssignment::LineIdToMarker(const double stamp,
                                    const Eigen::Vector3d& point,
                                    const std::string& id,
                                    adsfi_proto::viz::Marker* marker) {
  static int id_set = 0;
  marker->mutable_header()->set_frameid("map");
  marker->mutable_header()->mutable_timestamp()->set_sec(stamp);
  marker->mutable_header()->mutable_timestamp()->set_nsec(stamp);
  marker->set_id(id_set++);
  marker->set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
  marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
  marker->mutable_pose()->mutable_orientation()->set_x(0.);
  marker->mutable_pose()->mutable_orientation()->set_y(0.);
  marker->mutable_pose()->mutable_orientation()->set_z(0.);
  marker->mutable_pose()->mutable_orientation()->set_w(1.);
  const double text_size = 1.5;
  marker->mutable_scale()->set_z(text_size);
  marker->mutable_lifetime()->set_sec(0);
  marker->mutable_lifetime()->set_nsec(200000000);
  marker->mutable_color()->set_a(1.0);
  marker->mutable_color()->set_r(0);
  marker->mutable_color()->set_g(1);
  marker->mutable_color()->set_b(0);
  marker->mutable_pose()->mutable_position()->set_x(point.x());
  marker->mutable_pose()->mutable_position()->set_y(point.y());
  marker->mutable_pose()->mutable_position()->set_z(point.z());

  auto text = marker->mutable_text();
  *text = "ID: " + id;
}

bool TopoAssignment::PerpendicularFootInSegment(const Eigen::Vector2d& p0,
                                                const Eigen::Vector2d& p1,
                                                const Eigen::Vector2d& pt) {
  //! 分别计算：p0p1向量与p0pt向量的夹角，以及p1p0向量与p1pt向量的夹角，
  //! 有一个为钝角就表示不在范围内
  Eigen::Vector2d p0p1 = p1 - p0;
  p0p1.normalize();
  Eigen::Vector2d p0pt = pt - p0;
  p0pt.normalize();
  double theta0 = std::acos(p0p1.transpose() * p0pt);

  Eigen::Vector2d p1p0 = p0 - p1;
  p1p0.normalize();
  Eigen::Vector2d p1pt = pt - p1;
  p1pt.normalize();
  double theta1 = std::acos(p1p0.transpose() * p1pt);

  return theta0 < M_PI_2 && theta1 < M_PI_2;
}

std::vector<Eigen::Vector2d> TopoAssignment::GetLaneStartAndEndPoint(
    const hozon::hdmap::Lane& lane, const bool left) {
  std::vector<Eigen::Vector2d> points;
  if (left) {
    if (!lane.left_boundary().curve().segment().empty()) {
      auto segment_size = lane.left_boundary().curve().segment_size();
      if (!lane.left_boundary()
               .curve()
               .segment()[0]
               .line_segment()
               .point()
               .empty() &&
          !lane.left_boundary()
               .curve()
               .segment()[segment_size - 1]
               .line_segment()
               .point()
               .empty()) {
        auto points_size_end = lane.left_boundary()
                                   .curve()
                                   .segment()[segment_size - 1]
                                   .line_segment()
                                   .point_size();
        auto point_start_utm =
            lane.left_boundary().curve().segment(0).line_segment().point(0);

        int zone = 51;
        double x = point_start_utm.x();
        double y = point_start_utm.y();
        auto ret = hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
        Eigen::Vector2d point_start;
        if (ret) {
          Eigen::Vector3d point_gcj(y, x, 0);
          Eigen::Vector3d point_enu =
              util::Geo::Gcj02ToEnu(point_gcj, ref_point_);
          point_start.x() = point_enu.x();
          point_start.y() = point_enu.y();
        } else {
          HLOG_ERROR << "=== UTM2GCS failed";
        }
        auto point_end_utm = lane.left_boundary()
                                 .curve()
                                 .segment(segment_size - 1)
                                 .line_segment()
                                 .point(points_size_end - 1);

        x = point_end_utm.x();
        y = point_end_utm.y();
        ret = hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
        Eigen::Vector2d point_end;
        if (ret) {
          Eigen::Vector3d point_gcj(y, x, 0);
          Eigen::Vector3d point_enu =
              util::Geo::Gcj02ToEnu(point_gcj, ref_point_);
          point_end.x() = point_enu.x();
          point_end.y() = point_enu.y();
        } else {
          HLOG_ERROR << "=== UTM2GCS failed";
        }

        points.emplace_back(point_start);
        points.emplace_back(point_end);
      }
    }
  } else {
    if (!lane.right_boundary().curve().segment().empty()) {
      auto segment_size = lane.right_boundary().curve().segment_size();
      if (!lane.right_boundary()
               .curve()
               .segment()[0]
               .line_segment()
               .point()
               .empty() &&
          !lane.right_boundary()
               .curve()
               .segment()[segment_size - 1]
               .line_segment()
               .point()
               .empty()) {
        auto points_size_end = lane.right_boundary()
                                   .curve()
                                   .segment()[segment_size - 1]
                                   .line_segment()
                                   .point_size();
        auto point_start_utm =
            lane.right_boundary().curve().segment(0).line_segment().point(0);

        int zone = 51;
        double x = point_start_utm.x();
        double y = point_start_utm.y();
        auto ret = hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
        Eigen::Vector2d point_start;
        if (ret) {
          Eigen::Vector3d point_gcj(y, x, 0);
          Eigen::Vector3d point_enu =
              util::Geo::Gcj02ToEnu(point_gcj, ref_point_);
          point_start.x() = point_enu.x();
          point_start.y() = point_enu.y();
        } else {
          HLOG_ERROR << "=== UTM2GCS failed";
        }
        auto point_end_utm = lane.right_boundary()
                                 .curve()
                                 .segment(segment_size - 1)
                                 .line_segment()
                                 .point(points_size_end - 1);

        x = point_end_utm.x();
        y = point_end_utm.y();
        ret = hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
        Eigen::Vector2d point_end;
        if (ret) {
          Eigen::Vector3d point_gcj(y, x, 0);
          Eigen::Vector3d point_enu =
              util::Geo::Gcj02ToEnu(point_gcj, ref_point_);
          point_end.x() = point_enu.x();
          point_end.y() = point_enu.y();
        } else {
          HLOG_ERROR << "=== UTM2GCS failed";
        }
        points.emplace_back(point_start);
        points.emplace_back(point_end);
      }
    }
  }

  return points;
}

int TopoAssignment::FindNearestPointIndex(
    const Eigen::Vector2d& point,
    const std::vector<hozon::common::Point3D>& points) {
  double miniDist = std::numeric_limits<double>::max();
  int index = 0;
  for (int i = 0; i < points.size(); ++i) {
    Eigen::Vector2d point_vector(points[i].x(), points[i].y());
    auto dist = (point_vector - point).norm();
    if (dist < miniDist) {
      miniDist = dist;
      index = i;
    }
  }
  return index;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
