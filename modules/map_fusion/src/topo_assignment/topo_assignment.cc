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
#include "map_fusion/map_service/global_hd_map.h"
#include "util/temp_log.h"
#include "util/tic_toc.h"

// NOLINTBEGIN
DEFINE_bool(topo_rviz, false, "topo assignment use rviz or not");
DEFINE_double(topo_lane_line_dist, 1.5,
              "the distance threshold for laneline belonging to a lane");
// NOLINTEND

namespace hozon {
namespace mp {
namespace mf {

int TopoAssignment::Init() {
  if (!RVIZ_AGENT.Ok()) {
    HLOG_WARN << "RvizAgent start failed";
  } else {
    if (FLAGS_topo_rviz) {
      int ret = RVIZ_AGENT.Register<adsfi_proto::viz::TransformStamped>(
          kTopicTopoAsignTf);
      if (ret < 0) {
        HLOG_WARN << "RvizAgent register " << kTopicTopoAsignTf << " failed";
      }

      ret =
          RVIZ_AGENT.Register<adsfi_proto::viz::Path>(kTopicTopoAsignLocation);
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
  }

  topo_map_ = std::make_shared<hozon::hdmap::Map>();
  local_map_ = std::make_shared<hozon::mapping::LocalMap>();

  return 0;
}

void TopoAssignment::OnLocalization(
    const std::shared_ptr<hozon::localization::Localization>& msg) {
  if (!msg) {
    HLOG_ERROR << "nullptr input localization";
    return;
  }

  auto stamp = msg->header().publish_stamp();

  // 提取全局定位
  //! TBD：这里将utm转成了gcj02，但实际内部需要的也是utm，后面考虑
  Eigen::Vector3d pos_global_utm(msg->pose().pos_utm_01().x(),
                                 msg->pose().pos_utm_01().y(),
                                 msg->pose().pos_utm_01().z());

  double utm_x = msg->pose().pos_utm_01().x();
  double utm_y = msg->pose().pos_utm_01().y();
  bool ret = hozon::common::coordinate_convertor::UTM2GCS(
      msg->pose().utm_zone_01(), &utm_x, &utm_y);
  if (!ret) {
    HLOG_ERROR << "UTM2GCS failed";
    return;
  }

  // gcj02
  Eigen::Vector3d pos_global(utm_y, utm_x, 0);

  auto yaw = msg->pose().euler_angles().z();
  auto roll = msg->pose().euler_angles().x();
  auto pitch = msg->pose().euler_angles().y();
  Eigen::Quaterniond quat_global =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());

  // 更新车辆ins位置
  {
    std::lock_guard<std::mutex> lock_vehicle_pose(vehicle_pose_mtx_);
    vehicle_pose_ = pos_global;
  }

  // orientation in local enu frame
  Eigen::Quaterniond q_W_V = quat_global;
  if (!init_) {
    HLOG_INFO << "ref_point_ = pose";
    ref_point_ = vehicle_pose_;

    init_pose_.Clear();
    init_pose_.mutable_gcj02()->set_x(utm_y);
    init_pose_.mutable_gcj02()->set_y(utm_x);
    init_pose_.mutable_gcj02()->set_z(0);
    init_pose_.mutable_pos_utm_01()->CopyFrom(msg->pose().pos_utm_01());
    init_pose_.set_utm_zone_01(msg->pose().utm_zone_01());
    init_pose_.mutable_euler_angles()->CopyFrom(msg->pose().euler_angles());
    init_pose_.mutable_local_pose()->CopyFrom(msg->pose().local_pose());
    init_pose_.mutable_euler_angles_local()->CopyFrom(
        msg->pose().euler_angles_local());
    init_pose_ser_ = init_pose_.SerializeAsString();
    init_ = true;
  }

  Eigen::Vector3d enu = util::Geo::Gcj02ToEnu(vehicle_pose_, ref_point_);

  // 可视化vehicle position
  if (FLAGS_topo_rviz) {
    VizLocation(enu, q_W_V, stamp);
  }

  {
    std::lock_guard<std::mutex> lock_pose(pose_mtx_);
    ins_q_w_v_ = q_W_V;
    ins_pose_ = enu;
  }
}

void TopoAssignment::OnInsNodeInfo(
    const std::shared_ptr<hozon::localization::HafNodeInfo>& msg) {
  Eigen::Vector3d pos;
  pos << msg->pos_gcj02().x(), msg->pos_gcj02().y(), msg->pos_gcj02().z();

  Eigen::Quaterniond quat(msg->quaternion().w(), msg->quaternion().x(),
                          msg->quaternion().y(), msg->quaternion().z());
  quat.normalize();
  OnLocationInGlobal(pos, quat, msg->header().gnss_stamp());
}

void TopoAssignment::OnLocationInGlobal(const Eigen::Vector3d& pos,
                                        const Eigen::Quaterniond& quat,
                                        double stamp) {}

void TopoAssignment::OnLocalMap(
    const std::shared_ptr<hozon::mapping::LocalMap>& msg) {
  if (msg->lane_lines().empty()) {
    return;
  }
  // 可视化hq map
  if (FLAGS_topo_rviz && RVIZ_AGENT.Ok()) {
    std::shared_ptr<hozon::hdmap::Map> hq_map =
        std::make_shared<hozon::hdmap::Map>();
    GLOBAL_HD_MAP->GetMap(hq_map.get());
    std::vector<adsfi_proto::viz::MarkerArray> result =
        marker_rviz_.LaneToMarker(hq_map, ref_point_, true);
    // adsfi_proto::viz::MarkerArray lane = result[0];
    adsfi_proto::viz::MarkerArray left = result[1];
    adsfi_proto::viz::MarkerArray right = result[2];

    // RVIZ_AGENT.Publish("lane", lane);
    RVIZ_AGENT.Publish("left", left);
    RVIZ_AGENT.Publish("right", right);
  }

  // 更新时间信息
  cur_timestamp_ = msg->header().publish_stamp();

  local_map_->CopyFrom(*msg);

  // 通过ins位置拿到hq中最近的车道
  double x = 0.;
  double y = 0.;
  {
    std::lock_guard<std::mutex> lock_vehicle_pose(vehicle_pose_mtx_);
    x = vehicle_pose_.x();
    y = vehicle_pose_.y();
  }

  hozon::common::coordinate_convertor::GCS2UTM(51, &y, &x);
  // utm位置
  hozon::common::PointENU enupos;
  enupos.set_x(y);
  enupos.set_y(x);
  enupos.set_z(0);

  double nearest_s = 0.;
  double nearest_l = 0.;
  hozon::hdmap::LaneInfoConstPtr lane_ptr = nullptr;

  if (GLOBAL_HD_MAP->Empty()) {
    HLOG_ERROR << "hqmap server load map failed";
    return;
  }

  int ret =
      GLOBAL_HD_MAP->GetNearestLane(enupos, &lane_ptr, &nearest_s, &nearest_l);

  if (ret != 0 || lane_ptr == nullptr) {
    HLOG_ERROR << "get nearest lane failed";
    return;
  }

  all_lane_info_.clear();
  const double range = 300;
  std::vector<hozon::hdmap::LaneInfoConstPtr> lanes;
  if (GLOBAL_HD_MAP->GetLanes(enupos, range, &lanes) != 0) {
    HLOG_ERROR << "GLOBAL_HD_MAP->GetLanes failed";
    return;
  }
  for (const auto& it : lanes) {
    const auto& id = it->id().id();
    AllLaneInfo info;
    info.id = id;
    info.left_start_end = GetLaneStartAndEndPoint(it->lane(), true);
    info.right_start_end = GetLaneStartAndEndPoint(it->lane(), false);
    for (const auto& itt : it->lane().left_neighbor_forward_lane_id()) {
      info.left_lane_ids.emplace_back(itt.id());
    }
    for (const auto& itt : it->lane().right_neighbor_forward_lane_id()) {
      info.right_lane_ids.emplace_back(itt.id());
    }
    for (const auto& itt : it->lane().predecessor_id()) {
      info.prev_lane_ids.emplace_back(itt.id());
    }
    for (const auto& itt : it->lane().successor_id()) {
      info.next_lane_ids.emplace_back(itt.id());
    }
    all_lane_info_.insert_or_assign(id, info);
  }

  location_left_id_.clear();
  location_right_id_.clear();
  location_left_id_next_.clear();
  location_right_id_next_.clear();

  hozon::hdmap::Lane nearest_lane = lane_ptr->lane();
  location_left_id_.emplace_back(nearest_lane.id().id());
  auto id_string = nearest_lane.id().id();
  while (all_lane_info_.find(id_string) != all_lane_info_.end() &&
         !all_lane_info_[id_string].left_lane_ids.empty()) {
    auto id = all_lane_info_[id_string].left_lane_ids[0];
    location_left_id_.emplace_back(id);
    id_string = id;
  }

  id_string = nearest_lane.id().id();
  while (all_lane_info_.find(id_string) != all_lane_info_.end() &&
         !all_lane_info_[id_string].right_lane_ids.empty()) {
    auto id = all_lane_info_[id_string].right_lane_ids[0];
    location_right_id_.emplace_back(id);
    id_string = id;
  }

  id_string = "";
  if (all_lane_info_.find(nearest_lane.id().id()) != all_lane_info_.end() &&
      !all_lane_info_[nearest_lane.id().id()].next_lane_ids.empty()) {
    id_string = all_lane_info_[nearest_lane.id().id()].next_lane_ids.back();
  }

  std::string next_lane_id = "";
  if (!id_string.empty()) {
    next_lane_id = id_string;
    location_left_id_next_.emplace_back(next_lane_id);
  }
  while (all_lane_info_.find(id_string) != all_lane_info_.end() &&
         !all_lane_info_[id_string].left_lane_ids.empty()) {
    auto id = all_lane_info_[id_string].left_lane_ids[0];
    location_left_id_next_.emplace_back(id);
    id_string = id;
  }

  id_string = next_lane_id;
  while (all_lane_info_.find(id_string) != all_lane_info_.end() &&
         !all_lane_info_[id_string].right_lane_ids.empty()) {
    auto id = all_lane_info_[id_string].right_lane_ids[0];
    location_right_id_next_.emplace_back(id);
    id_string = id;
  }

  // HLOG_ERROR << "id sizexxxxxxxxxxxxxxx:" << location_left_id_.size() << ", "
  //            << location_right_id_.size();
}

std::shared_ptr<hozon::hdmap::Map> TopoAssignment::GetTopoMap() {
  return topo_map_;
}

void TopoAssignment::TopoAssign() {
  // vehicle pose in local enu
  Eigen::Isometry3d T_U_V;
  {
    std::lock_guard<std::mutex> lock_pose(pose_mtx_);
    T_U_V.setIdentity();
    Eigen::Matrix3d orient = ins_q_w_v_.toRotationMatrix();
    T_U_V.rotate(orient);
    T_U_V.pretranslate(ins_pose_);
  }

  if (FLAGS_topo_rviz) {
    VizLocalMap(local_map_, T_U_V);
  }

  // 更新all_lanelines
  std::map<int32_t, LaneLine> all_lanelines;
  for (const auto& lane_line_it : local_map_->lane_lines()) {
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
    //   Eigen::Vector3d point_enu_start = T_U_V * point_local_start;
    //   Eigen::Vector3d point_enu_end = T_U_V * point_local_end;
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
    //         all_lanelines_[lane_line_it.track_id()].left_lanes[size -
    //         1]);
    //     if (hq_map_server_->GetLaneById(lane_id)) {
    //       auto hq_lane = hq_map_server_->GetLaneById(lane_id)->lane();
    //       auto hq_lane_left_points = GetLaneStartAndEndPoint(hq_lane,
    //       true);

    //       while (!hq_lane_left_points.empty() &&
    //              !PerpendicularFootInSegment(hq_lane_left_points[0],
    //                                          hq_lane_left_points[1], p1))
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
    //        i <
    //        all_lanelines_[lane_line_it.track_id()].right_lanes.size();
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
    //                                      hq_lane_right_points[point_size
    //                                      - 1], p0)) {
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
    //         all_lanelines_[lane_line_it.track_id()].right_lanes[size -
    //         1]);
    //     if (hq_map_server_->GetLaneById(lane_id)) {
    //       auto hq_lane = hq_map_server_->GetLaneById(lane_id)->lane();
    //       auto hq_lane_right_points = GetLaneStartAndEndPoint(hq_lane,
    //       false);

    //       while (!hq_lane_right_points.empty() &&
    //              !PerpendicularFootInSegment(hq_lane_right_points[0],
    //                                          hq_lane_right_points[1],
    //                                          p1))
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
    //         hq_lane_right_points = GetLaneStartAndEndPoint(hq_lane,
    //         false);
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
    //     Eigen::Vector3d point_enu = T_U_V * point_local;
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
    Eigen::Vector3d point_enu_start = T_U_V * point_local_start;
    Eigen::Vector3d point_enu_end = T_U_V * point_local_end;
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
          auto lane_id = location_left_id_[4];
          AppendLaneLine(lane_id, &lane_line, p0, p1, true);
        } else {
          if (location_left_id_next_.size() >= 5) {
            auto lane_id = location_left_id_next_[4];
            AppendLaneLine(lane_id, &lane_line, p0, p1, true);
          }
          if (location_left_id_next_.size() >= 6) {
            auto lane_id = location_left_id_next_[5];
            AppendLaneLine(lane_id, &lane_line, p0, p1, false);
          }
        }
        if (location_left_id_.size() >= 6) {
          auto lane_id = location_left_id_[5];
          AppendLaneLine(lane_id, &lane_line, p0, p1, false);
        }
        break;
      case hozon::mapping::LanePositionType::LanePositionType_FOURTH_LEFT:
        if (location_left_id_.size() >= 4) {
          auto lane_id = location_left_id_[3];
          AppendLaneLine(lane_id, &lane_line, p0, p1, true);
        } else {
          if (location_left_id_next_.size() >= 4) {
            auto lane_id = location_left_id_next_[3];
            AppendLaneLine(lane_id, &lane_line, p0, p1, true);
          }
          if (location_left_id_next_.size() >= 5) {
            auto lane_id = location_left_id_next_[4];
            AppendLaneLine(lane_id, &lane_line, p0, p1, false);
          }
        }
        if (location_left_id_.size() >= 5) {
          auto lane_id = location_left_id_[4];
          AppendLaneLine(lane_id, &lane_line, p0, p1, false);
        }
        break;
      case hozon::mapping::LanePositionType::LanePositionType_THIRD_LEFT:
        if (location_left_id_.size() >= 3) {
          auto lane_id = location_left_id_[2];
          AppendLaneLine(lane_id, &lane_line, p0, p1, true);
        } else {
          if (location_left_id_next_.size() >= 3) {
            auto lane_id = location_left_id_next_[2];
            AppendLaneLine(lane_id, &lane_line, p0, p1, true);
          }
          if (location_left_id_next_.size() >= 4) {
            auto lane_id = location_left_id_next_[3];
            AppendLaneLine(lane_id, &lane_line, p0, p1, false);
          }
        }
        if (location_left_id_.size() >= 4) {
          auto lane_id = location_left_id_[3];
          AppendLaneLine(lane_id, &lane_line, p0, p1, false);
        }
        break;
      case hozon::mapping::LanePositionType::LanePositionType_ADJACENT_LEFT:
        if (location_left_id_.size() >= 2) {
          auto lane_id = location_left_id_[1];
          AppendLaneLine(lane_id, &lane_line, p0, p1, true);
        } else {
          if (location_left_id_next_.size() >= 2) {
            auto lane_id = location_left_id_next_[1];
            AppendLaneLine(lane_id, &lane_line, p0, p1, true);
          }
          if (location_left_id_next_.size() >= 3) {
            auto lane_id = location_left_id_next_[2];
            AppendLaneLine(lane_id, &lane_line, p0, p1, false);
          }
        }
        if (location_left_id_.size() >= 3) {
          auto lane_id = location_left_id_[2];
          AppendLaneLine(lane_id, &lane_line, p0, p1, false);
        }
        break;
      case hozon::mapping::LanePositionType::LanePositionType_EGO_LEFT:
        if (location_left_id_.size() >= 1) {
          auto lane_id = location_left_id_[0];
          AppendLaneLine(lane_id, &lane_line, p0, p1, true);
        } else {
          if (location_left_id_next_.size() >= 1) {
            auto lane_id = location_left_id_next_[0];
            AppendLaneLine(lane_id, &lane_line, p0, p1, true);
          }
          if (location_left_id_next_.size() >= 2) {
            auto lane_id = location_left_id_next_[1];
            AppendLaneLine(lane_id, &lane_line, p0, p1, false);
          }
        }
        if (location_left_id_.size() >= 2) {
          auto lane_id = location_left_id_[1];
          AppendLaneLine(lane_id, &lane_line, p0, p1, false);
        }
        break;
      case hozon::mapping::LanePositionType::LanePositionType_EGO_RIGHT:
        if (location_left_id_.size() >= 1) {
          auto lane_id = location_left_id_[0];
          AppendLaneLine(lane_id, &lane_line, p0, p1, false);
        } else {
          if (location_left_id_next_.size() >= 1) {
            auto lane_id = location_left_id_next_[0];
            AppendLaneLine(lane_id, &lane_line, p0, p1, false);
          }
          if (location_right_id_next_.size() >= 1) {
            auto lane_id = location_right_id_next_[0];
            AppendLaneLine(lane_id, &lane_line, p0, p1, true);
          }
        }
        if (location_right_id_.size() >= 1) {
          auto lane_id = location_right_id_[0];
          AppendLaneLine(lane_id, &lane_line, p0, p1, true);
        }
        break;
      case hozon::mapping::LanePositionType::LanePositionType_ADJACENT_RIGHT:
        if (location_right_id_.size() >= 1) {
          auto lane_id = location_right_id_[0];
          AppendLaneLine(lane_id, &lane_line, p0, p1, false);
        } else {
          if (location_right_id_next_.size() >= 1) {
            auto lane_id = location_right_id_next_[0];
            AppendLaneLine(lane_id, &lane_line, p0, p1, false);
          }
          if (location_right_id_next_.size() >= 2) {
            auto lane_id = location_right_id_next_[1];
            AppendLaneLine(lane_id, &lane_line, p0, p1, true);
          }
        }
        if (location_right_id_.size() >= 2) {
          auto lane_id = location_right_id_[1];
          AppendLaneLine(lane_id, &lane_line, p0, p1, true);
        }
        break;
      case hozon::mapping::LanePositionType::LanePositionType_THIRD_RIGHT:
        if (location_right_id_.size() >= 2) {
          auto lane_id = location_right_id_[1];
          AppendLaneLine(lane_id, &lane_line, p0, p1, false);
        } else {
          if (location_right_id_next_.size() >= 2) {
            auto lane_id = location_right_id_next_[1];
            AppendLaneLine(lane_id, &lane_line, p0, p1, false);
          }
          if (location_right_id_next_.size() >= 3) {
            auto lane_id = location_right_id_next_[2];
            AppendLaneLine(lane_id, &lane_line, p0, p1, true);
          }
        }
        if (location_right_id_.size() >= 3) {
          auto lane_id = location_right_id_[2];
          AppendLaneLine(lane_id, &lane_line, p0, p1, true);
        }
        break;
      case hozon::mapping::LanePositionType::LanePositionType_FOURTH_RIGHT:
        if (location_right_id_.size() >= 3) {
          auto lane_id = location_right_id_[2];
          AppendLaneLine(lane_id, &lane_line, p0, p1, false);
        } else {
          if (location_right_id_next_.size() >= 3) {
            auto lane_id = location_right_id_next_[2];
            AppendLaneLine(lane_id, &lane_line, p0, p1, false);
          }
          if (location_right_id_next_.size() >= 4) {
            auto lane_id = location_right_id_next_[3];
            AppendLaneLine(lane_id, &lane_line, p0, p1, true);
          }
        }
        if (location_right_id_.size() >= 4) {
          auto lane_id = location_right_id_[3];
          AppendLaneLine(lane_id, &lane_line, p0, p1, true);
        }
        break;
      case hozon::mapping::LanePositionType::LanePositionType_BOLLARD_RIGHT:
        if (location_right_id_.size() >= 4) {
          auto lane_id = location_right_id_[3];
          AppendLaneLine(lane_id, &lane_line, p0, p1, false);
        } else {
          if (location_right_id_next_.size() >= 4) {
            auto lane_id = location_right_id_next_[3];
            AppendLaneLine(lane_id, &lane_line, p0, p1, false);
          }
          if (location_right_id_next_.size() >= 5) {
            auto lane_id = location_right_id_next_[4];
            AppendLaneLine(lane_id, &lane_line, p0, p1, true);
          }
        }
        if (location_right_id_.size() >= 5) {
          auto lane_id = location_right_id_[4];
          AppendLaneLine(lane_id, &lane_line, p0, p1, true);
        }
        break;

      default:
        break;
    }

    // 更新points全部转到enu坐标系下面
    // 同时构建每条local map感知车道线的kdtree
    std::vector<cv::Point2f> kdtree_points;
    for (const auto& line_point_it : lane_line_it.points()) {
      Eigen::Vector3d point_local(line_point_it.x(), line_point_it.y(),
                                  line_point_it.z());
      Eigen::Vector3d point_enu = T_U_V * point_local;
      hozon::common::Point3D lane_line_point;
      lane_line_point.set_x(point_enu.x());
      lane_line_point.set_y(point_enu.y());
      lane_line_point.set_z(point_enu.z());
      lane_line.points.emplace_back(lane_line_point);
      kdtree_points.emplace_back(static_cast<float>(point_enu.x()),
                                 static_cast<float>(point_enu.y()));
    }
    cv::flann::KDTreeIndexParams index_params(1);
    std::shared_ptr<cv::flann::Index> kdtree_ptr =
        std::make_shared<cv::flann::Index>(cv::Mat(kdtree_points).reshape(1),
                                           index_params);
    lane_line.lane_line_kdtree = kdtree_ptr;

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
  //! 从lane line构造lane
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
  // 设置header,这边复用localmap的header
  topo_map_->mutable_header()->mutable_header()->CopyFrom(local_map_->header());

  // 注意：用map.header.id来承载初始化的位姿，包含local enu站心
  topo_map_->mutable_header()->set_id(init_pose_ser_);

  AppendTopoMap(all_lanes, topo_map_);
  if (FLAGS_topo_rviz) {
    VizTopoMap(topo_map_);
  }
}

void TopoAssignment::AppendLaneLine(const std::string& lane_id,
                                    hozon::mp::mf::LaneLine* lane_line,
                                    const Eigen::Vector2d& p0,
                                    const Eigen::Vector2d& p1,
                                    const bool left) {
  bool flag_p0 = false;
  bool flag_p1 = false;
  bool flag_pt = false;

  if (all_lane_info_.find(lane_id) == all_lane_info_.end()) {
    return;
  }

  std::vector<Eigen::Vector2d> hq_lane_points;
  if (left) {
    hq_lane_points = all_lane_info_[lane_id].left_start_end;
  } else {
    hq_lane_points = all_lane_info_[lane_id].right_start_end;
  }

  double distance_p0 = 0.;
  double distance_p1 = 0.;
  if (!hq_lane_points.empty()) {
    flag_p0 =
        PerpendicularFootInSegment(hq_lane_points[0], hq_lane_points[1], p0);
    flag_p1 =
        PerpendicularFootInSegment(hq_lane_points[0], hq_lane_points[1], p1);
    flag_pt = PerpendicularFootInSegment(p0, p1, hq_lane_points[0]);
    distance_p0 = (hq_lane_points[0] - p0).norm();
    distance_p1 = (hq_lane_points[0] - p1).norm();
  }
  if (!flag_p0 && !flag_p1 && !flag_pt) {
    if (distance_p0 >= distance_p1) {
      std::vector<std::string> head_lane_id_p0 =
          FindLaneLineHead(lane_id, p0, left);
      std::vector<std::string> head_lane_id_p1 =
          FindLaneLineHead(lane_id, p1, left);
      auto size = head_lane_id_p1.size() - 1;
      if (size >= 0) {
        for (int i = size; i < head_lane_id_p0.size(); ++i) {
          if (left) {
            lane_line->left_lanes.emplace_back(head_lane_id_p0[i]);
          } else {
            lane_line->right_lanes.emplace_back(head_lane_id_p0[i]);
          }
        }

        if (left) {
          std::reverse(lane_line->left_lanes.begin(),
                       lane_line->left_lanes.end());
          if (!lane_line->left_lanes.empty()) {
            if (LaneLineBelongToLane(lane_line->left_lanes, left, p0, p1)) {
              lane_line->left_lanes.clear();
            }
          }
        } else {
          std::reverse(lane_line->right_lanes.begin(),
                       lane_line->right_lanes.end());
          if (!lane_line->right_lanes.empty()) {
            if (LaneLineBelongToLane(lane_line->right_lanes, left, p0, p1)) {
              lane_line->right_lanes.clear();
            }
          }
        }
      }
    } else {
      std::vector<std::string> tail_lane_id_p0 =
          FindLaneLineTail(lane_id, p1, left);
      std::vector<std::string> tail_lane_id_p1 =
          FindLaneLineTail(lane_id, p0, left);
      auto size = tail_lane_id_p0.size() - 1;
      if (size >= 0) {
        for (int i = size; i < tail_lane_id_p1.size(); ++i) {
          if (left) {
            lane_line->left_lanes.emplace_back(tail_lane_id_p1[i]);
          } else {
            lane_line->right_lanes.emplace_back(tail_lane_id_p1[i]);
          }
        }

        if (left) {
          if (!lane_line->left_lanes.empty()) {
            if (LaneLineBelongToLane(lane_line->left_lanes, left, p0, p1)) {
              lane_line->left_lanes.clear();
            }
          }
        } else {
          if (!lane_line->right_lanes.empty()) {
            if (LaneLineBelongToLane(lane_line->right_lanes, left, p0, p1)) {
              lane_line->right_lanes.clear();
            }
          }
        }
      }
    }
  } else {
    if (left) {
      lane_line->left_lanes.emplace_back(lane_id);
    } else {
      lane_line->right_lanes.emplace_back(lane_id);
    }

    std::string curr_lane_id = lane_id;
    while (
        !hq_lane_points.empty() &&
        !PerpendicularFootInSegment(hq_lane_points[0], hq_lane_points[1], p0)) {
      if (all_lane_info_[curr_lane_id].prev_lane_ids.empty()) {
        break;
      }

      auto prev_id = all_lane_info_[curr_lane_id].prev_lane_ids.back();
      if (all_lane_info_.find(prev_id) == all_lane_info_.end()) {
        break;
      }
      if (left) {
        hq_lane_points = all_lane_info_[prev_id].left_start_end;
      } else {
        hq_lane_points = all_lane_info_[prev_id].right_start_end;
      }

      if (left) {
        lane_line->left_lanes.emplace_back(prev_id);
      } else {
        lane_line->right_lanes.emplace_back(prev_id);
      }
      curr_lane_id = prev_id;
    }

    if (left) {
      std::reverse(lane_line->left_lanes.begin(), lane_line->left_lanes.end());
    } else {
      std::reverse(lane_line->right_lanes.begin(),
                   lane_line->right_lanes.end());
    }

    // 追踪最后一个点
    if (left) {
      hq_lane_points = all_lane_info_[lane_id].left_start_end;
    } else {
      hq_lane_points = all_lane_info_[lane_id].right_start_end;
    }

    curr_lane_id = lane_id;
    while (
        !hq_lane_points.empty() &&
        !PerpendicularFootInSegment(hq_lane_points[0], hq_lane_points[1], p1)) {
      if (all_lane_info_[curr_lane_id].next_lane_ids.empty()) {
        break;
      }
      auto next_id = all_lane_info_[curr_lane_id].next_lane_ids.back();
      if (all_lane_info_.find(next_id) == all_lane_info_.end()) {
        break;
      }

      if (left) {
        hq_lane_points = all_lane_info_[next_id].left_start_end;
      } else {
        hq_lane_points = all_lane_info_[next_id].right_start_end;
      }

      if (left) {
        lane_line->left_lanes.emplace_back(next_id);
      } else {
        lane_line->right_lanes.emplace_back(next_id);
      }
      curr_lane_id = next_id;
    }

    if (left) {
      if (!lane_line->left_lanes.empty()) {
        if (LaneLineBelongToLane(lane_line->left_lanes, left, p0, p1)) {
          lane_line->left_lanes.clear();
        }
      }
    } else {
      if (!lane_line->right_lanes.empty()) {
        if (LaneLineBelongToLane(lane_line->right_lanes, left, p0, p1)) {
          lane_line->right_lanes.clear();
        }
      }
    }
  }
}

std::vector<std::string> TopoAssignment::FindLaneLineHead(
    const std::string& lane_id, const Eigen::Vector2d& pt, const bool left) {
  std::vector<std::string> lane_id_vec;
  if (all_lane_info_.find(lane_id) == all_lane_info_.end()) {
    return {};
  }
  std::vector<Eigen::Vector2d> hq_lane_points;
  if (left) {
    hq_lane_points = all_lane_info_[lane_id].left_start_end;
  } else {
    hq_lane_points = all_lane_info_[lane_id].right_start_end;
  }

  lane_id_vec.emplace_back(lane_id);

  std::string curr_lane_id = lane_id;
  while (
      !hq_lane_points.empty() &&
      !PerpendicularFootInSegment(hq_lane_points[0], hq_lane_points[1], pt)) {
    if (all_lane_info_[curr_lane_id].prev_lane_ids.empty()) {
      break;
    }

    auto prev_id = all_lane_info_[curr_lane_id].prev_lane_ids.back();
    if (all_lane_info_.find(prev_id) == all_lane_info_.end()) {
      break;
    }
    if (left) {
      hq_lane_points = all_lane_info_[prev_id].left_start_end;
    } else {
      hq_lane_points = all_lane_info_[prev_id].right_start_end;
    }

    lane_id_vec.emplace_back(prev_id);
    curr_lane_id = prev_id;
  }
  return lane_id_vec;
}

std::vector<std::string> TopoAssignment::FindLaneLineTail(
    const std::string& lane_id, const Eigen::Vector2d& pt, const bool left) {
  std::vector<std::string> lane_id_vec;
  if (all_lane_info_.find(lane_id) == all_lane_info_.end()) {
    return {};
  }

  std::vector<Eigen::Vector2d> hq_lane_points;
  if (left) {
    hq_lane_points = all_lane_info_[lane_id].left_start_end;
  } else {
    hq_lane_points = all_lane_info_[lane_id].right_start_end;
  }

  lane_id_vec.emplace_back(lane_id);

  std::string curr_lane_id = lane_id;
  while (
      !hq_lane_points.empty() &&
      !PerpendicularFootInSegment(hq_lane_points[0], hq_lane_points[1], pt)) {
    if (all_lane_info_[curr_lane_id].next_lane_ids.empty()) {
      break;
    }

    auto next_id = all_lane_info_[curr_lane_id].next_lane_ids.back();
    if (all_lane_info_.find(next_id) == all_lane_info_.end()) {
      break;
    }
    if (left) {
      hq_lane_points = all_lane_info_[next_id].left_start_end;
    } else {
      hq_lane_points = all_lane_info_[next_id].right_start_end;
    }
    lane_id_vec.emplace_back(next_id);
    curr_lane_id = next_id;
  }
  return lane_id_vec;
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
    const std::string lane_id = lane_it.second.id;
    if (all_lane_info_.find(lane_id) == all_lane_info_.end()) {
      continue;
    }

    const auto& left_ids = all_lane_info_[lane_id].left_lane_ids;

    for (const auto& left_lane_it : left_ids) {
      auto left_lane_it_id = left_lane_it;
      if (all_lanes->find(left_lane_it_id) != all_lanes->end()) {
        lane_it.second.left_lanes.emplace_back(left_lane_it_id);
      }
    }
    const auto& right_ids = all_lane_info_[lane_id].right_lane_ids;
    for (const auto& right_lane_it : right_ids) {
      auto right_lane_it_id = right_lane_it;
      if (all_lanes->find(right_lane_it_id) != all_lanes->end()) {
        lane_it.second.right_lanes.emplace_back(right_lane_it_id);
      }
    }
    const auto& prev_ids = all_lane_info_[lane_id].prev_lane_ids;
    for (const auto& prev_lane_it : prev_ids) {
      auto prev_lane_it_id = prev_lane_it;
      if (all_lanes->find(prev_lane_it_id) != all_lanes->end()) {
        lane_it.second.prev_lanes.emplace_back(prev_lane_it_id);
      }
    }
    const auto& next_ids = all_lane_info_[lane_id].next_lane_ids;
    for (const auto& next_lane_it : next_ids) {
      auto next_lane_it_id = next_lane_it;
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
    const std::string lane_id = lane_it.second.id;
    if (all_lane_info_.find(lane_id) == all_lane_info_.end()) {
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
    auto hq_lane_left_points = all_lane_info_[lane_id].left_start_end;
    auto hq_lane_right_points = all_lane_info_[lane_id].right_start_end;
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
            auto segment =
                lane->mutable_left_boundary()->mutable_curve()->add_segment();
            for (const auto& point : all_lanelines_[left_line_it].points) {
              auto topo_point = segment->mutable_line_segment()->add_point();
              topo_point->set_x(point.x());
              topo_point->set_y(point.y());
              topo_point->set_z(point.z());
            }
          }
          if (flag_p0 && !flag_p1) {
            // 找到后面一个点最近的index
            const int dim = 1;
            std::vector<float> query_points = std::vector<float>{
                static_cast<float>(
                    hq_lane_left_points[hq_lane_left_points_size - 1].x()),
                static_cast<float>(
                    hq_lane_left_points[hq_lane_left_points_size - 1].y())};
            auto index = KnnSearchNearestPointIndex(
                dim, query_points,
                all_lanelines_[left_line_it].lane_line_kdtree);
            auto segment =
                lane->mutable_left_boundary()->mutable_curve()->add_segment();
            for (int i = 0; i <= index; ++i) {
              auto topo_point = segment->mutable_line_segment()->add_point();
              topo_point->set_x(all_lanelines_[left_line_it].points[i].x());
              topo_point->set_y(all_lanelines_[left_line_it].points[i].y());
              topo_point->set_z(all_lanelines_[left_line_it].points[i].z());
            }
          }
          if (!flag_p0 && flag_p1) {
            // 找到前面一个点最近的index
            const int dim = 1;
            std::vector<float> query_points = std::vector<float>{
                static_cast<float>(hq_lane_left_points[0].x()),
                static_cast<float>(hq_lane_left_points[0].y())};

            auto index = KnnSearchNearestPointIndex(
                dim, query_points,
                all_lanelines_[left_line_it].lane_line_kdtree);
            auto segment =
                lane->mutable_left_boundary()->mutable_curve()->add_segment();
            for (int i = index; i < point_size; ++i) {
              auto topo_point = segment->mutable_line_segment()->add_point();
              topo_point->set_x(all_lanelines_[left_line_it].points[i].x());
              topo_point->set_y(all_lanelines_[left_line_it].points[i].y());
              topo_point->set_z(all_lanelines_[left_line_it].points[i].z());
            }
          }
          if (!flag_p0 && !flag_p1 && flag_pt) {
            // 找到两个点最近的index
            const int dim = 1;
            std::vector<float> query_points_start = std::vector<float>{
                static_cast<float>(hq_lane_left_points[0].x()),
                static_cast<float>(hq_lane_left_points[0].y())};
            std::vector<float> query_points_end = std::vector<float>{
                static_cast<float>(
                    hq_lane_left_points[hq_lane_left_points_size - 1].x()),
                static_cast<float>(
                    hq_lane_left_points[hq_lane_left_points_size - 1].y())};
            auto start_index = KnnSearchNearestPointIndex(
                dim, query_points_start,
                all_lanelines_[left_line_it].lane_line_kdtree);
            auto end_index = KnnSearchNearestPointIndex(
                dim, query_points_end,
                all_lanelines_[left_line_it].lane_line_kdtree);
            auto segment =
                lane->mutable_left_boundary()->mutable_curve()->add_segment();
            for (int i = start_index; i <= end_index; ++i) {
              auto topo_point = segment->mutable_line_segment()->add_point();
              topo_point->set_x(all_lanelines_[left_line_it].points[i].x());
              topo_point->set_y(all_lanelines_[left_line_it].points[i].y());
              topo_point->set_z(all_lanelines_[left_line_it].points[i].z());
            }
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
            auto segment =
                lane->mutable_right_boundary()->mutable_curve()->add_segment();
            for (const auto& point : all_lanelines_[right_line_it].points) {
              auto topo_point = segment->mutable_line_segment()->add_point();
              topo_point->set_x(point.x());
              topo_point->set_y(point.y());
              topo_point->set_z(point.z());
            }
          }
          if (flag_p0 && !flag_p1) {
            // 找到后面一个点最近的index
            const int dim = 1;
            std::vector<float> query_points = std::vector<float>{
                static_cast<float>(
                    hq_lane_left_points[hq_lane_left_points_size - 1].x()),
                static_cast<float>(
                    hq_lane_left_points[hq_lane_left_points_size - 1].y())};
            auto index = KnnSearchNearestPointIndex(
                dim, query_points,
                all_lanelines_[right_line_it].lane_line_kdtree);
            auto segment =
                lane->mutable_right_boundary()->mutable_curve()->add_segment();
            for (int i = 0; i <= index; ++i) {
              auto topo_point = segment->mutable_line_segment()->add_point();
              topo_point->set_x(all_lanelines_[right_line_it].points[i].x());
              topo_point->set_y(all_lanelines_[right_line_it].points[i].y());
              topo_point->set_z(all_lanelines_[right_line_it].points[i].z());
            }
          }
          if (!flag_p0 && flag_p1) {
            // 找到前面一个点最近的index
            const int dim = 1;
            std::vector<float> query_points = std::vector<float>{
                static_cast<float>(hq_lane_left_points[0].x()),
                static_cast<float>(hq_lane_left_points[0].y())};
            auto index = KnnSearchNearestPointIndex(
                dim, query_points,
                all_lanelines_[right_line_it].lane_line_kdtree);
            auto segment =
                lane->mutable_right_boundary()->mutable_curve()->add_segment();
            for (int i = index; i < point_size; ++i) {
              auto topo_point = segment->mutable_line_segment()->add_point();
              topo_point->set_x(all_lanelines_[right_line_it].points[i].x());
              topo_point->set_y(all_lanelines_[right_line_it].points[i].y());
              topo_point->set_z(all_lanelines_[right_line_it].points[i].z());
            }
          }
          if (!flag_p0 && !flag_p1 && flag_pt) {
            // 找到两个点最近的index
            const int dim = 1;
            std::vector<float> query_points_start = std::vector<float>{
                static_cast<float>(hq_lane_left_points[0].x()),
                static_cast<float>(hq_lane_left_points[0].y())};
            std::vector<float> query_points_end = std::vector<float>{
                static_cast<float>(
                    hq_lane_left_points[hq_lane_left_points_size - 1].x()),
                static_cast<float>(
                    hq_lane_left_points[hq_lane_left_points_size - 1].y())};
            auto start_index = KnnSearchNearestPointIndex(
                dim, query_points_start,
                all_lanelines_[right_line_it].lane_line_kdtree);
            auto end_index = KnnSearchNearestPointIndex(
                dim, query_points_end,
                all_lanelines_[right_line_it].lane_line_kdtree);
            auto segment =
                lane->mutable_right_boundary()->mutable_curve()->add_segment();
            for (int i = start_index; i <= end_index; ++i) {
              auto topo_point = segment->mutable_line_segment()->add_point();
              topo_point->set_x(all_lanelines_[right_line_it].points[i].x());
              topo_point->set_y(all_lanelines_[right_line_it].points[i].y());
              topo_point->set_z(all_lanelines_[right_line_it].points[i].z());
            }
          }
        }
      }
    }
  }
}

void TopoAssignment::VizLocalMap(
    const std::shared_ptr<hozon::mapping::LocalMap>& local_map,
    const Eigen::Isometry3d& T_U_V) {
  if (!RVIZ_AGENT.Ok()) {
    return;
  }
  adsfi_proto::viz::MarkerArray markers;
  for (const auto& i : local_map->lane_lines()) {
    std::vector<Eigen::Vector3d> lane_points;
    for (const auto& point : i.points()) {
      Eigen::Vector3d point_local(point.x(), point.y(), point.z());
      Eigen::Vector3d point_enu = T_U_V * point_local;
      lane_points.emplace_back(point_enu);
    }
    adsfi_proto::viz::Marker marker;
    PointsToMarker(local_map->header().publish_stamp(), lane_points, &marker,
                   1);
    if (marker.points().size() >= 2) {
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
          if (marker.points().size() >= 2) {
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
      if (marker.points().size() >= 2) {
        markers_lane.add_markers()->CopyFrom(marker);
      }
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
      if (marker.points().size() >= 2) {
        markers_lane.add_markers()->CopyFrom(marker);
      }
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
          PointsToMarker(cur_timestamp_, boundary_points, &marker, 1);
          if (marker.points().size() >= 2) {
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
      PointsToMarker(cur_timestamp_, lane_points, &marker, 1);
      if (marker.points().size() >= 2) {
        markers_lane.add_markers()->CopyFrom(marker);
      }
    }

    for (const auto& right_points :
         hq_lane.right_boundary().curve().segment()) {
      std::vector<Eigen::Vector3d> lane_points;
      for (const auto& point : right_points.line_segment().point()) {
        Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
        lane_points.emplace_back(point_utm);
      }
      adsfi_proto::viz::Marker marker;
      PointsToMarker(cur_timestamp_, lane_points, &marker, 1);
      if (marker.points().size() >= 2) {
        markers_lane.add_markers()->CopyFrom(marker);
      }
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
  color.set_r(0.1);
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
  marker->mutable_color()->set_r(1);
  marker->mutable_color()->set_g(0.1);
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

int TopoAssignment::KnnSearchNearestPointIndex(
    const int dim, const std::vector<float>& query_points,
    const std::shared_ptr<cv::flann::Index>& kd_tee) {
  std::vector<int> nearest_index(dim);
  std::vector<float> nearest_dist(dim);

  kd_tee->knnSearch(query_points, nearest_index, nearest_dist, dim,
                    cv::flann::SearchParams(-1));
  return nearest_index[0];
}

double TopoAssignment::PointDistanceToSegment(
    const std::vector<Eigen::Vector2d>& points, const Eigen::Vector2d& pt) {
  double distance = 0.;
  if (!points.empty()) {
    double A = points[1].y() - points[0].y();
    double B = points[0].x() - points[1].x();
    double C =
        (points[1].x() * points[0].y()) - (points[0].x() * points[1].y());

    distance =
        std::abs((A * pt.x() + B * pt.y() + C) / std::sqrt(A * A + B * B));
  }

  return distance;
}

bool TopoAssignment::LaneLineBelongToLane(const std::vector<std::string>& lanes,
                                          const bool left,
                                          const Eigen::Vector2d& p0,
                                          const Eigen::Vector2d& p1) {
  std::vector<Eigen::Vector2d> hq_lane_points;
  if (left) {
    hq_lane_points = all_lane_info_[lanes.front()].left_start_end;
  } else {
    hq_lane_points = all_lane_info_[lanes.front()].right_start_end;
  }
  double dist_p0 = PointDistanceToSegment(hq_lane_points, p0);

  if (left) {
    hq_lane_points = all_lane_info_[lanes.back()].left_start_end;
  } else {
    hq_lane_points = all_lane_info_[lanes.back()].right_start_end;
  }
  double dist_p1 = PointDistanceToSegment(hq_lane_points, p1);

  return dist_p0 >= FLAGS_topo_lane_line_dist &&
         dist_p1 >= FLAGS_topo_lane_line_dist;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
