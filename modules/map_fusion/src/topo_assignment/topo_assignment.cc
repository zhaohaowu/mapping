/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： topo_assignment.cc
 *   author     ： xuliang
 *   date       ： 2023.09
 ******************************************************************************/

#include "map_fusion/topo_assignment/topo_assignment.h"
#include <gflags/gflags.h>

#include <limits>

#include "Eigen/src/Core/Matrix.h"
#include "map_fusion/map_service/global_hd_map.h"
#include "util/mapping_log.h"
#include "util/tic_toc.h"

// NOLINTBEGIN
DEFINE_bool(topo_rviz, false, "topo assignment use rviz or not");
DEFINE_double(topo_lane_line_dist, 1.0,
              "the distance threshold for laneline belonging to a lane");
DEFINE_double(topo_lane_line_dist_point, 1.5,
              "the distance threshold for laneline belonging to a lane point");
DEFINE_double(topo_lane_line_point_dist, 10.0,
              "the distance threshold for laneline point belonging to a lane");
// NOLINTEND

namespace hozon {
namespace mp {
namespace mf {

int TopoAssignment::Init() {
  if (RVIZ_AGENT.Ok() && FLAGS_topo_rviz) {
    int ret = RVIZ_AGENT.Register<adsfi_proto::viz::TransformStamped>(
        kTopicTopoAssignTf);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register " << kTopicTopoAssignTf << " failed";
    }

    ret = RVIZ_AGENT.Register<adsfi_proto::viz::Path>(kTopicTopoAssignLocation);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register " << kTopicTopoAssignLocation
                << " failed";
    }

    std::vector<std::string> topic_vec = {KTopicTopoAssignLocalMap,
                                          KTopicTopoAssignHQMapRoad,
                                          KTopicTopoAssignHQMapLane,
                                          KTopicTopoAssignTopoMapRoad,
                                          KTopicTopoAssignTopoMapLane,
                                          KTopicTopoAssignTopoMapElements,
                                          "lane",
                                          "left",
                                          "right"};

    ret = RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(topic_vec);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register failed";
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

  auto stamp = msg->header().data_stamp();

  // 提取全局定位
  Eigen::Vector3d pos_global_utm(msg->pose().pos_utm_01().x(),
                                 msg->pose().pos_utm_01().y(),
                                 msg->pose().pos_utm_01().z());

  // gcj02
  Eigen::Vector3d pos_global(msg->pose().gcj02().x(), msg->pose().gcj02().y(),
                             msg->pose().gcj02().z());

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
    vehicle_pose_ = pos_global_utm;
  }

  if (!init_ref_point_) {
    HLOG_ERROR << "ref point not init yet";
    return;
  }

  Eigen::Vector3d enu = util::Geo::Gcj02ToEnu(pos_global, ref_point_);

  // Eigen::Vector3d pos_local;
  // pos_local << msg->pose().local_pose().x(), msg->pose().local_pose().y(),
  //     msg->pose().local_pose().z();

  // auto yaw_local = msg->pose().euler_angles_local().z();
  // auto roll_local = msg->pose().euler_angles_local().x();
  // auto pitch_local = msg->pose().euler_angles_local().y();

  // Eigen::Quaterniond quat_local =
  //     Eigen::AngleAxisd(yaw_local, Eigen::Vector3d::UnitZ()) *
  //     Eigen::AngleAxisd(roll_local, Eigen::Vector3d::UnitX()) *
  //     Eigen::AngleAxisd(pitch_local, Eigen::Vector3d::UnitY());

  // 可视化vehicle position
  if (FLAGS_topo_rviz) {
    VizLocation(enu, quat_global, stamp);
  }

  {
    std::lock_guard<std::mutex> lock_pose(pose_mtx_);
    ins_q_w_v_ = quat_global;
    ins_pose_ = enu;
  }
}

void TopoAssignment::OnLocalMap(
    const std::shared_ptr<hozon::mapping::LocalMap>& msg,
    const std::unordered_map<std::string, hozon::mp::mf::LaneInfo>&
        all_lane_info) {
  if (msg->lane_lines().empty()) {
    return;
  }
  // 可视化hq map
  // VizHQMap();

  // 更新时间信息
  cur_timestamp_ = msg->header().data_stamp();

  local_map_->CopyFrom(*msg);

  // 通过ins位置拿到hq中最近的车道
  hozon::common::PointENU enupos;

  {
    std::lock_guard<std::mutex> lock_vehicle_pose(vehicle_pose_mtx_);
    enupos.set_x(vehicle_pose_.x());
    enupos.set_y(vehicle_pose_.y());
    enupos.set_z(0);
  }

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
  // const double range = 300;
  // std::vector<hozon::hdmap::LaneInfoConstPtr> lanes;
  // if (GLOBAL_HD_MAP->GetLanes(enupos, range, &lanes) != 0) {
  //   HLOG_ERROR << "GLOBAL_HD_MAP->GetLanes failed";
  //   return;
  // }

  AppendAllLaneInfo(all_lane_info);
  AppendLocationLaneId(lane_ptr);

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
    // VizLocalMap(local_map_, T_U_V);
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

    const auto& start_point = lane_line_it.points(0);
    auto point_size = lane_line_it.points().size();
    const auto& end_point = lane_line_it.points(point_size - 1);
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
        AppendLaneLineCaseLeft(&lane_line, p0, p1, 5);
        break;
      case hozon::mapping::LanePositionType::LanePositionType_FOURTH_LEFT:
        AppendLaneLineCaseLeft(&lane_line, p0, p1, 4);
        break;
      case hozon::mapping::LanePositionType::LanePositionType_THIRD_LEFT:
        AppendLaneLineCaseLeft(&lane_line, p0, p1, 3);
        break;
      case hozon::mapping::LanePositionType::LanePositionType_ADJACENT_LEFT:
        AppendLaneLineCaseLeft(&lane_line, p0, p1, 2);
        break;
      case hozon::mapping::LanePositionType::LanePositionType_EGO_LEFT:
        AppendLaneLineCaseLeft(&lane_line, p0, p1, 1);
        break;
      case hozon::mapping::LanePositionType::LanePositionType_EGO_RIGHT:
        AppendLaneLineCaseLeftRight(&lane_line, p0, p1, 1);
        break;
      case hozon::mapping::LanePositionType::LanePositionType_ADJACENT_RIGHT:
        AppendLaneLineCaseRight(&lane_line, p0, p1, 1);
        break;
      case hozon::mapping::LanePositionType::LanePositionType_THIRD_RIGHT:
        AppendLaneLineCaseRight(&lane_line, p0, p1, 2);
        break;
      case hozon::mapping::LanePositionType::LanePositionType_FOURTH_RIGHT:
        AppendLaneLineCaseRight(&lane_line, p0, p1, 3);
        break;
      case hozon::mapping::LanePositionType::LanePositionType_BOLLARD_RIGHT:
        AppendLaneLineCaseRight(&lane_line, p0, p1, 4);
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
  std::pair<bool, std::map<std::string, Lane>> all_lanes;
  all_lanes.first = false;
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
  // topo_map_->mutable_header()->set_id(init_pose_ser_);

  std::shared_ptr<hozon::hdmap::Map> topo_map_geo =
      std::make_shared<hozon::hdmap::Map>();

  AppendTopoMapGeometry(&all_lanes, topo_map_geo);
  AppendTopoMapTopology(all_lanes.second, topo_map_geo, topo_map_);
  AppendTopoMapElements(topo_map_, T_U_V);
  if (FLAGS_topo_rviz) {
    VizTopoMap(topo_map_);
  }
}

void TopoAssignment::AppendAllLaneInfo(
    const std::unordered_map<std::string, hozon::mp::mf::LaneInfo>&
        all_lane_info) {
  for (const auto& it : all_lane_info) {
    AllLaneInfo info;
    info.id = it.first;
    std::vector<Eigen::Vector2d> points_vec;
    Eigen::Vector2d point;
    if (it.second.pred_left_line.size() >= 2) {
      point.x() = it.second.pred_left_line.front().x();
      point.y() = it.second.pred_left_line.front().y();
      points_vec.emplace_back(point);
      point.x() = it.second.pred_left_line.back().x();
      point.y() = it.second.pred_left_line.back().y();
      points_vec.emplace_back(point);
      info.left_start_end = points_vec;
    }
    points_vec.clear();
    if (it.second.pred_right_line.size() >= 2) {
      point.x() = it.second.pred_right_line.front().x();
      point.y() = it.second.pred_right_line.front().y();
      points_vec.emplace_back(point);
      point.x() = it.second.pred_right_line.back().x();
      point.y() = it.second.pred_right_line.back().y();
      points_vec.emplace_back(point);
      info.right_start_end = points_vec;
    }
    info.left_lane_ids = it.second.left_lane_ids;
    info.right_lane_ids = it.second.right_lane_ids;
    info.prev_lane_ids = it.second.prev_lane_ids;
    info.next_lane_ids = it.second.next_lane_ids;
    info.extra_boundary = it.second.extra_boundary;
    all_lane_info_.insert_or_assign(info.id, info);
    if (!init_ref_point_) {
      ref_point_ = it.second.ref_point;
      init_ref_point_ = true;
    }
  }
}

void TopoAssignment::AppendLocationLaneId(
    const hozon::hdmap::LaneInfoConstPtr& lane_ptr) {
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

  std::string next_lane_id;
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
}

void TopoAssignment::AppendLaneLineCaseLeft(hozon::mp::mf::LaneLine* lane_line,
                                            const Eigen::Vector2d& p0,
                                            const Eigen::Vector2d& p1,
                                            const size_t size) {
  if (location_left_id_.size() >= size) {
    auto lane_id = location_left_id_[size - 1];
    AppendLaneLineLeft(lane_id, lane_line, p0, p1);
  } else {
    if (location_left_id_next_.size() >= size) {
      auto lane_id = location_left_id_next_[size - 1];
      AppendLaneLineLeft(lane_id, lane_line, p0, p1);
    }
    if (location_left_id_next_.size() >= size + 1) {
      auto lane_id = location_left_id_next_[size];
      AppendLaneLineRight(lane_id, lane_line, p0, p1);
    }
  }
  if (location_left_id_.size() >= size + 1) {
    auto lane_id = location_left_id_[size];
    AppendLaneLineRight(lane_id, lane_line, p0, p1);
  }
}

void TopoAssignment::AppendLaneLineCaseRight(hozon::mp::mf::LaneLine* lane_line,
                                             const Eigen::Vector2d& p0,
                                             const Eigen::Vector2d& p1,
                                             const size_t size) {
  if (location_right_id_.size() >= size) {
    auto lane_id = location_right_id_[size - 1];
    AppendLaneLineRight(lane_id, lane_line, p0, p1);
  } else {
    if (location_right_id_next_.size() >= size) {
      auto lane_id = location_right_id_next_[size - 1];
      AppendLaneLineRight(lane_id, lane_line, p0, p1);
    }
    if (location_right_id_next_.size() >= size + 1) {
      auto lane_id = location_right_id_next_[size];
      AppendLaneLineLeft(lane_id, lane_line, p0, p1);
    }
  }
  if (location_right_id_.size() >= size + 1) {
    auto lane_id = location_right_id_[size];
    AppendLaneLineLeft(lane_id, lane_line, p0, p1);
  }
}

void TopoAssignment::AppendLaneLineCaseLeftRight(
    hozon::mp::mf::LaneLine* lane_line, const Eigen::Vector2d& p0,
    const Eigen::Vector2d& p1, const size_t size) {
  if (location_left_id_.size() >= size) {
    auto lane_id = location_left_id_[size - 1];
    AppendLaneLineRight(lane_id, lane_line, p0, p1);
  } else {
    if (location_left_id_next_.size() >= size) {
      auto lane_id = location_left_id_next_[size - 1];
      AppendLaneLineRight(lane_id, lane_line, p0, p1);
    }
    if (location_right_id_next_.size() >= size) {
      auto lane_id = location_right_id_next_[size - 1];
      AppendLaneLineLeft(lane_id, lane_line, p0, p1);
    }
  }
  if (location_right_id_.size() >= size) {
    auto lane_id = location_right_id_[size - 1];
    AppendLaneLineLeft(lane_id, lane_line, p0, p1);
  }
}

void TopoAssignment::AppendLaneLineLeft(const std::string& lane_id,
                                        hozon::mp::mf::LaneLine* lane_line,
                                        const Eigen::Vector2d& p0,
                                        const Eigen::Vector2d& p1) {
  if (all_lane_info_.find(lane_id) == all_lane_info_.end()) {
    return;
  }

  std::vector<Eigen::Vector2d> hq_lane_points;
  hq_lane_points = all_lane_info_[lane_id].left_start_end;

  if (hq_lane_points.empty()) {
    return;
  }

  bool flag_p0 = false;
  bool flag_p1 = false;
  bool flag_pt = false;
  double distance_p0 = 0.;
  double distance_p1 = 0.;

  flag_p0 =
      PerpendicularFootInSegment(hq_lane_points[0], hq_lane_points[1], p0);
  flag_p1 =
      PerpendicularFootInSegment(hq_lane_points[0], hq_lane_points[1], p1);
  flag_pt = PerpendicularFootInSegment(p0, p1, hq_lane_points[0]);
  distance_p0 = (hq_lane_points[0] - p0).norm();
  distance_p1 = (hq_lane_points[0] - p1).norm();

  if (!flag_p0 && !flag_p1 && !flag_pt) {
    AppendLaneLineLeftOutside(lane_id, lane_line, p0, p1, distance_p0,
                              distance_p1);
  } else {
    std::vector<std::string> lane_id_p0 = FindLaneLineHead(lane_id, p0, true);
    lane_line->left_lanes = lane_id_p0;
    std::reverse(lane_line->left_lanes.begin(), lane_line->left_lanes.end());

    lane_line->left_lanes.pop_back();

    std::vector<std::string> lane_id_p1 = FindLaneLineTail(lane_id, p1, true);
    lane_line->left_lanes.insert(lane_line->left_lanes.end(),
                                 lane_id_p1.begin(), lane_id_p1.end());

    if (lane_line->left_lanes.empty()) {
      return;
    }

    if (LaneLineBelongToLane(lane_line->left_lanes, true, p0, p1)) {
      lane_line->left_lanes.clear();
    }
  }
}

void TopoAssignment::AppendLaneLineLeftOutside(
    const std::string& lane_id, hozon::mp::mf::LaneLine* lane_line,
    const Eigen::Vector2d& p0, const Eigen::Vector2d& p1,
    const double distance_p0, const double distance_p1) {
  if (distance_p0 >= distance_p1) {
    std::vector<std::string> head_lane_id_p0 =
        FindLaneLineHead(lane_id, p0, true);
    std::vector<std::string> head_lane_id_p1 =
        FindLaneLineHead(lane_id, p1, true);
    if (head_lane_id_p1.empty()) {
      return;
    }
    auto size = head_lane_id_p1.size() - 1;
    for (size_t i = size; i < head_lane_id_p0.size(); ++i) {
      lane_line->left_lanes.emplace_back(head_lane_id_p0[i]);
    }
    std::reverse(lane_line->left_lanes.begin(), lane_line->left_lanes.end());
    if (lane_line->left_lanes.empty()) {
      return;
    }

    if (LaneLineBelongToLane(lane_line->left_lanes, true, p0, p1)) {
      lane_line->left_lanes.clear();
    }

  } else {
    std::vector<std::string> tail_lane_id_p0 =
        FindLaneLineTail(lane_id, p1, true);
    std::vector<std::string> tail_lane_id_p1 =
        FindLaneLineTail(lane_id, p0, true);
    if (tail_lane_id_p0.empty()) {
      return;
    }
    auto size = tail_lane_id_p0.size() - 1;
    for (size_t i = size; i < tail_lane_id_p1.size(); ++i) {
      lane_line->left_lanes.emplace_back(tail_lane_id_p1[i]);
    }
    if (lane_line->left_lanes.empty()) {
      return;
    }

    if (LaneLineBelongToLane(lane_line->left_lanes, true, p0, p1)) {
      lane_line->left_lanes.clear();
    }
  }
}

void TopoAssignment::AppendLaneLineRight(const std::string& lane_id,
                                         hozon::mp::mf::LaneLine* lane_line,
                                         const Eigen::Vector2d& p0,
                                         const Eigen::Vector2d& p1) {
  if (all_lane_info_.find(lane_id) == all_lane_info_.end()) {
    return;
  }

  std::vector<Eigen::Vector2d> hq_lane_points;
  hq_lane_points = all_lane_info_[lane_id].right_start_end;

  if (hq_lane_points.empty()) {
    return;
  }

  bool flag_p0 = false;
  bool flag_p1 = false;
  bool flag_pt = false;
  double distance_p0 = 0.;
  double distance_p1 = 0.;

  flag_p0 =
      PerpendicularFootInSegment(hq_lane_points[0], hq_lane_points[1], p0);
  flag_p1 =
      PerpendicularFootInSegment(hq_lane_points[0], hq_lane_points[1], p1);
  flag_pt = PerpendicularFootInSegment(p0, p1, hq_lane_points[0]);
  distance_p0 = (hq_lane_points[0] - p0).norm();
  distance_p1 = (hq_lane_points[0] - p1).norm();

  if (!flag_p0 && !flag_p1 && !flag_pt) {
    AppendLaneLineRightOutside(lane_id, lane_line, p0, p1, distance_p0,
                               distance_p1);
  } else {
    std::vector<std::string> lane_id_p0 = FindLaneLineHead(lane_id, p0, false);
    lane_line->right_lanes = lane_id_p0;
    std::reverse(lane_line->right_lanes.begin(), lane_line->right_lanes.end());

    lane_line->right_lanes.pop_back();

    std::vector<std::string> lane_id_p1 = FindLaneLineTail(lane_id, p1, false);
    lane_line->right_lanes.insert(lane_line->right_lanes.end(),
                                  lane_id_p1.begin(), lane_id_p1.end());

    if (lane_line->right_lanes.empty()) {
      return;
    }

    if (LaneLineBelongToLane(lane_line->right_lanes, false, p0, p1)) {
      lane_line->right_lanes.clear();
    }
  }
}

void TopoAssignment::AppendLaneLineRightOutside(
    const std::string& lane_id, hozon::mp::mf::LaneLine* lane_line,
    const Eigen::Vector2d& p0, const Eigen::Vector2d& p1,
    const double distance_p0, const double distance_p1) {
  if (distance_p0 >= distance_p1) {
    std::vector<std::string> head_lane_id_p0 =
        FindLaneLineHead(lane_id, p0, false);
    std::vector<std::string> head_lane_id_p1 =
        FindLaneLineHead(lane_id, p1, false);
    if (head_lane_id_p1.empty()) {
      return;
    }
    auto size = head_lane_id_p1.size() - 1;
    for (size_t i = size; i < head_lane_id_p0.size(); ++i) {
      lane_line->right_lanes.emplace_back(head_lane_id_p0[i]);
    }
    std::reverse(lane_line->right_lanes.begin(), lane_line->right_lanes.end());
    if (lane_line->right_lanes.empty()) {
      return;
    }

    if (LaneLineBelongToLane(lane_line->right_lanes, false, p0, p1)) {
      lane_line->right_lanes.clear();
    }

  } else {
    std::vector<std::string> tail_lane_id_p0 =
        FindLaneLineTail(lane_id, p1, false);
    std::vector<std::string> tail_lane_id_p1 =
        FindLaneLineTail(lane_id, p0, false);
    if (tail_lane_id_p0.empty()) {
      return;
    }
    auto size = tail_lane_id_p0.size() - 1;
    for (size_t i = size; i < tail_lane_id_p1.size(); ++i) {
      lane_line->right_lanes.emplace_back(tail_lane_id_p1[i]);
    }
    if (lane_line->right_lanes.empty()) {
      return;
    }

    if (LaneLineBelongToLane(lane_line->right_lanes, false, p0, p1)) {
      lane_line->right_lanes.clear();
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
    std::pair<bool, std::map<std::string, Lane>>* all_lanes) {
  for (const auto& line_it : all_lanelines) {
    for (const auto& left_lane_it : line_it.second.left_lanes) {
      if (all_lanes->second.find(left_lane_it) == all_lanes->second.end()) {
        Lane lane;
        lane.id = left_lane_it;
        lane.left_lines.emplace_back(line_it.second.id);
        (*all_lanes).second[left_lane_it] = lane;
      } else {
        (*all_lanes)
            .second[left_lane_it]
            .left_lines.emplace_back(line_it.second.id);
      }
    }
    for (const auto& right_lane_it : line_it.second.right_lanes) {
      if (all_lanes->second.find(right_lane_it) == all_lanes->second.end()) {
        Lane lane;
        lane.id = right_lane_it;
        lane.right_lines.emplace_back(line_it.second.id);
        (*all_lanes).second[right_lane_it] = lane;
      } else {
        (*all_lanes)
            .second[right_lane_it]
            .right_lines.emplace_back(line_it.second.id);
      }
    }
  }

  AppendLaneLanes(all_lanes);
}

void TopoAssignment::AppendLaneLanes(
    std::pair<bool, std::map<std::string, Lane>>* all_lanes) {
  // 再构造前后左右
  for (auto& lane_it : (*all_lanes).second) {
    const std::string lane_id = lane_it.second.id;
    if (all_lane_info_.find(lane_id) == all_lane_info_.end()) {
      continue;
    }
    lane_it.second.extra_boundary = all_lane_info_[lane_id].extra_boundary;
    const auto& left_ids = all_lane_info_[lane_id].left_lane_ids;

    for (const auto& left_lane_it : left_ids) {
      auto left_lane_it_id = left_lane_it;
      if (all_lanes->second.find(left_lane_it_id) != all_lanes->second.end()) {
        lane_it.second.left_lanes.emplace_back(left_lane_it_id);
      }
    }
    const auto& right_ids = all_lane_info_[lane_id].right_lane_ids;
    for (const auto& right_lane_it : right_ids) {
      auto right_lane_it_id = right_lane_it;
      if (all_lanes->second.find(right_lane_it_id) != all_lanes->second.end()) {
        lane_it.second.right_lanes.emplace_back(right_lane_it_id);
      }
    }
    const auto& prev_ids = all_lane_info_[lane_id].prev_lane_ids;
    for (const auto& prev_lane_it : prev_ids) {
      auto prev_lane_it_id = prev_lane_it;
      if (all_lanes->second.find(prev_lane_it_id) != all_lanes->second.end()) {
        lane_it.second.prev_lanes.emplace_back(prev_lane_it_id);
      }
    }
    const auto& next_ids = all_lane_info_[lane_id].next_lane_ids;
    for (const auto& next_lane_it : next_ids) {
      auto next_lane_it_id = next_lane_it;
      if (all_lanes->second.find(next_lane_it_id) != all_lanes->second.end()) {
        lane_it.second.next_lanes.emplace_back(next_lane_it_id);
      }
    }
  }
  AppendLaneLanesSplit(all_lanes);
}

void TopoAssignment::AppendLaneLanesSplit(
    std::pair<bool, std::map<std::string, Lane>>* all_lanes) {
  // 判断split case
  for (auto& lane_it : (*all_lanes).second) {
    if (lane_it.second.extra_boundary > 0) {
      (*all_lanes).first = true;
      break;
    }
  }
}

void TopoAssignment::AppendTopoMapGeometry(
    std::pair<bool, std::map<std::string, Lane>>* all_lanes,
    const std::shared_ptr<hozon::hdmap::Map>& topo_map_geo) {
  for (auto lane_it = all_lanes->second.begin();
       lane_it != all_lanes->second.end();) {
    const std::string lane_id = lane_it->second.id;
    if (all_lane_info_.find(lane_id) == all_lane_info_.end()) {
      continue;
    }

    if (lane_it->second.left_lines.empty() &&
        lane_it->second.right_lines.empty()) {
      continue;
    }

    // 后续需要该一下改成map里面有点再加入
    auto* lane = topo_map_geo->add_lane();
    lane->mutable_id()->set_id(lane_id);
    // 更新topo map的左右车道线
    auto hq_lane_left_points = all_lane_info_[lane_id].left_start_end;
    auto hq_lane_right_points = all_lane_info_[lane_id].right_start_end;
    auto hq_lane_left_points_size = hq_lane_left_points.size();
    auto hq_lane_right_points_size = hq_lane_right_points.size();

    AppendTopoMapLeftLanes(lane_it->second, lane, hq_lane_left_points,
                           hq_lane_left_points_size, all_lanes->first);
    AppendTopoMapRightLanes(lane_it->second, lane, hq_lane_right_points,
                            hq_lane_right_points_size, all_lanes->first);

    if (lane->left_boundary().curve().segment().empty() &&
        lane->right_boundary().curve().segment().empty()) {
      topo_map_geo->mutable_lane()->RemoveLast();
      lane_it = all_lanes->second.erase(lane_it);
    } else {
      ++lane_it;
    }
  }
}

void TopoAssignment::AppendTopoMapTopology(
    const std::map<std::string, Lane>& all_lanes,
    const std::shared_ptr<hozon::hdmap::Map>& topo_map_geo,
    const std::shared_ptr<hozon::hdmap::Map>& topo_map) {
  for (const auto& lane_geo : topo_map_geo->lane()) {
    auto* lane = topo_map->add_lane();
    lane->CopyFrom(lane_geo);

    auto lane_it = all_lanes.find(lane_geo.id().id());
    if (lane_it == all_lanes.end()) {
      continue;
    }

    for (const auto& left_lane_it : lane_it->second.left_lanes) {
      if (all_lanes.find(left_lane_it) != all_lanes.end()) {
        lane->add_left_neighbor_forward_lane_id()->set_id(left_lane_it);
      }
    }
    for (const auto& right_lane_it : lane_it->second.right_lanes) {
      if (all_lanes.find(right_lane_it) != all_lanes.end()) {
        lane->add_right_neighbor_forward_lane_id()->set_id(right_lane_it);
      }
    }
    for (const auto& prev_lane_it : lane_it->second.prev_lanes) {
      if (all_lanes.find(prev_lane_it) != all_lanes.end()) {
        lane->add_predecessor_id()->set_id(prev_lane_it);
      }
    }
    for (const auto& next_lane_it : lane_it->second.next_lanes) {
      if (all_lanes.find(next_lane_it) != all_lanes.end()) {
        lane->add_successor_id()->set_id(next_lane_it);
      }
    }
  }
}

void TopoAssignment::AppendTopoMapElements(
    const std::shared_ptr<hozon::hdmap::Map>& topo_map,
    const Eigen::Isometry3d& T_U_V) {
  // stop lines
  for (const auto& stop_line_it : local_map_->stop_lines()) {
    auto* stop_lane = topo_map->add_stop_line();
    stop_lane->set_id(std::to_string(stop_line_it.track_id()));

    Eigen::Vector3d left_point;
    left_point << stop_line_it.left_point().x(), stop_line_it.left_point().y(),
        stop_line_it.left_point().z();
    left_point = T_U_V * left_point;

    Eigen::Vector3d right_point;
    right_point << stop_line_it.right_point().x(),
        stop_line_it.right_point().y(), stop_line_it.right_point().z();
    right_point = T_U_V * right_point;

    auto* point_left = stop_lane->mutable_shape()->add_point();
    point_left->set_x(left_point.x());
    point_left->set_y(left_point.y());
    point_left->set_z(left_point.z());

    auto* point_right = stop_lane->mutable_shape()->add_point();
    point_right->set_x(right_point.x());
    point_right->set_y(right_point.y());
    point_right->set_z(right_point.z());
  }
  // cross_walks
  for (const auto& cross_walk_it : local_map_->cross_walks()) {
    if (cross_walk_it.points().point_size() != 4) {
      continue;
    }
    auto* cross_walk = topo_map->add_crosswalk();
    cross_walk->mutable_id()->set_id(std::to_string(cross_walk_it.track_id()));

    for (const auto& point_it : cross_walk_it.points().point()) {
      Eigen::Vector3d point(point_it.x(), point_it.y(), point_it.z());
      point = T_U_V * point;
      auto* cross_walk_point = cross_walk->mutable_polygon()->add_point();
      cross_walk_point->set_x(point.x());
      cross_walk_point->set_y(point.y());
      cross_walk_point->set_z(point.z());
    }
  }
  // arrows
  for (const auto& arrow_it : local_map_->arrows()) {
    if (arrow_it.points().point_size() != 4) {
      continue;
    }
    auto* arrow = topo_map->add_arraw();
    arrow->set_id(std::to_string(arrow_it.track_id()));
    // arrow->set_type(arrow_it.arrow_type());

    for (const auto& point_it : arrow_it.points().point()) {
      Eigen::Vector3d point(point_it.x(), point_it.y(), point_it.z());
      point = T_U_V * point;
      auto* arrow_point = arrow->mutable_shape()->add_point();
      arrow_point->set_x(point.x());
      arrow_point->set_y(point.y());
      arrow_point->set_z(point.z());
    }

    auto center_point = GetIntersection(arrow_it.points());
    center_point = T_U_V * center_point;
    arrow->mutable_center_point()->set_x(center_point.x());
    arrow->mutable_center_point()->set_y(center_point.y());

    Eigen::Quaterniond quat_arrow_in_veh(
        Eigen::AngleAxisd(arrow_it.heading(), Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond quat_veh_to_local_enu(T_U_V.rotation());
    Eigen::Quaterniond quat_arrow_in_local_enu =
        quat_veh_to_local_enu * quat_arrow_in_veh;

    Eigen::Vector3d euler_arrow =
        quat_arrow_in_local_enu.toRotationMatrix().eulerAngles(2, 0, 1);

    arrow->set_heading(euler_arrow[0]);
  }
}

void TopoAssignment::AppendTopoMapLeftLanes(
    const hozon::mp::mf::Lane& lane_it, hozon::hdmap::Lane* lane,
    const std::vector<Eigen::Vector2d>& hq_lane_left_points, const size_t size,
    const bool split) {
  if (hq_lane_left_points.empty()) {
    return;
  }
  for (const auto& left_line_it : lane_it.left_lines) {
    // 打断给到lane_it 里面去
    bool flag_p0 = false;
    bool flag_p1 = false;
    bool flag_pt = false;
    if (all_lanelines_.find(left_line_it) != all_lanelines_.end()) {
      if (all_lanelines_[left_line_it].points.empty()) {
        continue;
      }

      auto start_point = all_lanelines_[left_line_it].points[0];
      const auto point_size =
          static_cast<int>(all_lanelines_[left_line_it].points.size());
      auto end_point = all_lanelines_[left_line_it].points[point_size - 1];

      Eigen::Vector2d p0(start_point.x(), start_point.y());
      Eigen::Vector2d p1(end_point.x(), end_point.y());

      flag_p0 = PerpendicularFootInSegment(hq_lane_left_points[0],
                                           hq_lane_left_points[size - 1], p0);
      flag_p1 = PerpendicularFootInSegment(hq_lane_left_points[0],
                                           hq_lane_left_points[size - 1], p1);
      flag_pt = PerpendicularFootInSegment(p0, p1, hq_lane_left_points[0]);

      double dist_p0 = PointDistanceToSegment(hq_lane_left_points, p0);
      double dist_p1 = PointDistanceToSegment(hq_lane_left_points, p1);

      flag_p0 = flag_p0 && (dist_p0 <= FLAGS_topo_lane_line_point_dist);
      flag_p1 = flag_p1 && (dist_p1 <= FLAGS_topo_lane_line_point_dist);

      if (flag_p0 && flag_p1) {
        // 不用找index
        AppendTopoMapLeftLanePoints(hq_lane_left_points, lane, 0,
                                    point_size - 1, left_line_it, split);
      }
      if (flag_p0 && !flag_p1) {
        // 找到后面一个点最近的index
        const int dim = 1;
        std::vector<float> query_points = std::vector<float>{
            static_cast<float>(hq_lane_left_points[size - 1].x()),
            static_cast<float>(hq_lane_left_points[size - 1].y())};
        const auto index = KnnSearchNearestPointIndex(
            dim, query_points, all_lanelines_[left_line_it].lane_line_kdtree);

        AppendTopoMapLeftLanePoints(hq_lane_left_points, lane, 0, index,
                                    left_line_it, split);
      }
      if (!flag_p0 && flag_p1) {
        // 找到前面一个点最近的index
        const int dim = 1;
        std::vector<float> query_points =
            std::vector<float>{static_cast<float>(hq_lane_left_points[0].x()),
                               static_cast<float>(hq_lane_left_points[0].y())};

        const auto index = KnnSearchNearestPointIndex(
            dim, query_points, all_lanelines_[left_line_it].lane_line_kdtree);
        const auto point_size =
            static_cast<int>(all_lanelines_[left_line_it].points.size());

        AppendTopoMapLeftLanePoints(hq_lane_left_points, lane, index,
                                    point_size - 1, left_line_it, split);
      }
      if (!flag_p0 && !flag_p1 && flag_pt) {
        // 找到两个点最近的index
        const int dim = 1;
        std::vector<float> query_points_start =
            std::vector<float>{static_cast<float>(hq_lane_left_points[0].x()),
                               static_cast<float>(hq_lane_left_points[0].y())};
        std::vector<float> query_points_end = std::vector<float>{
            static_cast<float>(hq_lane_left_points[size - 1].x()),
            static_cast<float>(hq_lane_left_points[size - 1].y())};
        const auto start_index = KnnSearchNearestPointIndex(
            dim, query_points_start,
            all_lanelines_[left_line_it].lane_line_kdtree);
        const auto end_index = KnnSearchNearestPointIndex(
            dim, query_points_end,
            all_lanelines_[left_line_it].lane_line_kdtree);

        AppendTopoMapLeftLanePoints(hq_lane_left_points, lane, start_index,
                                    end_index, left_line_it, split);
      }
    }
  }
}

void TopoAssignment::AppendTopoMapRightLanes(
    const hozon::mp::mf::Lane& lane_it, hozon::hdmap::Lane* lane,
    const std::vector<Eigen::Vector2d>& hq_lane_right_points, const size_t size,
    const bool split) {
  if (hq_lane_right_points.empty()) {
    return;
  }
  for (const auto& right_line_it : lane_it.right_lines) {
    // 打断给到lant_it 里面去
    bool flag_p0 = false;
    bool flag_p1 = false;
    bool flag_pt = false;
    if (all_lanelines_.find(right_line_it) != all_lanelines_.end()) {
      if (all_lanelines_[right_line_it].points.empty()) {
        continue;
      }

      auto start_point = all_lanelines_[right_line_it].points[0];
      const auto point_size =
          static_cast<int>(all_lanelines_[right_line_it].points.size());
      auto end_point = all_lanelines_[right_line_it].points[point_size - 1];

      Eigen::Vector2d p0(start_point.x(), start_point.y());
      Eigen::Vector2d p1(end_point.x(), end_point.y());

      flag_p0 = PerpendicularFootInSegment(hq_lane_right_points[0],
                                           hq_lane_right_points[size - 1], p0);
      flag_p1 = PerpendicularFootInSegment(hq_lane_right_points[0],
                                           hq_lane_right_points[size - 1], p1);
      flag_pt = PerpendicularFootInSegment(p0, p1, hq_lane_right_points[0]);

      double dist_p0 = PointDistanceToSegment(hq_lane_right_points, p0);
      double dist_p1 = PointDistanceToSegment(hq_lane_right_points, p1);

      flag_p0 = flag_p0 && (dist_p0 <= FLAGS_topo_lane_line_point_dist);
      flag_p1 = flag_p1 && (dist_p1 <= FLAGS_topo_lane_line_point_dist);

      if (flag_p0 && flag_p1) {
        // 不用找index
        AppendTopoMapRightLanePoints(hq_lane_right_points, lane, 0,
                                     point_size - 1, right_line_it, split);
      }
      if (flag_p0 && !flag_p1) {
        // 找到后面一个点最近的index
        const int dim = 1;
        std::vector<float> query_points = std::vector<float>{
            static_cast<float>(hq_lane_right_points[size - 1].x()),
            static_cast<float>(hq_lane_right_points[size - 1].y())};
        const auto index = KnnSearchNearestPointIndex(
            dim, query_points, all_lanelines_[right_line_it].lane_line_kdtree);

        AppendTopoMapRightLanePoints(hq_lane_right_points, lane, 0, index,
                                     right_line_it, split);
      }
      if (!flag_p0 && flag_p1) {
        // 找到前面一个点最近的index
        const int dim = 1;
        std::vector<float> query_points =
            std::vector<float>{static_cast<float>(hq_lane_right_points[0].x()),
                               static_cast<float>(hq_lane_right_points[0].y())};
        const auto index = KnnSearchNearestPointIndex(
            dim, query_points, all_lanelines_[right_line_it].lane_line_kdtree);
        const auto point_size =
            static_cast<int>(all_lanelines_[right_line_it].points.size());

        AppendTopoMapRightLanePoints(hq_lane_right_points, lane, index,
                                     point_size - 1, right_line_it, split);
      }
      if (!flag_p0 && !flag_p1 && flag_pt) {
        // 找到两个点最近的index
        const int dim = 1;
        std::vector<float> query_points_start =
            std::vector<float>{static_cast<float>(hq_lane_right_points[0].x()),
                               static_cast<float>(hq_lane_right_points[0].y())};
        std::vector<float> query_points_end = std::vector<float>{
            static_cast<float>(hq_lane_right_points[size - 1].x()),
            static_cast<float>(hq_lane_right_points[size - 1].y())};
        const auto start_index = KnnSearchNearestPointIndex(
            dim, query_points_start,
            all_lanelines_[right_line_it].lane_line_kdtree);
        const auto end_index = KnnSearchNearestPointIndex(
            dim, query_points_end,
            all_lanelines_[right_line_it].lane_line_kdtree);

        AppendTopoMapRightLanePoints(hq_lane_right_points, lane, start_index,
                                     end_index, right_line_it, split);
      }
    }
  }
}

void TopoAssignment::AppendTopoMapLeftLanePoints(
    const std::vector<Eigen::Vector2d>& lane_points, hozon::hdmap::Lane* lane,
    const int start_index, const int end_index, const int track_id,
    const bool split) {
  auto start_point = all_lanelines_[track_id].points[start_index];
  auto end_point = all_lanelines_[track_id].points[end_index];

  Eigen::Vector2d p0(start_point.x(), start_point.y());
  Eigen::Vector2d p1(end_point.x(), end_point.y());
  if (LaneBelongToLaneLine(lane_points, p0, p1) && split) {
    return;
  }
  auto* segment = lane->mutable_left_boundary()->mutable_curve()->add_segment();
  for (int i = start_index; i <= end_index; ++i) {
    auto* topo_point = segment->mutable_line_segment()->add_point();
    topo_point->set_x(all_lanelines_[track_id].points[i].x());
    topo_point->set_y(all_lanelines_[track_id].points[i].y());
    topo_point->set_z(all_lanelines_[track_id].points[i].z());
  }
}

void TopoAssignment::AppendTopoMapRightLanePoints(
    const std::vector<Eigen::Vector2d>& lane_points, hozon::hdmap::Lane* lane,
    const int start_index, const int end_index, const int track_id,
    const bool split) {
  auto start_point = all_lanelines_[track_id].points[start_index];
  auto end_point = all_lanelines_[track_id].points[end_index];

  Eigen::Vector2d p0(start_point.x(), start_point.y());
  Eigen::Vector2d p1(end_point.x(), end_point.y());
  if (LaneBelongToLaneLine(lane_points, p0, p1) && split) {
    return;
  }
  auto* segment =
      lane->mutable_right_boundary()->mutable_curve()->add_segment();
  for (int i = start_index; i <= end_index; ++i) {
    auto* topo_point = segment->mutable_line_segment()->add_point();
    topo_point->set_x(all_lanelines_[track_id].points[i].x());
    topo_point->set_y(all_lanelines_[track_id].points[i].y());
    topo_point->set_z(all_lanelines_[track_id].points[i].z());
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

  RVIZ_AGENT.Publish(KTopicTopoAssignLocalMap, markers);
}

void TopoAssignment::VizLocation(const Eigen::Vector3d& pose,
                                 const Eigen::Quaterniond& q_W_V,
                                 const double stamp) {
  if (RVIZ_AGENT.Ok()) {
    static uint32_t seq = 0;
    uint32_t curr_seq = seq++;
    adsfi_proto::viz::TransformStamped geo_tf;
    geo_tf.mutable_header()->set_seq(curr_seq);
    auto time_sec = static_cast<uint32_t>(stamp);
    auto time_nsec = static_cast<uint32_t>((stamp - time_sec) * 1e9);
    geo_tf.mutable_header()->mutable_timestamp()->set_sec(time_sec);
    geo_tf.mutable_header()->mutable_timestamp()->set_nsec(time_nsec);
    geo_tf.mutable_header()->set_frameid("map");
    geo_tf.set_child_frame_id("vehicle");
    geo_tf.mutable_transform()->mutable_translation()->set_x(pose.x());
    geo_tf.mutable_transform()->mutable_translation()->set_y(pose.y());
    geo_tf.mutable_transform()->mutable_translation()->set_z(pose.z());
    geo_tf.mutable_transform()->mutable_rotation()->set_x(q_W_V.x());
    geo_tf.mutable_transform()->mutable_rotation()->set_y(q_W_V.y());
    geo_tf.mutable_transform()->mutable_rotation()->set_z(q_W_V.z());
    geo_tf.mutable_transform()->mutable_rotation()->set_w(q_W_V.w());
    RVIZ_AGENT.Publish(kTopicTopoAssignTf, geo_tf);

    auto* location_pose = location_path_.add_poses();

    location_path_.mutable_header()->set_seq(curr_seq);
    location_path_.mutable_header()->mutable_timestamp()->set_sec(
        static_cast<uint32_t>(stamp));
    location_path_.mutable_header()->set_frameid("map");

    location_pose->mutable_header()->set_seq(curr_seq);
    location_pose->mutable_header()->mutable_timestamp()->set_sec(
        static_cast<uint32_t>(stamp));
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
    RVIZ_AGENT.Publish(kTopicTopoAssignLocation, location_path_);
  }
}

void TopoAssignment::VizHQMap(const std::shared_ptr<hozon::hdmap::Map>& msg) {
  if (!RVIZ_AGENT.Ok()) {
    return;
  }
  adsfi_proto::viz::MarkerArray markers_road;

  VizHQMapRoad(msg, &markers_road);

  RVIZ_AGENT.Publish(KTopicTopoAssignHQMapRoad, markers_road);

  adsfi_proto::viz::MarkerArray markers_lane;

  VizHQMapLane(msg, &markers_lane);

  RVIZ_AGENT.Publish(KTopicTopoAssignHQMapLane, markers_lane);
}

void TopoAssignment::VizHQMapRoad(const std::shared_ptr<hozon::hdmap::Map>& msg,
                                  adsfi_proto::viz::MarkerArray* markers_road) {
  for (const auto& hq_road : msg->road()) {
    for (const auto& hq_road_section : hq_road.section()) {
      for (const auto& hq_road_section_edge :
           hq_road_section.boundary().outer_polygon().edge()) {
        for (const auto& edge : hq_road_section_edge.curve().segment()) {
          std::vector<Eigen::Vector3d> boundary_points;
          for (const auto& point : edge.line_segment().original_point()) {
            // 这里点的坐标是在gcj坐标系下，需要将其转到enu坐标系下
            Eigen::Vector3d point_enu = GcjPtToLocalEnu(point);
            boundary_points.emplace_back(point_enu);
          }
          adsfi_proto::viz::Marker marker;
          PointsToMarker(cur_timestamp_, boundary_points, &marker, 0.9);
          if (marker.points().size() >= 2) {
            markers_road->add_markers()->CopyFrom(marker);
          }
        }
      }
    }
  }
}

void TopoAssignment::VizHQMapLane(const std::shared_ptr<hozon::hdmap::Map>& msg,
                                  adsfi_proto::viz::MarkerArray* markers_lane) {
  for (const auto& hq_lane : msg->lane()) {
    for (const auto& left_points : hq_lane.left_boundary().curve().segment()) {
      std::vector<Eigen::Vector3d> lane_points;
      for (const auto& point : left_points.line_segment().original_point()) {
        // 这里点的坐标是在gcj坐标系下，需要将其转到enu坐标系下
        Eigen::Vector3d point_enu = GcjPtToLocalEnu(point);
        lane_points.emplace_back(point_enu);
      }
      adsfi_proto::viz::Marker marker;
      PointsToMarker(cur_timestamp_, lane_points, &marker, 0.5);
      if (marker.points().size() >= 2) {
        markers_lane->add_markers()->CopyFrom(marker);
      }
    }

    for (const auto& right_points :
         hq_lane.right_boundary().curve().segment()) {
      std::vector<Eigen::Vector3d> lane_points;
      for (const auto& point : right_points.line_segment().original_point()) {
        // 这里点的坐标是在gcj坐标系下，需要将其转到enu坐标系下
        Eigen::Vector3d point_enu = GcjPtToLocalEnu(point);
        lane_points.emplace_back(point_enu);
      }
      adsfi_proto::viz::Marker marker;
      PointsToMarker(cur_timestamp_, lane_points, &marker, 0.5);
      if (marker.points().size() >= 2) {
        markers_lane->add_markers()->CopyFrom(marker);
      }
    }
    if (!hq_lane.central_curve().segment().empty()) {
      auto point = hq_lane.central_curve()
                       .segment()[0]
                       .line_segment()
                       .original_point()[0];
      Eigen::Vector3d point_enu = GcjPtToLocalEnu(point);
      adsfi_proto::viz::Marker marker_id;
      auto id = hq_lane.id().id();
      LineIdToMarker(cur_timestamp_, point_enu, id, &marker_id);
      markers_lane->add_markers()->CopyFrom(marker_id);
    }
  }
}

Eigen::Vector3d TopoAssignment::GcjPtToLocalEnu(
    const hozon::common::PointENU& point_gcj) {
  Eigen::Vector3d pt_gcj(point_gcj.y(), point_gcj.x(), 0);
  Eigen::Vector3d point_enu = util::Geo::Gcj02ToEnu(pt_gcj, ref_point_);
  return point_enu;
}

void TopoAssignment::VizHQMap() {
  if (FLAGS_topo_rviz && RVIZ_AGENT.Ok()) {
    std::shared_ptr<hozon::hdmap::Map> hq_map =
        std::make_shared<hozon::hdmap::Map>();
    GLOBAL_HD_MAP->GetMap(hq_map.get());
    std::vector<adsfi_proto::viz::MarkerArray> result =
        MapProtoMarker::LaneToMarker(hq_map, ref_point_, true);
    // adsfi_proto::viz::MarkerArray lane = result[0];
    adsfi_proto::viz::MarkerArray left = result[1];
    adsfi_proto::viz::MarkerArray right = result[2];

    // RVIZ_AGENT.Publish("lane", lane);
    RVIZ_AGENT.Publish("left", left);
    RVIZ_AGENT.Publish("right", right);
  }
}

void TopoAssignment::VizTopoMap(const std::shared_ptr<hozon::hdmap::Map>& msg) {
  if (!RVIZ_AGENT.Ok()) {
    return;
  }
  adsfi_proto::viz::MarkerArray markers_road;

  VizTopoMapRoad(msg, &markers_road);

  RVIZ_AGENT.Publish(KTopicTopoAssignTopoMapRoad, markers_road);

  adsfi_proto::viz::MarkerArray markers_lane;

  VizTopoMapLane(msg, &markers_lane);

  RVIZ_AGENT.Publish(KTopicTopoAssignTopoMapLane, markers_lane);

  adsfi_proto::viz::MarkerArray markers_elements;

  VizTopoMapElements(msg, &markers_elements);

  RVIZ_AGENT.Publish(KTopicTopoAssignTopoMapElements, markers_elements);
}

void TopoAssignment::VizTopoMapRoad(
    const std::shared_ptr<hozon::hdmap::Map>& msg,
    adsfi_proto::viz::MarkerArray* markers_road) const {
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
            markers_road->add_markers()->CopyFrom(marker);
          }
        }
      }
    }
  }
}

void TopoAssignment::VizTopoMapLane(
    const std::shared_ptr<hozon::hdmap::Map>& msg,
    adsfi_proto::viz::MarkerArray* markers_lane) const {
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
        markers_lane->add_markers()->CopyFrom(marker);
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
        markers_lane->add_markers()->CopyFrom(marker);
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
        markers_lane->add_markers()->CopyFrom(marker_id);
      }
    }
  }
}

void TopoAssignment::VizTopoMapElements(
    const std::shared_ptr<hozon::hdmap::Map>& msg,
    adsfi_proto::viz::MarkerArray* markers_elements) const {
  static int id_stop_line = 0;
  for (const auto& stop_line : msg->stop_line()) {
    adsfi_proto::viz::Marker marker;
    marker.mutable_header()->set_frameid("map");
    marker.mutable_header()->mutable_timestamp()->set_sec(
        static_cast<uint32_t>(cur_timestamp_));
    marker.mutable_header()->mutable_timestamp()->set_nsec(
        static_cast<uint32_t>(cur_timestamp_));
    marker.set_id(id_stop_line++);
    marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
    marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
    marker.mutable_pose()->mutable_position()->set_x(0);
    marker.mutable_pose()->mutable_position()->set_y(0);
    marker.mutable_pose()->mutable_position()->set_z(0);
    marker.mutable_pose()->mutable_orientation()->set_x(0.);
    marker.mutable_pose()->mutable_orientation()->set_y(0.);
    marker.mutable_pose()->mutable_orientation()->set_z(0.);
    marker.mutable_pose()->mutable_orientation()->set_w(1.);
    marker.mutable_scale()->set_x(0.2);
    marker.mutable_scale()->set_y(0.2);
    marker.mutable_scale()->set_z(0.2);
    marker.mutable_lifetime()->set_sec(0);
    marker.mutable_lifetime()->set_nsec(200000000);
    adsfi_proto::viz::ColorRGBA color;
    color.set_a(1.0);
    color.set_r(1.0);
    color.set_g(1.0);
    color.set_b(1.0);
    marker.mutable_color()->CopyFrom(color);
    if (stop_line.shape().point_size() != 2) {
      continue;
    }
    auto point = stop_line.shape().point(0);
    auto* left_point = marker.add_points();
    left_point->set_x(static_cast<float>(point.x()));
    left_point->set_y(static_cast<float>(point.y()));
    left_point->set_z(static_cast<float>(point.z()));
    point = stop_line.shape().point(1);
    auto* right_point = marker.add_points();
    right_point->set_x(static_cast<float>(point.x()));
    right_point->set_y(static_cast<float>(point.y()));
    right_point->set_z(static_cast<float>(point.z()));
    markers_elements->add_markers()->CopyFrom(marker);
  }

  static int id_arrow = 0;
  for (const auto& arrow : msg->arraw()) {
    adsfi_proto::viz::Marker point_marker;
    point_marker.mutable_header()->set_frameid("map");
    point_marker.mutable_header()->mutable_timestamp()->set_sec(
        static_cast<uint32_t>(cur_timestamp_));
    point_marker.mutable_header()->mutable_timestamp()->set_nsec(
        static_cast<uint32_t>(cur_timestamp_));
    point_marker.set_id(id_arrow);
    point_marker.set_ns("points_arrow");
    point_marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
    point_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
    point_marker.mutable_pose()->mutable_position()->set_x(0);
    point_marker.mutable_pose()->mutable_position()->set_y(0);
    point_marker.mutable_pose()->mutable_position()->set_z(0);
    point_marker.mutable_pose()->mutable_orientation()->set_x(0.);
    point_marker.mutable_pose()->mutable_orientation()->set_y(0.);
    point_marker.mutable_pose()->mutable_orientation()->set_z(0.);
    point_marker.mutable_pose()->mutable_orientation()->set_w(1.);
    point_marker.mutable_scale()->set_x(0.2);
    point_marker.mutable_scale()->set_y(0.2);
    point_marker.mutable_scale()->set_z(0.2);
    point_marker.mutable_lifetime()->set_sec(0);
    point_marker.mutable_lifetime()->set_nsec(200000000);
    adsfi_proto::viz::ColorRGBA color;
    color.set_a(1.0);
    color.set_r(1.0);
    color.set_g(1.0);
    color.set_b(1.0);
    point_marker.mutable_color()->CopyFrom(color);
    if (arrow.shape().point_size() != 4) {
      continue;
    }
    Eigen::Vector3d point_0(arrow.shape().point(0).x(),
                            arrow.shape().point(0).y(), 0);
    Eigen::Vector3d point_1(arrow.shape().point(1).x(),
                            arrow.shape().point(1).y(), 0);
    Eigen::Vector3d point_2(arrow.shape().point(2).x(),
                            arrow.shape().point(2).y(), 0);
    Eigen::Vector3d point_3(arrow.shape().point(3).x(),
                            arrow.shape().point(3).y(), 0);
    auto* point_msg = point_marker.add_points();
    point_msg->set_x(point_0.x());
    point_msg->set_y(point_0.y());
    point_msg = point_marker.add_points();
    point_msg->set_x(point_1.x());
    point_msg->set_y(point_1.y());
    point_msg = point_marker.add_points();
    point_msg->set_x(point_2.x());
    point_msg->set_y(point_2.y());
    point_msg = point_marker.add_points();
    point_msg->set_x(point_3.x());
    point_msg->set_y(point_3.y());
    point_msg = point_marker.add_points();
    point_msg->set_x(point_0.x());
    point_msg->set_y(point_0.y());
    markers_elements->add_markers()->CopyFrom(point_marker);

    adsfi_proto::viz::Marker heading_marker;
    heading_marker.mutable_header()->set_frameid("map");
    heading_marker.mutable_header()->mutable_timestamp()->set_sec(
        static_cast<uint32_t>(cur_timestamp_));
    heading_marker.mutable_header()->mutable_timestamp()->set_nsec(
        static_cast<uint32_t>(cur_timestamp_));
    heading_marker.set_id(id_arrow++);
    heading_marker.set_ns("heading_arrow");
    heading_marker.set_type(adsfi_proto::viz::MarkerType::ARROW);
    heading_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
    heading_marker.mutable_pose()->mutable_position()->set_x(
        arrow.center_point().x());
    heading_marker.mutable_pose()->mutable_position()->set_y(
        arrow.center_point().y());
    heading_marker.mutable_pose()->mutable_position()->set_z(0);
    Eigen::Quaterniond quat_arrow(
        Eigen::AngleAxisd(arrow.heading(), Eigen::Vector3d::UnitZ()));
    heading_marker.mutable_pose()->mutable_orientation()->set_x(quat_arrow.x());
    heading_marker.mutable_pose()->mutable_orientation()->set_y(quat_arrow.y());
    heading_marker.mutable_pose()->mutable_orientation()->set_z(quat_arrow.z());
    heading_marker.mutable_pose()->mutable_orientation()->set_w(quat_arrow.w());
    heading_marker.mutable_scale()->set_x(2);
    heading_marker.mutable_scale()->set_y(0.1);
    heading_marker.mutable_scale()->set_z(0.1);
    heading_marker.mutable_lifetime()->set_sec(0);
    heading_marker.mutable_lifetime()->set_nsec(200000000);
    adsfi_proto::viz::ColorRGBA color_heading;
    color_heading.set_a(1.0);
    color_heading.set_r(1.0);
    color_heading.set_g(1.0);
    color_heading.set_b(1.0);
    heading_marker.mutable_color()->CopyFrom(color_heading);
    markers_elements->add_markers()->CopyFrom(heading_marker);
  }
}

void TopoAssignment::PointsToMarker(const double stamp,
                                    const std::vector<Eigen::Vector3d>& points,
                                    adsfi_proto::viz::Marker* marker,
                                    double color_type) {
  static int id = 0;
  marker->Clear();
  marker->mutable_header()->set_frameid("map");
  marker->mutable_header()->mutable_timestamp()->set_sec(
      static_cast<uint32_t>(stamp));
  marker->mutable_header()->mutable_timestamp()->set_nsec(
      static_cast<uint32_t>(stamp));
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
  color.set_g(static_cast<float>(color_type));
  color.set_b(0);

  marker->mutable_color()->CopyFrom(color);
  for (const auto& point : points) {
    auto* pt = marker->add_points();
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
  marker->mutable_header()->mutable_timestamp()->set_sec(
      static_cast<uint32_t>(stamp));
  marker->mutable_header()->mutable_timestamp()->set_nsec(
      static_cast<uint32_t>(stamp));
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

  auto* text = marker->mutable_text();
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

std::vector<Eigen::Vector2d> TopoAssignment::GetLaneLeftStartAndEndPoint(
    const hozon::hdmap::Lane& lane) {
  std::vector<Eigen::Vector2d> points;
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
      auto point_start_gcj =
          lane.left_boundary().curve().segment(0).line_segment().original_point(
              0);

      Eigen::Vector2d point_start;
      Eigen::Vector3d point_enu = GcjPtToLocalEnu(point_start_gcj);
      point_start.x() = point_enu.x();
      point_start.y() = point_enu.y();

      auto point_end_gcj = lane.left_boundary()
                               .curve()
                               .segment(segment_size - 1)
                               .line_segment()
                               .original_point(points_size_end - 1);

      Eigen::Vector2d point_end;
      point_enu = GcjPtToLocalEnu(point_end_gcj);
      point_end.x() = point_enu.x();
      point_end.y() = point_enu.y();

      points.emplace_back(point_start);
      points.emplace_back(point_end);
    }
  }
  return points;
}

std::vector<Eigen::Vector2d> TopoAssignment::GetLaneRightStartAndEndPoint(
    const hozon::hdmap::Lane& lane) {
  std::vector<Eigen::Vector2d> points;
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
      auto point_start_gcj = lane.right_boundary()
                                 .curve()
                                 .segment(0)
                                 .line_segment()
                                 .original_point(0);

      Eigen::Vector2d point_start;
      Eigen::Vector3d point_enu = GcjPtToLocalEnu(point_start_gcj);
      point_start.x() = point_enu.x();
      point_start.y() = point_enu.y();

      auto point_end_gcj = lane.right_boundary()
                               .curve()
                               .segment(segment_size - 1)
                               .line_segment()
                               .original_point(points_size_end - 1);

      Eigen::Vector2d point_end;
      point_enu = GcjPtToLocalEnu(point_end_gcj);
      point_end.x() = point_enu.x();
      point_end.y() = point_enu.y();

      points.emplace_back(point_start);
      points.emplace_back(point_end);
    }
  }
  return points;
}

size_t TopoAssignment::FindNearestPointIndex(
    const Eigen::Vector2d& point,
    const std::vector<hozon::common::Point3D>& points) {
  double miniDist = std::numeric_limits<double>::max();
  size_t index = 0;
  for (size_t i = 0; i < points.size(); ++i) {
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
    const std::shared_ptr<cv::flann::Index>& kd_tree) {
  std::vector<int> nearest_index(dim);
  std::vector<float> nearest_dist(dim);

  kd_tree->knnSearch(query_points, nearest_index, nearest_dist, dim,
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

  return dist_p0 >= FLAGS_topo_lane_line_dist_point &&
         dist_p1 >= FLAGS_topo_lane_line_dist_point;
}

void TopoAssignment::LaneLineBelongToLane(std::vector<std::string>* lanes,
                                          const bool left,
                                          const Eigen::Vector2d& p0,
                                          const Eigen::Vector2d& p1) {
  std::vector<Eigen::Vector2d> hq_lane_points;
  std::vector<Eigen::Vector2d> line_points{p0, p1};

  for (auto lane_it = (*lanes).begin(); lane_it != (*lanes).end();) {
    if (left) {
      hq_lane_points = all_lane_info_[*lane_it].left_start_end;
    } else {
      hq_lane_points = all_lane_info_[*lane_it].right_start_end;
    }
    if (hq_lane_points.empty()) {
      ++lane_it;
    } else {
      double dist_p0 = PointDistanceToSegment(line_points, hq_lane_points[0]);
      double dist_p1 = PointDistanceToSegment(line_points, hq_lane_points[1]);
      if (dist_p0 >= FLAGS_topo_lane_line_dist &&
          dist_p1 >= FLAGS_topo_lane_line_dist) {
        lane_it = (*lanes).erase(lane_it);
      } else {
        ++lane_it;
      }
    }
  }
}

bool TopoAssignment::LaneBelongToLaneLine(
    const std::vector<Eigen::Vector2d>& lane_points, const Eigen::Vector2d& p0,
    const Eigen::Vector2d& p1) {
  double dist_p0 = PointDistanceToSegment(lane_points, p0);
  double dist_p1 = PointDistanceToSegment(lane_points, p1);

  return dist_p0 >= FLAGS_topo_lane_line_dist ||
         dist_p1 >= FLAGS_topo_lane_line_dist;
}

Eigen::Vector3d TopoAssignment::GetIntersection(
    const hozon::common::Polygon& points) {
  Eigen::Vector3d point(0., 0., 0.);
  if (points.point_size() != 4) {
    return point;
  }
  double d1 = (points.point(2).x() - points.point(0).x()) *
                  (points.point(1).y() - points.point(0).y()) -
              (points.point(1).x() - points.point(0).x()) *
                  (points.point(2).y() - points.point(0).y());
  double d2 = (points.point(2).x() - points.point(0).x()) *
                  (points.point(3).y() - points.point(0).y()) -
              (points.point(3).x() - points.point(0).x()) *
                  (points.point(2).y() - points.point(0).y());
  if (fabs(d1 - d2) < 1e-6) {
    return point;
  }
  double t = d1 / (d1 - d2);
  point.x() =
      points.point(1).x() + (points.point(3).x() - points.point(1).x()) * t;
  point.y() =
      points.point(1).y() + (points.point(3).y() - points.point(1).y()) * t;
  return point;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
