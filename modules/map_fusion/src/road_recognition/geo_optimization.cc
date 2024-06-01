/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： geo_optimization.cc
 *   author     ： zhangzhike
 *   date       ： 2023.12
 ******************************************************************************/

#include "map_fusion/road_recognition/geo_optimization.h"
#include <gflags/gflags.h>
#include <math.h>
#include <sys/param.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <unordered_map>

#include "base/utils/log.h"
#include "depend/common/utm_projection/coordinate_convertor.h"
#include "map_fusion/fusion_common/calc_util.h"
#include "map_fusion/map_service/global_hd_map.h"
#include "util/mapping_log.h"
#include "util/tic_toc.h"

#define PI acos(-1)

DEFINE_bool(road_recognition_rviz, false, "road recognition use rviz or not");

namespace hozon {
namespace mp {
namespace mf {

int GeoOptimization::Init() {
  // 一些maker等发布的定义（补充）
  //
  if (FLAGS_road_recognition_rviz && RVIZ_AGENT.Ok()) {
    int ret = RVIZ_AGENT.Register<adsfi_proto::viz::TransformStamped>(
        kTopicRoadRecognitionTf);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register " << kTopicRoadRecognitionTf
                << " failed";
    }
    ret = RVIZ_AGENT.Register<adsfi_proto::viz::Path>(
        kTopicRoadRecognitionLocation);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register " << kTopicRoadRecognitionLocation
                << " failed";
    }
    std::vector<std::string> topic_vec = {
        KTopicRoadRecognitionLocalMap, KTopicRoadRecognitionTopoMapRoad,
        KTopicRoadRecognitionTopoMapLane, KTopicRoadRecognitionElementMap,
        KTopicRoadRecognitionLineLable};
    ret = RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(topic_vec);
  }
  elem_map_ = std::make_shared<hozon::mp::mf::em::ElementMap>();
  local_map_ = std::make_shared<hozon::mapping::LocalMap>();
  // pilot_map_ = std::make_shared<hozon::hdmap::Map>();
  local_map_use_ = std::make_shared<hozon::mapping::LocalMap>();
  // map_lanes_ = std::make_shared<std::vector<LanePilot>>();
  HLOG_WARN << "inital success!";
  return 0;
}

void GeoOptimization::PointsToMarker(const double stamp,
                                     const std::vector<Eigen::Vector3d>& points,
                                     adsfi_proto::viz::Marker* marker,
                                     const std::vector<float>& color_type) {
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
  color.set_r(color_type[0]);
  color.set_g(color_type[1]);
  color.set_b(color_type[2]);

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

void GeoOptimization::LineIdToMarker(const double stamp,
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
  const double text_size = 3;
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

void GeoOptimization::VizRRMapLane(
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
      std::vector<float> color = color_palette.find('g')->second;
      PointsToMarker(cur_timestamp_, lane_points, &marker, color);
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
      std::vector<float> color = color_palette.find('g')->second;
      PointsToMarker(cur_timestamp_, lane_points, &marker, color);
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

void GeoOptimization::VizRRMapRoad(
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
          std::vector<float> color = color_palette.find('b')->second;
          PointsToMarker(cur_timestamp_, boundary_points, &marker, color);
          if (marker.points().size() >= 2) {
            markers_road->add_markers()->CopyFrom(marker);
          }
        }
      }
    }
  }
}

// void GeoOptimization::VizRRMap(const std::shared_ptr<hozon::hdmap::Map>& msg)
// {
//   if (!FLAGS_road_recognition_rviz || !RVIZ_AGENT.Ok()) {
//     return;
//   }
//   adsfi_proto::viz::MarkerArray markers_road;

//   VizRRMapRoad(msg, &markers_road);

//   RVIZ_AGENT.Publish(KTopicRoadRecognitionTopoMapRoad, markers_road);

//   adsfi_proto::viz::MarkerArray markers_lane;

//   VizRRMapLane(msg, &markers_lane);

//   RVIZ_AGENT.Publish(KTopicRoadRecognitionTopoMapLane, markers_lane);
// }

std::shared_ptr<hozon::mp::mf::em::ElementMap> GeoOptimization::GetElemMap() {
  std::lock_guard<std::mutex> lock_map(mtx_);

  return elem_map_;
}

// std::shared_ptr<hozon::hdmap::Map> GeoOptimization::GetPilotMap() {
//   std::lock_guard<std::mutex> lock_pilot_map(mtxp_);
//   return pilot_map_;
// }

void GeoOptimization::OnLocalization(
    const std::shared_ptr<hozon::localization::Localization>& msg) {
  if (!msg) {
    HLOG_ERROR << "nullptr input localization";
    return;
  }
  auto stamp = msg->header().data_stamp();
  // 提取全局定位
  //! TBD：这里将utm转成了gcj02，但实际内部需要的也是utm，后面考虑
  Eigen::Vector3d pos_global_utm(msg->pose().pos_utm_01().x(),
                                 msg->pose().pos_utm_01().y(),
                                 msg->pose().pos_utm_01().z());

  double utm_x = msg->pose().pos_utm_01().x();
  double utm_y = msg->pose().pos_utm_01().y();
  utm_zone_ = static_cast<int>(msg->pose().utm_zone_01());

  bool ret =
      hozon::common::coordinate_convertor::UTM2GCS(utm_zone_, &utm_x, &utm_y);
  if (!ret) {
    HLOG_ERROR << "UTM2GCS failed";
    return;
  }
  // gcj02
  Eigen::Vector3d pos_global(utm_y, utm_x, 0);

  // 更新车辆ins位置
  {
    std::lock_guard<std::mutex> lock_vehicle_pose(vehicle_pose_mtx_);
    vehicle_pose_ = pos_global;
  }

  if (!init_) {
    HLOG_ERROR << "ref_point_ = pose";
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

  // local信息
  pos_local_ << msg->pose().local_pose().x(), msg->pose().local_pose().y(),
      msg->pose().local_pose().z();
  auto yaw = msg->pose().euler_angles_local().z();
  auto roll = msg->pose().euler_angles_local().x();
  auto pitch = msg->pose().euler_angles_local().y();
  quat_local_ = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());

  yaw = msg->pose().euler_angles().z();
  roll = msg->pose().euler_angles().x();
  pitch = msg->pose().euler_angles().y();
  Eigen::Quaterniond quat_global =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
  Eigen::Isometry3d T_veh_to_local;
  T_veh_to_local.setIdentity();
  T_veh_to_local.rotate(quat_local_);
  T_veh_to_local.pretranslate(pos_local_);

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
  T_veh_to_local_enu.pretranslate(enu);

  T_local_enu_to_local_.setIdentity();
  T_local_enu_to_local_ = T_veh_to_local * T_veh_to_local_enu.inverse();

  Eigen::Isometry3d T_veh_to_utm;
  T_veh_to_local_enu.setIdentity();
  T_veh_to_local_enu.rotate(quat_enu);
  T_veh_to_local_enu.pretranslate(pos_global_utm);

  T_utm_to_local_.setIdentity();
  T_utm_to_local_ = T_veh_to_local * T_veh_to_local_enu.inverse();

  {
    std::lock_guard<std::mutex> lock_pose(pose_mtx_);
    ins_q_w_v_ = quat_global;
    ins_pose_ = enu;
  }
  // 是否需要可视化vehicle position
  // 可视化！
  // VisulPos();
  // 可视化vehicle position
  if (FLAGS_road_recognition_rviz) {
    VizLocation(enu, quat_global, stamp);
  }
}

void GeoOptimization::VizLocation(const Eigen::Vector3d& pose,
                                  const Eigen::Quaterniond& q_W_V,
                                  const double stamp) {
  if (RVIZ_AGENT.Ok()) {
    static uint32_t seq = 0;
    uint32_t curr_seq = seq++;
    adsfi_proto::viz::TransformStamped geo_tf;
    geo_tf.mutable_header()->set_seq(curr_seq);

    uint32_t sec = 0;

    uint32_t nsec = 0;
    SplitSeconds(stamp, &sec, &nsec);
    geo_tf.mutable_header()->mutable_timestamp()->set_sec(sec);
    geo_tf.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    geo_tf.mutable_header()->set_frameid("map");
    geo_tf.set_child_frame_id("vehicle");
    geo_tf.mutable_transform()->mutable_translation()->set_x(pose.x());
    geo_tf.mutable_transform()->mutable_translation()->set_y(pose.y());
    geo_tf.mutable_transform()->mutable_translation()->set_z(pose.z());
    geo_tf.mutable_transform()->mutable_rotation()->set_x(q_W_V.x());
    geo_tf.mutable_transform()->mutable_rotation()->set_y(q_W_V.y());
    geo_tf.mutable_transform()->mutable_rotation()->set_z(q_W_V.z());
    geo_tf.mutable_transform()->mutable_rotation()->set_w(q_W_V.w());
    RVIZ_AGENT.Publish(kTopicRoadRecognitionTf, geo_tf);

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
    RVIZ_AGENT.Publish(kTopicRoadRecognitionLocation, location_path_);
  }
}

// void GeoOptimization::UpdateLaneByLocalmap(
//     const std::shared_ptr<hozon::mapping::LocalMap>& local_map) {
//   // 构建laneline的所在的车道和车道左右关联
//   if (local_map->lane_lines_size() < 2) {
//     return;
//   }
//   int left_lanepos = 0;
//   int right_lanepos = 0;
//   for (const auto& lane_line : local_map->lane_lines()) {
//     if (lane_line.points().empty() ||
//         lane_line.lanepos() == LanePositionType::OTHER) {
//       continue;
//     }
//     left_lanepos =
//         std::min(left_lanepos, static_cast<int>(lane_line.lanepos()));
//     right_lanepos =
//         std::max(right_lanepos, static_cast<int>(lane_line.lanepos()));
//   }
//   if (left_lanepos == 0 || right_lanepos == 0) {
//     return;
//   }
//   std::vector<LanePilot> map_lanes;

//   // 找出左右车道及中间车道
//   for (int i = left_lanepos; i < -1; i++) {
//     LanePilot lane;
//     lane.lane_id_ = i + 1;
//     for (const auto& cur_lane_line : local_map->lane_lines()) {
//       if (static_cast<int>(cur_lane_line.lanepos()) == i) {
//         lane.left_line_ = cur_lane_line;
//       }
//       if (static_cast<int>(cur_lane_line.lanepos()) == i + 1) {
//         lane.right_line_ = cur_lane_line;
//       }
//     }
//     map_lanes.push_back(lane);
//   }

//   LanePilot cur_lane;
//   cur_lane.lane_id_ = 0;
//   for (const auto& cur_lane_line : local_map->lane_lines()) {
//     if (static_cast<int>(cur_lane_line.lanepos()) == -1) {
//       cur_lane.left_line_ = cur_lane_line;
//     }
//     if (static_cast<int>(cur_lane_line.lanepos()) == 1) {
//       cur_lane.right_line_ = cur_lane_line;
//     }
//   }
//   map_lanes.push_back(cur_lane);

//   for (int i = 1; i < right_lanepos; i++) {
//     LanePilot lane;
//     lane.lane_id_ = i;
//     for (const auto& cur_lane_line : local_map->lane_lines()) {
//       if (static_cast<int>(cur_lane_line.lanepos()) == i) {
//         lane.left_line_ = cur_lane_line;
//       }
//       if (static_cast<int>(cur_lane_line.lanepos()) == i + 1) {
//         lane.right_line_ = cur_lane_line;
//       }
//     }
//     map_lanes.push_back(lane);
//   }

//   // add width, center_points, left_lane_id and right_lane_id
//   for (auto& lane : map_lanes) {
//     if (static_cast<int>(lane.left_line_.lanepos()) == left_lanepos &&
//         static_cast<int>(lane.right_line_.lanepos()) == right_lanepos) {
//       lane.left_lane_id_ = 1000;
//       lane.right_lane_id_ = 1000;
//       break;
//     }
//     if (static_cast<int>(lane.left_line_.lanepos()) == left_lanepos) {
//       lane.left_lane_id_ = 1000;
//       lane.right_lane_id_ = lane.lane_id_ + 1;
//       continue;
//     }
//     if (static_cast<int>(lane.right_line_.lanepos()) == right_lanepos) {
//       lane.left_lane_id_ = lane.lane_id_ - 1;
//       lane.right_lane_id_ = 1000;
//       continue;
//     }
//     lane.left_lane_id_ = lane.lane_id_ - 1;
//     lane.right_lane_id_ = lane.lane_id_ + 1;
//   }
//   if (map_lanes.empty()) {
//     return;
//   }
//   // local_map->map_lanes_ = map_lanes;
//   map_lanes_ = std::make_shared<std::vector<LanePilot>>(map_lanes);
// }

// void GeoOptimization::PridictFrontMapLanes() {
//   double short_lane_x = PRIDICT_FRONT_LANES;
//   double k = DBL_MAX;
//   for (auto& map_lane : (*map_lanes_)) {
//     int lanesize = static_cast<int>(map_lane.left_line_.points().size());
//     if (lanesize >= 2 &&
//         map_lane.left_line_.points()[lanesize - 1].x() < short_lane_x &&
//         (static_cast<int>(map_lane.left_line_.lanepos()) == 1 ||
//             static_cast<int>(map_lane.left_line_.lanepos()) == -1)) {
//       short_lane_x = map_lane.left_line_.points()[lanesize - 1].x();
//       k = (map_lane.left_line_.points()[lanesize - 1].y() -
//           map_lane.left_line_.points()[lanesize - 2].y()) /
//           (map_lane.left_line_.points()[lanesize - 1].x() -
//               map_lane.left_line_.points()[lanesize - 2].x());
//     }
//     lanesize = static_cast<int>(map_lane.right_line_.points().size());
//     if (lanesize >= 2 &&
//         map_lane.right_line_.points()[lanesize - 1].x() < short_lane_x &&
//         (static_cast<int>(map_lane.right_line_.lanepos()) == 1 ||
//             static_cast<int>(map_lane.right_line_.lanepos()) == -1)) {
//       short_lane_x = map_lane.right_line_.points()[lanesize - 1].x();
//       k = (map_lane.right_line_.points()[lanesize - 1].y() -
//           map_lane.right_line_.points()[lanesize - 2].y()) /
//           (map_lane.right_line_.points()[lanesize - 1].x() -
//               map_lane.right_line_.points()[lanesize - 2].x());
//     }
//   }
//   // HLOG_ERROR << "目前总共的车道数："
//   //            << static_cast<int>(local_map->map_lanes_.size());
//   for (auto& map_lane : (*map_lanes_)) {
//     int lanesize = static_cast<int>(map_lane.left_line_.points().size());
//     if (lanesize < 2) {
//       continue;
//     }
//     // HLOG_ERROR << "车道：" << map_lane.lane_id_ << "  leftsize:" <<
//     lanesize;

//     // std::vector<Eigen::Vector3d> predict_front_points;
//     int index = 0;
//     for (double x = map_lane.left_line_.points()[lanesize - 1].x() + 1; x <
//     260;
//          x++) {
//       double y = k * (x - map_lane.left_line_.points()[lanesize - 1].x()) +
//           map_lane.left_line_.points()[lanesize - 1].y();
//       // Eigen::Vector3d point = {x, y, 0.0};
//       // predict_front_points.push_back(point);
//       auto* pt = map_lane.pilot_left_line_.add_points();
//       pt->set_x(x);
//       pt->set_y(y);
//       pt->set_z(0.0);
//     }
//     lanesize = static_cast<int>(map_lane.right_line_.points().size());
//     if (lanesize < 2) {
//       continue;
//     }
//     // HLOG_ERROR << "车道：" << map_lane.lane_id_ << "  rightsize:" <<
//     // lanesize;
//     for (double x = map_lane.right_line_.points()[lanesize - 1].x() + 1;
//          x < 260; x++) {
//       double y = k * (x - map_lane.right_line_.points()[lanesize - 1].x()) +
//           map_lane.right_line_.points()[lanesize - 1].y();
//       auto* pt = map_lane.pilot_right_line_.add_points();
//       pt->set_x(x);
//       pt->set_y(y);
//       pt->set_z(0.0);
//     }
//   }
// }

// void GeoOptimization::PridictBackMapLanes() {
//   double short_lane_x = PRIDICT_BACK_LANES;
//   double k = DBL_MAX;
//   double predict_length = DBL_MAX;
//   for (auto& map_lane : (*map_lanes_)) {
//     int lanesize = static_cast<int>(map_lane.left_line_.points().size());
//     if (lanesize != 0) {
//       predict_length =
//           std::min(predict_length, map_lane.left_line_.points()[0].x());
//     }
//     if (lanesize >= 2 && map_lane.left_line_.points()[0].x() < short_lane_x
//     &&
//         (static_cast<int>(map_lane.left_line_.lanepos()) == 1 ||
//             static_cast<int>(map_lane.left_line_.lanepos()) == -1)) {
//       short_lane_x = map_lane.left_line_.points()[0].x();
//       k = (map_lane.left_line_.points()[0].y() -
//           map_lane.left_line_.points()[1].y()) /
//           (map_lane.left_line_.points()[0].x() -
//               map_lane.left_line_.points()[1].x());
//     }
//     lanesize = static_cast<int>(map_lane.right_line_.points().size());
//     if (lanesize != 0) {
//       predict_length =
//           std::min(predict_length, map_lane.right_line_.points()[0].x());
//     }
//     if (lanesize >= 2 && map_lane.right_line_.points()[0].x() < short_lane_x
//     &&
//         (static_cast<int>(map_lane.right_line_.lanepos()) == 1 ||
//             static_cast<int>(map_lane.right_line_.lanepos()) == -1)) {
//       short_lane_x = map_lane.right_line_.points()[0].x();
//       k = (map_lane.right_line_.points()[0].y() -
//           map_lane.right_line_.points()[1].y()) /
//           (map_lane.right_line_.points()[0].x() -
//               map_lane.right_line_.points()[1].x());
//     }
//   }
//   // HLOG_ERROR << "目前总共的车道数："
//   // << static_cast<int>(local_map->map_lanes_.size());
//   for (auto& map_lane : (*map_lanes_)) {
//     int lanesize = static_cast<int>(map_lane.left_line_.points().size());
//     if (lanesize < 2) {
//       continue;
//     }
//     // HLOG_ERROR << "车道：" << map_lane.lane_id_ << "  leftsize:" <<
//     lanesize; for (double x = short_lane_x; x <
//     map_lane.left_line_.points()[0].x();
//          x++) {
//       double y = k * (x - map_lane.left_line_.points()[0].x()) +
//           map_lane.left_line_.points()[0].y();
//       auto* pt = map_lane.pilot_left_line_.add_points();
//       pt->set_x(x);
//       pt->set_y(y);
//       pt->set_z(0.0);
//     }

//     // 按顺序添加当前车道点
//     for (const auto& point : map_lane.left_line_.points()) {
//       auto* pt = map_lane.pilot_left_line_.add_points();
//       pt->set_x(point.x());
//       pt->set_y(point.y());
//       pt->set_z(point.z());
//     }
//     lanesize = static_cast<int>(map_lane.right_line_.points().size());
//     if (lanesize < 2) {
//       continue;
//     }
//     // HLOG_ERROR << "车道：" << map_lane.lane_id_ << "  rightsize:" <<
//     // lanesize;
//     for (double x = short_lane_x; x < map_lane.right_line_.points()[0].x();
//          x++) {
//       double y = k * (x - map_lane.right_line_.points()[0].x()) +
//           map_lane.right_line_.points()[0].y();
//       auto* pt = map_lane.pilot_right_line_.add_points();
//       pt->set_x(x);
//       pt->set_y(y);
//       pt->set_z(0.0);
//     }
//     // 按顺序添加当前车道点
//     for (const auto& point : map_lane.right_line_.points()) {
//       auto* pt = map_lane.pilot_right_line_.add_points();
//       pt->set_x(point.x());
//       pt->set_y(point.y());
//       pt->set_z(point.z());
//     }
//   }
// }

// void GeoOptimization::PridictCenterMapLanes() {
//   // 调用接口计算车道线中点

//   for (auto& map_lane : (*map_lanes_)) {
//     std::vector<common::math::Vec2d> left_points;
//     std::vector<common::math::Vec2d> right_points;
//     for (const auto& point : map_lane.pilot_left_line_.points()) {
//       left_points.emplace_back(point.x(), point.y());
//     }
//     for (const auto& point : map_lane.pilot_right_line_.points()) {
//       right_points.emplace_back(point.x(), point.y());
//     }
//     std::vector<common::math::Vec2d> cent_points;
//     if (left_points.empty() || right_points.empty()) {
//       continue;
//     }
//     common::math::GenerateCenterPoint(left_points, right_points,
//     &cent_points); for (const auto& point : cent_points) {
//       auto* pt = map_lane.pilot_center_line_.add_points();
//       pt->set_x(static_cast<double>(point.x()));
//       pt->set_y(static_cast<double>(point.y()));
//       pt->set_z(0.0);
//     }
//   }
// }

void GeoOptimization::VizLocalMap() {
  if ((!FLAGS_road_recognition_rviz) || (!RVIZ_AGENT.Ok())) {
    return;
  }
  adsfi_proto::viz::MarkerArray markers;
  for (const auto& i : local_map_->lane_lines()) {
    std::vector<Eigen::Vector3d> lane_points;
    for (const auto& point : i.points()) {
      Eigen::Vector3d point_local(point.x(), point.y(), point.z());
      Eigen::Vector3d point_enu = T_U_V_ * point_local;
      lane_points.emplace_back(point_enu);
    }
    adsfi_proto::viz::Marker marker;
    std::vector<float> color = color_palette.find('c')->second;
    PointsToMarker(local_map_->header().publish_stamp(), lane_points, &marker,
                   color);
    if (marker.points().size() >= 2) {
      markers.add_markers()->CopyFrom(marker);
    }
    if (!lane_points.empty()) {
      adsfi_proto::viz::Marker marker_id;
      auto id = std::to_string(i.lanepos());
      LineIdToMarker(local_map_->header().publish_stamp(), lane_points[0], id,
                     &marker_id);
      markers.add_markers()->CopyFrom(marker_id);
    }
  }

  for (const auto& i : local_map_->road_edges()) {
    std::vector<Eigen::Vector3d> lane_points;
    for (const auto& point : i.points()) {
      Eigen::Vector3d point_local(point.x(), point.y(), point.z());
      Eigen::Vector3d point_enu = T_U_V_ * point_local;
      lane_points.emplace_back(point_enu);
    }
    adsfi_proto::viz::Marker marker;
    std::vector<float> color = color_palette.find('b')->second;
    PointsToMarker(local_map_->header().publish_stamp(), lane_points, &marker,
                   color);
    if (marker.points().size() >= 2) {
      markers.add_markers()->CopyFrom(marker);
    }
    if (!lane_points.empty()) {
      adsfi_proto::viz::Marker marker_id;
      auto id = std::to_string(i.lanepos());
      LineIdToMarker(local_map_->header().publish_stamp(), lane_points[0], id,
                     &marker_id);
      markers.add_markers()->CopyFrom(marker_id);
    }
  }

  for (const auto& i : local_map_->stop_lines()) {
    std::vector<Eigen::Vector3d> lane_points;
    Eigen::Vector3d point_left(i.left_point().x(), i.left_point().y(), 0);
    Eigen::Vector3d point_right(i.right_point().x(), i.right_point().y(), 0);
    lane_points.emplace_back(T_U_V_ * point_left);
    lane_points.emplace_back(T_U_V_ * point_right);
    adsfi_proto::viz::Marker marker;
    std::vector<float> color = color_palette.find('o')->second;
    PointsToMarker(local_map_->header().publish_stamp(), lane_points, &marker,
                   color);
    markers.add_markers()->CopyFrom(marker);
  }

  for (const auto& i : local_map_->arrows()) {
    if (i.points().point_size() < 1) {
      continue;
    }
    std::vector<Eigen::Vector3d> arrow_points;
    for (const auto& point_arrow : i.points().point()) {
      Eigen::Vector3d arrow_pt(point_arrow.x(), point_arrow.y(),
                               point_arrow.z());
      arrow_points.emplace_back(T_U_V_ * arrow_pt);
    }
    arrow_points.emplace_back(arrow_points[0]);
    adsfi_proto::viz::Marker marker;
    std::vector<float> color = color_palette.find('w')->second;
    PointsToMarker(local_map_->header().publish_stamp(), arrow_points, &marker,
                   color);
    markers.add_markers()->CopyFrom(marker);
  }

  for (const auto& i : local_map_->cross_walks()) {
    if (i.points().point_size() < 1) {
      continue;
    }
    std::vector<Eigen::Vector3d> zebra_points;
    for (const auto& zebra_pt : i.points().point()) {
      Eigen::Vector3d pt(zebra_pt.x(), zebra_pt.y(), zebra_pt.z());
      zebra_points.emplace_back(T_U_V_ * pt);
    }
    zebra_points.emplace_back(zebra_points[0]);
    adsfi_proto::viz::Marker marker;
    std::vector<float> color = color_palette.find('c')->second;
    PointsToMarker(local_map_->header().publish_stamp(), zebra_points, &marker,
                   color);
    markers.add_markers()->CopyFrom(marker);
  }

  RVIZ_AGENT.Publish(KTopicRoadRecognitionLocalMap, markers);
}

void GeoOptimization::VizElementMap() {
  if ((!FLAGS_road_recognition_rviz) || (!RVIZ_AGENT.Ok())) {
    return;
  }
  adsfi_proto::viz::MarkerArray markers;
  for (const auto& i : local_map_use_->lane_lines()) {
    std::vector<Eigen::Vector3d> lane_points;
    for (const auto& point : i.points()) {
      Eigen::Vector3d point_local(point.x(), point.y(), point.z());
      Eigen::Vector3d point_enu = T_U_V_ * point_local;
      lane_points.emplace_back(point_enu);
    }
    adsfi_proto::viz::Marker marker;
    std::vector<float> color = color_palette.find('r')->second;
    PointsToMarker(local_map_->header().publish_stamp(), lane_points, &marker,
                   color);
    if (marker.points().size() >= 2) {
      markers.add_markers()->CopyFrom(marker);
    }
    if (!lane_points.empty()) {
      adsfi_proto::viz::Marker marker_id;
      auto id = std::to_string(i.lanepos());
      LineIdToMarker(local_map_->header().publish_stamp(), lane_points[0], id,
                     &marker_id);
      markers.add_markers()->CopyFrom(marker_id);
    }
  }

  RVIZ_AGENT.Publish(KTopicRoadRecognitionElementMap, markers);
}

// void GeoOptimization::
bool GeoOptimization::IsNeighberLine(int lanei_pos, int lanej_pos) {
  if (lanei_pos * lanej_pos) {
    if ((lanei_pos + 1 == lanej_pos) || (lanei_pos - 1 == lanej_pos)) {
      return true;
    }
    return false;
  } else {
    if (lanei_pos * lanej_pos == -1) {
      return true;
    }
    return false;
  }
  return false;
}

double GeoOptimization::ComputerPoint2Line(
    const hozon::mapping::LaneLine& lane, const hozon::common::Point3D& point) {
  Eigen::Vector3d P(point.x(), point.y(), point.z());
  for (size_t i = 1; i < lane.points_size(); i++) {
    Eigen::Vector3d A(lane.points()[i - 1].x(), lane.points()[i - 1].y(),
                      lane.points()[i - 1].z());
    Eigen::Vector3d B(lane.points()[i].x(), lane.points()[i].y(),
                      lane.points()[i].z());
    Eigen::Vector3d AB = B - A;
    Eigen::Vector3d AP = P - A;
    double ABLength = AB.norm();
    if (ABLength < 0.01) {
      continue;
    }
    double t = AB.dot(AP) / ABLength;
    if (t < -0.1 || t > 1.1) {
      // 不在其中
      continue;
    }
    double res = (AB.cross(AP)).norm() / ABLength;  // 该点到两点组成直线距离
    return res;
  }
  return 0.0;
}

bool GeoOptimization::ComputerPointIsInLine(const Eigen::Vector3d& P,
                                            const Eigen::Vector3d& A,
                                            const Eigen::Vector3d& B) {
  Eigen::Vector3d AB = B - A;
  Eigen::Vector3d AP = P - A;
  double ABLength = AB.norm();
  if (ABLength < 0.01) {
    return false;
  }
  double t = AB.dot(AP) / ABLength;
  if (t < -0.01 || t > 1.01) {
    return false;
  }
  return true;
}

bool GeoOptimization::IsRight(const Eigen::Vector3d& P,
                              const Eigen::Vector3d& A,
                              const Eigen::Vector3d& B) {
  Eigen::Vector3d AB = B - A;
  Eigen::Vector3d AP = P - A;
  double ABLength = AB.norm();
  double t = AB.dot(AP) / ABLength;
  Eigen::Vector3d C = A + t * AB;
  return C.y() > P.y();  // 由于y轴是指向左边，所以c<p的意思是p是否在c的右边
}

double GeoOptimization::ComputerPoint2Line(const Eigen::Vector3d& P,
                                           const Eigen::Vector3d& A,
                                           const Eigen::Vector3d& B) {
  Eigen::Vector3d AB = B - A;
  Eigen::Vector3d AP = P - A;
  double ABLength = AB.norm();
  return (AB.cross(AP)).norm() / ABLength;
}

void GeoOptimization::FilterLocalMapLine(
    const std::shared_ptr<hozon::mapping::LocalMap>& local_map) {
  // 构建kdtree
  std::map<int, std::vector<Line_kd>> all_lines;
  std::set<int> last_track_id;
  if (!last_track_id_.empty()) {
    last_track_id = last_track_id_;
    last_track_id_.clear();
  }
  for (const auto& line : local_map->lane_lines()) {
    if (line.points().size() < 2) {
      continue;
    }
    // if (static_cast<int>(line.track_id()) > 90) {
    //   continue;  // 滤除不明线
    // }
    // if (static_cast<int>(line.lanepos()) > 90) {
    //   continue;
    // }
    if (!last_track_id.empty() &&
        last_track_id.find(line.track_id()) != last_track_id.end() &&
        line.points_size() < 25) {
      //  上一帧的trackid已经被滤除了这一帧不进行保存
      last_track_id_.insert(line.track_id());
      continue;
    }
    std::vector<cv::Point2f> kdtree_points;
    Line_kd line_kd;
    for (const auto& point : line.points()) {
      if (std::isnan(point.x()) || std::isnan(point.y()) ||
          std::isnan(point.z())) {
        HLOG_ERROR << "found nan point in local map lane line";
        continue;
      }
      Eigen::Vector3d point_local(point.x(), point.y(), point.z());
      // Eigen::Vector3d point_enu = T_U_V_ * point_local;
      kdtree_points.emplace_back(static_cast<float>(point_local.x()),
                                 static_cast<float>(point_local.y()));
    }
    cv::flann::KDTreeIndexParams index_param(1);
    std::shared_ptr<cv::flann::Index> kdtree_ptr =
        std::make_shared<cv::flann::Index>(cv::Mat(kdtree_points).reshape(1),
                                           index_param);
    line_kd.line_kdtree = kdtree_ptr;
    line_kd.line = std::make_shared<hozon::mapping::LaneLine>(line);
    all_lines[static_cast<int>(line.lanepos())].emplace_back(line_kd);
  }

  all_lines_.clear();

  // 计算线段的第一个点、最后一个点、中间点
  // 到左右lanepos直线的距离（后面可以优化成到最近一条直线的距离）
  int lane_size = local_map->lane_lines().size();
  if (lane_size < 2) {
    all_lines_ = all_lines;
    return;
  }
  int left_lanepos = 0, right_lanepos = 0;
  for (const auto& line : local_map->lane_lines()) {
    if (line.points().empty()) {
      continue;
    }
    left_lanepos = std::min(left_lanepos, static_cast<int>(line.lanepos()));
    right_lanepos = std::max(right_lanepos, static_cast<int>(line.lanepos()));
  }
  // HLOG_ERROR << "left_lanepos = " << left_lanepos
  //           << "  right_lanepos = " << right_lanepos;

  // !!!!!!!!!需要把类型也添加进考虑范围内！！！！！！

  // if (left_lanepos < -1 || right_lanepos > 1) {
  //   // 最左边线是否保存
  //   if (left_lanepos < -1) {
  //     // 计算最左边线们
  //     for (auto& line_left : all_lines[left_lanepos]) {
  //       int num_calculate = 0, num_thresh = 0;
  //       for (const auto& point : line_left.line->points()) {
  //         // 计算line_left每个点在line_right内的最近距离
  //         if (all_lines.find(left_lanepos + 1) == all_lines.end() ||
  //             all_lines[left_lanepos + 1].empty())
  //           break;
  //         for (auto& line_right : all_lines[left_lanepos + 1]) {
  //           int dim = 2;
  //           std::vector<int> nearest_index(dim);
  //           std::vector<float> nearest_dis(dim);
  //           std::vector<float> query_point = std::vector<float>{
  //               static_cast<float>(point.x()),
  //               static_cast<float>(point.y())};
  //           line_right.line_kdtree->knnSearch(query_point, nearest_index,
  //                                             nearest_dis, dim,
  //                                             cv::flann::SearchParams(-1));
  //           if (nearest_index.size() < 2) {
  //             continue;
  //           }
  //           Eigen::Vector3d point_e(point.x(), point.y(), 0.0);
  //           Eigen::Vector3d line_right11(
  //               line_right.line->points(nearest_index[0]).x(),
  //               line_right.line->points(nearest_index[0]).y(), 0.0);
  //           Eigen::Vector3d line_right12(
  //               line_right.line->points(nearest_index[1]).x(),
  //               line_right.line->points(nearest_index[1]).y(), 0.0);
  //           if (ComputerPointIsInLine(point_e, line_right11, line_right12)) {
  //             num_calculate++;
  //             if (ComputerPoint2Line(point_e, line_right11, line_right12) >
  //                 2.5) {
  //               num_thresh++;
  //             }
  //             break;
  //           }
  //         }
  //       }
  //       if (num_calculate > 0 && static_cast<double>(num_thresh) /
  //                                        static_cast<double>(num_calculate) <
  //                                    0.5) {
  //         line_left.store = false;
  //       }
  //     }
  //   }
  //   // 同理最右边线是否保存
  //   if (right_lanepos > 1) {
  //     // 计算最左边线们
  //     for (auto& line_right : all_lines[right_lanepos]) {
  //       int num_calculate = 0, num_thresh = 0;
  //       for (const auto& point : line_right.line->points()) {
  //         // 计算line_left每个点在line_right内的最近距离
  //         if (all_lines.find(right_lanepos - 1) == all_lines.end() ||
  //             all_lines[right_lanepos - 1].empty())
  //           break;
  //         for (auto& line_left : all_lines[right_lanepos - 1]) {
  //           int dim = 2;
  //           std::vector<int> nearest_index(dim);
  //           std::vector<float> nearest_dis(dim);
  //           std::vector<float> query_point = std::vector<float>{
  //               static_cast<float>(point.x()),
  //               static_cast<float>(point.y())};
  //           line_left.line_kdtree->knnSearch(query_point, nearest_index,
  //                                            nearest_dis, dim,
  //                                            cv::flann::SearchParams(-1));
  //           if (nearest_index.size() < 2) {
  //             continue;
  //           }
  //           Eigen::Vector3d point_e(point.x(), point.y(), 0.0);
  //           Eigen::Vector3d line_left11(
  //               line_left.line->points(nearest_index[0]).x(),
  //               line_left.line->points(nearest_index[0]).y(), 0.0);
  //           Eigen::Vector3d line_left12(
  //               line_left.line->points(nearest_index[1]).x(),
  //               line_left.line->points(nearest_index[1]).y(), 0.0);
  //           if (ComputerPointIsInLine(point_e, line_left11, line_left12)) {
  //             num_calculate++;
  //             if (ComputerPoint2Line(point_e, line_left11, line_left12)
  //             > 2.5) {
  //               num_thresh++;
  //             }
  //             break;
  //           }
  //         }
  //       }
  //       if (num_calculate > 0 && static_cast<double>(num_thresh) /
  //                                        static_cast<double>(num_calculate) <
  //                                    0.5) {
  //         line_right.store = false;
  //       }
  //     }
  //   }
  // }
  all_lines_ = all_lines;
  return;
  // for(int i = 0; i < lane_size - 1; i++){
  //   if (local_map->lane_lines()[i].points().empty()){
  //     continue;
  //   }
  //   int lanei_size = local_map->lane_lines()[i].points_size();
  //   int lanei_pos = local_map->lane_lines()[i].lanepos();
  //   int flag = 1;  // 距离是否全部大于3m
  //   for(int j = i + 1; j < lane_size; j++){
  //     int lanej_pos = local_map->lane_lines()[j].lanepos();
  //     if (local_map->lane_lines()[j].points().empty() ||
  //     !IsNeighberLine(lanei_pos, lanej_pos)) {
  //       continue;
  //     }
  //     hozon::common::Point3D point_first =
  //     local_map->lane_lines()[i].points()[0]; hozon::common::Point3D
  //     point_end = local_map->lane_lines()[i].points()[lanei_size - 1];
  //     hozon::common::Point3D point_middle =
  //     local_map->lane_lines()[i].points()[lanei_size / 2]; if
  //     (ComputerPoint2Line(local_map->lane_lines()[j], point_first) < 2.0 ||
  //     ComputerPoint2Line(local_map->lane_lines()[j], point_end) < 2.0 ||
  //     ComputerPoint2Line(local_map->lane_lines()[j], point_middle) < 2.0) {
  //       flag = 0;
  //       break;
  //     }
  //     flag = 1;
  //   }
  //   if (flag == 1) {
  //     auto* lane_use = local_map_use->add_lane_lines();
  //     lane_use->CopyFrom(local_map->lane_lines()[i]);
  //   }
  // }
}

bool GeoOptimization::IsOppisiteLine(Line_kd* line) {
  int flag = 0;
  for (const auto& edge_line : local_map_->road_edges()) {
    if (edge_line.lanepos() < -1 || edge_line.lanepos() > 1 ||
        edge_line.points().empty()) {
      continue;
    }
    int num_thresh = 0, num_calculate = 0;
    flag = 1;
    for (const auto& point : edge_line.points()) {
      if (std::isnan(point.x()) || std::isnan(point.y()) ||
          std::isnan(point.z())) {
        HLOG_ERROR << "found nan point in local map edge line";
        continue;
      }
      int dim = 2;
      std::vector<int> nearest_index(dim);
      std::vector<float> nearest_dis(dim);
      std::vector<float> query_point = std::vector<float>{
          static_cast<float>(point.x()), static_cast<float>(point.y())};
      line->line_kdtree->knnSearch(query_point, nearest_index, nearest_dis, dim,
                                   cv::flann::SearchParams(-1));
      if (nearest_index.size() < 2) {
        continue;
      }
      Eigen::Vector3d point_e(point.x(), point.y(), point.z());
      Eigen::Vector3d line11(line->line->points(nearest_index[0]).x(),
                             line->line->points(nearest_index[0]).y(), 0.0);
      Eigen::Vector3d line12(line->line->points(nearest_index[1]).x(),
                             line->line->points(nearest_index[1]).y(), 0.0);
      if (ComputerPointIsInLine(point_e, line11, line12)) {
        num_calculate++;
        if (IsRight(point_e, line11, line12)) {
          num_thresh++;
        }
        break;
      }
    }
    if (edge_line.lanepos() == -1) {
      if (num_calculate < 1 ||
          static_cast<double>(num_thresh) / static_cast<double>(num_calculate) >
              0.5) {
        line->is_ego_road = false;
        return false;
      }
    } else {
      if (num_calculate < 1 ||
          static_cast<double>(num_thresh) / static_cast<double>(num_calculate) <
              0.5) {
        line->is_ego_road = false;
        return false;
      }
    }
  }
  if (flag) return true;
  return false;
}

bool GeoOptimization::Intersect(const hozon::common::Point3D& line1_s,
                                const hozon::common::Point3D& line1_e,
                                const hozon::common::Point3D& line2_s,
                                const hozon::common::Point3D& line2_e) {
  Eigen::Vector3d l1_s(line1_s.x(), line1_s.y(),
                       0.0);  // line1.line_points[0];
  Eigen::Vector3d l1_e(line1_e.x(), line1_e.y(),
                       0.0);  //  = line1.line_points[line1_size - 1];
  Eigen::Vector3d l2_s(line2_s.x(), line2_s.y(),
                       0.0);  // = line2.line_points[0];
  Eigen::Vector3d l2_e(line2_e.x(), line2_e.y(),
                       0.0);  //= line2.line_points[line2_size - 1];
  // 快速排除不可能相交的线
  if ((l1_s.x() > l1_e.x() ? l1_s.x() : l1_e.x()) <
          (l2_s.x() < l2_e.x() ? l2_s.x() : l2_e.x()) ||
      (l2_s.x() > l2_e.x() ? l2_s.x() : l2_e.x()) <
          (l1_s.x() < l1_e.x() ? l1_s.x() : l1_e.x()) ||
      (l1_s.y() > l1_e.y() ? l1_s.y() : l1_e.y()) <
          (l2_s.y() < l2_e.y() ? l2_s.y() : l2_e.y()) ||
      (l2_s.y() > l2_e.y() ? l2_s.y() : l2_e.y()) <
          (l1_s.y() < l1_e.y() ? l1_s.y() : l1_e.y())) {
    return false;
  }
  // 叉乘判断是否相交, 叉乘的正负代表逆时针顺时针即可判断方向 AB.cross(AP)
  if (((l1_e - l2_s).cross(l2_e - l2_s)).dot((l1_s - l2_s).cross(l2_e - l2_s)) >
          0 ||
      ((l2_e - l1_s).cross(l1_e - l1_s)).dot((l2_s - l1_s).cross(l1_e - l1_s)) >
          0) {
    return false;
  }
  return true;
}

bool GeoOptimization::Intersect(const Line_kd& line1, const Line_kd& line2) {
  int line1_size = line1.line->points_size(),
      line2_size = line2.line->points_size();

  Eigen::Vector3d l1_s(line1.line->points(0).x(), line1.line->points(0).y(),
                       0.0);  // line1.line_points[0];
  Eigen::Vector3d l1_e(line1.line->points(line1_size - 1).x(),
                       line1.line->points(line1_size - 1).y(),
                       0.0);  //  = line1.line_points[line1_size - 1];
  Eigen::Vector3d l2_s(line2.line->points(0).x(), line2.line->points(0).y(),
                       0.0);  // = line2.line_points[0];
  Eigen::Vector3d l2_e(line2.line->points(line2_size - 1).x(),
                       line2.line->points(line2_size - 1).y(),
                       0.0);  //= line2.line_points[line2_size - 1];
  // 快速排除不可能相交的线
  if ((l1_s.x() > l1_e.x() ? l1_s.x() : l1_e.x()) <
          (l2_s.x() < l2_e.x() ? l2_s.x() : l2_e.x()) ||
      (l2_s.x() > l2_e.x() ? l2_s.x() : l2_e.x()) <
          (l1_s.x() < l1_e.x() ? l1_s.x() : l1_e.x()) ||
      (l1_s.y() > l1_e.y() ? l1_s.y() : l1_e.y()) <
          (l2_s.y() < l2_e.y() ? l2_s.y() : l2_e.y()) ||
      (l2_s.y() > l2_e.y() ? l2_s.y() : l2_e.y()) <
          (l1_s.y() < l1_e.y() ? l1_s.y() : l1_e.y())) {
    return false;
  }
  // 叉乘判断是否相交, 叉乘的正负代表逆时针顺时针即可判断方向 AB.cross(AP)
  if (((l1_e - l2_s).cross(l2_e - l2_s)).dot((l1_s - l2_s).cross(l2_e - l2_s)) >
          0 ||
      ((l2_e - l1_s).cross(l1_e - l1_s)).dot((l2_s - l1_s).cross(l1_e - l1_s)) >
          0) {
    return false;
  }
  return true;
}

void GeoOptimization::MergeCount(int* num, Eigen::MatrixXi* intersect_mat) {
  std::deque<int> dq;
  for (int i = 0; i < intersect_mat->cols(); ++i) {
    for (int j = 0; j < intersect_mat->cols(); ++j) {
      if ((*intersect_mat)(i, j) == -1) {
        continue;
      }
      if ((*intersect_mat)(i, j) == 1) {
        dq.push_back(i);
        dq.push_back(j);
        (*intersect_mat)(i, j) = *num + 2;
        (*intersect_mat)(j, i) = *num + 2;
        break;
      }
      (*intersect_mat)(i, j) = -1;
      (*intersect_mat)(j, i) = -1;
    }
  }
  while (!dq.empty()) {
    int& tmp_index = dq.front();
    for (int i = 0; i < intersect_mat->cols(); ++i) {
      if ((*intersect_mat)(i, tmp_index) == -1) {
        continue;
      }
      if ((*intersect_mat)(i, tmp_index) == 1) {
        dq.push_back(i);
        (*intersect_mat)(tmp_index, i) = *num + 2;
        (*intersect_mat)(i, tmp_index) = *num + 2;
        continue;
      }
      (*intersect_mat)(i, tmp_index) = -1;
      (*intersect_mat)(tmp_index, i) = -1;
    }
    dq.pop_front();
  }
  *num = *num + 1;
}

void GeoOptimization::FilterIntersectLine() {
  const int line_size = all_lines_.size();  // 一共几个lanepos对应的车道线
  if (line_size < 3) {
    return;
  }
  // Eigen::MatrixXi intersect_mat =
  //     Eigen::MatrixXi::Ones(line_size, line_size) * (-1);
  std::vector<int> laneposes;
  laneposes.resize(line_size);
  int index_i = 0;
  for (const auto& t : all_lines_) {
    laneposes[index_i++] = t.first;
  }

  if (laneposes.size() != line_size) {
    HLOG_ERROR << "the laneposes size is not equal to line_size";
    return;
  }
  // 不判断比较长的线
  for (int i = 0; i < line_size - 1; ++i) {
    for (int j = i + 1; j < line_size; ++j) {
      for (auto& line : all_lines_[laneposes[i]]) {
        // 点数小于20
        if (line.line->points_size() > 25) {
          continue;
        }
        for (auto& line2 : all_lines_[laneposes[j]]) {
          if (line2.line->points_size() > 25) {
            continue;
          }

          if (Intersect(line, line2)) {
            line.store = false;
            line2.store = false;
          }
        }
      }
    }
  }
  // 短线跟长线交叉 删除短线
  for (int i = 0; i < line_size - 1; ++i) {
    for (int j = i + 1; j < line_size; ++j) {
      for (auto& line : all_lines_[laneposes[i]]) {
        if (!line.store) {
          continue;
        }
        for (auto& line2 : all_lines_[laneposes[j]]) {
          if (!line2.store) {
            continue;
          }
          int size1 = static_cast<int>(line.line->points_size());
          int size2 = static_cast<int>(line2.line->points_size());
          if (size1 < 2 || size2 < 2) {
            continue;
          }
          if (line.line->points(size1 - 1).y() < line2.line->points(0).y() ||
              line2.line->points(size2 - 1).y() < line.line->points(0).y()) {
            // 如果不可能存在交叉可能性
            continue;
          }
          // 只有长线和短线才进行操作
          if (size1 > 25 && size2 <= 25) {
            if (Intersect(line.line->points(0), line.line->points(size1 - 1),
                          line2.line->points(0),
                          line2.line->points(size2 - 1))) {
              line2.store = false;
            }
          } else if (size1 <= 25 && size2 > 25) {
            if (Intersect(line2.line->points(0), line2.line->points(size2 - 1),
                          line.line->points(0), line.line->points(size1 - 1))) {
              line.store = false;
            }
          }
        }
      }
    }
  }
}

void GeoOptimization::FilterOppositeLine() {
  // 应该满足如下条件：
  // 1.感知漏检road edge
  // 2.同一根road edge检测成两条线
  // 3.在汇入路口感知也检测到汇入的road edge汇入的line不应该被滤除
  // 4.如果是同一根line lanepos它们是否被滤除应该一致
  // 策略：
  // 1.如果road edge只有-1或者1,则只判断一边或者不判断
  // 2.按照lanepos处理。在-1 road edge的右侧，在 1 road
  // edge的左侧保留，用y值判断左右 3.汇入用is_merge来判断。如果is_merge = true,
  // store = false还是会存储 4.遍历lanepos的lane

  // 从小到大遍历
  for (auto& line : all_lines_) {
    if (line.second.empty()) {
      continue;
    }
    bool flag = false;
    for (auto& line_vector : line.second) {
      if (line_vector.is_ego_road) {
        if (IsOppisiteLine(&line_vector)) {
          flag = true;
          break;
        }
      }
    }
    if (flag) {
      for (auto& line_vector : line.second) {
        line_vector.is_ego_road = true;
      }
    }
  }
}

void GeoOptimization::FilterReverseLine() {
  // 处理路口逆向车道的车道线
  /*
    路口场景
              |    |    |
              |    |    |
              |  ↓ |  ↑ |
              |    |    |
              /         \
             /           \



             \           /
              \         /
               | | | | |
               | | |↑| |
               | | | | |
    方案： 1.获取起始点大于0的模型路沿，并使用平均y值进行排序
          2.计算相邻两根车道线平均距离，根据距离和相对关系判断是否是逆向车道
          3.如果是，对逆向车道中的车道线进行标识，供下游使用和判别
  */
  if (!local_map_) {
    HLOG_ERROR << "local_map_ is nullptr";
    return;
  }
  std::vector<std::vector<Eigen::Vector3d>> forward_road_edges;
  for (const auto& local_road : local_map_->road_edges()) {
    if (local_road.points_size() < 2 ||
        (!local_road.points().empty() && local_road.points().at(0).x() < 0)) {
      continue;
    }
    std::vector<Eigen::Vector3d> pts;
    for (const auto& it : local_road.points()) {
      Eigen::Vector3d pt(it.x(), it.y(), it.z());
      pts.emplace_back(pt);
    }
    forward_road_edges.emplace_back(pts);
  }
  std::vector<Eigen::Vector3d> road_edge_pts;
  road_edge_pts = FindTargetPoints(forward_road_edges);

  // 双黄线
  std::vector<std::vector<Eigen::Vector3d>> double_solid_yellow_line;
  for (const auto& line_vec : all_lines_) {
    if (line_vec.second.empty()) {
      continue;
    }
    for (const auto& line : line_vec.second) {
      if (line.line->lanetype() ==
              hozon::mapping::LaneType::LaneType_DOUBLE_SOLID &&
          line.line->color() == hozon::mapping::Color::YELLOW &&
          line.line->points_size() > 1 && line.line->points().at(0).x() > 0) {
        std::vector<Eigen::Vector3d> pts;
        for (const auto& it : line.line->points()) {
          Eigen::Vector3d pt(it.x(), it.y(), it.z());
          pts.emplace_back(pt);
        }
        double_solid_yellow_line.emplace_back(pts);
      }
    }
  }

  // 确定双黄线的左右侧是否有车道线
  std::vector<Eigen::Vector3d> double_solid_yellow_pts;
  double_solid_yellow_pts = FindTargetPoints(double_solid_yellow_line);
  // 确定路口逆向车道(是否可以考虑双黄线？或者结合OCC进行判别？)
  /*
    1.若有双黄线，将双黄线左侧的车道线做标记
    2.若无双黄线，根据模型路沿或者occ路沿的相对关系来进行判别
  */
  if (!road_edge_pts.empty()) {
    HandleOppisiteLine(road_edge_pts);
  } else {
    if (!double_solid_yellow_pts.empty()) {
      HandleOppisiteLine(double_solid_yellow_pts);
    }
  }
}

std::vector<Eigen::Vector3d> GeoOptimization::FindTargetPoints(
    const std::vector<std::vector<Eigen::Vector3d>>& forward_road_edges) {
  std::vector<Eigen::Vector3d> res;
  for (const auto& road_edge : forward_road_edges) {
    bool have_left_line = false;
    bool have_right_line = false;
    for (const auto& line_vec : all_lines_) {
      if (line_vec.second.empty()) {
        continue;
      }
      for (const auto& line : line_vec.second) {
        if (line.line->points_size() < 2 ||
            (!line.line->points().empty() &&
             line.line->points().at(0).x() < 0)) {
          continue;
        }
        std::vector<double> widths;
        std::vector<Eigen::Vector3d> line_pts;
        for (const auto& it : line.line->points()) {
          Eigen::Vector3d pt(it.x(), it.y(), it.z());
          line_pts.emplace_back(pt);
        }
        ComputerLineDis(road_edge, line_pts, &widths);
        double avg_width = std::accumulate(widths.begin(), widths.end(), 0.0) /
                           static_cast<double>(widths.size());
        if (IsTatgetOnLineRight(road_edge, line) && avg_width > 3.0) {
          have_right_line = true;
        } else if (!IsTatgetOnLineRight(road_edge, line) && avg_width > 3.0) {
          have_left_line = true;
        }
        if (have_left_line && have_right_line) {
          res = road_edge;
          return res;
        }
      }
    }
  }
  return res;
}

bool GeoOptimization::IsTatgetOnLineRight(
    const std::vector<Eigen::Vector3d>& target_line, const Line_kd& line) {
  int num_thresh = 0, num_calculate = 0;
  for (const auto& point : target_line) {
    if (std::isnan(point.x()) || std::isnan(point.y()) ||
        std::isnan(point.z())) {
      HLOG_ERROR << "found nan point in double_solid_yellow_line";
      continue;
    }
    int dim = 2;
    std::vector<int> nearest_index(dim);
    std::vector<float> nearest_dis(dim);
    std::vector<float> query_point = std::vector<float>{
        static_cast<float>(point.x()), static_cast<float>(point.y())};
    line.line_kdtree->knnSearch(query_point, nearest_index, nearest_dis, dim,
                                cv::flann::SearchParams(-1));
    if (nearest_index.size() < 2) {
      continue;
    }
    Eigen::Vector3d point_e(point.x(), point.y(), point.z());
    Eigen::Vector3d linel1(line.line->points(nearest_index[0]).x(),
                           line.line->points(nearest_index[0]).y(), 0.0);
    Eigen::Vector3d linel2(line.line->points(nearest_index[1]).x(),
                           line.line->points(nearest_index[1]).y(), 0.0);
    if (ComputerPointIsInLine(point_e, linel1, linel2)) {
      num_calculate++;
      if (IsRight(point_e, linel1, linel2)) {
        num_thresh++;
      }
    }
  }
  if (num_calculate < 1 || num_thresh < 1 ||
      (num_calculate != 0 && (static_cast<double>(num_thresh) /
                              static_cast<double>(num_calculate)) < 0.5)) {
    return false;
  }
  return true;
}

void GeoOptimization::HandleOppisiteLine(
    const std::vector<Eigen::Vector3d>& target_line) {
  if (target_line.empty()) {
    return;
  }
  for (auto& line_vector : all_lines_) {
    if (line_vector.second.empty()) {
      continue;
    }
    for (auto& line : line_vector.second) {
      if (line.line->points_size() < 2 ||
          (!line.line->points().empty() && line.line->points().at(0).x() < 0)) {
        continue;
      }
      std::vector<double> widths;
      std::vector<Eigen::Vector3d> line_pts;
      for (const auto& it : line.line->points()) {
        Eigen::Vector3d pt(it.x(), it.y(), it.z());
        line_pts.emplace_back(pt);
      }
      ComputerLineDis(target_line, line_pts, &widths);
      double avg_width = std::accumulate(widths.begin(), widths.end(), 0.0) /
                         static_cast<double>(widths.size());
      if (IsTatgetOnLineRight(target_line, line) && avg_width > 3.0) {
        line.is_ego_road = false;
      }
    }
  }
}

void GeoOptimization::FitLaneLine(
    const std::shared_ptr<hozon::mapping::LaneLine>& pts,
    std::vector<double>* c) {
  // 直接法拟合曲线
  int n = static_cast<int>(pts->points_size()) / 4;  // 按照每4个点来采样
  Eigen::Matrix<double, Eigen::Dynamic, 3> A(n, 3);
  Eigen::VectorXd x(n);
  Eigen::VectorXd b(n);
  for (int i = 0; i < n; i++) {
    double xi = pts->points(i * 4).x();
    double yi = pts->points(i * 4).y();
    A(i, 0) = 1.0;
    A(i, 1) = xi;
    A(i, 2) = xi * xi;
    b[i] = yi;
  }
  x = (A.transpose() * A).inverse() * A.transpose() * b;
  c->at(0) = x[0];
  c->at(1) = x[1];
  c->at(2) = x[2];
}

void GeoOptimization::FitLaneLine(const std::vector<Eigen::Vector3d>& pts,
                                  std::vector<double>* c) {
  // 直接法拟合曲线
  int n = static_cast<int>(pts.size());
  Eigen::Matrix<double, Eigen::Dynamic, 3> A(n, 3);
  Eigen::VectorXd x(n);
  Eigen::VectorXd b(n);
  for (int i = 0; i < n; i++) {
    double xi = pts[i][0];
    double yi = pts[i][1];
    A(i, 0) = 1.0;
    A(i, 1) = xi;
    A(i, 2) = xi * xi;
    b[i] = yi;
  }
  x = (A.transpose() * A).inverse() * A.transpose() * b;
  c->at(0) = x[0];
  c->at(1) = x[1];
  c->at(2) = x[2];
}

Eigen::Vector3d GeoOptimization::FitLaneLine(
    const std::vector<Eigen::Vector3d>& pts) {
  // 直接法拟合曲线
  int n = static_cast<int>(pts.size());
  Eigen::Matrix<double, Eigen::Dynamic, 3> A(n, 3);
  Eigen::VectorXd x(n);
  Eigen::VectorXd b(n);
  for (int i = 0; i < n; i++) {
    double xi = pts[i][0];
    double yi = pts[i][1];
    A(i, 0) = 1.0;
    A(i, 1) = xi;
    A(i, 2) = xi * xi;
    b[i] = yi;
  }
  x = (A.transpose() * A).inverse() * A.transpose() * b;
  Eigen::Vector3d res;
  res[0] = x[0];
  res[1] = x[1];
  res[2] = x[2];
  return res;
}

double GeoOptimization::cubicCurveModel(const Eigen::Vector3d& coefficients,
                                        double x) {
  return coefficients[0] + coefficients[1] * x + coefficients[2] * x * x;
}

// RANSAC拟合3次曲线方程
Eigen::Vector3d GeoOptimization::fitCubicCurveRANSAC(
    const std::vector<Eigen::Vector3d>& data, int maxIterations,
    double threshold) {
  // 随机数生成器
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<int> dist(0, data.size() - 1);

  Eigen::Vector3d bestCoefficients;  // 最佳拟合系数
  int bestInliersCount = 0;

  for (int iteration = 0; iteration < maxIterations; ++iteration) {
    // 随机选择3个样本点
    std::vector<Eigen::Vector3d> sampledPoints;
    for (int i = 0; i < 3; ++i) {
      int index = dist(gen);
      sampledPoints.emplace_back(data[index]);
    }

    // 拟合3次曲线方程
    Eigen::Vector3d coefficients = FitLaneLine(sampledPoints);
    // 计算内点数量
    int inliersCount = 0;
    for (const auto& point : data) {
      double error =
          std::abs(point[1] - cubicCurveModel(coefficients, point[0]));
      if (error < threshold) {
        inliersCount++;
      }
    }

    // 更新最佳拟合参数
    if (inliersCount > bestInliersCount) {
      bestInliersCount = inliersCount;
      bestCoefficients = coefficients;
    }
  }

  return bestCoefficients;
}

double GeoOptimization::LineLength(const std::vector<Eigen::Vector3d>& pts) {
  double length = 0.0;
  int line_size = pts.size();
  if (line_size < 2) return length;
  for (int i = 0; i < line_size - 1; ++i) {
    length += (pts[i + 1] - pts[i]).norm();
  }
  return length;
}

void GeoOptimization::FilterShortLine() {
  for (auto& line : all_lines_) {
    for (auto& line_vector : line.second) {
      if (line_vector.line->points_size() < 20) {
        line_vector.store = false;
      }
    }
  }

  // int short_thresh = 20.0;        // 长度阈值，由于给的是等间距1m
  // double angle_thresh = 6.0;      // 角度阈值
  // std::vector<double> angle_all;  // 所有线的角度
  // for (auto& line : all_lines_) {
  //   for (auto& line_vector : line.second) {
  //     std::vector<double> line_fit(3, 0.0);
  //     FitLaneLine(line_vector.line, &line_fit);
  //     line_vector.param = line_fit;
  //     angle_all.emplace_back(abs(atan(line_fit[1]) / PI * 180.0));
  //   }
  // }
  // std::vector<double> a_tmp = angle_all;
  // sort(angle_all.begin(), angle_all.end());
  // int num = 0;
  // double angel = 0.0;
  // for (int i = 0; i < angle_all.size();) {
  //   double tmp_angel = angle_all[i];
  //   int tmp_num = i;
  //   for (int j = i + 1; j < angle_all.size(); ++j) {
  //     if (angle_all[j] - angle_all[j - 1] > 5) {
  //       tmp_num = j - 1;
  //       break;
  //     }
  //   }
  //   if (num < tmp_num - i + 1) {
  //     num = tmp_num - i + 1;
  //     angel = accumulate(angle_all.begin() + i, angle_all.begin() + tmp_num +
  //     1,
  //                        0.0) /
  //             static_cast<double>(num);
  //     slope_ = angel;
  //   }
  //   if (num > (angle_all.size() - 1) / 2) break;  // 如果已经是众数了，就退出
  //   i = tmp_num + 1;
  // }
  // int i = 0;
  // for (auto& line : all_lines_) {
  //   for (auto& line_vector : line.second) {
  //     if (line_vector.store) {
  //       // double len = LineLength(line_vector.line_points);
  //       int len = line_vector.line->points_size();
  //       // HLOG_ERROR<<"len = "<<len;
  //       if (len < short_thresh && abs(a_tmp[i] - angel) > angle_thresh) {
  //         line_vector.store = false;
  //         HLOG_ERROR << "len = " << len << "  a_tmp[i] = " << a_tmp[i];
  //       }
  //     }
  //     i++;
  //   }
  // }
  // HLOG_ERROR << "accurate angel = " << angel << "  num = " << num << "  ";
}

double GeoOptimization::Dis2Line(const Line_kd& line1, const Line_kd& line2) {
  int num = 0;
  double dis_all = 0.0;
  for (const auto& point : line1.line->points()) {
    int dim = 2;
    std::vector<int> nearest_index(dim);
    std::vector<float> nearest_dis(dim);
    std::vector<float> query_point = std::vector<float>{
        static_cast<float>(point.x()), static_cast<float>(point.y())};
    line2.line_kdtree->knnSearch(query_point, nearest_index, nearest_dis, dim,
                                 cv::flann::SearchParams(-1));
    if (nearest_index.size() < 2) {
      continue;
    }
    Eigen::Vector3d point_e(point.x(), point.y(), 0.0);
    Eigen::Vector3d point11(line2.line->points(nearest_index[0]).x(),
                            line2.line->points(nearest_index[0]).y(), 0.0);
    Eigen::Vector3d point12(line2.line->points(nearest_index[1]).x(),
                            line2.line->points(nearest_index[1]).y(), 0.0);
    if (ComputerPointIsInLine(point_e, point11, point12)) {
      num++;
      dis_all = ComputerPoint2Line(point_e, point11, point12) + dis_all;
    }
  }
  // HLOG_ERROR<<"dis_all = "<<dis_all<<"  num="<<num<<"  dis_all /
  // static_cast<double>(num) = "<<dis_all / static_cast<double>(num);
  if (num > 1) return dis_all / static_cast<double>(num);

  return 0.0;
}

double GeoOptimization::Angledis(double a1, double a2) {
  if (a1 * a2 < 0.0) {
    return std::min(abs(abs(a1 - a2) - 180), abs(a1 - a2));
  }
  return abs(a1 - a2);
}

int GeoOptimization::IsNeighbor(const std::vector<double>& line_param1,
                                const std::vector<double>& line_param2,
                                double b) {
  // abs(abs(line_param1[0] - line_param2[0]) - b) < 1.0 &&
  //    Angledis(line_param1[1], line_param2[1]) < 4 &&
  //    abs(line_param1[2] - line_param2[2]) < 0.1
  if (abs(abs(line_param1[0] - line_param2[0]) - b) < 1.0 &&
      Angledis(line_param1[1], line_param2[1]) < 5 &&
      abs(line_param1[2] - line_param2[2]) < 0.1)
    return 1;
  return 0;
}

void GeoOptimization::ExpandCluster(int dataIndex, int* clusterID,
                                    const Eigen::MatrixXi& distance_mat,
                                    std::vector<int>* label) {
  // 1. region query

  std::deque<int> seeds_index;
  seeds_index.emplace_back(dataIndex);
  for (size_t col = 0; col < distance_mat.cols(); ++col) {
    if (col == dataIndex) {
      continue;
    }
    if (distance_mat(dataIndex, col)) {
      seeds_index.emplace_back(col);
    }
  }
  // 2.label point
  for (size_t i = 0; i < seeds_index.size(); ++i) {
    label->at(seeds_index[i]) = *clusterID;
  }
  // 3. neighbor clusting
  seeds_index.pop_front();
  while (!seeds_index.empty()) {
    auto& row = seeds_index.front();
    for (size_t col = 0; col < distance_mat.cols(); ++col) {
      if (distance_mat(row, col) && label->at(col) == -1) {
        label->at(col) = *clusterID;
        seeds_index.push_back(col);
      }
    }
    seeds_index.pop_front();
  }
  *clusterID = *clusterID + 1;
  return;
}

void GeoOptimization::VizLableLine(const std::vector<int>& index,
                                   const std::vector<int>& lable) {
  if ((!FLAGS_road_recognition_rviz) || (!RVIZ_AGENT.Ok())) {
    return;
  }
  adsfi_proto::viz::MarkerArray markers;
  for (int i = 0; i < index.size(); ++i) {
    std::vector<Eigen::Vector3d> lane_points;
    for (const auto& line : all_lines_[index[i]]) {
      for (const auto& point : line.line->points()) {
        Eigen::Vector3d point_local(point.x(), point.y(), point.z());
        Eigen::Vector3d point_enu = T_U_V_ * point_local;
        lane_points.emplace_back(point_enu);
      }

      // lane_points.insert(lane_points.end(), line.line_points.begin(),
      //                    line.line_points.end());
    }

    adsfi_proto::viz::Marker marker;
    std::vector<float> color;
    if (lable[i] == 1) {
      color = color_palette.find('w')->second;
      PointsToMarker(local_map_->header().publish_stamp(), lane_points, &marker,
                     color);
    } else if (lable[i] == 2) {
      color = color_palette.find('y')->second;
      PointsToMarker(local_map_->header().publish_stamp(), lane_points, &marker,
                     color);
    } else {
      color = color_palette.find('o')->second;
      PointsToMarker(local_map_->header().publish_stamp(), lane_points, &marker,
                     color);
    }
    if (marker.points().size() >= 2) {
      markers.add_markers()->CopyFrom(marker);
    }
    if (!lane_points.empty()) {
      adsfi_proto::viz::Marker marker_id;
      auto id = std::to_string(all_lines_[index[i]].begin()->line->lanepos());
      LineIdToMarker(local_map_->header().publish_stamp(), lane_points[0], id,
                     &marker_id);
      markers.add_markers()->CopyFrom(marker_id);
    }
  }
  RVIZ_AGENT.Publish(KTopicRoadRecognitionLineLable, markers);
}

void GeoOptimization::MergeSplitLine() {
  // // 判断是否是汇入汇出线 mergeline splitline
  // // 想法：
  // // 1.计算每根线的方程y = c0 + c1*x + c2*x*x
  // // 2.聚类线。c1转化为角度angle 距离方程为：||(c01-c02)|-b| <0.5 &&
  // // |angle1-angle2|<5 && |c21-c22|<0.1。b为lanepos间距中位数
  // // 问题：
  // // 线段过短，前后差距大等问题

  // std::vector<std::vector<double>> line_param;  // 参与计算的c0 angel c2
  // std::vector<int> index;   // 参与聚类算法的线中点的index
  // std::vector<int> lable;   // 将线分成几种类型
  // std::vector<double> dis;  // 计算中位数

  // int size_line = all_lines_.size();
  // if (size_line < 2) {
  //   return;
  // }
  // for (auto lines_ = all_lines_.begin(); lines_ != all_lines_.end();
  // lines_++) {
  //   int lanepos1 = static_cast<int>(lines_->second.begin()->line->lanepos());
  //   // HLOG_ERROR << "lanepos1 = " << lanepos1;
  //   int lanepos2 = lanepos1 + 1;
  //   if (lanepos1 == -1) lanepos2++;
  //   if (all_lines_[lanepos2].empty()) break;
  //   auto& lines2_ = all_lines_[lanepos2];
  //   double dist = 0.0;
  //   for (int i = 0; i < lines_->second.size(); i++) {
  //     for (int j = 0; j < lines2_.size(); j++) {
  //       dist = std::max(Dis2Line(lines_->second[i], lines2_[j]), dist);
  //     }
  //   }
  //   // HLOG_ERROR << "########DIS = " << dist;  // 相邻pos两根线的距离
  //   if (dist < 0.1) {
  //     continue;
  //   }
  //   dis.push_back(dist);
  // }

  // double b = 3.2;
  // if (dis.size() > 0) {
  //   std::vector<double> tmp = dis;
  //   sort(tmp.begin(), tmp.end());
  //   b = std::max(tmp[dis.size() / 2], 3.2);  // 不能小于三米 正常车道间距
  // }

  // // HLOG_ERROR<<"b = "<<b;
  // // 计算每根pos的斜率等
  // for (const auto& lines_ : all_lines_) {
  //   std::vector<Eigen::Vector3d> line;
  //   for (const auto& line_vector : lines_.second) {
  //     // for (const auto& point : line_vector.line_points) {
  //     //   if (point.x() > -20 && point.x() < 20) line.emplace_back(point);
  //     // }
  //     line.insert(line.end(), line_vector.line_points.begin(),
  //                 line_vector.line_points.end());
  //   }
  //   if (line.size() < 4) {
  //     continue;
  //   }
  //   // std::vector<double> res(3, 0);
  //   // FitLaneLine(line, &res);  // 直接法拟合直线
  //   Eigen::Vector3d res;
  //   res = fitCubicCurveRANSAC(line, 30, 0.1);  // ransac拟合直线
  //   line_param.push_back({res[0], atan(res[1]) / PI * 180, res[2]});
  //   HLOG_ERROR << "res[0] = " << res[0]
  //              << "   atan(res[1]) / PI * 180=" << atan(res[1]) / PI * 180
  //              << "  res[2] = " << res[2];
  //   index.push_back(lines_.first);
  // }
  // // HLOG_ERROR<<"index.size() = "<<index.size();
  // int index_size = index.size();
  // if (index_size < 2) {
  //   return;
  // }
  // // dbscan算法聚类
  // lable.assign(index_size, -1);  // -1表示没有visit过
  // Eigen::MatrixXi distance_mat = Eigen::MatrixXi::Zero(index_size,
  // index_size); for (int i = 0; i < index_size; ++i) {
  //   for (int j = i; j < index_size; ++j) {
  //     if (j != i) {
  //       distance_mat(i, j) = IsNeighbor(line_param[i], line_param[j], b);
  //       distance_mat(j, i) = distance_mat(i, j);
  //     }
  //   }
  // }
  // //
  // int clusterId = 1;
  // for (size_t i = 0; i < index_size; ++i) {
  //   if (lable[i] != -1) {
  //     continue;
  //   }
  //   ExpandCluster(i, &clusterId, distance_mat, &lable);
  // }
  // HLOG_ERROR << "###############result = " << clusterId - 1;
  // for (size_t i = 0; i < index_size; ++i) {
  //   HLOG_ERROR << "lable[i] = " << lable[i];
  // }
  // VizLableLine(index, lable);
}

void GeoOptimization::ContinueLocalMapUseLine() {
  // 最终存储
  for (auto& line : all_lines_) {
    if (line.second.empty()) {
      continue;
    }

    int size_line = line.second.size();
    // for(int i = size_line - 1; i >=0; --i){
    //   if(line.second[i])
    // }

    for (auto& line_vector : line.second) {
      if (line_vector.store) {
        auto* lane_line = local_map_use_->add_lane_lines();
        lane_line->CopyFrom(*line_vector.line);
      } else {
        last_track_id_.insert(line_vector.line->track_id());
      }
    }
  }
}

// 如果有sdmap再写个重载来实现函数
void GeoOptimization::OnLocalMap(
    const std::shared_ptr<hozon::mapping::LocalMap>& msg) {
  if (msg->lane_lines().empty()) {
    HLOG_ERROR << "lane lines empty!";
    return;
  }
  if (cur_timestamp_ > 0 &&
      msg->header().data_stamp() - cur_timestamp_ < 0.001) {
    HLOG_ERROR << "cur_timestamp_ is near last time stamp!";
    return;
  }
  // 更新时间信息
  cur_timestamp_ = msg->header().data_stamp();
  Eigen::Isometry3d T_U_V;
  {
    std::lock_guard<std::mutex> lock_pose(pose_mtx_);
    T_U_V.setIdentity();
    Eigen::Matrix3d orient = ins_q_w_v_.toRotationMatrix();
    T_U_V.rotate(orient);
    T_U_V.pretranslate(ins_pose_);
    T_U_V_ = T_U_V;
  }
  local_map_ = std::make_shared<hozon::mapping::LocalMap>();
  local_map_->CopyFrom(*msg);

  // visual local map
  // 可视化 local map
  // 函数

  VizLocalMap();

  local_map_use_ = std::make_shared<hozon::mapping::LocalMap>();
  local_map_use_->mutable_header()->CopyFrom(local_map_->header());

  // 目前处理线的想法：
  // 1.
  // 根据横向关系，将可能是自行车道等非机动车道滤除（根据左右车道线的横向距离是否小于三米），构建line_kdtree
  // 2. 根据线段过短等滤除一些杂乱的线
  // 3. 根据前后车道线的关系距离是否过近来融合相邻车道线并填入local_map_use_
  // 4. 处理merge或者split对线段进行拆分，并填入element_map包括line的前后继关系
  // 还是根据具体case分析具体问题
  FilterLocalMapLine(local_map_);

  // 路沿标记本road以外的车道
  // FilterOppositeLine();

  // 处理路口场景逆向车道车道线
  FilterReverseLine();

  // 过滤路口场景或“y”行场景等车道线检测不准导致检测出多条线的情况
  // 车道线相交，长度不长，斜率曲率等与其他线的斜率有一定差距（可以换成车后方的斜率差距）

  FilterIntersectLine();

  // 过滤过短的线段且其斜率不是众数的线段
  //  FilterShortLine();

  // 过滤感知长线交叉的问题

  // 判断长线与长线较为靠前的交点，该情况考虑merge线是否相交的问题
  // 前10个点

  // 汇入车道判断
  // MergeSplitLine();

  // 测试
  ContinueLocalMapUseLine();

  // 构建关联关系及可视化pilotmap等
  // map_lanes_ = std::make_shared<std::vector<LanePilot>>();

  // UpdateLaneByLocalmap(local_map_use_);

  // PridictBackMapLanes();  // 按顺序填入

  // PridictFrontMapLanes();

  // PridictCenterMapLanes();

  // pilot_map_ = std::make_shared<hozon::hdmap::Map>();

  // OnPilotMap();

  // // 可视化pilotmap
  // VizRRMap(pilot_map_);

  // 可视化滤除之后的线
  VizElementMap();

  // TODO(zhangshuo) 对漏检车道线进行补全
  CompleteLocalMap();

  // 将输出填入elementmap
  AppendElemtMap(local_map_use_);
}

void GeoOptimization::CompleteLocalMap() {
  /*
    分场景：
                case1: 简单的缺失场景(缺失整条线)
                *| | |   |*
                *| | |   |*
                *| | |   |*
                *| | |   |*
                      ↑
                case2: split缺失场景（2变3）
                （真实道路）
                | | | | |  |
                | | | | | |
                | | |/ / |
                | | | | |
                | | | | |
                     ↑
                （local map1产生超宽车道）
                | | |   |  |
                | | |   | |
                | | |  / /
                | | | | |
                | | | | |
                     ↑
                （local map2误检）
                | | | | | |  |
                | | | | | | |
                | | | |  / /
                | |   | | |
                | |   | | |
                       ↑
                case3: 中间车道线有缺失
                | | | | | |
                | | | | | |
                 \       /
                  \     /
                   | | |
                   | | |
                   | | |
                   | | |
                      ↑
                case4: 左侧车道线部分缺失
                |    | |
                |    | |
                 \   | |
                  \  | |
                   | | |
                   | | |
                    ↑
                case5: 单边线场景
                |
                |
                |
                |
                |
                |
                  ↑

  */
  /*
    方案设计1:
    1、将车道相关信息存储为哈希表（表中包含lane_pos，相邻车道线距离，车道线到左侧路沿距离和到右测路沿距离）；
    2、依次遍历local_map_中的车道线，评估车道线与相邻车道线的距离是否过大（超宽车道或漏检）；
    3、如果是超宽车道，根据车道线和路沿信息对该超宽车道拟合出一条新的车道线；
    方案设计2:
    1、判断最右车道线到右边路沿的距离和最左车道线到左边路沿的距离；
    2、根据距离判断是否有缺失的车道线；
    3、根据车道线和路沿进行补全；
  */
  if (!local_map_use_) {
    HLOG_ERROR << "local_map_use_ is nullptr";
    return;
  }
  // 更新local_map_use_，原则如下：
  /*
     1、删掉车前方每根线的第一个点大于0的line，并更新lane_pos
     2、删掉车后方每根线的最后一个点小于0的line，并更新lane_pos
  */
  // UpdateLocalMapLine();
  // 存储local map中的line信息，便于查找
  CreateLocalLineTable();
  // 对自车相邻的两个车道线进行补齐
  AlignmentVecLane();
  // // 处理超宽车道
  HandleExtraWideLane();
  // 处理单边线
  HandleSingleSideLine();
}

void GeoOptimization::CreateLocalLineTable() {
  // 存储local map每根车道线的lane_pos和几何
  local_line_table_.clear();
  for (const auto& local_line : local_map_use_->lane_lines()) {
    local_line_info line_info;
    auto lane_pos = local_line.lanepos();
    std::vector<Eigen::Vector3d> local_pts;
    for (const auto& pt : local_line.points()) {
      Eigen::Vector3d point(pt.x(), pt.y(), pt.z());
      local_pts.emplace_back(point);
    }
    line_info.lane_pos = lane_pos;
    line_info.local_line_pts = local_pts;
    local_line_table_.insert_or_assign(lane_pos, line_info);
  }

  // 存储local map中每根道路边沿的几何
  std::vector<Eigen::Vector3d> right_road_pts;
  std::vector<Eigen::Vector3d> left_road_pts;
  for (const auto& local_road : local_map_use_->road_edges()) {
    if (static_cast<int>(local_road.lanepos()) == 1) {
      for (const auto& road_pt : local_road.points()) {
        Eigen::Vector3d pt(road_pt.x(), road_pt.y(), road_pt.z());
        right_road_pts.emplace_back(pt);
      }
    }
    if (static_cast<int>(local_road.lanepos()) == -1) {
      for (const auto& road_pt : local_road.points()) {
        Eigen::Vector3d pt(road_pt.x(), road_pt.y(), road_pt.z());
        left_road_pts.emplace_back(pt);
      }
    }
  }

  for (const auto& local_line : local_map_use_->lane_lines()) {
    auto lane_pos = local_line.lanepos();
    auto right_line_pos =
        static_cast<hozon::mapping::LanePositionType>(lane_pos + 1);
    if (static_cast<int>(lane_pos) + 1 == 0) {
      right_line_pos =
          static_cast<hozon::mapping::LanePositionType>(lane_pos + 2);
    }
    auto left_line_pos =
        static_cast<hozon::mapping::LanePositionType>(lane_pos - 1);
    if (static_cast<int>(lane_pos) - 1 == 0) {
      left_line_pos =
          static_cast<hozon::mapping::LanePositionType>(lane_pos - 2);
    }
    if (local_line_table_.find(right_line_pos) == local_line_table_.end()) {
      local_line_table_.at(lane_pos).flag = 1;
      HLOG_WARN << "right_line_pos not in local_line_table";
      continue;
    }
    if (local_line_table_.find(left_line_pos) == local_line_table_.end()) {
      local_line_table_.at(lane_pos).flag = 0;
    } else {
      local_line_table_.at(lane_pos).flag = 2;
    }
    auto line_pts = local_line_table_.at(lane_pos).local_line_pts;
    auto right_line_pts = local_line_table_.at(right_line_pos).local_line_pts;
    // 存储每根车道线到相邻右车道线的距离
    std::vector<double> line_dis;
    ComputerLineDis(line_pts, right_line_pts, &line_dis);
    local_line_table_.at(lane_pos).right_width = line_dis;
    // 存储每根车道线到右侧路沿的距离
    std::vector<double> line_road_r;
    ComputerLineDis(line_pts, right_road_pts, &line_road_r);
    local_line_table_.at(lane_pos).right_road_width = line_road_r;
    // 存储每根车道线到左侧路沿的距离
    std::vector<double> line_road_l;
    ComputerLineDis(line_pts, left_road_pts, &line_road_l);
    local_line_table_.at(lane_pos).left_road_width = line_road_l;
  }
}

void GeoOptimization::UpdateLocalMapLine() {
  // update table
  if (!local_map_use_ || local_map_use_->lane_lines().empty()) {
    HLOG_ERROR << "local_map_use_ is empty!";
    return;
  }
  std::vector<hozon::mapping::LanePositionType> filt_pos;
  std::vector<hozon::mapping::LanePositionType> local_pos;
  for (const auto& line_info : local_map_use_->lane_lines()) {
    auto line_pos = line_info.lanepos();
    auto line_pts = line_info.points();
    auto line_size = line_pts.size();
    if (line_pts.at(0).x() > 0 || line_pts.at(line_size - 1).x() < 0) {
      filt_pos.emplace_back(line_pos);
      continue;
    }
    local_pos.emplace_back(line_pos);
  }

  // local_map_use_删除不满足要求的车道线
  for (const auto& line_pos : filt_pos) {
    local_map_use_->mutable_lane_lines()->erase(
        std::remove_if(local_map_use_->mutable_lane_lines()->begin(),
                       local_map_use_->mutable_lane_lines()->end(),
                       [&](const hozon::mapping::LaneLine& local_line) {
                         return (local_line.lanepos() == line_pos);
                       }),
        local_map_use_->mutable_lane_lines()->end());
  }

  for (const auto& line_pos : local_pos) {
    int count = 0;
    hozon::mapping::LanePositionType new_pos = line_pos;
    if (static_cast<int>(line_pos) < 0) {
      for (const auto& pos : filt_pos) {
        if (static_cast<int>(line_pos) < static_cast<int>(pos) &&
            static_cast<int>(pos) < 0) {
          count += 1;
        }
      }
      new_pos = static_cast<hozon::mapping::LanePositionType>(line_pos + count);
    }

    if (static_cast<int>(line_pos) > 0) {
      for (const auto& pos : filt_pos) {
        if (static_cast<int>(pos) < static_cast<int>(line_pos) &&
            static_cast<int>(pos) > 0) {
          count += 1;
        }
      }
      new_pos = static_cast<hozon::mapping::LanePositionType>(line_pos - count);
    }

    // update local_map_use_
    for (auto& local_line : *local_map_use_->mutable_lane_lines()) {
      if (local_line.lanepos() == line_pos) {
        local_line.set_lanepos(new_pos);
        break;
      }
    }
  }
}

void GeoOptimization::AlignmentVecLane() {
  // 自车所在的两个车道线对齐
  auto left_pos = static_cast<hozon::mapping::LanePositionType>(-1);
  auto right_pos = static_cast<hozon::mapping::LanePositionType>(1);
  if (local_line_table_.find(left_pos) == local_line_table_.end() ||
      local_line_table_.find(right_pos) == local_line_table_.end()) {
    return;
  }
  auto left_line = local_line_table_.at(left_pos).local_line_pts;
  auto right_line = local_line_table_.at(right_pos).local_line_pts;
  if (left_line.empty() || right_line.empty()) {
    HLOG_ERROR << "line is empty";
    return;
  }
  std::vector<Eigen::Vector3d> new_left_pts;
  std::vector<Eigen::Vector3d> new_right_pts;
  if (left_line.back().x() < right_line.back().x()) {
    // 左侧车道线进行延伸
    if (right_line.size() < 2 || left_line.size() < 2) {
      return;
    }
    const auto& P = left_line.back();
    bool flag = false;
    double dis = 0.;
    Eigen::Vector3d end_dir(-(right_line[1] - right_line[0]).normalized().y(),
                            (right_line[1] - right_line[0]).normalized().x(),
                            0);
    for (int i = 0; i < right_line.size() - 1; ++i) {
      const auto& A = right_line[i];
      const auto& B = right_line[i + 1];
      const auto& AB = B - A;
      const auto& AP = P - A;
      if (flag && (A - B).norm() > 0.1 && dis > 0) {
        // 对点向左预测
        Eigen::Vector3d per_direction(-AB.normalized().y(), AB.normalized().x(),
                                      0);
        auto new_pt = A + per_direction * dis;
        new_left_pts.emplace_back(new_pt);
        end_dir = per_direction;
        continue;
      }
      double ABLengthSquared = AB.squaredNorm();
      if (std::fabs(ABLengthSquared) < 1e-6) {
        continue;
      }
      double t = AB.dot(AP) / ABLengthSquared;
      if (t < -0.01 || t > 1.01) {
        continue;
      }
      // t = std::max(0.0, std::min(t, 1.0));
      auto C = A + t * AB;  // 点到线段的最近点
      dis = (P - C).norm();
      flag = true;
    }
    if (dis > 0) {
      auto new_pt = right_line.back() + end_dir * dis;
      new_left_pts.emplace_back(new_pt);
    }
  } else {
    // 右侧车道线进行延伸
    if (right_line.size() < 2 || left_line.size() < 2) {
      return;
    }
    const auto& P = right_line.back();
    bool flag = false;
    double dis = 0.;
    Eigen::Vector3d end_dir(-(left_line[1] - left_line[0]).normalized().y(),
                            (left_line[1] - left_line[0]).normalized().x(), 0);
    for (int i = 0; i < left_line.size() - 1; ++i) {
      const auto& A = left_line[i];
      const auto& B = left_line[i + 1];
      const auto& AB = B - A;
      const auto& AP = P - A;
      if (flag && (A - B).norm() > 0.1 && dis > 0) {
        // 对点向左预测
        Eigen::Vector3d per_direction(-AB.normalized().y(), AB.normalized().x(),
                                      0);
        auto new_pt = A - per_direction * dis;
        new_right_pts.emplace_back(new_pt);
        end_dir = per_direction;
      }
      double ABLengthSquared = AB.squaredNorm();
      if (std::fabs(ABLengthSquared) < 1e-6) {
        continue;
      }
      double t = AB.dot(AP) / ABLengthSquared;
      if (t < -0.01 || t > 1.01) {
        continue;
      }
      // t = std::max(0.0, std::min(t, 1.0));
      auto C = A + t * AB;  // 点到线段的最近点
      dis = (P - C).norm();
      flag = true;
    }
    if (dis > 0) {
      auto new_pt = left_line.back() - end_dir * dis;
      new_right_pts.emplace_back(new_pt);
    }
  }

  for (auto& local_line : *local_map_use_->mutable_lane_lines()) {
    if (local_line.lanepos() == left_pos && !new_left_pts.empty()) {
      for (const auto& pt : new_left_pts) {
        auto* new_pt = local_line.add_points();
        new_pt->set_x(pt.x());
        new_pt->set_y(pt.y());
        new_pt->set_z(pt.z());
        local_line_table_.at(left_pos).local_line_pts.emplace_back(pt);
      }
    }
    if (local_line.lanepos() == right_pos && !new_right_pts.empty()) {
      for (const auto& pt : new_right_pts) {
        auto* new_pt = local_line.add_points();
        new_pt->set_x(pt.x());
        new_pt->set_y(pt.y());
        new_pt->set_z(pt.z());
        local_line_table_.at(right_pos).local_line_pts.emplace_back(pt);
      }
    }
  }
}

void GeoOptimization::ComputerLineDis(
    const std::vector<Eigen::Vector3d>& line_pts,
    const std::vector<Eigen::Vector3d>& right_line_pts,
    std::vector<double>* line_dis) {
  // 计算线与线之间距离
  if (line_pts.empty() || right_line_pts.empty()) {
    return;
  }
  std::vector<double> val_dis;
  for (const auto& P : line_pts) {
    for (size_t i = 1; i < right_line_pts.size(); i++) {
      const auto& A = right_line_pts[i - 1];
      const auto& B = right_line_pts[i];
      const auto& AB = B - A;
      const auto& AP = P - A;
      double ABLengthSquared = AB.squaredNorm();
      if (std::fabs(ABLengthSquared) < 1e-6) {
        continue;
      }
      double t = AB.dot(AP) / ABLengthSquared;
      if (t < 0 || t > 1) {
        continue;
      }
      t = std::max(0.0, std::min(t, 1.0));
      auto C = A + t * AB;  // 点到线段的最近点
      val_dis.emplace_back((P - C).norm());
      break;
    }
  }
  double sum = std::accumulate(val_dis.begin(), val_dis.end(), 0.0);
  double mean = sum / static_cast<double>(val_dis.size());
  *line_dis = val_dis;
}

void GeoOptimization::HandleExtraWideLane() {
  // 处理超宽车道（漏检）场景
  if (!local_map_use_ || local_map_use_->lane_lines_size() == 0) {
    return;
  }
  std::vector<hozon::mapping::LanePositionType> extra_pos;
  for (const auto& local_line : local_map_use_->lane_lines()) {
    if (local_line_table_.find(local_line.lanepos()) ==
        local_line_table_.end()) {
      continue;
    }
    auto lane_width = local_line_table_.at(local_line.lanepos()).right_width;
    // 检测是否有超宽（漏检）车道
    bool width_flag = false;
    for (const auto& wid : lane_width) {
      if (wid >= 5.5) {
        HLOG_WARN << "have extra wide lanes!";
        width_flag = true;  // 漏检
        break;
      }
    }
    // 这里是否要考虑超宽车道是否是在车的两侧
    if (width_flag && static_cast<int>(local_line.lanepos()) == -1) {
      extra_pos.emplace_back(local_line.lanepos());
    }
  }
  // 针对超宽（漏检）车道进行拟合漏检车道线
  for (const auto& ex : extra_pos) {
    FitMissedLaneLine(ex);
  }
}

void GeoOptimization::FitMissedLaneLine(
    const hozon::mapping::LanePositionType& ex) {
  // 拟合漏检车道线
  if (local_line_table_.find(ex) == local_line_table_.end()) {
    HLOG_ERROR << "ex not in local_line_table_";
    return;
  }
  auto right_ex = static_cast<hozon::mapping::LanePositionType>(ex + 1);
  if (ex + 1 == 0) {
    right_ex = static_cast<hozon::mapping::LanePositionType>(ex + 2);
  }
  if (local_line_table_.find(right_ex) == local_line_table_.end()) {
    HLOG_ERROR << "right_ex not in local_line_table_";
  }
  auto ex_pts = local_line_table_.at(ex).local_line_pts;
  auto right_ex_pts = local_line_table_.at(right_ex).local_line_pts;

  // 根据车离两侧线的距离来确认base线
  auto left_dis = ComputeVecToLaneDis(ex_pts);
  auto right_dis = ComputeVecToLaneDis(right_ex_pts);
  std::vector<std::vector<Eigen::Vector3d>> new_lines;
  std::vector<Eigen::Vector3d> base_line;
  std::vector<double> base_width;
  if (left_dis < right_dis) {
    // vehicle left line is baseline
    for (const auto& P : ex_pts) {
      for (size_t i = 1; i < right_ex_pts.size(); i++) {
        const auto& A = right_ex_pts[i - 1];
        const auto& B = right_ex_pts[i];
        const auto& AB = B - A;
        const auto& AP = P - A;
        double ABLengthSquared = AB.squaredNorm();
        if (std::fabs(ABLengthSquared) < 1e-6) {
          continue;
        }
        double t = AB.dot(AP) / ABLengthSquared;
        if (t < 0 || t > 1) {
          continue;
        }
        t = std::max(0.0, std::min(t, 1.0));
        auto C = A + t * AB;  // 点到线段的最近点
        if ((P - C).norm() > 5.5) {
          base_line.emplace_back(P);
        } else {
          base_width.emplace_back((P - C).norm());
        }
        break;
      }
    }
    int lane_num = std::ceil(left_dis / 3.75);
    double lane_width = 3.75;
    if (!base_width.empty()) {
      lane_width = base_width.back();
      // double lane_width =
      //     std::reduce(base_width.begin(), base_width.end()) /
      //     base_width.size();
    }
    FitSingleSideLine(&base_line, &new_lines, lane_num, true, 0, lane_width);
  } else {
    // vecicle right line is baseline
    for (const auto& P : right_ex_pts) {
      for (size_t i = 1; i < ex_pts.size(); i++) {
        const auto& A = ex_pts[i - 1];
        const auto& B = ex_pts[i];
        const auto& AB = B - A;
        const auto& AP = P - A;
        double ABLengthSquared = AB.squaredNorm();
        if (std::fabs(ABLengthSquared) < 1e-6) {
          continue;
        }
        double t = AB.dot(AP) / ABLengthSquared;
        if (t < 0 || t > 1) {
          continue;
        }
        t = std::max(0.0, std::min(t, 1.0));
        auto C = A + t * AB;  // 点到线段的最近点
        if ((P - C).norm() > 5.0) {
          base_line.emplace_back(P);
        } else {
          base_width.emplace_back((P - C).norm());
        }
        break;
      }
    }
    int lane_num = std::ceil(right_dis / 3.75);
    double lane_width = 3.75;
    if (!base_width.empty()) {
      lane_width = base_width.back();
      // double lane_width =
      //     std::reduce(base_width.begin(), base_width.end()) /
      //     base_width.size();
    }
    FitSingleSideLine(&base_line, &new_lines, lane_num, false, 0, lane_width);
  }

  if (new_lines.empty()) {
    HLOG_WARN << "fit new line failed";
    return;
  }
  for (const auto& line : new_lines) {
    auto* new_line = local_map_use_->add_lane_lines();
    for (const auto& pt : line) {
      auto* new_pt = new_line->add_points();
      new_pt->set_x(pt.x());
      new_pt->set_y(pt.y());
      new_pt->set_z(pt.z());
    }

    new_line->set_lanepos(
        static_cast<hozon::mapping::LanePositionType>(extra_val));
    new_line->set_track_id(extra_val);
    extra_val++;
    new_line->set_color(static_cast<hozon::mapping::Color>(1));
    // new_line->set_allocated_lane_param(static_cast<hozon::mapping::LaneCubicSpline>(500));
    // new_line->set_confidence(static_cast<hozon::mapping::LanePositionType>(500));
    new_line->set_lanetype(static_cast<hozon::mapping::LaneType>(2));
  }
}

void GeoOptimization::HandleSingleSideLine() {
  // handle single side line
  /*
    1、首先判断车的两侧是否只有一侧有车道线
    2、如果是，找到离车最近的车道线或路沿
    3、根据单侧车道线和路沿预测车道线
  */
  std::vector<int> left_lanepos;
  std::vector<int> right_lanepos;
  std::vector<hozon::mapping::LanePositionType> lane_pos;
  for (const auto& local_line : local_map_use_->lane_lines()) {
    lane_pos.emplace_back(local_line.lanepos());
    if (local_line.points().size() < 2) {
      continue;
    }
    if (static_cast<int>(local_line.lanepos()) < 0) {
      left_lanepos.emplace_back(static_cast<int>(local_line.lanepos()));
    } else if (static_cast<int>(local_line.lanepos()) > 0 &&
               static_cast<int>(local_line.lanepos()) != 99) {
      right_lanepos.emplace_back(static_cast<int>(local_line.lanepos()));
    }
  }
  std::vector<Eigen::Vector3d> right_road_pts;
  std::vector<Eigen::Vector3d> left_road_pts;
  for (const auto& local_road : local_map_->road_edges()) {
    if (static_cast<int>(local_road.lanepos()) == 1) {
      for (const auto& road_pt : local_road.points()) {
        Eigen::Vector3d pt(road_pt.x(), road_pt.y(), road_pt.z());
        right_road_pts.emplace_back(pt);
      }
    }
    if (static_cast<int>(local_road.lanepos()) == -1) {
      for (const auto& road_pt : local_road.points()) {
        Eigen::Vector3d pt(road_pt.x(), road_pt.y(), road_pt.z());
        left_road_pts.emplace_back(pt);
      }
    }
  }

  if (!right_road_pts.empty()) {
    std::sort(right_road_pts.begin(), right_road_pts.end(),
              [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
                return a.x() < b.x();
              });
  }

  if (!left_road_pts.empty()) {
    std::sort(left_road_pts.begin(), left_road_pts.end(),
              [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
                return a.x() < b.x();
              });
  }
  // 几种情况
  /*
    1、右侧无车道线 {
      左侧仅有车道线-->使用左车道线进行预测
      左侧仅有路沿 {
        若右侧无路沿-->使用左侧路沿进行预测
        若右侧有路沿-->使用离车最近的路沿进行预测
      }
      左侧有车道线和路沿-->使用左车道线进行预测
    }
    2、左侧无车道线 {
      右侧仅有车道线-->使用右车道线进行预测
      右侧仅有路沿 {
        若左侧无路沿-->使用右侧路沿进行预测
        若左侧有路沿-->使用离车最近的路沿进行预测
      }
      右侧有车道线和路沿-->使用右车道线进行预测
    }
  */
  std::vector<std::vector<Eigen::Vector3d>> new_lines;
  if (right_lanepos.empty() && !right_road_pts.empty()) {
    // 车右侧车道线为空，但路沿不为空，可以进行预测，但预测的车道线不能超过路沿
    if (!left_lanepos.empty()) {
      // use the nearest line to predict
      auto max_element_iter =
          std::max_element(left_lanepos.begin(), left_lanepos.end());
      auto max_pos =
          static_cast<hozon::mapping::LanePositionType>(*max_element_iter);
      if (local_line_table_.find(max_pos) == local_line_table_.end()) {
        return;
      }
      auto max_line = local_line_table_.at(max_pos).local_line_pts;
      // compute lane nums
      int lane_num = std::ceil(ComputeVecToLaneDis(max_line) / 3.5);
      FitSingleSideLine(&max_line, &new_lines, lane_num, true, 0, 3.5);
      // 检测预测的车道线是否会超过路沿，若超过路沿，则使用路沿线当做预测的边线
      if (!new_lines.empty() &&
          JudgeLineOverRoad(new_lines.back(), right_road_pts)) {
        for (auto& pt : right_road_pts) {
          pt.y() = pt.y() + 0.1;
        }
        new_lines.back() = right_road_pts;
      }
    } else if (left_lanepos.empty() && !left_road_pts.empty()) {
      if (std::abs(left_road_pts.front().y()) <
          std::abs(right_road_pts.front().y())) {
        int lane_num = std::ceil(ComputeVecToLaneDis(left_road_pts) / 3.5);
        if (left_road_pts.front().x() > 0 || left_road_pts.back().x() < 0 ||
            !lane_pos.empty()) {
          return;
        }
        FitSingleSideLine(&left_road_pts, &new_lines, lane_num, true, 1, 3.5);
        if (!new_lines.empty() &&
            JudgeLineOverRoad(new_lines.back(), right_road_pts)) {
          for (auto& pt : right_road_pts) {
            pt.y() = pt.y() + 0.1;
          }
          new_lines.back() = right_road_pts;
        }
      } else {
        int lane_num = std::ceil(ComputeVecToLaneDis(right_road_pts) / 3.5);
        if (right_road_pts.front().x() > 0 || right_road_pts.back().x() < 0 ||
            !lane_pos.empty()) {
          return;
        }
        FitSingleSideLine(&right_road_pts, &new_lines, lane_num, false, 2, 3.5);
        // 检测预测的车道线是否会超过路沿，若超过路沿，则使用路沿线当做预测的边线
        if (!new_lines.empty() &&
            JudgeLineOverRoad(new_lines.back(), left_road_pts)) {
          for (auto& pt : left_road_pts) {
            pt.y() = pt.y() - 0.1;
          }
          new_lines.back() = left_road_pts;
        }
      }
    }
  }

  if (left_lanepos.empty() && !left_road_pts.empty()) {
    if (!right_lanepos.empty()) {
      // use the nearest line to predict
      auto min_element_iter =
          std::min_element(right_lanepos.begin(), right_lanepos.end());
      auto min_pos =
          static_cast<hozon::mapping::LanePositionType>(*min_element_iter);
      if (local_line_table_.find(min_pos) == local_line_table_.end()) {
        HLOG_ERROR << "max_pos not in local_line_table";
        return;
      }
      auto min_line = local_line_table_.at(min_pos).local_line_pts;
      int lane_num = std::ceil(ComputeVecToLaneDis(min_line) / 3.5);
      FitSingleSideLine(&min_line, &new_lines, lane_num, false, 0, 3.5);
      // 检测预测的车道线是否会超过路沿，若超过路沿，则使用路沿线当做预测的边线
      if (!new_lines.empty() &&
          JudgeLineOverRoad(new_lines.back(), left_road_pts)) {
        for (auto& pt : left_road_pts) {
          pt.y() = pt.y() - 0.1;
        }
        new_lines.back() = left_road_pts;
      }
    }
  }

  if (new_lines.empty()) {
    HLOG_ERROR << "fit new line failed";
    return;
  }

  // 将虚拟的路沿线设置为实线，非路沿线设置为虚线
  for (size_t i = 0; i < new_lines.size(); ++i) {
    auto* new_line = local_map_use_->add_lane_lines();
    for (const auto& pt : new_lines[i]) {
      auto* new_pt = new_line->add_points();
      new_pt->set_x(pt.x());
      new_pt->set_y(pt.y());
      new_pt->set_z(pt.z());
    }

    new_line->set_lanepos(
        static_cast<hozon::mapping::LanePositionType>(extra_val));
    new_line->set_track_id(extra_val);
    extra_val++;
    new_line->set_color(static_cast<hozon::mapping::Color>(1));
    // new_line->set_allocated_lane_param(static_cast<hozon::mapping::LaneCubicSpline>(500));
    // new_line->set_confidence(static_cast<hozon::mapping::LanePositionType>(500));
    if (i == new_lines.size() - 1) {
      new_line->set_lanetype(static_cast<hozon::mapping::LaneType>(1));
      continue;
    }
    new_line->set_lanetype(static_cast<hozon::mapping::LaneType>(2));
  }
}

double GeoOptimization::ComputeVecToLaneDis(
    const std::vector<Eigen::Vector3d>& base_line) {
  // compute lane nums;
  if (base_line.empty()) {
    return 0;
  }
  Eigen::Vector3d P(0., 0., 0.);
  double dis = 0.;
  std::vector<double> distance;
  for (size_t i = 1; i < base_line.size(); i++) {
    const auto& A = base_line[i - 1];
    const auto& B = base_line[i];
    const auto& AB = B - A;
    const auto& AP = P - A;
    double ABLengthSquared = AB.squaredNorm();
    if (std::fabs(ABLengthSquared) < 1e-6) {
      continue;
    }
    double t = AB.dot(AP) / ABLengthSquared;
    if (t < 0 || t > 1) {
      continue;
    }
    t = std::max(0.0, std::min(t, 1.0));
    auto C = A + t * AB;  // 点到线段的最近点
    dis = (P - C).norm();
    distance.emplace_back(dis);
  }
  if (distance.empty()) {
    return dis;
  } else {
    auto min_distance = std::min_element(distance.begin(), distance.end());
    return *min_distance;
  }
}

void GeoOptimization::FitSingleSideLine(
    std::vector<Eigen::Vector3d>* base_line,
    std::vector<std::vector<Eigen::Vector3d>>* new_lines, const int& lane_num,
    const bool& flag, const int& is_boundary, const double& lane_width) {
  // use base line to fit
  if (base_line->empty()) {
    HLOG_ERROR << "base line is empty";
    return;
  }
  std::sort(base_line->begin(), base_line->end(),
            [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
              return a.x() < b.x();
            });
  std::vector<std::vector<Eigen::Vector3d>> new_pts(lane_num);

  // boundary
  if (is_boundary == 1 || is_boundary == 2) {
    auto* new_line = local_map_use_->add_lane_lines();
    for (const auto& pt : *base_line) {
      auto* new_pt = new_line->add_points();
      new_pt->set_x(pt.x());
      if (is_boundary == 1) {
        new_pt->set_y(pt.y() - 0.1);
      }
      if (is_boundary == 2) {
        new_pt->set_y(pt.y() + 0.1);
      }
      // new_pt->set_y(pt.y());
      new_pt->set_z(pt.z());
    }

    new_line->set_lanepos(
        static_cast<hozon::mapping::LanePositionType>(extra_val));
    new_line->set_track_id(extra_val);
    extra_val++;
    new_line->set_color(static_cast<hozon::mapping::Color>(1));
    // new_line->set_allocated_lane_param(static_cast<hozon::mapping::LaneCubicSpline>(500));
    // new_line->set_confidence(static_cast<hozon::mapping::LanePositionType>(500));
    new_line->set_lanetype(static_cast<hozon::mapping::LaneType>(1));
  }

  if (flag) {
    // predict the right line
    for (size_t i = 0; i < base_line->size() - 1; ++i) {
      if (base_line->at(i + 1) == base_line->at(i) ||
          base_line->at(i + 1).x() == base_line->at(i).x() ||
          (base_line->at(i + 1) - base_line->at(i)).norm() < 0.1) {
        continue;
      }
      auto direction = (base_line->at(i + 1) - base_line->at(i)).normalized();
      Eigen::Vector3d per_direction(-direction.y(), direction.x(), 0);
      auto p = base_line->at(i);
      for (int i = 0; i < lane_num; i++) {
        auto new_pt = p - per_direction * lane_width * (i + 1);
        new_pts[i].emplace_back(new_pt);
      }
    }
  } else {
    // predict the left line
    for (size_t i = 0; i < base_line->size() - 1; ++i) {
      if (base_line->at(i + 1) == base_line->at(i) ||
          base_line->at(i + 1).x() == base_line->at(i).x() ||
          (base_line->at(i + 1) - base_line->at(i)).norm() < 0.1) {
        continue;
      }
      auto direction = (base_line->at(i + 1) - base_line->at(i)).normalized();
      Eigen::Vector3d per_direction(-direction.y(), direction.x(), 0);
      auto p = base_line->at(i + 1);
      for (int i = 0; i < lane_num; i++) {
        auto new_pt = p + per_direction * lane_width * (i + 1);
        new_pts[i].emplace_back(new_pt);
      }
    }
  }
  *new_lines = new_pts;
}

bool GeoOptimization::JudgeLineOverRoad(
    const std::vector<Eigen::Vector3d>& lane_line,
    const std::vector<Eigen::Vector3d>& road_line) {
  // 判断预测的边线是否超过路边沿(是否交叉)
  if (lane_line.empty() || road_line.empty()) {
    return false;
  }
  double lane_dis = ComputeVecToLaneDis(lane_line);
  double road_dis = ComputeVecToLaneDis(road_line);

  // bool flag2 = false;
  // for (size_t i = 0; i < lane_line.size() - 1; ++i) {
  //   for (size_t j = 0; j < road_line.size() - 1; ++j) {
  //     auto p1 = lane_line[i];
  //     auto q1 = lane_line[i + 1];
  //     auto p2 = road_line[i];
  //     auto q2 = road_line[i + 1];
  //     auto r = q1 - p1;
  //     auto s = q2 - p2;
  //     double r_cross_s = r.cross(s).norm();
  //     if (r_cross_s < 1e-9) {
  //       // 线段平行或共线
  //       continue;
  //     }
  //     auto q_minus_p = q2 - p1;
  //     double t = q_minus_p.cross(s).norm() / r_cross_s;
  //     double u = q_minus_p.cross(r).norm() / r_cross_s;
  //     if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
  //       // 有交叉
  //       flag2 = true;
  //       break;
  //     }
  //   }
  // }
  // if (flag1 && !flag2) {
  //   return true;
  // } else {
  //   return false;
  // }

  return lane_dis > road_dis;
}

void GeoOptimization::AppendElemtMap(
    const std::shared_ptr<hozon::mapping::LocalMap>& msg) {
  // 将元素填充进element
  int32_t node_name = 1, centerline_name = 1, arrows_name = 1, stop_lines = 1;

  elem_map_ = std::make_shared<hozon::mp::mf::em::ElementMap>();
  elem_map_->map_info.stamp = msg->header().data_stamp();
  for (const auto& lane_line_it : msg->lane_lines()) {
    // 过滤空的点
    if (lane_line_it.points_size() <= 0) {
      continue;
    }
    hozon::mp::mf::em::Boundary lane_line;

    auto point_size = lane_line_it.points().size();
    Eigen::Vector3f last_point;  // 为了把太近的点过滤掉
    int last_point_exisit = 0;
    for (const auto& line_point_it : lane_line_it.points()) {
      if (std::isnan(line_point_it.x()) || std::isnan(line_point_it.y()) ||
          std::isnan(line_point_it.z())) {
        HLOG_ERROR << "found nan in localmap";
        continue;
      }
      Eigen::Vector3f point_local(line_point_it.x(), line_point_it.y(),
                                  line_point_it.z());
      if (last_point_exisit == 0) {
        last_point = point_local;
        last_point_exisit = 1;
      } else {
        if ((last_point - point_local).norm() < 0.1) {
          continue;
        }
        last_point = point_local;
      }
      // Eigen::Vector3d point_enu = T_U_V_ * point_local;
      hozon::mp::mf::em::BoundaryNode node;
      node.point = point_local;
      node.id = node_name;
      elem_map_->boundary_nodes[node_name] =
          std::make_shared<hozon::mp::mf::em::BoundaryNode>(node);
      lane_line.nodes.push_back(elem_map_->boundary_nodes[node_name]);
      node_name++;
    }
    lane_line.id = lane_line_it.track_id();
    for (const auto& delete_id : lane_line_it.deleted_track_ids()) {
      lane_line.delete_ids.emplace_back(delete_id);
    }
    FillLanePos(&lane_line, lane_line_it.lanepos());
    FillLaneType(&lane_line, lane_line_it.lanetype());
    FillLaneColor(&lane_line, lane_line_it.color());
    for (const auto& line :
         all_lines_[static_cast<int>(lane_line_it.lanepos())]) {
      if (line.line->track_id() == lane_line_it.track_id()) {
        Fillego(&lane_line, line.is_ego_road);
        break;
      }
    }

    // lane_line.boundary_type = lane_line_it.lanetype();
    elem_map_->boundaries[lane_line.id] =
        std::make_shared<hozon::mp::mf::em::Boundary>(lane_line);
  }

  // stop_line
  for (const auto& i : local_map_->stop_lines()) {
    em::StopLine stopline;
    stopline.id = i.track_id();
    em::Point slpt(i.left_point().x(), i.left_point().y(), i.left_point().z());
    stopline.points.emplace_back(slpt);
    em::Point slpt2(i.right_point().x(), i.right_point().y(),
                    i.right_point().z());
    stopline.points.emplace_back(slpt2);
    elem_map_->stop_lines[stopline.id] =
        std::make_shared<em::StopLine>(stopline);
  }

  // arrow
  for (const auto& i : local_map_->arrows()) {
    if (i.points().point_size() != 4) {
      continue;
    }
    em::Arrow arw;
    arw.id = i.track_id();
    FillArrowType(&arw, i.arrow_type());
    for (const auto& pt : i.points().point()) {
      em::Point arpt(pt.x(), pt.y(), pt.z());
      arw.polygon.points.emplace_back(arpt);
    }
    arw.heading = i.heading();
    elem_map_->arrows[arw.id] = std::make_shared<em::Arrow>(arw);
  }

  // zebra
  for (const auto& i : local_map_->cross_walks()) {
    if (i.points().point_size() != 4) {
      continue;
    }
    em::CrossWalk crw;
    crw.id = i.track_id();
    for (const auto& pt : i.points().point()) {
      em::Point crwpt(pt.x(), pt.y(), pt.z());
      crw.polygon.points.emplace_back(crwpt);
    }
    elem_map_->cross_walks[crw.id] = std::make_shared<em::CrossWalk>(crw);
  }
}

void GeoOptimization::Fillego(hozon::mp::mf::em::Boundary* lane_line,
                              bool is_ego) {
  if (is_ego) {
    lane_line->is_ego = em::IsEgo::Ego_Road;
  } else {
    lane_line->is_ego = em::IsEgo::Other_Road;
  }
}

void GeoOptimization::FillArrowType(em::Arrow* arrow,
                                    hozon::hdmap::ArrowData_Type arrowtype) {
  switch (arrowtype) {
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_UNKNOWN_TURN:
      arrow->type = em::ArrowType::UNKNOWN_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_STRAIGHT:
      arrow->type = em::ArrowType::STRAIGHT_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_RIGHT_TURN:
      arrow->type = em::ArrowType::RIGHT_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_LEFT_TURN:
      arrow->type = em::ArrowType::LEFT_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_U_TURN:
      arrow->type = em::ArrowType::U_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_STRAIGHT_LEFT_TURN:
      arrow->type = em::ArrowType::STRAIGHT_LEFT_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_STRAIGHT_RIGHT_TURN:
      arrow->type = em::ArrowType::STRAIGHT_RIGHT_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_STRAIGHT_U_TURN:
      arrow->type = em::ArrowType::STRAIGHT_U_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_LEFT_U_TURN:
      arrow->type = em::ArrowType::LEFT_U_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_LEFT_RIGHT_TURN:
      arrow->type = em::ArrowType::LEFT_RIGHT_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_LEFT_FRONT_TURN:
      arrow->type = em::ArrowType::LEFT_FRONT_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_RIGHT_FRONT_TURN:
      arrow->type = em::ArrowType::RIGHT_FRONT_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_STRAIGHT_LEFT_RIGHT_TURN:
      arrow->type = em::ArrowType::STRAIGHT_LEFT_RIGHT_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_STRAIGHT_LEFT_U_TURN:
      arrow->type = em::ArrowType::STRAIGHT_LEFT_U_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_RIGHT_U_TURN:
      arrow->type = em::ArrowType::RIGHT_U_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_FORBID_LEFT_TURN:
      arrow->type = em::ArrowType::FORBID_LEFT_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_FORBID_RIGHT_TURN:
      arrow->type = em::ArrowType::FORBID_RIGHT_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_FORBID_U_TURN:
      arrow->type = em::ArrowType::FORBID_U_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_FRONT_NEAR_CROSSWALK:
      arrow->type = em::ArrowType::FRONT_NEAR_CROSSWALK_ARROW;
      break;
    default:
      break;
  }
}

void GeoOptimization::FillLanePos(
    hozon::mp::mf::em::Boundary* lane_line,
    hozon::mapping::LanePositionType lanepostype) {
  switch (lanepostype) {
    case hozon::mapping::LanePositionType::LanePositionType_BOLLARD_LEFT:
      lane_line->lanepos =
          hozon::mp::mf::em::LanePos::LanePositionType_BOLLARD_LEFT;
      break;
    case hozon::mapping::LanePositionType::LanePositionType_FOURTH_LEFT:
      lane_line->lanepos =
          hozon::mp::mf::em::LanePos::LanePositionType_FOURTH_LEFT;
      break;
    case hozon::mapping::LanePositionType::LanePositionType_THIRD_LEFT:
      lane_line->lanepos =
          hozon::mp::mf::em::LanePos::LanePositionType_THIRD_LEFT;
      break;
    case hozon::mapping::LanePositionType::LanePositionType_ADJACENT_LEFT:
      lane_line->lanepos =
          hozon::mp::mf::em::LanePos::LanePositionType_ADJACENT_LEFT;
      break;
    case hozon::mapping::LanePositionType::LanePositionType_EGO_LEFT:
      lane_line->lanepos =
          hozon::mp::mf::em::LanePos::LanePositionType_EGO_LEFT;
      break;
    case hozon::mapping::LanePositionType::LanePositionType_EGO_RIGHT:
      lane_line->lanepos =
          hozon::mp::mf::em::LanePos::LanePositionType_EGO_RIGHT;
      break;
    case hozon::mapping::LanePositionType::LanePositionType_ADJACENT_RIGHT:
      lane_line->lanepos =
          hozon::mp::mf::em::LanePos::LanePositionType_ADJACENT_RIGHT;
      break;
    case hozon::mapping::LanePositionType::LanePositionType_THIRD_RIGHT:
      lane_line->lanepos =
          hozon::mp::mf::em::LanePos::LanePositionType_THIRD_RIGHT;
      break;
    case hozon::mapping::LanePositionType::LanePositionType_FOURTH_RIGHT:
      lane_line->lanepos =
          hozon::mp::mf::em::LanePos::LanePositionType_FOURTH_RIGHT;
      break;
    case hozon::mapping::LanePositionType::LanePositionType_BOLLARD_RIGHT:
      lane_line->lanepos =
          hozon::mp::mf::em::LanePos::LanePositionType_BOLLARD_RIGHT;
      break;
    default:
      lane_line->lanepos = hozon::mp::mf::em::LanePos::LanePositionType_OTHER;
      break;
  }
}

void GeoOptimization::FillLaneColor(hozon::mp::mf::em::Boundary* lane_line,
                                    hozon::mapping::Color lanecolor) {
  switch (lanecolor) {
    case hozon::mapping::Color::UNKNOWN:
      lane_line->color = em::UNKNOWN_COLOR;
      break;
    case hozon::mapping::Color::WHITE:
      lane_line->color = em::WHITE;
      break;
    case hozon::mapping::Color::YELLOW:
      lane_line->color = em::YELLOW;
      break;
    case hozon::mapping::Color::GREEN:
      lane_line->color = em::GREEN;
      break;
    case hozon::mapping::Color::RED:
      lane_line->color = em::RED;
      break;
    case hozon::mapping::Color::BLACK:
      lane_line->color = em::BLACK;
      break;
    default:
      break;
  }
}

void GeoOptimization::FillLaneType(hozon::mp::mf::em::Boundary* lane_line,
                                   hozon::mapping::LaneType lanetype) {
  switch (lanetype) {
    case hozon::mapping::LaneType::LaneType_UNKNOWN:
      lane_line->linetype = hozon::mp::mf::em::LineType::LaneType_UNKNOWN;
      break;
    case hozon::mapping::LaneType::LaneType_SOLID:
      lane_line->linetype = hozon::mp::mf::em::LineType::LaneType_SOLID;
      break;
    case hozon::mapping::LaneType::LaneType_DASHED:
      lane_line->linetype = hozon::mp::mf::em::LineType::LaneType_DASHED;
      break;
    case hozon::mapping::LaneType::LaneType_SHORT_DASHED:
      lane_line->linetype = hozon::mp::mf::em::LineType::LaneType_SHORT_DASHED;
      break;
    case hozon::mapping::LaneType::LaneType_DOUBLE_SOLID:
      lane_line->linetype = hozon::mp::mf::em::LineType::LaneType_DOUBLE_SOLID;
      break;
    case hozon::mapping::LaneType::LaneType_DOUBLE_DASHED:
      lane_line->linetype = hozon::mp::mf::em::LineType::LaneType_DOUBLE_DASHED;
      break;
    case hozon::mapping::LaneType::LaneType_LEFT_SOLID_RIGHT_DASHED:
      lane_line->linetype =
          hozon::mp::mf::em::LineType::LaneType_LEFT_SOLID_RIGHT_DASHED;
      break;
    case hozon::mapping::LaneType::LaneType_RIGHT_SOLID_LEFT_DASHED:
      lane_line->linetype =
          hozon::mp::mf::em::LineType::LaneType_RIGHT_SOLID_LEFT_DASHED;
      break;
    case hozon::mapping::LaneType::LaneType_SHADED_AREA:
      lane_line->linetype = hozon::mp::mf::em::LineType::LaneType_SHADED_AREA;
      break;
    case hozon::mapping::LaneType::LaneType_LANE_VIRTUAL_MARKING:
      lane_line->linetype =
          hozon::mp::mf::em::LineType::LaneType_LANE_VIRTUAL_MARKING;
      break;
    case hozon::mapping::LaneType::LaneType_INTERSECTION_VIRTUAL_MARKING:
      lane_line->linetype =
          hozon::mp::mf::em::LineType::LaneType_INTERSECTION_VIRTUAL_MARKING;
      break;
    case hozon::mapping::LaneType::LaneType_CURB_VIRTUAL_MARKING:
      lane_line->linetype =
          hozon::mp::mf::em::LineType::LaneType_CURB_VIRTUAL_MARKING;
      break;
    case hozon::mapping::LaneType::LaneType_UNCLOSED_ROAD:
      lane_line->linetype = hozon::mp::mf::em::LineType::LaneType_UNCLOSED_ROAD;
      break;
    case hozon::mapping::LaneType::LaneType_ROAD_VIRTUAL:
      lane_line->linetype = hozon::mp::mf::em::LineType::LaneType_ROAD_VIRTUAL;
      break;
    case hozon::mapping::LaneType::LaneType_LANE_CHANG_VIRTUAL:
      lane_line->linetype =
          hozon::mp::mf::em::LineType::LaneType_LANE_CHANG_VIRTUAL;
      break;
    case hozon::mapping::LaneType::LaneType_FISHBONE_SOLID:
      lane_line->linetype =
          hozon::mp::mf::em::LineType::LaneType_FISHBONE_SOLID;
      break;
    case hozon::mapping::LaneType::LaneType_FISHBONE_DASHED:
      lane_line->linetype =
          hozon::mp::mf::em::LineType::LaneType_FISHBONE_DASHED;
      break;
    default:
      lane_line->linetype = hozon::mp::mf::em::LineType::LaneType_OTHER;
      break;
  }
}

// void GeoOptimization::OnPilotMap() {
//   pilot_map_ = std::make_shared<hozon::hdmap::Map>();

//   // 设置header,这边复用localmap的header
//   pilot_map_->mutable_header()->mutable_header()->CopyFrom(
//       local_map_->header());
//   // 注意：用map.header.id来承载初始化的位姿，包含local enu站心
//   pilot_map_->mutable_header()->set_id(init_pose_ser_);
//   if ((*map_lanes_).empty()) return;
//   // 喂数据
//   // 少color数据 和laneboudary的type
//   for (auto lane_it = map_lanes_->begin(); lane_it != map_lanes_->end();
//        lane_it++) {
//     //
//     由于一些没点的line和lane类型不对的已经在UpdateLaneByLocalmap被滤除掉了，因此这边不做边界操作
//     auto* lane = pilot_map_->add_lane();
//     lane->mutable_id()->set_id(std::to_string(lane_it->lane_id_));
//     // central_curve
//     auto* central_segment = lane->mutable_central_curve()->add_segment();
//     for (const auto& point_msg : lane_it->pilot_center_line_.points()) {
//       Eigen::Vector3d point_local(point_msg.x(), point_msg.y(),
//       point_msg.z()); Eigen::Vector3d point_enu = T_U_V_ * point_local; auto*
//       point = central_segment->mutable_line_segment()->add_point();
//       point->set_x(point_enu.x());
//       point->set_y(point_enu.y());
//       point->set_z(point_enu.z());
//     }
//     central_segment->set_heading(0);
//     central_segment->set_length(0);
//     // left_boundary
//     lane->mutable_left_boundary()->set_virtual_(true);
//     hozon::hdmap::LaneBoundaryType::Type left_type =
//         hozon::hdmap::LaneBoundaryType::Type::LaneBoundaryType_Type_UNKNOWN;
//     // ConvertInnerMapLaneType(lane_it->left_line_.color_, true,
//     //                                     lane_it->left_line_.lanetype_,
//     //                                     &left_type);
//     // color和boundary都没传出来没添加进去
//     lane->mutable_left_boundary()->add_boundary_type()->add_types(left_type);
//     lane->mutable_left_boundary()->set_length(0);
//     auto* segment =
//         lane->mutable_left_boundary()->mutable_curve()->add_segment();
//     for (const auto& point_msg : lane_it->pilot_left_line_.points()) {
//       auto* left_point = segment->mutable_line_segment()->add_point();
//       Eigen::Vector3d point_local(point_msg.x(), point_msg.y(),
//       point_msg.z()); Eigen::Vector3d point_enu = T_U_V_ * point_local;
//       left_point->set_x(point_enu.x());
//       left_point->set_y(point_enu.y());
//       left_point->set_z(point_enu.z());
//     }
//     // right_boundary
//     lane->mutable_right_boundary()->set_virtual_(true);
//     hozon::hdmap::LaneBoundaryType::Type right_type =
//         hozon::hdmap::LaneBoundaryType::Type::LaneBoundaryType_Type_UNKNOWN;
//     // DataConvert::ConvertInnerMapLaneType(lane_msg.right_line_.color_,
//     false,
//     //                                     lane_msg.right_line_.lanetype_,
//     //                                     &right_type);
//     lane->mutable_right_boundary()->add_boundary_type()->add_types(right_type);
//     lane->mutable_right_boundary()->set_length(0);
//     segment = lane->mutable_right_boundary()->mutable_curve()->add_segment();
//     for (const auto& point_msg : lane_it->pilot_right_line_.points()) {
//       auto* right_point = segment->mutable_line_segment()->add_point();
//       Eigen::Vector3d point_local(point_msg.x(), point_msg.y(),
//       point_msg.z()); Eigen::Vector3d point_enu = T_U_V_ * point_local;
//       right_point->set_x(point_enu.x());
//       right_point->set_y(point_enu.y());
//       right_point->set_z(point_enu.z());
//     }
//     // length
//     lane->set_length(0);
//     // speed_limit
//     lane->set_speed_limit(0);
//     // left_neighbor_forward_lane_id
//     lane->add_left_neighbor_forward_lane_id()->set_id(
//         std::to_string(lane_it->left_lane_id_));
//     // right_neighbor_forward_lane_id
//     lane->add_right_neighbor_forward_lane_id()->set_id(
//         std::to_string(lane_it->right_lane_id_));
//     // type
//     hozon::hdmap::Lane::LaneType lane_type =
//         hozon::hdmap::Lane::LaneType::Lane_LaneType_NONE;
//     lane->set_type(lane_type);
//     // turn
//     hozon::hdmap::Lane::LaneTurn lane_turn =
//         hozon::hdmap::Lane::LaneTurn::Lane_LaneTurn_NO_TURN;
//     lane->set_turn(lane_turn);
//     // map_lane_type
//     lane->mutable_map_lane_type()->set_unknow(true);
//   }
//   // set road
//   // road
//   auto* road = pilot_map_->add_road();
//   // id
//   road->mutable_id()->set_id(std::to_string(0));
//   // // section
//   auto* section = road->add_section();
//   section->mutable_id()->set_id(std::to_string(0));
//   for (auto lane_msg = map_lanes_->begin(); lane_msg != map_lanes_->end();
//        lane_msg++) {
//     section->add_lane_id()->set_id(std::to_string(lane_msg->lane_id_));
//   }
//   // std::map<int, hozon::mapping::LaneLine> left_lane_lines;
//   // std::map<int, hozon::mapping::LaneLine, std::greater<>>
//   right_lane_lines;
//   // for (auto lane_msg : (*map_lanes_)) {
//   //     int left_lanepos = lane_msg.left_line_.lanepos();
//   //     int right_lanepos = lane_msg.right_line_.lanepos();
//   //     left_lane_lines[left_lanepos] = lane_msg.left_line_;
//   //     right_lane_lines[right_lanepos] = lane_msg.right_line_;
//   // }
//   // hozon::mapping::LaneLine left_edge_line =
//   left_lane_lines.begin()->second;
//   // hozon::mapping::LaneLine right_edge_line =
//   // right_lane_lines.begin()->second;
//   auto* left_edge =
//       section->mutable_boundary()->mutable_outer_polygon()->add_edge();
//   left_edge->set_type(hozon::hdmap::BoundaryEdge::UNKNOWN);
//   auto* left_segment =
//       left_edge->mutable_curve()->add_segment()->mutable_line_segment();
//   for (const auto& point_msg : (*map_lanes_).begin()->left_line_.points()) {
//     auto* left_edge_point = left_segment->add_point();
//     Eigen::Vector3d point_local(point_msg.x(), point_msg.y(), point_msg.z());
//     Eigen::Vector3d point_enu = T_U_V_ * point_local;
//     left_edge_point->set_x(point_enu.x());
//     left_edge_point->set_y(point_enu.y());
//     left_edge_point->set_z(point_enu.z());
//   }
//   auto* right_edge =
//       section->mutable_boundary()->mutable_outer_polygon()->add_edge();
//   right_edge->set_type(hozon::hdmap::BoundaryEdge::UNKNOWN);
//   auto* right_segment =
//       right_edge->mutable_curve()->add_segment()->mutable_line_segment();
//   int size_lane = (*map_lanes_).size();
//   if (size_lane > 0) {
//     for (const auto& point_msg :
//         (*map_lanes_)[size_lane - 1].right_line_.points()) {
//       auto* right_edge_point = right_segment->add_point();
//       Eigen::Vector3d point_local(point_msg.x(), point_msg.y(),
//       point_msg.z()); Eigen::Vector3d point_enu = T_U_V_ * point_local;
//       right_edge_point->set_x(point_enu.x());
//       right_edge_point->set_y(point_enu.y());
//       right_edge_point->set_z(point_enu.z());
//     }
//   }
// }

void GeoOptimization::ConvertInnerMapLaneType(
    const Color& color, const bool& is_left, const LaneType& inner_lanetype,
    hozon::hdmap::LaneBoundaryType::Type* lanetype) {
  if (inner_lanetype == LaneType::DashedLine ||
      inner_lanetype == LaneType::ShortDashedLine ||
      inner_lanetype == LaneType::DoubleDashedLine ||
      (inner_lanetype == LaneType::LeftSolidRightDashed && is_left) ||
      (inner_lanetype == LaneType::RightSolidLeftDashed && !is_left)) {
    if (color == WHITE) {
      *lanetype = hozon::hdmap::LaneBoundaryType::Type::
          LaneBoundaryType_Type_DOTTED_WHITE;
    } else if (color == YELLOW) {
      *lanetype = hozon::hdmap::LaneBoundaryType::Type::
          LaneBoundaryType_Type_DOTTED_YELLOW;
    }
  } else if (inner_lanetype == LaneType::SolidLine ||
             (inner_lanetype == LaneType::LeftSolidRightDashed && !is_left) ||
             (inner_lanetype == LaneType::RightSolidLeftDashed && is_left)) {
    if (color == WHITE) {
      *lanetype = hozon::hdmap::LaneBoundaryType::Type::
          LaneBoundaryType_Type_SOLID_WHITE;
    } else if (color == YELLOW) {
      *lanetype = hozon::hdmap::LaneBoundaryType::Type::
          LaneBoundaryType_Type_SOLID_YELLOW;
    }
  } else if ((inner_lanetype == LaneType::DoubleDashedLine) &&
             color == YELLOW) {
    *lanetype = hozon::hdmap::LaneBoundaryType::Type::
        LaneBoundaryType_Type_DOUBLE_YELLOW;
  } else {
    *lanetype =
        hozon::hdmap::LaneBoundaryType::Type::LaneBoundaryType_Type_UNKNOWN;
  }
}

void GeoOptimization::ConvertInnerLaneType(const LaneType& inner_lanetype,
                                           hozon::mapping::LaneType* lanetype) {
  switch (inner_lanetype) {
    case LaneType::Unknown:
      *lanetype = hozon::mapping::LaneType::LaneType_UNKNOWN;
      break;
    case LaneType::SolidLine:
      *lanetype = hozon::mapping::LaneType::LaneType_SOLID;
      break;
    case LaneType::DashedLine:
      *lanetype = hozon::mapping::LaneType::LaneType_DASHED;
      break;
    case LaneType::ShortDashedLine:
      *lanetype = hozon::mapping::LaneType::LaneType_SHORT_DASHED;
      break;
    case LaneType::DoubleSolidLine:
      *lanetype = hozon::mapping::LaneType::LaneType_DOUBLE_SOLID;
      break;
    case LaneType::DoubleDashedLine:
      *lanetype = hozon::mapping::LaneType::LaneType_DOUBLE_DASHED;
      break;
    case LaneType::LeftSolidRightDashed:
      *lanetype = hozon::mapping::LaneType::LaneType_LEFT_SOLID_RIGHT_DASHED;
      break;
    case LaneType::RightSolidLeftDashed:
      *lanetype = hozon::mapping::LaneType::LaneType_RIGHT_SOLID_LEFT_DASHED;
      break;
    case LaneType::ShadedArea:
      *lanetype = hozon::mapping::LaneType::LaneType_SHADED_AREA;
      break;
    case LaneType::LaneVirtualMarking:
      *lanetype = hozon::mapping::LaneType::LaneType_LANE_VIRTUAL_MARKING;
      break;
    case LaneType::IntersectionVirualMarking:
      *lanetype =
          hozon::mapping::LaneType::LaneType_INTERSECTION_VIRTUAL_MARKING;
      break;
    case LaneType::CurbVirtualMarking:
      *lanetype = hozon::mapping::LaneType::LaneType_CURB_VIRTUAL_MARKING;
      break;
    case LaneType::UnclosedRoad:
      *lanetype = hozon::mapping::LaneType::LaneType_UNCLOSED_ROAD;
      break;
    case LaneType::RoadVirtualLine:
      *lanetype = hozon::mapping::LaneType::LaneType_ROAD_VIRTUAL;
      break;
    case LaneType::LaneChangeVirtualLine:
      *lanetype = hozon::mapping::LaneType::LaneType_LANE_CHANG_VIRTUAL;
      break;
    case LaneType::Other:
      *lanetype = hozon::mapping::LaneType::LaneType_OTHER;
      break;
    default:
      break;
  }
}
}  // namespace mf
}  // namespace mp
}  // namespace hozon
