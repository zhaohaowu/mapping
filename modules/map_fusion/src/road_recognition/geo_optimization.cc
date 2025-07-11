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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "base/utils/log.h"
#include "depend/common/utm_projection/coordinate_convertor.h"
#include "map_fusion/fusion_common/calc_util.h"
#include "map_fusion/fusion_common/elemap2proto.h"
#include "map_fusion/fusion_common/element_map.h"
#include "map_fusion/map_prediction/viz_map.h"
#include "map_fusion/map_service/global_hd_map.h"
#include "map_fusion/road_recognition/occ_guideline_manager.h"
#include "third_party/orin/gflags/include/gflags/gflags.h"
#include "util/mapping_log.h"
#include "util/tic_toc.h"

#define PI acos(-1)

using hozon::common::math::Vec2d;

DEFINE_bool(road_recognition_rviz, false, "road recognition use rviz or not");
DEFINE_int32(virtual_line_id, 2000, "the virtual line id");

namespace hozon {
namespace mp {
namespace mf {

int GeoOptimization::Init() {
  occ_guideline_manager_ = std::make_shared<OccGuideLineManager>();
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
        KTopicRoadRecognitionLocalMap,    KTopicRoadRecognitionTopoMapRoad,
        KTopicRoadRecognitionTopoMapLane, KTopicRoadRecognitionElementMap,
        KTopicRoadRecognitionLineLable,   KTopicRoadRecognitionOccRoad};
    ret = RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(topic_vec);
  }
  extra_val_ = FLAGS_virtual_line_id;
  elem_map_ = std::make_shared<hozon::mp::mf::em::ElementMap>();
  local_map_ = std::make_shared<hozon::mapping::LocalMap>();
  // pilot_map_ = std::make_shared<hozon::hdmap::Map>();
  local_map_use_ = std::make_shared<hozon::mapping::LocalMap>();
  // map_lanes_ = std::make_shared<std::vector<LanePilot>>();
  history_objs_.set_capacity(history_objs_size_);
  history_objs_.clear();
  inverse_history_objs_.set_capacity(inverse_history_objs_size_);
  inverse_history_objs_.clear();
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

void GeoOptimization::VizOccRoad() {
  // 可视化OccRoad
  HLOG_ERROR << "FLAGS_road_recognition_rviz" << FLAGS_road_recognition_rviz;
  if ((!FLAGS_road_recognition_rviz) || (!RVIZ_AGENT.Ok())) {
    return;
  }
  adsfi_proto::viz::MarkerArray markers;
  for (const auto& occ : local_map_->occs()) {
    std::vector<Eigen::Vector3d> occ_points;
    for (const auto& point : occ.points()) {
      Eigen::Vector3d point_local(point.x(), point.y(), point.z());
      Eigen::Vector3d point_enu = T_U_V_ * point_local;
      occ_points.emplace_back(point_local);
    }
    adsfi_proto::viz::Marker marker;
    std::vector<float> color = color_palette.find('b')->second;
    PointsToMarker(local_map_->header().publish_stamp(), occ_points, &marker,
                   color);
    if (marker.points().size() >= 2) {
      markers.add_markers()->CopyFrom(marker);
    }
    if (!occ_points.empty()) {
      adsfi_proto::viz::Marker marker_id;
      auto id = std::to_string(occ.track_id());
      LineIdToMarker(local_map_->header().publish_stamp(), occ_points[0], id,
                     &marker_id);
      markers.add_markers()->CopyFrom(marker_id);
    }
  }
  RVIZ_AGENT.Publish(KTopicRoadRecognitionOccRoad, markers);
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
  double t = AB.dot(AP) / (ABLength * ABLength);
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
    int pt_interval = 1;
    for (int i = 0; i < line.points_size(); i += pt_interval) {
      const auto& point = line.points(i);
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
    if (kdtree_points.size() < 2) {
      continue;
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
  all_lines_ = std::move(all_lines);
  return;
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

void GeoOptimization::FilterNoEgoLineNoCrossing() {
  if (!local_map_) {
    HLOG_ERROR << "local_map_ is nullptr";
    return;
  }
  // 模型路沿
  std::vector<std::vector<Eigen::Vector3d>> nearby_road_edges;
  for (const auto& local_road : local_map_->road_edges()) {
    if (local_road.points_size() == 0 || local_road.points().empty()) {
      continue;
    }
    std::vector<Eigen::Vector3d> pts;
    for (const auto& it : local_road.points()) {
      Eigen::Vector3d pt(it.x(), it.y(), it.z());
      pts.emplace_back(pt);
    }
    nearby_road_edges.emplace_back(pts);
  }

  std::vector<Eigen::Vector3d> road_edge_pts;
  road_edge_pts = FindTargetPointsNoCrossing(nearby_road_edges);
  if (!road_edge_pts.empty()) {
    HandleOppisiteLineNoCrossing(road_edge_pts);
  }
}

void GeoOptimization::CollectOpenings() {
  openings_.clear();
  opening_all_lines_.clear();
  opening_all_line_ids_.clear();
  int opening_id = 100;
  /* 分别将id和对应的line存储到无序map中便于通过id使用line，将id存到
  vector中从左到右排序后便于使用和减少遍历的次数*/
  for (const auto& line_vector : all_lines_) {
    for (const auto& line : line_vector.second) {
      const auto& points = line.line->points();
      if (points.at(0).x() > 0.0) {
        opening_all_lines_[line.line->track_id()] = line;
        opening_all_line_ids_.push_back(line.line->track_id());
      }
    }
  }
  // 车道线数量不足２，直接返回
  if (opening_all_line_ids_.size() < 2 || opening_all_lines_.size() < 2) {
    return;
  }
  // 按照车道线起点处的y值进行排序
  std::sort(opening_all_line_ids_.begin(), opening_all_line_ids_.end(),
            [&](int lhs, int rhs) {
              return opening_all_lines_[lhs].line->points().at(0).y() >
                     opening_all_lines_[rhs].line->points().at(0).y();
            });
  // 0.找到所有车前车道线的x的取值范围
  double max_line_x = std::numeric_limits<double>::min();
  double min_line_x = std::numeric_limits<double>::max();
  for (const auto& cur_id : opening_all_line_ids_) {
    const auto& start_point = opening_all_lines_[cur_id].line->points(0);
    if (start_point.x() < min_line_x) {
      min_line_x = start_point.x();
    }
    const auto& end_point = opening_all_lines_[cur_id].line->points(
        opening_all_lines_[cur_id].line->points_size() - 1);
    if (end_point.x() > max_line_x) {
      max_line_x = end_point.x();
    }
  }
  // 1.收集在x范围内所有的occ路沿和模型路沿
  std::vector<std::vector<Eigen::Vector3d>> all_edges;
  // HLOG_ERROR << "模型路沿数量: " << local_map_->road_edges().size();
  for (const auto& local_road : local_map_->road_edges()) {
    if (local_road.points_size() < 2 ||
        (!local_road.points().empty() &&
         local_road.points().at(0).x() > max_line_x) ||
        (!local_road.points().empty() &&
         local_road.points().at(local_road.points_size() - 1).x() <
             min_line_x)) {
      continue;
    }
    std::vector<Eigen::Vector3d> pts;
    for (const auto& it : local_road.points()) {
      if (it.x() > min_line_x && it.x() < max_line_x) {
        Eigen::Vector3d pt(it.x(), it.y(), it.z());
        pts.emplace_back(pt);
      }
    }
    if (pts.size() > 10) {
      all_edges.emplace_back(pts);
    }
  }

  std::map<int, em::Boundary::Ptr> stable_occ_roads = GetStableOcc();
  // HLOG_ERROR << "occ路沿数量: " << stable_occ_roads.size();
  for (const auto& occ_pair : stable_occ_roads) {
    const auto occ_road = occ_pair.second->nodes;
    if (occ_road.size() < 2 ||
        (!occ_road.empty() && occ_road.at(0)->point.x() > max_line_x) ||
        (!occ_road.empty() &&
         occ_road.at(occ_road.size() - 1)->point.x() < min_line_x)) {
      continue;
    }
    std::vector<Eigen::Vector3d> pts;
    for (const auto& it : occ_road) {
      if (it->point.x() > min_line_x && it->point.x() < max_line_x) {
        Eigen::Vector3d pt(it->point.x(), it->point.y(), it->point.z());
        pts.emplace_back(pt);
      }
    }
    if (pts.size() > 10) {
      all_edges.emplace_back(pts);
    }
  }

  // 2.对所有路沿按起始点y从大到小排序
  if (all_edges.size() > 1) {
    std::sort(all_edges.begin(), all_edges.end(),
              [](const std::vector<Eigen::Vector3d>& edge1,
                 const std::vector<Eigen::Vector3d>& edge2) {
                // 获取每条路沿的第一个点（即起始点）的 y 坐标
                double y1 = edge1.front().y();
                double y2 = edge2.front().y();

                // 按 y 坐标从大到小,从左往右排序
                return y1 > y2;
              });
  }

  // HLOG_ERROR << "各种路沿线的总数量1: " << all_edges.size();

  // 确保豁口线是将车道线包含在内的，如果不是则创建新的路沿线
  std::vector<Eigen::Vector3d> opening_first_line_pts;
  const auto& first_line =
      opening_all_lines_[opening_all_line_ids_.front()].line;
  for (const auto& it : first_line->points()) {
    Eigen::Vector3d pt(it.x(), it.y(), it.z());
    opening_first_line_pts.emplace_back(pt);
  }
  std::vector<Eigen::Vector3d> opening_last_line_points;
  const auto& last_line = opening_all_lines_[opening_all_line_ids_.back()].line;
  for (const auto& it : last_line->points()) {
    Eigen::Vector3d pt(it.x(), it.y(), it.z());
    opening_last_line_points.emplace_back(pt);
  }
  if (all_edges.empty()) {
    std::vector<Eigen::Vector3d> pts1, pts2;
    for (const auto& line_pt : opening_first_line_pts) {
      Eigen::Vector3d pt(line_pt.x(), line_pt.y() + 1.0, line_pt.z());
      pts1.emplace_back(pt);
    }
    for (const auto& line_pt : opening_last_line_points) {
      Eigen::Vector3d pt(line_pt.x(), line_pt.y() - 1.0, line_pt.z());
      pts2.emplace_back(pt);
    }
    all_edges.emplace_back(pts1);
    all_edges.emplace_back(pts2);
  } else {
    if (IsTargetOnLineRight(all_edges.front(), opening_first_line_pts) ==
        RelativePosition::RIGHT) {
      std::vector<Eigen::Vector3d> pts;
      for (const auto& line_pt : opening_first_line_pts) {
        Eigen::Vector3d pt(line_pt.x(), line_pt.y() + 1.0, line_pt.z());
        pts.emplace_back(pt);
      }
      all_edges.insert(all_edges.begin(), pts);
    }
    if (IsTargetOnLineRight(all_edges.back(), opening_last_line_points) ==
        RelativePosition::LEFT) {
      std::vector<Eigen::Vector3d> pts;
      for (const auto& line_pt : opening_last_line_points) {
        Eigen::Vector3d pt(line_pt.x(), line_pt.y() - 1.0, line_pt.z());
        pts.emplace_back(pt);
      }
      all_edges.emplace_back(pts);
    }
  }
  // HLOG_ERROR << "各种路沿线的总数量2: " << all_edges.size();

  // 3.对于相邻两个路沿，判断它们中间是否有至少一根车道线，
  // 有且计算距离>6的直接构建豁口对
  // 将line按照各豁口的边界分别放到对应的豁口中
  std::vector<int> line_ids = opening_all_line_ids_;

  for (int index = 0; index < all_edges.size() - 1; index++) {
    opening cur_opening;
    cur_opening.left_boundary = all_edges[index];
    cur_opening.right_boundary = all_edges[index + 1];
    for (auto it = line_ids.begin(); it != line_ids.end(); it++) {
      // HLOG_ERROR << "当前车道线id0: " << *it;
      std::vector<Eigen::Vector3d> line_pts;
      const auto& line = opening_all_lines_[*it].line;
      for (const auto& it : line->points()) {
        Eigen::Vector3d pt(it.x(), it.y(), it.z());
        line_pts.emplace_back(pt);
      }
      if (all_edges[index].size() < 2 || line_pts.size() < 2 ||
          all_edges[index + 1].size() < 2) {
        continue;
      }
      RelativePosition left_status =
          IsTargetOnLineRight(all_edges[index], line_pts);
      RelativePosition right_status =
          IsTargetOnLineRight(all_edges[index + 1], line_pts);
      // 计算车道线到两个豁口的距离和
      double avg_widths_left;
      double avg_widths_right;
      // 没有重合区域需要计算距离时，路沿找一个点，往车道线最近的两个点构成的线段上做投影
      if (all_edges[index].front().x() >= line_pts.back().x() ||
          all_edges[index].back().x() <= line_pts.front().x()) {
        if (!ComputerLineDisNoOverlap(all_edges[index], line_pts,
                                      &avg_widths_left)) {
          // HLOG_INFO << 111111111;
          continue;
        }
      } else {
        if (!ComputerLineDis(all_edges[index], line_pts, &avg_widths_left)) {
          // HLOG_ERROR << "all_edges[index].size(): " <<
          // all_edges[index].size(); HLOG_ERROR << "all_edges[index]: " <<
          // all_edges[index].front().x()
          //            << " " << all_edges[index].front().y();
          // HLOG_ERROR << "all_edges[index]: " << all_edges[index].back().x()
          //            << " " << all_edges[index].back().y();
          // HLOG_INFO << 111111111;
          continue;
        }
      }
      if (all_edges[index + 1].front().x() >= line_pts.back().x() ||
          all_edges[index + 1].back().x() <= line_pts.front().x()) {
        if (!ComputerLineDisNoOverlap(all_edges[index + 1], line_pts,
                                      &avg_widths_right)) {
          // HLOG_INFO << 111111111;
          continue;
        }
      } else {
        if (!ComputerLineDis(all_edges[index + 1], line_pts,
                             &avg_widths_right)) {
          // HLOG_INFO << 111111111;
          continue;
        }
      }
      // if (!ComputerLineDis(all_edges[index+1]], line_pts, &avg_widths_left)
      // ||
      //     !ComputerLineDis(all_edges[index + 1], line_pts,
      //     &avg_widths_right)) {
      //   HLOG_ERROR << "车道线距离计算出错";
      //   continue;
      // }
      // HLOG_ERROR << "当前车道线id1: " << *it;
      // HLOG_ERROR << "left_status: " << int(left_status);
      // HLOG_ERROR << "right_status: " << int(right_status);
      // HLOG_ERROR << "avg_widths_left: " << avg_widths_left;
      // HLOG_ERROR << "avg_widths_right: " << avg_widths_right;
      if ((left_status == RelativePosition::LEFT &&
           right_status == RelativePosition::RIGHT &&
           ((avg_widths_left + avg_widths_right) > 6.0)) ||
          (left_status == RelativePosition::RIGHT &&
           right_status == RelativePosition::RIGHT &&
           (avg_widths_left < 2.0)) ||
          (left_status == RelativePosition::LEFT &&
           right_status == RelativePosition::LEFT &&
           (avg_widths_right < 2.0))) {
        // HLOG_ERROR << "在豁口里的车道线id2: " << *it;
        cur_opening.line_ids.emplace_back(*it);
        cur_opening.start_x = min_line_x;
        cur_opening.end_x = max_line_x;
        // 豁口内有双黄线,且离两边线距离大于均>2.5的直接进行标记
        if (line->lanetype() ==
                hozon::mapping::LaneType::LaneType_DOUBLE_SOLID &&
            line->color() == hozon::mapping::Color::YELLOW &&
            avg_widths_left > 2.5 && avg_widths_right > 2.5) {
          cur_opening.double_solid_yellow.exist = true;
          // HLOG_ERROR << "这个豁口存在满足条件的双黄线";
          cur_opening.double_solid_yellow.ids.emplace_back(*it);
        }
      }
    }
    if (cur_opening.line_ids.size() > 1) {
      cur_opening.opening_id = opening_id;
      opening_id++;
      openings_.emplace_back(cur_opening);
    }
  }

  return;
}

bool GeoOptimization::GetBoundaryLineObs(
    const std::vector<int> line_ids, const Eigen::Vector3d& point, int obj_id,
    std::unordered_map<int, std::vector<int>>* ids) {
  if (!PointInLineRight(opening_all_lines_[line_ids.front()].line->points(),
                        point) ||
      PointInLineRight(opening_all_lines_[line_ids.back()].line->points(),
                       point)) {
    return false;
  }
  for (int i = 1; i < line_ids.size(); ++i) {
    if (!PointInLineRight(opening_all_lines_[line_ids[i]].line->points(),
                          point)) {
      std::vector<int> left_right_line_id{line_ids[i - 1], line_ids[i]};
      (*ids)[obj_id] = left_right_line_id;
      return true;
    }
  }
  return false;
}

void GeoOptimization::OpeningDealObs() {
  for (auto& open : openings_) {
    // for (const auto& id : open.line_ids) {
    //   HLOG_ERROR << "openings line id: " << id
    //              << " x: " << opening_all_lines_[id].line->points().at(0).x()
    //              << " ,y: " <<
    //              opening_all_lines_[id].line->points().at(0).y();
    // }
    std::unordered_map<int, std::vector<int>> objid_same_ids;
    std::unordered_map<int, std::vector<int>> objid_inverse_ids;
    opening::objection same_direction_obs;
    opening::objection inverse_obs;
    same_direction_obs.left_dis = DBL_MAX;
    same_direction_obs.right_dis = DBL_MAX;
    inverse_obs.left_dis = DBL_MAX;
    inverse_obs.right_dis = DBL_MAX;
    for (auto obj_it = geo_obj_ids_.begin(); obj_it != geo_obj_ids_.end();
         obj_it++) {
      // 没有历史障碍物点位
      // HLOG_ERROR << "obj_it: " << *obj_it;
      if (*obj_it == -1 && geo_objs_.find(-1) == geo_objs_.end()) {
        continue;
      }
      const auto& obj = geo_objs_[*obj_it];
      std::vector<Eigen::Vector3d> obs_points;
      Eigen::Vector3d begin_point =
          T_U_V_.inverse() * obj.his_positions.front();
      obs_points.emplace_back(begin_point);
      Eigen::Vector3d end_point = T_U_V_.inverse() * obj.his_positions.back();
      obs_points.emplace_back(end_point);
      // HLOG_ERROR << "id: " << *obj_it;
      // HLOG_ERROR << "**begin_point x:: " << begin_point.x()
      //            << " ,y: " << begin_point.y()
      //            << " , end_point x: " << end_point.x()
      //            << " ,y: " << end_point.y()
      //            << " , is_inverse: " << obj.is_inverse;

      // 去掉纵向上不在豁口里的obs

      // 收集左右侧的line id
      if (obj.is_inverse) {
        for (const auto& point : obs_points) {
          // HLOG_ERROR << "openings_id: " << open.opening_id
          //            << " ,start x:: " << open.start_x
          //            << " ,end_x: " << open.end_x;
          // HLOG_ERROR << "point x:: " << point.x() << " ,y: " << point.y();
          if (point.x() > open.start_x + 10.0 || point.x() < open.start_x ||
              point.x() > open.end_x) {
            continue;
          }
          if (GetBoundaryLineObs(open.line_ids, point, *obj_it,
                                 &objid_inverse_ids)) {
            auto left_dis = PointToLineDis(open.left_boundary, point);
            auto right_dis = PointToLineDis(open.right_boundary, point);
            if (left_dis > 0.0 && left_dis < inverse_obs.left_dis) {
              inverse_obs.left_dis = left_dis;
              inverse_obs.left_point = point;
            }
            if (left_dis > 0.0 && right_dis < inverse_obs.right_dis) {
              inverse_obs.right_dis = right_dis;
              inverse_obs.right_point = point;
            }
          }
        }
        // HLOG_ERROR << "openings_id: " << open.opening_id
        //            << " ,start x:: " << open.start_x
        //            << " ,end_x: " << open.end_x;
        // HLOG_ERROR << "inverse_obs.left_point x:: "
        //            << inverse_obs.left_point.x()
        //            << " ,y: " << inverse_obs.left_point.y();
        // HLOG_ERROR << "inverse_obs.right_point x:: "
        //            << inverse_obs.right_point.x()
        //            << " ,y: " << inverse_obs.right_point.y();
      } else {
        for (const auto& point : obs_points) {
          if (point.x() < open.start_x || point.x() > open.end_x) {
            continue;
          }
          if (GetBoundaryLineObs(open.line_ids, point, *obj_it,
                                 &objid_same_ids)) {
            auto left_dis = PointToLineDis(open.left_boundary, point);
            auto right_dis = PointToLineDis(open.right_boundary, point);
            if (left_dis > 0.0 && left_dis < same_direction_obs.left_dis) {
              same_direction_obs.left_dis = left_dis;
              same_direction_obs.left_point = point;
            }
            if (left_dis > 0.0 && right_dis < same_direction_obs.right_dis) {
              same_direction_obs.right_dis = right_dis;
              same_direction_obs.right_point = point;
            }
          }
        }
      }
    }
    // 得到正向/逆向的最左右侧的line id
    if (objid_same_ids.size() > 1) {
      open.same_direction_obs = same_direction_obs;
      open.same_direction_obs.exist = true;
      auto right_temp_pair = *std::min_element(
          objid_same_ids.begin(), objid_same_ids.end(),
          [&](std::pair<int, std::vector<int>> a,
              std::pair<int, std::vector<int>> b) {
            return opening_all_lines_[a.second[1]].line->points().begin()->y() <
                   opening_all_lines_[b.second[1]].line->points().begin()->y();
          });
      open.same_direction_obs.right_id = right_temp_pair.second[1];

      auto left_temp_pair = *std::max_element(
          objid_same_ids.begin(), objid_same_ids.end(),
          [&](std::pair<int, std::vector<int>> a,
              std::pair<int, std::vector<int>> b) {
            return opening_all_lines_[a.second[0]].line->points().begin()->y() <
                   opening_all_lines_[b.second[0]].line->points().begin()->y();
          });
      open.same_direction_obs.left_id = left_temp_pair.second[0];
    }
    if (objid_inverse_ids.size() > 1) {
      open.opposite_obs = inverse_obs;
      open.opposite_obs.exist = true;
      auto right_temp_pair = *std::min_element(
          objid_inverse_ids.begin(), objid_inverse_ids.end(),
          [&](std::pair<int, std::vector<int>> a,
              std::pair<int, std::vector<int>> b) {
            return opening_all_lines_[a.second[1]].line->points().begin()->y() <
                   opening_all_lines_[b.second[1]].line->points().begin()->y();
          });
      open.opposite_obs.right_id = right_temp_pair.second[1];
      geoobjs history_right_obj = geo_objs_[right_temp_pair.first];
      // HLOG_ERROR << "障碍物id: " << right_temp_pair.first;
      history_right_obj.id = -1;
      geo_objs_[-1] = history_right_obj;
      auto left_temp_pair = *std::max_element(
          objid_inverse_ids.begin(), objid_inverse_ids.end(),
          [&](std::pair<int, std::vector<int>> a,
              std::pair<int, std::vector<int>> b) {
            return opening_all_lines_[a.second[0]].line->points().begin()->y() <
                   opening_all_lines_[b.second[0]].line->points().begin()->y();
          });
      open.opposite_obs.left_id = left_temp_pair.second[0];
    }
  }
  return;
}
void GeoOptimization::OpeningDealStopline() {
  // 获取前向stopline集合
  std::vector<std::vector<Eigen::Vector2f>> forward_stoplines;
  for (const auto& local_stopline : local_map_->stop_lines()) {
    if (!local_stopline.has_left_point() || !local_stopline.has_right_point() ||
        local_stopline.left_point().x() < 0 ||
        local_stopline.right_point().x() < 0) {
      continue;
    }
    bool connect_flag = true;
    Eigen::Vector2f pt_left(local_stopline.left_point().x(),
                            local_stopline.left_point().y());
    Eigen::Vector2f pt_right(local_stopline.right_point().x(),
                             local_stopline.right_point().y());
    for (auto& line_vector : all_lines_) {
      if (!connect_flag) {
        break;
      }
      if (line_vector.second.empty()) {
        continue;
      }
      int line_vector_size = line_vector.second.size();
      for (int j = 0; j < line_vector_size; ++j) {
        const auto line_points = line_vector.second[j].line->points();
        if (line_points.empty() || line_points.size() < 2 ||
            line_points.at(0).x() > 0) {
          continue;
        }
        const auto& back_point = line_points.at(line_points.size() - 1);
        if (PointToVectorDist(pt_left, pt_right, back_point) < 5.f) {
          connect_flag = false;
          break;
        }
      }
    }
    if (!connect_flag) {
      continue;
    }
    std::vector<Eigen::Vector2f> stopline_pts{pt_left, pt_right};
    forward_stoplines.emplace_back(stopline_pts);
  }
  if (forward_stoplines.empty()) {
    return;
  }

  for (auto& forward_stopline : forward_stoplines) {
    for (auto& open : openings_) {
      Eigen::Vector2f stopline_center =
          (forward_stopline[0] + forward_stopline[1]) / 2;
      // 停止线在豁口的左右侧都过滤掉
      auto left_y =
          opening_all_lines_[open.line_ids[0]].line->points().at(0).y();
      auto right_y = opening_all_lines_[open.line_ids[open.line_ids.size() - 1]]
                         .line->points()
                         .at(0)
                         .y();
      if (stopline_center.y() > left_y || stopline_center.y() < right_y) {
        continue;
      }
      for (int i = 0; i < open.line_ids.size(); i++) {
        const auto& line_points =
            opening_all_lines_[open.line_ids[i]].line->points();
        // 找到第一个在停止线左侧的点
        auto line_second_point_it = std::find_if(
            line_points.begin() + 1, line_points.end(), [&](const auto& point) {
              return PointInVectorSide(forward_stopline[0], forward_stopline[1],
                                       point) < 0;
            });

        if (line_second_point_it == line_points.end()) {
          line_second_point_it = line_points.end() - 1;
        }
        // 下游处理应注意停止线的右侧id代表的是被停止线过滤掉的line
        if (PointToVectorDist(forward_stopline[0], forward_stopline[1],
                              line_points.at(0)) < 10 &&
            PointInVectorSide(line_points.at(0), *line_second_point_it,
                              forward_stopline[1]) > 0 &&
            PointToVectorDist(line_points.at(0), *line_second_point_it,
                              forward_stopline[1]) > 1.5 &&
            PointInVectorSide(forward_stopline[0], forward_stopline[1],
                              line_points.at(line_points.size() - 1)) < 0 &&
            PointToVectorDist(forward_stopline[0], forward_stopline[1],
                              line_points.at(line_points.size() - 1)) > 5.0f) {
          open.stop_line.exist = true;
          open.stop_line.right_id = open.line_ids[i];
        }
      }
      if (open.stop_line.exist) {
        Eigen::Vector3d left_point(forward_stopline[1].x(),
                                   forward_stopline[1].y(), 0.0);
        open.stop_line.right_dis =
            PointToLineDis(open.right_boundary, left_point);
        open.stop_line.left_point = left_point;
      }
    }
  }
  return;
}
void GeoOptimization::OpeningDealSolidYellow() { return; }
void GeoOptimization::OpeningDealHeading() {
  em::ExitLaneInfo exit_lane = GetExitLane();
  if (!exit_lane.exist) {
    return;
  } else {
    auto point1 =
        curr_pose_.TransLocalToVehicle(exit_lane.left_boundary_points.back());
    auto point2 =
        curr_pose_.TransLocalToVehicle(exit_lane.right_boundary_points.back());
    Eigen::Vector2f exit_lane_point{(point1.x() + point2.x()) / 2,
                                    (point1.y() + point2.y()) / 2};
    // 遍历豁口对，与退出车道计算heading角度
    for (auto& it_opening : openings_) {
      auto opening_point1 = it_opening.left_boundary.front();
      auto opening_point2 = it_opening.right_boundary.front();
      Eigen::Vector2f opening_point{
          (opening_point1.x() + opening_point2.x()) / 2,
          (opening_point1.y() + opening_point2.y()) / 2};
      Eigen::Vector2f v = opening_point - exit_lane_point;
      // HLOG_ERROR << "v: " << v.x() << " " << v.y();
      it_opening.heading_err = atan2(v.y(), v.x());
      // HLOG_ERROR << "heading_err: " << it_opening.heading_err;
    }

    return;
  }
}
void GeoOptimization::DealOpenings() {
  if (openings_.empty()) {
    return;
  }

  OpeningDealObs();
  OpeningDealStopline();
  OpeningDealSolidYellow();
  OpeningDealHeading();
  return;
}

opening GeoOptimization::SelectOpening() {
  if (openings_.size() == 1) {
    // HLOG_ERROR << "只有一个豁口";
    return openings_[0];
  }
  // 如果豁口中有双黄线，则确定这个豁口
  for (const auto& it_opening : openings_) {
    if (it_opening.double_solid_yellow.exist) {
      // HLOG_ERROR << "由双黄线确定豁口";
      return it_opening;
    }
  }
  // 如果豁口中有正向障碍物，则确定这个豁口，多个正向豁口的时候选择角度小的
  opening temp_opening;
  temp_opening.heading_err = std::numeric_limits<double>::max();
  for (const auto& it_opening : openings_) {
    if (it_opening.same_direction_obs.exist &&
        std::fabs(it_opening.heading_err) < temp_opening.heading_err) {
      temp_opening = it_opening;
    }
  }
  if (!temp_opening.left_boundary.empty()) {
    // HLOG_ERROR << "由正向障碍物确定豁口";
    return temp_opening;
  }
  // 过滤最右侧车道有逆向障碍物或者停止线的豁口，之后选择角度小的
  for (const auto& it_opening : openings_) {
    if ((it_opening.opposite_obs.exist &&
         it_opening.opposite_obs.right_dis < 3.0) ||
        (it_opening.stop_line.exist && it_opening.stop_line.right_dis < 3.0)) {
      continue;
    }
    if (std::fabs(it_opening.heading_err) < temp_opening.heading_err) {
      temp_opening = it_opening;
    }
  }
  // if (!temp_opening.left_boundary.empty()) {
  //   return temp_opening;
  // }
  // HLOG_ERROR << "由停止线和逆向障碍物确定豁口";
  return temp_opening;
}

void GeoOptimization::FilterOpening(const opening& selected_opening) {
  auto left_boundary_it = selected_opening.line_ids.begin();
  if (selected_opening.double_solid_yellow.exist) {
    left_boundary_it = std::find(
        selected_opening.line_ids.begin(), selected_opening.line_ids.end(),
        selected_opening.double_solid_yellow.ids.at(0));
  } else {
    // HLOG_ERROR << "历史停止线关联ＩＤ: "
    //            << last_opening_stopline_correlated_line_id_;
    auto stop_line_it = left_boundary_it;
    auto last_stop_line_it = stop_line_it;
    if (selected_opening.stop_line.exist) {
      stop_line_it = std::find(selected_opening.line_ids.begin(),
                               selected_opening.line_ids.end(),
                               selected_opening.stop_line.right_id);
      last_stop_line_it = std::find(selected_opening.line_ids.begin(),
                                    selected_opening.line_ids.end(),
                                    last_opening_stopline_correlated_line_id_);
      if (last_stop_line_it != selected_opening.line_ids.end()) {
        if (stop_line_it < last_stop_line_it) {
          stop_line_it = last_stop_line_it;
        } else {
          if (*stop_line_it < 1000) {
            last_opening_stopline_correlated_line_id_ = *stop_line_it;
          }
        }
      } else {
        if (selected_opening.stop_line.right_id < 1000) {
          last_opening_stopline_correlated_line_id_ =
              selected_opening.stop_line.right_id;
        }
      }
    }
    auto opposite_obs_it = left_boundary_it;
    if (selected_opening.opposite_obs.exist) {
      opposite_obs_it = std::find(selected_opening.line_ids.begin(),
                                  selected_opening.line_ids.end(),
                                  selected_opening.opposite_obs.right_id);
    }
    left_boundary_it = stop_line_it + 1 > opposite_obs_it &&
                               stop_line_it != selected_opening.line_ids.begin()
                           ? stop_line_it + 1
                           : opposite_obs_it;
  }
  if (left_boundary_it == selected_opening.line_ids.end()) {
    return;
  }
  for (auto& line_vector : all_lines_) {
    for (auto& line : line_vector.second) {
      if (std::find(left_boundary_it, selected_opening.line_ids.end(),
                    line.line->track_id()) == selected_opening.line_ids.end() &&
          std::find(opening_all_line_ids_.begin(), opening_all_line_ids_.end(),
                    line.line->track_id()) != opening_all_line_ids_.end()) {
        line.is_ego_road = false;
      }
    }
  }
}

void GeoOptimization::FilterOpeningLine() {
  if (!local_map_ || all_lines_.empty() ||
      road_scene_ == RoadScene::NON_JUNCTION) {
    HLOG_ERROR << "local_map_ or all_lines_ is nullptr";
    return;
  }
  CollectOpenings();
  // HLOG_ERROR << "openings_.size(): " << openings_.size();
  if (openings_.empty()) {
    return;
  }
  DealOpenings();
  // 　选择最佳豁口
  opening select_opening = SelectOpening();

  // 左右可行驶边界确定
  if (select_opening.line_ids.size() < 2) {
    return;
  }
  // for (const auto& it : select_opening.line_ids) {
  //   HLOG_ERROR << "line_id: " << it;
  // }
  // if (select_opening.opposite_obs.exist) {
  //   HLOG_ERROR << "逆向障碍物：" << select_opening.opposite_obs.right_id;
  // }
  // if (select_opening.stop_line.exist) {
  //   HLOG_ERROR << "停止线: " << select_opening.stop_line.right_id;
  // }
  // if (select_opening.double_solid_yellow.exist) {
  //   HLOG_ERROR << "双黄线: " << select_opening.double_solid_yellow.ids[0];
  // }

  FilterOpening(select_opening);
  return;
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
  // 根据障碍物过滤对向车道线
  HandleOppisiteLineByObj();
  // 模型路沿
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

  // occ路沿
  std::vector<std::vector<Eigen::Vector3d>> forward_occ_road_edges;

  // 　找到车前车道线最大和最小的x值
  double max_line_x = std::numeric_limits<double>::min();
  double min_line_x = std::numeric_limits<double>::max();
  for (const auto& line : local_map_->lane_lines()) {
    if (line.points().empty() || line.points_size() < 2) {
      continue;
    }
    const auto& start_point = line.points(0);
    if (start_point.x() < 0.0) {
      continue;
    }
    if (start_point.x() < min_line_x) {
      min_line_x = start_point.x();
    }
    const auto& end_point = line.points(line.points_size() - 1);
    if (end_point.x() > max_line_x) {
      max_line_x = end_point.x();
    }
  }

  for (const auto& local_occ_road : local_map_->occs()) {
    if (local_occ_road.points_size() < 2 ||
        (!local_occ_road.points().empty() &&
         local_occ_road.points().at(0).x() < 0)) {
      continue;
    }
    std::vector<Eigen::Vector3d> pts;
    std::vector<float> curve_params;
    if (local_occ_road.has_lane_param() &&
        local_occ_road.lane_param().cubic_curve_set_size() == 1) {
      auto c0 = local_occ_road.lane_param().cubic_curve_set(0).c0();
      auto c1 = local_occ_road.lane_param().cubic_curve_set(0).c1();
      auto c2 = local_occ_road.lane_param().cubic_curve_set(0).c2();
      auto c3 = local_occ_road.lane_param().cubic_curve_set(0).c3();
      curve_params = {c0, c1, c2, c3};
      if (OccLineFitError(local_occ_road, curve_params) > 1.0) {
        continue;
      }
      auto start_x = std::max(
          min_line_x,
          static_cast<double>(
              local_occ_road.lane_param().cubic_curve_set(0).start_point_x()));
      auto end_x = std::min(
          max_line_x,
          static_cast<double>(
              local_occ_road.lane_param().cubic_curve_set(0).end_point_x()));
      for (double x = start_x; x < end_x;) {
        double y = c3 * x * x * x + c2 * x * x + c1 * x + c0;
        Eigen::Vector3d pt(x, y, 0);
        pts.emplace_back(pt);
        x += 1.0;
      }
    }
    forward_occ_road_edges.emplace_back(pts);
  }
  std::vector<Eigen::Vector3d> occ_road_edge_pts;
  occ_road_edge_pts = FindTargetPoints(forward_occ_road_edges);

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
  HandleOppisiteLineByStopline();  // 停止线过滤目前不稳定
  // 确定双黄线的左右侧是否有车道线
  std::vector<Eigen::Vector3d> double_solid_yellow_pts;
  double_solid_yellow_pts = FindTargetPoints(double_solid_yellow_line);
  // 确定路口逆向车道(是否可以考虑双黄线？或者结合OCC进行判别？)
  /*
    1.若有双黄线，将双黄线左侧的车道线做标记
    2.若无双黄线，根据模型路沿或者occ路沿的相对关系来进行判别
  */
  // 优先用路沿，其次用双黄线，再次用停止线
  if (!road_edge_pts.empty()) {
    HandleOppisiteLine(road_edge_pts);
  } else if (!occ_road_edge_pts.empty()) {
    HandleOppisiteLine(occ_road_edge_pts);
  } else if (!double_solid_yellow_pts.empty()) {
    HandleOppisiteLine(double_solid_yellow_pts);
  } else {
    // HandleOppisiteLineByStopline();停止线过滤目前不稳定
    // HandleOppisiteLineByObjAndYelloLine();  // 融合障碍物和黄线过滤对向车道
  }
}

void GeoOptimization::HandleOppisiteLineByObjAndYelloLine() {
  double max_y = -DBL_MAX;
  for (const auto& line_vec : all_lines_) {
    if (line_vec.second.empty()) {
      continue;
    }
    for (const auto& line : line_vec.second) {
      if (!line.is_ego_road) {
        continue;
      }

      if ((line.line->lanetype() ==
               hozon::mapping::LaneType::LaneType_DOUBLE_SOLID ||
           line.line->lanetype() == hozon::mapping::LaneType::LaneType_SOLID) &&
          line.line->color() == hozon::mapping::Color::YELLOW &&
          line.line->points_size() > 1 && line.line->points().at(0).x() > 0) {
        if (line.line->points().at(0).y() > max_y) {
          max_y = line.line->points().at(0).y();
        }
      }
    }
  }

  if (max_y == -DBL_MAX) {
    return;
  }
  for (auto& line_vec : all_lines_) {
    if (line_vec.second.empty()) {
      continue;
    }
    for (auto& line : line_vec.second) {
      if (!line.is_ego_road) {
        continue;
      }
      if (line.line->points_size() > 1 && line.line->points().at(0).x() > 0 &&
          line.line->points().at(0).y() > max_y) {
        line.is_ego_road = false;
      }
    }
  }
}

void GeoOptimization::HandleOppisiteLineByStopline() {
  if (!local_map_) {
    HLOG_ERROR << "local_map_ is nullptr";
    return;
  }
  // 获取前向stopline集合
  std::vector<std::vector<Eigen::Vector2f>> forward_stoplines;
  for (const auto& local_stopline : local_map_->stop_lines()) {
    if (!local_stopline.has_left_point() || !local_stopline.has_right_point() ||
        local_stopline.left_point().x() < 0 ||
        local_stopline.right_point().x() < 0) {
      continue;
    }
    Eigen::Vector2f pt_left(local_stopline.left_point().x(),
                            local_stopline.left_point().y());
    Eigen::Vector2f pt_right(local_stopline.right_point().x(),
                             local_stopline.right_point().y());
    std::vector<Eigen::Vector2f> stopline_pts{pt_left, pt_right};
    // 过滤离后向车道线back点距离很近的停止线
    bool is_stop_line_in_exit_lane = false;
    for (auto& line_vector : all_lines_) {
      if (line_vector.second.empty()) {
        continue;
      }
      for (auto& line : line_vector.second) {
        // 后向车道线
        if (line.line->points_size() < 2 ||
            (!line.line->points().empty() &&
             line.line->points().at(0).x() > 0)) {
          continue;
        }
        Eigen::Vector2f line_end_point(line.line->points().rbegin()->x(),
                                       line.line->points().rbegin()->y());
        if (PointToVectorDist(stopline_pts[0], stopline_pts[1],
                              line_end_point) < 5.0F) {
          is_stop_line_in_exit_lane = true;
          break;
        }
      }
      if (is_stop_line_in_exit_lane) {
        break;
      }
    }
    if (is_stop_line_in_exit_lane) {
      continue;
    }
    forward_stoplines.emplace_back(stopline_pts);
  }
  if (forward_stoplines.empty()) {
    return;
  }
  std::unordered_set<int> track_ids;
  for (auto& line_vector : all_lines_) {
    for (auto& line : line_vector.second) {
      track_ids.insert(line.line->track_id());
    }
  }
  for (auto it = is_not_ego_lane_track_id_.begin();
       it != is_not_ego_lane_track_id_.end();) {
    if (track_ids.find(*it) != track_ids.end()) {
      it++;
    } else {
      it = is_not_ego_lane_track_id_.erase(it);
    }
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
      if (is_not_ego_lane_track_id_.find(line.line->track_id()) !=
          is_not_ego_lane_track_id_.end()) {
        line.is_ego_road = false;
        continue;
      }
      // 前向车道线的第一个点能投影在前向停止线上,且投影距离<10,且停止线的右边点在车道线的右侧1.5米以外，
      // 且车道线的终点要在停止线的左边5m以外，认为是对向车道线
      Eigen::Vector2f line_first_point(line.line->points().at(0).x(),
                                       line.line->points().at(0).y());
      Eigen::Vector2f line_end_point(line.line->points().rbegin()->x(),
                                     line.line->points().rbegin()->y());
      for (auto& forward_stopline : forward_stoplines) {
        if (forward_stopline.size() < 2) {
          continue;
        }
        auto point_it = std::find_if(
            line.line->points().begin() + 1, line.line->points().end(),
            [&](const hozon::common::Point3D& point) {
              return PointInVectorSide(forward_stopline[0], forward_stopline[1],
                                       Eigen::Vector2f(point.x(), point.y())) <
                     0;
            });
        Eigen::Vector2f line_second_point;
        if (point_it == line.line->points().end()) {
          line_second_point << line.line->points().rbegin()->x(),
              line.line->points().rbegin()->y();
        } else {
          line_second_point << point_it->x(), point_it->y();
        }

        if (PointToVectorDist(forward_stopline[0], forward_stopline[1],
                              line_first_point) < 10 &&
            PointInVectorSide(line_first_point, line_second_point,
                              forward_stopline[1]) > 0 &&
            PointToVectorDist(line_first_point, line_second_point,
                              forward_stopline[1]) > 1.5 &&
            PointInVectorSide(forward_stopline[0], forward_stopline[1],
                              line_end_point) < 0 &&
            PointToVectorDist(forward_stopline[0], forward_stopline[1],
                              line_end_point) > 5.0f) {
          line.is_ego_road = false;
          if (line.line->track_id() < 1000) {
            is_not_ego_lane_track_id_.insert(line.line->track_id());
          }
        }
      }
    }
  }
  // for (const auto& id : is_not_ego_lane_track_id_) {
  //   HLOG_ERROR << "is_not_ego_lane_track_id: " << id;
  // }
}

// 10米内, 有车辆情况, 10米外, 有两辆以上运动的车辆
bool GeoOptimization::CheckOppisiteLineByObj(
    const std::vector<Eigen::Vector3d>& line_points) {
  std::vector<Eigen::Vector4d> obj_points;
  for (const auto& object : inverse_history_objs_) {
    Eigen::Vector3d p_local(object.position().x(), object.position().y(),
                            object.position().z());
    Eigen::Vector3d p_veh = T_U_V_.inverse() * p_local;
    Eigen::Vector4d p_veh_sp;
    p_veh_sp << p_veh, object.velocity().x();
    obj_points.emplace_back(p_veh_sp);
  }
  if (obj_points.empty()) {
    return false;
  }
  int line_size = static_cast<int>(line_points.size());
  if (line_size < 2 || line_points.at(0).x() < 0) {
    return false;
  }
  int right_vehicle_num = 0;
  for (auto& obj_point : obj_points) {
    if (obj_point(0) > line_points[line_size - 1].x() + 10 ||
        obj_point(0) < line_points[0].x()) {
      continue;
    }
    int num_thresh = 0;
    int num_calculate = 0;
    for (int line_index = 0; line_index < line_size - 1; line_index++) {
      Eigen::Vector3d linel1(line_points[line_index].x(),
                             line_points[line_index].y(),
                             line_points[line_index].z());
      Eigen::Vector3d linel2(line_points[line_index + 1].x(),
                             line_points[line_index + 1].y(),
                             line_points[line_index + 1].z());
      num_calculate++;
      Eigen::Vector3d v_3d(obj_point.head<3>());
      if (IsRight(v_3d, linel1, linel2) &&
          PointToVectorDist(linel1, linel2, v_3d) > 5.0) {
        num_thresh++;
      }
    }
    if (num_thresh >= 1 &&
        (num_calculate != 0 && (static_cast<double>(num_thresh) /
                                static_cast<double>(num_calculate)) > 0.5)) {
      // is_right_occ = true;
      // 10米范围,有车就算, 10米外要两辆车, 且速度 < 0
      if (std::fabs(obj_point(0) - line_points[0].x()) < 10) {
        return true;
      } else {
        if (obj_point(3) < -1) {
          ++right_vehicle_num;
        }
        if (right_vehicle_num >= 2) {
          return true;
        }
      }
    }
  }
  return false;
}

void GeoOptimization::HandleOppisiteLineByObj() {
  std::vector<Eigen::Vector3d> obj_points;
  for (const auto& history_obj : history_objs_) {
    for (const auto& object : history_obj->perception_obstacle()) {
      Eigen::Vector3d p_local(object.position().x(), object.position().y(),
                              object.position().z());
      Eigen::Vector3d p_veh = T_U_V_.inverse() * p_local;
      if (object.type() != hozon::perception::PerceptionObstacle::VEHICLE ||
          p_veh.x() <= 0 || object.velocity().x() > 0 ||
          (PI * 3 / -4 < object.theta() && object.theta() < PI * 3 / 4)) {
        continue;
      }
      // 如果是自车右侧逆向障碍物，且是静止状态(速度在正负１m/s之间，则不用)
      if (p_veh.y() < -2.5 && std::fabs(object.velocity().x()) < 1) {
        continue;
      }
      // 筛选出在车道线中间的object
      bool obj_on_line_left = false;
      bool obj_on_line_right = false;
      for (auto& line_vector : all_lines_) {
        if (line_vector.second.empty()) {
          continue;
        }
        for (auto& line : line_vector.second) {
          int line_size = line.line->points_size();
          if (line_size < 2 || (!line.line->points().empty() &&
                                line.line->points().at(0).x() < 0)) {
            continue;
          }

          if (p_veh.x() < line.line->points(0).x() ||
              p_veh.x() > line.line->points(line_size - 1).x()) {
            continue;
          }
          int line_index = 0;
          // const auto& points = line.line->points();
          // auto it = std::lower_bound(points.begin(), points.end(),
          // p_veh.x(),
          //                            [&](const hozon::common::Point3D& a,
          //                                double b) { return a.x() < b; });
          for (; line_index < line_size - 1; line_index++) {
            if (line.line->points(line_index).x() <= p_veh.x() &&
                line.line->points(line_index + 1).x() >= p_veh.x()) {
              break;
            }
          }

          Eigen::Vector3d linel1(line.line->points(line_index).x(),
                                 line.line->points(line_index).y(),
                                 line.line->points(line_index).z());
          Eigen::Vector3d linel2(line.line->points(line_index + 1).x(),
                                 line.line->points(line_index + 1).y(),
                                 line.line->points(line_index + 1).z());
          if (IsRight(p_veh, linel1, linel2)) {
            obj_on_line_right = true;
          } else {
            obj_on_line_left = true;
          }
        }
      }
      if (obj_on_line_left && obj_on_line_right) {
        obj_points.emplace_back(p_veh);
      }
    }
  }
  if (obj_points.empty()) {
    return;
  }
  for (auto& line_vector : all_lines_) {
    if (line_vector.second.empty()) {
      continue;
    }
    for (auto& line : line_vector.second) {
      int line_size = line.line->points_size();
      if (line_size < 2 ||
          (!line.line->points().empty() && line.line->points().at(0).x() < 0)) {
        continue;
      }
      for (auto& obj_point : obj_points) {
        bool stop_loop = false;
        if (obj_point.x() > line.line->points(line_size - 1).x() ||
            obj_point.x() < line.line->points(0).x()) {
          continue;
        }
        for (int line_index = 0; line_index < line_size - 1; line_index++) {
          if (line.line->points(line_index).x() <= obj_point.x() &&
              line.line->points(line_index + 1).x() >= obj_point.x()) {
            Eigen::Vector3d linel1(line.line->points(line_index).x(),
                                   line.line->points(line_index).y(),
                                   line.line->points(line_index).z());
            Eigen::Vector3d linel2(line.line->points(line_index + 1).x(),
                                   line.line->points(line_index + 1).y(),
                                   line.line->points(line_index + 1).z());
            if (IsRight(obj_point, linel1, linel2)) {
              line.is_ego_road = false;
              stop_loop = true;
              break;
            }
          }
        }
        if (stop_loop) {
          break;
        }
      }
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
        double avg_width = 0.0;
        if (!ComputerLineDis(road_edge, line_pts, &avg_width)) {
          continue;
        }
        if (IsTargetOnLineRight(road_edge, line_pts) ==
                RelativePosition::RIGHT &&
            avg_width > 3.0) {
          have_left_line = true;
        } else if (IsTargetOnLineRight(road_edge, line_pts) ==
                       RelativePosition::LEFT &&
                   avg_width > 3.0) {
          have_right_line = true;
        }
        if (have_left_line && have_right_line) {
          double road_edge_heading = CalMeanLineHeading(road_edge);
          if (IsRoadEdgeOnVehicleRight(road_edge, road_edge_heading) ==
              RelativePosition::LEFT) {
            res = road_edge;
            return res;
          }
        }
      }
    }
  }
  return res;
}

std::vector<Eigen::Vector3d> GeoOptimization::FindTargetPointsNoCrossing(
    const std::vector<std::vector<Eigen::Vector3d>>& nearby_road_edges) {
  std::vector<Eigen::Vector3d> res;
  for (const auto& road_edge : nearby_road_edges) {
    if (road_edge.size() < 2 ||
        (!road_edge.empty() && road_edge.at(0).x() > 0)) {
      continue;
    }

    bool have_left_line = false;
    bool have_right_line = false;
    for (const auto& line_vec : all_lines_) {
      if (line_vec.second.empty()) {
        continue;
      }
      for (const auto& line : line_vec.second) {
        if (line.line->points_size() < 2 ||
            (!line.line->points().empty() &&
             line.line->points().at(0).x() > 0)) {
          continue;
        }
        std::vector<double> widths;
        std::vector<Eigen::Vector3d> line_pts;
        for (const auto& it : line.line->points()) {
          Eigen::Vector3d pt(it.x(), it.y(), it.z());
          line_pts.emplace_back(pt);
        }
        double avg_width = 0.0;
        if (!ComputerLineDis(road_edge, line_pts, &avg_width)) {
          continue;
        }
        if (IsTargetOnLineRight(road_edge, line_pts) ==
                RelativePosition::RIGHT &&
            avg_width > 3.0) {
          have_left_line = true;
        } else if (IsTargetOnLineRight(road_edge, line_pts) ==
                       RelativePosition::LEFT &&
                   avg_width > 3.0) {
          have_right_line = true;
        }
        if (have_left_line && have_right_line) {
          if (IsRoadEdgeOnVehicleRightNocrossing(road_edge) ==
              RelativePosition::LEFT) {
            // 比较res和road_edge哪个离车更近——x最小的点的y值大小
            if (res.empty()) {
              res = road_edge;
            } else {
              Eigen::Vector3d res_point = res[0];
              Eigen::Vector3d road_edge_point = road_edge[0];
              for (auto point : res) {
                if (std::abs(point.x()) < std::abs(res_point.x())) {
                  res_point = point;
                }
              }
              for (auto point : road_edge) {
                if (std::abs(point.x()) < std::abs(road_edge_point.x())) {
                  road_edge_point = point;
                }
              }
              if (res_point.y() > road_edge_point.y()) {
                res = road_edge;
              }
            }
          }
        }
      }
    }
  }
  return res;
}

RelativePosition GeoOptimization::IsRoadEdgeOnVehicleRight(
    const std::vector<Eigen::Vector3d>& points, const double& heading) {
  if (points.empty() || std::abs(heading) > PI) {
    return RelativePosition::UNCERTAIN;
  }
  Eigen::Vector3d p1(0, 0, 0);
  Eigen::Vector3d p2(cos(heading), sin(heading), 0);
  int num_thresh = 0;
  int num_calculate = 0;
  for (const auto& point : points) {
    if (std::isnan(point.x()) || std::isnan(point.y()) ||
        std::isnan(point.z())) {
      HLOG_ERROR << "found nan point in road_edge_line";
      continue;
    }
    num_calculate++;
    if (IsRight(point, p1, p2)) {
      num_thresh++;
    }
  }
  if (num_thresh < 1 ||
      (num_calculate != 0 && (static_cast<double>(num_thresh) /
                              static_cast<double>(num_calculate)) < 0.5)) {
    return RelativePosition::LEFT;
  }
  return RelativePosition::RIGHT;
}

RelativePosition GeoOptimization::IsRoadEdgeOnVehicleRightNocrossing(
    const std::vector<Eigen::Vector3d>& points) {
  if (points.size() < 2) {
    return RelativePosition::UNCERTAIN;
  }
  Eigen::Vector3d nearst_point{180, 180, 0};
  for (const auto& point : points) {
    if (point.norm() < nearst_point.norm()) {
      nearst_point = point;
    }
  }
  if (nearst_point.y() > 0) {
    return RelativePosition::LEFT;
  }
  return RelativePosition::RIGHT;
}

double GeoOptimization::CalMeanLineHeading(
    const std::vector<Eigen::Vector3d>& points) {
  if (points.size() < 2) {
    return 0;
  }
  double mean_theta = 0.;
  double count = 0;
  for (size_t i = 0; i < points.size() - 1; i++) {
    size_t j = i + 1;
    for (; j < points.size(); j++) {
      const Eigen::Vector2f pa(points[i].x(), points[i].y());
      const Eigen::Vector2f pb(points[j].x(), points[j].y());
      Eigen::Vector2f v = pb - pa;
      if (v.norm() > 1.0) {
        double theta = 0;
        if (std::abs(v.x()) > 1e-2) {
          theta = atan2(v.y(), v.x());  // atan2计算出的角度范围是[-pi, pi]
          mean_theta += theta;
          count++;
          i = j;
          break;
        }
      }
    }
  }
  if (count > 0) {
    mean_theta /= count;
  }
  return mean_theta;
}

RelativePosition GeoOptimization::IsTargetOnLineRight(
    const std::vector<Eigen::Vector3d>& target_line,
    const std::vector<Eigen::Vector3d>& line) {
  int line_size = line.size();
  if (line_size < 2 || target_line.size() < 1) {
    return RelativePosition::UNCERTAIN;
  }
  // 　对没用重合区域的两根线用首尾点的ｙ判断左右
  if (target_line.front().x() >= line.back().x()) {
    if (target_line.front().y() > line.back().y()) {
      return RelativePosition::LEFT;
    } else if (target_line.front().y() < line.back().y()) {
      return RelativePosition::RIGHT;
    }
    return RelativePosition::UNCERTAIN;
  }
  if (line.front().x() >= target_line.back().x()) {
    if (target_line.back().y() > line.front().y()) {
      return RelativePosition::LEFT;
    } else if (target_line.back().y() < line.front().y()) {
      return RelativePosition::RIGHT;
    }
    return RelativePosition::UNCERTAIN;
  }
  int num_thresh = 0, num_calculate = 0;
  for (int target_index = 0, line_index = 0; target_index < target_line.size();
       target_index++) {
    if (std::isnan(target_line.at(target_index).x()) ||
        std::isnan(target_line.at(target_index).y()) ||
        std::isnan(target_line.at(target_index).z())) {
      HLOG_ERROR << "found nan point in double_solid_yellow_line or "
                    "road_edge_line";
      continue;
    }
    if (target_line.at(target_index).x() < line.front().x()) {
      continue;
    }
    while (line_index < line_size - 1 &&
           line[line_index + 1].x() < target_line.at(target_index).x()) {
      line_index++;
    }
    if (line_index == line_size - 1) {
      break;
    }
    num_calculate++;
    if (PointInVectorSide(line[line_index], line[line_index + 1],
                          target_line.at(target_index)) > 0.0) {
      num_thresh++;
    }
  }
  if (num_thresh < 1 ||
      (num_calculate != 0 && (static_cast<double>(num_thresh) /
                              static_cast<double>(num_calculate)) < 0.5)) {
    return RelativePosition::LEFT;
  }
  return RelativePosition::RIGHT;
}

void GeoOptimization::HandleOppisiteLineNoCrossing(
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
          (!line.line->points().empty() && line.line->points().at(0).x() > 0)) {
        continue;
      }
      std::vector<double> widths;
      std::vector<Eigen::Vector3d> line_pts;
      for (const auto& it : line.line->points()) {
        Eigen::Vector3d pt(it.x(), it.y(), it.z());
        line_pts.emplace_back(pt);
      }
      double avg_width = 0.0;
      if (!ComputerLineDis(target_line, line_pts, &avg_width)) {
        continue;
      }
      if (IsTargetOnLineRight(target_line, line_pts) ==
              RelativePosition::RIGHT &&
          avg_width > 2.0) {
        line.is_ego_road = false;
      }
    }
  }
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
      double avg_width = 0.0;
      if (!ComputerLineDis(target_line, line_pts, &avg_width)) {
        continue;
      }
      if (IsTargetOnLineRight(target_line, line_pts) ==
              RelativePosition::RIGHT &&
          avg_width > 2.0) {
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
  std::uniform_int_distribution<int> dist(0, static_cast<int>(data.size()) - 1);

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
  //     angel = accumulate(angle_all.begin() + i, angle_all.begin() +
  //     tmp_num
  //     + 1,
  //                        0.0) /
  //             static_cast<double>(num);
  //     slope_ = angel;
  //   }
  //   if (num > (angle_all.size() - 1) / 2) break;  //
  //   如果已经是众数了，就退出 i = tmp_num + 1;
  // }
  // int i = 0;
  // for (auto& line : all_lines_) {
  //   for (auto& line_vector : line.second) {
  //     if (line_vector.store) {
  //       // double len = LineLength(line_vector.line_points);
  //       int len = line_vector.line->points_size();
  //       // HLOG_ERROR<<"len = "<<len;
  //       if (len < short_thresh && abs(a_tmp[i] - angel) > angle_thresh)
  //       {
  //         line_vector.store = false;
  //         HLOG_ERROR << "len = " << len << "  a_tmp[i] = " << a_tmp[i];
  //       }
  //     }
  //     i++;
  //   }
  // }
  // HLOG_ERROR << "accurate angel = " << angel << "  num = " << num << "
  // ";
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

  // std::vector<std::vector<double>> line_param;  // 参与计算的c0 angel
  // c2 std::vector<int> index;   // 参与聚类算法的线中点的index
  // std::vector<int> lable;   // 将线分成几种类型
  // std::vector<double> dis;  // 计算中位数

  // int size_line = all_lines_.size();
  // if (size_line < 2) {
  //   return;
  // }
  // for (auto lines_ = all_lines_.begin(); lines_ != all_lines_.end();
  // lines_++) {
  //   int lanepos1 =
  //   static_cast<int>(lines_->second.begin()->line->lanepos());
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
  //   b = std::max(tmp[dis.size() / 2], 3.2);  // 不能小于三米
  //   正常车道间距
  // }

  // // HLOG_ERROR<<"b = "<<b;
  // // 计算每根pos的斜率等
  // for (const auto& lines_ : all_lines_) {
  //   std::vector<Eigen::Vector3d> line;
  //   for (const auto& line_vector : lines_.second) {
  //     // for (const auto& point : line_vector.line_points) {
  //     //   if (point.x() > -20 && point.x() < 20)
  //     line.emplace_back(point);
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
  //              << "   atan(res[1]) / PI * 180=" << atan(res[1]) / PI *
  //              180
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
  //       distance_mat(i, j) = IsNeighbor(line_param[i], line_param[j],
  //       b); distance_mat(j, i) = distance_mat(i, j);
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
    const std::shared_ptr<hozon::mapping::LocalMap>& msg,
    const std::shared_ptr<hozon::perception::PerceptionObstacles>& obj_msg) {
  if (msg->lane_lines().empty()) {
    HLOG_WARN << "lane lines empty!";
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
  // local_map_ = std::make_shared<hozon::mapping::LocalMap>();
  // local_map_->CopyFrom(*msg);
  local_map_ = msg;

  if (obj_msg != nullptr) {
    per_objs_ = std::make_shared<hozon::perception::PerceptionObstacles>();
    per_objs_->CopyFrom(*obj_msg);
    // 预处理障碍物
    if (per_objs_->perception_obstacle_size() < 1) {
      geo_obj_ids_.clear();
      // geo_objs_.clear();
    }
    for (auto& obj : geo_objs_) {
      obj.second.is_update = false;
    }
    for (auto& object : *per_objs_->mutable_perception_obstacle()) {
      geo_obj_ids_.clear();
      geo_obj_ids_.insert(-1);
      if (object.type() == hozon::perception::PerceptionObstacle::VEHICLE &&
          object.position().x() > 0) {
        if (geo_objs_.find(object.id()) == geo_objs_.end()) {
          auto& obj = geo_objs_[object.id()];
          obj.Init(objs_size_);
        }
        auto& obj = geo_objs_[object.id()];
        Eigen::Vector3d p_veh(object.position().x(), object.position().y(),
                              object.position().z());
        auto p_v = T_U_V_ * p_veh;
        obj.id = object.id();
        obj.position = p_veh;
        obj.his_positions.push_back(p_v);
        obj.velocity =
            Eigen::Vector3d(object.velocity().x(), object.velocity().y(),
                            object.velocity().z());
        obj.theta = object.theta();
        obj.length = object.length();
        obj.width = object.width();
        obj.is_update = true;
        if (object.velocity().x() <= 0 &&
            (object.theta() < PI * 3 / -4 || object.theta() > PI * 3 / 4)) {
          obj.is_inverse = true;
        }
      }
    }
    for (auto it = geo_objs_.begin(); it != geo_objs_.end();) {
      if (it->second.is_update) {
        geo_obj_ids_.insert(it->first);
        it++;
      } else {
        if (it->first != -1) {
          it = geo_objs_.erase(it);
        } else {
          it++;
        }
      }
    }
    // end
    for (auto& object : *per_objs_->mutable_perception_obstacle()) {
      Eigen::Vector3d p_veh(object.position().x(), object.position().y(),
                            object.position().z());
      auto p_v = T_U_V_ * p_veh;
      object.mutable_position()->set_x(p_v.x());
      object.mutable_position()->set_y(p_v.y());
      object.mutable_position()->set_z(0.0);
      if (object.type() == hozon::perception::PerceptionObstacle::VEHICLE &&
          p_veh.x() > 0 && object.velocity().x() <= 0 &&
          (object.theta() < PI * 3 / -4 || object.theta() > PI * 3 / 4)) {
        inverse_history_objs_.push_back(object);
      }
    }
    history_objs_.push_back(per_objs_);
  }

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
  // 3.
  // 根据前后车道线的关系距离是否过近来融合相邻车道线并填入local_map_use_
  // 4.
  // 处理merge或者split对线段进行拆分，并填入element_map包括line的前后继关系
  // 还是根据具体case分析具体问题
  FilterLocalMapLine(local_map_);
  // 路沿标记本road以外的车道
  // FilterOppositeLine();

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

  // 处理OCC提取路沿豁口，并填入elem_map_.occ_roads
  ExtractOccRoadGap();

  // 生成路口前方的虚拟车道线到all_lines_中.
  HLOG_INFO << "[debug mem boost] start ConstructOccGuideLine...";
  ConstructOccGuideLine();

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

  // 处理路口场景逆向车道车道线
  FilterOpeningLine();
  // FilterReverseLine();

  // 过滤非路口场景非主路车道线
  FilterNoEgoLineNoCrossing();

  // 将输出填入elementmap
  AppendElemtMap(local_map_use_, per_objs_);

  // VizOccRoad();
}

void GeoOptimization::ExtractOccRoadGap() {
  // 通过OCC提取路沿豁口信息
  /*
         路口场景
        |        |
        |        |
        |        |
        -        - ------目标豁口
        /         \
       /           \



       \           /
        \         /
         | | | | |
         | | |↑| |
         | | | | |
  方案： 1.获取起始点大于0的occ，并按照对应的y值从小到大进行排序
        2.计算相邻两根车道线平均距离
        3.判断是否属于同一个包络，如果是，继续遍历，如果不是判断两根线能否构成道；
  */
  elem_map_ = std::make_shared<hozon::mp::mf::em::ElementMap>();
  elem_map_->map_info.stamp = local_map_use_->header().data_stamp();

  // 模型路沿
  for (const auto& road_edge_it : local_map_->road_edges()) {
    // 过滤空的点
    if (road_edge_it.points_size() <= 0) {
      continue;
    }
    hozon::mp::mf::em::Boundary road_edge;
    auto point_size = road_edge_it.points().size();
    for (const auto& line_point_it : road_edge_it.points()) {
      if (std::isnan(line_point_it.x()) || std::isnan(line_point_it.y()) ||
          std::isnan(line_point_it.z())) {
        HLOG_ERROR << "found nan in localmap";
        continue;
      }
      Eigen::Vector3f point_local(line_point_it.x(), line_point_it.y(),
                                  line_point_it.z());
      // Eigen::Vector3d point_enu = T_U_V_ * point_local;
      hozon::mp::mf::em::BoundaryNode node;
      node.point = point_local;
      road_edge.nodes.push_back(
          std::make_shared<hozon::mp::mf::em::BoundaryNode>(node));
    }
    road_edge.id = road_edge_it.track_id();
    FillLanePos(&road_edge, road_edge_it.lanepos());
    // FillLaneType(&road_edge, road_edge_it.lanetype());
    // FillLaneColor(&road_edge, road_edge_it.color());
    // lane_line.boundary_type = lane_line_it.lanetype();
    elem_map_->road_edges[road_edge.id] =
        std::make_shared<hozon::mp::mf::em::Boundary>(road_edge);
  }

  if (local_map_->occs().empty()) {
    HLOG_WARN << "occs is empty!";
    return;
  }
  // if (local_map_->road_edges().empty()) {
  //   HLOG_WARN << "road edge is empty!";
  //   return;
  // }
  std::vector<std::pair<int, em::OccRoad>> vec_occ_line;
  // 找自车所在车道线远端x最大值

  double max_line_x = std::numeric_limits<double>::min();
  for (const auto& line : local_map_->lane_lines()) {
    if (line.points().empty()) {
      continue;
    }
    auto start_point = line.points(0);
    if (start_point.x() > 0.0) {
      continue;
    }
    auto end_point = line.points(line.points_size() - 1);
    if (end_point.x() > max_line_x) {
      max_line_x = end_point.x();
    }
  }
  // OCC路沿
  for (const auto& occ : local_map_->occs()) {
    // 注意！！!一个occ表示一根线，如果occ中的detect_id
    // 一致表明两根线属于同一个包络 过滤point_size小于2和起始点x小于0的occ
    if (occ.points_size() < 2 || occ.points().at(0).x() < 0) {
      continue;
    }
    // 根据曲线参数和起始点进行采样
    std::vector<Eigen::Vector3d> new_line_pts;
    std::vector<float> curve_params;
    if (!LineCublicSampling(occ, &curve_params, &new_line_pts)) {
      continue;
    }

    if (new_line_pts.empty()) {
      continue;
    }

    if (OccLineFitError(occ, curve_params) > 2.0) {
      HLOG_DEBUG << "OccLineFitError id: " << occ.track_id()
                 << ", detect_id: " << occ.detect_id();
      continue;
    }
    // 根据障碍物过滤对向occ路沿
    if (CheckOppisiteLineByObj(new_line_pts)) {
      HLOG_DEBUG << "CheckOppisiteLineByObj id: " << occ.track_id()
                 << ", detect_id: " << occ.detect_id();
      continue;
    }
    // HLOG_ERROR << "XXXXXXXXXXXXXXXXnew_line_pts: " <<
    // new_line_pts.size();
    em::OccRoad occ_road;
    // 注释掉这个距离过滤。
    // if (new_line_pts.front().x() < max_line_x) {
    //   continue;
    // }
    for (const auto& pt : occ.points()) {
      Eigen::Vector3d p(pt.x(), pt.y(), pt.z());
      occ_road.ori_detect_points.emplace_back(p);
    }
    for (const auto& pt : new_line_pts) {
      Eigen::Vector3d point(pt.x(), pt.y(), pt.z());
      occ_road.road_points.emplace_back(point);
    }
    if (occ_road.road_points.empty()) {
      continue;
    }
    occ_road.track_id = occ.track_id();
    occ_road.detect_id = occ.detect_id();
    occ_road.curve_params = curve_params;
    vec_occ_line.emplace_back(occ.track_id(), occ_road);
  }
  // 按照纵向距离对occ_road进行分组
  // 对occ_line按照每根线的y值平均值大小进行排序
  std::sort(vec_occ_line.begin(), vec_occ_line.end(),
            [](const auto& a, const auto& b) {
              double sumY_A = 0.0, sumY_B = 0.0;
              for (const auto& vec : a.second.road_points) {
                sumY_A += vec.y();
              }
              double avgY_A = sumY_A / a.second.road_points.size();
              for (const auto& vec : b.second.road_points) {
                sumY_B += vec.y();
              }
              double avgY_B = sumY_B / b.second.road_points.size();
              return avgY_A < avgY_B;
            });

  // 遍历vec_occ_line，计算平均距离，判断是否是一个包络，并判断能否构成道
  // if (!vec_occ_line.empty()) {
  //   // 临时修改策略
  //   std::vector<std::vector<std::pair<int, em::OccRoad>>> groupedLines;
  //   std::vector<std::pair<int, em::OccRoad>> currentGroup;
  //   for (const auto& line : vec_occ_line) {
  //     double ave_width = std::numeric_limits<double>::max();
  //     if (!currentGroup.empty()) {
  //       ComputerLineDis(line.second.road_points,
  //                       currentGroup.back().second.road_points,
  //                       &ave_width);
  //     }
  //     if (currentGroup.empty() || std::abs(ave_width) <= 1.0) {
  //       currentGroup.push_back(line);
  //     } else {
  //       groupedLines.push_back(currentGroup);
  //       currentGroup.clear();
  //       currentGroup.push_back(line);
  //     }
  //   }
  //   if (!currentGroup.empty()) {
  //     groupedLines.push_back(currentGroup);
  //   }
  //   // 组与组进行比较,返回满足条件的id对
  //   // std::vector<std::pair<int, int>> line_pairs;
  //   // CompareGroupLines(&groupedLines, &line_pairs);
  // }
  if (FindOCCGuidePoint()) {
    UpdateOCCRoadPoints();
  }

  elem_map_->occ_roads.clear();
  for (const auto& occ : local_map_->occs()) {
    if (elem_map_->occ_roads.find(occ.track_id()) !=
        elem_map_->occ_roads.end()) {
      continue;
    }
    em::OccRoad occ_road;
    occ_road.track_id = occ.track_id();
    // 根据曲线参数和起始点进行采样
    std::vector<Eigen::Vector3d> new_line_pts;
    std::vector<float> curve_params;
    if (!LineCublicSampling(occ, &curve_params, &new_line_pts)) {
      continue;
    }
    if (new_line_pts.empty()) {
      continue;
    }
    if (CheckOppisiteLineByObj(new_line_pts)) {
      HLOG_DEBUG << "CheckOppisiteLineByObj id: " << occ.track_id()
                 << ", detect_id: " << occ.detect_id();
      continue;
    }
    for (const auto& pt : new_line_pts) {
      Eigen::Vector3d point(pt.x(), pt.y(), pt.z());
      occ_road.road_points.emplace_back(point);
    }
    for (const auto& pt : occ.points()) {
      Eigen::Vector3d p(pt.x(), pt.y(), pt.z());
      occ_road.ori_detect_points.emplace_back(p);
    }
    occ_road.is_forward = false;
    elem_map_->occ_roads[occ_road.track_id] =
        std::make_shared<em::OccRoad>(occ_road);
  }
}

void GeoOptimization::ConstructOccGuideLine() {
  // 根据occ、车道线、模型路沿等信息生成路口前方的虚拟车道线，
  // 并实时更新其位置
  occ_guideline_manager_->Process(elem_map_, &all_lines_, curr_pose_,
                                  road_scene_);
  exit_lane_info_ = occ_guideline_manager_->GetExitLaneInfo();
  stable_occ_roads_ = occ_guideline_manager_->GetStableOccRoads();
}

void GeoOptimization::CompareOccLines(
    const std::vector<std::pair<int, em::OccRoad>>& left_group,
    const std::vector<std::pair<int, em::OccRoad>>& right_group,
    std::vector<std::pair<int, int>>* line_pairs) {
  for (const auto& occ1 : left_group) {
    bool flag = false;
    for (const auto& occ2 : right_group) {
      auto occ_l1 = occ1.second.road_points;
      auto occ_l2 = occ2.second.road_points;

      double ave_width = 0.0;
      ComputerLineDis(occ_l1, occ_l2, &ave_width);
      if (ave_width < 5) {
        continue;
      }
      auto overlay_ratio = GetOverLayRatioBetweenTwoLane(occ_l1, occ_l2);
      HLOG_DEBUG << "GetOverLayRatioBetweenTwoLane occ1: " << occ1.first
                 << ", y: " << occ_l1.back().y() << ", occ2: " << occ2.first
                 << ", y: " << occ_l2.back().y() << ", " << overlay_ratio;
      if (overlay_ratio < 0.1) {
        continue;
      }

      // 满足构建道的要求occ塞到element_map中
      if (elem_map_->occ_roads.find(occ1.first) == elem_map_->occ_roads.end()) {
        em::OccRoad occ_road1;
        occ_road1.track_id = occ1.first;
        occ_road1.group_id = 1;
        occ_road1.curve_params = occ1.second.curve_params;
        occ_road1.road_points = occ1.second.road_points;
        occ_road1.ori_road_points = occ1.second.road_points;
        occ_road1.ori_detect_points = occ1.second.ori_detect_points;
        // occ_road1.left_occ_id = occ2.first;
        // occ_road1.is_forward = true;
        elem_map_->occ_roads[occ_road1.track_id] =
            std::make_shared<em::OccRoad>(occ_road1);
      } else {
        if (elem_map_->occ_roads[occ1.first]->left_occ_id == -1) {
          elem_map_->occ_roads[occ1.first]->left_occ_id = occ2.first;
        }
      }

      if (elem_map_->occ_roads.find(occ2.first) == elem_map_->occ_roads.end()) {
        em::OccRoad occ_road2;
        occ_road2.track_id = occ2.first;
        occ_road2.group_id = 1;
        occ_road2.curve_params = occ2.second.curve_params;
        occ_road2.road_points = occ2.second.road_points;
        occ_road2.ori_road_points = occ2.second.road_points;
        occ_road2.ori_detect_points = occ2.second.ori_detect_points;
        // occ_road2.right_occ_id = occ1.first;
        // occ_road2.is_forward = true;
        elem_map_->occ_roads[occ_road2.track_id] =
            std::make_shared<em::OccRoad>(occ_road2);
      }

      line_pairs->emplace_back(std::make_pair(occ1.first, occ2.first));
      flag = true;
      break;
    }
    if (flag) {
      break;
    }
  }
}

void GeoOptimization::CompareGroupLines(
    std::vector<std::vector<std::pair<int, em::OccRoad>>>* groupedLines,
    std::vector<std::pair<int, int>>* line_pairs) {
  // 组与组进行比较，返回id对
  if (groupedLines->empty()) {
    return;
  }

  std::vector<std::pair<int, em::OccRoad>> curr_group;
  for (int i = 0; i < static_cast<int>(groupedLines->size()); i++) {
    // 对group中的line按照x的从小到大排序
    const auto& left_group = groupedLines->at(i);
    for (int j = i + 1; j < static_cast<int>(groupedLines->size()); ++j) {
      const auto& right_group = groupedLines->at(j);
      // 计算curr_group与group中成对的路沿
      CompareOccLines(left_group, right_group, line_pairs);
    }
  }
}
bool GeoOptimization::FindOCCGuidePoint() {
  if (nullptr == elem_map_) {
    return false;
  }
  if (elem_map_->occ_roads.empty()) {
    return false;
  }
  std::vector<em::OccRoad::Ptr> find_occs;
  std::vector<em::OccRoad::Ptr> lost_occs;
  for (auto& occ_road : elem_map_->occ_roads) {
    if (occ_road.second == nullptr) {
      continue;
    }
    if ((!occ_road.second->is_forward) ||
        (occ_road.second->road_points.empty())) {
      continue;
    }
    int point_index = -1;
    if (GetFirstOCCPoints(occ_road.second, &point_index)) {
      occ_road.second->guide_index = point_index;
      find_occs.push_back(occ_road.second);
    } else {
      // lost_occs.push_back(occ_road.second);
      occ_road.second->is_forward = false;
    }
  }
  // 斜率找不到的用已经找到的线来找第一个点
  for (const auto& occ : lost_occs) {
    int point_index = -1;
    GetFirstNearIndex(find_occs, occ, &point_index);
    occ->guide_index = point_index;
  }
  for (auto& occ_road : elem_map_->occ_roads) {
    HLOG_DEBUG << "FindOCCGuidePoint track_id:" << occ_road.second->track_id
               << ", guide_index :" << occ_road.second->guide_index
               << ", group_id:" << occ_road.second->group_id
               << ", left_occ_id:" << occ_road.second->left_occ_id
               << ", right_occ_id" << occ_road.second->right_occ_id;
  }
  return true;
}

void GeoOptimization::UpdateOCCRoadPoints() {
  for (auto& occ_road : elem_map_->occ_roads) {
    if (occ_road.second == nullptr) {
      occ_road.second->is_forward = false;
      continue;
    }
    auto guide_index = occ_road.second->guide_index;
    if (guide_index >
        static_cast<int>(occ_road.second->road_points.size()) - 1) {
      occ_road.second->is_forward = false;
      continue;
    }
    if (guide_index == -1) {
      occ_road.second->is_forward = false;
      continue;
    }
    // 保留至少两个点
    occ_road.second->guide_index = std::min(
        guide_index, static_cast<int>(occ_road.second->road_points.size() - 2));
    occ_road.second->road_points.erase(
        occ_road.second->road_points.begin(),
        occ_road.second->road_points.begin() + occ_road.second->guide_index);
  }
}

template <typename T1, typename T2>
float GeoOptimization::evalueHeadingDiff(const T1& x,
                                         const std::vector<T2>& params) {
  float sum = 0.0;
  float val = 1.0;
  // 二阶导
  if (params.size() == 4) {
    sum = 6.0 * params[3] * x + 2 * params[2];
  }
  return sum;
}

double GeoOptimization::OccWidth(const em::OccRoad::Ptr& occ_road_ptr) {
  double width = std::numeric_limits<double>::max();
  if (occ_road_ptr->road_points.size() < 6) {
    return width;
  }
  width = 0.0;
  Eigen::Vector3d start_pt = occ_road_ptr->road_points.front();
  Eigen::Vector3d end_pt = occ_road_ptr->road_points.back();
  for (const auto& pt : occ_road_ptr->road_points) {
    double dist = GetDistPointLine(pt, start_pt, end_pt);
    if (dist > width) {
      width = dist;
    }
  }
  return width;
}

bool GeoOptimization::GetFirstOCCPoints(const em::OccRoad::Ptr& occ_road_ptr,
                                        int* first_point_index) {
  if (occ_road_ptr->road_points.size() < 6) {
    return false;
  }
  std::vector<float> heading_vec;
  double width = OccWidth(occ_road_ptr);
  HLOG_DEBUG << "GetFirstOCCPoints track_id: " << occ_road_ptr->track_id
             << ", width: " << width;
  if (width < 0.5) {
    *first_point_index = 0;
    return true;
  }
  for (const auto& pt : occ_road_ptr->road_points) {
    auto heading = evalueHeadingDiff(pt.x(), occ_road_ptr->curve_params);
    HLOG_DEBUG << "x: " << pt.x() << ", " << heading;
    heading_vec.push_back(heading);
  }
  int count_low_slobe = 0;
  bool get_first_pt = false;
  for (int i = 0; i < static_cast<int>(heading_vec.size()) - 2; ++i) {
    if (std::abs(heading_vec[i]) < 0.02) {
      count_low_slobe++;
      if (count_low_slobe >= 3) {
        get_first_pt = true;
        *first_point_index = i - 2;
        break;
      }
    } else {
      count_low_slobe = 0;
    }
  }
  const auto& ori_detect_pts = occ_road_ptr->ori_detect_points;
  std::vector<hozon::common::math::Vec2d> new_fit_pts;
  if (get_first_pt && ori_detect_pts.size() - *first_point_index > 8) {
    for (int i = *first_point_index;
         i < static_cast<int>(ori_detect_pts.size()); ++i) {
      new_fit_pts.emplace_back(ori_detect_pts[i].x(), ori_detect_pts[i].y());
    }
    std::vector<double> new_fit_params;
    math::FitLaneLinePoint(new_fit_pts, &new_fit_params);
    const auto& c0 = new_fit_params[0];
    const auto& c1 = new_fit_params[1];
    const auto& c2 = new_fit_params[2];
    const auto& c3 = new_fit_params[3];
    std::vector<Eigen::Vector3d> new_road_points;
    bool fit_error = false;
    for (const auto& pt : occ_road_ptr->road_points) {
      const auto& x = pt.x();
      double new_y = c3 * x * x * x + c2 * x * x + c1 * x + c0;
      HLOG_DEBUG << "new_fit_occ_pts x: " << pt.x() << ", y: " << new_y;
      if (std::abs(new_y - pt.y()) > 5.0) {
        fit_error = true;
        break;
      }
      new_road_points.emplace_back(pt.x(), new_y, 0.0);
    }
    if (!fit_error) {
      occ_road_ptr->road_points = new_road_points;
    }
  }
  HLOG_DEBUG << "first_point_index: " << *first_point_index;
  return get_first_pt;
}

void GeoOptimization::CalDistNearOcc(
    const std::vector<em::OccRoad::Ptr>& vec_occs, bool left,
    const em::OccRoad::Ptr& occ_road_ptr, int* index, double* distance) {
  int occ_id = left ? occ_road_ptr->left_occ_id : occ_road_ptr->right_occ_id;
  int find_index = -1;
  double find_dist = std::numeric_limits<double>::max();
  while (occ_id != -1) {
    bool find_flag = false;
    // 这里可以不用vec_occs遍历，传进来便于后续做复杂逻辑
    for (const auto& occ : vec_occs) {
      if (occ->track_id == occ_id) {
        const auto& reft_pt = occ->road_points.at(occ->guide_index);
        for (int i = 0; i < static_cast<int>(occ_road_ptr->road_points.size());
             ++i) {
          const auto& pt = occ_road_ptr->road_points[i];
          if ((reft_pt - pt).norm() < find_dist) {
            find_flag = true;
            find_dist = (reft_pt - pt).norm();
            find_index = i;
          }
        }
      }
    }
    if (find_flag) {
      break;
    }
    auto iter = elem_map_->occ_roads.find(occ_id);
    if (iter != elem_map_->occ_roads.end()) {
      occ_id = left ? iter->second->left_occ_id : iter->second->right_occ_id;
    } else {
      break;
    }
  }
  // 至少保留两个点
  if (find_index > static_cast<int>(occ_road_ptr->road_points.size() - 2)) {
    find_index = static_cast<int>(occ_road_ptr->road_points.size() - 2);
  }
  *index = find_index;
  *distance = find_dist;
}

bool GeoOptimization::GetFirstNearIndex(
    const std::vector<em::OccRoad::Ptr>& vec_occs,
    const em::OccRoad::Ptr& occ_road_ptr, int* first_point_index) {
  if (occ_road_ptr->road_points.size() < 6) {
    return false;
  }
  int left_index = -1;
  double left_dist = 0.0;
  CalDistNearOcc(vec_occs, true, occ_road_ptr, &left_index, &left_dist);
  int right_index = -1;
  double right_dist = 0.0;
  CalDistNearOcc(vec_occs, false, occ_road_ptr, &right_index, &right_dist);
  if (left_index == -1 && right_index == -1) {
    return false;
  }
  if (left_index != -1 && right_index == -1) {
    *first_point_index = left_index;
  } else if (left_index == -1 && right_index != -1) {
    *first_point_index = right_index;
  } else {
    *first_point_index = left_dist < right_dist ? left_index : right_index;
  }
  return true;
}

bool GeoOptimization::CaluclateSlope(const Eigen::Vector3d& point1,
                                     const Eigen::Vector3d& point2,
                                     double* slope_value) {
  if (point1.y() == point2.y()) {
    return false;
  }
  *slope_value = (point2.x() - point1.x()) / (point2.y() - point1.y());
  return true;
}

bool GeoOptimization::LineCublicSampling(
    const hozon::mapping::Occ& occ, std::vector<float>* curve_params,
    std::vector<Eigen::Vector3d>* new_line_pts) {
  // 重采样
  if (occ.has_lane_param() && occ.lane_param().cubic_curve_set_size() == 1) {
    auto start_x = occ.lane_param().cubic_curve_set(0).start_point_x();
    auto end_x = occ.lane_param().cubic_curve_set(0).end_point_x();
    auto c0 = occ.lane_param().cubic_curve_set(0).c0();
    auto c1 = occ.lane_param().cubic_curve_set(0).c1();
    auto c2 = occ.lane_param().cubic_curve_set(0).c2();
    auto c3 = occ.lane_param().cubic_curve_set(0).c3();
    *curve_params = {c0, c1, c2, c3};
    for (float x = start_x; x < end_x;) {
      float y = c3 * x * x * x + c2 * x * x + c1 * x + c0;
      Eigen::Vector3d pt(x, y, 0);
      new_line_pts->emplace_back(pt);
      x += 1.0;
    }
    float end_y =
        c3 * end_x * end_x * end_x + c2 * end_x * end_x + c1 * end_x + c0;
    return true;
  }
  return false;
}

double GeoOptimization::OccLineFitError(const hozon::mapping::Occ& occ,
                                        const std::vector<float>& curve_param) {
  double avg_error = std::numeric_limits<double>::max();
  if (curve_param.size() != 4) {
    return avg_error;
  }
  avg_error = 0.0;
  double c0 = curve_param[0];
  double c1 = curve_param[1];
  double c2 = curve_param[2];
  double c3 = curve_param[3];
  for (const auto& pt : occ.points()) {
    double x = pt.x();
    double y = pt.y();
    double error = std::abs(c3 * x * x * x + c2 * x * x + c1 * x + c0 - y);
    avg_error += error;
  }
  return avg_error / (occ.points().size() + 1);
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
  // 将路沿构建成车道线
  MakeRoadEdgeToLaneLine();
  // 对自车相邻的两个车道线进行补齐
  // AlignmentVecLane();
  // 处理超宽车道
  HandleExtraWideLane();
  // 处理单边线
  // HandleSingleSideLine();
}

void GeoOptimization::CreateLocalLineTable() {
  // 存储local map每根车道线的lane_pos和几何
  local_line_table_.clear();
  for (const auto& local_line : local_map_use_->lane_lines()) {
    if (local_line.points().empty()) {
      continue;
    }
    local_line_info line_info;
    std::vector<cv::Point2f> kdtree_points;
    Line_kd line_kd;
    auto lane_pos = local_line.lanepos();
    auto line_track_id = local_line.track_id();
    std::vector<Eigen::Vector3d> local_pts;
    int pt_interval = 1;
    for (int i = 0; i < local_line.points_size(); i += pt_interval) {
      const auto& pt = local_line.points(i);
      if (std::isnan(pt.x()) || std::isnan(pt.y())) {
        continue;
      }
      Eigen::Vector3d point(pt.x(), pt.y(), pt.z());
      local_pts.emplace_back(point);
      kdtree_points.emplace_back(static_cast<float>(pt.x()),
                                 static_cast<float>(pt.y()));
    }
    cv::flann::KDTreeIndexParams index_param(1);
    std::shared_ptr<cv::flann::Index> kdtree_ptr =
        std::make_shared<cv::flann::Index>(cv::Mat(kdtree_points).reshape(1),
                                           index_param);
    line_kd.line_kdtree = kdtree_ptr;
    line_kd.line = std::make_shared<hozon::mapping::LaneLine>(local_line);
    line_info.lane_pos = lane_pos;
    line_info.local_line_pts = local_pts;
    line_info.line_track_id = line_track_id;
    line_info.kd_line = line_kd;
    local_line_table_.insert_or_assign(line_track_id, line_info);
  }
}

void GeoOptimization::MakeRoadEdgeToLaneLine() {
  // 将满足条件的路沿更新为车道线
  /*
    1.找到lane_pos为-1或1的路沿，并计算离路沿最近的车道线
    2.判断路沿和车道线距离是否满足条件
    3.判断车道线的属性是否是虚线
  */
  if (!local_map_use_ || local_map_use_->lane_lines_size() == 0) {
    return;
  }
  // 存储路沿(自车左右)
  for (const auto& road : local_map_->road_edges()) {
    std::vector<Eigen::Vector3d> road_pts;
    for (const auto& pt : road.points()) {
      if (std::isnan(pt.x()) || std::isnan(pt.y())) {
        continue;
      }
      Eigen::Vector3d point(pt.x(), pt.y(), pt.z());
      road_pts.emplace_back(point);
    }
    if (road_pts.empty()) {
      continue;
    }
    CompareRoadAndLines(road_pts, road.track_id());
  }
}

void GeoOptimization::CompareRoadAndLines(
    const std::vector<Eigen::Vector3d>& road_pts, const int& road_id) {
  // 比较路沿和车道线
  hozon::mapping::LaneLine target_line;
  double min_dis = std::numeric_limits<double>::max();
  for (const auto& line : local_map_use_->lane_lines()) {
    std::vector<Eigen::Vector3d> line_pts;
    for (const auto& pt : line.points()) {
      Eigen::Vector3d point(pt.x(), pt.y(), pt.z());
      line_pts.emplace_back(point);
    }
    double road_avg_dis = 0.0;
    if (!ComputerLineDis(road_pts, line_pts, &road_avg_dis)) {
      continue;
    }
    if (road_avg_dis < min_dis) {
      min_dis = road_avg_dis;
      target_line = line;
    }
  }

  if (target_line.lanetype() ==
      hozon::mapping::LaneType_INTERSECTION_VIRTUAL_MARKING) {
    return;
  }
  auto target_line_heading = ComputeLaneLineHeading(target_line);
  if (min_dis < 1.0) {
    // 路沿离车道线的距离小于1米,对离路沿最近的车道线进行增补
    auto tar_line_pt_size = target_line.points_size();
    if (road_pts.back().x() > target_line.points(tar_line_pt_size - 1).x()) {
      // 对车道线往前补,直至和路沿远端对齐
      for (auto& line : *local_map_use_->mutable_lane_lines()) {
        if (line.track_id() != target_line.track_id()) {
          continue;
        }
        Eigen::Vector3d fit_vec;
        for (const auto& road_pt : road_pts) {
          if (road_pt.x() > target_line.points(tar_line_pt_size - 1).x()) {
            Eigen::Vector3d target_first_pt(
                target_line.points(tar_line_pt_size - 1).x(),
                target_line.points(tar_line_pt_size - 1).y(),
                target_line.points(tar_line_pt_size - 1).z());
            fit_vec = target_first_pt - road_pt;
            break;
          }
        }
        std::vector<Eigen::Vector3d> new_road_pts;
        for (const auto& road_pt : road_pts) {
          if (road_pt.x() < target_line.points(tar_line_pt_size - 1).x()) {
            continue;
          }
          double fit_dis = fit_vec.norm();
          auto fit_vec_normlized = fit_vec.normalized();
          auto fit_road_pt = road_pt + fit_vec_normlized * fit_dis;
          new_road_pts.emplace_back(fit_road_pt);
        }
        if (new_road_pts.empty()) {
          break;
        }
        // 判断虚拟的点是否处于两根线中间
        if (IsBetweenLinesMid(new_road_pts, target_line, 0)) {
          break;
        }
        for (int i = 0; i < static_cast<int>(new_road_pts.size()) - 1; i++) {
          auto road_heading = new_road_pts[i + 1] - new_road_pts[i];
          auto angle =
              ComputeAngleBetweenVectors(target_line_heading, road_heading);
          if (angle > 15) {
            continue;
          }
          auto* new_pt = line.add_points();
          new_pt->set_x(new_road_pts[i].x());
          new_pt->set_y(new_road_pts[i].y());
          new_pt->set_z(new_road_pts[i].z());
        }
        break;
      }
    }
    if (road_pts.front().x() < target_line.points(0).x()) {
      // 对车道线后端进行补齐,补至跟路沿近端对其
      for (auto& line : *local_map_use_->mutable_lane_lines()) {
        if (line.track_id() != target_line.track_id()) {
          continue;
        }
        std::vector<hozon::common::Point3D> history_pts;
        for (const auto& pt : line.points()) {
          history_pts.emplace_back(pt);
        }
        Eigen::Vector3d fit_vec;
        for (const auto& road_pt : road_pts) {
          if (road_pt.x() < target_line.points(0).x()) {
            continue;
          }
          Eigen::Vector3d target_first_pt(target_line.points(0).x(),
                                          target_line.points(0).y(),
                                          target_line.points(0).z());
          fit_vec = target_first_pt - road_pt;
          break;
        }
        std::vector<Eigen::Vector3d> new_road_pts;
        for (const auto& road_pt : road_pts) {
          if (road_pt.x() > target_line.points(0).x()) {
            continue;
          }
          double fit_dis = fit_vec.norm();
          auto fit_vec_normlized = fit_vec.normalized();
          auto fit_road_pt = road_pt + fit_vec_normlized * fit_dis;
          new_road_pts.emplace_back(fit_road_pt);
        }
        // 判断虚拟的点是否处于两根线中间
        if (IsBetweenLinesMid(new_road_pts, target_line, 1)) {
          break;
        }
        line.mutable_points()->Clear();
        for (int i = 0; i < static_cast<int>(new_road_pts.size()) - 1; i++) {
          auto road_heading = new_road_pts[i + 1] - new_road_pts[i];
          auto angle =
              ComputeAngleBetweenVectors(target_line_heading, road_heading);
          if (angle > 15) {
            continue;
          }
          auto* new_pt = line.add_points();
          new_pt->set_x(new_road_pts[i].x());
          new_pt->set_y(new_road_pts[i].y());
          new_pt->set_z(new_road_pts[i].z());
        }
        for (const auto& tar_line_pt : history_pts) {
          auto* new_pt = line.add_points();
          new_pt->set_x(tar_line_pt.x());
          new_pt->set_y(tar_line_pt.y());
          new_pt->set_z(tar_line_pt.z());
        }
        break;
      }
    }
  }
  if (((target_line.lanetype() == em::LineType::LaneType_DASHED ||
        target_line.lanetype() == em::LineType::LaneType_FISHBONE_DASHED ||
        target_line.lanetype() == em::LineType::LaneType_DOUBLE_DASHED ||
        target_line.lanetype() == em::LineType::LaneType_SHORT_DASHED) &&
       min_dis > 2.0) ||
      ((target_line.lanetype() == em::LineType::LaneType_SOLID ||
        target_line.lanetype() == em::LineType::LaneType_DOUBLE_SOLID ||
        target_line.lanetype() == em::LineType::LaneType_FISHBONE_SOLID) &&
       min_dis > 3.0)) {
    auto* new_line = local_map_use_->add_lane_lines();
    // 计算target_line的平均heading, 通过heading对road_pts进行裁剪
    for (int i = 0; i < static_cast<int>(road_pts.size()) - 1; i++) {
      auto road_heading = road_pts[i + 1] - road_pts[i];
      auto angle =
          ComputeAngleBetweenVectors(target_line_heading, road_heading);
      if (angle > 15) {
        continue;
      }
      auto* new_pt = new_line->add_points();
      new_pt->set_x(road_pts[i].x());
      new_pt->set_y(road_pts[i].y());
      new_pt->set_z(road_pts[i].z());
    }
    auto new_track_id = road_id + 1000;
    new_line->set_lanepos(
        static_cast<hozon::mapping::LanePositionType>(new_track_id));
    new_line->set_track_id(new_track_id);
    new_line->set_color(static_cast<hozon::mapping::Color>(1));
    new_line->set_lanetype(static_cast<hozon::mapping::LaneType>(1));

    // 更新哈希表
    local_line_info local_line;
    local_line.line_track_id = new_track_id;
    local_line.local_line_pts = road_pts;
    local_line.lane_pos =
        static_cast<hozon::mapping::LanePositionType>(new_track_id);
    local_line_table_.insert_or_assign(new_track_id, local_line);

    // 更新all_lines
    std::vector<cv::Point2f> kdtree_points;
    Line_kd line_kd;
    hozon::mapping::LaneLine line;
    line.set_track_id(static_cast<int>(new_track_id));
    for (const auto& point : road_pts) {
      kdtree_points.emplace_back(static_cast<float>(point.x()),
                                 static_cast<float>(point.y()));
      auto* new_pt = line.add_points();
      new_pt->set_x(point.x());
      new_pt->set_y(point.y());
      new_pt->set_z(point.z());
    }
    cv::flann::KDTreeIndexParams index_param(1);
    std::shared_ptr<cv::flann::Index> kdtree_ptr =
        std::make_shared<cv::flann::Index>(cv::Mat(kdtree_points).reshape(1),
                                           index_param);
    line_kd.line_kdtree = kdtree_ptr;
    line_kd.line = std::make_shared<hozon::mapping::LaneLine>(line);
    all_lines_[static_cast<int>(new_track_id)].emplace_back(line_kd);
  }
}

bool GeoOptimization::IsBetweenLinesMid(
    const std::vector<Eigen::Vector3d>& new_road_pts,
    const hozon::mapping::LaneLine& target_line, const int& direction) {
  int left_count = 0;
  int right_count = 0;
  for (const auto& line : local_map_use_->lane_lines()) {
    if (line.points().empty() || line.track_id() == target_line.track_id()) {
      continue;
    }
    if (direction == 0) {
      // 只判断前向
      for (const auto& line_pt : line.points()) {
        if (line_pt.x() < new_road_pts.front().x()) {
          continue;
        }
        // 只判断第一个点
        if (line_pt.y() >= new_road_pts.front().y()) {
          left_count += 1;
        } else {
          right_count += 1;
        }
        break;
      }
      // 判断最后一个点
      if (line.points(line.points_size() - 1).y() >= new_road_pts.back().y()) {
        left_count += 1;
      } else {
        right_count += 1;
      }
    }
    if (direction == 1) {
      // 只判断后向
      for (const auto& line_pt : line.points()) {
        if (line_pt.x() > new_road_pts.back().x()) {
          continue;
        }
        // 只判断最后点
        if (line_pt.y() >= new_road_pts.back().y()) {
          left_count += 1;
        } else {
          right_count += 1;
        }
        break;
      }
      // 判断第一个点
      if (line.points(0).y() >= new_road_pts.front().y()) {
        left_count += 1;
      } else {
        right_count += 1;
      }
    }
  }
  HLOG_DEBUG << "left_count and right_count: " << left_count << ", "
             << right_count;
  if (left_count != 0 && right_count != 0) {
    return true;
  }
  return false;
}

double GeoOptimization::ComputeAngleBetweenVectors(const Eigen::Vector3d& v1,
                                                   const Eigen::Vector3d& v2) {
  double dot_product = v1.dot(v2);
  double v1_norm = v1.norm();
  double v2_norm = v2.norm();

  if (v1_norm == 0.0 || v2_norm == 0.0) {
    HLOG_WARN << "Error: Zero vector encountered.";
    return 0.0;
  }
  double cos_theta = dot_product / (v1_norm * v2_norm);
  cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
  double angle_rad = std::acos(cos_theta);
  double angle_deg = angle_rad * 180.0 / M_PI;
  return angle_deg;
}

void GeoOptimization::ConstructLaneLine(
    const std::vector<std::vector<Eigen::Vector3d>>& new_lines) {
  for (const auto& line : new_lines) {
    if (line.empty()) {
      continue;
    }
    auto* new_line = local_map_use_->add_lane_lines();
    for (const auto& pt : line) {
      if (std::isnan(pt.x()) || std::isnan(pt.y())) {
        continue;
      }
      auto* new_pt = new_line->add_points();
      new_pt->set_x(pt.x());
      new_pt->set_y(pt.y());
      new_pt->set_z(pt.z());
    }

    new_line->set_lanepos(
        static_cast<hozon::mapping::LanePositionType>(extra_val_));
    new_line->set_track_id(extra_val_);
    new_line->set_color(static_cast<hozon::mapping::Color>(1));
    // new_line->set_allocated_lane_param(static_cast<hozon::mapping::LaneCubicSpline>(500));
    // new_line->set_confidence(static_cast<hozon::mapping::LanePositionType>(500));
    new_line->set_lanetype(static_cast<hozon::mapping::LaneType>(2));

    // 更新哈希表
    local_line_info local_line;
    local_line.line_track_id = extra_val_;
    local_line.local_line_pts = line;
    local_line.lane_pos =
        static_cast<hozon::mapping::LanePositionType>(extra_val_);
    local_line_table_.insert_or_assign(extra_val_, local_line);
    extra_val_++;

    // 更新all_lines
    std::vector<cv::Point2f> kdtree_points;
    Line_kd line_kd;
    hozon::mapping::LaneLine kd_line;
    kd_line.set_track_id(static_cast<int>(static_cast<int>(extra_val_)));
    for (const auto& point : line) {
      if (std::isnan(point.x()) || std::isnan(point.y())) {
        continue;
      }
      kdtree_points.emplace_back(static_cast<float>(point.x()),
                                 static_cast<float>(point.y()));
      auto* new_pt = kd_line.add_points();
      new_pt->set_x(point.x());
      new_pt->set_y(point.y());
      new_pt->set_z(point.z());
    }
    cv::flann::KDTreeIndexParams index_param(1);
    std::shared_ptr<cv::flann::Index> kdtree_ptr =
        std::make_shared<cv::flann::Index>(cv::Mat(kdtree_points).reshape(1),
                                           index_param);
    line_kd.line_kdtree = kdtree_ptr;
    line_kd.line = std::make_shared<hozon::mapping::LaneLine>(kd_line);
    all_lines_[static_cast<int>(extra_val_)].emplace_back(line_kd);
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
  if (!local_map_use_ || local_map_use_->lane_lines_size() == 0) {
    return;
  }
  std::vector<std::pair<int, int>> ego_ids;
  std::vector<int> left_track_ids;
  std::vector<int> right_track_ids;
  std::vector<int> other_ids;
  for (const auto& local_line : local_map_use_->lane_lines()) {
    if (local_line_table_.find(local_line.track_id()) ==
        local_line_table_.end()) {
      continue;
    }
    // 确定主车道左右两根线
    if (local_line.lanepos() == -1) {
      left_track_ids.emplace_back(local_line.track_id());
    }
    if (local_line.lanepos() == 1) {
      right_track_ids.emplace_back(local_line.track_id());
    }
    if (local_line.lanepos() == 99) {
      other_ids.emplace_back(local_line.track_id());
    }
  }
  // 不存在ego lane
  if (left_track_ids.empty() || right_track_ids.empty()) {
    return;
  }
  for (const auto& left_id : left_track_ids) {
    for (const auto& right_id : right_track_ids) {
      bool flag = false;
      std::pair<int, int> ego_other_ids;
      VerifyEgoLane(left_id, right_id, other_ids, &flag, &ego_other_ids);
      int ego_left_id = left_id;
      int ego_right_id = right_id;
      if (!flag) {
        ego_left_id = ego_other_ids.first;
        ego_right_id = ego_other_ids.second;
      }
      auto left_pts = local_line_table_.at(ego_left_id).local_line_pts;
      auto right_pts = local_line_table_.at(ego_right_id).local_line_pts;
      double line_wids = 0.0;
      if (!ComputerLineDis(left_pts, right_pts, &line_wids)) {
        continue;
      }
      ego_ids.emplace_back(ego_left_id, ego_right_id);
    }
  }
  if (ego_ids.empty()) {
    HLOG_WARN << "ego_ids is empty!";
    return;
  }
  for (const auto& ego_id : ego_ids) {
    auto left_line = local_line_table_.at(ego_id.first).local_line_pts;
    auto right_line = local_line_table_.at(ego_id.second).local_line_pts;
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
      for (int i = 0; i < static_cast<int>(right_line.size()) - 1; ++i) {
        const auto& A = right_line[i];
        const auto& B = right_line[i + 1];
        const auto& AB = B - A;
        const auto& AP = P - A;
        if (flag && (A - B).norm() > 0.1 && dis > 0) {
          // 对点向左预测
          Eigen::Vector3d per_direction(-AB.normalized().y(),
                                        AB.normalized().x(), 0);
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
                              (left_line[1] - left_line[0]).normalized().x(),
                              0);
      for (int i = 0; i < static_cast<int>(left_line.size()) - 1; ++i) {
        const auto& A = left_line[i];
        const auto& B = left_line[i + 1];
        const auto& AB = B - A;
        const auto& AP = P - A;
        if (flag && (A - B).norm() > 0.1 && dis > 0) {
          // 对点向左预测
          Eigen::Vector3d per_direction(-AB.normalized().y(),
                                        AB.normalized().x(), 0);
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
      if (local_line.track_id() == ego_id.first && !new_left_pts.empty()) {
        for (const auto& pt : new_left_pts) {
          auto* new_pt = local_line.add_points();
          new_pt->set_x(pt.x());
          new_pt->set_y(pt.y());
          new_pt->set_z(pt.z());
          local_line_table_.at(ego_id.first).local_line_pts.emplace_back(pt);
        }
      }
      if (local_line.track_id() == ego_id.second && !new_right_pts.empty()) {
        for (const auto& pt : new_right_pts) {
          auto* new_pt = local_line.add_points();
          new_pt->set_x(pt.x());
          new_pt->set_y(pt.y());
          new_pt->set_z(pt.z());
          local_line_table_.at(ego_id.second).local_line_pts.emplace_back(pt);
        }
      }
    }
  }
}

bool GeoOptimization::ComputerLineDisNoOverlap(
    const std::vector<Eigen::Vector3d>& edge_line_pts,
    const std::vector<Eigen::Vector3d>& line_pts, double* avg_width) {
  Eigen::Vector2d edge_point, line_pt1, line_pt2;
  if (edge_line_pts.front().x() >= line_pts.back().x()) {
    edge_point = {edge_line_pts.front().x(), edge_line_pts.front().y()};
    line_pt1 = {line_pts[line_pts.size() - 2].x(),
                line_pts[line_pts.size() - 2].y()};
    line_pt2 = {line_pts[line_pts.size() - 1].x(),
                line_pts[line_pts.size() - 1].y()};
  } else if (edge_line_pts.back().x() <= line_pts.front().x()) {
    edge_point = {edge_line_pts.back().x(), edge_line_pts.back().y()};
    line_pt1 = {line_pts[0].x(), line_pts[0].y()};
    line_pt2 = {line_pts[1].x(), line_pts[1].y()};
  }
  Eigen::Vector2d BC = line_pt2 - line_pt1;
  Eigen::Vector2d BA = edge_point - line_pt1;
  if (abs(BC.norm()) < 0.0001) {
    *avg_width = abs(BA.y());
    return true;
  }
  double dist_proj = BA.dot(BC) / BC.norm();
  // 计算点到直线的距离
  *avg_width = sqrt(pow(BA.norm(), 2) - pow(dist_proj, 2));
  return true;
}

bool GeoOptimization::ComputerLineDis(
    const std::vector<Eigen::Vector3d>& line_pts,
    const std::vector<Eigen::Vector3d>& right_line_pts, double* avg_width,
    int pts_interval) {
  return ComputerLineDis(line_pts, right_line_pts, nullptr, avg_width,
                         pts_interval);
}
bool GeoOptimization::ComputerLineDis(
    const std::vector<Eigen::Vector3d>& line_pts,
    const std::vector<Eigen::Vector3d>& right_line_pts,
    std::vector<double>* line_dis, double* avg_width, int pts_interval) {
  // 计算线与线之间距离
  if (line_pts.empty() || right_line_pts.empty()) {
    HLOG_ERROR << "line_pts.empty() || right_line_pts.empty()";
    HLOG_ERROR << "line_pts.empty() || right_line_pts.empty()";
    return false;
  }
  int count_num = 0;
  bool flag = false;
  double original_val = *avg_width;
  *avg_width = 0.0;
  for (int index = 0; index < static_cast<int>(line_pts.size());
       index += pts_interval) {
    const auto& P = line_pts[index];
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
      auto dis = (P - C).norm();
      *avg_width += dis;
      if (line_dis != nullptr) {
        line_dis->emplace_back(dis);
      }
      count_num++;
      break;
    }
  }
  if (count_num > 0) {
    flag = true;
    *avg_width /= count_num;
  } else {
    flag = false;
    *avg_width = original_val;
  }
  return flag;
}

void GeoOptimization::HandleExtraWideLane() {
  // 处理超宽车道（漏检）场景
  if (!local_map_use_ || local_map_use_->lane_lines_size() == 0) {
    return;
  }
  std::vector<std::pair<int, int>> extra_ids;
  std::vector<int> left_track_ids;
  std::vector<int> right_track_ids;
  std::vector<int> other_ids;
  for (const auto& local_line : local_map_use_->lane_lines()) {
    if (local_line_table_.find(local_line.track_id()) ==
        local_line_table_.end()) {
      continue;
    }
    // 确定主车道左右两根线
    if (local_line.lanepos() == -1) {
      left_track_ids.emplace_back(local_line.track_id());
    }
    if (local_line.lanepos() == 1) {
      right_track_ids.emplace_back(local_line.track_id());
    }
    if (local_line.lanepos() == 99 && !local_line.points().empty() &&
        local_line.points(0).x() < 40.0) {
      other_ids.emplace_back(local_line.track_id());
    }
  }
  // 确定是否存在超宽车道
  if (left_track_ids.empty() || right_track_ids.empty()) {
    return;
  }
  for (const auto& left_id : left_track_ids) {
    for (const auto& right_id : right_track_ids) {
      // 校验自车左右所在的车道线(两根线的中间可能会夹有lane_pos为99的线)
      if (!CheckEgoLane(left_id, right_id, other_ids)) {
        continue;
      }
      int ego_left_id = left_id;
      int ego_right_id = right_id;
      auto left_pts = local_line_table_.at(ego_left_id).local_line_pts;
      auto right_pts = local_line_table_.at(ego_right_id).local_line_pts;
      std::vector<double> line_wids;
      double avg_width = 0.0;
      if (!ComputerLineDis(left_pts, right_pts, &line_wids, &avg_width)) {
        continue;
      }
      for (const auto& wid : line_wids) {
        if (wid >= 5.5) {
          HLOG_WARN << "have extra wide lanes!";
          extra_ids.emplace_back(ego_left_id, ego_right_id);
          break;
        }
      }
    }
  }
  if (extra_ids.empty()) {
    // 重置
    base_line_.base_line_flag = 0;
    base_line_.base_width = -1;
    base_line_.base_line_left_id = -1;
    base_line_.base_line_right_id = -1;
  }
  // 针对超宽（漏检）车道进行拟合漏检车道线
  for (const auto& ex : extra_ids) {
    FitMissedLaneLine(ex);
  }
}

void GeoOptimization::VerifyEgoLane(const int& left_id, const int& right_id,
                                    const std::vector<int>& other_ids,
                                    bool* flag,
                                    std::pair<int, int>* ego_other_ids) {
  // 校验ego_lane是否满足条件
  /*
    方案:确定每根线离车最近的点,根据最近点的y值判断主车道
  */
  if (other_ids.empty()) {
    *flag = true;
    return;
  }
  auto left_kd_line = local_line_table_.at(left_id).kd_line;
  auto right_kd_line = local_line_table_.at(right_id).kd_line;
  auto left_min_pt = FindMinDisLinePoint(left_kd_line);
  auto right_min_pt = FindMinDisLinePoint(right_kd_line);
  if (left_min_pt.z() == -10.0 || right_min_pt.z() == -10.0) {
    *flag = false;
    return;
  }
  HLOG_DEBUG << "left_min_pt:" << left_min_pt.x() << "," << left_min_pt.y();
  HLOG_DEBUG << "right_min_pt:" << right_min_pt.x() << "," << right_min_pt.y();
  bool is_mid = false;
  double greater_than_zero_dis = left_min_pt.y();
  double less_than_zero_dis = right_min_pt.y();
  int ego_left_id = left_id;
  int ego_right_id = right_id;
  for (const auto& other_id : other_ids) {
    HLOG_DEBUG << "other_id:" << other_id;
    auto other_kd_line = local_line_table_.at(other_id).kd_line;
    auto other_min_pt = FindMinDisLinePoint(other_kd_line);
    HLOG_DEBUG << "other_min_pt:" << other_min_pt.x() << ","
               << other_min_pt.y();
    if (other_min_pt.z() < -10.0) {
      continue;
    }
    if (other_min_pt.y() > 0 && other_min_pt.y() < greater_than_zero_dis &&
        fabs(other_min_pt.y() - greater_than_zero_dis) > 2.0) {
      ego_left_id = other_id;
      greater_than_zero_dis = other_min_pt.y();
    }
    if (other_min_pt.y() < 0 && other_min_pt.y() > less_than_zero_dis &&
        fabs(other_min_pt.y() - less_than_zero_dis) > 2.0) {
      ego_right_id = other_id;
      less_than_zero_dis = other_min_pt.y();
    }
    if (left_min_pt.y() >= other_min_pt.y() &&
        other_min_pt.y() >= right_min_pt.y()) {
      is_mid = true;
      break;
    }
  }
  if (is_mid) {
    // 此时的ego_lane
    ego_other_ids->first = ego_left_id;
    ego_other_ids->second = ego_right_id;
    *flag = false;
    return;
  } else {
    *flag = true;
    return;
  }
}

bool GeoOptimization::CheckEgoLane(const int& left_id, const int& right_id,
                                   const std::vector<int>& other_ids) {
  if (local_line_table_.find(left_id) == local_line_table_.end() ||
      local_line_table_.find(right_id) == local_line_table_.end()) {
    return false;
  }
  if (other_ids.empty()) {
    return true;
  }
  auto left_kd_line = local_line_table_.at(left_id).kd_line;
  auto right_kd_line = local_line_table_.at(right_id).kd_line;
  for (const auto& other_id : other_ids) {
    HLOG_DEBUG << "other_id:" << other_id;
    auto other_kd_line = local_line_table_.at(other_id).kd_line;
    auto other_min_pt = FindMinDisLinePoint(other_kd_line);
    HLOG_DEBUG << "other_min_pt:" << other_min_pt.x() << ","
               << other_min_pt.y();
    if (other_min_pt.z() < -10.0) {
      continue;
    }
    std::vector<float> query_point = {other_min_pt.x(), other_min_pt.y()};
    auto left_min_pt = FindMinDisLinePoint(left_kd_line, query_point);
    auto right_min_pt = FindMinDisLinePoint(right_kd_line, query_point);

    if (other_min_pt.y() > 0 && other_min_pt.y() < left_min_pt.y()) {
      return false;
    }
    if (other_min_pt.y() < 0 && other_min_pt.y() > right_min_pt.y()) {
      return false;
    }
  }
  return true;
}

Eigen::Vector3f GeoOptimization::FindMinDisLinePoint(
    const Line_kd& kd_line, std::vector<float> query_point) {
  int dim = 1;
  std::vector<int> nearest_index(dim);
  std::vector<float> nearest_dis(dim);
  kd_line.line_kdtree->knnSearch(query_point, nearest_index, nearest_dis, dim,
                                 cv::flann::SearchParams(-1));
  if (nearest_index.size() < 1) {
    Eigen::Vector3f pt(0.0, 0.0, -10.0);
    return pt;
  }
  Eigen::Vector3f min_pt(kd_line.line->points(nearest_index[0]).x(),
                         kd_line.line->points(nearest_index[0]).y(), 0.0);
  return min_pt;
}

void GeoOptimization::FitMissedLaneLine(const std::pair<int, int>& ex) {
  // 拟合漏检车道线
  if (local_line_table_.find(ex.first) == local_line_table_.end() ||
      local_line_table_.find(ex.second) == local_line_table_.end()) {
    HLOG_ERROR << "ex not in local_line_table_";
    return;
  }
  auto ex_pts = local_line_table_.at(ex.first).local_line_pts;
  auto right_ex_pts = local_line_table_.at(ex.second).local_line_pts;
  // 判断主车道的左右id是否突变
  if (base_line_.base_line_left_id != ex.first ||
      base_line_.base_line_right_id != ex.second) {
    base_line_.base_line_flag = 0;
    base_line_.base_width = -1;
    // base_line_.base_line_left_id = -1;
    // base_line_.base_line_right_id = -1;
    base_line_.base_line_left_id = ex.first;
    base_line_.base_line_right_id = ex.second;
  }
  // 根据车离两侧线的距离以及两根线的heading或平均曲率变化率来确认base线
  auto left_dis = ComputeVecToLaneDis(ex_pts);
  auto right_dis = ComputeVecToLaneDis(right_ex_pts);
  // 计算两根线的平均heading
  auto left_heading = ComputeLineHeading(ex_pts);
  auto right_heading = ComputeLineHeading(right_ex_pts);
  // 计算两根线的平均曲率变化率
  auto left_curve = ComputeCurvature(ex_pts);
  auto right_curve = ComputeCurvature(right_ex_pts);
  // if (left_heading < 0 || right_heading < 0) {
  //   return;
  // }
  if (left_curve < 0 || right_curve < 0) {
    return;
  }
  std::vector<std::vector<Eigen::Vector3d>> new_lines;
  std::vector<Eigen::Vector3d> base_line;
  std::vector<double> base_width;
  if (base_line_.base_line_flag == 0) {
    if ((std::abs(left_dis - right_dis) > 0.7 && left_dis < right_dis) ||
        (std::abs(left_dis - right_dis) < 0.7 && left_curve < right_curve)) {
      // vehicle left line is baseline
      ObtainBaseLine(&base_line, &base_width, ex_pts, right_ex_pts);
      double lane_width = 3.75;
      if (!base_width.empty()) {
        lane_width = base_width.front();
        // double lane_width =
        //     std::reduce(base_width.begin(), base_width.end()) /
        //     base_width.size();
      }
      if (lane_width > 5.5) {
        lane_width = 3.75;
      }
      int lane_num = std::ceil(left_dis / lane_width);
      FitSingleSideLine(&base_line, &new_lines, lane_num, true, 0, lane_width);
      base_line_.base_line_flag = 1;
      base_line_.base_width = lane_width;
    } else if ((std::abs(left_dis - right_dis) > 0.7 && right_dis < left_dis) ||
               (std::abs(left_dis - right_dis) < 0.7 &&
                right_curve < left_curve)) {
      // vecicle right line is baseline
      ObtainBaseLine(&base_line, &base_width, right_ex_pts, ex_pts);
      double lane_width = 3.75;
      if (!base_width.empty()) {
        lane_width = base_width.front();
      }
      if (lane_width > 5.5) {
        lane_width = 3.75;
      }
      int lane_num = std::ceil(right_dis / lane_width);
      FitSingleSideLine(&base_line, &new_lines, lane_num, false, 0, lane_width);
      base_line_.base_line_flag = 2;
      base_line_.base_width = lane_width;
    }
  } else if (base_line_.base_line_flag == 1) {
    // vehicle left line is baseline
    ObtainBaseLine(&base_line, &base_width, ex_pts, right_ex_pts);
    double lane_width = 3.75;
    // if (!base_width.empty()) {
    //   lane_width = base_width.back();
    // }
    lane_width = base_line_.base_width;
    int lane_num = std::ceil(left_dis / lane_width);
    FitSingleSideLine(&base_line, &new_lines, lane_num, true, 0, lane_width);
  } else if (base_line_.base_line_flag == 2) {
    // vecicle right line is baseline
    ObtainBaseLine(&base_line, &base_width, right_ex_pts, ex_pts);
    double lane_width = 3.75;
    // if (!base_width.empty()) {
    //   lane_width = base_width.back();
    // }
    lane_width = base_line_.base_width;
    int lane_num = std::ceil(right_dis / lane_width);
    FitSingleSideLine(&base_line, &new_lines, lane_num, false, 0, lane_width);
  }

  if (new_lines.empty()) {
    HLOG_WARN << "fit new line failed";
    return;
  }
  ConstructLaneLine(new_lines);
}

void GeoOptimization::ObtainBaseLine(
    std::vector<Eigen::Vector3d>* base_line, std::vector<double>* base_width,
    const std::vector<Eigen::Vector3d>& base_pts,
    const std::vector<Eigen::Vector3d>& line_pts) {
  // 获取base_line信息
  int index = 0;
  for (size_t j = 0; j < base_pts.size() - 1; ++j) {
    Eigen::Vector3d P = base_pts[j];
    bool flag = false;
    for (size_t i = 1; i < line_pts.size(); i++) {
      const auto& A = line_pts[i - 1];
      const auto& B = line_pts[i];
      const auto& AB = B - A;
      const auto& AP = P - A;
      double ABLengthSquared = AB.squaredNorm();
      if (std::fabs(ABLengthSquared) < 1e-6) {
        continue;
      }
      double t = AB.dot(AP) / ABLengthSquared;
      if (t < -0.1 || t > 1.1) {
        continue;
      }
      t = std::max(0.0, std::min(t, 1.0));
      auto C = A + t * AB;  // 点到线段的最近点
      // if ((P - C).norm() > 5.5) {
      //   base_line->emplace_back(P);
      // } else {
      //   base_width->emplace_back((P - C).norm());
      // }
      if (base_line_.base_line_flag == 0) {
        if (P.x() > 0.0) {
          index = static_cast<int>(j);
          flag = true;
          base_width->emplace_back((P - C).norm());
        }
      } else {
        if ((P - C).norm() >= base_line_.base_width) {
          index = static_cast<int>(j);
          flag = true;
        }
      }
      break;
    }
    if (flag) {
      break;
    }
  }
  base_line->assign(base_pts.begin() + index, base_pts.end());
}

double GeoOptimization::ComputeLineHeading(
    const std::vector<Eigen::Vector3d>& line_pts) {
  // 计算车前方每根线的heading
  if (line_pts.empty()) {
    return -1;
  }
  std::vector<double> headings;
  for (int i = 0; i < static_cast<int>(line_pts.size()) - 1; ++i) {
    if (line_pts[i].x() < 0) {
      continue;
    }
    auto pt_heading =
        std::abs((std::atan2((line_pts[i + 1].y() - line_pts[i].y()),
                             (line_pts[i + 1].x() - line_pts[i].x()))) *
                 (180 / M_PI));
    headings.emplace_back(pt_heading);
  }
  if (headings.empty()) {
    return -1;
  }
  double mean_heading = std::accumulate(headings.begin(), headings.end(), 0.0) /
                        static_cast<double>(headings.size());
  return mean_heading;
}

double GeoOptimization::ComputeCurvature(
    const std::vector<Eigen::Vector3d>& line_pts) {
  if (line_pts.empty()) {
    return -1;
  }
  std::vector<Vec2d> fit_points;
  std::vector<double> fit_result;
  Vec2d dist_point;
  for (const auto& point : line_pts) {
    if (point.x() < 0) {
      continue;
    }
    Vec2d temp_point;
    temp_point.set_x(dist_point.x() - point.x());
    temp_point.set_y(dist_point.y() - point.y());
    if (temp_point.Length() >= 0.5) {
      fit_points.emplace_back(point.x(), point.y());
    }
    dist_point.set_x(point.x());
    dist_point.set_y(point.y());
  }
  std::vector<double> kappas;
  std::vector<double> dkappas;
  math::FitLaneLinePoint(fit_points, &fit_result);
  math::ComputeDiscretePoints(fit_points, fit_result, &kappas, &dkappas);
  if (kappas.empty()) {
    HLOG_WARN << "kappas size is empty!";
    return -1;
  }
  double avg_curve = std::accumulate(kappas.begin(), kappas.end(), 0.0) /
                     static_cast<double>(kappas.size());
  return avg_curve;
}

Eigen::Vector3d GeoOptimization::ComputeLaneLineHeading(
    const hozon::mapping::LaneLine& lane_line) {
  if (lane_line.points().empty() || lane_line.points_size() < 2) {
    return {};
  }
  Eigen::Vector3d avg_heading;
  Eigen::Vector3d sum_heading(0.0, 0.0, 0.0);
  for (int i = 0; i < static_cast<int>(lane_line.points_size()) - 1; i++) {
    Eigen::Vector3d point1(lane_line.points(i).x(), lane_line.points(i).y(),
                           lane_line.points(i).z());
    Eigen::Vector3d point2(lane_line.points(i + 1).x(),
                           lane_line.points(i + 1).y(),
                           lane_line.points(i + 1).z());
    Eigen::Vector3d dir_heading = point2 - point1;
    sum_heading += dir_heading;
  }
  avg_heading = sum_heading / (lane_line.points_size() - 1);
  return avg_heading;
}

void GeoOptimization::HandleSingleSideLine() {
  // handle single side line
  /*
    1、首先判断车的两侧是否只有一侧有车道线
    2、如果是，找到离车最近的车道线或路沿
    3、根据单侧车道线和路沿预测车道线
  */
  std::vector<std::pair<int, double>> left_track_ids;
  std::vector<std::pair<int, double>> right_track_ids;
  // 判断离车最近点在车的左右
  for (const auto& local_line : local_map_use_->lane_lines()) {
    if (local_line.points().size() < 2) {
      continue;
    }
    // if (local_line.points(0).x() > 0 ||
    //     local_line.points(local_line.points_size() - 1).x() < 0) {
    //   continue;
    // }
    Eigen::Vector3d veh_pt(0., 0., 0.);
    double min_dis = std::numeric_limits<double>::max();
    Eigen::Vector3d min_pt;
    for (const auto& it : local_line.points()) {
      Eigen::Vector3d pt(it.x(), it.y(), it.z());
      double dis = (veh_pt - pt).norm();
      if (dis < min_dis) {
        min_dis = dis;
        min_pt = pt;
      }
    }
    if (min_pt.y() > 0) {
      left_track_ids.emplace_back(local_line.track_id(), min_pt.y());
    } else {
      right_track_ids.emplace_back(local_line.track_id(), min_pt.y());
    }
  }
  // 排序
  std::sort(
      left_track_ids.begin(), left_track_ids.end(),
      [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
        return a.second < b.second;
      });
  std::sort(
      right_track_ids.begin(), right_track_ids.end(),
      [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
        return a.second > b.second;
      });

  std::vector<Eigen::Vector3d> right_road_pts;
  std::vector<Eigen::Vector3d> left_road_pts;
  for (const auto& local_road : local_map_->road_edges()) {
    if (local_road.points(0).x() > 0 ||
        local_road.points(local_road.points_size() - 1).x() < 0) {
      continue;
    }
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
  if (right_track_ids.empty() && !right_road_pts.empty()) {
    // 车右侧车道线为空，但路沿不为空，可以进行预测，但预测的车道线不能超过路沿
    if (!left_track_ids.empty()) {
      // use the nearest line to predict
      // 确定离车最近的车道线
      auto nearest_id = left_track_ids.front().first;
      if (local_line_table_.find(nearest_id) == local_line_table_.end()) {
        return;
      }
      auto nearest_line = local_line_table_.at(nearest_id).local_line_pts;
      // compute lane nums
      int lane_num = std::ceil(ComputeVecToLaneDis(nearest_line) / 3.5);
      FitSingleSideLine(&nearest_line, &new_lines, lane_num, true, 0, 3.5);
      // 检测预测的车道线是否会超过路沿，若超过路沿，则使用路沿线当做预测的边线
      if (!new_lines.empty() &&
          JudgeLineOverRoad(new_lines.back(), right_road_pts)) {
        for (auto& pt : right_road_pts) {
          pt.y() = pt.y() + 0.1;
        }
        new_lines.back() = right_road_pts;
      }
    } else if (left_track_ids.empty() && !left_road_pts.empty()) {
      if (std::abs(left_road_pts.front().y()) <
          std::abs(right_road_pts.front().y())) {
        int lane_num = std::ceil(ComputeVecToLaneDis(left_road_pts) / 3.5);
        if (left_road_pts.front().x() > 0 || left_road_pts.back().x() < 0 ||
            !local_map_use_->lane_lines().empty()) {
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
            !local_map_use_->lane_lines().empty()) {
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

  if (left_track_ids.empty() && !left_road_pts.empty()) {
    if (!right_track_ids.empty()) {
      // use the nearest line to predict
      // 确定离车最近的车道线
      auto nearest_id = right_track_ids.front().first;
      if (local_line_table_.find(nearest_id) == local_line_table_.end()) {
        return;
      }
      auto nearest_line = local_line_table_.at(nearest_id).local_line_pts;
      int lane_num = std::ceil(ComputeVecToLaneDis(nearest_line) / 3.5);
      FitSingleSideLine(&nearest_line, &new_lines, lane_num, false, 0, 3.5);
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
    HLOG_DEBUG << "fit new line failed";
    return;
  }

  // 将虚拟的路沿线设置为实线，非路沿线设置为虚线
  for (int i = 0; i < static_cast<int>(new_lines.size()); ++i) {
    if (new_lines[i].empty()) {
      continue;
    }
    auto* new_line = local_map_use_->add_lane_lines();
    for (const auto& pt : new_lines[i]) {
      if (std::isnan(pt.x()) || std::isnan(pt.y())) {
        continue;
      }
      auto* new_pt = new_line->add_points();
      new_pt->set_x(pt.x());
      new_pt->set_y(pt.y());
      new_pt->set_z(pt.z());
    }

    new_line->set_lanepos(
        static_cast<hozon::mapping::LanePositionType>(extra_val_));
    new_line->set_track_id(extra_val_);
    new_line->set_color(static_cast<hozon::mapping::Color>(1));
    if (i == static_cast<int>(new_lines.size()) - 1) {
      new_line->set_lanetype(static_cast<hozon::mapping::LaneType>(1));
      continue;
    }
    new_line->set_lanetype(static_cast<hozon::mapping::LaneType>(2));

    // 更新哈希表
    local_line_info local_line;
    local_line.line_track_id = extra_val_;
    local_line.local_line_pts = new_lines[i];
    local_line.lane_pos =
        static_cast<hozon::mapping::LanePositionType>(extra_val_);
    local_line_table_.insert_or_assign(extra_val_, local_line);
    extra_val_++;

    // 更新all_lines
    std::vector<cv::Point2f> kdtree_points;
    Line_kd line_kd;
    hozon::mapping::LaneLine kd_line;
    kd_line.set_track_id(static_cast<int>(static_cast<int>(extra_val_)));
    for (const auto& point : new_lines[i]) {
      if (std::isnan(point.x()) || std::isnan(point.y())) {
        continue;
      }
      kdtree_points.emplace_back(static_cast<float>(point.x()),
                                 static_cast<float>(point.y()));
      auto* new_pt = kd_line.add_points();
      new_pt->set_x(point.x());
      new_pt->set_y(point.y());
      new_pt->set_z(point.z());
    }
    cv::flann::KDTreeIndexParams index_param(1);
    std::shared_ptr<cv::flann::Index> kdtree_ptr =
        std::make_shared<cv::flann::Index>(cv::Mat(kdtree_points).reshape(1),
                                           index_param);
    line_kd.line_kdtree = kdtree_ptr;
    line_kd.line = std::make_shared<hozon::mapping::LaneLine>(kd_line);
    all_lines_[static_cast<int>(extra_val_)].emplace_back(line_kd);
  }
}

double GeoOptimization::ComputeVecToLaneDis(
    const std::vector<Eigen::Vector3d>& base_line) {
  // compute lane nums;
  if (base_line.empty()) {
    return 0;
  }
  Eigen::Vector3d P(0., 0., 0.);
  double dis = fabs(base_line.front().y());
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
    if (t < -0.1 || t > 1.1) {
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
        static_cast<hozon::mapping::LanePositionType>(extra_val_));
    new_line->set_track_id(extra_val_);
    extra_val_++;
    new_line->set_color(static_cast<hozon::mapping::Color>(1));
    // new_line->set_allocated_lane_param(static_cast<hozon::mapping::LaneCubicSpline>(500));
    // new_line->set_confidence(static_cast<hozon::mapping::LanePositionType>(500));
    new_line->set_lanetype(static_cast<hozon::mapping::LaneType>(1));
  }

  if (flag) {
    // predict the right line
    for (int i = 0; i < static_cast<int>(base_line->size()) - 1; ++i) {
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
    for (int i = 0; i < static_cast<int>(base_line->size()) - 1; ++i) {
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
    const std::shared_ptr<hozon::mapping::LocalMap>& msg,
    const std::shared_ptr<hozon::perception::PerceptionObstacles>& obj_msg) {
  // 将元素填充进element
  int32_t node_name = 1, centerline_name = 1, arrows_name = 1, stop_lines = 1;

  std::vector<std::vector<Eigen::Vector3d>> roads;
  for (const auto& road : local_map_->road_edges()) {
    std::vector<Eigen::Vector3d> road_pts;
    for (const auto& pt : road.points()) {
      Eigen::Vector3d ptt(pt.x(), pt.y(), pt.z());
      road_pts.emplace_back(ptt);
    }
    if (road_pts.empty()) {
      continue;
    }
    roads.emplace_back(road_pts);
  }

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
    FillIsNearRoadLine(&lane_line, roads, lane_line_it);
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

  // obj
  if (obj_msg != nullptr) {
    for (const auto& i : obj_msg->perception_obstacle()) {
      em::Obj obj;
      obj.id = i.track_id();
      FillObjType(&obj, i.type());
      em::Point point(i.position().x(), i.position().y(), i.position().z());
      obj.position = point;
      em::Point v(i.velocity().x(), i.velocity().y(), i.velocity().z());
      obj.velocity = v;
      for (const auto& pt : i.polygon_point()) {
        em::Point objpts(pt.x(), pt.y(), pt.z());
        obj.polygon.points.emplace_back(objpts);
      }
      obj.heading = i.theta();
      obj.length = i.length();
      obj.width = i.width();
      elem_map_->objs[obj.id] = std::make_shared<em::Obj>(obj);
    }
  }
}

void GeoOptimization::FillIsNearRoadLine(
    hozon::mp::mf::em::Boundary* lane_line,
    const std::vector<std::vector<Eigen::Vector3d>>& roads,
    const hozon::mapping::LaneLine& local_line) {
  // 判断是否是靠近路沿的线
  if (roads.empty()) {
    return;
  }
  std::vector<Eigen::Vector3d> line_it;
  for (const auto& pt : local_line.points()) {
    Eigen::Vector3d ptt(pt.x(), pt.y(), pt.z());
    line_it.emplace_back(ptt);
  }
  for (const auto& road_it : roads) {
    double avg_dis = 0.0;
    if (!ComputerLineDis(line_it, road_it, &avg_dis)) {
      continue;
    }
    if (avg_dis < 1.0) {
      lane_line->is_near_road_edge = true;
      break;
    }
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

void GeoOptimization::FillObjType(
    hozon::mp::mf::em::Obj* obj,
    hozon::perception::PerceptionObstacle_Type objtype) {
  switch (objtype) {
    case hozon::perception::PerceptionObstacle_Type::
        PerceptionObstacle_Type_UNKNOWN:
      obj->type = hozon::mp::mf::em::ObjType::UNKNOWN;
      break;
    case hozon::perception::PerceptionObstacle_Type::
        PerceptionObstacle_Type_UNKNOWN_UNMOVABLE:
      obj->type = hozon::mp::mf::em::ObjType::UNKNOWN_UNMOVABLE;
      break;
    case hozon::perception::PerceptionObstacle_Type::
        PerceptionObstacle_Type_UNKNOWN_MOVABLE:
      obj->type = hozon::mp::mf::em::ObjType::UNKNOWN_MOVABLE;
      break;
    case hozon::perception::PerceptionObstacle_Type::
        PerceptionObstacle_Type_PEDESTRIAN:
      obj->type = hozon::mp::mf::em::ObjType::PEDESTRIAN;
      break;
    case hozon::perception::PerceptionObstacle_Type::
        PerceptionObstacle_Type_BICYCLE:
      obj->type = hozon::mp::mf::em::ObjType::BICYCLE;
      break;
    case hozon::perception::PerceptionObstacle_Type::
        PerceptionObstacle_Type_VEHICLE:
      obj->type = hozon::mp::mf::em::ObjType::VEHICLE;
      break;
    case hozon::perception::PerceptionObstacle_Type::
        PerceptionObstacle_Type_CYCLIST:
      obj->type = hozon::mp::mf::em::ObjType::CYCLIST;
      break;
    case hozon::perception::PerceptionObstacle_Type::
        PerceptionObstacle_Type_STATIC_OBSTACLE:
      obj->type = hozon::mp::mf::em::ObjType::STATIC_OBSTACLE;
      break;
    case hozon::perception::PerceptionObstacle_Type::
        PerceptionObstacle_Type_TRANSPORT_ELEMENT:
      obj->type = hozon::mp::mf::em::ObjType::TRANSPORT_ELEMENT;
      break;
    case hozon::perception::PerceptionObstacle_Type::
        PerceptionObstacle_Type_ANIMAL:
      obj->type = hozon::mp::mf::em::ObjType::ANIMAL;
      break;
    default:
      obj->type = hozon::mp::mf::em::ObjType::UNKNOWN;
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
//   for (auto lane_it = map_lanes_->begin(); lane_it !=
//   map_lanes_->end();
//        lane_it++) {
//     //
//     由于一些没点的line和lane类型不对的已经在UpdateLaneByLocalmap被滤除掉了，因此这边不做边界操作
//     auto* lane = pilot_map_->add_lane();
//     lane->mutable_id()->set_id(std::to_string(lane_it->lane_id_));
//     // central_curve
//     auto* central_segment =
//     lane->mutable_central_curve()->add_segment(); for (const auto&
//     point_msg : lane_it->pilot_center_line_.points()) {
//       Eigen::Vector3d point_local(point_msg.x(), point_msg.y(),
//       point_msg.z()); Eigen::Vector3d point_enu = T_U_V_ * point_local;
//       auto* point =
//       central_segment->mutable_line_segment()->add_point();
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
//     // lane_it->left_line_.lanetype_,
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
//     //
//     DataConvert::ConvertInnerMapLaneType(lane_msg.right_line_.color_,
//     false,
//     // lane_msg.right_line_.lanetype_,
//     //                                     &right_type);
//     lane->mutable_right_boundary()->add_boundary_type()->add_types(right_type);
//     lane->mutable_right_boundary()->set_length(0);
//     segment =
//     lane->mutable_right_boundary()->mutable_curve()->add_segment(); for
//     (const auto& point_msg : lane_it->pilot_right_line_.points()) {
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
//   for (auto lane_msg = map_lanes_->begin(); lane_msg !=
//   map_lanes_->end();
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
//   for (const auto& point_msg :
//   (*map_lanes_).begin()->left_line_.points())
//   {
//     auto* left_edge_point = left_segment->add_point();
//     Eigen::Vector3d point_local(point_msg.x(), point_msg.y(),
//     point_msg.z()); Eigen::Vector3d point_enu = T_U_V_ * point_local;
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
