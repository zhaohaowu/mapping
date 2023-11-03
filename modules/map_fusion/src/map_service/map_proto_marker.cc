/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： local_map_provider.cc
 *   author     ： zuodongsheng
 *   date       ： 2023.09
 ******************************************************************************/
#include <gflags/gflags.h>

#include "common/utm_projection/coordinate_convertor.h"
#include "map_fusion/map_service/map_proto_maker.h"
#include "util/temp_log.h"

namespace hozon {
namespace mp {
namespace mf {

adsfi_proto::viz::TransformStamped MapProtoMarker::CarTrackTF(
    const Eigen::Vector3d& pose, const Eigen::Quaterniond& q_W_V,
    const hozon::common::Header& stamp, bool utm) {
  if (!RVIZ_AGENT.Ok()) {
    return {};
  }
  adsfi_proto::viz::TransformStamped geo_tf;
  static uint32_t seq = 0;
  int curr_seq = seq++;
  geo_tf.mutable_header()->set_seq(curr_seq);
  auto sec = static_cast<uint32_t>(stamp.publish_stamp());
  auto nsec = static_cast<uint32_t>((stamp.publish_stamp() - sec) * 1e9);
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
  RVIZ_AGENT.Publish("/priormap/transform_stamped", geo_tf);
  return geo_tf;
}

void MapProtoMarker::CarTrack(const Eigen::Vector3d& pose,
                              const Eigen::Quaterniond& q_W_V,
                              adsfi_proto::viz::Path* location_path_,
                              const hozon::common::Header& stamp, bool utm) {
  static uint32_t seq = 0;
  int curr_seq = seq++;
  auto* location_pose = location_path_->add_poses();
  location_path_->mutable_header()->set_seq(curr_seq);
  auto sec = static_cast<uint32_t>(stamp.publish_stamp());
  auto nsec = static_cast<uint32_t>((stamp.publish_stamp() - sec) * 1e9);
  location_path_->mutable_header()->mutable_timestamp()->set_sec(sec);
  location_path_->mutable_header()->mutable_timestamp()->set_nsec(nsec);
  location_path_->mutable_header()->set_frameid("map");

  location_pose->mutable_header()->set_seq(curr_seq);
  location_pose->mutable_header()->mutable_timestamp()->set_sec(sec);
  location_pose->mutable_header()->mutable_timestamp()->set_nsec(nsec);
  location_pose->mutable_header()->set_frameid("map");
  location_pose->mutable_pose()->mutable_position()->set_x(pose.x());
  location_pose->mutable_pose()->mutable_position()->set_y(pose.y());
  location_pose->mutable_pose()->mutable_position()->set_z(pose.z());

  location_pose->mutable_pose()->mutable_orientation()->set_w(q_W_V.w());
  location_pose->mutable_pose()->mutable_orientation()->set_x(q_W_V.x());
  location_pose->mutable_pose()->mutable_orientation()->set_y(q_W_V.y());
  location_pose->mutable_pose()->mutable_orientation()->set_z(q_W_V.z());

  if (location_path_->poses().size() > 50) {
    location_path_->mutable_poses()->DeleteSubrange(0, 1);
  }
}
std::vector<adsfi_proto::viz::MarkerArray> MapProtoMarker::LaneToMarker(
    const std::shared_ptr<hozon::hdmap::Map>& prior_map,
    const Eigen::Vector3d& enupos, bool utm) {
  std::vector<adsfi_proto::viz::MarkerArray> lane_marker_arrays;
  adsfi_proto::viz::MarkerArray lane_markers;
  for (const auto& lane : prior_map->lane()) {
    auto lane_marker = std::make_unique<adsfi_proto::viz::Marker>();
    lane_marker->mutable_header()->set_frameid("map");
    u_int64_t laneId = std::stoll(lane.id().id());
    int remainder = laneId / 10000;
    lane_marker->set_ns("ns_prior_map_lane" + std::to_string(remainder));
    lane_marker->set_id(laneId % 10000);
    lane_marker->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
    lane_marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
    // 位姿
    lane_marker->mutable_pose()->mutable_position()->set_x(0);
    lane_marker->mutable_pose()->mutable_position()->set_y(0);
    lane_marker->mutable_pose()->mutable_position()->set_z(0);
    lane_marker->mutable_pose()->mutable_orientation()->set_x(0.);
    lane_marker->mutable_pose()->mutable_orientation()->set_y(0.);
    lane_marker->mutable_pose()->mutable_orientation()->set_z(0.);
    lane_marker->mutable_pose()->mutable_orientation()->set_w(1.);
    // 尺寸
    lane_marker->mutable_scale()->set_x(0.05);
    lane_marker->mutable_scale()->set_y(0.05);
    lane_marker->mutable_scale()->set_z(0.05);
    lane_marker->mutable_lifetime()->set_sec(0);
    lane_marker->mutable_lifetime()->set_nsec(200000000);
    adsfi_proto::viz::ColorRGBA color;
    color.set_a(0.8);
    color.set_r(1);
    color.set_g(1);
    color.set_b(1);
    lane_marker->mutable_color()->CopyFrom(color);
    for (int i = 0; i < lane.central_curve().segment().size(); ++i) {
      const auto& segment = lane.central_curve().segment(i);
      for (int j = 0; j < segment.line_segment().point().size(); ++j) {
        const auto& point = segment.line_segment().point(j);
        if (utm) {
          Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
          int zone = 51;
          double x = point_utm.x();
          double y = point_utm.y();
          hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
          Eigen::Vector3d point_gcj(y, x, 0);
          Eigen::Vector3d point_enu = util::Geo::Gcj02ToEnu(point_gcj, enupos);
          auto pt = lane_marker->add_points();
          pt->set_x(point_enu.x());
          pt->set_y(point_enu.y());
          pt->set_z(point_enu.z());
        } else {
          Eigen::Vector3d point_enu(point.x(), point.y(), point.z());
          auto pt = lane_marker->add_points();
          pt->set_x(point_enu.x());
          pt->set_y(point_enu.y());
          pt->set_z(point_enu.z());
        }
      }
    }
    if (lane_marker->points().empty()) {
      HLOG_WARN << "empty lane";
    }
    lane_markers.add_markers()->CopyFrom(*lane_marker);
  }
  lane_marker_arrays.push_back(lane_markers);
  adsfi_proto::viz::MarkerArray left_boundary_marker =
      LaneLeftBoundaryMarker(prior_map, enupos, utm);
  adsfi_proto::viz::MarkerArray right_boundary_marker =
      LaneRightBoundaryMarker(prior_map, enupos, utm);
  lane_marker_arrays.push_back(left_boundary_marker);
  lane_marker_arrays.push_back(right_boundary_marker);

  return lane_marker_arrays;
}
adsfi_proto::viz::MarkerArray MapProtoMarker::LaneLeftBoundaryMarker(
    const std::shared_ptr<hozon::hdmap::Map>& prior_map,
    const Eigen::Vector3d& enupos, bool utm) {
  adsfi_proto::viz::MarkerArray left_boundary_markers;
  for (const auto& lane : prior_map->lane()) {
    auto lane_marker = std::make_unique<adsfi_proto::viz::Marker>();
    lane_marker->mutable_header()->set_frameid("map");
    u_int64_t laneId = std::stoll(lane.id().id());
    int remainder = laneId / 10000;
    lane_marker->set_ns("ns_prior_map_lane" + std::to_string(remainder));
    lane_marker->set_id(laneId % 10000);
    lane_marker->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
    lane_marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
    // 位姿
    lane_marker->mutable_pose()->mutable_position()->set_x(0);
    lane_marker->mutable_pose()->mutable_position()->set_y(0);
    lane_marker->mutable_pose()->mutable_position()->set_z(0);
    lane_marker->mutable_pose()->mutable_orientation()->set_x(0.);
    lane_marker->mutable_pose()->mutable_orientation()->set_y(0.);
    lane_marker->mutable_pose()->mutable_orientation()->set_z(0.);
    lane_marker->mutable_pose()->mutable_orientation()->set_w(1.);
    // 尺寸
    lane_marker->mutable_scale()->set_x(0.05);
    lane_marker->mutable_scale()->set_y(0.05);
    lane_marker->mutable_scale()->set_z(0.05);
    lane_marker->mutable_lifetime()->set_sec(0);
    lane_marker->mutable_lifetime()->set_nsec(0);
    adsfi_proto::viz::ColorRGBA color;
    color.set_a(0.8);
    color.set_r(1.0);
    color.set_g(0.65);
    color.set_b(0);
    lane_marker->mutable_color()->CopyFrom(color);
    for (int i = 0; i < lane.left_boundary().curve().segment().size(); ++i) {
      const auto& segment = lane.left_boundary().curve().segment(i);
      for (int j = 0; j < segment.line_segment().point().size(); ++j) {
        const auto& point = segment.line_segment().point(j);
        if (utm) {
          Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
          int zone = 51;
          double x = point_utm.x();
          double y = point_utm.y();
          hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
          Eigen::Vector3d point_gcj(y, x, 0);
          Eigen::Vector3d point_enu = util::Geo::Gcj02ToEnu(point_gcj, enupos);
          auto pt = lane_marker->add_points();
          pt->set_x(point_enu.x());
          pt->set_y(point_enu.y());
          pt->set_z(point_enu.z());
        } else {
          Eigen::Vector3d point_enu(point.x(), point.y(), point.z());
          auto pt = lane_marker->add_points();
          pt->set_x(point_enu.x());
          pt->set_y(point_enu.y());
          pt->set_z(point_enu.z());
        }
      }
    }
    if (lane_marker->points().empty()) {
      HLOG_WARN << "empty lane";
    }
    left_boundary_markers.add_markers()->CopyFrom(*lane_marker);
  }
  return left_boundary_markers;
}
adsfi_proto::viz::MarkerArray MapProtoMarker::LaneRightBoundaryMarker(
    const std::shared_ptr<hozon::hdmap::Map>& prior_map,
    const Eigen::Vector3d& enupos, bool utm) {
  adsfi_proto::viz::MarkerArray right_boundary_markers;
  for (const auto& lane : prior_map->lane()) {
    auto lane_marker = std::make_unique<adsfi_proto::viz::Marker>();
    lane_marker->mutable_header()->set_frameid("map");
    u_int64_t laneId = std::stoll(lane.id().id());
    int remainder = laneId / 10000;
    lane_marker->set_ns("ns_prior_map_lane" + std::to_string(remainder));
    lane_marker->set_id(laneId % 10000);
    lane_marker->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
    lane_marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
    // 位姿
    lane_marker->mutable_pose()->mutable_position()->set_x(0);
    lane_marker->mutable_pose()->mutable_position()->set_y(0);
    lane_marker->mutable_pose()->mutable_position()->set_z(0);
    lane_marker->mutable_pose()->mutable_orientation()->set_x(0.);
    lane_marker->mutable_pose()->mutable_orientation()->set_y(0.);
    lane_marker->mutable_pose()->mutable_orientation()->set_z(0.);
    lane_marker->mutable_pose()->mutable_orientation()->set_w(1.);
    // 尺寸
    lane_marker->mutable_scale()->set_x(0.2);
    lane_marker->mutable_scale()->set_y(0.2);
    lane_marker->mutable_scale()->set_z(0.2);
    lane_marker->mutable_lifetime()->set_sec(0);
    lane_marker->mutable_lifetime()->set_nsec(0);
    adsfi_proto::viz::ColorRGBA color;
    color.set_a(0.8);
    color.set_r(1.0);
    color.set_g(0.65);
    color.set_b(0);
    lane_marker->mutable_color()->CopyFrom(color);
    for (int i = 0; i < lane.right_boundary().curve().segment().size(); ++i) {
      const auto& segment = lane.right_boundary().curve().segment(i);
      for (int j = 0; j < segment.line_segment().point().size(); ++j) {
        const auto& point = segment.line_segment().point(j);
        if (utm) {
          Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
          int zone = 51;
          double x = point_utm.x();
          double y = point_utm.y();
          hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
          Eigen::Vector3d point_gcj(y, x, 0);
          Eigen::Vector3d point_enu = util::Geo::Gcj02ToEnu(point_gcj, enupos);
          auto pt = lane_marker->add_points();
          pt->set_x(point_enu.x());
          pt->set_y(point_enu.y());
          pt->set_z(point_enu.z());
        } else {
          Eigen::Vector3d point_enu(point.x(), point.y(), point.z());
          auto pt = lane_marker->add_points();
          pt->set_x(point_enu.x());
          pt->set_y(point_enu.y());
          pt->set_z(point_enu.z());
        }
      }
    }
    if (lane_marker->points().empty()) {
      HLOG_WARN << "empty lane";
    }
    right_boundary_markers.add_markers()->CopyFrom(*lane_marker);
  }
  return right_boundary_markers;
}
adsfi_proto::viz::MarkerArray MapProtoMarker::LaneID(
    const std::shared_ptr<hozon::hdmap::Map>& prior_map,
    const Eigen::Vector3d& enupos, bool utm) {
  adsfi_proto::viz::MarkerArray ID_markers;
  for (const auto& lane : prior_map->lane()) {
    u_int64_t laneId = std::stoll(lane.id().id());
    int remainder = laneId / 10000;
    for (int i = 0; i < lane.left_boundary().curve().segment().size(); ++i) {
      const auto& segment = lane.left_boundary().curve().segment(i);
      for (int j = 0; j < segment.line_segment().point().size(); ++j) {
        const auto& point = segment.line_segment().point(j);
        Eigen::Vector3d point_enu;
        if (utm) {
          Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
          int zone = 51;
          double x = point_utm.x();
          double y = point_utm.y();
          hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
          Eigen::Vector3d point_gcj(y, x, 0);
          Eigen::Vector3d point_enu = util::Geo::Gcj02ToEnu(point_gcj, enupos);
        } else {
          Eigen::Vector3d point_enu(point.x(), point.y(), point.z());
        }
        auto text_marker = std::make_unique<adsfi_proto::viz::Marker>();
        text_marker->mutable_header()->set_frameid("map");
        text_marker->set_ns("ns_prior_map_lane" + std::to_string(remainder));
        text_marker->set_id(laneId % 10000);
        text_marker->set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
        text_marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
        text_marker->mutable_pose()->mutable_position()->set_x(point_enu.x());
        text_marker->mutable_pose()->mutable_position()->set_y(point_enu.y());
        text_marker->mutable_pose()->mutable_position()->set_z(point_enu.z() +
                                                               1.0);
        text_marker->mutable_pose()->mutable_orientation()->set_x(0.);
        text_marker->mutable_pose()->mutable_orientation()->set_y(0.);
        text_marker->mutable_pose()->mutable_orientation()->set_z(0.);
        text_marker->mutable_pose()->mutable_orientation()->set_w(1.);
        text_marker->set_text(lane.id().id());
        text_marker->mutable_scale()->set_x(1.0);
        text_marker->mutable_scale()->set_y(1.0);
        text_marker->mutable_scale()->set_z(1.0);
        text_marker->mutable_lifetime()->set_sec(0);
        text_marker->mutable_lifetime()->set_nsec(0);
        text_marker->mutable_color()->set_a(1.0);
        text_marker->mutable_color()->set_r(1.0);
        text_marker->mutable_color()->set_g(1.0);
        text_marker->mutable_color()->set_b(1.0);
        ID_markers.add_markers()->CopyFrom(*text_marker);
      }
    }
  }
  return ID_markers;
}
adsfi_proto::viz::MarkerArray MapProtoMarker::LaneRightNeighborForward(
    const std::shared_ptr<hozon::hdmap::Map>& prior_map,
    const Eigen::Vector3d& enupos, bool utm) {
  adsfi_proto::viz::MarkerArray arrow_markers;
  int color_index = 0;
  for (const auto& lane : prior_map->lane()) {
    u_int64_t laneId = std::stoll(lane.id().id());
    int remainder = laneId / 10000;
    const auto start_lane_curve_left = lane.left_boundary().curve();
    const auto start_lane_curve_right = lane.right_boundary().curve();
    if (start_lane_curve_right.segment().empty() ||
        start_lane_curve_left.segment().empty() ||
        start_lane_curve_left.segment()[0].line_segment().point().empty() ||
        start_lane_curve_right.segment()[0].line_segment().point().empty()) {
      continue;
    }
    const auto start_lane_curve_left_point0 =
        start_lane_curve_left.segment()[0].line_segment().point()[0];
    const auto start_lane_curve_right_point0 =
        start_lane_curve_right.segment()[0].line_segment().point()[0];
    hozon::common::PointENU start_point;
    if (start_lane_curve_left.segment_size() > 0 &&
        start_lane_curve_right.segment_size() > 0) {
      start_point.set_x((start_lane_curve_left_point0.x() +
                         start_lane_curve_right_point0.x()) /
                        2);
      start_point.set_y((start_lane_curve_left_point0.y() +
                         start_lane_curve_right_point0.y()) /
                        2);
      start_point.set_z((start_lane_curve_left_point0.z() +
                         start_lane_curve_right_point0.z()) /
                        2);
    } else if (start_lane_curve_left.segment_size() > 0 &&
               start_lane_curve_right.segment_size() == 0) {
      bool all_points_size_one = true;
      for (const auto& segment : start_lane_curve_left.segment()) {
        if (segment.line_segment().point_size() > 1) {
          const auto& start_segment = segment.line_segment();
          const auto& start_point_0 = start_segment.point(0);
          const auto& start_point_1 = start_segment.point(1);
          // 计算向量长度和方向
          double dx = start_point_1.x() - start_point_0.x();
          double dy = start_point_1.y() - start_point_0.y();
          double length = std::sqrt(dx * dx + dy * dy);
          double direction_x = dx / length;
          double direction_y = dy / length;
          // 计算在向量左侧的点坐标
          double offset = 0.1;  // 偏移量，可以根据需求调整
          double target_x = start_point_0.x() - direction_y * offset * length;
          double target_y = start_point_0.y() + direction_x * offset * length;
          start_point.set_x(target_x);
          start_point.set_y(target_y);
          all_points_size_one = false;
          break;
        }
      }
      if (all_points_size_one) {
        start_point.set_x(start_lane_curve_left_point0.x());
        start_point.set_y(start_lane_curve_left_point0.y());
        start_point.set_z(start_lane_curve_left_point0.z());
      }
    } else if (start_lane_curve_left.segment_size() == 0 &&
               start_lane_curve_right.segment_size() > 0) {
      bool all_points_size_one = true;
      for (const auto& segment : start_lane_curve_right.segment()) {
        if (segment.line_segment().point_size() > 1) {
          const auto& start_segment = segment.line_segment();
          const auto& start_point_0 = start_segment.point(0);
          const auto& start_point_1 = start_segment.point(1);

          // 计算向量长度和方向
          double dx = start_point_1.x() - start_point_0.x();
          double dy = start_point_1.y() - start_point_0.y();
          double length = std::sqrt(dx * dx + dy * dy);
          double direction_x = dx / length;
          double direction_y = dy / length;

          // 计算在向量右侧的点坐标
          double offset = 0.1;  // 偏移量，可以根据需求调整
          double target_x = start_point_0.x() + direction_y * offset * length;
          double target_y = start_point_0.y() - direction_x * offset * length;

          start_point.set_x(target_x);
          start_point.set_y(target_y);
          all_points_size_one = false;
          break;
        }
      }
      if (all_points_size_one) {
        start_point.set_x(start_lane_curve_right_point0.x());
        start_point.set_y(start_lane_curve_right_point0.y());
        start_point.set_z(start_lane_curve_right_point0.z());
      }
    }
    Eigen::Vector3d point_enu_start;
    if (utm) {
      point_enu_start = ConvertPoint(start_point, enupos);
    } else {
      point_enu_start =
          Eigen::Vector3d(start_point.x(), start_point.y(), start_point.z());
    }
    if (lane.right_neighbor_forward_lane_id_size()) {
      // 获取右侧相邻的lane ID
      for (const auto& neighbor_id : lane.right_neighbor_forward_lane_id()) {
        const std::string& neighbor_lane_id = neighbor_id.id();
        // 遍历所有lane，找到匹配的相邻lane
        // int color_index = 0;
        for (const auto& target_lane : prior_map->lane()) {
          if (target_lane.id().id() == neighbor_lane_id) {
            // 确定箭头终点位置
            const auto lane_curve_left = target_lane.left_boundary().curve();
            const auto lane_curve_right = target_lane.right_boundary().curve();
            if (lane_curve_right.segment().empty() ||
                lane_curve_left.segment().empty() ||
                lane_curve_left.segment()[0].line_segment().point().empty() ||
                lane_curve_right.segment()[0].line_segment().point().empty()) {
              continue;
            }
            const auto lane_curve_left_point0 =
                lane_curve_left.segment()[0].line_segment().point()[0];
            const auto lane_curve_right_point0 =
                lane_curve_right.segment()[0].line_segment().point()[0];
            hozon::common::PointENU end_point;
            if (lane_curve_left.segment_size() > 0 &&
                lane_curve_right.segment_size() > 0) {
              end_point.set_x(
                  (lane_curve_left_point0.x() + lane_curve_right_point0.x()) /
                  2);
              end_point.set_y(
                  (lane_curve_left_point0.y() + lane_curve_right_point0.y()) /
                  2);
              end_point.set_z(
                  (lane_curve_left_point0.z() + lane_curve_right_point0.z()) /
                  2);
            } else if (lane_curve_left.segment_size() > 0 &&
                       lane_curve_right.segment_size() == 0) {
              bool all_points_size_one = true;
              for (const auto& segment : lane_curve_left.segment()) {
                if (segment.line_segment().point_size() > 1) {
                  const auto& start_segment = segment.line_segment();
                  const auto& end_point_0 = start_segment.point(0);
                  const auto& end_point_1 = start_segment.point(1);
                  // 计算向量长度和方向
                  double dx = end_point_1.x() - end_point_0.x();
                  double dy = end_point_1.y() - end_point_0.y();
                  double length = std::sqrt(dx * dx + dy * dy);
                  double direction_x = dx / length;
                  double direction_y = dy / length;
                  // 计算在向量左侧的点坐标
                  double offset = 0.1;  // 偏移量，可以根据需求调整
                  double target_x =
                      end_point_0.x() - direction_y * offset * length;
                  double target_y =
                      end_point_0.y() + direction_x * offset * length;
                  end_point.set_x(target_x);
                  end_point.set_y(target_y);
                  all_points_size_one = false;
                  break;
                }
              }
              if (all_points_size_one) {
                end_point.set_x(lane_curve_left_point0.x());
                end_point.set_y(lane_curve_left_point0.y());
                end_point.set_z(lane_curve_left_point0.z());
              }
            } else if (lane_curve_left.segment_size() == 0 &&
                       lane_curve_right.segment_size() > 0) {
              bool all_points_size_one = true;
              for (const auto& segment : lane_curve_right.segment()) {
                if (segment.line_segment().point_size() > 1) {
                  const auto& start_segment = segment.line_segment();
                  const auto& end_point_0 = start_segment.point(0);
                  const auto& end_point_1 = start_segment.point(1);

                  // 计算向量长度和方向
                  double dx = end_point_1.x() - end_point_0.x();
                  double dy = end_point_1.y() - end_point_0.y();
                  double length = std::sqrt(dx * dx + dy * dy);
                  double direction_x = dx / length;
                  double direction_y = dy / length;

                  // 计算在向量右侧的点坐标
                  double offset = 0.1;  // 偏移量，可以根据需求调整
                  double target_x =
                      end_point_0.x() + direction_y * offset * length;
                  double target_y =
                      end_point_0.y() - direction_x * offset * length;

                  end_point.set_x(target_x);
                  end_point.set_y(target_y);
                  all_points_size_one = false;
                  break;
                }
              }
              if (all_points_size_one) {
                end_point.set_x(lane_curve_right_point0.x());
                end_point.set_y(lane_curve_right_point0.y());
                end_point.set_z(lane_curve_right_point0.z());
              }
            }
            Eigen::Vector3d point_enu_end;
            if (utm) {
              point_enu_end = ConvertPoint(end_point, enupos);
            } else {
              point_enu_end =
                  Eigen::Vector3d(end_point.x(), end_point.y(), end_point.z());
            }
            auto arrow_marker = std::make_unique<adsfi_proto::viz::Marker>();
            arrow_marker->clear_points();
            arrow_marker->mutable_header()->set_frameid("map");
            arrow_marker->set_ns("ns_right_neighbor_forward" +
                                 std::to_string(remainder));
            arrow_marker->set_id(laneId % 10000);
            arrow_marker->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
            arrow_marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
            auto start_point_marker = arrow_marker->add_points();
            start_point_marker->set_x(point_enu_start.x());
            start_point_marker->set_y(point_enu_start.y());
            start_point_marker->set_z(point_enu_start.z() - 2);

            // 添加终点
            auto end_point_marker = arrow_marker->add_points();
            end_point_marker->set_x(point_enu_end.x());
            end_point_marker->set_y(point_enu_end.y());
            end_point_marker->set_z(point_enu_end.z() - 2);

            // 设置颜色和尺寸
            arrow_marker->mutable_scale()->set_x(0.2);  // 设置线段的宽度
            arrow_marker->mutable_scale()->set_y(0.2);  // 设置线段的高度
            arrow_marker->mutable_scale()->set_z(0.2);  // 设置线段的高度
            arrow_marker->mutable_lifetime()->set_sec(0);
            arrow_marker->mutable_lifetime()->set_nsec(0);
            arrow_marker->mutable_pose()->mutable_orientation()->set_x(0.);
            arrow_marker->mutable_pose()->mutable_orientation()->set_y(0.);
            arrow_marker->mutable_pose()->mutable_orientation()->set_z(0.);
            arrow_marker->mutable_pose()->mutable_orientation()->set_w(1.);
            arrow_marker->mutable_lifetime()->set_sec(0);
            arrow_marker->mutable_lifetime()->set_nsec(0);
            adsfi_proto::viz::ColorRGBA color;
            color.set_a(1);
            color.set_r(0);    // 增加颜色分量变化幅度
            color.set_g(0.6);  // 增加颜色分量变化幅度
            color.set_b(1);    // 增加颜色分量变化幅度
            arrow_marker->mutable_color()->CopyFrom(color);
            // sphere marker
            auto sphere_marker = std::make_unique<adsfi_proto::viz::Marker>();
            sphere_marker->mutable_header()->set_frameid("map");
            sphere_marker->set_ns("ns_right_neighbor_forward_sphere" +
                                  std::to_string(remainder));
            sphere_marker->set_id(laneId % 10000);

            sphere_marker->set_type(adsfi_proto::viz::MarkerType::CUBE);
            sphere_marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
            sphere_marker->mutable_pose()->mutable_position()->set_x(
                point_enu_end.x());
            sphere_marker->mutable_pose()->mutable_position()->set_y(
                point_enu_end.y());
            sphere_marker->mutable_pose()->mutable_position()->set_z(
                point_enu_end.z() - 2);

            // 设置颜色和尺寸
            sphere_marker->mutable_scale()->set_x(0.4);  // 设置球体直径
            sphere_marker->mutable_scale()->set_y(0.4);
            sphere_marker->mutable_scale()->set_z(0.4);
            sphere_marker->mutable_pose()->mutable_orientation()->set_x(0.);
            sphere_marker->mutable_pose()->mutable_orientation()->set_y(0.);
            sphere_marker->mutable_pose()->mutable_orientation()->set_z(0.);
            sphere_marker->mutable_pose()->mutable_orientation()->set_w(1.);
            sphere_marker->mutable_lifetime()->set_sec(0);
            sphere_marker->mutable_lifetime()->set_nsec(0);
            adsfi_proto::viz::ColorRGBA colors;
            colors.set_a(1);
            colors.set_r(0);  // 增加颜色分量变化幅度
            colors.set_g(0);  // 增加颜色分量变化幅度
            colors.set_b(0);  // 增加颜色分量变化幅度
            // 设置颜色和尺寸
            sphere_marker->mutable_color()->CopyFrom(colors);
            arrow_markers.add_markers()->CopyFrom(*arrow_marker);
            arrow_markers.add_markers()->CopyFrom(*sphere_marker);
            break;
          }
        }
      }
    }
  }
  return arrow_markers;
}  // namespace mf
adsfi_proto::viz::MarkerArray MapProtoMarker::LaneLeftNeighborForward(
    const std::shared_ptr<hozon::hdmap::Map>& prior_map,
    const Eigen::Vector3d& enupos, bool utm) {
  adsfi_proto::viz::MarkerArray arrow_markers;
  int color_index = 0;
  for (const auto& lane : prior_map->lane()) {
    // HLOG_ERROR << "===== laneId " << lane.id().id();
    u_int64_t laneId = std::stoll(lane.id().id());
    int remainder = laneId / 10000;
    const auto start_lane_curve_left = lane.left_boundary().curve();
    const auto start_lane_curve_right = lane.right_boundary().curve();
    if (start_lane_curve_left.segment().empty() ||
        start_lane_curve_right.segment().empty() ||
        start_lane_curve_left.segment(0).line_segment().point().empty() ||
        start_lane_curve_right.segment(0).line_segment().point().empty()) {
      continue;
    }

    hozon::common::PointENU start_point;
    if (start_lane_curve_left.segment_size() > 0 &&
        start_lane_curve_right.segment_size() > 0) {
      const auto start_lane_curve_left_point0 =
          start_lane_curve_left.segment()[0].line_segment().point()[0];
      const auto start_lane_curve_right_point0 =
          start_lane_curve_right.segment()[0].line_segment().point()[0];
      start_point.set_x((start_lane_curve_left_point0.x() +
                         start_lane_curve_right_point0.x()) /
                        2);
      start_point.set_y((start_lane_curve_left_point0.y() +
                         start_lane_curve_right_point0.y()) /
                        2);
      start_point.set_z((start_lane_curve_left_point0.z() +
                         start_lane_curve_right_point0.z()) /
                        2);
    } else if (start_lane_curve_left.segment_size() > 0 &&
               start_lane_curve_right.segment_size() == 0) {
      const auto start_lane_curve_left_point0 =
          start_lane_curve_left.segment()[0].line_segment().point()[0];
      bool all_points_size_one = true;
      for (const auto& segment : start_lane_curve_left.segment()) {
        if (segment.line_segment().point_size() > 1) {
          const auto& start_segment = segment.line_segment();
          const auto& start_point_0 = start_segment.point(0);
          const auto& start_point_1 = start_segment.point(1);
          // 计算向量长度和方向
          double dx = start_point_1.x() - start_point_0.x();
          double dy = start_point_1.y() - start_point_0.y();
          double length = std::sqrt(dx * dx + dy * dy);
          double direction_x = dx / length;
          double direction_y = dy / length;
          // 计算在向量左侧的点坐标
          double offset = 0.1;  // 偏移量，可以根据需求调整
          double target_x = start_point_0.x() - direction_y * offset * length;
          double target_y = start_point_0.y() + direction_x * offset * length;
          start_point.set_x(target_x);
          start_point.set_y(target_y);
          all_points_size_one = false;
          break;
        }
      }
      if (all_points_size_one) {
        start_point.set_x(start_lane_curve_left_point0.x());
        start_point.set_y(start_lane_curve_left_point0.y());
        start_point.set_z(start_lane_curve_left_point0.z());
      }
    } else if (start_lane_curve_left.segment_size() == 0 &&
               start_lane_curve_right.segment_size() > 0) {
      const auto start_lane_curve_right_point0 =
          start_lane_curve_right.segment()[0].line_segment().point()[0];
      bool all_points_size_one = true;
      for (const auto& segment : start_lane_curve_right.segment()) {
        if (segment.line_segment().point_size() > 1) {
          const auto& start_segment = segment.line_segment();
          const auto& start_point_0 = start_segment.point(0);
          const auto& start_point_1 = start_segment.point(1);

          // 计算向量长度和方向
          double dx = start_point_1.x() - start_point_0.x();
          double dy = start_point_1.y() - start_point_0.y();
          double length = std::sqrt(dx * dx + dy * dy);
          double direction_x = dx / length;
          double direction_y = dy / length;

          // 计算在向量右侧的点坐标
          double offset = 0.1;  // 偏移量，可以根据需求调整
          double target_x = start_point_0.x() + direction_y * offset * length;
          double target_y = start_point_0.y() - direction_x * offset * length;

          start_point.set_x(target_x);
          start_point.set_y(target_y);
          all_points_size_one = false;
          break;
        }
      }
      if (all_points_size_one) {
        start_point.set_x(start_lane_curve_right_point0.x());
        start_point.set_y(start_lane_curve_right_point0.y());
        start_point.set_z(start_lane_curve_right_point0.z());
      }
    }
    Eigen::Vector3d point_enu_start;
    if (utm) {
      point_enu_start = ConvertPoint(start_point, enupos);
    } else {
      point_enu_start =
          Eigen::Vector3d(start_point.x(), start_point.y(), start_point.z());
    }
    if (lane.left_neighbor_forward_lane_id_size()) {
      // 获取右侧相邻的lane ID
      for (const auto& neighbor_id : lane.left_neighbor_forward_lane_id()) {
        const std::string& neighbor_lane_id = neighbor_id.id();
        // 遍历所有lane，找到匹配的相邻lane
        for (const auto& target_lane : prior_map->lane()) {
          if (target_lane.id().id() == neighbor_lane_id) {
            // 确定箭头终点位置
            const auto lane_curve_left = target_lane.left_boundary().curve();
            const auto lane_curve_right = target_lane.right_boundary().curve();

            if (lane_curve_left.segment().empty() ||
                lane_curve_right.segment().empty() ||
                lane_curve_left.segment()[0].line_segment().point().empty() ||
                lane_curve_right.segment()[0].line_segment().point().empty()) {
              continue;
            }

            hozon::common::PointENU end_point;
            if (lane_curve_left.segment_size() > 0 &&
                lane_curve_right.segment_size() > 0) {
              const auto lane_curve_left_point0 =
                  lane_curve_left.segment()[0].line_segment().point()[0];
              const auto lane_curve_right_point0 =
                  lane_curve_right.segment()[0].line_segment().point()[0];
              end_point.set_x(
                  (lane_curve_left_point0.x() + lane_curve_right_point0.x()) /
                  2);
              end_point.set_y(
                  (lane_curve_left_point0.y() + lane_curve_right_point0.y()) /
                  2);
              end_point.set_z(
                  (lane_curve_left_point0.z() + lane_curve_right_point0.z()) /
                  2);
            } else if (lane_curve_left.segment_size() > 0 &&
                       lane_curve_right.segment_size() == 0) {
              const auto lane_curve_left_point0 =
                  lane_curve_left.segment()[0].line_segment().point()[0];
              bool all_points_size_one = true;
              for (const auto& segment : lane_curve_left.segment()) {
                if (segment.line_segment().point_size() > 1) {
                  const auto& start_segment = segment.line_segment();
                  const auto& end_point_0 = start_segment.point(0);
                  const auto& end_point_1 = start_segment.point(1);
                  // 计算向量长度和方向
                  double dx = end_point_1.x() - end_point_0.x();
                  double dy = end_point_1.y() - end_point_0.y();
                  double length = std::sqrt(dx * dx + dy * dy);
                  double direction_x = dx / length;
                  double direction_y = dy / length;
                  // 计算在向量左侧的点坐标
                  double offset = 0.1;  // 偏移量，可以根据需求调整
                  double target_x =
                      end_point_0.x() - direction_y * offset * length;
                  double target_y =
                      end_point_0.y() + direction_x * offset * length;
                  end_point.set_x(target_x);
                  end_point.set_y(target_y);
                  all_points_size_one = false;
                  break;
                }
              }
              if (all_points_size_one) {
                end_point.set_x(lane_curve_left_point0.x());
                end_point.set_y(lane_curve_left_point0.y());
                end_point.set_z(lane_curve_left_point0.z());
              }
            } else if (lane_curve_left.segment_size() == 0 &&
                       lane_curve_right.segment_size() > 0) {
              const auto lane_curve_right_point0 =
                  lane_curve_right.segment()[0].line_segment().point()[0];
              bool all_points_size_one = true;
              for (const auto& segment : lane_curve_right.segment()) {
                if (segment.line_segment().point_size() > 1) {
                  const auto& start_segment = segment.line_segment();
                  const auto& end_point_0 = start_segment.point(0);
                  const auto& end_point_1 = start_segment.point(1);

                  // 计算向量长度和方向
                  double dx = end_point_1.x() - end_point_0.x();
                  double dy = end_point_1.y() - end_point_0.y();
                  double length = std::sqrt(dx * dx + dy * dy);
                  double direction_x = dx / length;
                  double direction_y = dy / length;

                  // 计算在向量右侧的点坐标
                  double offset = 0.1;  // 偏移量，可以根据需求调整
                  double target_x =
                      end_point_0.x() + direction_y * offset * length;
                  double target_y =
                      end_point_0.y() - direction_x * offset * length;

                  end_point.set_x(target_x);
                  end_point.set_y(target_y);
                  all_points_size_one = false;
                  break;
                }
              }
              if (all_points_size_one) {
                end_point.set_x(lane_curve_right_point0.x());
                end_point.set_y(lane_curve_right_point0.y());
                end_point.set_z(lane_curve_right_point0.z());
              }
            }
            Eigen::Vector3d point_enu_end;
            if (utm) {
              point_enu_end = ConvertPoint(end_point, enupos);
            } else {
              point_enu_end =
                  Eigen::Vector3d(end_point.x(), end_point.y(), end_point.z());
            }
            // 创建箭头Marker
            auto arrow_left_marker =
                std::make_unique<adsfi_proto::viz::Marker>();
            arrow_left_marker->clear_points();
            arrow_left_marker->mutable_header()->set_frameid("map");
            arrow_left_marker->set_ns("ns_left_neighbor_forward" +
                                      std::to_string(remainder));
            arrow_left_marker->set_id(laneId % 10000);
            arrow_left_marker->set_type(
                adsfi_proto::viz::MarkerType::LINE_STRIP);
            arrow_left_marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
            auto start_point_marker = arrow_left_marker->add_points();
            start_point_marker->set_x(point_enu_start.x());
            start_point_marker->set_y(point_enu_start.y());
            start_point_marker->set_z(point_enu_start.z() - 3);

            // 添加终点
            auto end_point_marker = arrow_left_marker->add_points();
            end_point_marker->set_x(point_enu_end.x());
            end_point_marker->set_y(point_enu_end.y());
            end_point_marker->set_z(point_enu_end.z() - 3);

            // 设置颜色和尺寸
            arrow_left_marker->mutable_scale()->set_x(0.2);  // 设置线段的宽度
            arrow_left_marker->mutable_scale()->set_y(0.2);  // 设置线段的高度
            arrow_left_marker->mutable_scale()->set_z(0.2);  // 设置线段的高度
            arrow_left_marker->mutable_lifetime()->set_sec(0);
            arrow_left_marker->mutable_lifetime()->set_nsec(0);
            arrow_left_marker->mutable_pose()->mutable_orientation()->set_x(0.);
            arrow_left_marker->mutable_pose()->mutable_orientation()->set_y(0.);
            arrow_left_marker->mutable_pose()->mutable_orientation()->set_z(0.);
            arrow_left_marker->mutable_pose()->mutable_orientation()->set_w(1.);
            arrow_left_marker->mutable_lifetime()->set_sec(0);
            arrow_left_marker->mutable_lifetime()->set_nsec(0);
            adsfi_proto::viz::ColorRGBA color;
            color.set_a(1);
            color.set_r(1);  // 增加颜色分量变化幅度
            color.set_g(1);  // 增加颜色分量变化幅度
            color.set_b(1);  // 增加颜色分量变化幅度
            arrow_left_marker->mutable_color()->CopyFrom(color);
            auto sphere_marker = std::make_unique<adsfi_proto::viz::Marker>();
            sphere_marker->mutable_header()->set_frameid("map");
            sphere_marker->set_ns("ns_left_neighbor_forward_sphere" +
                                  std::to_string(remainder));
            sphere_marker->set_id(laneId % 10000);

            sphere_marker->set_type(adsfi_proto::viz::MarkerType::CUBE);
            sphere_marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
            sphere_marker->mutable_pose()->mutable_position()->set_x(
                point_enu_end.x());
            sphere_marker->mutable_pose()->mutable_position()->set_y(
                point_enu_end.y());
            sphere_marker->mutable_pose()->mutable_position()->set_z(
                point_enu_end.z() - 3);

            // 设置颜色和尺寸
            sphere_marker->mutable_scale()->set_x(0.3);  // 设置球体直径
            sphere_marker->mutable_scale()->set_y(0.3);
            sphere_marker->mutable_scale()->set_z(0.3);
            sphere_marker->mutable_pose()->mutable_orientation()->set_x(0.);
            sphere_marker->mutable_pose()->mutable_orientation()->set_y(0.);
            sphere_marker->mutable_pose()->mutable_orientation()->set_z(0.);
            sphere_marker->mutable_pose()->mutable_orientation()->set_w(1.);
            sphere_marker->mutable_lifetime()->set_sec(0);
            sphere_marker->mutable_lifetime()->set_nsec(0);
            adsfi_proto::viz::ColorRGBA colors;
            colors.set_a(1);
            colors.set_r(1);  // 增加颜色分量变化幅度
            colors.set_g(1);  // 增加颜色分量变化幅度
            colors.set_b(0);  // 增加颜色分量变化幅度
            // 设置颜色和尺寸
            sphere_marker->mutable_color()->CopyFrom(colors);
            arrow_markers.add_markers()->CopyFrom(*arrow_left_marker);
            arrow_markers.add_markers()->CopyFrom(*sphere_marker);
            break;
          }
        }
      }
    }
  }
  return arrow_markers;
}

adsfi_proto::viz::MarkerArray MapProtoMarker::LanePredecessor(
    const std::shared_ptr<hozon::hdmap::Map>& prior_map,
    const Eigen::Vector3d& enupos, bool utm) {
  adsfi_proto::viz::MarkerArray arrow_markers;
  for (const auto& lane : prior_map->lane()) {
    u_int64_t laneId = std::stoll(lane.id().id());
    int remainder = laneId / 10000;
    const auto lane_left_curve = lane.left_boundary().curve();
    uint32_t seg_size_left = lane_left_curve.segment().size();
    // right
    const auto lane_right_curve = lane.right_boundary().curve();
    uint32_t seg_size = lane_right_curve.segment().size();

    hozon::common::PointENU start_point_a;
    hozon::common::PointENU start_point_b;
    hozon::common::PointENU start_point;
    if (seg_size_left > 0 && seg_size > 0) {
      uint32_t point_size_left = lane_left_curve.segment()[seg_size_left - 1]
                                     .line_segment()
                                     .point()
                                     .size();
      const auto lane_left_curve_point0 =
          lane_left_curve.segment()[0].line_segment().point()[0];
      const auto lane_left_curve_point1 =
          lane_left_curve.segment()[seg_size_left - 1]
              .line_segment()
              .point()[point_size_left - 1];

      uint32_t point_size = lane_right_curve.segment()[seg_size - 1]
                                .line_segment()
                                .point()
                                .size();
      const auto lane_right_curve_point0 =
          lane_right_curve.segment()[0].line_segment().point()[0];
      const auto lane_right_curve_point1 =
          lane_right_curve.segment()[seg_size - 1]
              .line_segment()
              .point()[point_size - 1];
      start_point_a.set_x(
          (lane_left_curve_point0.x() + lane_right_curve_point0.x()) / 2);
      start_point_a.set_y(
          (lane_left_curve_point0.y() + lane_right_curve_point0.y()) / 2);
      start_point_a.set_z(
          (lane_left_curve_point0.z() + lane_right_curve_point0.z()) / 2);
      start_point_b.set_x(
          (lane_left_curve_point1.x() + lane_right_curve_point1.x()) / 2);
      start_point_b.set_y(
          (lane_left_curve_point1.y() + lane_right_curve_point1.y()) / 2);
      start_point_b.set_z(
          (lane_left_curve_point1.z() + lane_right_curve_point1.z()) / 2);
      Eigen::Vector3d start_point_a_enu;
      if (utm) {
        start_point_a_enu = ConvertPoint(start_point_a, enupos);
      } else {
        start_point_a_enu = Eigen::Vector3d(
            start_point_a.x(), start_point_a.y(), start_point_a.z());
      }
      Eigen::Vector3d start_point_b_enu;
      if (utm) {
        start_point_b_enu = ConvertPoint(start_point_b, enupos);
      } else {
        start_point_b_enu = Eigen::Vector3d(
            start_point_b.x(), start_point_b.y(), start_point_b.z());
      }
      auto vec_ab = hozon::common::PointENU();
      vec_ab.set_x(start_point_b_enu.x() - start_point_a_enu.x());
      vec_ab.set_y(start_point_b_enu.y() - start_point_a_enu.y());
      vec_ab.set_z(start_point_b_enu.z() - start_point_a_enu.z());
      double norm_ab = sqrt(pow(vec_ab.x(), 2) + pow(vec_ab.y(), 2) +
                            pow(vec_ab.z(), 2));  // 向量的模
      if (norm_ab > 0) {
        vec_ab.set_x(vec_ab.x() / norm_ab);
        vec_ab.set_y(vec_ab.y() / norm_ab);
        vec_ab.set_z(vec_ab.z() / norm_ab);
      }
      const double kDistance = 1.0;  // 在向量上所需移动的距离
      start_point.set_x(start_point_a_enu.x() + kDistance * vec_ab.x());
      start_point.set_y(start_point_a_enu.y() + kDistance * vec_ab.y());
      start_point.set_z(start_point_a_enu.z() + kDistance * vec_ab.z());
    } else if (seg_size_left > 0 && seg_size == 0) {
      uint32_t point_size_left = lane_left_curve.segment()[seg_size_left - 1]
                                     .line_segment()
                                     .point()
                                     .size();

      const auto lane_left_curve_point0 =
          lane_left_curve.segment()[0].line_segment().point()[0];
      if (point_size_left > 1) {
        const auto lane_left_curve_point1 =
            lane_left_curve.segment()[seg_size_left - 1]
                .line_segment()
                .point()[point_size_left - 1];
        // 计算向量长度
        auto vec_ab = hozon::common::PointENU();
        vec_ab.set_x(lane_left_curve_point1.x() - lane_left_curve_point0.x());
        vec_ab.set_y(lane_left_curve_point1.y() - lane_left_curve_point0.y());
        vec_ab.set_z(lane_left_curve_point1.z() - lane_left_curve_point0.z());
        double norm_ab = sqrt(pow(vec_ab.x(), 2) + pow(vec_ab.y(), 2) +
                              pow(vec_ab.z(), 2));  // 向量的模
        if (norm_ab > 0) {
          vec_ab.set_x(vec_ab.x() / norm_ab);
          vec_ab.set_y(vec_ab.y() / norm_ab);
          vec_ab.set_z(vec_ab.z() / norm_ab);
        }
        const double kDistance = 1.0;  // 在向量上所需移动的距离
        start_point.set_x(lane_left_curve_point0.x() + kDistance * vec_ab.x());
        start_point.set_y(lane_left_curve_point0.y() + kDistance * vec_ab.y());
        start_point.set_z(lane_left_curve_point0.z() + kDistance * vec_ab.z());
        // 计算垂直单位向量vec_vertical_unit
        auto vec_vertical_unit = hozon::common::PointENU();
        vec_vertical_unit.set_x(-vec_ab.y());
        vec_vertical_unit.set_y(vec_ab.x());
        vec_vertical_unit.set_z(vec_ab.z());
        // 定义平移距离kShiftDistance
        double kShiftDistance = 1.0;  // 根据实际需要设定平移距离
        // 计算平移向量shift_vector 和新点
        start_point.set_x(start_point.x() +
                          vec_vertical_unit.x() * kShiftDistance);
        start_point.set_y(start_point.y() +
                          vec_vertical_unit.y() * kShiftDistance);
        start_point.set_y(start_point.z() +
                          vec_vertical_unit.z() * kShiftDistance);
      } else {
        start_point.set_x(lane_left_curve_point0.x());
        start_point.set_y(lane_left_curve_point0.y());
        start_point.set_y(lane_left_curve_point0.z());
      }
    } else if (seg_size_left == 0 && seg_size > 0) {
      uint32_t point_size = lane_right_curve.segment()[seg_size - 1]
                                .line_segment()
                                .point()
                                .size();
      const auto lane_right_curve_point0 =
          lane_right_curve.segment()[0].line_segment().point()[0];
      if (point_size > 1) {
        const auto lane_right_curve_point1 =
            lane_right_curve.segment()[seg_size - 1]
                .line_segment()
                .point()[point_size - 1];
        // 计算向量长度
        auto vec_ab = hozon::common::PointENU();
        vec_ab.set_x(lane_right_curve_point1.x() - lane_right_curve_point0.x());
        vec_ab.set_y(lane_right_curve_point1.y() - lane_right_curve_point0.y());
        vec_ab.set_z(lane_right_curve_point1.z() - lane_right_curve_point0.z());
        double norm_ab = sqrt(pow(vec_ab.x(), 2) + pow(vec_ab.y(), 2) +
                              pow(vec_ab.z(), 2));  // 向量的模
        if (norm_ab > 0) {
          vec_ab.set_x(vec_ab.x() / norm_ab);
          vec_ab.set_y(vec_ab.y() / norm_ab);
          vec_ab.set_z(vec_ab.z() / norm_ab);
        }
        const double kDistance = 1.0;  // 在向量上所需移动的距离
        start_point.set_x(lane_right_curve_point0.x() + kDistance * vec_ab.x());
        start_point.set_y(lane_right_curve_point0.y() + kDistance * vec_ab.y());
        start_point.set_z(lane_right_curve_point0.z() + kDistance * vec_ab.z());
        // 计算垂直单位向量vec_vertical_unit
        auto vec_vertical_unit = hozon::common::PointENU();
        vec_vertical_unit.set_x(-vec_ab.y());
        vec_vertical_unit.set_y(vec_ab.x());
        vec_vertical_unit.set_z(vec_ab.z());
        // 定义平移距离kShiftDistance
        double kShiftDistance = 1.0;  // 根据实际需要设定平移距离
        // 计算平移向量shift_vector 和新点
        start_point.set_x(start_point.x() -
                          vec_vertical_unit.x() * kShiftDistance);
        start_point.set_y(start_point.y() -
                          vec_vertical_unit.y() * kShiftDistance);
        start_point.set_y(start_point.z() -
                          vec_vertical_unit.z() * kShiftDistance);
      } else {
        start_point.set_x(lane_right_curve_point0.x());
        start_point.set_y(lane_right_curve_point0.y());
        start_point.set_y(lane_right_curve_point0.z());
      }
    }
    if (lane.predecessor_id_size()) {
      int s = 0;
      for (const auto& neighbor_id : lane.predecessor_id()) {
        const std::string& neighbor_lane_id = neighbor_id.id();
        for (const auto& target_lane : prior_map->lane()) {
          if (target_lane.id().id() == neighbor_lane_id) {
            hozon::common::PointENU end_point_a;
            hozon::common::PointENU end_point_b;
            hozon::common::PointENU end_point;
            const auto target_lane_left_curve =
                target_lane.left_boundary().curve();
            uint32_t seg_size_left = target_lane_left_curve.segment().size();

            // right
            const auto target_lane_right_curve =
                target_lane.right_boundary().curve();
            uint32_t seg_size = target_lane_right_curve.segment().size();
            if (seg_size_left > 0 && seg_size > 0) {
              uint32_t point_size_left =
                  target_lane_left_curve.segment()[seg_size_left - 1]
                      .line_segment()
                      .point()
                      .size();
              const auto target_lane_left_curve_point0 =
                  target_lane_left_curve.segment()[0].line_segment().point()[0];
              const auto target_lane_left_curve_point1 =
                  target_lane_left_curve.segment()[seg_size_left - 1]
                      .line_segment()
                      .point()[point_size_left - 1];
              uint32_t point_size =
                  target_lane_right_curve.segment()[seg_size - 1]
                      .line_segment()
                      .point()
                      .size();
              const auto target_lane_right_curve_point0 =
                  target_lane_right_curve.segment()[0]
                      .line_segment()
                      .point()[0];
              const auto target_lane_right_curve_point1 =
                  target_lane_right_curve.segment()[seg_size - 1]
                      .line_segment()
                      .point()[point_size - 1];
              end_point_a.set_x((target_lane_left_curve_point0.x() +
                                 target_lane_right_curve_point0.x()) /
                                2);
              end_point_a.set_y((target_lane_left_curve_point0.y() +
                                 target_lane_right_curve_point0.y()) /
                                2);
              end_point_a.set_z((target_lane_left_curve_point0.z() +
                                 target_lane_right_curve_point0.z()) /
                                2);
              end_point_b.set_x((target_lane_left_curve_point1.x() +
                                 target_lane_right_curve_point1.x()) /
                                2);
              end_point_b.set_y((target_lane_left_curve_point1.y() +
                                 target_lane_right_curve_point1.y()) /
                                2);
              end_point_b.set_z((target_lane_left_curve_point1.z() +
                                 target_lane_right_curve_point1.z()) /
                                2);
              Eigen::Vector3d end_point_a_enu;
              if (utm) {
                end_point_a_enu = ConvertPoint(end_point_a, enupos);
              } else {
                end_point_a_enu = Eigen::Vector3d(
                    end_point_a.x(), end_point_a.y(), end_point_a.z());
              }
              Eigen::Vector3d end_point_b_enu;
              if (utm) {
                end_point_b_enu = ConvertPoint(end_point_b, enupos);
              } else {
                end_point_b_enu = Eigen::Vector3d(
                    end_point_b.x(), end_point_b.y(), end_point_b.z());
              }

              auto vec_ab = hozon::common::PointENU();
              vec_ab.set_x(end_point_b_enu.x() - end_point_a_enu.x());
              vec_ab.set_y(end_point_b_enu.y() - end_point_a_enu.y());
              vec_ab.set_z(end_point_b_enu.z() - end_point_a_enu.z());
              double norm_ab = sqrt(pow(vec_ab.x(), 2) + pow(vec_ab.y(), 2) +
                                    pow(vec_ab.z(), 2));  // 向量的模
              if (norm_ab > 0) {
                vec_ab.set_x(vec_ab.x() / norm_ab);
                vec_ab.set_y(vec_ab.y() / norm_ab);
                vec_ab.set_z(vec_ab.z() / norm_ab);
              }
              const double kDistance = 1.0;  // 在向量上所需移动的距离
              end_point.set_x(end_point_b_enu.x() - kDistance * vec_ab.x());
              end_point.set_y(end_point_b_enu.y() - kDistance * vec_ab.y());
              end_point.set_z(end_point_b_enu.z() - kDistance * vec_ab.z());
            } else if (seg_size_left > 0 && seg_size == 0) {
              uint32_t point_size_left =
                  target_lane_left_curve.segment()[seg_size_left - 1]
                      .line_segment()
                      .point()
                      .size();
              const auto target_lane_left_curve_point0 =
                  target_lane_left_curve.segment()[0].line_segment().point()[0];
              if (point_size_left > 1) {
                const auto target_lane_left_curve_point1 =
                    target_lane_left_curve.segment()[seg_size_left - 1]
                        .line_segment()
                        .point()[point_size_left - 1];
                // 计算向量长度
                auto vec_ab = hozon::common::PointENU();
                vec_ab.set_x(target_lane_left_curve_point1.x() -
                             target_lane_left_curve_point0.x());
                vec_ab.set_y(target_lane_left_curve_point1.y() -
                             target_lane_left_curve_point0.y());
                vec_ab.set_z(target_lane_left_curve_point1.z() -
                             target_lane_left_curve_point0.z());
                double norm_ab = sqrt(pow(vec_ab.x(), 2) + pow(vec_ab.y(), 2) +
                                      pow(vec_ab.z(), 2));  // 向量的模
                if (norm_ab > 0) {
                  vec_ab.set_x(vec_ab.x() / norm_ab);
                  vec_ab.set_y(vec_ab.y() / norm_ab);
                  vec_ab.set_z(vec_ab.z() / norm_ab);
                }
                const double kDistance = 1.0;  // 在向量上所需移动的距离
                end_point.set_x(target_lane_left_curve_point1.x() -
                                kDistance * vec_ab.x());
                end_point.set_y(target_lane_left_curve_point1.y() -
                                kDistance * vec_ab.y());
                end_point.set_z(target_lane_left_curve_point1.z() -
                                kDistance * vec_ab.z());
                // 计算垂直单位向量vec_vertical_unit
                auto vec_vertical_unit = hozon::common::PointENU();
                vec_vertical_unit.set_x(-vec_ab.y());
                vec_vertical_unit.set_y(vec_ab.x());
                vec_vertical_unit.set_z(vec_ab.z());
                // 定义平移距离kShiftDistance
                double kShiftDistance = 1.0;  // 根据实际需要设定平移距离
                // 计算平移向量shift_vector 和新点
                end_point.set_x(end_point.x() +
                                vec_vertical_unit.x() * kShiftDistance);
                end_point.set_y(end_point.y() +
                                vec_vertical_unit.y() * kShiftDistance);
                end_point.set_y(end_point.z() +
                                vec_vertical_unit.z() * kShiftDistance);
              } else {
                end_point.set_x(target_lane_left_curve_point0.x());
                end_point.set_y(target_lane_left_curve_point0.y());
                end_point.set_y(target_lane_left_curve_point0.z());
              }
            } else if (seg_size_left == 0 && seg_size > 0) {
              uint32_t point_size =
                  target_lane_right_curve.segment()[seg_size - 1]
                      .line_segment()
                      .point()
                      .size();
              const auto target_lane_right_curve_point0 =
                  target_lane_right_curve.segment()[0]
                      .line_segment()
                      .point()[0];
              if (point_size > 1) {
                const auto target_lane_right_curve_point1 =
                    target_lane_right_curve.segment()[seg_size - 1]
                        .line_segment()
                        .point()[point_size - 1];
                // 计算向量长度
                auto vec_ab = hozon::common::PointENU();
                vec_ab.set_x(target_lane_right_curve_point1.x() -
                             target_lane_right_curve_point0.x());
                vec_ab.set_y(target_lane_right_curve_point1.y() -
                             target_lane_right_curve_point0.y());
                vec_ab.set_z(target_lane_right_curve_point1.z() -
                             target_lane_right_curve_point0.z());
                double norm_ab = sqrt(pow(vec_ab.x(), 2) + pow(vec_ab.y(), 2) +
                                      pow(vec_ab.z(), 2));  // 向量的模
                if (norm_ab > 0) {
                  vec_ab.set_x(vec_ab.x() / norm_ab);
                  vec_ab.set_y(vec_ab.y() / norm_ab);
                  vec_ab.set_z(vec_ab.z() / norm_ab);
                }
                const double kDistance = 1.0;  // 在向量上所需移动的距离
                end_point.set_x(target_lane_right_curve_point1.x() -
                                kDistance * vec_ab.x());
                end_point.set_y(target_lane_right_curve_point1.y() -
                                kDistance * vec_ab.y());
                end_point.set_z(target_lane_right_curve_point1.z() -
                                kDistance * vec_ab.z());
                // 计算垂直单位向量vec_vertical_unit
                auto vec_vertical_unit = hozon::common::PointENU();
                vec_vertical_unit.set_x(-vec_ab.y());
                vec_vertical_unit.set_y(vec_ab.x());
                vec_vertical_unit.set_z(vec_ab.z());
                // 定义平移距离kShiftDistance
                double kShiftDistance = 1.0;  // 根据实际需要设定平移距离
                // 计算平移向量shift_vector 和新点
                end_point.set_x(end_point.x() -
                                vec_vertical_unit.x() * kShiftDistance);
                end_point.set_y(end_point.y() -
                                vec_vertical_unit.y() * kShiftDistance);
                end_point.set_y(end_point.z() -
                                vec_vertical_unit.z() * kShiftDistance);
              } else {
                end_point.set_x(target_lane_right_curve_point0.x());
                end_point.set_y(target_lane_right_curve_point0.y());
                end_point.set_y(target_lane_right_curve_point0.z());
              }
            }
            // 创建箭头Marker
            auto arrow_predecessor_marker =
                std::make_unique<adsfi_proto::viz::Marker>();
            arrow_predecessor_marker->mutable_header()->set_frameid("map");
            arrow_predecessor_marker->set_ns("ns_prior_map_arrow_predecessor" +
                                             std::to_string(remainder) +
                                             std::to_string(s));
            arrow_predecessor_marker->set_id(laneId % 10000);
            // arrow_predecessor_marker->set_type(
            //     adsfi_proto::viz::MarkerType::LINE_STRIP);
            arrow_predecessor_marker->set_type(
                adsfi_proto::viz::MarkerType::ARROW);

            arrow_predecessor_marker->set_action(
                adsfi_proto::viz::MarkerAction::ADD);
            auto start_point_marker = arrow_predecessor_marker->add_points();
            start_point_marker->set_x(start_point.x());
            start_point_marker->set_y(start_point.y());
            start_point_marker->set_z(start_point.z());

            // 添加终点
            auto end_point_marker = arrow_predecessor_marker->add_points();
            end_point_marker->set_x(end_point.x());
            end_point_marker->set_y(end_point.y());
            end_point_marker->set_z(end_point.z());

            // 设置颜色和尺寸
            arrow_predecessor_marker->mutable_scale()->set_x(
                0.2);  // 设置线段的宽度
            arrow_predecessor_marker->mutable_scale()->set_y(
                0.2);  // 设置线段的高度
            arrow_predecessor_marker->mutable_scale()->set_z(
                0.2);  // 设置线段的高度
            arrow_predecessor_marker->mutable_lifetime()->set_sec(0);
            arrow_predecessor_marker->mutable_lifetime()->set_nsec(0);
            arrow_predecessor_marker->mutable_pose()
                ->mutable_orientation()
                ->set_x(0.);
            arrow_predecessor_marker->mutable_pose()
                ->mutable_orientation()
                ->set_y(0.);
            arrow_predecessor_marker->mutable_pose()
                ->mutable_orientation()
                ->set_z(0.);
            arrow_predecessor_marker->mutable_pose()
                ->mutable_orientation()
                ->set_w(1.);
            arrow_predecessor_marker->mutable_lifetime()->set_sec(0);
            arrow_predecessor_marker->mutable_lifetime()->set_nsec(0);
            adsfi_proto::viz::ColorRGBA color;
            color.set_a(1);
            color.set_r(0);
            color.set_g(1);
            color.set_b(0);
            arrow_predecessor_marker->mutable_color()->CopyFrom(color);

            auto sphere_marker = std::make_unique<adsfi_proto::viz::Marker>();
            sphere_marker->mutable_header()->set_frameid("map");
            sphere_marker->set_ns("ns_prior_map_arrow_predecessor_sphere" +
                                  std::to_string(remainder) +
                                  std::to_string(s));
            sphere_marker->set_id(laneId % 10000);

            sphere_marker->set_type(adsfi_proto::viz::MarkerType::SPHERE);
            sphere_marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
            sphere_marker->mutable_pose()->mutable_position()->set_x(
                end_point.x());
            sphere_marker->mutable_pose()->mutable_position()->set_y(
                end_point.y());
            sphere_marker->mutable_pose()->mutable_position()->set_z(
                end_point.z());

            // 设置颜色和尺寸
            sphere_marker->mutable_scale()->set_x(0.4);  // 设置球体直径
            sphere_marker->mutable_scale()->set_y(0.4);
            sphere_marker->mutable_scale()->set_z(0.4);
            sphere_marker->mutable_pose()->mutable_orientation()->set_x(0.);
            sphere_marker->mutable_pose()->mutable_orientation()->set_y(0.);
            sphere_marker->mutable_pose()->mutable_orientation()->set_z(0.);
            sphere_marker->mutable_pose()->mutable_orientation()->set_w(1.);
            sphere_marker->mutable_lifetime()->set_sec(0);
            sphere_marker->mutable_lifetime()->set_nsec(0);
            adsfi_proto::viz::ColorRGBA colors;
            colors.set_a(1);
            colors.set_r(1);
            colors.set_g(0);
            colors.set_b(0);
            // 设置颜色和尺寸
            sphere_marker->mutable_color()->CopyFrom(colors);
            arrow_markers.add_markers()->CopyFrom(*arrow_predecessor_marker);
            arrow_markers.add_markers()->CopyFrom(*sphere_marker);
            break;
          }
        }
        s += 1;
      }
    }
  }
  return arrow_markers;
}
adsfi_proto::viz::MarkerArray MapProtoMarker::LaneSuccessor(
    const std::shared_ptr<hozon::hdmap::Map>& prior_map,
    const Eigen::Vector3d& enupos, bool utm) {
  adsfi_proto::viz::MarkerArray arrow_markers;
  for (const auto& lane : prior_map->lane()) {
    u_int64_t laneId = std::stoll(lane.id().id());
    int remainder = laneId / 10000;
    const auto lane_left_curve = lane.left_boundary().curve();
    uint32_t seg_size_left = lane_left_curve.segment().size();
    // right
    const auto lane_right_curve = lane.right_boundary().curve();
    uint32_t seg_size = lane_right_curve.segment().size();
    hozon::common::PointENU start_point_a;
    hozon::common::PointENU start_point_b;
    hozon::common::PointENU start_point;
    if (seg_size_left > 0 && seg_size > 0) {
      uint32_t point_size_left = lane_left_curve.segment()[seg_size_left - 1]
                                     .line_segment()
                                     .point()
                                     .size();
      const auto lane_left_curve_point0 =
          lane_left_curve.segment()[0].line_segment().point()[0];
      const auto lane_left_curve_point1 =
          lane_left_curve.segment()[seg_size_left - 1]
              .line_segment()
              .point()[point_size_left - 1];

      uint32_t point_size = lane_right_curve.segment()[seg_size - 1]
                                .line_segment()
                                .point()
                                .size();
      const auto lane_right_curve_point0 =
          lane_right_curve.segment()[0].line_segment().point()[0];
      const auto lane_right_curve_point1 =
          lane_right_curve.segment()[seg_size - 1]
              .line_segment()
              .point()[point_size - 1];
      start_point_a.set_x(
          (lane_left_curve_point0.x() + lane_right_curve_point0.x()) / 2);
      start_point_a.set_y(
          (lane_left_curve_point0.y() + lane_right_curve_point0.y()) / 2);
      start_point_a.set_z(
          (lane_left_curve_point0.z() + lane_right_curve_point0.z()) / 2);
      start_point_b.set_x(
          (lane_left_curve_point1.x() + lane_right_curve_point1.x()) / 2);
      start_point_b.set_y(
          (lane_left_curve_point1.y() + lane_right_curve_point1.y()) / 2);
      start_point_b.set_z(
          (lane_left_curve_point1.z() + lane_right_curve_point1.z()) / 2);
      Eigen::Vector3d start_point_a_enu;
      if (utm) {
        start_point_a_enu = ConvertPoint(start_point_a, enupos);
      } else {
        start_point_a_enu = Eigen::Vector3d(
            start_point_a.x(), start_point_a.y(), start_point_a.z());
      }
      Eigen::Vector3d start_point_b_enu;
      if (utm) {
        start_point_b_enu = ConvertPoint(start_point_b, enupos);
      } else {
        start_point_b_enu = Eigen::Vector3d(
            start_point_b.x(), start_point_b.y(), start_point_b.z());
      }
      auto vec_ab = hozon::common::PointENU();
      vec_ab.set_x(start_point_b_enu.x() - start_point_a_enu.x());
      vec_ab.set_y(start_point_b_enu.y() - start_point_a_enu.y());
      vec_ab.set_z(start_point_b_enu.z() - start_point_a_enu.z());
      double norm_ab = sqrt(pow(vec_ab.x(), 2) + pow(vec_ab.y(), 2) +
                            pow(vec_ab.z(), 2));  // 向量的模
      if (norm_ab > 0) {
        vec_ab.set_x(vec_ab.x() / norm_ab);
        vec_ab.set_y(vec_ab.y() / norm_ab);
        vec_ab.set_z(vec_ab.z() / norm_ab);
      }
      const double kDistance = 1.0;  // 在向量上所需移动的距离
      start_point.set_x(start_point_b_enu.x() - kDistance * vec_ab.x());
      start_point.set_y(start_point_b_enu.y() - kDistance * vec_ab.y());
      start_point.set_z(start_point_b_enu.z() - kDistance * vec_ab.z());
    } else if (seg_size_left > 0 && seg_size == 0) {
      uint32_t point_size_left = lane_left_curve.segment()[seg_size_left - 1]
                                     .line_segment()
                                     .point()
                                     .size();

      const auto lane_left_curve_point0 =
          lane_left_curve.segment()[0].line_segment().point()[0];
      if (point_size_left > 1) {
        const auto lane_left_curve_point1 =
            lane_left_curve.segment()[seg_size_left - 1]
                .line_segment()
                .point()[point_size_left - 1];
        // 计算向量长度
        auto vec_ab = hozon::common::PointENU();
        vec_ab.set_x(lane_left_curve_point1.x() - lane_left_curve_point0.x());
        vec_ab.set_y(lane_left_curve_point1.y() - lane_left_curve_point0.y());
        vec_ab.set_z(lane_left_curve_point1.z() - lane_left_curve_point0.z());
        double norm_ab = sqrt(pow(vec_ab.x(), 2) + pow(vec_ab.y(), 2) +
                              pow(vec_ab.z(), 2));  // 向量的模
        if (norm_ab > 0) {
          vec_ab.set_x(vec_ab.x() / norm_ab);
          vec_ab.set_y(vec_ab.y() / norm_ab);
          vec_ab.set_z(vec_ab.z() / norm_ab);
        }
        const double kDistance = 1.0;  // 在向量上所需移动的距离
        start_point.set_x(lane_left_curve_point1.x() - kDistance * vec_ab.x());
        start_point.set_y(lane_left_curve_point1.y() - kDistance * vec_ab.y());
        start_point.set_z(lane_left_curve_point1.z() - kDistance * vec_ab.z());
        // 计算垂直单位向量vec_vertical_unit
        auto vec_vertical_unit = hozon::common::PointENU();
        vec_vertical_unit.set_x(-vec_ab.y());
        vec_vertical_unit.set_y(vec_ab.x());
        vec_vertical_unit.set_z(vec_ab.z());
        // 定义平移距离kShiftDistance
        double kShiftDistance = 1.0;  // 根据实际需要设定平移距离
        // 计算平移向量shift_vector 和新点
        start_point.set_x(start_point.x() +
                          vec_vertical_unit.x() * kShiftDistance);
        start_point.set_y(start_point.y() +
                          vec_vertical_unit.y() * kShiftDistance);
        start_point.set_y(start_point.z() +
                          vec_vertical_unit.z() * kShiftDistance);
      } else {
        start_point.set_x(lane_left_curve_point0.x());
        start_point.set_y(lane_left_curve_point0.y());
        start_point.set_y(lane_left_curve_point0.z());
      }
    } else if (seg_size_left == 0 && seg_size > 0) {
      uint32_t point_size = lane_right_curve.segment()[seg_size - 1]
                                .line_segment()
                                .point()
                                .size();
      const auto lane_right_curve_point0 =
          lane_right_curve.segment()[0].line_segment().point()[0];
      if (point_size > 1) {
        const auto lane_right_curve_point1 =
            lane_right_curve.segment()[seg_size - 1]
                .line_segment()
                .point()[point_size - 1];
        // 计算向量长度
        auto vec_ab = hozon::common::PointENU();
        vec_ab.set_x(lane_right_curve_point1.x() - lane_right_curve_point0.x());
        vec_ab.set_y(lane_right_curve_point1.y() - lane_right_curve_point0.y());
        vec_ab.set_z(lane_right_curve_point1.z() - lane_right_curve_point0.z());
        double norm_ab = sqrt(pow(vec_ab.x(), 2) + pow(vec_ab.y(), 2) +
                              pow(vec_ab.z(), 2));  // 向量的模
        if (norm_ab > 0) {
          vec_ab.set_x(vec_ab.x() / norm_ab);
          vec_ab.set_y(vec_ab.y() / norm_ab);
          vec_ab.set_z(vec_ab.z() / norm_ab);
        }
        const double kDistance = 1.0;  // 在向量上所需移动的距离
        start_point.set_x(lane_right_curve_point1.x() - kDistance * vec_ab.x());
        start_point.set_y(lane_right_curve_point1.y() - kDistance * vec_ab.y());
        start_point.set_z(lane_right_curve_point1.z() - kDistance * vec_ab.z());
        // 计算垂直单位向量vec_vertical_unit
        auto vec_vertical_unit = hozon::common::PointENU();
        vec_vertical_unit.set_x(-vec_ab.y());
        vec_vertical_unit.set_y(vec_ab.x());
        vec_vertical_unit.set_z(vec_ab.z());
        // 定义平移距离kShiftDistance
        double kShiftDistance = 1.0;  // 根据实际需要设定平移距离
        // 计算平移向量shift_vector 和新点
        start_point.set_x(start_point.x() -
                          vec_vertical_unit.x() * kShiftDistance);
        start_point.set_y(start_point.y() -
                          vec_vertical_unit.y() * kShiftDistance);
        start_point.set_y(start_point.z() -
                          vec_vertical_unit.z() * kShiftDistance);
      } else {
        start_point.set_x(lane_right_curve_point0.x());
        start_point.set_y(lane_right_curve_point0.y());
        start_point.set_y(lane_right_curve_point0.z());
      }
    }

    if (lane.successor_id_size()) {
      int s = 0;
      for (const auto& neighbor_id : lane.successor_id()) {
        const std::string& neighbor_lane_id = neighbor_id.id();
        for (const auto& target_lane : prior_map->lane()) {
          if (target_lane.id().id() == neighbor_lane_id) {
            hozon::common::PointENU end_point_a;
            hozon::common::PointENU end_point_b;
            hozon::common::PointENU end_point;
            if (target_lane.left_boundary().curve().segment_size() > 0 &&
                target_lane.right_boundary().curve().segment_size() > 0) {
              uint32_t seg_size_left_target =
                  target_lane.left_boundary().curve().segment().size();
              uint32_t point_size_left_target =
                  target_lane.left_boundary()
                      .curve()
                      .segment()[seg_size_left_target - 1]
                      .line_segment()
                      .point()
                      .size();
              uint32_t seg_size_target =
                  target_lane.right_boundary().curve().segment().size();
              uint32_t point_size_target = target_lane.right_boundary()
                                               .curve()
                                               .segment()[seg_size_target - 1]
                                               .line_segment()
                                               .point()
                                               .size();
              end_point_a.set_x((target_lane.left_boundary()
                                     .curve()
                                     .segment()[0]
                                     .line_segment()
                                     .point()[0]
                                     .x() +
                                 target_lane.right_boundary()
                                     .curve()
                                     .segment()[0]
                                     .line_segment()
                                     .point()[0]
                                     .x()) /
                                2);
              end_point_a.set_y((target_lane.left_boundary()
                                     .curve()
                                     .segment()[0]
                                     .line_segment()
                                     .point()[0]
                                     .y() +
                                 target_lane.right_boundary()
                                     .curve()
                                     .segment()[0]
                                     .line_segment()
                                     .point()[0]
                                     .y()) /
                                2);
              end_point_a.set_z((target_lane.left_boundary()
                                     .curve()
                                     .segment()[0]
                                     .line_segment()
                                     .point()[0]
                                     .z() +
                                 target_lane.right_boundary()
                                     .curve()
                                     .segment()[0]
                                     .line_segment()
                                     .point()[0]
                                     .z()) /
                                2);

              end_point_b.set_x((target_lane.left_boundary()
                                     .curve()
                                     .segment()[seg_size_left_target - 1]
                                     .line_segment()
                                     .point()[point_size_left_target - 1]
                                     .x() +
                                 target_lane.right_boundary()
                                     .curve()
                                     .segment()[seg_size_target - 1]
                                     .line_segment()
                                     .point()[point_size_target - 1]
                                     .x()) /
                                2);
              end_point_b.set_y((target_lane.left_boundary()
                                     .curve()
                                     .segment()[seg_size_left_target - 1]
                                     .line_segment()
                                     .point()[point_size_left_target - 1]
                                     .y() +
                                 target_lane.right_boundary()
                                     .curve()
                                     .segment()[seg_size_target - 1]
                                     .line_segment()
                                     .point()[point_size_target - 1]
                                     .y()) /
                                2);
              end_point_b.set_z((target_lane.left_boundary()
                                     .curve()
                                     .segment()[seg_size_left_target - 1]
                                     .line_segment()
                                     .point()[point_size_left_target - 1]
                                     .z() +
                                 target_lane.right_boundary()
                                     .curve()
                                     .segment()[seg_size_target - 1]
                                     .line_segment()
                                     .point()[point_size_target - 1]
                                     .z()) /
                                2);
              Eigen::Vector3d end_point_a_enu;
              if (utm) {
                end_point_a_enu = ConvertPoint(end_point_a, enupos);
              } else {
                end_point_a_enu = Eigen::Vector3d(
                    end_point_a.x(), end_point_a.y(), end_point_a.z());
              }
              Eigen::Vector3d end_point_b_enu;
              if (utm) {
                end_point_b_enu = ConvertPoint(end_point_b, enupos);
              } else {
                end_point_b_enu = Eigen::Vector3d(
                    end_point_b.x(), end_point_b.y(), end_point_b.z());
              }
              auto vec_ab_end = hozon::common::PointENU();
              vec_ab_end.set_x(end_point_b_enu.x() - end_point_a_enu.x());
              vec_ab_end.set_y(end_point_b_enu.y() - end_point_a_enu.y());
              vec_ab_end.set_z(end_point_b_enu.z() - end_point_a_enu.z());
              double norm_ab_end =
                  sqrt(pow(vec_ab_end.x(), 2) + pow(vec_ab_end.y(), 2) +
                       pow(vec_ab_end.z(), 2));  // 向量的模
              if (norm_ab_end > 0) {
                vec_ab_end.set_x(vec_ab_end.x() / norm_ab_end);
                vec_ab_end.set_y(vec_ab_end.y() / norm_ab_end);
                vec_ab_end.set_z(vec_ab_end.z() / norm_ab_end);
              }
              const double kDistance = 1.0;  // 在向量上所需移动的距离
              end_point.set_x(end_point_a_enu.x() + kDistance * vec_ab_end.x());
              end_point.set_y(end_point_a_enu.y() + kDistance * vec_ab_end.y());
              end_point.set_z(end_point_a_enu.z() + kDistance * vec_ab_end.z());
            }

            // 创建箭头Marker
            auto arrow_predecessor_marker =
                std::make_unique<adsfi_proto::viz::Marker>();
            arrow_predecessor_marker->mutable_header()->set_frameid("map");
            arrow_predecessor_marker->set_ns("ns_prior_map_arrow_successor" +
                                             std::to_string(remainder) +
                                             std::to_string(s));
            arrow_predecessor_marker->set_id(laneId % 10000);
            // arrow_predecessor_marker->set_type(
            //     adsfi_proto::viz::MarkerType::LINE_STRIP);
            arrow_predecessor_marker->set_type(
                adsfi_proto::viz::MarkerType::ARROW);

            arrow_predecessor_marker->set_action(
                adsfi_proto::viz::MarkerAction::ADD);
            auto start_point_marker = arrow_predecessor_marker->add_points();
            start_point_marker->set_x(start_point.x());
            start_point_marker->set_y(start_point.y());
            start_point_marker->set_z(start_point.z() + 2);

            // 添加终点
            auto end_point_marker = arrow_predecessor_marker->add_points();
            end_point_marker->set_x(end_point.x());
            end_point_marker->set_y(end_point.y());
            end_point_marker->set_z(end_point.z() + 2);

            // 设置颜色和尺寸
            arrow_predecessor_marker->mutable_scale()->set_x(
                0.2);  // 设置线段的宽度
            arrow_predecessor_marker->mutable_scale()->set_y(
                0.2);  // 设置线段的高度
            arrow_predecessor_marker->mutable_scale()->set_z(
                0.2);  // 设置线段的高度
            arrow_predecessor_marker->mutable_lifetime()->set_sec(0);
            arrow_predecessor_marker->mutable_lifetime()->set_nsec(0);
            arrow_predecessor_marker->mutable_pose()
                ->mutable_orientation()
                ->set_x(0.);
            arrow_predecessor_marker->mutable_pose()
                ->mutable_orientation()
                ->set_y(0.);
            arrow_predecessor_marker->mutable_pose()
                ->mutable_orientation()
                ->set_z(0.);
            arrow_predecessor_marker->mutable_pose()
                ->mutable_orientation()
                ->set_w(1.);
            arrow_predecessor_marker->mutable_lifetime()->set_sec(0);
            arrow_predecessor_marker->mutable_lifetime()->set_nsec(0);
            adsfi_proto::viz::ColorRGBA color;
            color.set_a(1);
            color.set_r(0);
            color.set_g(0);
            color.set_b(1);
            arrow_predecessor_marker->mutable_color()->CopyFrom(color);

            auto sphere_marker = std::make_unique<adsfi_proto::viz::Marker>();
            sphere_marker->mutable_header()->set_frameid("map");
            sphere_marker->set_ns("ns_prior_map_arrow_successor_sphere" +
                                  std::to_string(remainder) +
                                  std::to_string(s));
            sphere_marker->set_id(laneId % 10000);

            sphere_marker->set_type(adsfi_proto::viz::MarkerType::SPHERE);
            sphere_marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
            sphere_marker->mutable_pose()->mutable_position()->set_x(
                end_point.x());
            sphere_marker->mutable_pose()->mutable_position()->set_y(
                end_point.y());
            sphere_marker->mutable_pose()->mutable_position()->set_z(
                end_point.z() + 2);

            // 设置颜色和尺寸
            sphere_marker->mutable_scale()->set_x(0.4);  // 设置球体直径
            sphere_marker->mutable_scale()->set_y(0.4);
            sphere_marker->mutable_scale()->set_z(0.4);
            sphere_marker->mutable_pose()->mutable_orientation()->set_x(0.);
            sphere_marker->mutable_pose()->mutable_orientation()->set_y(0.);
            sphere_marker->mutable_pose()->mutable_orientation()->set_z(0.);
            sphere_marker->mutable_pose()->mutable_orientation()->set_w(1.);
            sphere_marker->mutable_lifetime()->set_sec(0);
            sphere_marker->mutable_lifetime()->set_nsec(0);
            adsfi_proto::viz::ColorRGBA colors;
            colors.set_a(0.8);
            colors.set_r(255);
            colors.set_g(0);
            colors.set_b(255);
            // 设置颜色和尺寸
            sphere_marker->mutable_color()->CopyFrom(colors);
            arrow_markers.add_markers()->CopyFrom(*arrow_predecessor_marker);
            arrow_markers.add_markers()->CopyFrom(*sphere_marker);
            break;
          }
        }
        s += 1;
      }
    }
  }
  return arrow_markers;
}
adsfi_proto::viz::MarkerArray MapProtoMarker::JunctionToMarker(
    const std::shared_ptr<hozon::hdmap::Map>& prior_map,
    const Eigen::Vector3d& enupos, bool utm) {
  adsfi_proto::viz::MarkerArray junction_markers;
  for (const auto& junction : prior_map->junction()) {
    auto junction_marker = std::make_unique<adsfi_proto::viz::Marker>();

    junction_marker->mutable_header()->set_frameid("map");

    u_int64_t JunctionId = std::stoll(junction.id().id());
    int remainder = JunctionId / 10000;
    junction_marker->set_ns("ns_prior_map_junction" +
                            std::to_string(remainder));
    junction_marker->set_id(JunctionId % 10000);

    junction_marker->set_type(adsfi_proto::viz::MarkerType::SPHERE);
    junction_marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
    // 位姿
    junction_marker->mutable_pose()->mutable_position()->set_x(0);
    junction_marker->mutable_pose()->mutable_position()->set_y(0);
    junction_marker->mutable_pose()->mutable_position()->set_z(0);
    junction_marker->mutable_pose()->mutable_orientation()->set_x(0.);
    junction_marker->mutable_pose()->mutable_orientation()->set_y(0.);
    junction_marker->mutable_pose()->mutable_orientation()->set_z(0.);
    junction_marker->mutable_pose()->mutable_orientation()->set_w(1.);
    // 尺寸
    junction_marker->mutable_scale()->set_x(3);
    junction_marker->mutable_scale()->set_y(3);
    junction_marker->mutable_scale()->set_z(3);
    junction_marker->mutable_lifetime()->set_sec(0);
    junction_marker->mutable_lifetime()->set_nsec(0);
    adsfi_proto::viz::ColorRGBA color;
    color.set_a(1.0);
    color.set_r(0);
    color.set_g(0);
    color.set_b(0);
    junction_marker->mutable_color()->CopyFrom(color);
    for (const auto& point : junction.polygon().point()) {
      auto pt = junction_marker->add_points();
      pt->set_x(point.x());
      pt->set_y(point.y());
      pt->set_z(point.z());
    }
    if (junction_marker->points().empty()) {
      HLOG_WARN << "empty junction";
    }
    junction_markers.add_markers()->CopyFrom(*junction_marker);
  }
  return junction_markers;
}
adsfi_proto::viz::MarkerArray MapProtoMarker::RoadToMarker(
    const std::shared_ptr<hozon::hdmap::Map>& prior_map,
    const Eigen::Vector3d& enupos, bool utm) {
  adsfi_proto::viz::MarkerArray road_markers;
  for (const auto& road : prior_map->road()) {
    for (const auto& road_section : road.section()) {
      for (const auto& road_section_line :
           road_section.boundary().outer_polygon().edge()) {
        for (const auto& segment : road_section_line.curve().segment()) {
          auto road_marker = std::make_unique<adsfi_proto::viz::Marker>();
          road_marker->mutable_header()->set_frameid("map");

          u_int64_t RoadId = std::stoll(road.id().id());
          int remainder = RoadId / 10000;
          road_marker->set_ns("ns_prior_map_road" + std::to_string(remainder));
          road_marker->set_id(RoadId % 10000);

          road_marker->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
          road_marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
          // 位姿
          road_marker->mutable_pose()->mutable_position()->set_x(0);
          road_marker->mutable_pose()->mutable_position()->set_y(0);
          road_marker->mutable_pose()->mutable_position()->set_z(0);
          road_marker->mutable_pose()->mutable_orientation()->set_x(0.);
          road_marker->mutable_pose()->mutable_orientation()->set_y(0.);
          road_marker->mutable_pose()->mutable_orientation()->set_z(0.);
          road_marker->mutable_pose()->mutable_orientation()->set_w(1.);
          // 尺寸
          road_marker->mutable_scale()->set_x(0.7);
          road_marker->mutable_scale()->set_y(0.7);
          road_marker->mutable_scale()->set_z(0.7);
          road_marker->mutable_lifetime()->set_sec(0);
          road_marker->mutable_lifetime()->set_nsec(0);

          adsfi_proto::viz::ColorRGBA color;
          color.set_a(0.8);
          color.set_r(0);
          color.set_g(0);
          color.set_b(1);
          road_marker->mutable_color()->CopyFrom(color);
          for (int j = 0; j < segment.line_segment().point().size(); ++j) {
            const auto& point = segment.line_segment().point(j);
            if (utm) {
              Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
              int zone = 51;
              double x = point_utm.x();
              double y = point_utm.y();
              hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
              Eigen::Vector3d point_gcj(y, x, 0);
              Eigen::Vector3d point_enu =
                  util::Geo::Gcj02ToEnu(point_gcj, enupos);
              auto pt = road_marker->add_points();
              pt->set_x(point_enu.x());
              pt->set_y(point_enu.y());
              pt->set_z(point_enu.z());
            } else {
              Eigen::Vector3d point_enu(point.x(), point.y(), point.z());
              auto pt = road_marker->add_points();
              pt->set_x(point_enu.x());
              pt->set_y(point_enu.y());
              pt->set_z(point_enu.z());
            }
          }
          if (road_marker->points().empty()) {
            HLOG_WARN << "empty road";
          }
          road_markers.add_markers()->CopyFrom(*road_marker);
        }
      }
    }
  }
  return road_markers;
}
adsfi_proto::viz::MarkerArray MapProtoMarker::SignalToMarker(
    const std::shared_ptr<hozon::hdmap::Map>& prior_map,
    const Eigen::Vector3d& enupos, bool utm) {
  adsfi_proto::viz::MarkerArray signal_markers;
  for (const auto& signal : prior_map->signal()) {
    auto signal_marker = std::make_unique<adsfi_proto::viz::Marker>();
    signal_marker->mutable_header()->set_frameid("map");

    u_int64_t SignalId = std::stoll(signal.id().id());
    int remainder = SignalId / 10000;
    signal_marker->set_ns("ns_prior_map_signal" + std::to_string(remainder));
    signal_marker->set_id(SignalId % 10000);

    signal_marker->set_type(adsfi_proto::viz::MarkerType::CUBE_LIST);  // SPHERE
    signal_marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
    // 位姿
    signal_marker->mutable_pose()->mutable_position()->set_x(0);
    signal_marker->mutable_pose()->mutable_position()->set_y(0);
    signal_marker->mutable_pose()->mutable_position()->set_z(0);
    signal_marker->mutable_pose()->mutable_orientation()->set_x(0.);
    signal_marker->mutable_pose()->mutable_orientation()->set_y(0.);
    signal_marker->mutable_pose()->mutable_orientation()->set_z(0.);
    signal_marker->mutable_pose()->mutable_orientation()->set_w(1.);
    // 尺寸
    signal_marker->mutable_scale()->set_x(0.5);
    signal_marker->mutable_scale()->set_y(0.5);
    signal_marker->mutable_scale()->set_z(0.5);
    signal_marker->mutable_lifetime()->set_sec(0);
    signal_marker->mutable_lifetime()->set_nsec(0);

    adsfi_proto::viz::ColorRGBA color;
    color.set_a(1.0);
    color.set_r(1);
    color.set_g(0);
    color.set_b(0);
    signal_marker->mutable_color()->CopyFrom(color);

    for (const auto& point : signal.boundary().point()) {
      if (utm) {
        Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
        int zone = 51;
        double x = point_utm.x();
        double y = point_utm.y();
        hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
        Eigen::Vector3d point_gcj(y, x, 0);
        Eigen::Vector3d point_enu = util::Geo::Gcj02ToEnu(point_gcj, enupos);
        auto pt = signal_marker->add_points();
        pt->set_x(point_enu.x());
        pt->set_y(point_enu.y());
        pt->set_z(point_enu.z());
      } else {
        Eigen::Vector3d point_enu(point.x(), point.y(), point.z());
        auto pt = signal_marker->add_points();
        pt->set_x(point_enu.x());
        pt->set_y(point_enu.y());
        pt->set_z(point_enu.z());
      }
    }
    if (signal_marker->points().empty()) {
      HLOG_WARN << "empty signal";
    }
    signal_markers.add_markers()->CopyFrom(*signal_marker);
  }
  return signal_markers;
}
adsfi_proto::viz::MarkerArray MapProtoMarker::CrossWalkToMarker(
    const std::shared_ptr<hozon::hdmap::Map>& prior_map,
    const Eigen::Vector3d& enupos, bool utm) {
  adsfi_proto::viz::MarkerArray cross_walk_markers;
  for (const auto& crosswalk : prior_map->crosswalk()) {
    auto cross_walk_marker = std::make_unique<adsfi_proto::viz::Marker>();
    cross_walk_marker->mutable_header()->set_frameid("map");

    u_int64_t CrossWalkId = std::stoll(crosswalk.id().id());
    int remainder = CrossWalkId / 10000;
    cross_walk_marker->set_ns("ns_prior_map_crosswalk" +
                              std::to_string(remainder));
    cross_walk_marker->set_id(CrossWalkId % 10000);

    cross_walk_marker->set_type(adsfi_proto::viz::MarkerType::LINE_LIST);
    cross_walk_marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
    // 位姿
    cross_walk_marker->mutable_pose()->mutable_position()->set_x(0);
    cross_walk_marker->mutable_pose()->mutable_position()->set_y(0);
    cross_walk_marker->mutable_pose()->mutable_position()->set_z(0);
    cross_walk_marker->mutable_pose()->mutable_orientation()->set_x(0.);
    cross_walk_marker->mutable_pose()->mutable_orientation()->set_y(0.);
    cross_walk_marker->mutable_pose()->mutable_orientation()->set_z(0.);
    cross_walk_marker->mutable_pose()->mutable_orientation()->set_w(1.);
    // 尺寸
    cross_walk_marker->mutable_scale()->set_x(0.3);
    cross_walk_marker->mutable_scale()->set_y(0.3);
    cross_walk_marker->mutable_scale()->set_z(0.3);
    cross_walk_marker->mutable_lifetime()->set_sec(0);
    cross_walk_marker->mutable_lifetime()->set_nsec(0);

    adsfi_proto::viz::ColorRGBA color;
    color.set_a(1.0);
    color.set_r(1.0);
    color.set_g(0.0);
    color.set_b(0.0);
    cross_walk_marker->mutable_color()->CopyFrom(color);

    for (const auto& point : crosswalk.polygon().point()) {
      Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
      int zone = 51;
      double x = point_utm.x();
      double y = point_utm.y();
      hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
      Eigen::Vector3d point_gcj(y, x, 0);
      Eigen::Vector3d point_enu = util::Geo::Gcj02ToEnu(point_gcj, enupos);
      auto pt = cross_walk_marker->add_points();
      pt->set_x(point_enu.x());
      pt->set_y(point_enu.y());
      pt->set_z(point_enu.z());
    }
    if (cross_walk_marker->points().empty()) {
      HLOG_WARN << "empty cross_walk";
    }
    cross_walk_markers.add_markers()->CopyFrom(*cross_walk_marker);
  }
  return cross_walk_markers;
}
adsfi_proto::viz::MarkerArray MapProtoMarker::StopSignToMarker(
    const std::shared_ptr<hozon::hdmap::Map>& prior_map,
    const Eigen::Vector3d& enupos, bool utm) {
  adsfi_proto::viz::MarkerArray stop_sign_markers;
  for (const auto& stop_sign : prior_map->stop_sign()) {
    auto stop_sign_marker = std::make_unique<adsfi_proto::viz::Marker>();
    stop_sign_marker->mutable_header()->set_frameid("map");

    u_int64_t StopSignId = std::stoll(stop_sign.id().id());
    int remainder = StopSignId / 10000;
    stop_sign_marker->set_ns("ns_prior_map_stop_sign" +
                             std::to_string(remainder));
    stop_sign_marker->set_id(StopSignId % 10000);

    stop_sign_marker->set_type(adsfi_proto::viz::MarkerType::CUBE);
    stop_sign_marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
    // 位姿
    stop_sign_marker->mutable_pose()->mutable_position()->set_x(0);
    stop_sign_marker->mutable_pose()->mutable_position()->set_y(0);
    stop_sign_marker->mutable_pose()->mutable_position()->set_z(0);
    stop_sign_marker->mutable_pose()->mutable_orientation()->set_x(0.);
    stop_sign_marker->mutable_pose()->mutable_orientation()->set_y(0.);
    stop_sign_marker->mutable_pose()->mutable_orientation()->set_z(0.);
    stop_sign_marker->mutable_pose()->mutable_orientation()->set_w(1.);
    // 尺寸
    stop_sign_marker->mutable_scale()->set_x(0.5);
    stop_sign_marker->mutable_scale()->set_y(0.5);
    stop_sign_marker->mutable_scale()->set_z(0.01);
    stop_sign_marker->mutable_lifetime()->set_sec(0);
    stop_sign_marker->mutable_lifetime()->set_nsec(0);

    adsfi_proto::viz::ColorRGBA color;
    color.set_a(1.0);
    color.set_r(1.0);
    color.set_g(0.0);
    color.set_b(0.0);
    stop_sign_marker->mutable_color()->CopyFrom(color);
    for (const auto& stop_line : stop_sign.stop_line()) {
      for (const auto& segment : stop_line.segment()) {
        for (int j = 0; segment.line_segment().point().size(); ++j) {
          const auto& point = segment.line_segment().point(j);
          Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
          int zone = 51;
          double x = point_utm.x();
          double y = point_utm.y();
          hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
          Eigen::Vector3d point_gcj(y, x, 0);
          Eigen::Vector3d point_enu = util::Geo::Gcj02ToEnu(point_gcj, enupos);
          auto pt = stop_sign_marker->add_points();
          pt->set_x(point_enu.x());
          pt->set_y(point_enu.y());
          pt->set_z(point_enu.z());
        }
      }
    }
    if (stop_sign_marker->points().empty()) {
      HLOG_WARN << "empty stop_line";
    }
    stop_sign_markers.add_markers()->CopyFrom(*stop_sign_marker);
  }
  return stop_sign_markers;
}
adsfi_proto::viz::MarkerArray MapProtoMarker::ClearAreaToMarker(
    const std::shared_ptr<hozon::hdmap::Map>& prior_map,
    const Eigen::Vector3d& enupos, bool utm) {
  adsfi_proto::viz::MarkerArray clear_area_markers;
  for (const auto& clear_area : prior_map->clear_area()) {
    auto clear_area_marker = std::make_unique<adsfi_proto::viz::Marker>();
    clear_area_marker->mutable_header()->set_frameid("map");

    u_int64_t ClearAreaId = std::stoll(clear_area.id().id());
    int remainder = ClearAreaId / 10000;
    clear_area_marker->set_ns("ns_prior_map_clear_area" +
                              std::to_string(remainder));
    clear_area_marker->set_id(ClearAreaId % 10000);

    clear_area_marker->set_type(adsfi_proto::viz::MarkerType::SPHERE);
    clear_area_marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
    // 位姿
    clear_area_marker->mutable_pose()->mutable_position()->set_x(0);
    clear_area_marker->mutable_pose()->mutable_position()->set_y(0);
    clear_area_marker->mutable_pose()->mutable_position()->set_z(0);
    clear_area_marker->mutable_pose()->mutable_orientation()->set_x(0.);
    clear_area_marker->mutable_pose()->mutable_orientation()->set_y(0.);
    clear_area_marker->mutable_pose()->mutable_orientation()->set_z(0.);
    clear_area_marker->mutable_pose()->mutable_orientation()->set_w(1.);
    // 尺寸
    clear_area_marker->mutable_scale()->set_x(0);
    clear_area_marker->mutable_scale()->set_y(0);
    clear_area_marker->mutable_scale()->set_z(0.1);
    clear_area_marker->mutable_lifetime()->set_sec(0);
    clear_area_marker->mutable_lifetime()->set_nsec(0);

    adsfi_proto::viz::ColorRGBA color;
    color.set_a(1.0);
    color.set_r(0.0);
    color.set_g(0.0);
    color.set_b(1.0);
    clear_area_marker->mutable_color()->CopyFrom(color);
    for (const auto& point : clear_area.polygon().point()) {
      Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
      int zone = 51;
      double x = point_utm.x();
      double y = point_utm.y();
      hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
      Eigen::Vector3d point_gcj(y, x, 0);
      Eigen::Vector3d point_enu = util::Geo::Gcj02ToEnu(point_gcj, enupos);
      auto pt = clear_area_marker->add_points();
      pt->set_x(point_enu.x());
      pt->set_y(point_enu.y());
      pt->set_z(point_enu.z());
    }
    if (clear_area_marker->points().empty()) {
      HLOG_WARN << "empty junction";
    }
    clear_area_markers.add_markers()->CopyFrom(*clear_area_marker);
  }
  return clear_area_markers;
}
adsfi_proto::viz::MarkerArray MapProtoMarker::SpeedBumpToMarker(
    const std::shared_ptr<hozon::hdmap::Map>& prior_map,
    const Eigen::Vector3d& enupos, bool utm) {
  adsfi_proto::viz::MarkerArray speed_bump_markers;
  for (const auto& speed_bump : prior_map->speed_bump()) {
    auto speed_bump_marker = std::make_unique<adsfi_proto::viz::Marker>();
    speed_bump_marker->mutable_header()->set_frameid("map");

    u_int64_t SpeedBumpId = std::stoll(speed_bump.id().id());
    int remainder = SpeedBumpId / 10000;
    speed_bump_marker->set_ns("ns_prior_map_speed_bump" +
                              std::to_string(remainder));
    speed_bump_marker->set_id(SpeedBumpId % 10000);

    speed_bump_marker->set_type(adsfi_proto::viz::MarkerType::CUBE);
    speed_bump_marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
    // 位姿
    speed_bump_marker->mutable_pose()->mutable_position()->set_x(0);
    speed_bump_marker->mutable_pose()->mutable_position()->set_y(0);
    speed_bump_marker->mutable_pose()->mutable_position()->set_z(0);
    speed_bump_marker->mutable_pose()->mutable_orientation()->set_x(0.);
    speed_bump_marker->mutable_pose()->mutable_orientation()->set_y(0.);
    speed_bump_marker->mutable_pose()->mutable_orientation()->set_z(0.);
    speed_bump_marker->mutable_pose()->mutable_orientation()->set_w(1.);
    // 尺寸
    speed_bump_marker->mutable_scale()->set_x(0);
    speed_bump_marker->mutable_scale()->set_y(0);
    speed_bump_marker->mutable_scale()->set_z(0.5);
    speed_bump_marker->mutable_lifetime()->set_sec(0);
    speed_bump_marker->mutable_lifetime()->set_nsec(0);

    adsfi_proto::viz::ColorRGBA color;
    color.set_a(1.0);
    color.set_r(1.0);
    color.set_g(0.0);
    color.set_b(1.0);
    speed_bump_marker->mutable_color()->CopyFrom(color);
    for (const auto& position : speed_bump.position()) {
      for (const auto& segment : position.segment()) {
        for (int j = 0; segment.line_segment().point().size(); ++j) {
          const auto& point = segment.line_segment().point(j);
          Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
          int zone = 51;
          double x = point_utm.x();
          double y = point_utm.y();
          hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
          Eigen::Vector3d point_gcj(y, x, 0);
          Eigen::Vector3d point_enu = util::Geo::Gcj02ToEnu(point_gcj, enupos);
          auto pt = speed_bump_marker->add_points();
          pt->set_x(point_enu.x());
          pt->set_y(point_enu.y());
          pt->set_z(point_enu.z());
        }
      }
    }
    if (speed_bump_marker->points().empty()) {
      HLOG_WARN << "empty speed_bump";
    }
    speed_bump_markers.add_markers()->CopyFrom(*speed_bump_marker);
  }
  return speed_bump_markers;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
