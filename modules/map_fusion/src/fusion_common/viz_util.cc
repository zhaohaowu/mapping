/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： viz_util.cc
 *   author     ： taoshaoyuan
 *   date       ： 2023.09
 ******************************************************************************/

#include "map_fusion/fusion_common/viz_util.h"

#include <cstdint>

#include <opencv2/opencv.hpp>

#include "adsfi_proto/viz/visualization_msgs.pb.h"
#include "base/utils/log.h"
#include "map_fusion/fusion_common/calc_util.h"
#include "map_fusion/fusion_common/common_data.h"
#include "map_fusion/fusion_common/element_map.h"
#include "util/mapping_log.h"

namespace hozon {
namespace mp {
namespace mf {
namespace viz {

#define FLOAT_RGB(IntRgb) (IntRgb / 255.0)

Rgb ColorRgb(Color color) {
  // rgb value refers to: https://tool.oschina.net/commons?type=3
  std::map<Color, Rgb> color_map = {
      {WHITE, {FLOAT_RGB(255), FLOAT_RGB(255), FLOAT_RGB(255)}},
      {GREY, {FLOAT_RGB(190), FLOAT_RGB(190), FLOAT_RGB(190)}},
      {BLACK, {FLOAT_RGB(0), FLOAT_RGB(0), FLOAT_RGB(0)}},
      {RED, {FLOAT_RGB(255), FLOAT_RGB(0), FLOAT_RGB(0)}},
      {ORANGE, {FLOAT_RGB(255), FLOAT_RGB(165), FLOAT_RGB(0)}},
      {YELLOW, {FLOAT_RGB(255), FLOAT_RGB(255), FLOAT_RGB(0)}},
      {GREEN, {FLOAT_RGB(0), FLOAT_RGB(255), FLOAT_RGB(0)}},
      {BLUE, {FLOAT_RGB(0), FLOAT_RGB(0), FLOAT_RGB(255)}},
      {CYAN, {FLOAT_RGB(0), FLOAT_RGB(255), FLOAT_RGB(255)}},
      {PURPLE, {FLOAT_RGB(160), FLOAT_RGB(32), FLOAT_RGB(240)}},
  };
  if (color_map.find(color) == color_map.end()) {
    color = WHITE;
  }
  return color_map[color];
}

void PoseToTf(const Pose& pose, const std::string& frame_id,
              const std::string& child_frame_id,
              adsfi_proto::viz::TransformStamped* tf) {
  if (tf == nullptr) {
    return;
  }

  auto sec = static_cast<uint32_t>(pose.stamp);
  auto nsec = static_cast<uint32_t>((pose.stamp - sec) * 1e9);
  tf->mutable_header()->set_frameid(frame_id);
  tf->mutable_header()->mutable_timestamp()->set_sec(sec);
  tf->mutable_header()->mutable_timestamp()->set_nsec(nsec);
  tf->set_child_frame_id(child_frame_id);
  tf->mutable_transform()->mutable_translation()->set_x(pose.pos.x());
  tf->mutable_transform()->mutable_translation()->set_y(pose.pos.y());
  tf->mutable_transform()->mutable_translation()->set_z(pose.pos.z());
  tf->mutable_transform()->mutable_rotation()->set_w(pose.quat.w());
  tf->mutable_transform()->mutable_rotation()->set_x(pose.quat.x());
  tf->mutable_transform()->mutable_rotation()->set_y(pose.quat.y());
  tf->mutable_transform()->mutable_rotation()->set_z(pose.quat.z());
}

void PoseToOdom(const Pose& pose, const std::string& frame_id,
                adsfi_proto::viz::Odometry* odom) {
  if (!odom) {
    return;
  }

  odom->Clear();
  odom->mutable_header()->set_frameid(frame_id);
  uint32_t sec, nsec;
  SplitSeconds(pose.stamp, &sec, &nsec);

  odom->mutable_header()->mutable_timestamp()->set_sec(sec);
  odom->mutable_header()->mutable_timestamp()->set_nsec(nsec);
  odom->mutable_pose()->mutable_pose()->mutable_position()->set_x(pose.pos.x());
  odom->mutable_pose()->mutable_pose()->mutable_position()->set_y(pose.pos.y());
  odom->mutable_pose()->mutable_pose()->mutable_position()->set_z(pose.pos.z());
  odom->mutable_pose()->mutable_pose()->mutable_orientation()->set_x(
      pose.quat.x());
  odom->mutable_pose()->mutable_pose()->mutable_orientation()->set_y(
      pose.quat.y());
  odom->mutable_pose()->mutable_pose()->mutable_orientation()->set_z(
      pose.quat.z());
  odom->mutable_pose()->mutable_pose()->mutable_orientation()->set_w(
      pose.quat.w());
}

void LineCubicToMarker(const LineCubic& cubic, const Rgb& rgb, LineType type,
                       const std::string& frame_id, const std::string& ns,
                       double stamp, double lifetime, int32_t id,
                       float sample_dist, adsfi_proto::viz::Marker* marker) {
  marker->Clear();
  marker->mutable_header()->set_frameid(frame_id);
  auto sec = static_cast<uint32_t>(stamp);
  auto nsec = static_cast<uint32_t>((stamp - sec) * 1e9);
  marker->mutable_header()->mutable_timestamp()->set_sec(sec);
  marker->mutable_header()->mutable_timestamp()->set_nsec(nsec);
  marker->set_ns(ns);
  marker->set_id(id);
  marker->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker->mutable_pose()->mutable_orientation()->set_x(0.);
  marker->mutable_pose()->mutable_orientation()->set_y(0.);
  marker->mutable_pose()->mutable_orientation()->set_z(0.);
  marker->mutable_pose()->mutable_orientation()->set_w(1.);
  double line_width = 0.1;
  if (type == SHORT_SINGLE_DASHED) {
    line_width *= 2;
  }
  marker->mutable_scale()->set_x(line_width);
  sec = static_cast<uint32_t>(lifetime);
  nsec = static_cast<uint32_t>((lifetime - sec) * 1e9);
  marker->mutable_lifetime()->set_sec(sec);
  marker->mutable_lifetime()->set_nsec(nsec);
  marker->mutable_color()->set_a(1.0);
  marker->mutable_color()->set_r(rgb.r);
  marker->mutable_color()->set_g(rgb.g);
  marker->mutable_color()->set_b(rgb.b);
  marker->set_type(adsfi_proto::viz::MarkerType::LINE_LIST);
  if (type == SINGLE_SOLID) {
    marker->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
  }

  std::vector<adsfi_proto::viz::Point> sampled;
  for (float x = cubic.start_x; x < cubic.end_x;) {
    float y = cubic.c3 * x * x * x + cubic.c2 * x * x + cubic.c1 * x + cubic.c0;
    adsfi_proto::viz::Point pt;
    pt.set_x(x);
    pt.set_y(y);
    pt.set_z(0);
    sampled.emplace_back(pt);
    x += sample_dist;
  }
  float end_y = cubic.c3 * cubic.end_x * cubic.end_x * cubic.end_x +
                cubic.c2 * cubic.end_x * cubic.end_x + cubic.c1 * cubic.end_x +
                cubic.c0;
  adsfi_proto::viz::Point end_pt;
  end_pt.set_x(cubic.end_x);
  end_pt.set_y(end_y);
  end_pt.set_z(0);
  sampled.emplace_back(end_pt);

  // make sure points num is even
  if (marker->type() == adsfi_proto::viz::MarkerType::LINE_LIST &&
      sampled.size() % 2 != 0) {
    sampled.emplace_back(end_pt);
  }

  if (type == SINGLE_SOLID || type == SINGLE_DASHED ||
      type == SHORT_SINGLE_DASHED) {
    // single line
    for (const auto& spt : sampled) {
      auto pt = marker->add_points();
      pt->CopyFrom(spt);
    }
  } else {
    // double lines
    std::vector<adsfi_proto::viz::Point> sampled_left;
    std::vector<adsfi_proto::viz::Point> sampled_right;
    for (const auto& spt : sampled) {
      adsfi_proto::viz::Point lpt, rpt;
      lpt.CopyFrom(spt);
      rpt.CopyFrom(spt);
      lpt.set_y(spt.y() + line_width);
      rpt.set_y(spt.y() - line_width);
      sampled_left.emplace_back(lpt);
      sampled_right.emplace_back(rpt);
    }

    // solid left
    if (type == DOUBLE_SOLID || type == LEFT_SOLID_RIGHT_DASHED) {
      for (int i = 0; i != (static_cast<int>(sampled_left.size()) - 1); i++) {
        auto pt = marker->add_points();
        pt->CopyFrom(sampled_left[i]);
        pt = marker->add_points();
        pt->CopyFrom(sampled_left[i + 1]);
      }
    }

    // solid right
    if (type == DOUBLE_SOLID || type == LEFT_DASHED_RIGHT_SOLID) {
      for (int i = 0; i != (static_cast<int>(sampled_right.size()) - 1); i++) {
        auto pt = marker->add_points();
        pt->CopyFrom(sampled_right[i]);
        pt = marker->add_points();
        pt->CopyFrom(sampled_right[i + 1]);
      }
    }

    // dashed left
    if (type == DOUBLE_DASHED || type == LEFT_DASHED_RIGHT_SOLID) {
      for (const auto& spt : sampled_left) {
        auto pt = marker->add_points();
        pt->CopyFrom(spt);
      }
    }

    // dashed right
    if (type == DOUBLE_DASHED || type == LEFT_SOLID_RIGHT_DASHED) {
      for (const auto& spt : sampled_right) {
        auto pt = marker->add_points();
        pt->CopyFrom(spt);
      }
    }
  }
}

MarkerArrayPtr LaneInfoToMarkers(const hozon::perception::LaneInfo& lane,
                                 const std::string& frame_id,
                                 const std::string& ns, double stamp,
                                 double lifetime) {
  auto ma = std::make_shared<adsfi_proto::viz::MarkerArray>();

  Rgb rgb = ColorRgb(BLACK);
  switch (lane.color()) {
    case hozon::perception::WHITE:
      rgb = ColorRgb(WHITE);
      break;
    case hozon::perception::YELLOW:
      rgb = ColorRgb(YELLOW);
      break;
    case hozon::perception::GREEN:
      rgb = ColorRgb(GREEN);
      break;
    case hozon::perception::RED:
      rgb = ColorRgb(RED);
      break;
    case hozon::perception::BLACK:
      rgb = ColorRgb(BLACK);
      break;
    case hozon::perception::UNKNOWN:
    default:
      rgb = ColorRgb(CYAN);
      break;
  }

  // append track id to namespace
  std::string ns_id = ns + "/" + std::to_string(lane.track_id());
  LineType type = SINGLE_SOLID;
  switch (lane.lanetype()) {
    case hozon::perception::SolidLine:
      type = SINGLE_SOLID;
      break;
    case hozon::perception::DashedLine:
      type = SINGLE_DASHED;
      break;
    case hozon::perception::ShortDashedLine:
      type = SHORT_SINGLE_DASHED;
      break;
    case hozon::perception::DoubleSolidLine:
      type = DOUBLE_SOLID;
      break;
    case hozon::perception::DoubleDashedLine:
      type = DOUBLE_DASHED;
      break;
    case hozon::perception::LeftSolidRightDashed:
      type = LEFT_SOLID_RIGHT_DASHED;
      break;
    case hozon::perception::RightSolidLeftDashed:
      type = LEFT_DASHED_RIGHT_SOLID;
      break;
    default:
      type = SINGLE_SOLID;
      break;
  }

  float sample_dist = 1.0;

  for (int i = 0; i != lane.lane_param().cubic_curve_set_size(); ++i) {
    LineCubic cubic;
    cubic.start_x = lane.lane_param().cubic_curve_set(i).start_point_x();
    cubic.end_x = lane.lane_param().cubic_curve_set(i).end_point_x();
    cubic.c0 = lane.lane_param().cubic_curve_set(i).c0();
    cubic.c1 = lane.lane_param().cubic_curve_set(i).c1();
    cubic.c2 = lane.lane_param().cubic_curve_set(i).c2();
    cubic.c3 = lane.lane_param().cubic_curve_set(i).c3();

    auto m = ma->add_markers();
    LineCubicToMarker(cubic, rgb, type, frame_id, ns_id, stamp, lifetime, i,
                      sample_dist, m);
  }

  if (ma->markers().empty()) {
    return nullptr;
  }

  return ma;
}

MarkerArrayPtr TransportElementToMarkers(
    const hozon::perception::TransportElement& element,
    const std::string& frame_id, const std::string& ns, double lifetime) {
  auto ma = std::make_shared<adsfi_proto::viz::MarkerArray>();

  std::string ns_lane = ns + "/lane";
  for (int i = 0; i != element.lane_size(); ++i) {
    std::string ns_lane_i = ns_lane + "/" + std::to_string(i);
    auto lane_ma =
        LaneInfoToMarkers(element.lane(i), frame_id, ns_lane_i,
                          element.header().publish_stamp(), lifetime);
    if (lane_ma) {
      for (const auto& itt : lane_ma->markers()) {
        auto m = ma->add_markers();
        m->CopyFrom(itt);
      }
    }
  }

  if (ma->markers().empty()) {
    return nullptr;
  }

  return ma;
}

CompressedImagePtr JpegImageToVizImage(
    const hozon::soc::CompressedImage& sensor_img) {
  if (sensor_img.format() != "jpeg" && sensor_img.format() != "jpg") {
    HLOG_ERROR << "not support " << sensor_img.format() << ", only support "
               << "jpeg/jpg";
    return nullptr;
  }

  auto img = std::make_shared<adsfi_proto::viz::CompressedImage>();
  img->mutable_header()->set_seq(sensor_img.header().seq());
  img->mutable_header()->set_frameid(sensor_img.header().frame_id());
  auto sec = static_cast<uint32_t>(sensor_img.header().publish_stamp());
  auto nsec =
      static_cast<uint32_t>((sensor_img.header().publish_stamp() - sec) * 1e9);
  img->mutable_header()->mutable_timestamp()->set_sec(sec);
  img->mutable_header()->mutable_timestamp()->set_nsec(nsec);
  img->set_format(sensor_img.format());
  for (const auto& b : sensor_img.data()) {
    img->mutable_data()->push_back(b);
  }
  if (img->data().empty()) {
    return nullptr;
  }

  return img;
}

CompressedImagePtr YUVNV12ImageToVizImage(
    const std::shared_ptr<hozon::soc::Image>& yuv_image, int quality,
    double resize_factor) {
  if (yuv_image == nullptr) {
    HLOG_ERROR << "nullptr input yuv";
    return nullptr;
  }

  auto width = static_cast<int>(yuv_image->width());
  auto height = static_cast<int>(yuv_image->height());
  size_t nv12_bytes_num = height * width * 3 / 2;
  if (yuv_image->data().size() != nv12_bytes_num) {
    HLOG_ERROR << "invalid yuv data size";
    return nullptr;
  }

  if (quality <= 0 || quality > 100) {
    HLOG_ERROR << "invalid quality, should in range (0, 100]";
    return nullptr;
  }

  if (resize_factor <= 0. || resize_factor > 1.) {
    HLOG_ERROR << "invalid resize factor, should in range (0, 1]";
    return nullptr;
  }

  cv::Mat yuv_nv12;
  yuv_nv12.create(height * 3 / 2, width, CV_8UC1);
  memcpy(yuv_nv12.data, yuv_image->data().data(), nv12_bytes_num);
  cv::Mat bgr;
  cv::cvtColor(yuv_nv12, bgr, cv::COLOR_YUV2BGR_NV12);

  if (resize_factor < 1.0) {
    auto resized_cols = static_cast<int>(bgr.cols * resize_factor);
    auto resized_rows = static_cast<int>(bgr.rows * resize_factor);
    cv::resize(bgr, bgr, cv::Size(resized_cols, resized_rows));
  }

  std::vector<int> param(2);
  param[0] = cv::IMWRITE_JPEG_QUALITY;
  param[1] = quality;
  std::vector<uchar> buf;
  cv::imencode(".jpg", bgr, buf, param);

  auto viz_image = std::make_shared<adsfi_proto::viz::CompressedImage>();
  viz_image->mutable_header()->set_seq(yuv_image->header().seq());
  viz_image->mutable_header()->set_frameid(yuv_image->header().frame_id());
  auto raw_secs = yuv_image->header().sensor_stamp().camera_stamp();
  auto sec = static_cast<uint32_t>(raw_secs);
  auto nsec = static_cast<uint32_t>((raw_secs - sec) * 1e9);
  viz_image->mutable_header()->mutable_timestamp()->set_sec(sec);
  viz_image->mutable_header()->mutable_timestamp()->set_nsec(nsec);
  viz_image->set_format("jpeg");
  viz_image->mutable_data()->resize(buf.size());
  memcpy(viz_image->mutable_data()->data(), buf.data(), buf.size());

  return viz_image;
}

void ElementBoundaryNodesToMarker(
    const std::vector<em::BoundaryNode::Ptr>& nodes,
    const std::string& frame_id, const std::string& ns, int32_t id,
    double stamp, double lifetime, Color color,
    adsfi_proto::viz::Marker* marker) {
  if (!marker) {
    return;
  }

  marker->Clear();
  marker->mutable_header()->set_frameid(frame_id);
  auto sec = static_cast<uint32_t>(stamp);
  auto nsec = static_cast<uint32_t>((stamp - sec) * 1e9);
  marker->mutable_header()->mutable_timestamp()->set_sec(sec);
  marker->mutable_header()->mutable_timestamp()->set_nsec(nsec);
  std::string ns_nodes = ns + "/" + ID_PREFIX_NODE;
  marker->set_ns(ns_nodes);
  marker->set_id(id);
  marker->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  //  marker->mutable_pose()->mutable_orientation()->set_x(0.);
  //  marker->mutable_pose()->mutable_orientation()->set_y(0.);
  //  marker->mutable_pose()->mutable_orientation()->set_z(0.);
  //  marker->mutable_pose()->mutable_orientation()->set_w(1.);
  //  marker->mutable_pose()->mutable_position()->set_x(node.point.x());
  //  marker->mutable_pose()->mutable_position()->set_y(node.point.y());
  //  marker->mutable_pose()->mutable_position()->set_z(node.point.z());
  double diameter = 0.5;
  marker->mutable_scale()->set_x(diameter);
  marker->mutable_scale()->set_y(diameter);
  marker->mutable_scale()->set_z(diameter);

  sec = static_cast<uint32_t>(lifetime);
  nsec = static_cast<uint32_t>((lifetime - sec) * 1e9);
  marker->mutable_lifetime()->set_sec(sec);
  marker->mutable_lifetime()->set_nsec(nsec);

  Rgb rgb = ColorRgb(color);
  marker->mutable_color()->set_a(1.0);
  marker->mutable_color()->set_r(rgb.r);
  marker->mutable_color()->set_g(rgb.g);
  marker->mutable_color()->set_b(rgb.b);

  marker->set_type(adsfi_proto::viz::MarkerType::SPHERE_LIST);

  for (const auto& it : nodes) {
    if (!it) {
      HLOG_ERROR << "try viz nullptr node";
      continue;
    }
    auto* pt = marker->add_points();
    pt->set_x(it->point.x());
    pt->set_y(it->point.y());
    pt->set_z(it->point.z());
  }
}

void ElementBoundaryToMarker(const em::Boundary& boundary,
                             const std::string& frame_id, const std::string& ns,
                             int32_t id, double stamp, double lifetime,
                             Color color, adsfi_proto::viz::Marker* marker) {
  if (!marker) {
    return;
  }

  marker->Clear();
  marker->mutable_header()->set_frameid(frame_id);
  auto sec = static_cast<uint32_t>(stamp);
  auto nsec = static_cast<uint32_t>((stamp - sec) * 1e9);
  marker->mutable_header()->mutable_timestamp()->set_sec(sec);
  marker->mutable_header()->mutable_timestamp()->set_nsec(nsec);
  std::string ns_nodes = ns + "/" + ID_PREFIX_BOUNDARY;
  marker->set_ns(ns_nodes);
  marker->set_id(id);
  marker->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker->mutable_pose()->mutable_orientation()->set_x(0.);
  marker->mutable_pose()->mutable_orientation()->set_y(0.);
  marker->mutable_pose()->mutable_orientation()->set_z(0.);
  marker->mutable_pose()->mutable_orientation()->set_w(1.);

  double width = 0.2;
  marker->mutable_scale()->set_x(width);
  sec = static_cast<uint32_t>(lifetime);
  nsec = static_cast<uint32_t>((lifetime - sec) * 1e9);
  marker->mutable_lifetime()->set_sec(sec);
  marker->mutable_lifetime()->set_nsec(nsec);
  Rgb rgb = ColorRgb(color);
  marker->mutable_color()->set_a(1.0);
  marker->mutable_color()->set_r(rgb.r);
  marker->mutable_color()->set_g(rgb.g);
  marker->mutable_color()->set_b(rgb.b);
  marker->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);

  for (const auto& it : boundary.nodes) {
    if (!it) {
      HLOG_ERROR << "try viz nullptr node";
      continue;
    }
    auto* pt = marker->add_points();
    pt->set_x(it->point.x());
    pt->set_y(it->point.y());
    pt->set_z(it->point.z());
  }
}

void ElementOccEgoToMarker(const em::Boundary& boundary,
                           const std::string& frame_id, const std::string& ns,
                           int32_t id, double stamp, double lifetime,
                           Color color, adsfi_proto::viz::Marker* marker) {
  // 可视化ego
  if (!marker) {
    return;
  }
  marker->Clear();
  marker->mutable_header()->set_frameid(frame_id);
  auto sec = static_cast<uint32_t>(stamp);
  auto nsec = static_cast<uint32_t>((stamp - sec) * 1e9);
  marker->mutable_header()->mutable_timestamp()->set_sec(sec);
  marker->mutable_header()->mutable_timestamp()->set_nsec(nsec);
  std::string ns_nodes = ns + "/" + ID_PREFIX_BOUNDARY;
  marker->set_ns(ns_nodes);
  marker->set_id(id);
  marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
  marker->mutable_pose()->mutable_orientation()->set_x(0.);
  marker->mutable_pose()->mutable_orientation()->set_y(0.);
  marker->mutable_pose()->mutable_orientation()->set_z(0.);
  marker->mutable_pose()->mutable_orientation()->set_w(1.);

  marker->mutable_pose()->mutable_position()->set_x(
      boundary.nodes.at(0)->point.x());
  marker->mutable_pose()->mutable_position()->set_y(
      boundary.nodes.at(0)->point.y());
  marker->mutable_pose()->mutable_position()->set_z(
      boundary.nodes.at(0)->point.z());

  double width = 0.5;
  marker->mutable_scale()->set_z(width);
  sec = static_cast<uint32_t>(lifetime);
  nsec = static_cast<uint32_t>((lifetime - sec) * 1e9);
  marker->mutable_lifetime()->set_sec(sec);
  marker->mutable_lifetime()->set_nsec(nsec);
  // if (occ_road.is_forward) {
  //   color = RED;
  // }
  Rgb rgb = ColorRgb(color);
  marker->mutable_color()->set_a(1.0);
  marker->mutable_color()->set_r(rgb.r);
  marker->mutable_color()->set_g(rgb.g);
  marker->mutable_color()->set_b(rgb.b);
  marker->set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);

  // if (occ_road.road_points.empty()) {
  //   HLOG_ERROR << "try viz nullptr occ_road";
  //   return;
  // }
  // for (const auto& it : occ_road.road_points) {
  //   auto* pt = marker->add_points();
  //   pt->set_x(it.x());
  //   pt->set_y(it.y());
  //   pt->set_z(it.z());
  // }
  auto* text = marker->mutable_text();
  *text = "Geo: " + std::to_string(boundary.is_ego) + "\n" +
          "is_near_road_edge: " + std::to_string(boundary.is_near_road_edge);
}

void AddOccRoadToMarker(const std::string& frame_id, const std::string& ns,
                        int32_t id, double stamp, double lifetime, Color color,
                        float alpha, const std::vector<Eigen::Vector3d>& pts,
                        adsfi_proto::viz::Marker* marker) {
  if (!marker) {
    return;
  }
  if (pts.empty()) {
    return;
  }
  marker->Clear();
  marker->mutable_header()->set_frameid(frame_id);
  auto sec = static_cast<uint32_t>(stamp);
  auto nsec = static_cast<uint32_t>((stamp - sec) * 1e9);
  marker->mutable_header()->mutable_timestamp()->set_sec(sec);
  marker->mutable_header()->mutable_timestamp()->set_nsec(nsec);
  std::string ns_nodes = ns + "/" + "occ";
  marker->set_ns(ns_nodes);
  marker->set_id(id);
  marker->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker->mutable_pose()->mutable_orientation()->set_x(0.);
  marker->mutable_pose()->mutable_orientation()->set_y(0.);
  marker->mutable_pose()->mutable_orientation()->set_z(0.);
  marker->mutable_pose()->mutable_orientation()->set_w(1.);

  double width = 0.2;
  marker->mutable_scale()->set_x(width);
  sec = static_cast<uint32_t>(lifetime);
  nsec = static_cast<uint32_t>((lifetime - sec) * 1e9);
  marker->mutable_lifetime()->set_sec(sec);
  marker->mutable_lifetime()->set_nsec(nsec);
  Rgb rgb = ColorRgb(color);
  marker->mutable_color()->set_a(alpha);
  marker->mutable_color()->set_r(rgb.r);
  marker->mutable_color()->set_g(rgb.g);
  marker->mutable_color()->set_b(rgb.b);
  marker->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
  for (const auto& it : pts) {
    auto* pt = marker->add_points();
    pt->set_x(it.x());
    pt->set_y(it.y());
    pt->set_z(it.z());
  }
}

void ElementOccRoadToMarker(
    const em::OccRoad& occ_road, const std::string& frame_id,
    const std::string& ns, int32_t id, double stamp, double lifetime,
    Color color, std::shared_ptr<adsfi_proto::viz::MarkerArray> marker_array) {
  // occ_road可视化
  // 对于forward还需要可视化原始点
  if (occ_road.is_forward) {
    auto* marker = marker_array->add_markers();
    AddOccRoadToMarker(frame_id, ns, id, stamp, lifetime, viz::WHITE, 0.6,
                       occ_road.ori_road_points, marker);
    auto* ori_marker = marker_array->add_markers();
    AddOccRoadToMarker(frame_id, ns, id + 100, stamp, lifetime, viz::RED, 1.0,
                       occ_road.road_points, ori_marker);
    auto* detect_marker = marker_array->add_markers();
    AddOccRoadToMarker(frame_id, ns, id + 300, stamp, lifetime, viz::BLUE, 1.0,
                       occ_road.ori_detect_points, detect_marker);
  } else {
    auto* ori_marker = marker_array->add_markers();
    AddOccRoadToMarker(frame_id, ns, id + 200, stamp, lifetime, viz::GREY, 1.0,
                       occ_road.road_points, ori_marker);
    auto* detect_marker = marker_array->add_markers();
    AddOccRoadToMarker(frame_id, ns, id + 400, stamp, lifetime, viz::BLUE, 1.0,
                       occ_road.ori_detect_points, detect_marker);
  }

  Eigen::Vector3d txt_point{occ_road.road_points.back().x(),
                            occ_road.road_points.back().y(), 0};
  auto* txt_marker = marker_array->add_markers();
  txt_marker->set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
  txt_marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
  txt_marker->set_id(id + 3);
  auto sec = static_cast<uint32_t>(lifetime);
  auto nsec = static_cast<uint32_t>((lifetime - sec) * 1e9);
  txt_marker->mutable_lifetime()->set_sec(sec);
  txt_marker->mutable_lifetime()->set_nsec(nsec);
  sec = static_cast<uint32_t>(stamp);
  nsec = static_cast<uint32_t>((stamp - sec) * 1e9);
  txt_marker->mutable_header()->mutable_timestamp()->set_sec(sec);
  txt_marker->mutable_header()->mutable_timestamp()->set_nsec(nsec);
  txt_marker->mutable_header()->set_frameid(frame_id);
  txt_marker->mutable_pose()->mutable_position()->set_x(txt_point[0]);
  txt_marker->mutable_pose()->mutable_position()->set_y(txt_point[1]);
  txt_marker->mutable_pose()->mutable_position()->set_z(2);
  txt_marker->mutable_pose()->mutable_orientation()->set_x(0.);
  txt_marker->mutable_pose()->mutable_orientation()->set_y(0.);
  txt_marker->mutable_pose()->mutable_orientation()->set_z(0.);
  txt_marker->mutable_pose()->mutable_orientation()->set_w(1.);
  txt_marker->mutable_color()->set_a(1);
  txt_marker->mutable_color()->set_r(1.0);
  txt_marker->mutable_color()->set_g(0.75);
  txt_marker->mutable_color()->set_b(0.8);
  txt_marker->set_text(std::to_string(static_cast<int>(occ_road.track_id)));
  txt_marker->mutable_scale()->set_x(2);
  txt_marker->mutable_scale()->set_y(2);
  txt_marker->mutable_scale()->set_z(2);
}

MarkerArrayPtr ElementMapToMarkers(const em::ElementMap& map,
                                   const std::string& frame_id,
                                   const std::string& ns, double stamp,
                                   double lifetime) {
  auto ma = std::make_shared<adsfi_proto::viz::MarkerArray>();
  std::vector<em::BoundaryNode::Ptr> nodes;
  for (const auto& it : map.boundary_nodes) {
    if (!it.second) {
      HLOG_ERROR << "nullptr node";
      continue;
    }
    nodes.emplace_back(it.second);
  }

  if (!nodes.empty()) {
    auto* mn = ma->add_markers();
    ElementBoundaryNodesToMarker(nodes, frame_id, ns, 0, stamp, lifetime,
                                 viz::GREEN, mn);
  }

  static int idd = 0;
  std::vector<em::BoundaryNode::Ptr> untracked_nodes;
  for (const auto& it : map.boundaries) {
    if (!it.second) {
      HLOG_ERROR << "nullptr boundary";
      continue;
    }
    // auto id = RetrieveNumInId<uint64_t>(it.second->id);
    auto id = it.second->id;
    auto* mb = ma->add_markers();

    if (it.second->linetype ==
        em::LineType::LaneType_INTERSECTION_VIRTUAL_MARKING) {
      ElementBoundaryToMarker(*it.second, frame_id, ns,
                              static_cast<int32_t>(idd++), stamp, lifetime,
                              viz::YELLOW, mb);
    } else {
      ElementBoundaryToMarker(*it.second, frame_id, ns,
                              static_cast<int32_t>(idd++), stamp, lifetime,
                              viz::GREY, mb);
    }

    for (const auto& n : it.second->nodes) {
      if (map.boundary_nodes.find(n->id) != map.boundary_nodes.end()) {
        continue;
      }
      untracked_nodes.emplace_back(n);
    }
  }

  for (const auto& occ : map.occ_roads) {
    if (!occ.second) {
      HLOG_ERROR << "nullptr occ";
      continue;
    }
    ElementOccRoadToMarker(*occ.second, frame_id, ns,
                           static_cast<int32_t>(idd++), stamp, lifetime,
                           viz::BLUE, ma);
  }
  for (const auto& it : map.boundaries) {
    if (!it.second) {
      HLOG_ERROR << "nullptr boundary";
      continue;
    }
    // int id = it.second->id;
    auto* md = ma->add_markers();
    ElementOccEgoToMarker(*it.second, frame_id, ns, static_cast<int32_t>(idd++),
                          stamp, lifetime, viz::RED, md);
  }
  if (!untracked_nodes.empty()) {
    auto* mn = ma->add_markers();
    ElementBoundaryNodesToMarker(nodes, frame_id, ns, -1, stamp, lifetime,
                                 viz::RED, mn);
  }

  if (ma->markers().empty()) {
    return nullptr;
  }
  return ma;
}

PoseArrayPtr PosesToPoseArray(const std::vector<Pose>& poses,
                              const std::string& frame_id, double stamp) {
  auto pose_array = std::make_shared<adsfi_proto::viz::PoseArray>();
  pose_array->mutable_header()->set_frameid(frame_id);
  uint32_t sec = 0;
  uint32_t nsec = 0;
  SplitSeconds(stamp, &sec, &nsec);
  pose_array->mutable_header()->mutable_timestamp()->set_sec(sec);
  pose_array->mutable_header()->mutable_timestamp()->set_nsec(nsec);
  for (const auto& p : poses) {
    auto* po = pose_array->add_poses();
    po->mutable_position()->set_x(p.pos.x());
    po->mutable_position()->set_y(p.pos.y());
    po->mutable_position()->set_z(p.pos.z());
    po->mutable_orientation()->set_w(p.quat.w());
    po->mutable_orientation()->set_x(p.quat.x());
    po->mutable_orientation()->set_y(p.quat.y());
    po->mutable_orientation()->set_z(p.quat.z());
  }
  return pose_array;
}

}  // namespace viz
}  // namespace mf
}  // namespace mp
}  // namespace hozon
