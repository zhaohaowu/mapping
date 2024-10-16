/***************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： location_rviz.h
 *   author     ： zhaohaowu
 *   date       ： 2024.01
 ******************************************************************************/
#include "modules/rviz/location_rviz.h"
#include <unistd.h>

#include <cstddef>
#include <thread>

#include "Eigen/src/Core/Matrix.h"
#include "base/utils/log.h"
#include "modules/location/pose_estimation/lib/pose_estimate/pose_estimate.h"
#include "modules/location/pose_estimation/lib/reloc/reloc.hpp"
#include "modules/util/include/util/rviz_agent/rviz_agent.h"

namespace hozon {
namespace mp {
namespace loc {

std::string LocationRviz::Name() const { return "LocationRviz"; }

// init没有放在构造函数中,因为init用到了其他单例,会有初始化顺序问题
LocationRviz::LocationRviz() = default;

LocationRviz::~LocationRviz() {
  loc_rviz_thread_run_ = false;
  if (loc_rviz_thread_.joinable()) {
    loc_rviz_thread_.join();
  }
}

bool LocationRviz::Init() {
  inited_ = true;
  loc_rviz_thread_ = std::thread(&LocationRviz::LocRvizRun, this);
  loc_rviz_thread_run_ = true;
  return true;
}

template <typename T0, typename T1>
void SetXYZ(const T0& t0, T1* const t1) {
  t1->set_x(t0.x());
  t1->set_y(t0.y());
  t1->set_z(t0.z());
}

template <typename T0, typename T1>
void SetXYZW(const T0& t0, T1* const t1) {
  t1->set_w(t0.w());
  t1->set_x(t0.x());
  t1->set_y(t0.y());
  t1->set_z(t0.z());
}

// GPS周数转北京时间（包括毫秒）
void GPSToBJTimeWithMillis(int gpsWeek, double gpsTOW, struct tm* const bjTime,
                           int* const milliseconds) {
  // 计算GPS时间的总秒数
  time_t gpsSeconds =
      static_cast<time_t>(gpsWeek) * 604800 + static_cast<time_t>(gpsTOW) - 18;
  // GPS时间从1980年1月6日开始计算的总秒数
  const time_t GPSTimeOffset = 315964800;  // 1980-01-06 00:00:00 UTC
  // 加上GPS时间偏移量
  gpsSeconds += GPSTimeOffset;
  // 转换为struct tm结构体
  gmtime_r(&gpsSeconds, bjTime);  // 使用gmtime_r以获取UTC时间
  // 北京时间比UTC时间晚8小时
  bjTime->tm_hour += 8;
  // 获取北京时间的毫秒部分
  *milliseconds = static_cast<int>((gpsTOW - static_cast<int>(gpsTOW)) * 1000);
  // 调整到合法的时间范围
  mktime(bjTime);
}

void LocationRviz::PubFcOdom(const Eigen::Affine3d& T_W_V, uint64_t sec,
                             uint64_t nsec, const std::string& topic) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  static bool register_flag = true;
  if (register_flag) {
    RVIZ_AGENT.Register<adsfi_proto::viz::Odometry>(topic);
    register_flag = false;
  }
  Eigen::Vector3d p = T_W_V.translation();
  Eigen::Quaterniond q(T_W_V.rotation());
  adsfi_proto::viz::Odometry odom_msg;
  odom_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
  odom_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  odom_msg.mutable_header()->set_frameid("map");
  odom_msg.set_child_frame_id("base");
  SetXYZ(p, odom_msg.mutable_pose()->mutable_pose()->mutable_position());
  SetXYZW(q, odom_msg.mutable_pose()->mutable_pose()->mutable_orientation());
  for (size_t i = 0; i < 36; ++i) {
    odom_msg.mutable_pose()->add_covariance(0.);
  }
  RVIZ_AGENT.Publish(topic, odom_msg);
}

void LocationRviz::PubInsEstimateOdom(const Eigen::Affine3d& T_W_V,
                                      uint64_t sec, uint64_t nsec,
                                      const std::string& topic) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  static bool register_flag = true;
  if (register_flag) {
    RVIZ_AGENT.Register<adsfi_proto::viz::Odometry>(topic);
    register_flag = false;
  }
  Eigen::Vector3d p = T_W_V.translation();
  Eigen::Quaterniond q(T_W_V.rotation());
  adsfi_proto::viz::Odometry odom_msg;
  odom_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
  odom_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  odom_msg.mutable_header()->set_frameid("map");
  odom_msg.set_child_frame_id("base");
  SetXYZ(p, odom_msg.mutable_pose()->mutable_pose()->mutable_position());
  SetXYZW(q, odom_msg.mutable_pose()->mutable_pose()->mutable_orientation());
  for (size_t i = 0; i < 36; ++i) {
    odom_msg.mutable_pose()->add_covariance(0.);
  }
  RVIZ_AGENT.Publish(topic, odom_msg);
}

void LocationRviz::PubMmOdom(const Eigen::Affine3d& T_W_V, uint64_t sec,
                             uint64_t nsec, const std::string& topic) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  static bool register_flag = true;
  if (register_flag) {
    RVIZ_AGENT.Register<adsfi_proto::viz::Odometry>(topic);
    register_flag = false;
  }
  Eigen::Vector3d p = T_W_V.translation();
  Eigen::Quaterniond q(T_W_V.rotation());
  adsfi_proto::viz::Odometry odom_msg;
  odom_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
  odom_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  odom_msg.mutable_header()->set_frameid("map");
  odom_msg.set_child_frame_id("base");
  SetXYZ(p, odom_msg.mutable_pose()->mutable_pose()->mutable_position());
  SetXYZW(q, odom_msg.mutable_pose()->mutable_pose()->mutable_orientation());
  for (size_t i = 0; i < 36; ++i) {
    odom_msg.mutable_pose()->add_covariance(0.);
  }
  RVIZ_AGENT.Publish(topic, odom_msg);
}

void LocationRviz::PubDrOdom(const Eigen::Affine3d& T_W_V, uint64_t sec,
                             uint64_t nsec, const std::string& topic) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  static bool register_flag = true;
  if (register_flag) {
    RVIZ_AGENT.Register<adsfi_proto::viz::Odometry>(topic);
    register_flag = false;
  }
  Eigen::Vector3d p = T_W_V.translation();
  Eigen::Quaterniond q(T_W_V.rotation());
  adsfi_proto::viz::Odometry odom_msg;
  odom_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
  odom_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  odom_msg.mutable_header()->set_frameid("localmap");
  odom_msg.set_child_frame_id("base");
  SetXYZ(p, odom_msg.mutable_pose()->mutable_pose()->mutable_position());
  SetXYZW(q, odom_msg.mutable_pose()->mutable_pose()->mutable_orientation());
  for (size_t i = 0; i < 36; ++i) {
    odom_msg.mutable_pose()->add_covariance(0.);
  }
  RVIZ_AGENT.Publish(topic, odom_msg);
}

void LocationRviz::PubInsOdom(const Eigen::Affine3d& T_W_V, uint64_t sec,
                              uint64_t nsec, const std::string& topic) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  static bool register_flag = true;
  if (register_flag) {
    RVIZ_AGENT.Register<adsfi_proto::viz::Odometry>(topic);
    register_flag = false;
  }
  Eigen::Vector3d p = T_W_V.translation();
  Eigen::Quaterniond q(T_W_V.rotation());
  adsfi_proto::viz::Odometry odom_msg;
  odom_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
  odom_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  odom_msg.mutable_header()->set_frameid("map");
  odom_msg.set_child_frame_id("base");
  SetXYZ(p, odom_msg.mutable_pose()->mutable_pose()->mutable_position());
  SetXYZW(q, odom_msg.mutable_pose()->mutable_pose()->mutable_orientation());
  for (size_t i = 0; i < 36; ++i) {
    odom_msg.mutable_pose()->add_covariance(0.);
  }
  RVIZ_AGENT.Publish(topic, odom_msg);
}

void LocationRviz::PubInputOdom(const Eigen::Affine3d& T_W_V, uint64_t sec,
                                uint64_t nsec, const std::string& topic) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  static bool register_flag = true;
  if (register_flag) {
    RVIZ_AGENT.Register<adsfi_proto::viz::Odometry>(topic);
    register_flag = false;
  }
  Eigen::Vector3d p = T_W_V.translation();
  Eigen::Quaterniond q(T_W_V.rotation());
  adsfi_proto::viz::Odometry odom_msg;
  odom_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
  odom_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  odom_msg.mutable_header()->set_frameid("map");
  odom_msg.set_child_frame_id("base");
  SetXYZ(p, odom_msg.mutable_pose()->mutable_pose()->mutable_position());
  SetXYZW(q, odom_msg.mutable_pose()->mutable_pose()->mutable_orientation());
  for (size_t i = 0; i < 36; ++i) {
    odom_msg.mutable_pose()->add_covariance(0.);
  }
  RVIZ_AGENT.Publish(topic, odom_msg);
}

void LocationRviz::PubRelocOdom(const Eigen::Affine3d& T_W_V, uint64_t sec,
                                uint64_t nsec, const std::string& topic) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  static bool register_flag = true;
  if (register_flag) {
    RVIZ_AGENT.Register<adsfi_proto::viz::Odometry>(topic);
    register_flag = false;
  }
  Eigen::Vector3d p = T_W_V.translation();
  Eigen::Quaterniond q(T_W_V.rotation());
  adsfi_proto::viz::Odometry odom_msg;
  odom_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
  odom_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  odom_msg.mutable_header()->set_frameid("map");
  odom_msg.set_child_frame_id("base");
  SetXYZ(p, odom_msg.mutable_pose()->mutable_pose()->mutable_position());
  SetXYZW(q, odom_msg.mutable_pose()->mutable_pose()->mutable_orientation());
  for (size_t i = 0; i < 36; ++i) {
    odom_msg.mutable_pose()->add_covariance(0.);
  }
  RVIZ_AGENT.Publish(topic, odom_msg);
}

void LocationRviz::PubFcTf(const Eigen::Affine3d& T_W_V,
                           const std::string& topic) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  Eigen::Vector3d p = T_W_V.translation();
  Eigen::Quaterniond q(T_W_V.rotation());
  static bool register_flag = true;
  if (register_flag) {
    RVIZ_AGENT.Register<adsfi_proto::viz::TransformStamped>(topic);
    register_flag = false;
  }
  int seq = 0;
  adsfi_proto::viz::TransformStamped tf_msg;
  tf_msg.mutable_header()->set_seq(seq++);
  tf_msg.mutable_header()->mutable_timestamp()->set_sec(sec_);
  tf_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec_);
  tf_msg.mutable_header()->set_frameid("map");
  tf_msg.set_child_frame_id("base");
  SetXYZ(p, tf_msg.mutable_transform()->mutable_translation());
  SetXYZW(q, tf_msg.mutable_transform()->mutable_rotation());
  RVIZ_AGENT.Publish(topic, tf_msg);
}

void LocationRviz::PubPerceptionByInput(const Eigen::Affine3d& T_input,
                                        const pe::TrackingManager& perception,
                                        const std::string& topic) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  static bool register_flag = true;
  if (register_flag) {
    RVIZ_AGENT.Register<adsfi_proto::viz::PointCloud>(topic);
    register_flag = false;
  }
  adsfi_proto::viz::PointCloud points_msg;
  points_msg.mutable_header()->mutable_timestamp()->set_sec(sec_);
  points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec_);
  points_msg.mutable_header()->set_frameid("map");
  auto* channels = points_msg.add_channels();
  channels->set_name("rgb");
  for (const auto& lane_line : perception.lane_lines) {
    for (const auto& element : lane_line.second.points) {
      Eigen::Vector3d point_vehicle{element.x(), element.y(), element.z()};
      auto point_world = T_input * point_vehicle;
      auto* point_msg = points_msg.add_points();
      SetXYZ(point_world, point_msg);
    }
  }
  RVIZ_AGENT.Publish(topic, points_msg);
}

void LocationRviz::PubPerceptionMarkerByFc(
    const Eigen::Affine3d& T_fc_10hz, const pe::TrackingManager& perception,
    const std::string& topic) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  static bool register_flag = true;
  if (register_flag) {
    RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(topic);
    register_flag = false;
  }
  adsfi_proto::viz::MarkerArray markers;
  int id = 0;
  for (const auto& lane_line : perception.lane_lines) {
    if (lane_line.second.points.empty()) {
      continue;
    }
    adsfi_proto::viz::Marker point_marker;
    point_marker.mutable_header()->set_frameid("map");
    point_marker.mutable_header()->mutable_timestamp()->set_sec(sec_);
    point_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec_);
    point_marker.set_id(id++);
    point_marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
    point_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
    point_marker.mutable_pose()->mutable_orientation()->set_x(0.);
    point_marker.mutable_pose()->mutable_orientation()->set_y(0.);
    point_marker.mutable_pose()->mutable_orientation()->set_z(0.);
    point_marker.mutable_pose()->mutable_orientation()->set_w(1.);
    point_marker.mutable_scale()->set_x(0.05);
    point_marker.mutable_scale()->set_y(0.05);
    point_marker.mutable_scale()->set_z(0.05);
    point_marker.mutable_lifetime()->set_sec(0);
    point_marker.mutable_lifetime()->set_nsec(200000000);
    point_marker.mutable_color()->set_a(1);
    point_marker.mutable_color()->set_r(1);
    point_marker.mutable_color()->set_g(0);
    point_marker.mutable_color()->set_b(0);
    for (const auto& element : lane_line.second.points) {
      Eigen::Vector3d point_vehicle{element.x(), element.y(), element.z()};
      auto point_world = T_fc_10hz * point_vehicle;
      auto* point_msg = point_marker.add_points();
      SetXYZ(point_world, point_msg);
    }
    markers.add_markers()->CopyFrom(point_marker);

    adsfi_proto::viz::Marker txt_marker;
    txt_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
    txt_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
    txt_marker.set_id(id++);
    txt_marker.mutable_lifetime()->set_sec(0);
    txt_marker.mutable_lifetime()->set_nsec(200000000);
    txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec_);
    txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec_);
    txt_marker.mutable_header()->set_frameid("map");
    txt_marker.mutable_pose()->mutable_orientation()->set_x(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_y(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_z(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_w(1.);
    Eigen::Vector3d point_vehicle{lane_line.second.points[0].x(),
                                  lane_line.second.points[0].y(),
                                  lane_line.second.points[0].z()};
    auto point_world = T_fc_10hz * point_vehicle;
    txt_marker.mutable_pose()->mutable_position()->set_x(point_world.x());
    txt_marker.mutable_pose()->mutable_position()->set_y(point_world.y());
    txt_marker.mutable_pose()->mutable_position()->set_z(2);
    txt_marker.mutable_color()->set_r(1);
    txt_marker.mutable_color()->set_g(0);
    txt_marker.mutable_color()->set_b(0);
    txt_marker.mutable_color()->set_a(1);
    std::string txt;
    switch (lane_line.second.lane_type) {
      case pe::UNKNOWN:
        txt = "unknown ";
        break;
      case pe::SOLID_LINE:
        txt = "solid_line ";
        break;
      case pe::DASHED_LINE:
        txt = "dashed_line ";
        break;
      case pe::Road_Edge:
        txt = "road_edge ";
        break;
    }
    txt_marker.set_text(txt + std::to_string(lane_line.first));
    txt_marker.mutable_scale()->set_x(0.5);
    txt_marker.mutable_scale()->set_y(0.5);
    txt_marker.mutable_scale()->set_z(0.5);
    markers.add_markers()->CopyFrom(txt_marker);
  }
  RVIZ_AGENT.Publish(topic, markers);
}

void LocationRviz::PubHdmapMarker(const Eigen::Affine3d& T_fc_10hz,
                                  const pe::MappingManager& hdmap,
                                  const std::string& topic) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  static bool register_flag = true;
  if (register_flag) {
    RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(topic);
    register_flag = false;
  }
  adsfi_proto::viz::MarkerArray markers;
  int id = 0;
  for (const auto& lane_line : hdmap.lane_lines) {
    if (lane_line.second.lane_type == pe::Road_Edge) {
      continue;
    }
    if (lane_line.second.points.empty()) {
      continue;
    }
    adsfi_proto::viz::Marker point_marker;
    point_marker.mutable_header()->set_frameid("map");
    point_marker.mutable_header()->mutable_timestamp()->set_sec(sec_);
    point_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec_);
    point_marker.set_id(id++);
    point_marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
    point_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
    point_marker.mutable_pose()->mutable_orientation()->set_x(0.);
    point_marker.mutable_pose()->mutable_orientation()->set_y(0.);
    point_marker.mutable_pose()->mutable_orientation()->set_z(0.);
    point_marker.mutable_pose()->mutable_orientation()->set_w(1.);
    point_marker.mutable_scale()->set_x(0.05);
    point_marker.mutable_scale()->set_y(0.05);
    point_marker.mutable_scale()->set_z(0.05);
    point_marker.mutable_lifetime()->set_sec(0);
    point_marker.mutable_lifetime()->set_nsec(200000000);
    point_marker.mutable_color()->set_a(1.0);
    point_marker.mutable_color()->set_r(1);
    point_marker.mutable_color()->set_g(1);
    point_marker.mutable_color()->set_b(1);
    for (const auto& element : lane_line.second.points) {
      Eigen::Vector3d point_vehicle{element.x(), element.y(), element.z()};
      auto point_world = T_fc_10hz * point_vehicle;
      auto* point_msg = point_marker.add_points();
      SetXYZ(point_world, point_msg);
    }
    markers.add_markers()->CopyFrom(point_marker);

    adsfi_proto::viz::Marker txt_marker;
    txt_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
    txt_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
    txt_marker.set_id(id++);
    txt_marker.mutable_lifetime()->set_sec(0);
    txt_marker.mutable_lifetime()->set_nsec(200000000);
    txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec_);
    txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec_);
    txt_marker.mutable_header()->set_frameid("map");
    txt_marker.mutable_pose()->mutable_orientation()->set_x(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_y(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_z(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_w(1.);
    Eigen::Vector3d point_vehicle{lane_line.second.points[0].x(),
                                  lane_line.second.points[0].y(),
                                  lane_line.second.points[0].z()};
    auto point_world = T_fc_10hz * point_vehicle;
    txt_marker.mutable_pose()->mutable_position()->set_x(point_world.x());
    txt_marker.mutable_pose()->mutable_position()->set_y(point_world.y());
    txt_marker.mutable_pose()->mutable_position()->set_z(2);
    txt_marker.mutable_color()->set_r(0);
    txt_marker.mutable_color()->set_g(1);
    txt_marker.mutable_color()->set_b(0);
    txt_marker.mutable_color()->set_a(1);
    std::string txt;
    switch (lane_line.second.lane_type) {
      case pe::UNKNOWN:
        txt = "unknown ";
        break;
      case pe::SOLID_LINE:
        txt = "solid_line ";
        break;
      case pe::DASHED_LINE:
        txt = "dashed_line ";
        break;
      case pe::Road_Edge:
        txt = "road_edge ";
        break;
    }
    txt_marker.set_text(txt + std::to_string(lane_line.first));
    txt_marker.mutable_scale()->set_x(0.5);
    txt_marker.mutable_scale()->set_y(0.5);
    txt_marker.mutable_scale()->set_z(0.5);
    markers.add_markers()->CopyFrom(txt_marker);
  }
  for (const auto& lane_line : hdmap.lane_lines) {
    if (lane_line.second.lane_type != pe::Road_Edge) {
      continue;
    }
    if (lane_line.second.points.empty()) {
      continue;
    }
    adsfi_proto::viz::Marker point_marker;
    point_marker.mutable_header()->set_frameid("map");
    point_marker.mutable_header()->mutable_timestamp()->set_sec(sec_);
    point_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec_);
    point_marker.set_id(id++);
    point_marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
    point_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
    point_marker.mutable_pose()->mutable_orientation()->set_x(0.);
    point_marker.mutable_pose()->mutable_orientation()->set_y(0.);
    point_marker.mutable_pose()->mutable_orientation()->set_z(0.);
    point_marker.mutable_pose()->mutable_orientation()->set_w(1.);
    point_marker.mutable_scale()->set_x(0.05);
    point_marker.mutable_scale()->set_y(0.05);
    point_marker.mutable_scale()->set_z(0.05);
    point_marker.mutable_lifetime()->set_sec(0);
    point_marker.mutable_lifetime()->set_nsec(200000000);
    point_marker.mutable_color()->set_a(1.0);
    point_marker.mutable_color()->set_r(0.5);
    point_marker.mutable_color()->set_g(0.5);
    point_marker.mutable_color()->set_b(0.5);
    for (const auto& element : lane_line.second.points) {
      Eigen::Vector3d point_vehicle{element.x(), element.y(), element.z()};
      auto point_world = T_fc_10hz * point_vehicle;
      auto* point_msg = point_marker.add_points();
      SetXYZ(point_world, point_msg);
    }
    markers.add_markers()->CopyFrom(point_marker);

    adsfi_proto::viz::Marker txt_marker;
    txt_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
    txt_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
    txt_marker.set_id(id++);
    txt_marker.mutable_lifetime()->set_sec(0);
    txt_marker.mutable_lifetime()->set_nsec(200000000);
    txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec_);
    txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec_);
    txt_marker.mutable_header()->set_frameid("map");
    txt_marker.mutable_pose()->mutable_orientation()->set_x(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_y(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_z(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_w(1.);
    Eigen::Vector3d point_vehicle{lane_line.second.points[0].x(),
                                  lane_line.second.points[0].y(),
                                  lane_line.second.points[0].z()};
    auto point_world = T_fc_10hz * point_vehicle;
    txt_marker.mutable_pose()->mutable_position()->set_x(point_world.x());
    txt_marker.mutable_pose()->mutable_position()->set_y(point_world.y());
    txt_marker.mutable_pose()->mutable_position()->set_z(2);
    txt_marker.mutable_color()->set_r(0);
    txt_marker.mutable_color()->set_g(1);
    txt_marker.mutable_color()->set_b(0);
    txt_marker.mutable_color()->set_a(1);
    std::string txt;
    switch (lane_line.second.lane_type) {
      case pe::UNKNOWN:
        txt = "unknown ";
        break;
      case pe::SOLID_LINE:
        txt = "solid_line ";
        break;
      case pe::DASHED_LINE:
        txt = "dashed_line ";
        break;
      case pe::Road_Edge:
        txt = "road_edge ";
        break;
    }
    txt_marker.set_text(txt + std::to_string(lane_line.first));
    txt_marker.mutable_scale()->set_x(0.5);
    txt_marker.mutable_scale()->set_y(0.5);
    txt_marker.mutable_scale()->set_z(0.5);
    markers.add_markers()->CopyFrom(txt_marker);
  }
  RVIZ_AGENT.Publish(topic, markers);
}

void LocationRviz::PubInsLocationState(
    int ins_state, double ins_sd_position, double ins_height,
    double ins_heading, double gps_week, double gps_sec, int fc_state,
    double velocity_vrf, double fc_heading, const std::string& fc_conv,
    bool mm_valid, int warn_info, double per_time, const std::string& topic) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  static bool register_flag = true;
  if (register_flag) {
    RVIZ_AGENT.Register<adsfi_proto::viz::Marker>(topic);
    register_flag = false;
  }
  adsfi_proto::viz::Marker text_marker;
  text_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
  text_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
  text_marker.set_id(0);
  text_marker.mutable_lifetime()->set_sec(0);
  text_marker.mutable_header()->mutable_timestamp()->set_sec(sec_);
  text_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec_);
  text_marker.mutable_header()->set_frameid("base");
  text_marker.mutable_pose()->mutable_position()->set_x(-5);
  text_marker.mutable_pose()->mutable_position()->set_y(0);
  text_marker.mutable_pose()->mutable_position()->set_z(0);
  text_marker.mutable_pose()->mutable_orientation()->set_x(0);
  text_marker.mutable_pose()->mutable_orientation()->set_y(0);
  text_marker.mutable_pose()->mutable_orientation()->set_z(0);
  text_marker.mutable_pose()->mutable_orientation()->set_w(1);
  text_marker.mutable_color()->set_r(1);
  text_marker.mutable_color()->set_g(1);
  text_marker.mutable_color()->set_b(1);
  text_marker.mutable_color()->set_a(1);
  auto ToString = [](double value) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << value;
    return ss.str();
  };
  struct tm bjTime;
  int milliseconds = 0;
  GPSToBJTimeWithMillis(static_cast<int>(gps_week), gps_sec, &bjTime,
                        &milliseconds);
  char chinaTime[27] = {0};
  snprintf(chinaTime, sizeof(chinaTime), "%04d-%02d-%02d %02d:%02d:%.3f",
           bjTime.tm_year + 1900, bjTime.tm_mon + 1, bjTime.tm_mday,
           bjTime.tm_hour, bjTime.tm_min,
           bjTime.tm_sec + milliseconds / 1000.0);

  text_marker.set_text(
      "ins_state: " + std::to_string(ins_state) +
      "\nins_sd_position: " + ToString(ins_sd_position) +
      "\nins_height: " + ToString(ins_height) + "\nChinaTime: " + chinaTime +

      "\nfc_state: " + std::to_string(fc_state) +
      "\nvelocity: " + ToString(velocity_vrf * 3.6) + "km/h" +
      "\nfc_heading: " + ToString(fc_heading) + " ins_heading: " +
      ToString(ins_heading) + "\nfc_conv (*e-4): " + fc_conv +

      "\nmm_valid " + std::to_string(static_cast<int>(mm_valid)) +
      "\nwarn_info " + std::to_string(warn_info) +

      "\nperception_time: " + ToString(per_time));
  text_marker.mutable_scale()->set_x(0.1);
  text_marker.mutable_scale()->set_y(0);
  text_marker.mutable_scale()->set_z(0.8);
  RVIZ_AGENT.Publish(topic, text_marker);
}

void LocationRviz::PubOriginConnectMapPoints(
    const hozon::mp::loc::Connect& origin_connect, uint64_t sec, uint64_t nsec,
    const std::string& topic) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  static bool register_flag = true;
  if (register_flag) {
    RVIZ_AGENT.Register<adsfi_proto::viz::PointCloud>(topic);
    register_flag = false;
  }
  adsfi_proto::viz::PointCloud lane_points;
  int curr_seq = 0;
  lane_points.mutable_header()->set_seq(curr_seq++);
  lane_points.mutable_header()->mutable_timestamp()->set_sec(sec);
  lane_points.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  lane_points.mutable_header()->set_frameid("map");
  auto* channels = lane_points.add_channels();
  channels->set_name("rgb");
  for (const auto& pair : origin_connect.lane_line_match_pairs) {
    auto* points_ = lane_points.add_points();
    points_->set_x(static_cast<float>(pair.map_pw.x()));
    points_->set_y(static_cast<float>(pair.map_pw.y()));
    points_->set_z(static_cast<float>(pair.map_pw.z()));
  }
  RVIZ_AGENT.Publish(topic, lane_points);
}

void LocationRviz::PubOriginConnectPercepPoints(
    const hozon::mp::loc::Connect& origin_connect, const Eigen::Affine3d& T_W_V,
    uint64_t sec, uint64_t nsec, const std::string& topic) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  static bool register_flag = true;
  if (register_flag) {
    RVIZ_AGENT.Register<adsfi_proto::viz::PointCloud>(topic);
    register_flag = false;
  }
  adsfi_proto::viz::PointCloud lane_points;
  int curr_seq = 0;
  lane_points.mutable_header()->set_seq(curr_seq++);
  lane_points.mutable_header()->mutable_timestamp()->set_sec(sec);
  lane_points.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  lane_points.mutable_header()->set_frameid("map");
  auto* channels = lane_points.add_channels();
  channels->set_name("rgb");
  for (const auto& pair : origin_connect.lane_line_match_pairs) {
    auto* points_ = lane_points.add_points();
    auto p = T_W_V * pair.pecep_pv;
    points_->set_x(static_cast<float>(p.x()));
    points_->set_y(static_cast<float>(p.y()));
    points_->set_z(static_cast<float>(p.z()));
  }
  RVIZ_AGENT.Publish(topic, lane_points);
}

void LocationRviz::PubConnectMapPoints(const hozon::mp::loc::Connect& connect,
                                       uint64_t sec, uint64_t nsec,
                                       const std::string& topic) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  static bool register_flag = true;
  if (register_flag) {
    RVIZ_AGENT.Register<adsfi_proto::viz::PointCloud>(topic);
    register_flag = false;
  }
  adsfi_proto::viz::PointCloud lane_points;
  int curr_seq = 0;
  lane_points.mutable_header()->set_seq(curr_seq++);
  lane_points.mutable_header()->mutable_timestamp()->set_sec(sec);
  lane_points.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  lane_points.mutable_header()->set_frameid("map");
  auto* channels = lane_points.add_channels();
  channels->set_name("rgb");
  for (const auto& pair : connect.lane_line_match_pairs) {
    auto* points_ = lane_points.add_points();
    points_->set_x(static_cast<float>(pair.map_pw.x()));
    points_->set_y(static_cast<float>(pair.map_pw.y()));
    points_->set_z(static_cast<float>(pair.map_pw.z()));
  }
  RVIZ_AGENT.Publish(topic, lane_points);
}

void LocationRviz::PubConnectPercepPoints(
    const hozon::mp::loc::Connect& connect, const Eigen::Affine3d& T_W_V,
    uint64_t sec, uint64_t nsec, const std::string& topic) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  static bool register_flag = true;
  if (register_flag) {
    RVIZ_AGENT.Register<adsfi_proto::viz::PointCloud>(topic);
    register_flag = false;
  }
  adsfi_proto::viz::PointCloud lane_points;
  int curr_seq = 0;
  lane_points.mutable_header()->set_seq(curr_seq++);
  lane_points.mutable_header()->mutable_timestamp()->set_sec(sec);
  lane_points.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  lane_points.mutable_header()->set_frameid("map");
  auto* channels = lane_points.add_channels();
  channels->set_name("rgb");
  for (const auto& pair : connect.lane_line_match_pairs) {
    auto* points_ = lane_points.add_points();
    auto p = T_W_V * pair.pecep_pv;
    points_->set_x(static_cast<float>(p.x()));
    points_->set_y(static_cast<float>(p.y()));
    points_->set_z(static_cast<float>(p.z()));
  }
  RVIZ_AGENT.Publish(topic, lane_points);
}

void LocationRviz::PubMergeMapLines(
    const std::unordered_map<std::string, std::vector<ControlPoint>>&
        merged_map_lines,
    const Eigen::Affine3d& T_W_V, uint64_t sec, uint64_t nsec,
    const std::string& topic) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  static bool register_flag = true;
  if (register_flag) {
    RVIZ_AGENT.Register<adsfi_proto::viz::PointCloud>(topic);
    register_flag = false;
  }
  adsfi_proto::viz::PointCloud lane_points;
  int curr_seq = 0;
  lane_points.mutable_header()->set_seq(curr_seq++);
  lane_points.mutable_header()->mutable_timestamp()->set_sec(sec);
  lane_points.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  lane_points.mutable_header()->set_frameid("map");
  auto* channels = lane_points.add_channels();
  channels->set_name("rgb");
  for (const auto& line : merged_map_lines) {
    for (const auto& ponit : line.second) {
      auto* points_ = lane_points.add_points();
      auto p = T_W_V * ponit.point;
      points_->set_x(static_cast<float>(p.x()));
      points_->set_y(static_cast<float>(p.y()));
      points_->set_z(static_cast<float>(p.z()));
    }
  }
  RVIZ_AGENT.Publish(topic, lane_points);
}

void LocationRviz::SetHdmap(const pe::MappingManager& hd_map) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  loc_rviz_mutex_.lock();
  hd_map_ = hd_map;
  loc_rviz_mutex_.unlock();
}

void LocationRviz::SetPerception(const pe::TrackingManager& perception) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  loc_rviz_mutex_.lock();
  perception_ = perception;
  loc_rviz_mutex_.unlock();
}

void LocationRviz::SetFcTf(const Eigen::Affine3d& T_fc_100hz) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  loc_rviz_mutex_.lock();
  T_fc_100hz_ = T_fc_100hz;
  loc_rviz_mutex_.unlock();
}

void LocationRviz::SetFc(const Eigen::Affine3d& T_fc_10hz) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  loc_rviz_mutex_.lock();
  T_fc_10hz_ = T_fc_10hz;
  loc_rviz_mutex_.unlock();
}

void LocationRviz::SetInputPose(const Eigen::Affine3d& T_input) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  loc_rviz_mutex_.lock();
  T_input_ = T_input;
  loc_rviz_mutex_.unlock();
}

void LocationRviz::SetInsState(int ins_state) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  loc_rviz_mutex_.lock();
  ins_state_ = ins_state;
  loc_rviz_mutex_.unlock();
}

void LocationRviz::SetInsSdPosition(double ins_sd_position) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  loc_rviz_mutex_.lock();
  ins_sd_position_ = ins_sd_position;
  loc_rviz_mutex_.unlock();
}

void LocationRviz::SetInsHeight(double ins_height) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  loc_rviz_mutex_.lock();
  ins_height_ = ins_height;
  loc_rviz_mutex_.unlock();
}

void LocationRviz::SetInsHeading(double ins_heading) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  loc_rviz_mutex_.lock();
  ins_heading_ = ins_heading;
  loc_rviz_mutex_.unlock();
}

void LocationRviz::SetGpsSec(double gps_sec) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  loc_rviz_mutex_.lock();
  gps_sec_ = gps_sec;
  loc_rviz_mutex_.unlock();
}

void LocationRviz::SetGpsWeek(double gps_week) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  loc_rviz_mutex_.lock();
  gps_week_ = gps_week;
  loc_rviz_mutex_.unlock();
}

void LocationRviz::SetFcState(int fc_state) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  loc_rviz_mutex_.lock();
  fc_state_ = fc_state;
  loc_rviz_mutex_.unlock();
}

void LocationRviz::SetVelocityVrf(double velocity_vrf) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  loc_rviz_mutex_.lock();
  velocity_vrf_ = velocity_vrf;
  loc_rviz_mutex_.unlock();
}

void LocationRviz::SetFcHeading(double fc_heading) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  loc_rviz_mutex_.lock();
  fc_heading_ = fc_heading;
  loc_rviz_mutex_.unlock();
}

void LocationRviz::SetFcConv(const std::string& fc_conv) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  loc_rviz_mutex_.lock();
  fc_conv_ = fc_conv;
  loc_rviz_mutex_.unlock();
}

void LocationRviz::SetWarnInfo(int warn_info) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  loc_rviz_mutex_.lock();
  warn_info_ = warn_info;
  loc_rviz_mutex_.unlock();
}

void LocationRviz::SetMmValid(bool mm_valid) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  loc_rviz_mutex_.lock();
  mm_valid_ = mm_valid;
  loc_rviz_mutex_.unlock();
}

void LocationRviz::SetPerTime(double per_time) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  loc_rviz_mutex_.lock();
  per_time_ = per_time;
  loc_rviz_mutex_.unlock();
}

void LocationRviz::LocRvizRun() {
  if (!inited_) {
    return;
  }
  while (loc_rviz_thread_run_) {
    // 获取系统时间
    timespec cur_time{};
    clock_gettime(CLOCK_REALTIME, &cur_time);
    sec_ = cur_time.tv_sec;
    nsec_ = cur_time.tv_nsec;
    // 可视化
    loc_rviz_mutex_.lock();
    auto T_fc_10hz = T_fc_10hz_;
    auto T_fc_100hz = T_fc_100hz_;
    auto T_input = T_input_;
    auto hd_map = hd_map_;
    auto perception = perception_;
    // ins信息
    auto ins_state = ins_state_;
    auto ins_sd_position = ins_sd_position_;
    auto ins_height = ins_height_;
    auto ins_heading = ins_heading_;
    auto gps_week = gps_week_;
    auto gps_sec = gps_sec_;
    // fc信息
    auto fc_state = fc_state_;
    auto velocity_vrf = velocity_vrf_;
    auto fc_heading = fc_heading_;
    auto fc_conv = fc_conv_;
    // mm信息
    auto mm_valid = mm_valid_;
    auto warn_info = warn_info_;
    // 感知时间戳
    auto per_time = per_time_;
    loc_rviz_mutex_.unlock();
    PubHdmapMarker(T_fc_10hz, hd_map, "/pe/hdmap_marker");
    PubFcTf(T_fc_100hz, "/pe/fc_tf");
    PubPerceptionMarkerByFc(T_fc_10hz, perception,
                            "/pe/perception_marker_by_fc");
    PubPerceptionByInput(T_input, perception, "/pe/perception_by_input");
    PubInsLocationState(ins_state, ins_sd_position, ins_height, ins_heading,
                        gps_week, gps_sec, fc_state, velocity_vrf, fc_heading,
                        fc_conv, mm_valid, warn_info, per_time,
                        "/pe/ins_location_state");
    // hd_map_.lane_lines.clear();
    // perception_.lane_lines.clear();
    usleep(1e5);
  }
}

}  // namespace loc
}  // namespace mp
}  // namespace hozon
