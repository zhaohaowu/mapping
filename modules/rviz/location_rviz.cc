/***************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： location_rviz.h
 *   author     ： zhaohaowu
 *   date       ： 2024.01
 ******************************************************************************/
#include "modules/rviz/location_rviz.h"

#include <cstddef>

#include "Eigen/src/Core/Matrix.h"
#include "modules/location/pose_estimation/lib/pose_estimate/pose_estimate.h"
#include "modules/util/include/util/rviz_agent/rviz_agent.h"

namespace hozon {
namespace mp {
namespace loc {

std::string LocationRviz::Name() const { return "LocationRviz"; }

// init没有放在构造函数中,因为init用到了其他单例,会有初始化顺序问题
LocationRviz::LocationRviz() = default;

bool LocationRviz::Init() {
  inited_ = true;
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

void LocationRviz::PubFcTf(const Eigen::Affine3d& T_W_V, uint64_t sec,
                           uint64_t nsec, const std::string& topic) {
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
  tf_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
  tf_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  tf_msg.mutable_header()->set_frameid("map");
  tf_msg.set_child_frame_id("base");
  SetXYZ(p, tf_msg.mutable_transform()->mutable_translation());
  SetXYZW(q, tf_msg.mutable_transform()->mutable_rotation());
  RVIZ_AGENT.Publish(topic, tf_msg);
}

void LocationRviz::PubMapLocalmapTf(const Eigen::Affine3d& T_W_V, uint64_t sec,
                                    uint64_t nsec, const std::string& topic) {
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
  tf_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
  tf_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  tf_msg.mutable_header()->set_frameid("map");
  tf_msg.set_child_frame_id("localmap");
  SetXYZ(p, tf_msg.mutable_transform()->mutable_translation());
  SetXYZW(q, tf_msg.mutable_transform()->mutable_rotation());
  RVIZ_AGENT.Publish(topic, tf_msg);
}

void LocationRviz::PubFcPath(const Eigen::Affine3d& T_W_V, uint64_t sec,
                             uint64_t nsec, const std::string& topic) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  Eigen::Vector3d p = T_W_V.translation();
  Eigen::Quaterniond q(T_W_V.rotation());
  static bool register_flag = true;
  if (register_flag) {
    RVIZ_AGENT.Register<adsfi_proto::viz::Path>(topic);
    register_flag = false;
  }
  static adsfi_proto::viz::Path path_msg;
  auto* pose = path_msg.add_poses();
  path_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
  path_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  path_msg.mutable_header()->set_frameid("map");
  pose->mutable_header()->mutable_timestamp()->set_sec(sec);
  pose->mutable_header()->mutable_timestamp()->set_nsec(nsec);
  pose->mutable_header()->set_frameid("map");
  SetXYZ(p, pose->mutable_pose()->mutable_position());
  SetXYZW(q, pose->mutable_pose()->mutable_orientation());
  if (path_msg.poses().size() > 250) {
    path_msg.mutable_poses()->DeleteSubrange(0, 1);
  }
  RVIZ_AGENT.Publish(topic, path_msg);
}

void LocationRviz::PubPerceptionByFc(const Eigen::Affine3d& T_W_V,
                                     const pe::TrackingManager& perception,
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
  adsfi_proto::viz::PointCloud points_msg;
  points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
  points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  points_msg.mutable_header()->set_frameid("map");
  auto* channels = points_msg.add_channels();
  channels->set_name("rgb");
  for (const auto& lane_line : perception.lane_lines) {
    for (const auto& element : lane_line.second.points) {
      Eigen::Vector3d point_vehicle{element.x(), element.y(), element.z()};
      auto point_world = T_W_V * point_vehicle;
      auto* point_msg = points_msg.add_points();
      SetXYZ(point_world, point_msg);
    }
  }
  RVIZ_AGENT.Publish(topic, points_msg);
}

void LocationRviz::PubPerceptionByInput(const Eigen::Affine3d& T_W_V,
                                        const pe::TrackingManager& perception,
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
  adsfi_proto::viz::PointCloud points_msg;
  points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
  points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  points_msg.mutable_header()->set_frameid("map");
  auto* channels = points_msg.add_channels();
  channels->set_name("rgb");
  for (const auto& lane_line : perception.lane_lines) {
    for (const auto& element : lane_line.second.points) {
      Eigen::Vector3d point_vehicle{element.x(), element.y(), element.z()};
      auto point_world = T_W_V * point_vehicle;
      auto* point_msg = points_msg.add_points();
      SetXYZ(point_world, point_msg);
    }
  }
  RVIZ_AGENT.Publish(topic, points_msg);
}

void LocationRviz::PubPerceptionMarkerByFc(
    const Eigen::Affine3d& T_W_V, const pe::TrackingManager& perception,
    uint64_t sec, uint64_t nsec, const std::string& topic) {
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
    point_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
    point_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
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
      auto point_world = T_W_V * point_vehicle;
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
    txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
    txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    txt_marker.mutable_header()->set_frameid("map");
    txt_marker.mutable_pose()->mutable_orientation()->set_x(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_y(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_z(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_w(1.);
    Eigen::Vector3d point_vehicle{lane_line.second.points[0].x(),
                                  lane_line.second.points[0].y(),
                                  lane_line.second.points[0].z()};
    auto point_world = T_W_V * point_vehicle;
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

void LocationRviz::PubPerceptionMarkerReloc(
    const Eigen::Affine3d& T_W_V, const pe::TrackingManager& perception,
    uint64_t sec, uint64_t nsec, const std::string& topic) {
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
    point_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
    point_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    point_marker.set_id(id++);
    point_marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
    point_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
    point_marker.mutable_pose()->mutable_orientation()->set_x(0.);
    point_marker.mutable_pose()->mutable_orientation()->set_y(0.);
    point_marker.mutable_pose()->mutable_orientation()->set_z(0.);
    point_marker.mutable_pose()->mutable_orientation()->set_w(1.);
    point_marker.mutable_scale()->set_x(0.4);
    point_marker.mutable_scale()->set_y(0.4);
    point_marker.mutable_scale()->set_z(0.4);
    point_marker.mutable_lifetime()->set_sec(0);
    point_marker.mutable_lifetime()->set_nsec(200000000);
    point_marker.mutable_color()->set_a(0.5);
    point_marker.mutable_color()->set_r(1);
    point_marker.mutable_color()->set_g(1);
    point_marker.mutable_color()->set_b(0);
    for (const auto& element : lane_line.second.points) {
      Eigen::Vector3d point_vehicle{element.x(), element.y(), element.z()};
      auto point_world = T_W_V * point_vehicle;
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
    txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
    txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    txt_marker.mutable_header()->set_frameid("map");
    txt_marker.mutable_pose()->mutable_orientation()->set_x(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_y(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_z(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_w(1.);
    Eigen::Vector3d point_vehicle{lane_line.second.points[0].x(),
                                  lane_line.second.points[0].y(),
                                  lane_line.second.points[0].z()};
    auto point_world = T_W_V * point_vehicle;
    txt_marker.mutable_pose()->mutable_position()->set_x(point_world.x());
    txt_marker.mutable_pose()->mutable_position()->set_y(point_world.y());
    txt_marker.mutable_pose()->mutable_position()->set_z(2);
    txt_marker.mutable_color()->set_r(1);
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

void LocationRviz::PubHdmap(const Eigen::Affine3d& T_W_V,
                            const pe::MappingManager& hdmap, uint64_t sec,
                            uint64_t nsec, const std::string& topic) {
  if (!inited_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  static bool register_flag = true;
  if (register_flag) {
    RVIZ_AGENT.Register<adsfi_proto::viz::PointCloud>(topic);
    register_flag = false;
  }
  adsfi_proto::viz::PointCloud points_msg;
  points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
  points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  points_msg.mutable_header()->set_frameid("map");
  auto* channels = points_msg.add_channels();
  channels->set_name("rgb");
  for (const auto& lane_line : hdmap.lane_lines) {
    for (const auto& element : lane_line.second.points) {
      Eigen::Vector3d point_vehicle{element.x(), element.y(), element.z()};
      auto point_world = T_W_V * point_vehicle;
      auto* point_msg = points_msg.add_points();
      SetXYZ(point_world, point_msg);
    }
  }
  RVIZ_AGENT.Publish(topic, points_msg);
}

void LocationRviz::PubHdmapMarker(const Eigen::Affine3d& T_W_V,
                                  const pe::MappingManager& hdmap, uint64_t sec,
                                  uint64_t nsec, const std::string& topic) {
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
    point_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
    point_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
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
      auto point_world = T_W_V * point_vehicle;
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
    txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
    txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    txt_marker.mutable_header()->set_frameid("map");
    txt_marker.mutable_pose()->mutable_orientation()->set_x(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_y(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_z(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_w(1.);
    Eigen::Vector3d point_vehicle{lane_line.second.points[0].x(),
                                  lane_line.second.points[0].y(),
                                  lane_line.second.points[0].z()};
    auto point_world = T_W_V * point_vehicle;
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
    point_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
    point_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
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
      auto point_world = T_W_V * point_vehicle;
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
    txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
    txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    txt_marker.mutable_header()->set_frameid("map");
    txt_marker.mutable_pose()->mutable_orientation()->set_x(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_y(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_z(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_w(1.);
    Eigen::Vector3d point_vehicle{lane_line.second.points[0].x(),
                                  lane_line.second.points[0].y(),
                                  lane_line.second.points[0].z()};
    auto point_world = T_W_V * point_vehicle;
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
    int ins_state, double sd_position, int location_state, double timestamp,
    double velocity, double fc_heading, double ins_heading,
    const std::string& conv, double gps_week, double gps_sec, uint64_t sec,
    uint64_t nsec, const std::string& topic) {
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
  text_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
  text_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
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

  text_marker.set_text("location_state: " + std::to_string(location_state) +
                       "\nins_state: " + std::to_string(ins_state) +
                       "\nsd_position: " + ToString(sd_position) +
                       "\ntimestamp: " + ToString(timestamp) +
                       "\nvelocity: " + ToString(velocity * 3.6) + "km/h" +
                       "\nfc_heading: " + ToString(fc_heading) +
                       " ins_heading: " + ToString(ins_heading) +
                       "\nconv (*e-4): " + conv + "\nChinaTime: " + chinaTime);
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

}  // namespace loc
}  // namespace mp
}  // namespace hozon
