/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： type_converter.h
 *   author     ： taoshaoyuan
 *   date       ： 2022.6
 ******************************************************************************/

#pragma once

#include <adsfi_proto/viz/geometry_msgs.pb.h>
#include <adsfi_proto/viz/nav_msgs.pb.h>
#include <adsfi_proto/viz/sensor_msgs.pb.h>
#include <adsfi_proto/viz/tf2_msgs.pb.h>
#include <adsfi_proto/viz/visualization_msgs.pb.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <proto/dead_reckoning/dr.pb.h>
#include <proto/localization/localization.pb.h>
#include <proto/localization/node_info.pb.h>
#include <proto/soc/chassis.pb.h>
#include <proto/soc/sensor_imu_ins.pb.h>
#include <rviz_msgs/Chassis.h>
#include <rviz_msgs/DeadReckoning.h>
#include <rviz_msgs/HafNodeInfo.h>
#include <rviz_msgs/ImuIns.h>
#include <rviz_msgs/Localization.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <vector>

namespace hozon {
namespace mp {
namespace util {

class TypeConverter {
 public:
  template <typename P, typename R>
  static void ConvertXYZ(const P& p, R* r) {
    r->x = p.x();
    r->y = p.y();
    r->z = p.z();
  }

  template <typename P, typename R>
  static void ConvertWXYZ(const P& p, R* r) {
    r->w = p.w();
    r->x = p.x();
    r->y = p.y();
    r->z = p.z();
  }

  static void Convert(const adsfi_proto::hz_Adsfi::AlgHeader& proto,
                      std_msgs::Header* ros);
  static void Convert(const hozon::common::Header& proto,
                      std_msgs::Header* ros);
  static void Convert(const hozon::common::Header& proto,
                      rviz_msgs::Header* ros);
  static void Convert(const adsfi_proto::viz::CompressedImage& proto,
                      sensor_msgs::CompressedImage* ros);
  static void Convert(const adsfi_proto::viz::Odometry& proto,
                      nav_msgs::Odometry* ros);
  static void Convert(const adsfi_proto::viz::Path& proto, nav_msgs::Path* ros);
  static void Convert(const adsfi_proto::viz::TransformStamped& proto,
                      geometry_msgs::TransformStamped* ros);
  static void Convert(const adsfi_proto::viz::Marker& proto,
                      visualization_msgs::Marker* ros);
  static void Convert(const adsfi_proto::viz::MarkerArray& proto,
                      visualization_msgs::MarkerArray* ros);
  static void Convert(const adsfi_proto::viz::TwistStamped& proto,
                      geometry_msgs::TwistStamped* ros);
  static void Convert(const adsfi_proto::viz::PolygonStamped& proto,
                      geometry_msgs::PolygonStamped* ros);
  static void Convert(const adsfi_proto::viz::PointCloud& proto,
                      sensor_msgs::PointCloud* ros);
  static void Convert(const adsfi_proto::viz::PointCloud2& proto,
                      sensor_msgs::PointCloud2* ros);
  static void Convert(const adsfi_proto::viz::PoseArray& proto,
                      geometry_msgs::PoseArray* ros);
  static void Convert(const adsfi_proto::viz::OccupancyGrid& proto,
                      nav_msgs::OccupancyGrid* ros);

  static void Convert(const hozon::common::Pose& proto, rviz_msgs::Pose* ros);
  static void Convert(const hozon::localization::Localization& proto,
                      rviz_msgs::Localization* ros);
  static void Convert(const hozon::localization::HafNodeInfo& proto,
                      rviz_msgs::HafNodeInfo* ros);
  static void Convert(const hozon::soc::ImuInfo& proto,
                      rviz_msgs::ImuInfo* ros);
  static void Convert(const hozon::soc::InsInfo& proto,
                      rviz_msgs::InsInfo* ros);
  static void Convert(const hozon::soc::ImuIns& proto, rviz_msgs::ImuIns* ros);
  static void Convert(const hozon::dead_reckoning::DeadReckoning& proto,
                      rviz_msgs::DeadReckoning* ros);
  static void Convert(const hozon::soc::Chassis& proto,
                      rviz_msgs::Chassis* ros);
};

}  // namespace util
}  // namespace mp
}  // namespace hozon
