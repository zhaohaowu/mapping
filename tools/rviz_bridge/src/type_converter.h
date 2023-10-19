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
  static void Convert(const adsfi_proto::hz_Adsfi::AlgHeader &proto,
                      std_msgs::Header *ros);
  static void Convert(const adsfi_proto::viz::CompressedImage &proto,
                      sensor_msgs::CompressedImage *ros);
  static void Convert(const adsfi_proto::viz::Odometry &proto,
                      nav_msgs::Odometry *ros);
  static void Convert(const adsfi_proto::viz::Path &proto, nav_msgs::Path *ros);
  static void Convert(const adsfi_proto::viz::TransformStamped &proto,
                      geometry_msgs::TransformStamped *ros);
  static void Convert(const adsfi_proto::viz::Marker &proto,
                      visualization_msgs::Marker *ros);
  static void Convert(const adsfi_proto::viz::MarkerArray &proto,
                      visualization_msgs::MarkerArray *ros);
  static void Convert(const adsfi_proto::viz::TwistStamped &proto,
                      geometry_msgs::TwistStamped *ros);
  static void Convert(const adsfi_proto::viz::PolygonStamped &proto,
                      geometry_msgs::PolygonStamped *ros);
  static void Convert(const adsfi_proto::viz::PointCloud &proto,
                      sensor_msgs::PointCloud *ros);
  static void Convert(const adsfi_proto::viz::PointCloud2 &proto,
                      sensor_msgs::PointCloud2 *ros);
  static void Convert(const adsfi_proto::viz::PoseArray &proto,
                      geometry_msgs::PoseArray *ros);
  static void Convert(const adsfi_proto::viz::OccupancyGrid &proto,
                      nav_msgs::OccupancyGrid *ros);
};

}  // namespace util
}  // namespace mp
}  // namespace hozon
