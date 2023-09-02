/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： msg_alias.h
 *   author     ： taoshaoyuan
 *   date       ： 2022.10
 ******************************************************************************/

#pragma once

#include <string>

// auto generated
#include <adsfi_proto/viz/geometry_msgs.pb.h>
#include <adsfi_proto/viz/msg_info.pb.h>
#include <adsfi_proto/viz/nav_msgs.pb.h>
#include <adsfi_proto/viz/sensor_msgs.pb.h>
#include <adsfi_proto/viz/tf2_msgs.pb.h>
#include <adsfi_proto/viz/visualization_msgs.pb.h>

/*
 * 能不能设计成这样：所有支持的类型都是在这个头文件里定义，从类型alias直接得到具体的类型；
 * RvizAgent和RvizAgentClient里不体现具体的类型
 */

namespace hozon {
namespace mp {
namespace util {

// type alias
static const std::string kCompressedImage = "CompressedImage";
static const std::string kOdometry = "Odometry";
static const std::string kPath = "Path";
static const std::string kTransformStamped = "TransformStamped";
static const std::string kMarker = "Marker";
static const std::string kMarkerArray = "MarkerArray";
static const std::string kTwistStamped = "TwistStamped";
static const std::string kPolygonStamped = "PolygonStamped";
static const std::string kPointCloud = "PointCloud";
static const std::string kPointCloud2 = "PointCloud2";
static const std::string kPoseArray = "PoseArray";
static const std::string kOccupancyGrid = "OccupancyGrid";

static const std::string kCtrlTopic = "/RvizAgent/Ctrl";

class MetaMsg {
 public:
  MetaMsg() = default;
  MetaMsg(const std::string& topic_, const std::string& data_)
      : topic(topic_), data(data_) {}

  std::string topic;
  std::string data;
};

using ImageCallback = std::function<void(
    const std::string&, std::shared_ptr<adsfi_proto::viz::CompressedImage>)>;
using OdometryCallback = std::function<void(
    const std::string&, std::shared_ptr<adsfi_proto::viz::Odometry>)>;
using PathCallback = std::function<void(
    const std::string&, std::shared_ptr<adsfi_proto::viz::Path>)>;
using TransformStampedCallback = std::function<void(
    const std::string&, std::shared_ptr<adsfi_proto::viz::TransformStamped>)>;
using MarkerCallback = std::function<void(
    const std::string&, std::shared_ptr<adsfi_proto::viz::Marker>)>;
using MarkerArrayCallback = std::function<void(
    const std::string&, std::shared_ptr<adsfi_proto::viz::MarkerArray>)>;
using TwistStampedCallback = std::function<void(
    const std::string&, std::shared_ptr<adsfi_proto::viz::TwistStamped>)>;
using PolygonStampedCallback = std::function<void(
    const std::string&, std::shared_ptr<adsfi_proto::viz::PolygonStamped>)>;
using PointCloudCallback = std::function<void(
    const std::string&, std::shared_ptr<adsfi_proto::viz::PointCloud>)>;
using PointCloud2Callback = std::function<void(
    const std::string&, std::shared_ptr<adsfi_proto::viz::PointCloud2>)>;
using PoseArrayCallback = std::function<void(
    const std::string&, std::shared_ptr<adsfi_proto::viz::PoseArray>)>;
using OccupancyGridCallback = std::function<void(
    const std::string&, std::shared_ptr<adsfi_proto::viz::OccupancyGrid>)>;

#define TYPEDEF_COMMON_TYPES \
  typedef void Void;         \
  typedef int Int;           \
  typedef double Double;     \
  typedef bool Bool;         \
  typedef std::string String;

template <typename T>
struct Checker;

template <>
struct Checker<adsfi_proto::viz::CompressedImage> {
  TYPEDEF_COMMON_TYPES
};

template <>
struct Checker<adsfi_proto::viz::Odometry> {
  TYPEDEF_COMMON_TYPES
};

template <>
struct Checker<adsfi_proto::viz::Path> {
  TYPEDEF_COMMON_TYPES
};

template <>
struct Checker<adsfi_proto::viz::TransformStamped> {
  TYPEDEF_COMMON_TYPES
};

template <>
struct Checker<adsfi_proto::viz::Marker> {
  TYPEDEF_COMMON_TYPES
};

template <>
struct Checker<adsfi_proto::viz::MarkerArray> {
  TYPEDEF_COMMON_TYPES
};

template <>
struct Checker<adsfi_proto::viz::TwistStamped> {
  TYPEDEF_COMMON_TYPES
};

template <>
struct Checker<adsfi_proto::viz::PolygonStamped> {
  TYPEDEF_COMMON_TYPES
};

template <>
struct Checker<adsfi_proto::viz::PointCloud> {
  TYPEDEF_COMMON_TYPES
};

template <>
struct Checker<adsfi_proto::viz::PointCloud2> {
  TYPEDEF_COMMON_TYPES
};

template <>
struct Checker<adsfi_proto::viz::PoseArray> {
  TYPEDEF_COMMON_TYPES
};

template <>
struct Checker<adsfi_proto::viz::OccupancyGrid> {
  TYPEDEF_COMMON_TYPES
};

template <typename ProtoType>
typename Checker<ProtoType>::String GetTypeAlias() {
  std::string alias;
  if (std::is_same<ProtoType, adsfi_proto::viz::CompressedImage>::value) {
    alias = kCompressedImage;
  } else if (std::is_same<ProtoType, adsfi_proto::viz::Odometry>::value) {
    alias = kOdometry;
  } else if (std::is_same<ProtoType, adsfi_proto::viz::Path>::value) {
    alias = kPath;
  } else if (std::is_same<ProtoType,
                          adsfi_proto::viz::TransformStamped>::value) {
    alias = kTransformStamped;
  } else if (std::is_same<ProtoType, adsfi_proto::viz::Marker>::value) {
    alias = kMarker;
  } else if (std::is_same<ProtoType, adsfi_proto::viz::MarkerArray>::value) {
    alias = kMarkerArray;
  } else if (std::is_same<ProtoType, adsfi_proto::viz::TwistStamped>::value) {
    alias = kTwistStamped;
  } else if (std::is_same<ProtoType, adsfi_proto::viz::PolygonStamped>::value) {
    alias = kPolygonStamped;
  } else if (std::is_same<ProtoType, adsfi_proto::viz::PointCloud>::value) {
    alias = kPointCloud;
  } else if (std::is_same<ProtoType, adsfi_proto::viz::PointCloud2>::value) {
    alias = kPointCloud2;
  } else if (std::is_same<ProtoType, adsfi_proto::viz::PoseArray>::value) {
    alias = kPoseArray;
  } else if (std::is_same<ProtoType, adsfi_proto::viz::OccupancyGrid>::value) {
    alias = kOccupancyGrid;
  }

  return alias;
}

// template <typename ProtoType>
// typename Checker<ProtoType>::Bool
// IsValidType() {
//  std::string alias = GetTypeAlias<ProtoType>();
//  return !alias.empty();
//}

}  // namespace util
}  // namespace mp
}  // namespace hozon