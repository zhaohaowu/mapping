/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： msg_alias.h
 *   author     ： taoshaoyuan
 *   date       ： 2022.10
 ******************************************************************************/

#pragma once

// auto generated
#include <adsfi_proto/viz/geometry_msgs.pb.h>
#include <adsfi_proto/viz/msg_info.pb.h>
#include <adsfi_proto/viz/nav_msgs.pb.h>
#include <adsfi_proto/viz/sensor_msgs.pb.h>
#include <adsfi_proto/viz/tf2_msgs.pb.h>
#include <adsfi_proto/viz/visualization_msgs.pb.h>
// #include <adsfi_proto/viz/proto_msg.pb.h>
#include <proto/dead_reckoning/dr.pb.h>
#include <proto/localization/localization.pb.h>
#include <proto/localization/node_info.pb.h>
#include <proto/soc/chassis.pb.h>
#include <proto/soc/sensor_imu_ins.pb.h>

#include <memory>
#include <string>

/*
 * 能不能设计成这样：所有支持的类型都是在这个头文件里定义，从类型alias直接得到具体的类型；
 * RvizAgent和RvizAgentClient里不体现具体的类型
 */

namespace hozon {
namespace mp {
namespace util {

// type alias
static const std::string kCompressedImage = "CompressedImage";    // NOLINT
static const std::string kOdometry = "Odometry";                  // NOLINT
static const std::string kPath = "Path";                          // NOLINT
static const std::string kTransformStamped = "TransformStamped";  // NOLINT
static const std::string kMarker = "Marker";                      // NOLINT
static const std::string kMarkerArray = "MarkerArray";            // NOLINT
static const std::string kTwistStamped = "TwistStamped";          // NOLINT
static const std::string kPolygonStamped = "PolygonStamped";      // NOLINT
static const std::string kPointCloud = "PointCloud";              // NOLINT
static const std::string kPointCloud2 = "PointCloud2";            // NOLINT
static const std::string kPoseArray = "PoseArray";                // NOLINT
static const std::string kOccupancyGrid = "OccupancyGrid";        // NOLINT
// custom proto visualization types
static const std::string kLocalization = "Localization";    // NOLINT
static const std::string kDeadReckoning = "DeadReckoning";  // NOLINT
static const std::string kImuIns = "ImuIns";                // NOLINT
static const std::string kHafNodeInfo = "HafNodeInfo";      // NOLINT
static const std::string kChassis = "Chassis";              // NOLINT

static const std::string kCtrlTopic = "/RvizAgent/Ctrl";  // NOLINT

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
using LocalizationCallback = std::function<void(
    const std::string&, std::shared_ptr<hozon::localization::Localization>)>;
using DeadReckoningCallback = std::function<void(
    const std::string&, std::shared_ptr<hozon::dead_reckoning::DeadReckoning>)>;
using ImuInsCallback = std::function<void(const std::string&,
                                          std::shared_ptr<hozon::soc::ImuIns>)>;
using HafNodeInfoCallback = std::function<void(
    const std::string&, std::shared_ptr<hozon::localization::HafNodeInfo>)>;
using ChassisCallback = std::function<void(
    const std::string&, std::shared_ptr<hozon::soc::Chassis>)>;

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

template <>
struct Checker<hozon::localization::Localization> {
  TYPEDEF_COMMON_TYPES
};

template <>
struct Checker<hozon::dead_reckoning::DeadReckoning> {
  TYPEDEF_COMMON_TYPES
};

template <>
struct Checker<hozon::soc::ImuIns> {
  TYPEDEF_COMMON_TYPES
};

template <>
struct Checker<hozon::localization::HafNodeInfo> {
  TYPEDEF_COMMON_TYPES
};

template <>
struct Checker<hozon::soc::Chassis> {
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
  } else if (std::is_same<ProtoType,
                          hozon::localization::Localization>::value) {
    alias = kLocalization;
  } else if (std::is_same<ProtoType,
                          hozon::dead_reckoning::DeadReckoning>::value) {
    alias = kDeadReckoning;
  } else if (std::is_same<ProtoType, hozon::soc::ImuIns>::value) {
    alias = kImuIns;
  } else if (std::is_same<ProtoType, hozon::localization::HafNodeInfo>::value) {
    alias = kHafNodeInfo;
  } else if (std::is_same<ProtoType, hozon::soc::Chassis>::value) {
    alias = kChassis;
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
