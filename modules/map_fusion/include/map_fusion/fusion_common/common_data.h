/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： common_data.h
 *   author     ： taoshaoyuan
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <depend/proto/dead_reckoning/dr.pb.h>
#include <depend/proto/localization/node_info.pb.h>
#include <depend/proto/map/map.pb.h>
#include <depend/proto/perception/transport_element.pb.h>
#include <depend/proto/soc/sensor_image.pb.h>
#include <depend/proto/soc/sensor_imu_ins.pb.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <vector>

#include "map_fusion/fusion_common/element_map.h"

namespace hozon {
namespace mp {
namespace mf {

using DrPtr = std::shared_ptr<hozon::dead_reckoning::DeadReckoning>;
using HafNodeInfoPtr = std::shared_ptr<hozon::localization::HafNodeInfo>;
using TransportElementPtr =
    std::shared_ptr<hozon::perception::TransportElement>;

enum class PoseState {
  NORMAL = 0,
  UTURN = 1,
  REVERS = 2,
  STAY = 3,
};

struct Pose {
  Pose() : stamp(0) {
    pos.setZero();
    quat.setIdentity();
  }

  virtual void Reset(double reset_stamp) {
    stamp = reset_stamp;
    pos.setZero();
    quat.setIdentity();
  }

  Eigen::Vector3f TransformPoint(const Eigen::Vector3f& pt) const {
    Eigen::Vector3f new_pt = quat * pt + pos;
    return new_pt;
  }

  Pose Inverse() const {
    Pose inv;
    inv.stamp = stamp;
    inv.quat = quat.inverse();
    inv.pos = inv.quat * pos * (-1);
    return inv;
  }

  double stamp = 0;
  Eigen::Vector3f pos;
  Eigen::Quaternionf quat;
  PoseState state = PoseState::NORMAL;

  DEFINE_PTR(Pose)
};

struct KinePose : public Pose {
  Eigen::Vector3f vel;
  Eigen::Vector3f acc;
  Eigen::Vector3f ang_vel;

  void Reset(double reset_stamp) override {
    Pose::Reset(reset_stamp);
    vel.setZero();
    acc.setZero();
    ang_vel.setZero();
  }

  DEFINE_PTR(KinePose)
};

struct PoseLla {
  double stamp = 0.;
  double lon = 0.;
  double lat = 0.;
  double alt = 0.;

  Eigen::Quaternionf quat;

  DEFINE_PTR(PoseLla)
};

struct PercepBoundary {
  em::LineCubic cubic;
  std::vector<em::Point> points;
  em::BoundaryType type;

  DEFINE_PTR(PercepBoundary)
};

struct PercepElements {
  //  em::Id id;
  double stamp;
  std::vector<PercepBoundary::Ptr> boundaries;

  DEFINE_PTR(PercepElements)
};

#define ID_PREFIX_ARROW "arrow/"
#define ID_PREFIX_NODE "node/"
#define ID_PREFIX_BOUNDARY "boundary/"
#define ID_PREFIX_CENTER_LINE "center_line/"
#define ID_PREFIX_CROSS_WALK "cross_walk/"
#define ID_PREFIX_LANE "lane/"
#define ID_PREFIX_ROAD "road/"
#define ID_PREFIX_STOP_LINE "stop_line/"
#define ID_PREFIX_SYMBOL "symbol/"
#define ID_PREFIX_TRAFFIC_LIGHT "traffic_light/"

// struct AssociatedLaneLine {
//   em::Id map_boundary_id;
//   PercepLaneLine percep_line;
//
// };

struct PercepBoundWithCloud {
  PercepBoundary::Ptr bound = nullptr;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = nullptr;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree = nullptr;
};

struct MapBoundWithCloud {
  em::Boundary::Ptr bound = nullptr;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = nullptr;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree = nullptr;
};

struct AssociatedBoundary {
  // PercepBoundary::Ptr percep_boundary = nullptr;
  PercepBoundWithCloud percep_boundary;
  MapBoundWithCloud map_boundary;
};

struct AssociatedElements {
  std::vector<AssociatedBoundary> boundaries;
};

template <typename ElementType>
struct PossibleAssociated {
  typename ElementType::Ptr map_element = nullptr;
  float score = 0;
};

#define PRINT_TBD                                                 \
  std::cout << "!!! TBD: " << __FILE__ << ": " << __LINE__ << " " \
            << __FUNCTION__ << std::endl;

}  // namespace mf
}  // namespace mp
}  // namespace hozon
