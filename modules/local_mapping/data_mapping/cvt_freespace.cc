/********************************************************
 * Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 * Author: Hozon
 *******************************************************/
#include <bitset>

#include "modules/local_mapping/base/scene/occedge.h"
#include "modules/local_mapping/data_mapping/data_mapping.h"
#include "proto/local_mapping/local_map.pb.h"
#include "proto/perception/perception_freespace.pb.h"
#include "proto/perception/transport_element.pb.h"
namespace hozon {
namespace mp {
namespace lm {
namespace data_mapping {

static OccEdgeType CvtPb2OccEdgeType(
    hozon::perception::FreeSpaceOut::ClassType cls) {
  switch (cls) {
    // 锥桶
    case hozon::perception::FreeSpaceOut::ClassType::
        FreeSpaceOut_ClassType_CONE_POLE:
      return OccEdgeType::CONE_EDGE;
    // 路沿
    case hozon::perception::FreeSpaceOut::ClassType::
        FreeSpaceOut_ClassType_CURBSTONE:
      return OccEdgeType::ROAD_EDGE;
    default:
      return OccEdgeType::UNKNOWN;
  }
}
static OccGrid::OccupancyState CvtPbOccState(int state) {
  switch (state) {
    case 1:
      return OccGrid::OccupancyState::FREE;
    case 2:
      return OccGrid::OccupancyState::OCC;
    case 3:
      return OccGrid::OccupancyState::STATIC;
    case 4:
      return OccGrid::OccupancyState::DYNAMIC;
    default:
      return OccGrid::OccupancyState::UNKNOWN;
  }
}
bool DataMapping::CvtPbFreeSpace2FreeSpace(
    const hozon::perception::FreeSpaceOut& freespaceinfo,
    OccEdgePtr occedgeptr) {
  occedgeptr->detect_id = static_cast<int>(freespaceinfo.freespace_seq());
  occedgeptr->type = CvtPb2OccEdgeType(freespaceinfo.cls());
  for (auto& item : freespaceinfo.freespace_point()) {
    if (std::isnan(item.x()) || std::isnan(item.y()) || std::isnan(item.z())) {
      return false;
    }
    Eigen::Vector3d llpt;
    llpt.x() = item.x();
    llpt.y() = item.y();
    llpt.z() = item.z();
    occedgeptr->vehicle_points.push_back(llpt);
  }
  return true;
}
static hozon::mapping::FreeSpaceOutput_FreeSpaceCls CvtfreespaceCls2Pb(
    hozon::mp::lm::FreeSpaceCls cls) {
  switch (cls) {
    case hozon::mp::lm::FreeSpaceCls::UNKNOWN:
      return hozon::mapping::FreeSpaceOutput_FreeSpaceCls::
          FreeSpaceOutput_FreeSpaceCls_UNKNOWN;
    case hozon::mp::lm::FreeSpaceCls::GROUND:
      return hozon::mapping::FreeSpaceOutput_FreeSpaceCls::
          FreeSpaceOutput_FreeSpaceCls_GROUND;
    case hozon::mp::lm::FreeSpaceCls::FENCE:
      return hozon::mapping::FreeSpaceOutput_FreeSpaceCls::
          FreeSpaceOutput_FreeSpaceCls_FENCE;
    case hozon::mp::lm::FreeSpaceCls::ROADSIDE:
      return hozon::mapping::FreeSpaceOutput_FreeSpaceCls::
          FreeSpaceOutput_FreeSpaceCls_ROADSIDE;
    case hozon::mp::lm::FreeSpaceCls::NATURE:
      return hozon::mapping::FreeSpaceOutput_FreeSpaceCls::
          FreeSpaceOutput_FreeSpaceCls_NATURE;
    case hozon::mp::lm::FreeSpaceCls::ROADOBSTACLE:
      return hozon::mapping::FreeSpaceOutput_FreeSpaceCls::
          FreeSpaceOutput_FreeSpaceCls_ROADOBSTACLE;
    case hozon::mp::lm::FreeSpaceCls::SIDEWALK:
      return hozon::mapping::FreeSpaceOutput_FreeSpaceCls::
          FreeSpaceOutput_FreeSpaceCls_SIDEWALK;
    case hozon::mp::lm::FreeSpaceCls::VEHICLE:
      return hozon::mapping::FreeSpaceOutput_FreeSpaceCls::
          FreeSpaceOutput_FreeSpaceCls_VEHICLE;
    case hozon::mp::lm::FreeSpaceCls::PEDESTRIAN:
      return hozon::mapping::FreeSpaceOutput_FreeSpaceCls::
          FreeSpaceOutput_FreeSpaceCls_PEDESTRIAN;
    case hozon::mp::lm::FreeSpaceCls::CYCLIST:
      return hozon::mapping::FreeSpaceOutput_FreeSpaceCls::
          FreeSpaceOutput_FreeSpaceCls_CYCLIST;
    case hozon::mp::lm::FreeSpaceCls::TRAFFICBARRIER:
      return hozon::mapping::FreeSpaceOutput_FreeSpaceCls::
          FreeSpaceOutput_FreeSpaceCls_TRAFFICBARRIER;
    case hozon::mp::lm::FreeSpaceCls::FREE:
      return hozon::mapping::FreeSpaceOutput_FreeSpaceCls::
          FreeSpaceOutput_FreeSpaceCls_FREE;
    case hozon::mp::lm::FreeSpaceCls::UNFREE:
      return hozon::mapping::FreeSpaceOutput_FreeSpaceCls::
          FreeSpaceOutput_FreeSpaceCls_UNFREE;
    default:
      return hozon::mapping::FreeSpaceOutput_FreeSpaceCls::
          FreeSpaceOutput_FreeSpaceCls_UNKNOWN;
  }
}
bool DataMapping::CvtFreeSpace2Pb(
    const FreeSpaceOutputPtr& freespaceoutptr,
    hozon::mapping::FreeSpaceOutput* pb_freespace) {
  if (nullptr == freespaceoutptr || nullptr == pb_freespace) {
    HLOG_ERROR << " msg  or pb_object is nullptr.";
    return false;
  }

  // if (pb_freespace->has_cls()) {

  hozon::mapping::FreeSpaceOutput_FreeSpaceCls cls =
      CvtfreespaceCls2Pb(freespaceoutptr->cls);
  pb_freespace->set_cls(cls);
  // }
  // if (pb_freespace->has_area()) {
  pb_freespace->set_area(freespaceoutptr->area);

  // }
  for (auto& edges_points : freespaceoutptr->vec_edges_points) {
    auto instance = pb_freespace->add_instances();
    for (auto& point : edges_points) {
      auto boundary_point = instance->add_boundary_points();
      boundary_point->set_x(point.x());
      boundary_point->set_y(point.y());
      boundary_point->set_z(point.z());
    }
  }
  return true;
}
}  // namespace data_mapping
}  // namespace lm
}  // namespace mp
}  //  namespace hozon
