/********************************************************
 * Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 * Author: Hozon
 *******************************************************/
#include <cmath>

#include "base/scene/roadedge.h"
#include "modules/local_mapping/data_mapping/data_mapping.h"
#include "proto/local_mapping/local_map.pb.h"
#include "proto/perception/transport_element.pb.h"

namespace hozon {
namespace mp {

namespace lm {
namespace data_mapping {

static RoadEdgeType CvtPb2RoadEdgeType(
    hozon::perception::RoadEdge::RoadEdgeType shape) {
  switch (shape) {
    case hozon::perception::RoadEdge::RoadEdgeType::
        RoadEdge_RoadEdgeType_ROAD_EDGE:
      return RoadEdgeType::ROAD_EDGE;
    case hozon::perception::RoadEdge::RoadEdgeType::
        RoadEdge_RoadEdgeType_GROUND_EDGE:
      return RoadEdgeType::GROUND_EDGE;
    case hozon::perception::RoadEdge::RoadEdgeType::
        RoadEdge_RoadEdgeType_CONE_EDGE:
      return RoadEdgeType::CONE_EDGE;
    case hozon::perception::RoadEdge::RoadEdgeType::
        RoadEdge_RoadEdgeType_WATERHORSE_EDGE:
      return RoadEdgeType::WATERHORSE_EDGE;
    case hozon::perception::RoadEdge::RoadEdgeType::
        RoadEdge_RoadEdgeType_FENCE_EDGE:
      return RoadEdgeType::FENCE_EDGE;
    default:
      return RoadEdgeType::UNKNOWN;
  }
}

static hozon::mapping::LanePositionType CvtRoadEdgePosType2Pb(
    LaneLinePosition type) {
  switch (type) {
    case LaneLinePosition::OTHER:
      return hozon::mapping::LanePositionType::LanePositionType_OTHER;
    case LaneLinePosition::FOURTH_LEFT:
      return hozon::mapping::LanePositionType::LanePositionType_FOURTH_LEFT;
    case LaneLinePosition::THIRD_LEFT:
      return hozon::mapping::LanePositionType::LanePositionType_THIRD_LEFT;
    case LaneLinePosition::ADJACENT_LEFT:
      return hozon::mapping::LanePositionType::LanePositionType_ADJACENT_LEFT;
    case LaneLinePosition::EGO_LEFT:
      return hozon::mapping::LanePositionType::LanePositionType_EGO_LEFT;
    case LaneLinePosition::EGO_RIGHT:
      return hozon::mapping::LanePositionType::LanePositionType_EGO_RIGHT;
    case LaneLinePosition::ADJACENT_RIGHT:
      return hozon::mapping::LanePositionType::LanePositionType_ADJACENT_RIGHT;
    case LaneLinePosition::THIRD_RIGHT:
      return hozon::mapping::LanePositionType::LanePositionType_THIRD_RIGHT;
    case LaneLinePosition::FOURTH_RIGHT:
      return hozon::mapping::LanePositionType::LanePositionType_FOURTH_RIGHT;
    default:
      return hozon::mapping::LanePositionType::LanePositionType_OTHER;
  }
}
static hozon::perception::LanePositionType CvtRoadEdgePosType2TePb(
    LaneLinePosition type) {
  switch (type) {
    case LaneLinePosition::OTHER:
      return hozon::perception::LanePositionType::OTHER;
    case LaneLinePosition::FOURTH_LEFT:
      return hozon::perception::LanePositionType::FOURTH_LEFT;
    case LaneLinePosition::THIRD_LEFT:
      return hozon::perception::LanePositionType::THIRD_LEFT;
    case LaneLinePosition::ADJACENT_LEFT:
      return hozon::perception::LanePositionType::ADJACENT_LEFT;
    case LaneLinePosition::EGO_LEFT:
      return hozon::perception::LanePositionType::EGO_LEFT;
    case LaneLinePosition::EGO_RIGHT:
      return hozon::perception::LanePositionType::EGO_RIGHT;
    case LaneLinePosition::ADJACENT_RIGHT:
      return hozon::perception::LanePositionType::ADJACENT_RIGHT;
    case LaneLinePosition::THIRD_RIGHT:
      return hozon::perception::LanePositionType::THIRD_RIGHT;
    case LaneLinePosition::FOURTH_RIGHT:
      return hozon::perception::LanePositionType::FOURTH_RIGHT;
    default:
      return hozon::perception::LanePositionType::OTHER;
  }
}
bool DataMapping::CvtRoadEdge2TePb(const RoadEdgePtr& roadedge_msg,
                                   hozon::perception::RoadEdge* pb_roadedge) {
  if (nullptr == roadedge_msg || nullptr == pb_roadedge) {
    HLOG_ERROR << "arrow msg  or pb_arrow is nullptr.";
    return false;
  }
  pb_roadedge->set_id(roadedge_msg->id);
  pb_roadedge->set_confidence(1.0);
  hozon::perception::LanePositionType send_pos_type =
      CvtRoadEdgePosType2TePb(roadedge_msg->te_position);
  for (auto& item_pt : roadedge_msg->vehicle_points) {
    if (item_pt.x() < 0) {
      continue;
    }
    auto pb_pt = pb_roadedge->add_points();
    pb_pt->set_x(item_pt.x());
    pb_pt->set_y(item_pt.y());
    pb_pt->set_z(item_pt.z());
  }

  auto* curve_lane = pb_roadedge->mutable_vehicle_curve();
  if (roadedge_msg->vehicle_curve.coeffs.size() == 4) {
    // 车道线三次方程系数跟踪方案发送字段
    curve_lane->set_c0(roadedge_msg->vehicle_curve.coeffs[0]);
    curve_lane->set_c1(roadedge_msg->vehicle_curve.coeffs[1]);
    curve_lane->set_c2(roadedge_msg->vehicle_curve.coeffs[2]);
    curve_lane->set_c3(roadedge_msg->vehicle_curve.coeffs[3]);
    curve_lane->set_start_point_x(roadedge_msg->vehicle_curve.min);
    curve_lane->set_end_point_x(roadedge_msg->vehicle_curve.max);
  }

  return true;
}

bool DataMapping::CvtRoadEdge2Pb(const RoadEdgePtr& roadedge_msg,
                                 hozon::mapping::RoadEdge* pb_roadedge) {
  if (nullptr == roadedge_msg || nullptr == pb_roadedge) {
    HLOG_ERROR << "arrow msg  or pb_arrow is nullptr.";
    return false;
  }
  pb_roadedge->set_track_id(roadedge_msg->id);
  pb_roadedge->set_confidence(roadedge_msg->type_confidence);

  for (auto& item_pt : roadedge_msg->vehicle_points) {
    auto pb_pt = pb_roadedge->add_points();
    pb_pt->set_x(item_pt.x());
    pb_pt->set_y(item_pt.y());
    pb_pt->set_z(item_pt.z());
  }
  hozon::mapping::LanePositionType send_pos_type =
      CvtRoadEdgePosType2Pb(roadedge_msg->position);
  // HLOG_INFO << "=====******====roadedge send_pos_type:"
  //           << static_cast<int>(send_pos_type);
  auto* curve_lane = pb_roadedge->mutable_lane_param()->add_cubic_curve_set();
  pb_roadedge->set_lanepos(send_pos_type);
  pb_roadedge->set_confidence(1.0);
  if (roadedge_msg->vehicle_curve.coeffs.size() == 4) {
    // 车道线三次方程系数跟踪方案发送字段
    curve_lane->set_c0(roadedge_msg->vehicle_curve.coeffs[0]);
    curve_lane->set_c1(roadedge_msg->vehicle_curve.coeffs[1]);
    curve_lane->set_c2(roadedge_msg->vehicle_curve.coeffs[2]);
    curve_lane->set_c3(roadedge_msg->vehicle_curve.coeffs[3]);
    curve_lane->set_start_point_x(roadedge_msg->vehicle_curve.min);
    curve_lane->set_end_point_x(roadedge_msg->vehicle_curve.max);
  }

  return true;
}

bool DataMapping::CvtPb2RoadEdgeMeasurement(
    const hozon::perception::RoadEdge& roadedge, RoadEdgePtr roadedge_ptr) {
  roadedge_ptr->id = roadedge.id();
  roadedge_ptr->type = CvtPb2RoadEdgeType(roadedge.type());
  roadedge_ptr->type_confidence = roadedge.confidence();

  for (auto& item : roadedge.points()) {
    if (std::isnan(item.x()) || std::isnan(item.y()) || std::isnan(item.z())) {
      return false;
    }
    Eigen::Vector3d llpt;
    llpt.x() = item.x();
    llpt.y() = item.y();
    llpt.z() = item.z();
    roadedge_ptr->vehicle_points.push_back(llpt);
  }

  return true;
}

}  // namespace data_mapping
}  // namespace lm
}  // namespace mp
}  //  namespace hozon
