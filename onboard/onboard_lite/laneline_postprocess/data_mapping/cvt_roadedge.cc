/********************************************************
 * Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 * Author: Hozon
 *******************************************************/
#include "base/measurement/roadedges_measurement.h"
#include "base/scene/roadedges.h"
#include "onboard/onboard_lite/laneline_postprocess/data_mapping/data_mapping.h"
#include "proto/perception/transport_element.pb.h"

namespace hozon {
namespace mp {
namespace common_onboard {

static bool CvtRoadEdgeToPb(const base::RoadEdgePtr &arrow_msg,
                            RoadEdge *pb_object);

static RoadEdge::RoadEdgeType CvtRoadEdgeType2Pb(base::RoadEdgeType shape) {
  switch (shape) {
    case base::RoadEdgeType::ROAD_EDGE:
      return RoadEdge::RoadEdgeType::RoadEdge_RoadEdgeType_ROAD_EDGE;
    case base::RoadEdgeType::GROUND_EDGE:
      return RoadEdge::RoadEdgeType::RoadEdge_RoadEdgeType_GROUND_EDGE;
    case base::RoadEdgeType::CONE_EDGE:
      return RoadEdge::RoadEdgeType::RoadEdge_RoadEdgeType_CONE_EDGE;
    case base::RoadEdgeType::WATERHORSE_EDGE:
      return RoadEdge::RoadEdgeType::RoadEdge_RoadEdgeType_WATERHORSE_EDGE;
    case base::RoadEdgeType::FENCE_EDGE:
      return RoadEdge::RoadEdgeType::RoadEdge_RoadEdgeType_FENCE_EDGE;
    default:
      return RoadEdge::RoadEdgeType::RoadEdge_RoadEdgeType_UNKNOWN;
  }
}
static base::RoadEdgeType CvtPb2RoadEdgeType(RoadEdge::RoadEdgeType shape) {
  switch (shape) {
    case RoadEdge::RoadEdgeType::RoadEdge_RoadEdgeType_ROAD_EDGE:
      return base::RoadEdgeType::ROAD_EDGE;
    case RoadEdge::RoadEdgeType::RoadEdge_RoadEdgeType_GROUND_EDGE:
      return base::RoadEdgeType::GROUND_EDGE;
    case RoadEdge::RoadEdgeType::RoadEdge_RoadEdgeType_CONE_EDGE:
      return base::RoadEdgeType::CONE_EDGE;
    case RoadEdge::RoadEdgeType::RoadEdge_RoadEdgeType_WATERHORSE_EDGE:
      return base::RoadEdgeType::WATERHORSE_EDGE;
    case RoadEdge::RoadEdgeType::RoadEdge_RoadEdgeType_FENCE_EDGE:
      return base::RoadEdgeType::FENCE_EDGE;
    default:
      return base::RoadEdgeType::UNKNOWN;
  }
}

bool DataMapping::CvtMultiRoadEdgesToPb(
    const std::vector<base::RoadEdgePtr> &roadedge_msgs,
    NetaTransportElementPtr pb_objects) {
  if (roadedge_msgs.size() == 0) {
    HLOG_DEBUG << "roadedge msg size is 0";
    return true;
  }
  if (nullptr == pb_objects) {
    HLOG_ERROR << "pb_objects is nullptr.";
    return false;
  }

  uint32_t roadedge_size = roadedge_msgs.size();
  for (size_t i = 0; i < roadedge_size; ++i) {
    auto pb_roadedge = pb_objects->add_road_edges();
    if (!CvtRoadEdgeToPb(roadedge_msgs[i], pb_roadedge)) {
      HLOG_ERROR << "cvt arrow to proto struct failed.";
      return false;
    }
  }
  return true;
}

bool DataMapping::CvtRoadEdgeToPb(const base::RoadEdgePtr &roadedge_msg,
                                  RoadEdge *pb_roadedge) {
  if (nullptr == roadedge_msg || nullptr == pb_roadedge) {
    HLOG_ERROR << "arrow msg  or pb_arrow is nullptr.";
    return false;
  }
  pb_roadedge->set_id(roadedge_msg->id);
  RoadEdge::RoadEdgeType send_type = CvtRoadEdgeType2Pb(roadedge_msg->type);
  pb_roadedge->set_type(send_type);
  pb_roadedge->set_confidence(roadedge_msg->confidence);

  if (roadedge_msg->point_set.size() != 0) {
    for (auto &item_pt : roadedge_msg->point_set) {
      auto pb_pt = pb_roadedge->add_points();
      pb_pt->set_x(item_pt.vehicle_point.x);
      pb_pt->set_y(item_pt.vehicle_point.y);
      pb_pt->set_z(item_pt.vehicle_point.z);
    }
  }

  auto curve_info = pb_roadedge->mutable_vehicle_curve();
  if (roadedge_msg->vehicle_curve.coeffs.size() == 4) {
    // 车道线三次方程系数跟踪方案发送字段
    curve_info->set_c0(roadedge_msg->vehicle_curve.coeffs[0]);
    curve_info->set_c1(roadedge_msg->vehicle_curve.coeffs[1]);
    curve_info->set_c2(roadedge_msg->vehicle_curve.coeffs[2]);
    curve_info->set_c3(roadedge_msg->vehicle_curve.coeffs[3]);
    curve_info->set_start_point_x(roadedge_msg->vehicle_curve.min);
    curve_info->set_end_point_x(roadedge_msg->vehicle_curve.max);
  }

  return true;
}

bool DataMapping::CvtRoadEdgeMeasurementToPb(
    const std::vector<base::RoadEdgeMeasurementPtr> &roadedge_measures,
    hozon::perception::TransportElement *transport_element) {
  for (auto &item : roadedge_measures) {
    auto pb_roadedge = transport_element->add_road_edges();
    pb_roadedge->set_id(item->id);
    pb_roadedge->set_type(CvtRoadEdgeType2Pb(item->type));
    pb_roadedge->set_confidence(item->confidence);
    if (item->point_set.size() != 0) {
      for (auto &item_pt : item->point_set) {
        auto arrow_points = pb_roadedge->mutable_points();
        auto pb_pt = pb_roadedge->add_points();
        pb_pt->set_x(item_pt.vehicle_point.x);
        pb_pt->set_y(item_pt.vehicle_point.y);
        pb_pt->set_z(item_pt.vehicle_point.z);
      }
    }
  }
  return true;
}

bool DataMapping::CvtPb2RoadEdgeMeasurement(
    const hozon::perception::RoadEdge &roadedge,
    base::RoadEdgeMeasurementPtr roadedgeptr) {
  roadedgeptr->id = roadedge.id();
  roadedgeptr->type = CvtPb2RoadEdgeType(roadedge.type());
  roadedgeptr->confidence = roadedge.confidence();

  for (auto &item : roadedge.points()) {
    base::LaneLinePoint llpt;
    llpt.vehicle_point.x = item.x();
    llpt.vehicle_point.y = item.y();
    llpt.vehicle_point.z = item.z();
    roadedgeptr->point_set.push_back(std::move(llpt));
  }

  return true;
}

}  //  namespace common_onboard
}  // namespace mp
}  //  namespace hozon
