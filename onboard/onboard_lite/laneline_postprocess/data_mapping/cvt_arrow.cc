/********************************************************
 * Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 * Author: Hozon
 *******************************************************/

#include "onboard/onboard_lite/laneline_postprocess/data_mapping/data_mapping.h"
#include "perception-base/base/scene/arrow.h"

namespace hozon {
namespace mp {
namespace common_onboard {

static bool CvtArrowToPb(const perception_base::ArrowPtr& arrow_msg,
                         hozon::perception::Arrow* pb_object);

static hozon::perception::ArrowType CvtArrowType2Pb(
    perception_base::ArrowType shape) {
  switch (shape) {
    case perception_base::ArrowType::STRAIGHT_FORWARD:
      return hozon::perception::ArrowType::STRAIGHT_FORWARD;
    case perception_base::ArrowType::STRAIGHT_FORWARD_OR_TURN_LEFT:
      return hozon::perception::ArrowType::STRAIGHT_FORWARD_OR_TURN_LEFT;
    case perception_base::ArrowType::STRAIGHT_FORWARD_OR_TURN_RIGHT:
      return hozon::perception::ArrowType::STRAIGHT_FORWARD_OR_TURN_RIGHT;
    case perception_base::ArrowType::
        STRAIGHT_FORWARD_OR_TURN_LEFT_OR_TURN_RIGHT:
      return hozon::perception::ArrowType::
          STRAIGHT_FORWARD_OR_TURN_LEFT_OR_TURN_RIGHT;
    case perception_base::ArrowType::STRAIGHT_FORWARD_OR_TURN_AROUND:
      return hozon::perception::ArrowType::STRAIGHT_FORWARD_OR_TURN_AROUND;
    case perception_base::ArrowType::
        STRAIGHT_FORWARD_OR_TURN_AROUND_OR_TURN_LEFT:
      return hozon::perception::ArrowType::
          STRAIGHT_FORWARD_OR_TURN_AROUND_OR_TURN_LEFT;
    case perception_base::ArrowType::TURN_LEFT:
      return hozon::perception::ArrowType::TURN_LEFT;
    case perception_base::ArrowType::TURN_LEFT_OR_MERGE_LEFT:
      return hozon::perception::ArrowType::TURN_LEFT_OR_MERGE_LEFT;
    case perception_base::ArrowType::TURN_LEFT_OR_TURN_AROUND:
      return hozon::perception::ArrowType::TURN_LEFT_OR_TURN_AROUND;
    case perception_base::ArrowType::TURN_LEFT_OR_TURN_RIGHT:
      return hozon::perception::ArrowType::TURN_LEFT_OR_TURN_RIGHT;
    case perception_base::ArrowType::TURN_RIGHT:
      return hozon::perception::ArrowType::TURN_RIGHT;
    case perception_base::ArrowType::TURN_RIGHT_OR_MERGE_RIGHT:
      return hozon::perception::ArrowType::TURN_RIGHT_OR_MERGE_RIGHT;
    case perception_base::ArrowType::TURN_RIGHT_OR_TURN_AROUND:
      return hozon::perception::ArrowType::TURN_RIGHT_OR_TURN_AROUND;
    case perception_base::ArrowType::TURN_AROUND:
      return hozon::perception::ArrowType::TURN_AROUND;
    case perception_base::ArrowType::FORBID_TURN_LEFT:
      return hozon::perception::ArrowType::FORBID_TURN_LEFT;
    case perception_base::ArrowType::FORBID_TURN_RIGHT:
      return hozon::perception::ArrowType::FORBID_TURN_RIGHT;
    case perception_base::ArrowType::FORBID_TURN_AROUND:
      return hozon::perception::ArrowType::FORBID_TURN_AROUND;
    case perception_base::ArrowType::FRONT_NEAR_CROSSWALK:
      return hozon::perception::ArrowType::FRONT_NEAR_CROSSWALK;
    default:
      return hozon::perception::ArrowType::ARROWTYPE_UNKNOWN;
  }
}

bool DataMapping::CvtMultiArrowsToPb(
    const std::vector<perception_base::ArrowPtr>& arrow_msgs,
    NetaTransportElementPtr pb_objects) {
  if (arrow_msgs.size() == 0) {
    HLOG_DEBUG << "arrows msg size is 0";
    return true;
  }
  if (nullptr == pb_objects) {
    HLOG_ERROR << "pb_objects is nullptr.";
    return false;
  }

  uint32_t arrow_size = arrow_msgs.size();
  for (size_t i = 0; i < arrow_size; ++i) {
    auto pb_arrow = pb_objects->add_arrow();
    if (!CvtArrowToPb(arrow_msgs[i], pb_arrow)) {
      HLOG_ERROR << "cvt arrow to proto struct failed.";
      return false;
    }
  }
  return true;
}

bool DataMapping::CvtArrowToPb(const perception_base::ArrowPtr& arrow_msg,
                               hozon::perception::Arrow* pb_arrow) {
  if (nullptr == arrow_msg || nullptr == pb_arrow) {
    HLOG_ERROR << "arrow msg  or pb_arrow is nullptr.";
    return false;
  }
  pb_arrow->set_track_id(arrow_msg->id);
  hozon::perception::ArrowType send_type = CvtArrowType2Pb(arrow_msg->type);
  pb_arrow->set_type(send_type);
  pb_arrow->set_confidence(arrow_msg->confidence);

  if (arrow_msg->point_set_3d.size() != 0) {
    for (auto& item_pt : arrow_msg->point_set_3d) {
      auto arrow_points = pb_arrow->mutable_points();
      auto pb_pt = arrow_points->add_point();
      pb_pt->set_x(item_pt.x);
      pb_pt->set_y(item_pt.y);
      pb_pt->set_z(item_pt.z);
    }
  } else if (arrow_msg->point_set_2d.size() != 0) {
    for (auto& item_pt : arrow_msg->point_set_2d) {
      auto arrow_points = pb_arrow->mutable_points();
      auto pb_pt = arrow_points->add_point();
      pb_pt->set_x(item_pt.x);
      pb_pt->set_y(item_pt.y);
      pb_pt->set_z(0.0);
      // HLOG_DEBUG << "============x: " << item_pt.x << " y:" << item_pt.y;
    }
  }

  return true;
}

bool DataMapping::CvtArrowMeasurementToPb(
    const std::vector<perception_base::ArrowMeasurementPtr>& arrow_measure,
    hozon::perception::TransportElement* transport_element) {
  for (auto& item : arrow_measure) {
    auto pb_arrow = transport_element->add_arrow();
    pb_arrow->set_track_id(item->id);
    pb_arrow->set_type(CvtArrowType2Pb(item->type));
    pb_arrow->set_confidence(item->confidence);
    if (item->point_set_3d.size() != 0) {
      for (auto& item_pt : item->point_set_3d) {
        auto arrow_points = pb_arrow->mutable_points();
        auto pb_pt = arrow_points->add_point();
        pb_pt->set_x(item_pt.x);
        pb_pt->set_y(item_pt.y);
        pb_pt->set_z(item_pt.z);
      }
    } else if (item->point_set_2d.size() != 0) {
      for (auto& item_pt : item->point_set_2d) {
        auto arrow_points = pb_arrow->mutable_points();
        auto pb_pt = arrow_points->add_point();
        pb_pt->set_x(item_pt.x);
        pb_pt->set_y(item_pt.y);
        pb_pt->set_z(0.0);
      }
    }
  }
  return true;
}

bool DataMapping::CvtPb2ArrowMeasurement(
    const hozon::perception::Arrow& arrow,
    perception_base::ArrowMeasurementPtr arrowptr) {
  arrowptr->id = arrow.track_id();
  arrowptr->confidence = arrow.confidence();

  for (auto& item : arrow.points().point()) {
    perception_base::Point3DF llpt;
    llpt.x = item.x();
    llpt.y = item.y();
    llpt.z = item.z();

    arrowptr->point_set_3d.push_back(std::move(llpt));
  }

  return true;
}

}  //  namespace common_onboard
}  // namespace mp
}  //  namespace hozon
