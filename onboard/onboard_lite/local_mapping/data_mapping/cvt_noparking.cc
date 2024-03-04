/********************************************************
 * Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 * Author: Hozon
 *******************************************************/

#include "onboard/onboard_lite/local_mapping/data_mapping/data_mapping.h"
#include "perception-base/base/scene/noparking.h"
#include "proto/perception/transport_element.pb.h"

namespace hozon {
namespace mp {
namespace common_onboard {

static bool CvtNoParkingToPb(const perception_base::NoParkingPtr& noparking_msg,
                             hozon::perception::NoParkingZone* pb_object);

bool DataMapping::CvtMultiNoParkingsToPb(
    const std::vector<perception_base::NoParkingPtr>& noparking_msgs,
    NetaTransportElementPtr pb_objects) {
  if (noparking_msgs.size() == 0) {
    HLOG_DEBUG << "noparking_msg msg size is 0";
    return true;
  }
  if (nullptr == pb_objects) {
    HLOG_ERROR << "pb_objects is nullptr.";
    return false;
  }

  uint32_t noparking_size = noparking_msgs.size();
  for (size_t i = 0; i < noparking_size; ++i) {
    auto pb_noparking = pb_objects->add_no_parking_zone();
    if (!CvtNoParkingToPb(noparking_msgs[i], pb_noparking)) {
      HLOG_ERROR << "cvt zebracrossing to proto struct failed.";
      return false;
    }
  }
  return true;
}

bool DataMapping::CvtNoParkingToPb(
    const perception_base::NoParkingPtr& noparking_msg,
    hozon::perception::NoParkingZone* pb_noparking) {
  if (nullptr == noparking_msg || nullptr == pb_noparking) {
    HLOG_ERROR << "noparking msg  or pb_noparking is nullptr.";
    return false;
  }
  pb_noparking->set_track_id(noparking_msg->id);
  pb_noparking->set_confidence(noparking_msg->confidence);

  if (noparking_msg->point_set_3d.size() != 0) {
    for (auto& item_pt : noparking_msg->point_set_3d) {
      auto arrow_points = pb_noparking->mutable_points();
      auto pb_pt = arrow_points->add_point();
      pb_pt->set_x(item_pt.x);
      pb_pt->set_y(item_pt.y);
      pb_pt->set_z(item_pt.z);
    }
  } else if (noparking_msg->point_set_2d.size() != 0) {
    for (auto& item_pt : noparking_msg->point_set_2d) {
      auto arrow_points = pb_noparking->mutable_points();
      auto pb_pt = arrow_points->add_point();
      pb_pt->set_x(item_pt.x);
      pb_pt->set_y(item_pt.y);
      pb_pt->set_z(0.0);
      // HLOG_DEBUG << "============x: " << item_pt.x << " y:" << item_pt.y;
    }
  }

  return true;
}

bool DataMapping::CvtNoParkingMeasurementToPb(
    const std::vector<perception_base::NoparkingMeasurementPtr>&
        noparking_measure,
    hozon::perception::TransportElement* transport_element) {
  for (auto& item : noparking_measure) {
    auto pb_noparking = transport_element->add_turn_waiting_zone();
    pb_noparking->set_track_id(item->id);
    // pb_noparking->set_type(CvtArrowType2Pb(item->type));
    pb_noparking->set_confidence(item->confidence);
    if (item->point_set_3d.size() != 0) {
      for (auto& item_pt : item->point_set_3d) {
        auto arrow_points = pb_noparking->mutable_points();
        auto pb_pt = arrow_points->add_point();
        pb_pt->set_x(item_pt.x);
        pb_pt->set_y(item_pt.y);
        pb_pt->set_z(item_pt.z);
      }
    } else if (item->point_set_2d.size() != 0) {
      for (auto& item_pt : item->point_set_2d) {
        auto arrow_points = pb_noparking->mutable_points();
        auto pb_pt = arrow_points->add_point();
        pb_pt->set_x(item_pt.x);
        pb_pt->set_y(item_pt.y);
        pb_pt->set_z(0.0);
      }
    }
  }
  return true;
}

}  //  namespace common_onboard
}  // namespace mp
}  //  namespace hozon
