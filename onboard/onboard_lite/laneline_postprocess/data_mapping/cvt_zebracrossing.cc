/********************************************************
 * Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 * Author: Hozon
 *******************************************************/

#include "onboard/onboard_lite/laneline_postprocess/data_mapping/data_mapping.h"
#include "perception-base/base/scene/zebra_crossing.h"
#include "proto/perception/transport_element.pb.h"

namespace hozon {
namespace mp {
namespace common_onboard {

static bool CvtZebraCrossingToPb(
    const perception_base::ZebraCrossingPtr& zebracrossing_msg,
    hozon::perception::ZebraCrossing* pb_object);

bool DataMapping::CvtMultiZebraCrossingsToPb(
    const std::vector<perception_base::ZebraCrossingPtr>& zebracrossing_msgs,
    NetaTransportElementPtr pb_objects) {
  if (zebracrossing_msgs.size() == 0) {
    HLOG_DEBUG << "zebracrossing msg size is 0";
    return true;
  }
  if (nullptr == pb_objects) {
    HLOG_ERROR << "pb_objects is nullptr.";
    return false;
  }

  uint32_t zebracrossing_size = zebracrossing_msgs.size();
  for (size_t i = 0; i < zebracrossing_size; ++i) {
    auto pb_zebracrossing = pb_objects->add_zebra_crossing();
    if (!CvtZebraCrossingToPb(zebracrossing_msgs[i], pb_zebracrossing)) {
      HLOG_ERROR << "cvt zebracrossing to proto struct failed.";
      return false;
    }
  }
  return true;
}

bool DataMapping::CvtZebraCrossingToPb(
    const perception_base::ZebraCrossingPtr& zebracrossing_msg,
    hozon::perception::ZebraCrossing* pb_zebracrossing) {
  if (nullptr == zebracrossing_msg || nullptr == pb_zebracrossing) {
    HLOG_ERROR << "zebracrossing msg  or pb_zebracrossing is nullptr.";
    return false;
  }
  pb_zebracrossing->set_track_id(zebracrossing_msg->id);
  pb_zebracrossing->set_confidence(zebracrossing_msg->confidence);

  if (zebracrossing_msg->point_set_3d.size() != 0) {
    for (auto& item_pt : zebracrossing_msg->point_set_3d) {
      auto arrow_points = pb_zebracrossing->mutable_points();
      auto pb_pt = arrow_points->add_point();
      pb_pt->set_x(item_pt.x);
      pb_pt->set_y(item_pt.y);
      pb_pt->set_z(item_pt.z);
    }
  } else if (zebracrossing_msg->point_set_2d.size() != 0) {
    for (auto& item_pt : zebracrossing_msg->point_set_2d) {
      auto arrow_points = pb_zebracrossing->mutable_points();
      auto pb_pt = arrow_points->add_point();
      pb_pt->set_x(item_pt.x);
      pb_pt->set_y(item_pt.y);
      pb_pt->set_z(0.0);
      // HLOG_DEBUG << "============x: " << item_pt.x << " y:" << item_pt.y;
    }
  }

  return true;
}

bool DataMapping::CvtZebraCrossingMeasurementToPb(
    const std::vector<perception_base::ZebraCrossingMeasurementPtr>&
        zebracrossing_measure,
    hozon::perception::TransportElement* transport_element) {
  for (auto& item : zebracrossing_measure) {
    auto pb_zebracrossing = transport_element->add_zebra_crossing();
    pb_zebracrossing->set_track_id(item->id);
    pb_zebracrossing->set_confidence(item->confidence);
    if (item->point_set_3d.size() != 0) {
      for (auto& item_pt : item->point_set_3d) {
        auto zebracross_points = pb_zebracrossing->mutable_points();
        auto pb_pt = zebracross_points->add_point();
        pb_pt->set_x(item_pt.x);
        pb_pt->set_y(item_pt.y);
        pb_pt->set_z(item_pt.z);
      }
    } else if (item->point_set_2d.size() != 0) {
      for (auto& item_pt : item->point_set_2d) {
        auto zebracross_points = pb_zebracrossing->mutable_points();
        auto pb_pt = zebracross_points->add_point();
        pb_pt->set_x(item_pt.x);
        pb_pt->set_y(item_pt.y);
        pb_pt->set_z(0.0);
      }
    }
  }
  return true;
}
bool DataMapping::CvtPb2ZebraCrossingMeasurement(
    const hozon::perception::ZebraCrossing& zebracrossing,
    perception_base::ZebraCrossingMeasurementPtr zebracrossingptr) {
  zebracrossingptr->id = zebracrossing.track_id();
  zebracrossingptr->confidence = zebracrossing.confidence();

  for (auto& item : zebracrossing.points().point()) {
    perception_base::Point3DF llpt;
    llpt.x = item.x();
    llpt.y = item.y();
    llpt.z = item.z();

    zebracrossingptr->point_set_3d.push_back(std::move(llpt));
  }

  return true;
}
}  //  namespace common_onboard
}  // namespace mp
}  //  namespace hozon
