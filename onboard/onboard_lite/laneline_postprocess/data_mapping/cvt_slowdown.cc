/********************************************************
 * Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 * Author: Hozon
 *******************************************************/

#include "onboard/onboard_lite/laneline_postprocess/data_mapping/data_mapping.h"
#include "perception-base/base/scene/slowdown.h"
#include "proto/perception/transport_element.pb.h"

namespace hozon {
namespace mp {
namespace common_onboard {

static bool CvtSlowDownToPb(const perception_base::SlowDownPtr& slowdown_msg,
                            hozon::perception::SlowDown* pb_object);

bool DataMapping::CvtMultiSlowDownsToPb(
    const std::vector<perception_base::SlowDownPtr>& slowdown_msgs,
    NetaTransportElementPtr pb_objects) {
  if (slowdown_msgs.size() == 0) {
    HLOG_DEBUG << "slowdowns msg size is 0";
    return true;
  }
  if (nullptr == pb_objects) {
    HLOG_ERROR << "pb_objects is nullptr.";
    return false;
  }

  uint32_t slowdown_size = slowdown_msgs.size();
  for (size_t i = 0; i < slowdown_size; ++i) {
    auto pb_slowdown = pb_objects->add_slow_downs();
    if (!CvtSlowDownToPb(slowdown_msgs[i], pb_slowdown)) {
      HLOG_ERROR << "cvt slowdowns to proto struct failed.";
      return false;
    }
  }
  return true;
}

bool DataMapping::CvtSlowDownToPb(
    const perception_base::SlowDownPtr& slowdown_msg,
    hozon::perception::SlowDown* pb_slowdown) {
  if (nullptr == slowdown_msg || nullptr == pb_slowdown) {
    HLOG_ERROR << "slowdown msg  or pb_slowdown is nullptr.";
    return false;
  }
  pb_slowdown->set_track_id(slowdown_msg->id);
  pb_slowdown->set_confidence(slowdown_msg->confidence);

  if (slowdown_msg->point_set_3d.size() != 0) {
    for (auto& item_pt : slowdown_msg->point_set_3d) {
      auto arrow_points = pb_slowdown->mutable_points();
      auto pb_pt = arrow_points->add_point();
      pb_pt->set_x(item_pt.x);
      pb_pt->set_y(item_pt.y);
      pb_pt->set_z(item_pt.z);
    }
  } else if (slowdown_msg->point_set_2d.size() != 0) {
    for (auto& item_pt : slowdown_msg->point_set_2d) {
      auto arrow_points = pb_slowdown->mutable_points();
      auto pb_pt = arrow_points->add_point();
      pb_pt->set_x(item_pt.x);
      pb_pt->set_y(item_pt.y);
      pb_pt->set_z(0.0);
      // HLOG_DEBUG << "============x: " << item_pt.x << " y:" << item_pt.y;
    }
  }

  return true;
}

bool DataMapping::CvtSlowDownMeasurementToPb(
    const std::vector<perception_base::SlowdownMeasurementPtr>&
        slowdown_measure,
    hozon::perception::TransportElement* transport_element) {
  for (auto& item : slowdown_measure) {
    auto pb_slowdown = transport_element->add_turn_waiting_zone();
    pb_slowdown->set_track_id(item->id);
    // pb_slowdown->set_type(CvtArrowType2Pb(item->type));
    pb_slowdown->set_confidence(item->confidence);
    if (item->point_set_3d.size() != 0) {
      for (auto& item_pt : item->point_set_3d) {
        auto arrow_points = pb_slowdown->mutable_points();
        auto pb_pt = arrow_points->add_point();
        pb_pt->set_x(item_pt.x);
        pb_pt->set_y(item_pt.y);
        pb_pt->set_z(item_pt.z);
      }
    } else if (item->point_set_2d.size() != 0) {
      for (auto& item_pt : item->point_set_2d) {
        auto arrow_points = pb_slowdown->mutable_points();
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
