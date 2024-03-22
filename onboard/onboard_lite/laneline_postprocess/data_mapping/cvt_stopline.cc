/********************************************************
 * Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 * Author: Hozon
 *******************************************************/

#include "onboard/onboard_lite/laneline_postprocess/data_mapping/data_mapping.h"
#include "perception-base/base/scene/stopline.h"
#include "proto/perception/transport_element.pb.h"
namespace hozon {
namespace mp {
namespace common_onboard {

static bool CvtStopLineToPb(const perception_base::StopLinePtr& stopline_msg,
                            hozon::perception::StopLine* pb_object);

bool DataMapping::CvtMultiStopLinesToPb(
    const std::vector<perception_base::StopLinePtr>& stopline_msgs,
    NetaTransportElementPtr pb_objects) {
  if (stopline_msgs.size() == 0) {
    HLOG_DEBUG << "stoplines msg size is 0";
    return true;
  }
  if (nullptr == pb_objects) {
    HLOG_ERROR << "pb_objects is nullptr.";
    return false;
  }

  uint32_t stopline_size = stopline_msgs.size();
  for (size_t i = 0; i < stopline_size; ++i) {
    auto pb_stopline = pb_objects->add_stopline();
    if (!CvtStopLineToPb(stopline_msgs[i], pb_stopline)) {
      HLOG_ERROR << "cvt stopline to proto struct failed.";
      return false;
    }
  }
  return true;
}

bool DataMapping::CvtStopLineToPb(
    const perception_base::StopLinePtr& stopline_msg,
    hozon::perception::StopLine* pb_stopline) {
  if (nullptr == stopline_msg || nullptr == pb_stopline) {
    HLOG_ERROR << "stopline msg  or pb_arrow is nullptr.";
    return false;
  }
  pb_stopline->set_track_id(stopline_msg->id);
  pb_stopline->set_confidence(stopline_msg->confidence);

  if (stopline_msg->point_set_3d.size() != 20) {
    HLOG_ERROR << "stopline msg point num error.";
    return false;
  }

  auto left_point = pb_stopline->mutable_left_point();
  left_point->set_x(stopline_msg->point_set_3d[0].x);
  left_point->set_y(stopline_msg->point_set_3d[0].y);
  left_point->set_z(stopline_msg->point_set_3d[0].z);

  auto right_point = pb_stopline->mutable_right_point();
  right_point->set_x(stopline_msg->point_set_3d[19].x);
  right_point->set_y(stopline_msg->point_set_3d[19].y);
  right_point->set_z(stopline_msg->point_set_3d[19].z);

  return true;
}

bool DataMapping::CvtStopLineMeasurementToPb(
    const std::vector<perception_base::StopLineMeasurementPtr>&
        stopline_measure,
    hozon::perception::TransportElement* transport_element) {
  for (auto& item : stopline_measure) {
    auto pb_stopline = transport_element->add_stopline();
    pb_stopline->set_track_id(item->id);
    pb_stopline->set_confidence(item->confidence);

    if (item->point_set_3d.size() != 20) {
      return false;
    }

    auto left_point = pb_stopline->mutable_left_point();
    left_point->set_x(item->point_set_3d[0].x);
    left_point->set_y(item->point_set_3d[0].y);
    left_point->set_z(item->point_set_3d[0].z);

    auto right_point = pb_stopline->mutable_right_point();
    right_point->set_x(item->point_set_3d[19].x);
    right_point->set_y(item->point_set_3d[19].y);
    right_point->set_z(item->point_set_3d[19].z);
  }
  return true;
}
bool DataMapping::CvtPb2StopLineMeasurement(
    const hozon::perception::StopLine& stopline,
    perception_base::StopLineMeasurementPtr stoplineptr) {
  stoplineptr->id = stopline.track_id();
  stoplineptr->confidence = stopline.confidence();
  stoplineptr->point_set_3d.resize(20);
  perception_base::Point3DF left_pt, right_pt;

  stoplineptr->point_set_3d[0].x = stopline.left_point().x();
  stoplineptr->point_set_3d[0].y = stopline.left_point().y();
  stoplineptr->point_set_3d[0].z = stopline.left_point().z();

  stoplineptr->point_set_3d[19].x = stopline.right_point().x();
  stoplineptr->point_set_3d[19].y = stopline.right_point().y();
  stoplineptr->point_set_3d[19].z = stopline.right_point().z();

  return true;
}

}  //  namespace common_onboard
}  // namespace mp
}  //  namespace hozon
