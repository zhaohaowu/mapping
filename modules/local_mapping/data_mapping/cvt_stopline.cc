/********************************************************
 * Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 * Author: Hozon
 *******************************************************/

#include "Eigen/Core"
#include "modules/local_mapping/data_mapping/data_mapping.h"
#include "proto/local_mapping/local_map.pb.h"
#include "proto/perception/transport_element.pb.h"
namespace hozon {
namespace mp {

namespace lm {
namespace data_mapping {

bool DataMapping::CvtStopLine2Pb(const StopLinePtr& stopline_msg,
                                 hozon::mapping::StopLine* pb_stopline) {
  if (nullptr == stopline_msg || nullptr == pb_stopline) {
    HLOG_ERROR << "stopline msg  or pb_arrow is nullptr.";
    return false;
  }
  pb_stopline->set_track_id(stopline_msg->id);

  auto* left_point = pb_stopline->mutable_left_point();
  left_point->set_x(stopline_msg->left_point.x());
  left_point->set_y(stopline_msg->left_point.y());
  left_point->set_z(stopline_msg->left_point.z());

  auto* right_point = pb_stopline->mutable_right_point();
  right_point->set_x(stopline_msg->right_point.x());
  right_point->set_y(stopline_msg->right_point.y());
  right_point->set_z(stopline_msg->right_point.z());

  return true;
}

bool DataMapping::CvtPb2StopLineMeasurement(
    const hozon::perception::StopLine& stopline, StopLinePtr stoplineptr) {
  stoplineptr->id = stopline.track_id();
  stoplineptr->confidence = static_cast<float>(stopline.confidence());

  stoplineptr->left_point.x() = stopline.left_point().x();
  stoplineptr->left_point.y() = stopline.left_point().y();
  stoplineptr->left_point.z() = stopline.left_point().z();

  stoplineptr->right_point.x() = stopline.right_point().x();
  stoplineptr->right_point.y() = stopline.right_point().y();
  stoplineptr->right_point.z() = stopline.right_point().z();

  stoplineptr->center_point.x() =
      0.5 * (stopline.left_point().x() + stopline.right_point().x());
  stoplineptr->center_point.y() =
      0.5 * (stopline.left_point().y() + stopline.right_point().y());
  stoplineptr->center_point.z() =
      0.5 * (stopline.left_point().z() + stopline.right_point().z());

  stoplineptr->length =
      (stoplineptr->left_point - stoplineptr->right_point).norm();
  stoplineptr->heading =
      atan2(stoplineptr->left_point.y() - stoplineptr->right_point.y(),
            stoplineptr->left_point.x() - stoplineptr->right_point.x());
  stoplineptr->heading =
      (stoplineptr->heading > -M_PI && stoplineptr->heading < 0)
          ? stoplineptr->heading + M_PI
          : stoplineptr->heading;

  return true;
}

}  // namespace data_mapping
}  // namespace lm
}  // namespace mp
}  //  namespace hozon
