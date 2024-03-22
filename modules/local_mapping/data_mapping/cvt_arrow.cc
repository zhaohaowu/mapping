/********************************************************
 * Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 * Author: Hozon
 *******************************************************/

#include "modules/local_mapping/data_mapping/data_mapping.h"

namespace hozon {
namespace mp {
namespace lm {
namespace data_mapping {

static hozon::hdmap::ArrowData::Type CvtArrowType2Pb(ArrowType shape) {
  switch (shape) {
    case ArrowType::STRAIGHT_FORWARD:
      return hozon::hdmap::ArrowData::Type::ArrowData_Type_STRAIGHT;
    case ArrowType::STRAIGHT_FORWARD_OR_TURN_LEFT:
      return hozon::hdmap::ArrowData::Type::ArrowData_Type_STRAIGHT_LEFT_TURN;
    case ArrowType::STRAIGHT_FORWARD_OR_TURN_RIGHT:
      return hozon::hdmap::ArrowData::Type::ArrowData_Type_STRAIGHT_RIGHT_TURN;
    case ArrowType::STRAIGHT_FORWARD_OR_TURN_LEFT_OR_TURN_RIGHT:
      return hozon::hdmap::ArrowData::Type::
          ArrowData_Type_STRAIGHT_LEFT_RIGHT_TURN;
    case ArrowType::STRAIGHT_FORWARD_OR_TURN_AROUND:
      return hozon::hdmap::ArrowData::Type::ArrowData_Type_STRAIGHT_U_TURN;
    case ArrowType::STRAIGHT_FORWARD_OR_TURN_AROUND_OR_TURN_LEFT:
      return hozon::hdmap::ArrowData::Type::ArrowData_Type_STRAIGHT_LEFT_U_TURN;
    case ArrowType::TURN_LEFT:
      return hozon::hdmap::ArrowData::Type::ArrowData_Type_LEFT_TURN;
    case ArrowType::TURN_LEFT_OR_MERGE_LEFT:
      return hozon::hdmap::ArrowData::Type::ArrowData_Type_LEFT_FRONT_TURN;
    case ArrowType::TURN_LEFT_OR_TURN_AROUND:
      return hozon::hdmap::ArrowData::Type::ArrowData_Type_LEFT_U_TURN;
    case ArrowType::TURN_LEFT_OR_TURN_RIGHT:
      return hozon::hdmap::ArrowData::Type::ArrowData_Type_LEFT_RIGHT_TURN;
    case ArrowType::TURN_RIGHT:
      return hozon::hdmap::ArrowData::Type::ArrowData_Type_RIGHT_TURN;
    case ArrowType::TURN_RIGHT_OR_MERGE_RIGHT:
      return hozon::hdmap::ArrowData::Type::ArrowData_Type_RIGHT_FRONT_TURN;
    case ArrowType::TURN_RIGHT_OR_TURN_AROUND:
      return hozon::hdmap::ArrowData::Type::ArrowData_Type_RIGHT_U_TURN;
    case ArrowType::TURN_AROUND:
      return hozon::hdmap::ArrowData::Type::ArrowData_Type_U_TURN;
    case ArrowType::FORBID_TURN_LEFT:
      return hozon::hdmap::ArrowData::Type::ArrowData_Type_FORBID_LEFT_TURN;
    case ArrowType::FORBID_TURN_RIGHT:
      return hozon::hdmap::ArrowData::Type::ArrowData_Type_FORBID_RIGHT_TURN;
    case ArrowType::FORBID_TURN_AROUND:
      return hozon::hdmap::ArrowData::Type::ArrowData_Type_FORBID_U_TURN;
    case ArrowType::FRONT_NEAR_CROSSWALK:
      return hozon::hdmap::ArrowData::Type::ArrowData_Type_FRONT_NEAR_CROSSWALK;
    default:
      return hozon::hdmap::ArrowData::Type::ArrowData_Type_UNKNOWN_TURN;
  }
}

static ArrowType CvtPb2ArrowType(hozon::perception::ArrowType shape) {
  switch (shape) {
    case hozon::perception::ArrowType::STRAIGHT_FORWARD:
      return ArrowType::STRAIGHT_FORWARD;
    case hozon::perception::ArrowType::STRAIGHT_FORWARD_OR_TURN_LEFT:
      return ArrowType::STRAIGHT_FORWARD_OR_TURN_LEFT;
    case hozon::perception::ArrowType::STRAIGHT_FORWARD_OR_TURN_RIGHT:
      return ArrowType::STRAIGHT_FORWARD_OR_TURN_RIGHT;
    case hozon::perception::ArrowType::
        STRAIGHT_FORWARD_OR_TURN_LEFT_OR_TURN_RIGHT:
      return ArrowType::STRAIGHT_FORWARD_OR_TURN_LEFT_OR_TURN_RIGHT;
    case hozon::perception::ArrowType::STRAIGHT_FORWARD_OR_TURN_AROUND:
      return ArrowType::STRAIGHT_FORWARD_OR_TURN_AROUND;
    case hozon::perception::ArrowType::
        STRAIGHT_FORWARD_OR_TURN_AROUND_OR_TURN_LEFT:
      return ArrowType::STRAIGHT_FORWARD_OR_TURN_AROUND_OR_TURN_LEFT;
    case hozon::perception::ArrowType::TURN_LEFT:
      return ArrowType::TURN_LEFT;
    case hozon::perception::ArrowType::TURN_LEFT_OR_MERGE_LEFT:
      return ArrowType::TURN_LEFT_OR_MERGE_LEFT;
    case hozon::perception::ArrowType::TURN_LEFT_OR_TURN_AROUND:
      return ArrowType::TURN_LEFT_OR_TURN_AROUND;
    case hozon::perception::ArrowType::TURN_LEFT_OR_TURN_RIGHT:
      return ArrowType::TURN_LEFT_OR_TURN_RIGHT;
    case hozon::perception::ArrowType::TURN_RIGHT:
      return ArrowType::TURN_RIGHT;
    case hozon::perception::ArrowType::TURN_RIGHT_OR_MERGE_RIGHT:
      return ArrowType::TURN_RIGHT_OR_MERGE_RIGHT;
    case hozon::perception::ArrowType::TURN_RIGHT_OR_TURN_AROUND:
      return ArrowType::TURN_RIGHT_OR_TURN_AROUND;
    case hozon::perception::ArrowType::TURN_AROUND:
      return ArrowType::TURN_AROUND;
    case hozon::perception::ArrowType::FORBID_TURN_LEFT:
      return ArrowType::FORBID_TURN_LEFT;
    case hozon::perception::ArrowType::FORBID_TURN_RIGHT:
      return ArrowType::FORBID_TURN_RIGHT;
    case hozon::perception::ArrowType::FORBID_TURN_AROUND:
      return ArrowType::FORBID_TURN_AROUND;
    case hozon::perception::ArrowType::FRONT_NEAR_CROSSWALK:
      return ArrowType::FRONT_NEAR_CROSSWALK;
    default:
      return ArrowType::UNKNOWN;
  }
}

bool DataMapping::CvtArrow2Pb(const ArrowPtr& arrow_msg,
                              hozon::mapping::Arrow* pb_arrow) {
  if (nullptr == arrow_msg || nullptr == pb_arrow) {
    HLOG_ERROR << "arrow msg  or pb_arrow is nullptr.";
    return false;
  }
  pb_arrow->set_track_id(arrow_msg->id);
  hozon::hdmap::ArrowData::Type send_type = CvtArrowType2Pb(arrow_msg->type);
  pb_arrow->set_arrow_type(send_type);

  for (auto& item_pt : arrow_msg->vehicle_points) {
    auto arrow_points = pb_arrow->mutable_points();
    auto pb_pt = arrow_points->add_point();
    pb_pt->set_x(item_pt.x());
    pb_pt->set_y(item_pt.y());
    pb_pt->set_z(item_pt.z());
  }

  return true;
}

bool DataMapping::CvtPb2ArrowMeasurement(const hozon::perception::Arrow& arrow,
                                         ArrowPtr arrowptr) {
  arrowptr->id = arrow.track_id();
  arrowptr->confidence = static_cast<float>(arrow.confidence());

  std::vector<Eigen::Vector3d> points;
  points.clear();
  for (const auto& item : arrow.points().point()) {
    Eigen::Vector3d llpt;
    llpt.x() = item.x();
    llpt.y() = item.y();
    llpt.z() = item.z();
    points.push_back(llpt);
  }
  sort(points.begin(), points.end(),
       [](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
         return p1.x() > p2.x();
       });
  std::vector<Eigen::Vector3d> points_order(4);
  if (points[0].y() > points[1].y()) {
    points_order[0] = points[0];
    points_order[3] = points[1];
  } else {
    points_order[0] = points[1];
    points_order[3] = points[0];
  }
  if (points[2].y() > points[3].y()) {
    points_order[1] = points[2];
    points_order[2] = points[3];
  } else {
    points_order[1] = points[3];
    points_order[2] = points[2];
  }
  arrowptr->vehicle_points = points_order;
  auto dis = [](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
    return std::sqrt(std::pow(p1.x() - p2.x(), 2) +
                     std::pow(p1.y() - p2.y(), 2));
  };
  arrowptr->length = (dis(points_order[0], points_order[1]) +
                      dis(points_order[2], points_order[3])) /
                     2;
  arrowptr->width = (dis(points_order[0], points_order[3]) +
                     dis(points_order[1], points_order[2])) /
                    2;
  arrowptr->center_point =
      (points_order[0] + points_order[1] + points_order[2] + points_order[3]) /
      4;
  Eigen::Vector3d top_point = (points_order[0] + points_order[3]) / 2;
  Eigen::Vector3d bottom_point = (points_order[1] + points_order[2]) / 2;
  arrowptr->heading =
      atan2(top_point.y() - bottom_point.y(), top_point.x() - bottom_point.x());
  arrowptr->type = CvtPb2ArrowType(arrow.type());
  return true;
}

}  // namespace data_mapping
}  // namespace lm
}  // namespace mp
}  //  namespace hozon
