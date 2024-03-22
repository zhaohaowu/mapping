/********************************************************
 * Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 * Author: Hozon
 *******************************************************/
#include <cstdint>

#include "Eigen/Core"
#include "modules/local_mapping/data_mapping/data_mapping.h"
#include "modules/local_mapping/utils/common.h"
#include "proto/local_mapping/local_map.pb.h"
#include "proto/perception/transport_element.pb.h"

namespace hozon {
namespace mp {

namespace lm {
namespace data_mapping {

bool DataMapping::CvtZebraCrossing2Pb(
    const ZebraCrossingPtr& zebracrossing_msg,
    hozon::mapping::CrossWalk* pb_zebracrossing) {
  if (nullptr == zebracrossing_msg || nullptr == pb_zebracrossing) {
    HLOG_ERROR << "zebracrossing msg  or pb_zebracrossing is nullptr.";
    return false;
  }
  pb_zebracrossing->set_track_id(static_cast<int32_t>(zebracrossing_msg->id));
  // pb_zebracrossing->set_confidence(zebracrossing_msg->confidence);

  for (const auto& item_pt : zebracrossing_msg->vehicle_points) {
    auto arrow_points = pb_zebracrossing->mutable_points();
    auto pb_pt = arrow_points->add_point();
    pb_pt->set_x(item_pt.x());
    pb_pt->set_y(item_pt.y());
    pb_pt->set_z(item_pt.z());
  }

  return true;
}

bool DataMapping::CvtPb2ZebraCrossingMeasurement(
    const hozon::perception::ZebraCrossing& zebracrossing,
    ZebraCrossingPtr zebracrossing_ptr) {
  zebracrossing_ptr->id = zebracrossing.track_id();
  zebracrossing_ptr->confidence = static_cast<float>(zebracrossing.confidence());

  std::vector<Eigen::Vector3d> points;
  points.clear();
  for (const auto& item : zebracrossing.points().point()) {
    Eigen::Vector3d llpt;
    llpt.x() = item.x();
    llpt.y() = item.y();
    llpt.z() = item.z();
    points.push_back(llpt);
  }
  sort(points.begin(), points.end(),
       [](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
         return p1.y() > p2.y();
       });
  std::vector<Eigen::Vector3d> points_order(4);
  if (points[0].x() > points[1].x()) {
    points_order[0] = points[0];
    points_order[1] = points[1];
  } else {
    points_order[0] = points[1];
    points_order[1] = points[0];
  }
  if (points[2].x() > points[3].x()) {
    points_order[2] = points[3];
    points_order[3] = points[2];
  } else {
    points_order[2] = points[2];
    points_order[3] = points[3];
  }
  zebracrossing_ptr->vehicle_points = points_order;
  auto dis = [](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
    return std::sqrt(std::pow(p1.x() - p2.x(), 2) +
                     std::pow(p1.y() - p2.y(), 2));
  };
  zebracrossing_ptr->length = (dis(points_order[0], points_order[3]) +
                               dis(points_order[1], points_order[2])) /
                              2;
  zebracrossing_ptr->width = (dis(points_order[0], points_order[1]) +
                              dis(points_order[2], points_order[3])) /
                             2;
  zebracrossing_ptr->center_point =
      (points_order[0] + points_order[1] + points_order[2] + points_order[3]) /
      4;
  Eigen::Vector3d top_point = (points_order[0] + points_order[3]) / 2;
  Eigen::Vector3d bottom_point = (points_order[1] + points_order[2]) / 2;
  zebracrossing_ptr->heading =
      atan2(top_point.y() - bottom_point.y(), top_point.x() - bottom_point.x());
  return true;
}
}  // namespace data_mapping
}  // namespace lm
}  // namespace mp
}  //  namespace hozon
