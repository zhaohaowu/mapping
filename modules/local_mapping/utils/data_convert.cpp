/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: shenliangchen
 *Date: 2023-09-05
 *****************************************************************************/
#include "modules/local_mapping/utils/data_convert.h"

namespace hozon {
namespace mp {
namespace lm {

void DataConvert::SetLocation(const hozon::localization::Localization& msg,
                              Location* location) {}

void DataConvert::SetDr(const hozon::dead_reckoning::DeadReckoning& msg,
                        Location* dr_location) {
  dr_location->timestamp_ = msg.header().data_stamp();
  dr_location->position_.x() = msg.pose().pose_local().position().x();
  dr_location->position_.y() = msg.pose().pose_local().position().y();
  dr_location->position_.z() = msg.pose().pose_local().position().z();
  dr_location->quaternion_.x() = msg.pose().pose_local().quaternion().x();
  dr_location->quaternion_.y() = msg.pose().pose_local().quaternion().y();
  dr_location->quaternion_.z() = msg.pose().pose_local().quaternion().z();
  dr_location->quaternion_.w() = msg.pose().pose_local().quaternion().w();
  dr_location->euler_angle_.x() = msg.pose().pose_local().euler_angle().x();
  dr_location->euler_angle_.y() = msg.pose().pose_local().euler_angle().y();
  dr_location->euler_angle_.z() = msg.pose().pose_local().euler_angle().z();
  dr_location->linear_vrf_.x() = msg.velocity().twist_vrf().linear_vrf().x();
  dr_location->linear_vrf_.y() = msg.velocity().twist_vrf().linear_vrf().y();
  dr_location->linear_vrf_.z() = msg.velocity().twist_vrf().linear_vrf().z();
  dr_location->angular_vrf_.x() = msg.velocity().twist_vrf().angular_vrf().x();
  dr_location->angular_vrf_.y() = msg.velocity().twist_vrf().angular_vrf().y();
  dr_location->angular_vrf_.z() = msg.velocity().twist_vrf().angular_vrf().z();
  dr_location->heading_ = msg.pose().pose_local().heading();
}

void DataConvert::SetLaneLinePoint(
    const hozon::perception::TransportElement& msg, Perception* lane_lines) {
  lane_lines->timestamp_ = msg.header().data_stamp();
  for (const auto& lane_line : msg.lane()) {
    if (lane_line.points().empty() ||
        lane_line.lanepos() == hozon::perception::OTHER ||
        lane_line.lanetype() == hozon::perception::RoadEdge) {
      continue;
    }
    LaneLine lane_line_tmp;
    lane_line_tmp.track_id_ = lane_line.track_id();
    // HLOG_ERROR << "lane_line.track_id(): " << lane_line.track_id();
    DataConvert::ConvertProtoLanePos(lane_line.lanepos(),
                                     &lane_line_tmp.lanepos_);
    DataConvert::ConvertProtoLaneType(lane_line.lanetype(),
                                      &lane_line_tmp.lanetype_);
    for (const auto& point : lane_line.points()) {
      Eigen::Vector3d point_tmp = {point.x(), point.y(), point.z()};
      // HLOG_ERROR << "point_tmp: " << point_tmp;
      lane_line_tmp.points_.push_back(point_tmp);
    }
    lane_line_tmp.start_point_x_ = lane_line.points()[0].x();
    lane_line_tmp.end_point_x_ =
        lane_line.points()[lane_line.points().size() - 1].x();

    if (lane_line_tmp.points_.empty() || lane_line_tmp.end_point_x_ > 150) {
      continue;
    }
    CommonUtil::FitLaneLine(lane_line_tmp.points_, &lane_line_tmp);
    lane_line_tmp.points_.clear();
    double x = lane_line_tmp.start_point_x_;
    while (x < lane_line_tmp.end_point_x_) {
      double y = lane_line_tmp.c0_ + lane_line_tmp.c1_ * x +
                 lane_line_tmp.c2_ * x * x + lane_line_tmp.c3_ * x * x * x;
      Eigen::Vector3d point_tmp = {x, y, 0};
      lane_line_tmp.points_.emplace_back(point_tmp);
      x++;
    }
    lane_lines->lane_lines_.push_back(lane_line_tmp);
  }
}

void DataConvert::SetLaneLine(const hozon::perception::TransportElement& msg,
                              Perception* lane_lines) {
  lane_lines->timestamp_ = msg.header().data_stamp();
  for (const auto& lane_line : msg.lane()) {
    if (lane_line.lanepos() == hozon::perception::OTHER ||
        lane_line.lanetype() == hozon::perception::RoadEdge ||
        lane_line.lane_param().cubic_curve_set().empty()) {
      continue;
    }
    LaneLine lane_line_tmp;
    lane_line_tmp.track_id_ = lane_line.track_id();
    DataConvert::ConvertProtoLanePos(lane_line.lanepos(),
                                     &lane_line_tmp.lanepos_);
    DataConvert::ConvertProtoLaneType(lane_line.lanetype(),
                                      &lane_line_tmp.lanetype_);
    auto param = lane_line.lane_param().cubic_curve_set()[0];
    lane_line_tmp.start_point_x_ = param.start_point_x();
    lane_line_tmp.end_point_x_ = param.end_point_x();
    lane_line_tmp.c3_ = param.c3();
    lane_line_tmp.c2_ = param.c2();
    lane_line_tmp.c1_ = param.c1();
    lane_line_tmp.c0_ = param.c0();
    double x = lane_line_tmp.start_point_x_;
    while (x < lane_line_tmp.end_point_x_) {
      double y = lane_line_tmp.c0_ + lane_line_tmp.c1_ * x +
                 lane_line_tmp.c2_ * x * x + lane_line_tmp.c3_ * x * x * x;
      Eigen::Vector3d point_tmp = {x, y, 0};
      lane_line_tmp.points_.emplace_back(point_tmp);
      x++;
    }
    if (lane_line_tmp.points_.empty()) {
      continue;
    }
    lane_lines->lane_lines_.push_back(lane_line_tmp);
  }
}

void DataConvert::SetEdgeLine(const hozon::perception::TransportElement& msg,
                              Perception* lane_edges) {}

void DataConvert::ConvertProtoLanePos(
    const hozon::perception::LanePositionType& raw_lanepos,
    LanePositionType* lanepos) {
  switch (raw_lanepos) {
    case hozon::perception::LanePositionType::BOLLARD_LEFT:
      *lanepos = LanePositionType::BOLLARD_LEFT;
      break;
    case hozon::perception::LanePositionType::FOURTH_LEFT:
      *lanepos = LanePositionType::FOURTH_LEFT;
      break;
    case hozon::perception::LanePositionType::THIRD_LEFT:
      *lanepos = LanePositionType::THIRD_LEFT;
      break;
    case hozon::perception::LanePositionType::ADJACENT_LEFT:
      *lanepos = LanePositionType::ADJACENT_LEFT;
      break;
    case hozon::perception::LanePositionType::EGO_LEFT:
      *lanepos = LanePositionType::EGO_LEFT;
      break;
    case hozon::perception::LanePositionType::EGO_RIGHT:
      *lanepos = LanePositionType::EGO_RIGHT;
      break;
    case hozon::perception::LanePositionType::ADJACENT_RIGHT:
      *lanepos = LanePositionType::ADJACENT_RIGHT;
      break;
    case hozon::perception::LanePositionType::THIRD_RIGHT:
      *lanepos = LanePositionType::THIRD_RIGHT;
      break;
    case hozon::perception::LanePositionType::FOURTH_RIGHT:
      *lanepos = LanePositionType::FOURTH_RIGHT;
      break;
    case hozon::perception::LanePositionType::BOLLARD_RIGHT:
      *lanepos = LanePositionType::BOLLARD_RIGHT;
      break;
    case hozon::perception::LanePositionType::OTHER:
      *lanepos = LanePositionType::OTHER;
      break;
    default:
      break;
  }
}

void DataConvert::ConvertProtoLaneType(
    const hozon::perception::LaneType& raw_lanetype, LaneType* lanetype) {
  switch (raw_lanetype) {
    case hozon::perception::LaneType::Unknown:
      *lanetype = LaneType::Unknown;
      break;
    case hozon::perception::LaneType::SolidLine:
      *lanetype = LaneType::SolidLine;
      break;
    case hozon::perception::LaneType::DashedLine:
      *lanetype = LaneType::DashedLine;
      break;
    case hozon::perception::LaneType::ShortDashedLine:
      *lanetype = LaneType::ShortDashedLine;
      break;
    case hozon::perception::LaneType::DoubleSolidLine:
      *lanetype = LaneType::DoubleSolidLine;
      break;
    case hozon::perception::LaneType::DoubleDashedLine:
      *lanetype = LaneType::DoubleDashedLine;
      break;
    case hozon::perception::LaneType::LeftSolidRightDashed:
      *lanetype = LaneType::LeftSolidRightDashed;
      break;
    case hozon::perception::LaneType::RightSolidLeftDashed:
      *lanetype = LaneType::RightSolidLeftDashed;
      break;
    case hozon::perception::LaneType::ShadedArea:
      *lanetype = LaneType::ShadedArea;
      break;
    case hozon::perception::LaneType::LaneVirtualMarking:
      *lanetype = LaneType::LaneVirtualMarking;
      break;
    case hozon::perception::LaneType::IntersectionVirualMarking:
      *lanetype = LaneType::IntersectionVirualMarking;
      break;
    case hozon::perception::LaneType::CurbVirtualMarking:
      *lanetype = LaneType::CurbVirtualMarking;
      break;
    case hozon::perception::LaneType::UnclosedRoad:
      *lanetype = LaneType::UnclosedRoad;
      break;
    case hozon::perception::LaneType::RoadVirtualLine:
      *lanetype = LaneType::RoadVirtualLine;
      break;
    case hozon::perception::LaneType::LaneChangeVirtualLine:
      *lanetype = LaneType::LaneChangeVirtualLine;
      break;
    case hozon::perception::LaneType::RoadEdge:
      *lanetype = LaneType::RoadEdge;
      break;
    case hozon::perception::LaneType::Other:
      *lanetype = LaneType::Other;
      break;
    default:
      break;
  }
}

void DataConvert::ConvertInnerLanePos(
    const LanePositionType& inner_lanepos,
    hozon::mapping::LanePositionType* lanepos) {
  switch (inner_lanepos) {
    case LanePositionType::BOLLARD_LEFT:
      *lanepos =
          hozon::mapping::LanePositionType::LanePositionType_BOLLARD_LEFT;
      break;
    case LanePositionType::FOURTH_LEFT:
      *lanepos = hozon::mapping::LanePositionType::LanePositionType_FOURTH_LEFT;
      break;
    case LanePositionType::THIRD_LEFT:
      *lanepos = hozon::mapping::LanePositionType::LanePositionType_THIRD_LEFT;
      break;
    case LanePositionType::ADJACENT_LEFT:
      *lanepos =
          hozon::mapping::LanePositionType::LanePositionType_ADJACENT_LEFT;
      break;
    case LanePositionType::EGO_LEFT:
      *lanepos = hozon::mapping::LanePositionType::LanePositionType_EGO_LEFT;
      break;
    case LanePositionType::EGO_RIGHT:
      *lanepos = hozon::mapping::LanePositionType::LanePositionType_EGO_RIGHT;
      break;
    case LanePositionType::ADJACENT_RIGHT:
      *lanepos =
          hozon::mapping::LanePositionType::LanePositionType_ADJACENT_RIGHT;
      break;
    case LanePositionType::THIRD_RIGHT:
      *lanepos = hozon::mapping::LanePositionType::LanePositionType_THIRD_RIGHT;
      break;
    case LanePositionType::FOURTH_RIGHT:
      *lanepos =
          hozon::mapping::LanePositionType::LanePositionType_FOURTH_RIGHT;
      break;
    case LanePositionType::BOLLARD_RIGHT:
      *lanepos =
          hozon::mapping::LanePositionType::LanePositionType_BOLLARD_RIGHT;
      break;
    case LanePositionType::OTHER:
      *lanepos = hozon::mapping::LanePositionType::LanePositionType_OTHER;
      break;
    default:
      break;
  }
}

void DataConvert::ConvertInnerLaneType(const LaneType& inner_lanetype,
                                       hozon::mapping::LaneType* lanetype) {
  switch (inner_lanetype) {
    case LaneType::Unknown:
      *lanetype = hozon::mapping::LaneType::LaneType_UNKNOWN;
      break;
    case LaneType::SolidLine:
      *lanetype = hozon::mapping::LaneType::LaneType_SOLID;
      break;
    case LaneType::DashedLine:
      *lanetype = hozon::mapping::LaneType::LaneType_DASHED;
      break;
    case LaneType::ShortDashedLine:
      *lanetype = hozon::mapping::LaneType::LaneType_SHORT_DASHED;
      break;
    case LaneType::DoubleSolidLine:
      *lanetype = hozon::mapping::LaneType::LaneType_DOUBLE_SOLID;
      break;
    case LaneType::DoubleDashedLine:
      *lanetype = hozon::mapping::LaneType::LaneType_DOUBLE_DASHED;
      break;
    case LaneType::LeftSolidRightDashed:
      *lanetype = hozon::mapping::LaneType::LaneType_LEFT_SOLID_RIGHT_DASHED;
      break;
    case LaneType::RightSolidLeftDashed:
      *lanetype = hozon::mapping::LaneType::LaneType_RIGHT_SOLID_LEFT_DASHED;
      break;
    case LaneType::ShadedArea:
      *lanetype = hozon::mapping::LaneType::LaneType_SHADED_AREA;
      break;
    case LaneType::LaneVirtualMarking:
      *lanetype = hozon::mapping::LaneType::LaneType_LANE_VIRTUAL_MARKING;
      break;
    case LaneType::IntersectionVirualMarking:
      *lanetype =
          hozon::mapping::LaneType::LaneType_INTERSECTION_VIRTUAL_MARKING;
      break;
    case LaneType::CurbVirtualMarking:
      *lanetype = hozon::mapping::LaneType::LaneType_CURB_VIRTUAL_MARKING;
      break;
    case LaneType::UnclosedRoad:
      *lanetype = hozon::mapping::LaneType::LaneType_UNCLOSED_ROAD;
      break;
    case LaneType::RoadVirtualLine:
      *lanetype = hozon::mapping::LaneType::LaneType_ROAD_VIRTUAL;
      break;
    case LaneType::LaneChangeVirtualLine:
      *lanetype = hozon::mapping::LaneType::LaneType_LANE_CHANG_VIRTUAL;
      break;
    case LaneType::Other:
      *lanetype = hozon::mapping::LaneType::LaneType_OTHER;
      break;
    default:
      break;
  }
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
