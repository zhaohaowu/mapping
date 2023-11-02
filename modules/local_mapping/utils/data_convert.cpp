/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: shenliangchen
 *Date: 2023-09-05
 *****************************************************************************/
#include "modules/local_mapping/utils/data_convert.h"

namespace hozon {
namespace mp {
namespace lm {

void DataConvert::SetLocation(const hozon::localization::Localization msg,
                              std::shared_ptr<Location> location) {}

void DataConvert::SetDr(const hozon::dead_reckoning::DeadReckoning msg,
                        std::shared_ptr<Location> dr_location) {
  dr_location->timestamp_ = msg.header().gnss_stamp();
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

void DataConvert::SetLaneLine(const hozon::perception::TransportElement& msg,
                              std::shared_ptr<Lanes> lanes) {
  lanes->lanes_.clear();
  lanes->timestamp_ = msg.header().gnss_stamp();
  for (size_t i = 0; i < msg.lane_size(); i++) {
    if (msg.lane()[i].lanepos() == hozon::perception::OTHER) continue;
    if (msg.lane()[i].lanetype() == hozon::perception::RoadEdge) continue;
    std::shared_ptr<Lane> lane = std::make_shared<Lane>();
    lane->track_id_ = msg.lane()[i].track_id();
    DataConvert::ConvertProtoLanePos(msg.lane()[i].lanepos(), &lane->lanepos_);
    DataConvert::ConvertProtoLaneType(msg.lane()[i].lanetype(),
                                      &lane->lanetype_);
    // 与感知文海同学沟通结论：proto虽然是分段三次曲线，但实际数据只有一段
    for (size_t j = 0; j < msg.lane()[i].lane_param().cubic_curve_set_size();
         j++) {
      lane->x_start_vrf_ =
          msg.lane()[i].lane_param().cubic_curve_set()[j].start_point_x();
      lane->x_end_vrf_ =
          msg.lane()[i].lane_param().cubic_curve_set()[j].end_point_x();
      lane->lane_fit_a_ = msg.lane()[i].lane_param().cubic_curve_set()[j].c3();
      lane->lane_fit_b_ = msg.lane()[i].lane_param().cubic_curve_set()[j].c2();
      lane->lane_fit_c_ = msg.lane()[i].lane_param().cubic_curve_set()[j].c1();
      lane->lane_fit_d_ = msg.lane()[i].lane_param().cubic_curve_set()[j].c0();
      break;
    }
    lanes->lanes_.emplace_back(*lane);
  }
}

void DataConvert::SetEdgeLine(const hozon::perception::TransportElement& msg,
                              std::shared_ptr<Lanes> lanes) {
  lanes->lanes_.clear();
  lanes->timestamp_ = msg.header().gnss_stamp();
  for (size_t i = 0; i < msg.lane_size(); i++) {
    std::shared_ptr<Lane> lane = std::make_shared<Lane>();
    lane->track_id_ = msg.lane()[i].track_id();
    DataConvert::ConvertProtoLanePos(msg.lane()[i].lanepos(), &lane->lanepos_);

    // 与感知文海同学沟通结论：proto虽然是分段三次曲线，但实际数据只有一段
    for (size_t j = 0; j < msg.lane()[i].lane_param().cubic_curve_set_size();
         j++) {
      lane->x_start_vrf_ =
          msg.lane()[i].lane_param().cubic_curve_set()[j].start_point_x();
      lane->x_end_vrf_ =
          msg.lane()[i].lane_param().cubic_curve_set()[j].end_point_x();
      lane->lane_fit_a_ = msg.lane()[i].lane_param().cubic_curve_set()[j].c3();
      lane->lane_fit_b_ = msg.lane()[i].lane_param().cubic_curve_set()[j].c2();
      lane->lane_fit_c_ = msg.lane()[i].lane_param().cubic_curve_set()[j].c1();
      lane->lane_fit_d_ = msg.lane()[i].lane_param().cubic_curve_set()[j].c0();
      break;
    }
    lanes->lanes_.emplace_back(*lane);
  }
}

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
