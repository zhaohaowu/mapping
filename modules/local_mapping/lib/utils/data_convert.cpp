/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: shenliangchen
 *Date: 2023-09-05
 *****************************************************************************/
#include "modules/local_mapping/lib/utils/data_convert.h"

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
    if (msg.lane()[i].lanepos() == hozon::perception::RoadEdge) continue;
    std::shared_ptr<Lane> lane = std::make_shared<Lane>();
    lane->lane_id_ = msg.lane()[i].track_id();
    if (msg.lane()[i].lanepos() == hozon::perception::OTHER) {
      continue;
    }
    DataConvert::ConvertProtoLanePoseType(msg.lane()[i].lanepos(),
                                          &lane->pos_type_);

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

void DataConvert::ConvertProtoLanePoseType(
    const hozon::perception::LanePositionType& raw_lane_pose_type,
    LanePoseType* lane_pose_type) {
  switch (raw_lane_pose_type) {
    case hozon::perception::LanePositionType::BOLLARD_LEFT:
      *lane_pose_type = LanePoseType::BOLLARD_LEFT;
      break;
    case hozon::perception::LanePositionType::FOURTH_LEFT:
      *lane_pose_type = LanePoseType::FOURTH_LEFT;
      break;
    case hozon::perception::LanePositionType::THIRD_LEFT:
      *lane_pose_type = LanePoseType::THIRD_LEFT;
      break;
    case hozon::perception::LanePositionType::ADJACENT_LEFT:
      *lane_pose_type = LanePoseType::ADJACENT_LEFT;
      break;
    case hozon::perception::LanePositionType::EGO_LEFT:
      *lane_pose_type = LanePoseType::EGO_LEFT;
      break;
    case hozon::perception::LanePositionType::EGO_RIGHT:
      *lane_pose_type = LanePoseType::EGO_RIGHT;
      break;
    case hozon::perception::LanePositionType::ADJACENT_RIGHT:
      *lane_pose_type = LanePoseType::ADJACENT_RIGHT;
      break;
    case hozon::perception::LanePositionType::THIRD_RIGHT:
      *lane_pose_type = LanePoseType::THIRD_RIGHT;
      break;
    case hozon::perception::LanePositionType::FOURTH_RIGHT:
      *lane_pose_type = LanePoseType::FOURTH_RIGHT;
      break;
    case hozon::perception::LanePositionType::BOLLARD_RIGHT:
      *lane_pose_type = LanePoseType::BOLLARD_RIGHT;
      break;
    case hozon::perception::LanePositionType::OTHER:
      *lane_pose_type = LanePoseType::OTHER;
      break;
    default:
      break;
  }
}

void DataConvert::ConvertInnerLanePoseType(
    const LanePoseType& inner_lane_pose_type,
    hozon::mapping::LanePositionType* proto_type) {
  switch (inner_lane_pose_type) {
    case LanePoseType::BOLLARD_LEFT:
      *proto_type =
          hozon::mapping::LanePositionType::LanePositionType_BOLLARD_LEFT;
      break;
    case LanePoseType::FOURTH_LEFT:
      *proto_type =
          hozon::mapping::LanePositionType::LanePositionType_FOURTH_LEFT;
      break;
    case LanePoseType::THIRD_LEFT:
      *proto_type =
          hozon::mapping::LanePositionType::LanePositionType_THIRD_LEFT;
      break;
    case LanePoseType::ADJACENT_LEFT:
      *proto_type =
          hozon::mapping::LanePositionType::LanePositionType_ADJACENT_LEFT;
      break;
    case LanePoseType::EGO_LEFT:
      *proto_type = hozon::mapping::LanePositionType::LanePositionType_EGO_LEFT;
      break;
    case LanePoseType::EGO_RIGHT:
      *proto_type =
          hozon::mapping::LanePositionType::LanePositionType_EGO_RIGHT;
      break;
    case LanePoseType::ADJACENT_RIGHT:
      *proto_type =
          hozon::mapping::LanePositionType::LanePositionType_ADJACENT_RIGHT;
      break;
    case LanePoseType::THIRD_RIGHT:
      *proto_type =
          hozon::mapping::LanePositionType::LanePositionType_THIRD_RIGHT;
      break;
    case LanePoseType::FOURTH_RIGHT:
      *proto_type =
          hozon::mapping::LanePositionType::LanePositionType_FOURTH_RIGHT;
      break;
    case LanePoseType::BOLLARD_RIGHT:
      *proto_type =
          hozon::mapping::LanePositionType::LanePositionType_BOLLARD_RIGHT;
      break;
    case LanePoseType::OTHER:
      *proto_type = hozon::mapping::LanePositionType::LanePositionType_OTHER;
      break;
    default:
      *proto_type = hozon::mapping::LanePositionType::LanePositionType_OTHER;
      break;
  }
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
