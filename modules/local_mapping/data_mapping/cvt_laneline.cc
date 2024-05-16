/********************************************************
 * Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 * Author: Hozon
 *******************************************************/
#include <string>

#include "modules/local_mapping/data_mapping/data_mapping.h"
#include "proto/local_mapping/local_map.pb.h"
#include "proto/perception/transport_element.pb.h"

namespace hozon {
namespace mp {
namespace lm {
namespace data_mapping {

static CrossType CvtPbCrossPointType(hozon::perception::CrossType type);

static CrossType CvtPbCrossPointType(hozon::perception::CrossType type) {
  switch (type) {
    case hozon::perception::CrossType::SPLIT:
      return CrossType::SPLIT;
    case hozon::perception::CrossType::MERGE:
      return CrossType::MERGE;
    default:
      return CrossType::UNKNOWN;
  }
}

static hozon::perception::LanePositionType CvtLanePosType2TePb(
    LaneLinePosition type) {
  switch (type) {
    case LaneLinePosition::OTHER:
      return hozon::perception::LanePositionType::OTHER;
    case LaneLinePosition::FOURTH_LEFT:
      return hozon::perception::LanePositionType::FOURTH_LEFT;
    case LaneLinePosition::THIRD_LEFT:
      return hozon::perception::LanePositionType::THIRD_LEFT;
    case LaneLinePosition::ADJACENT_LEFT:
      return hozon::perception::LanePositionType::ADJACENT_LEFT;
    case LaneLinePosition::EGO_LEFT:
      return hozon::perception::LanePositionType::EGO_LEFT;
    case LaneLinePosition::EGO_RIGHT:
      return hozon::perception::LanePositionType::EGO_RIGHT;
    case LaneLinePosition::ADJACENT_RIGHT:
      return hozon::perception::LanePositionType::ADJACENT_RIGHT;
    case LaneLinePosition::THIRD_RIGHT:
      return hozon::perception::LanePositionType::THIRD_RIGHT;
    case LaneLinePosition::FOURTH_RIGHT:
      return hozon::perception::LanePositionType::FOURTH_RIGHT;
    default:
      return hozon::perception::LanePositionType::OTHER;
  }
}

static hozon::mapping::LanePositionType CvtLanePosType2Pb(
    LaneLinePosition type) {
  switch (type) {
    case LaneLinePosition::OTHER:
      return hozon::mapping::LanePositionType::LanePositionType_OTHER;
    case LaneLinePosition::FOURTH_LEFT:
      return hozon::mapping::LanePositionType::LanePositionType_FOURTH_LEFT;
    case LaneLinePosition::THIRD_LEFT:
      return hozon::mapping::LanePositionType::LanePositionType_THIRD_LEFT;
    case LaneLinePosition::ADJACENT_LEFT:
      return hozon::mapping::LanePositionType::LanePositionType_ADJACENT_LEFT;
    case LaneLinePosition::EGO_LEFT:
      return hozon::mapping::LanePositionType::LanePositionType_EGO_LEFT;
    case LaneLinePosition::EGO_RIGHT:
      return hozon::mapping::LanePositionType::LanePositionType_EGO_RIGHT;
    case LaneLinePosition::ADJACENT_RIGHT:
      return hozon::mapping::LanePositionType::LanePositionType_ADJACENT_RIGHT;
    case LaneLinePosition::THIRD_RIGHT:
      return hozon::mapping::LanePositionType::LanePositionType_THIRD_RIGHT;
    case LaneLinePosition::FOURTH_RIGHT:
      return hozon::mapping::LanePositionType::LanePositionType_FOURTH_RIGHT;
    default:
      return hozon::mapping::LanePositionType::LanePositionType_OTHER;
  }
}

static LaneLinePosition CvtPbLanePosType(
    hozon::perception::LanePositionType type) {
  switch (type) {
    case hozon::perception::LanePositionType::OTHER:
      return LaneLinePosition::OTHER;
    case hozon::perception::LanePositionType::FOURTH_LEFT:
      return LaneLinePosition::FOURTH_LEFT;
    case hozon::perception::LanePositionType::THIRD_LEFT:
      return LaneLinePosition::THIRD_LEFT;
    case hozon::perception::LanePositionType::ADJACENT_LEFT:
      return LaneLinePosition::ADJACENT_LEFT;
    case hozon::perception::LanePositionType::EGO_LEFT:
      return LaneLinePosition::EGO_LEFT;
    case hozon::perception::LanePositionType::EGO_RIGHT:
      return LaneLinePosition::EGO_RIGHT;
    case hozon::perception::LanePositionType::ADJACENT_RIGHT:
      return LaneLinePosition::ADJACENT_RIGHT;
    case hozon::perception::LanePositionType::THIRD_RIGHT:
      return LaneLinePosition::THIRD_RIGHT;
    case hozon::perception::LanePositionType::FOURTH_RIGHT:
      return LaneLinePosition::FOURTH_RIGHT;
    default:
      return LaneLinePosition::OTHER;
  }
}

static hozon::perception::Color CvtLaneColor2TePb(LaneLineColor color) {
  switch (color) {
    case LaneLineColor::UNKNOWN:
      return hozon::perception::Color::UNKNOWN;
    case LaneLineColor::WHITE:
      return hozon::perception::Color::WHITE;
    case LaneLineColor::YELLOW:
      return hozon::perception::Color::YELLOW;
    case LaneLineColor::GREEN:
      return hozon::perception::Color::GREEN;
    case LaneLineColor::RED:
      return hozon::perception::Color::RED;
    case LaneLineColor::BLACK:
      return hozon::perception::Color::BLACK;
    default:
      return hozon::perception::Color::UNKNOWN;
  }
}

static hozon::mapping::Color CvtLaneColor2Pb(LaneLineColor color) {
  switch (color) {
    case LaneLineColor::UNKNOWN:
      return hozon::mapping::Color::UNKNOWN;
    case LaneLineColor::WHITE:
      return hozon::mapping::Color::WHITE;
    case LaneLineColor::YELLOW:
      return hozon::mapping::Color::YELLOW;
    case LaneLineColor::GREEN:
      return hozon::mapping::Color::GREEN;
    case LaneLineColor::RED:
      return hozon::mapping::Color::RED;
    case LaneLineColor::BLACK:
      return hozon::mapping::Color::BLACK;
    default:
      return hozon::mapping::Color::UNKNOWN;
  }
}

static LaneLineColor CvtPb2LaneColor(hozon::perception::Color color) {
  switch (color) {
    case hozon::perception::Color::UNKNOWN:
      return LaneLineColor::UNKNOWN;
    case hozon::perception::Color::WHITE:
      return LaneLineColor::WHITE;
    case hozon::perception::Color::YELLOW:
      return LaneLineColor::YELLOW;
    case hozon::perception::Color::GREEN:
      return LaneLineColor::GREEN;
    case hozon::perception::Color::RED:
      return LaneLineColor::RED;
    case hozon::perception::Color::BLACK:
      return LaneLineColor::BLACK;
    default:
      return LaneLineColor::UNKNOWN;
  }
}

static hozon::mapping::LaneType CvtLaneType2Pb(LaneLineType shape) {
  switch (shape) {
    case LaneLineType::SOLID_LINE:  // 单实线
      return hozon::mapping::LaneType::LaneType_SOLID;
    case LaneLineType::DASHED_LINE:  // 单虚线
      return hozon::mapping::LaneType::LaneType_DASHED;
    case LaneLineType::SHORT_DASHED_LINE:  // 短虚线
      return hozon::mapping::LaneType::LaneType_SHORT_DASHED;
    case LaneLineType::DOUBLE_SOLID_LINE:  // 双实线
      return hozon::mapping::LaneType::LaneType_DOUBLE_SOLID;
    case LaneLineType::DOUBLE_DASHED_LINE:  // 双虚线
      return hozon::mapping::LaneType::LaneType_DOUBLE_DASHED;
    case LaneLineType::LEFT_SOLID_RIGHT_DASHED:  // 左实右虚
      return hozon::mapping::LaneType::LaneType_LEFT_SOLID_RIGHT_DASHED;
    case LaneLineType::RIGHT_SOLID_LEFT_DASHED:  // 右实左虚
      return hozon::mapping::LaneType::LaneType_RIGHT_SOLID_LEFT_DASHED;
    case LaneLineType::OTHER:  // 其他
      return hozon::mapping::LaneType::LaneType_OTHER;
    case LaneLineType::FISHBONE:  // 鱼骨实线
      return hozon::mapping::LaneType::LaneType_FISHBONE_SOLID;
    case LaneLineType::FISHBONE_DASHED_LINE:  // 鱼骨虚线
      return hozon::mapping::LaneType::LaneType_FISHBONE_DASHED;
    case LaneLineType::UNKNOWN:  // 未知
      return hozon::mapping::LaneType::LaneType_UNKNOWN;
    default:
      return hozon::mapping::LaneType::LaneType_UNKNOWN;
  }
}

static hozon::perception::LaneType CvtLaneType2TePb(LaneLineType shape) {
  switch (shape) {
    case LaneLineType::SOLID_LINE:  // 单实线
      return hozon::perception::LaneType::SolidLine;
    case LaneLineType::DASHED_LINE:  // 单虚线
      return hozon::perception::LaneType::DashedLine;
    case LaneLineType::SHORT_DASHED_LINE:  // 短虚线
      return hozon::perception::LaneType::ShortDashedLine;
    case LaneLineType::DOUBLE_SOLID_LINE:  // 双实线
      return hozon::perception::LaneType::DoubleSolidLine;
    case LaneLineType::DOUBLE_DASHED_LINE:  // 双虚线
      return hozon::perception::LaneType::DoubleDashedLine;
    case LaneLineType::LEFT_SOLID_RIGHT_DASHED:  // 左实右虚
      return hozon::perception::LaneType::LeftSolidRightDashed;
    case LaneLineType::RIGHT_SOLID_LEFT_DASHED:  // 右实左虚
      return hozon::perception::LaneType::RightSolidLeftDashed;
    case LaneLineType::OTHER:  // 其他
      return hozon::perception::LaneType::Other;
    case LaneLineType::FISHBONE:  // 鱼骨线
      return hozon::perception::LaneType::FishBoneLine;
    case LaneLineType::FISHBONE_DASHED_LINE:  // 鱼骨虚线
      return hozon::perception::LaneType::FishBoneDashedLine;
    case LaneLineType::UNKNOWN:  // 未知
      return hozon::perception::LaneType::Unknown;
    default:
      return hozon::perception::LaneType::Unknown;
  }
}

static LaneLineSceneType CvtPb2LaneSceneType(
    hozon::perception::LaneInfo::LaneLineSceneType type) {
  switch (type) {
    case hozon::perception::LaneInfo::LaneLineSceneType::
        LaneInfo_LaneLineSceneType_NORMAL:
      return LaneLineSceneType::NORMAL;  // 单实线
    case hozon::perception::LaneInfo::LaneLineSceneType::
        LaneInfo_LaneLineSceneType_FORK:
      return LaneLineSceneType::FORK;  // 单虚线
    case hozon::perception::LaneInfo::LaneLineSceneType::
        LaneInfo_LaneLineSceneType_CONVERGE:
      return LaneLineSceneType::CONVERGE;  // 双实线
    default:
      return LaneLineSceneType::UNKNOWN;  // 未知
  }
}

static LaneLineType CvtPb2LaneType(hozon::perception::LaneType type) {
  switch (type) {
    case hozon::perception::LaneType::SolidLine:
      return LaneLineType::SOLID_LINE;  // 单实线
    case hozon::perception::LaneType::DashedLine:
      return LaneLineType::DASHED_LINE;  // 单虚线
    case hozon::perception::LaneType::DoubleSolidLine:
      return LaneLineType::DOUBLE_SOLID_LINE;  // 双实线
    case hozon::perception::LaneType::DoubleDashedLine:
      return LaneLineType::DOUBLE_DASHED_LINE;  // 双虚线
    case hozon::perception::LaneType::LeftSolidRightDashed:
      return LaneLineType::LEFT_SOLID_RIGHT_DASHED;  // 左实右虚
    case hozon::perception::LaneType::RightSolidLeftDashed:
      return LaneLineType::RIGHT_SOLID_LEFT_DASHED;  // 右实左虚
    case hozon::perception::LaneType::Other:
      return LaneLineType::OTHER;  // 其他
    case hozon::perception::LaneType::FishBoneLine:
      return LaneLineType::FISHBONE;
    case hozon::perception::LaneType::FishBoneDashedLine:
      return LaneLineType::FISHBONE_DASHED_LINE;
    case hozon::perception::LaneType::Unknown:
      return LaneLineType::UNKNOWN;  // 未知
    default:
      return LaneLineType::UNKNOWN;  // 未知
  }
}

bool DataMapping::CvtLaneLine2TePb(const LaneLinePtr& lane_msg,
                                   hozon::perception::LaneInfo* pb_lane) {
  if (nullptr == lane_msg || nullptr == pb_lane) {
    HLOG_ERROR << " msg  or pb_object is nullptr.";
    return false;
  }
  pb_lane->set_track_id(lane_msg->id);
  hozon::perception::LaneType send_type = CvtLaneType2TePb(lane_msg->type);
  pb_lane->set_lanetype(send_type);
  // hozon::perception::LanePositionType send_pos_type =
  //     CvtLanePosType2TePb(lane_msg->te_position);
  hozon::perception::LanePositionType send_pos_type =
      CvtLanePosType2TePb(lane_msg->position);
  pb_lane->set_lanepos(send_pos_type);
  pb_lane->set_is_stable(lane_msg->send_postlane);
  auto curve_lane = pb_lane->mutable_lane_param()->add_cubic_curve_set();
  if (lane_msg->vehicle_curve.coeffs.size() == 4) {
    // 车道线三次方程系数跟踪方案发送字段
    curve_lane->set_c0(lane_msg->vehicle_curve.coeffs[0]);
    curve_lane->set_c1(lane_msg->vehicle_curve.coeffs[1]);
    curve_lane->set_c2(lane_msg->vehicle_curve.coeffs[2]);
    curve_lane->set_c3(lane_msg->vehicle_curve.coeffs[3]);
    curve_lane->set_start_point_x(lane_msg->vehicle_curve.min);
    curve_lane->set_end_point_x(lane_msg->vehicle_curve.max);
  }
  // 车道线点跟踪方案发送字段
  for (auto& item_pt : lane_msg->vehicle_points) {
    if (item_pt.x() < 0) {
      continue;
    }
    auto pb_pt = pb_lane->add_points();
    pb_pt->set_x(item_pt.x());
    pb_pt->set_y(item_pt.y());
    pb_pt->set_z(item_pt.z());
  }

  hozon::perception::Color send_color = CvtLaneColor2TePb(lane_msg->color);
  pb_lane->set_color(send_color);
  // pb_lane->set_confidence(lane_msg->geo_confidence);
  pb_lane->set_confidence(1.0);
  return true;
}

bool DataMapping::CvtLaneLine2Pb(const LaneLinePtr& lane_msg,
                                 hozon::mapping::LaneLine* pb_lane) {
  if (nullptr == lane_msg || nullptr == pb_lane) {
    HLOG_ERROR << " msg  or pb_object is nullptr.";
    return false;
  }
  pb_lane->set_track_id(lane_msg->id);
  hozon::mapping::LaneType send_type = CvtLaneType2Pb(lane_msg->type);
  pb_lane->set_lanetype(send_type);
  hozon::mapping::LanePositionType send_pos_type =
      CvtLanePosType2Pb(lane_msg->position);

  // HLOG_INFO << "cam3 lane id:" << lane_msg->id
  //           << " lane pos:" << static_cast<int>(lane_msg->position)
  //           << " lane cvt pos:" << static_cast<int>(send_pos_type);
  pb_lane->set_lanepos(send_pos_type);
  auto curve_lane = pb_lane->mutable_lane_param()->add_cubic_curve_set();
  if (lane_msg->vehicle_curve.coeffs.size() == 4) {
    // 车道线三次方程系数跟踪方案发送字段
    curve_lane->set_c0(lane_msg->vehicle_curve.coeffs[0]);
    curve_lane->set_c1(lane_msg->vehicle_curve.coeffs[1]);
    curve_lane->set_c2(lane_msg->vehicle_curve.coeffs[2]);
    curve_lane->set_c3(lane_msg->vehicle_curve.coeffs[3]);
    curve_lane->set_start_point_x(lane_msg->vehicle_curve.min);
    curve_lane->set_end_point_x(lane_msg->vehicle_curve.max);
  }
  // 车道线点跟踪方案发送字段
  for (auto& item_pt : lane_msg->vehicle_points) {
    auto pb_pt = pb_lane->add_points();
    pb_pt->set_x(item_pt.x());
    pb_pt->set_y(item_pt.y());
    pb_pt->set_z(item_pt.z());
  }

  hozon::mapping::Color send_color = CvtLaneColor2Pb(lane_msg->color);
  pb_lane->set_color(send_color);
  pb_lane->set_confidence(1.0);
  // pb_lane->set_confidence(lane_msg->geo_confidence);
  pb_lane->set_lost_age(lane_msg->lost_age);
  HLOG_DEBUG << "DEBUG:"
             << "id, " << lane_msg->id << ", lost_age, " << lane_msg->lost_age
             << ", tracked_age: " << lane_msg->tracked_count
             << ", latest_tracked_age, "
             << std::to_string(lane_msg->latest_tracked_time);
  pb_lane->set_tracked_age(lane_msg->tracked_count);
  pb_lane->set_latest_tracked_time(lane_msg->latest_tracked_time);
  // 车道线稳定性误差
  pb_lane->mutable_stability_error()->CopyFrom(lane_msg->stability_error);
  return true;
}

bool DataMapping::CvtPb2LaneLineMeasurement(
    const hozon::perception::LaneInfo& laneinfo, LaneLinePtr laneptr) {
  laneptr->id = laneinfo.track_id();
  laneptr->type = CvtPb2LaneType(laneinfo.lanetype());
  laneptr->type_confidence = static_cast<float>(laneinfo.confidence());
  laneptr->color = CvtPb2LaneColor(laneinfo.color());
  laneptr->position = CvtPbLanePosType(laneinfo.lanepos());

  for (auto& item : laneinfo.points()) {
    if (std::isnan(item.x()) || std::isnan(item.y()) || std::isnan(item.z())) {
      return false;
    }
    Eigen::Vector3d llpt;
    llpt.x() = item.x();
    llpt.y() = item.y();
    llpt.z() = item.z();
    laneptr->vehicle_points.push_back(llpt);
  }

  return true;
}

bool DataMapping::CvtPb2CrossPointMeasurement(
    const hozon::perception::CrossPoint& crosspoint_info, CrossPointPtr cpptr) {
  cpptr->id = crosspoint_info.id();
  cpptr->confidence = static_cast<float>(crosspoint_info.confidence());
  cpptr->vehicle_point.x() = crosspoint_info.point_3d().x();
  cpptr->vehicle_point.y() = crosspoint_info.point_3d().y();
  cpptr->vehicle_point.z() = crosspoint_info.point_3d().z();
  cpptr->type = CvtPbCrossPointType(crosspoint_info.type());

  return true;
}

}  // namespace data_mapping
}  // namespace lm
}  // namespace mp
}  //  namespace hozon
