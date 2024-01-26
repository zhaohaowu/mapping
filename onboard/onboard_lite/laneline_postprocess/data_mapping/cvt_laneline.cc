/********************************************************
 * Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 * Author: Hozon
 *******************************************************/

#include "onboard/onboard_lite/laneline_postprocess/data_mapping/data_mapping.h"
#include "perception-base/base/laneline/base_laneline.h"
#include "proto/perception/transport_element.pb.h"
namespace hozon {
namespace mp {
namespace common_onboard {

static bool CvtLaneToPb(const base::LaneLinePtr &lane_msg, LaneInfo *pb_object);

static CrossType CvtCrossPointType2Pb(base::CrossType type) {
  switch (type) {
    case base::CrossType::SPLIT:
      return CrossType::SPLIT;
    case base::CrossType::MERGE:
      return CrossType::SPLIT;
    default:
      return CrossType::CROSSTYPE_UNKNOWN;
  }
}

static LanePositionType CvtLanePosType(base::LaneLinePosition type) {
  switch (type) {
    case base::LaneLinePosition::OTHER:
      return LanePositionType::OTHER;
    case base::LaneLinePosition::FOURTH_LEFT:
      return LanePositionType::FOURTH_LEFT;
    case base::LaneLinePosition::THIRD_LEFT:
      return LanePositionType::THIRD_LEFT;
    case base::LaneLinePosition::ADJACENT_LEFT:
      return LanePositionType::ADJACENT_LEFT;
    case base::LaneLinePosition::EGO_LEFT:
      return LanePositionType::EGO_LEFT;
    case base::LaneLinePosition::EGO_RIGHT:
      return LanePositionType::EGO_RIGHT;
    case base::LaneLinePosition::ADJACENT_RIGHT:
      return LanePositionType::ADJACENT_RIGHT;
    case base::LaneLinePosition::THIRD_RIGHT:
      return LanePositionType::THIRD_RIGHT;
    case base::LaneLinePosition::FOURTH_RIGHT:
      return LanePositionType::FOURTH_RIGHT;
    default:
      return LanePositionType::OTHER;
  }
}

static base::LaneLinePosition CvtPbLanePosType(LanePositionType type) {
  switch (type) {
    case LanePositionType::OTHER:
      return base::LaneLinePosition::OTHER;
    case LanePositionType::FOURTH_LEFT:
      return base::LaneLinePosition::FOURTH_LEFT;
    case LanePositionType::THIRD_LEFT:
      return base::LaneLinePosition::THIRD_LEFT;
    case LanePositionType::ADJACENT_LEFT:
      return base::LaneLinePosition::ADJACENT_LEFT;
    case LanePositionType::EGO_LEFT:
      return base::LaneLinePosition::EGO_LEFT;
    case LanePositionType::EGO_RIGHT:
      return base::LaneLinePosition::EGO_RIGHT;
    case LanePositionType::ADJACENT_RIGHT:
      return base::LaneLinePosition::ADJACENT_RIGHT;
    case LanePositionType::THIRD_RIGHT:
      return base::LaneLinePosition::THIRD_RIGHT;
    case LanePositionType::FOURTH_RIGHT:
      return base::LaneLinePosition::FOURTH_RIGHT;
    default:
      return base::LaneLinePosition::OTHER;
  }
}

static Color CvtLaneColor2Pb(base::LaneLineColor color) {
  switch (color) {
    case base::LaneLineColor::UNKNOWN:
      return Color::UNKNOWN;
    case base::LaneLineColor::WHITE:
      return Color::WHITE;
    case base::LaneLineColor::YELLOW:
      return Color::YELLOW;
    case base::LaneLineColor::GREEN:
      return Color::GREEN;
    case base::LaneLineColor::RED:
      return Color::RED;
    case base::LaneLineColor::BLACK:
      return Color::BLACK;
    default:
      return Color::UNKNOWN;
  }
}

static base::LaneLineColor CvtPb2LaneColor(Color color) {
  switch (color) {
    case Color::UNKNOWN:
      return base::LaneLineColor::UNKNOWN;
    case Color::WHITE:
      return base::LaneLineColor::WHITE;
    case Color::YELLOW:
      return base::LaneLineColor::YELLOW;
    case Color::GREEN:
      return base::LaneLineColor::GREEN;
    case Color::RED:
      return base::LaneLineColor::RED;
    case Color::BLACK:
      return base::LaneLineColor::BLACK;
    default:
      return base::LaneLineColor::UNKNOWN;
  }
}

static LaneType CvtLaneType2Pb(base::LaneLineType shape) {
  switch (shape) {
    case base::LaneLineType::SolidLine:  // 单实线
      return LaneType::SolidLine;
    case base::LaneLineType::DashedLine:  // 单虚线
      return LaneType::DashedLine;
    case base::LaneLineType::DoubleSolidLine:  // 双实线
      return LaneType::DoubleSolidLine;
    case base::LaneLineType::DoubleDashedLine:  // 双虚线
      return LaneType::DoubleDashedLine;
    case base::LaneLineType::LeftSolidRightDashed:  // 左实右虚
      return LaneType::LeftSolidRightDashed;
    case base::LaneLineType::RightSolidLeftDashed:  // 右实左虚
      return LaneType::RightSolidLeftDashed;
    case base::LaneLineType::Other:  // 其他
      return LaneType::Other;
    case base::LaneLineType::FishBone:
      return LaneType::FishBoneLine;
    case base::LaneLineType::Unknown:  // 未知
      return LaneType::Unknown;
    default:
      return LaneType::Unknown;
  }
}

static LaneInfo::LaneLineSceneType CvtLaneSceneType2Pb(
    base::LaneLineSceneType scene_type) {
  switch (scene_type) {
    case base::LaneLineSceneType::NORMAL:  // 正常场景
      return LaneInfo::LaneLineSceneType::LaneInfo_LaneLineSceneType_NORMAL;
    case base::LaneLineSceneType::FORK:  // 双实线
      return LaneInfo::LaneLineSceneType::LaneInfo_LaneLineSceneType_FORK;
    case base::LaneLineSceneType::CONVERGE:  // 双虚线
      return LaneInfo::LaneLineSceneType::LaneInfo_LaneLineSceneType_CONVERGE;
    default:
      return LaneInfo::LaneLineSceneType::LaneInfo_LaneLineSceneType_UNKNOWN;
  }
}

static base::LaneLineSceneType CvtPb2LaneSceneType(
    LaneInfo::LaneLineSceneType type) {
  switch (type) {
    case LaneInfo::LaneLineSceneType::LaneInfo_LaneLineSceneType_NORMAL:
      return base::LaneLineSceneType::NORMAL;  // 单实线
    case LaneInfo::LaneLineSceneType::LaneInfo_LaneLineSceneType_FORK:
      return base::LaneLineSceneType::FORK;  // 单虚线
    case LaneInfo::LaneLineSceneType::LaneInfo_LaneLineSceneType_CONVERGE:
      return base::LaneLineSceneType::CONVERGE;  // 双实线
    default:
      return base::LaneLineSceneType::UNKNOWN;  // 未知
  }
}

static base::LaneLineType CvtPb2LaneType(LaneType type) {
  switch (type) {
    case LaneType::SolidLine:
      return base::LaneLineType::SolidLine;  // 单实线
    case LaneType::DashedLine:
      return base::LaneLineType::DashedLine;  // 单虚线
    case LaneType::DoubleSolidLine:
      return base::LaneLineType::DoubleSolidLine;  // 双实线
    case LaneType::DoubleDashedLine:
      return base::LaneLineType::DoubleDashedLine;  // 双虚线
    case LaneType::LeftSolidRightDashed:
      return base::LaneLineType::LeftSolidRightDashed;  // 左实右虚
    case LaneType::RightSolidLeftDashed:
      return base::LaneLineType::RightSolidLeftDashed;  // 右实左虚
    case LaneType::Other:
      return base::LaneLineType::Other;  // 其他
    case LaneType::FishBoneLine:
      return base::LaneLineType::FishBone;
    case LaneType::Unknown:
      return base::LaneLineType::Unknown;  // 未知
    default:
      return base::LaneLineType::Unknown;  // 未知
  }
}

bool DataMapping::CvtMultiLanesToPb(
    const std::vector<base::LaneLinePtr> &lanes_msg,
    NetaTransportElementPtr pb_objects) {
  if (lanes_msg.size() == 0) {
    HLOG_DEBUG << "lines msg size is 0";
    return true;
  }
  if (nullptr == pb_objects) {
    HLOG_ERROR << "pb_objects is nullptr.";
    return false;
  }
  // header
  auto pb_header = pb_objects->mutable_header();
  static uint32_t seq = 1;
  pb_header->set_seq(seq++);
  // Todo by zhangwenhai
  // pb_header->set_publish_stamp(common::TimeUtil::GetCurrentTime());

  uint32_t lanes_size = lanes_msg.size();
  for (size_t i = 0; i < lanes_size; ++i) {
    auto pb_lane = pb_objects->add_lane();
    if (!CvtLaneToPb(lanes_msg[i], pb_lane)) {
      HLOG_ERROR << "cvt lane to proto struct failed.";
      return false;
    }
  }
  return true;
}

bool DataMapping::CvtLaneToPb(const base::LaneLinePtr &lane_msg,
                              LaneInfo *pb_object) {
  if (nullptr == lane_msg || nullptr == pb_object) {
    HLOG_ERROR << "lane msg  or pb_object is nullptr.";
    return false;
  }
  pb_object->set_track_id(lane_msg->id);
  LaneType send_type = CvtLaneType2Pb(lane_msg->type);
  pb_object->set_lanetype(send_type);
  LanePositionType send_pos_type = CvtLanePosType(lane_msg->position);
  pb_object->set_lanepos(send_pos_type);
  LaneInfo::LaneLineSceneType scene_type =
      CvtLaneSceneType2Pb(lane_msg->scene_type);
  pb_object->set_scene_type(scene_type);
  auto curve_lane = pb_object->mutable_lane_param()->add_cubic_curve_set();
  if (lane_msg->vehicle_curve.coeffs.size() == 4) {
    // 车道线三次方程系数跟踪方案发送字段
    curve_lane->set_c0(lane_msg->vehicle_curve.coeffs[0]);
    curve_lane->set_c1(lane_msg->vehicle_curve.coeffs[1]);
    curve_lane->set_c2(lane_msg->vehicle_curve.coeffs[2]);
    curve_lane->set_c3(lane_msg->vehicle_curve.coeffs[3]);
    curve_lane->set_start_point_x(lane_msg->vehicle_curve.min);
    curve_lane->set_end_point_x(lane_msg->vehicle_curve.max);
  }
  if (lane_msg->point_set.size() != 0) {
    // 车道线点跟踪方案发送字段
    for (auto &item_pt : lane_msg->point_set) {
      auto pb_pt = pb_object->add_points();
      pb_pt->set_x(item_pt.vehicle_point.x);
      pb_pt->set_y(item_pt.vehicle_point.y);
      pb_pt->set_z(item_pt.vehicle_point.z);
    }
  }

  Color send_color = CvtLaneColor2Pb(lane_msg->color);
  pb_object->set_color(send_color);
  pb_object->set_confidence(lane_msg->geo_confidence);
  return true;
}

bool DataMapping::CvtLaneMeasurementToPb(
    const base::LaneLinesMeasurementPtr &laneline_measures,
    hozon::perception::TransportElement *transport_element) {
  if (nullptr == laneline_measures) return true;

  const auto &laneline_measure = laneline_measures->lanelines;
  for (auto &item : laneline_measure) {
    auto pb_lane = transport_element->add_lane();

    pb_lane->set_track_id(item->id);

    pb_lane->set_lanetype(CvtLaneType2Pb(item->type));
#if 0
    pb_lane->set_scene_type(CvtLaneSceneType2Pb(item->split));
#endif

    pb_lane->set_scene_type(CvtLaneSceneType2Pb(item->split));

    pb_lane->set_confidence(item->type_confidence);

    pb_lane->set_color(CvtLaneColor2Pb(item->color));

    for (auto &item_pt : item->point_set) {
      auto pb_pt = pb_lane->add_points();
      pb_pt->set_x(item_pt.vehicle_point.x);
      pb_pt->set_y(item_pt.vehicle_point.y);
      pb_pt->set_z(item_pt.vehicle_point.z);
    }
  }

  const auto &crosspoint_measure = laneline_measures->crosspoints;
  for (auto &item : crosspoint_measure) {
    auto pb_crosspoint = transport_element->add_cross_points();
    pb_crosspoint->set_id(item->id);
    pb_crosspoint->set_type(CvtCrossPointType2Pb(item->type));
    pb_crosspoint->set_confidence(item->confidence);
    auto point = pb_crosspoint->mutable_point_3d();
    point->set_x(item->point.vehicle_point.x);
    point->set_y(item->point.vehicle_point.y);
    point->set_z(item->point.vehicle_point.z);
  }

  return true;
}

bool DataMapping::CvtPb2LaneLineMeasurement(
    const hozon::perception::LaneInfo &laneinfo,
    base::LaneLineMeasurementPtr laneptr) {
  laneptr->id = laneinfo.track_id();
  laneptr->type = CvtPb2LaneType(laneinfo.lanetype());
  laneptr->type_confidence = laneinfo.confidence();
  laneptr->color = CvtPb2LaneColor(laneinfo.color());
  laneptr->position = CvtPbLanePosType(laneinfo.lanepos());

  for (auto &item : laneinfo.points()) {
    base::LaneLinePoint llpt;
    llpt.vehicle_point.x = item.x();
    llpt.vehicle_point.y = item.y();
    llpt.vehicle_point.z = item.z();
    laneptr->point_set.push_back(std::move(llpt));
  }

  return true;
}

}  //  namespace common_onboard
}  // namespace mp
}  //  namespace hozon
