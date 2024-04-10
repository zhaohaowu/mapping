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

static bool CvtLaneToPb(const perception_base::LaneLinePtr& lane_msg,
                        hozon::perception::LaneInfo* pb_object);

static hozon::perception::CrossType CvtCrossPointType2Pb(
    perception_base::CrossType type) {
  switch (type) {
    case perception_base::CrossType::SPLIT:
      return hozon::perception::CrossType::SPLIT;
    case perception_base::CrossType::MERGE:
      return hozon::perception::CrossType::SPLIT;
    default:
      return hozon::perception::CrossType::CROSSTYPE_UNKNOWN;
  }
}

static hozon::perception::LanePositionType CvtLanePosType(
    perception_base::LaneLinePosition type) {
  switch (type) {
    case perception_base::LaneLinePosition::OTHER:
      return hozon::perception::LanePositionType::OTHER;
    case perception_base::LaneLinePosition::FOURTH_LEFT:
      return hozon::perception::LanePositionType::FOURTH_LEFT;
    case perception_base::LaneLinePosition::THIRD_LEFT:
      return hozon::perception::LanePositionType::THIRD_LEFT;
    case perception_base::LaneLinePosition::ADJACENT_LEFT:
      return hozon::perception::LanePositionType::ADJACENT_LEFT;
    case perception_base::LaneLinePosition::EGO_LEFT:
      return hozon::perception::LanePositionType::EGO_LEFT;
    case perception_base::LaneLinePosition::EGO_RIGHT:
      return hozon::perception::LanePositionType::EGO_RIGHT;
    case perception_base::LaneLinePosition::ADJACENT_RIGHT:
      return hozon::perception::LanePositionType::ADJACENT_RIGHT;
    case perception_base::LaneLinePosition::THIRD_RIGHT:
      return hozon::perception::LanePositionType::THIRD_RIGHT;
    case perception_base::LaneLinePosition::FOURTH_RIGHT:
      return hozon::perception::LanePositionType::FOURTH_RIGHT;
    default:
      return hozon::perception::LanePositionType::OTHER;
  }
}

static perception_base::LaneLinePosition CvtPbLanePosType(
    hozon::perception::LanePositionType type) {
  switch (type) {
    case hozon::perception::LanePositionType::OTHER:
      return perception_base::LaneLinePosition::OTHER;
    case hozon::perception::LanePositionType::FOURTH_LEFT:
      return perception_base::LaneLinePosition::FOURTH_LEFT;
    case hozon::perception::LanePositionType::THIRD_LEFT:
      return perception_base::LaneLinePosition::THIRD_LEFT;
    case hozon::perception::LanePositionType::ADJACENT_LEFT:
      return perception_base::LaneLinePosition::ADJACENT_LEFT;
    case hozon::perception::LanePositionType::EGO_LEFT:
      return perception_base::LaneLinePosition::EGO_LEFT;
    case hozon::perception::LanePositionType::EGO_RIGHT:
      return perception_base::LaneLinePosition::EGO_RIGHT;
    case hozon::perception::LanePositionType::ADJACENT_RIGHT:
      return perception_base::LaneLinePosition::ADJACENT_RIGHT;
    case hozon::perception::LanePositionType::THIRD_RIGHT:
      return perception_base::LaneLinePosition::THIRD_RIGHT;
    case hozon::perception::LanePositionType::FOURTH_RIGHT:
      return perception_base::LaneLinePosition::FOURTH_RIGHT;
    default:
      return perception_base::LaneLinePosition::OTHER;
  }
}

static hozon::perception::Color CvtLaneColor2Pb(
    perception_base::LaneLineColor color) {
  switch (color) {
    case perception_base::LaneLineColor::UNKNOWN:
      return hozon::perception::Color::UNKNOWN;
    case perception_base::LaneLineColor::WHITE:
      return hozon::perception::Color::WHITE;
    case perception_base::LaneLineColor::YELLOW:
      return hozon::perception::Color::YELLOW;
    case perception_base::LaneLineColor::GREEN:
      return hozon::perception::Color::GREEN;
    case perception_base::LaneLineColor::RED:
      return hozon::perception::Color::RED;
    case perception_base::LaneLineColor::BLACK:
      return hozon::perception::Color::BLACK;
    default:
      return hozon::perception::Color::UNKNOWN;
  }
}

static perception_base::LaneLineColor CvtPb2LaneColor(
    hozon::perception::Color color) {
  switch (color) {
    case hozon::perception::Color::UNKNOWN:
      return perception_base::LaneLineColor::UNKNOWN;
    case hozon::perception::Color::WHITE:
      return perception_base::LaneLineColor::WHITE;
    case hozon::perception::Color::YELLOW:
      return perception_base::LaneLineColor::YELLOW;
    case hozon::perception::Color::GREEN:
      return perception_base::LaneLineColor::GREEN;
    case hozon::perception::Color::RED:
      return perception_base::LaneLineColor::RED;
    case hozon::perception::Color::BLACK:
      return perception_base::LaneLineColor::BLACK;
    default:
      return perception_base::LaneLineColor::UNKNOWN;
  }
}

static hozon::perception::LaneType CvtLaneType2Pb(
    perception_base::LaneLineType shape) {
  switch (shape) {
    case perception_base::LaneLineType::SolidLine:  // 单实线
      return hozon::perception::LaneType::SolidLine;
    case perception_base::LaneLineType::DashedLine:  // 单虚线
      return hozon::perception::LaneType::DashedLine;
    case perception_base::LaneLineType::DoubleSolidLine:  // 双实线
      return hozon::perception::LaneType::DoubleSolidLine;
    case perception_base::LaneLineType::DoubleDashedLine:  // 双虚线
      return hozon::perception::LaneType::DoubleDashedLine;
    case perception_base::LaneLineType::LeftSolidRightDashed:  // 左实右虚
      return hozon::perception::LaneType::LeftSolidRightDashed;
    case perception_base::LaneLineType::RightSolidLeftDashed:  // 右实左虚
      return hozon::perception::LaneType::RightSolidLeftDashed;
    case perception_base::LaneLineType::Other:  // 其他
      return hozon::perception::LaneType::Other;
    case perception_base::LaneLineType::FishBone:
      return hozon::perception::LaneType::FishBoneLine;
    case perception_base::LaneLineType::Unknown:  // 未知
      return hozon::perception::LaneType::Unknown;
    default:
      return hozon::perception::LaneType::Unknown;
  }
}

static hozon::perception::LaneInfo::LaneLineSceneType CvtLaneSceneType2Pb(
    perception_base::LaneLineSceneType scene_type) {
  switch (scene_type) {
    case perception_base::LaneLineSceneType::NORMAL:  // 正常场景
      return hozon::perception::LaneInfo::LaneLineSceneType::
          LaneInfo_LaneLineSceneType_NORMAL;
    case perception_base::LaneLineSceneType::FORK:  // 双实线
      return hozon::perception::LaneInfo::LaneLineSceneType::
          LaneInfo_LaneLineSceneType_FORK;
    case perception_base::LaneLineSceneType::CONVERGE:  // 双虚线
      return hozon::perception::LaneInfo::LaneLineSceneType::
          LaneInfo_LaneLineSceneType_CONVERGE;
    default:
      return hozon::perception::LaneInfo::LaneLineSceneType::
          LaneInfo_LaneLineSceneType_UNKNOWN;
  }
}

static perception_base::LaneLineSceneType CvtPb2LaneSceneType(
    hozon::perception::LaneInfo::LaneLineSceneType type) {
  switch (type) {
    case hozon::perception::LaneInfo::LaneLineSceneType::
        LaneInfo_LaneLineSceneType_NORMAL:
      return perception_base::LaneLineSceneType::NORMAL;  // 单实线
    case hozon::perception::LaneInfo::LaneLineSceneType::
        LaneInfo_LaneLineSceneType_FORK:
      return perception_base::LaneLineSceneType::FORK;  // 单虚线
    case hozon::perception::LaneInfo::LaneLineSceneType::
        LaneInfo_LaneLineSceneType_CONVERGE:
      return perception_base::LaneLineSceneType::CONVERGE;  // 双实线
    default:
      return perception_base::LaneLineSceneType::UNKNOWN;  // 未知
  }
}

static perception_base::LaneLineType CvtPb2LaneType(
    hozon::perception::LaneType type) {
  switch (type) {
    case hozon::perception::LaneType::SolidLine:
      return perception_base::LaneLineType::SolidLine;  // 单实线
    case hozon::perception::LaneType::DashedLine:
      return perception_base::LaneLineType::DashedLine;  // 单虚线
    case hozon::perception::LaneType::DoubleSolidLine:
      return perception_base::LaneLineType::DoubleSolidLine;  // 双实线
    case hozon::perception::LaneType::DoubleDashedLine:
      return perception_base::LaneLineType::DoubleDashedLine;  // 双虚线
    case hozon::perception::LaneType::LeftSolidRightDashed:
      return perception_base::LaneLineType::LeftSolidRightDashed;  // 左实右虚
    case hozon::perception::LaneType::RightSolidLeftDashed:
      return perception_base::LaneLineType::RightSolidLeftDashed;  // 右实左虚
    case hozon::perception::LaneType::Other:
      return perception_base::LaneLineType::Other;  // 其他
    case hozon::perception::LaneType::FishBoneLine:
      return perception_base::LaneLineType::FishBone;
    case hozon::perception::LaneType::Unknown:
      return perception_base::LaneLineType::Unknown;  // 未知
    default:
      return perception_base::LaneLineType::Unknown;  // 未知
  }
}

bool DataMapping::CvtMultiLanesToPb(
    const std::vector<perception_base::LaneLinePtr>& lanes_msg,
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
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
      tp = std::chrono::time_point_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now());
  pb_header->set_publish_stamp(tp.time_since_epoch().count() * 1.0e-9);
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

bool DataMapping::CvtLaneToPb(const perception_base::LaneLinePtr& lane_msg,
                              hozon::perception::LaneInfo* pb_object) {
  if (nullptr == lane_msg || nullptr == pb_object) {
    HLOG_ERROR << "lane msg  or pb_object is nullptr.";
    return false;
  }
  pb_object->set_track_id(lane_msg->id);
  hozon::perception::LaneType send_type = CvtLaneType2Pb(lane_msg->type);
  pb_object->set_lanetype(send_type);
  hozon::perception::LanePositionType send_pos_type =
      CvtLanePosType(lane_msg->position);
  pb_object->set_lanepos(send_pos_type);
  hozon::perception::LaneInfo::LaneLineSceneType scene_type =
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
    for (auto& item_pt : lane_msg->point_set) {
      auto pb_pt = pb_object->add_points();
      pb_pt->set_x(item_pt.vehicle_point.x);
      pb_pt->set_y(item_pt.vehicle_point.y);
      pb_pt->set_z(item_pt.vehicle_point.z);
    }
  }

  hozon::perception::Color send_color = CvtLaneColor2Pb(lane_msg->color);
  pb_object->set_color(send_color);
  pb_object->set_confidence(lane_msg->geo_confidence);
  return true;
}

bool DataMapping::CvtLaneMeasurementToPb(
    const perception_base::LaneLinesMeasurementPtr& laneline_measures,
    hozon::perception::TransportElement* transport_element) {
  if (nullptr == laneline_measures) return true;

  const auto& laneline_measure = laneline_measures->lanelines;
  for (auto& item : laneline_measure) {
    auto pb_lane = transport_element->add_lane();

    pb_lane->set_track_id(item->id);

    pb_lane->set_lanetype(CvtLaneType2Pb(item->type));
#if 0
    pb_lane->set_scene_type(CvtLaneSceneType2Pb(item->split));
#endif

    pb_lane->set_scene_type(CvtLaneSceneType2Pb(item->split));

    pb_lane->set_confidence(item->type_confidence);

    pb_lane->set_color(CvtLaneColor2Pb(item->color));

    for (auto& item_pt : item->point_set) {
      auto pb_pt = pb_lane->add_points();
      pb_pt->set_x(item_pt.vehicle_point.x);
      pb_pt->set_y(item_pt.vehicle_point.y);
      pb_pt->set_z(item_pt.vehicle_point.z);
    }
  }

  const auto& crosspoint_measure = laneline_measures->crosspoints;
  for (auto& item : crosspoint_measure) {
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
    const hozon::perception::LaneInfo& laneinfo,
    perception_base::LaneLineMeasurementPtr laneptr) {
  laneptr->id = laneinfo.track_id();
  laneptr->type = CvtPb2LaneType(laneinfo.lanetype());
  laneptr->type_confidence = laneinfo.confidence();
  laneptr->color = CvtPb2LaneColor(laneinfo.color());
  laneptr->position = CvtPbLanePosType(laneinfo.lanepos());

  for (auto& item : laneinfo.points()) {
    perception_base::LaneLinePoint llpt;
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
