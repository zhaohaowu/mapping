/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： perception_lane_line.cc
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/location/pose_estimation/lib/perception/perception_lane_line.h"

namespace hozon {
namespace mp {
namespace loc {

PerceptionLaneLine::PerceptionLaneLine() {}
PerceptionLaneLine::PerceptionLaneLine(
    const ::adsfi_proto::hz_Adsfi::AlgLaneDetectionOut &lane_line) {
  curve_vehicle_coord_ = PolyLine<LaneLine>(lane_line);
}

int PerceptionLaneLine::Id() { return curve_vehicle_coord_.id_; }

bool PerceptionLaneLine::IsIn(const float x) {
  const auto &poly = curve_vehicle_coord_;
  double x_end = poly.min_ + 40;
  bool inx = x >= poly.min_ && x <= x_end && x < poly.max_;
  if (Probality() > mm_params.lane_confidence_thre && inx)
    return true;
  else
    return false;
}

float PerceptionLaneLine::Y(const float x) {
  const auto &poly = curve_vehicle_coord_;
  return (poly.c0_ + poly.c1_ * x + poly.c2_ * x * x + poly.c3_ * x * x * x);
}

float PerceptionLaneLine::Theta(const float &x) {
  const auto &poly = curve_vehicle_coord_;
  return (poly.c1_ + 2.0 * poly.c2_ * x + 3 * poly.c3_ * x * x);
}

float PerceptionLaneLine::Min(void) {
  const auto &poly = curve_vehicle_coord_;
  return poly.min_;
}

float PerceptionLaneLine::Max(void) {
  const auto &poly = curve_vehicle_coord_;
  return poly.max_;
}

float PerceptionLaneLine::Probality() {
  const auto &poly = curve_vehicle_coord_;
  return poly.confidence_;
}

void PerceptionLaneLine::Print() {
  HLOG_INFO << " curve_vehicle_coord_.min_:" << curve_vehicle_coord_.min_;
  HLOG_INFO << " curve_vehicle_coord_.max_:" << curve_vehicle_coord_.max_;
  HLOG_INFO << " curve_vehicle_coord_.c0_:" << curve_vehicle_coord_.c0_;
  HLOG_INFO << " curve_vehicle_coord_.c1_:" << curve_vehicle_coord_.c1_;
  HLOG_INFO << " curve_vehicle_coord_.c2_:" << curve_vehicle_coord_.c2_;
  HLOG_INFO << " curve_vehicle_coord_.c3_:" << curve_vehicle_coord_.c3_;
  HLOG_INFO << " curve_vehicle_coord_.id_:" << curve_vehicle_coord_.id_;
  HLOG_INFO << " curve_vehicle_coord_.confidence_:"
            << curve_vehicle_coord_.confidence_;
}

PerceptionLaneLineList::PerceptionLaneLineList(
    const ::adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray &lanes) {
  this->type_ = PERCEPTYION_LANE_BOUNDARY_LINE;
  for (auto i = 0; i < lanes.lane_detection_front_out_size(); ++i) {
    for (auto j = 0;
         j < lanes.lane_detection_front_out(i).lane_detection_out_size(); ++j) {
      auto line = lanes.lane_detection_front_out(i).lane_detection_out(j);
      auto new_line = std::make_shared<PerceptionLaneLine>(line);
      lane_line_list_.push_back(new_line);
    }
  }
  HLOG_INFO << "lane_line_list_.size() = " << lane_line_list_.size();
}

PerceptionLaneLineList::PerceptionLaneLineList() {}

void PerceptionLaneLineList::Print(void) {
  for (const auto line : lane_line_list_) {
    line->Print();
  }
}

}  // namespace loc
}  // namespace mp
}  // namespace hozon
