/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： perception.cc
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/location/pose_estimation/lib/perception/perception.h"

#include <vector>

namespace hozon {
namespace mp {
namespace loc {

Perception::Perception() {}
Perception::Perception(
    const ::adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray &lane_lines) {
  Set(lane_lines);
}

void Perception::Set(
    const ::adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray &lane_lines) {
  SetLaneLineList(lane_lines);
}

void Perception::SetLaneLineList(
    const ::adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray &lane_line) {
  auto p = std::make_shared<PerceptionLaneLineList>(lane_line);
  element_.emplace_back(p);
  HLOG_ERROR << "SetLaneLineList start_here element_.size = "
             << element_.size();
}

// void
// Perception<::adsfi_proto::hz_Adsfi::AlgLaneDetectionOut>::SetRoadEdgeList(
//     const ::adsfi_proto::hz_Adsfi::AlgLaneDetectionOut &lane_line) {
//   auto p =
//   std::make_shared<PerceptionRoadEdgeList<::adsfi_proto::hz_Adsfi::AlgLaneDetectionOut>>(lane_line);
//   element_.emplace_back(p);
// }

std::vector<PerceptionElement::Ptr> Perception::GetElement(int type) const {
  std::vector<PerceptionElement::Ptr> ret;
  for (auto elment : element_) {
    if (elment->type_ == type) {
      ret.emplace_back(elment);
    }
  }
  return ret;
}

}  // namespace loc
}  // namespace mp
}  // namespace hozon
