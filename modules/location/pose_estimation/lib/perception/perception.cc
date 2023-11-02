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
    const ::hozon::perception::TransportElement &transport_element) {
  Set(transport_element);
}

void Perception::Set(
    const ::hozon::perception::TransportElement &transport_element) {
  SetLaneLineList(transport_element);
}

void Perception::SetLaneLineList(
    const ::hozon::perception::TransportElement &transport_element) {
  auto p = std::make_shared<PerceptionLaneLineList>(transport_element);
  element_.emplace_back(p);
}

// void Perception<::adsfi_proto::hz_Adsfi::AlgLaneDetectionOut>::SetRoadEdgeList(
//     const ::adsfi_proto::hz_Adsfi::AlgLaneDetectionOut &lane_line) {
//   auto p = std::make_shared<
//       PerceptionRoadEdgeList<::adsfi_proto::hz_Adsfi::AlgLaneDetectionOut>>(
//       lane_line);
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
