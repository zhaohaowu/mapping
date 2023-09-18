/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： poly_line.h
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <interface/adsfi_proto/perception/lanes.pb.h>

#include <iostream>

namespace hozon {
namespace mp {
namespace loc {
using LaneLine = ::adsfi_proto::hz_Adsfi::AlgLaneDetectionOut;
// using RoadEdge = ::adsfi_proto::hz_Adsfi::AlgLaneDetectionOut::RoadEdge;

template <typename T>
class PolyLine {
 public:
  PolyLine() {}
  explicit PolyLine(T t);
  int id_;
  float min_;
  float max_;
  float c0_;
  float c1_;
  float c2_;
  float c3_;
  float confidence_;
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
