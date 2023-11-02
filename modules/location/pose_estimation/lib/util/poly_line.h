/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： poly_line.h
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <iostream>

#include "proto/perception/transport_element.pb.h"

namespace hozon {
namespace mp {
namespace loc {
using LaneLine = ::hozon::perception::LaneInfo;

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
