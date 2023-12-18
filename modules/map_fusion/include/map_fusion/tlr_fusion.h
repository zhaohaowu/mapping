/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： tlr_fusion.h
 *   author     ： taoshaoyuan
 *   date       ： 2023.12
 ******************************************************************************/

#pragma once

#include <depend/proto/map/junction_passable.pb.h>
#include <depend/proto/perception/transport_element.pb.h>

#include <memory>

namespace hozon {
namespace mp {
namespace mf {

class TlrFusion {
 public:
  TlrFusion() = default;
  ~TlrFusion() = default;
  int Proc(const std::shared_ptr<hozon::perception::TrafficLightDetection>& tlr,
           hozon::hdmap::JunctionPassable* junc_passable);
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
