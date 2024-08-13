/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： proto2elemap.h
 *   author     ： hozon
 *   date       ： 2024.08
 ******************************************************************************/

#pragma once

#include <depend/proto/local_mapping/local_map.pb.h>

#include "map_fusion/fusion_common/element_map.h"

namespace hozon {
namespace mp {
namespace mf {
namespace em {

void SetLanePos(const hozon::mp::mf::em::Boundary& lane_line,
                hozon::mapping::LanePositionType* lanepostype);

void SetLaneColor(const hozon::mp::mf::em::Boundary& lane_line,
                  hozon::mapping::Color* lanecolor);

void SetLaneType(const hozon::mp::mf::em::Boundary& lane_line,
                 hozon::mapping::LaneType* lanetype);

}  // namespace em
}  // namespace mf
}  // namespace mp
}  // namespace hozon
