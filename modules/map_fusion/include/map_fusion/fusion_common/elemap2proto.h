/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： elemap2proto.h
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

void FillLanePos(hozon::mp::mf::em::Boundary* lane_line,
                 hozon::mapping::LanePositionType lanepostype);

void FillLaneColor(hozon::mp::mf::em::Boundary* lane_line,
                   hozon::mapping::Color lanecolor);

void FillLaneType(hozon::mp::mf::em::Boundary* lane_line,
                  hozon::mapping::LaneType lanetype);

}  // namespace em
}  // namespace mf
}  // namespace mp
}  // namespace hozon
