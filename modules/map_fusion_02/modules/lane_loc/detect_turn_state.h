
/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： detect_turn_state.h
 *   author     ： zhaohaowu
 *   date       ： 2024.07
 ******************************************************************************/
#pragma once
#include <memory>

#include "modules/map_fusion_02/modules/lane_loc/base_lane_loc.h"
namespace hozon {
namespace mp {
namespace mf {
namespace lane_loc {

class DetectTurnState {
 public:
  std::shared_ptr<TurnState> Run(const Groups& groups);
};

}  // namespace lane_loc
}  // namespace mf
}  // namespace mp
}  // namespace hozon
