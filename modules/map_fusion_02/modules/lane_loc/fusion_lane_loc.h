/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： fusion_lane_loc.h
 *   author     ： zhaohaowu
 *   date       ： 2024.07
 ******************************************************************************/
#pragma once
#include <memory>
#include <vector>

#include "modules/map_fusion_02/modules/lane_loc/base_lane_loc.h"
namespace hozon {
namespace mp {
namespace mf {
namespace lane_loc {

class FusionLaneLoc {
 public:
  std::shared_ptr<std::vector<double>> Run(
      int* fusion_lane_index, std::vector<double>* p_predict,
      double measure_weight,
      const std::shared_ptr<const std::vector<double>>& p_measure_ptr,
      const TurnState& turn_state);
};

}  // namespace lane_loc
}  // namespace mf
}  // namespace mp
}  // namespace hozon
