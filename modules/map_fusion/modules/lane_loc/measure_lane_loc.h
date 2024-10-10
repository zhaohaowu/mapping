/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： measure_lane_loc.h
 *   author     ： zhaohaowu
 *   date       ： 2024.07
 ******************************************************************************/
#pragma once
#include <memory>
#include <string>
#include <vector>

#include "modules/map_fusion/modules/lane_loc/base_lane_loc.h"

namespace hozon {
namespace mp {
namespace mf {
namespace lane_loc {

class MeasureLaneLoc {
 public:
  std::shared_ptr<std::vector<double>> Run(
      double* measure_weight, int* lane_index, std::string* road_edge_state,
      int lane_num, const Section& map_section, const Section& cur_section);
};

}  // namespace lane_loc
}  // namespace mf
}  // namespace mp
}  // namespace hozon
