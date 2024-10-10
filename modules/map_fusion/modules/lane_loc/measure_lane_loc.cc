/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： measure_lane_loc.cc
 *   author     ： zhaohaowu
 *   date       ： 2024.07
 ******************************************************************************/
#include <modules/map_fusion/modules/lane_loc/measure_lane_loc.h>

#include <memory>
#include <vector>

#include "modules/map_fusion/modules/lane_loc/base_lane_loc.h"

namespace hozon {
namespace mp {
namespace mf {
namespace lane_loc {

std::shared_ptr<std::vector<double>> MeasureLaneLoc::Run(
    double* measure_weight, int* lane_index, std::string* road_edge_state,
    int lane_num, const Section& map_section, const Section& cur_section) {
  // 找到车辆当前车道
  Section::Lane cur_lane = cur_section.ego_lane;

  // 找到车辆最左侧车道
  Section::Lane leftest_lane = cur_section.leftest_lane;

  // 找到车辆最右侧车道
  Section::Lane rightest_lane = cur_section.rightest_lane;

  bool has_left_road_edge = leftest_lane.left_line.is_near_road_edge;

  bool has_right_road_edge = rightest_lane.right_line.is_near_road_edge;

  // 1. 如果最左侧车道存在路沿，最右侧车道不存在路沿，置信度为0.9
  if (has_left_road_edge && !has_right_road_edge) {
    *road_edge_state = "only_has_left_road_edge";
    *measure_weight = 0.9;
    *lane_index = static_cast<int>(cur_lane.left_lane_ids.size()) + 1;
    std::vector<double> measure_p_lanes(lane_num, 0);
    double sum = 0;
    for (int i = 0; i < static_cast<int>(measure_p_lanes.size()); i++) {
      measure_p_lanes[i] =
          std::exp(-(*lane_index - i - 1) * (*lane_index - i - 1));
      sum += measure_p_lanes[i];
    }
    for (auto& p : measure_p_lanes) {
      p = p / sum;
    }
    return std::make_shared<std::vector<double>>(measure_p_lanes);
  }

  // 2. 如果最左侧车道不存在路沿，最右侧车道存在路沿，置信度为0.9
  if (!has_left_road_edge && has_right_road_edge) {
    *road_edge_state = "only_has_right_road_edge";
    *measure_weight = 0.9;
    *lane_index = lane_num - static_cast<int>(cur_lane.right_lane_ids.size());
    std::vector<double> measure_p_lanes(lane_num, 0);
    double sum = 0;
    for (int i = 0; i < static_cast<int>(measure_p_lanes.size()); i++) {
      measure_p_lanes[i] =
          std::exp(-(*lane_index - i - 1) * (*lane_index - i - 1));
      sum += measure_p_lanes[i];
    }
    for (auto& p : measure_p_lanes) {
      p = p / sum;
    }
    return std::make_shared<std::vector<double>>(measure_p_lanes);
  }

  // 3. 如果最左侧车道存在路沿，最右侧也存在路沿，置信度为0.95
  if (has_left_road_edge && has_right_road_edge) {
    *road_edge_state = "both_left_and_right_road_edge";
    *measure_weight = 0.95;
    *lane_index = static_cast<int>(cur_lane.left_lane_ids.size()) + 1;
    std::vector<double> measure_p_lanes(lane_num, 0);
    double sum = 0;
    for (int i = 0; i < static_cast<int>(measure_p_lanes.size()); i++) {
      measure_p_lanes[i] =
          std::exp(-(*lane_index - i - 1) * (*lane_index - i - 1));
      sum += measure_p_lanes[i];
    }
    for (auto& p : measure_p_lanes) {
      p = p / sum;
    }
    return std::make_shared<std::vector<double>>(measure_p_lanes);
  }
  // 4. 如果最左侧和最右侧车道都没有路沿，置信度为0.5
  {
    *road_edge_state = "neither_left_nor_right_road_edge";
    *measure_weight = 0.5;
    *lane_index = static_cast<int>(cur_lane.left_lane_ids.size()) + 1;
    std::vector<double> measure_p_lanes(lane_num, 0);
    double sum = 0;
    for (int i = 0; i < static_cast<int>(measure_p_lanes.size()); i++) {
      measure_p_lanes[i] =
          std::exp(-(*lane_index - i - 1) * (*lane_index - i - 1));
      sum += measure_p_lanes[i];
    }
    for (auto& p : measure_p_lanes) {
      p = p / sum;
    }
    return std::make_shared<std::vector<double>>(measure_p_lanes);
  }
}

}  // namespace lane_loc
}  // namespace mf
}  // namespace mp
}  // namespace hozon
