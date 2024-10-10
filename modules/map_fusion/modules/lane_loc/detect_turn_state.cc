
/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： detect_turn_state.cc
 *   author     ： zhaohaowu
 *   date       ： 2024.07
 ******************************************************************************/
#include <modules/map_fusion/modules/lane_loc/detect_turn_state.h>

#include <memory>
#include <utility>

#include "base/utils/log.h"
#include "modules/map_fusion/modules/lane_loc/base_lane_loc.h"

namespace hozon {
namespace mp {
namespace mf {
namespace lane_loc {

std::shared_ptr<TurnState> DetectTurnState::Run(const Groups& groups) {
  // 找到车辆所在车道
  Lane::Ptr vehicle_lane_ptr = BaseLaneLoc::GetVehicleLane(groups);
  if (vehicle_lane_ptr == nullptr) {
    HLOG_WARN << "can not find vehicle lane";
    return nullptr;
  }

  // 找到当前距离车最近的车道线
  std::pair<double, double> nearest_line;
  auto left_y_ptr =
      BaseLaneLoc::GetYForMinAbsX(vehicle_lane_ptr->left_boundary);
  if (left_y_ptr == nullptr) {
    HLOG_WARN << "left_boundary points size is 0";
    return nullptr;
  }
  auto right_y_ptr =
      BaseLaneLoc::GetYForMinAbsX(vehicle_lane_ptr->right_boundary);
  if (right_y_ptr == nullptr) {
    HLOG_WARN << "right_boundary points size is 0";
    return nullptr;
  }
  if (std::abs(*left_y_ptr) < std::abs(*right_y_ptr)) {
    nearest_line.first = vehicle_lane_ptr->left_boundary->id;
    nearest_line.second = *left_y_ptr;
  } else {
    nearest_line.first = vehicle_lane_ptr->right_boundary->id;
    nearest_line.second = *right_y_ptr;
  }
  TurnState turn_state_msg = STARIGHT;
  static bool first_frame = true;
  static std::pair<double, double> last_nearest_line;
  if (first_frame) {
    first_frame = false;
    last_nearest_line = nearest_line;
    return std::make_shared<TurnState>(turn_state_msg);
  }
  // HLOG_ERROR << "last " << "first " << last_nearest_line.first << " second "
  //            << last_nearest_line.second;
  // HLOG_ERROR << "cur " << "first " << nearest_line.first << " second "
  //            << nearest_line.second;
  if (last_nearest_line.first == nearest_line.first) {
    if (last_nearest_line.second < 0 && nearest_line.second > 0) {
      turn_state_msg = TURN_RIGHT;
    } else if (last_nearest_line.second > 0 && nearest_line.second < 0) {
      turn_state_msg = TURN_LEFT;
    } else {
      turn_state_msg = STARIGHT;
    }
  } else {
    turn_state_msg = STARIGHT;
  }
  last_nearest_line = nearest_line;
  return std::make_shared<TurnState>(turn_state_msg);
}

}  // namespace lane_loc
}  // namespace mf
}  // namespace mp
}  // namespace hozon
