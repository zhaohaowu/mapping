/*
 * Copyright (c) hozon auto Co., Ltd. 2022-2023. All rights reserved.
 */
#pragma once
#include <cmath>
#include <iostream>
#include <tuple>
#include <utility>

#include "common/status/status.h"
#include "depend/proto/perception/transport_element.pb.h"
#include "modules/map_fusion/include/map_fusion/map_select/lane_line_delay.h"
#include "modules/util/include/util/mapping_log.h"
namespace hozon {
namespace mp {
namespace mf {
enum ChangeLaneState { None, Start, In, End };

class LaneChangeObserver {
 public:
  LaneChangeObserver() = default;
  ~LaneChangeObserver() = default;
  // apollo::common::Status Init();
  std::pair<bool, bool> Observer(
      const ::hozon::perception::LaneInfo& left_lane,
      const ::hozon::perception::LaneInfo& right_lane);
  void Reset();

 private:
  Delay<std::tuple<double, double, double, double>> change_lane_delay_{5};
  ChangeLaneState change_lane_state_ = ChangeLaneState::None;
  double change_laneline_jump_width_ = 2.0;
  double better_quality_threshold_ = 0.6;
  double good_quality_threshold_ = 0.3;
  bool is_left_change_ = false;
  bool is_right_change_ = false;
  int change_count_ = 0;
};
}  // namespace mf
}  // namespace mp
}  // namespace hozon
