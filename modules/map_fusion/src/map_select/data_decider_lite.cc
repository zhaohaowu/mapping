/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： data_decider_lite.cc
 *   author     ： zhangrui
 *   date       ： 2024.02
 ******************************************************************************/
#include "map_fusion/map_select/data_decider_lite.h"

namespace hozon {
namespace mp {
namespace mf {
constexpr double epslion = 1e-8;
DebounceModule::DebounceModule(const double rise_time, const double fall_time,
                               const double main_loop_time)
    : rise_time_limit_(rise_time),
      fall_time_limit_(fall_time),
      main_loop_time_(main_loop_time) {}

void DebounceModule::ResetTime(const double rise_time, const double fall_time,
                               const double main_loop_time) {
  rise_time_limit_ = rise_time;
  fall_time_limit_ = fall_time;
  main_loop_time_ = main_loop_time;
}

void DebounceModule::Reset() {
  in_pre_ = false;
  rise_time_val_ = 0.0;
  fall_time_val_ = 0.0;
}

bool DebounceModule::DealDebounce(bool input) {
  if (input && !in_pre_) {
    fall_time_val_ = 0.0;
    if (Compare(rise_time_val_, rise_time_limit_) == -1) {
      rise_time_val_ += main_loop_time_;
    } else {
      rise_time_val_ = 0.0;
      in_pre_ = input;
    }
  } else if (!input && in_pre_) {
    rise_time_val_ = 0.0;
    if (Compare(fall_time_val_, fall_time_limit_) == -1) {
      fall_time_val_ += main_loop_time_;
    } else {
      fall_time_val_ = 0.0;
      in_pre_ = input;
    }
  } else {
    fall_time_val_ = 0.0;
    rise_time_val_ = 0.0;
    in_pre_ = input;
  }
  return in_pre_;
}
int DebounceModule::Compare(const double x, const double y) {
  const double result = x - y;
  if (result < -epslion) {
    return -1;
  }
  if (result > epslion) {
    return 1;
  }
  return 0;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
