/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： data_decider_lite.h
 *   author     ： zhangrui
 *   date       ： 2024.02
 ******************************************************************************/
#pragma once

namespace hozon {
namespace mp {
namespace mf {
constexpr double kInitMinLoopTime = 0.1;
class DebounceModule {
 public:
  DebounceModule() = default;
  DebounceModule(double rise_time, double fall_time, double main_loop_time);
  ~DebounceModule() = default;
  void ResetTime(double rise_time, double fall_time, double main_loop_time);
  void Reset();
  bool DealDebounce(bool input);
  bool LastStatus() const { return in_pre_; }
  static int Compare(const double x, const double y);

 private:
  bool in_pre_ = false;
  double rise_time_val_ = 0.0;
  double fall_time_val_ = 0.0;
  double rise_time_limit_{kInitMinLoopTime};
  double fall_time_limit_{0.0};
  double main_loop_time_{kInitMinLoopTime};
};
}  // namespace mf
}  // namespace mp
}  // namespace hozon
