/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： rate.h
 *   author     ： taoshaoyuan
 *   date       ： 2022.07
 ******************************************************************************/

#pragma once

#include <cstdint>

namespace hozon {
namespace mp {
namespace util {

class Rate {
 public:
  explicit Rate(double hz);
  explicit Rate(uint64_t us);

  bool Sleep();
  void Reset();
  // Return actual cycle time, in seconds
  double CycleTime() const;
  // Return expected cycle time, in seconds
  double ExpectedCycleTime() const;

 private:
  static double NowInSeconds();
  double start_ = 0.;
  double expected_cycle_time_ = 0.;
  double actual_cycle_time_ = 0.;
};

}  // namespace util
}  // namespace mp
}  // namespace hozon
