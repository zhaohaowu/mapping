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
  double NowInSeconds();
  double start_;
  double expected_cycle_time_;
  double actual_cycle_time_;
};

}  // namespace util
}  // namespace mp
}  // namespace hozon
