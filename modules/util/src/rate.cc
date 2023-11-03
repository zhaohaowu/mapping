/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： rate.cc
 *   author     ： taoshaoyuan
 *   date       ： 2022.07
 ******************************************************************************/

#include "util/rate.h"

#include <chrono>
#include <thread>

namespace hozon {
namespace mp {
namespace util {

Rate::Rate(double hz) : expected_cycle_time_(1. / hz), actual_cycle_time_(0.) {
  start_ = NowInSeconds();
}

Rate::Rate(uint64_t us)
    : expected_cycle_time_(static_cast<double>(us) * 1e-6),
      actual_cycle_time_(0.) {
  start_ = NowInSeconds();
}

bool Rate::Sleep() {
  double expected_end = start_ + expected_cycle_time_;
  double actual_end = NowInSeconds();
  double sleep_time = expected_end - actual_end;
  actual_cycle_time_ = actual_end - start_;
  start_ = expected_end;
  if (sleep_time <= 0.) {
    if (actual_end > expected_end + expected_cycle_time_) {
      start_ = actual_end;
    }
    return false;
  }
  std::this_thread::sleep_for(
      std::chrono::nanoseconds(static_cast<int64_t>(sleep_time * 1e9)));
  return true;
}

void Rate::Reset() { start_ = NowInSeconds(); }

double Rate::CycleTime() const { return actual_cycle_time_; }

double Rate::ExpectedCycleTime() const { return expected_cycle_time_; }

double Rate::NowInSeconds() {
  timespec ts{};
  clock_gettime(CLOCK_REALTIME, &ts);
  double secs =
      static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
  return secs;
}

}  // namespace util
}  // namespace mp
}  // namespace hozon
