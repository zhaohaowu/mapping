/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Guo Zhichong <guozhichong@sensetime.com>
 */

#include <cmath>   // std::floor/std::round
#include <chrono>  // std::chrono
#include <mutex>   // std::mutex

#include "ad_time/ad_time.hpp"

namespace senseAD {
namespace time {

NativeClock::~NativeClock() {}

Time NativeClock::Now() {
    using typename std::chrono::high_resolution_clock;
    using typename std::chrono::nanoseconds;
    using typename std::chrono::time_point_cast;
    auto now_nanosec =
        time_point_cast<nanoseconds>(high_resolution_clock::now());
    return std::move(Time(now_nanosec.time_since_epoch().count()));
}

namespace {
static std::mutex clock_mutex_;
static NativeClock native_clock;
static ClockInterface *global_clock = &native_clock;
}  // namespace

Time Now() {
    std::lock_guard<std::mutex> lock(clock_mutex_);
    return global_clock->Now();
}

Time NativeNow() { return NativeClock().Now(); }

void SetGlobalClock(ClockInterface *clock) {
    std::lock_guard<std::mutex> lock(clock_mutex_);
    global_clock = clock;
}

}  // namespace time
}  // namespace senseAD
