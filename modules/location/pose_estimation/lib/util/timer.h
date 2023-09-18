/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： timer.h
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <chrono>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>

#include "modules/util/include/util/temp_log.h"

namespace hozon {
namespace mp {
namespace loc {

struct TimeInfo {
  double average_dt = -1;
  double times = 0;
};

class Timer {
 public:
  inline void start() {
    t_s_ = std::chrono::steady_clock::now();  // ns
  }

  inline void stop() { t_e_ = std::chrono::steady_clock::now(); }

  inline double duration() {
    return std::chrono::duration_cast<std::chrono::duration<double>>(t_e_ -
                                                                     t_s_)
               .count() *
           1000.0;
  }

  template <class F>
  void evaluate(F &&func, const std::string &func_name) {
    auto t1 = std::chrono::steady_clock::now();
    std::forward<F>(func)();
    auto t2 = std::chrono::steady_clock::now();
    double dt =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
            .count() *
        1000.0;
    auto &info = status_[func_name];
    if (info.average_dt < 0) {
      info.average_dt = dt;
      info.times++;
    } else {
      info.average_dt = 0.5 * (info.average_dt + dt);
      info.times++;
    }
  }
  void print(void) {
    // HLOG_ERROR << "\t \t \t The system time consumption statistics are as
    // follows";
    for (auto &p : status_) {
      HLOG_ERROR << ">> \t " << p.first << "\t \t"
                 << "average dt:" << p.second.average_dt << "\t(ms)";
    }
  }

 private:
  std::unordered_map<std::string, TimeInfo> status_;
  std::chrono::steady_clock::time_point t_s_;  // start time ponit
  std::chrono::steady_clock::time_point t_e_;  // stop time point
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
