/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： tic_toc.h
 *   author     ： taoshaoyuan
 *   date       ： 2022.07
 ******************************************************************************/

#pragma once

namespace hozon {
namespace mp {
namespace util {
class TicToc {
 public:
  TicToc() { Tic(); }

  void Tic() { clock_gettime(CLOCK_REALTIME, &start_); }

  // in ms
  double Toc() {
    clock_gettime(CLOCK_REALTIME, &end_);
    double start_ms = static_cast<double>(start_.tv_sec) * 1e3 +
                      static_cast<double>(start_.tv_nsec) * 1e-6;
    double end_ms = static_cast<double>(end_.tv_sec) * 1e3 +
                    static_cast<double>(end_.tv_nsec) * 1e-6;
    return end_ms - start_ms;
  }

 private:
  timespec start_{};
  timespec end_{};
};

}  // namespace util
}  // namespace mp
}  // namespace hozon
