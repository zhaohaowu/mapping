/*
 * Copyright (c) hozon auto Co., Ltd. 2022-2023. All rights reserved.
 */

#pragma once

namespace hozon {
namespace mp {
namespace mf {

/* 将两个突变信号变成一个，不区分先后，两个突变的中间时间也算突变，假设在一定时间内每个信号都突变一次
 *                _____
 *               |     |
 *first _________|     |__________________________
 *                   ______
 *                  |      |
 *second ___________|      |______________________
 */
class DoubleRiseDecider {
 public:
  DoubleRiseDecider() = default;
  ~DoubleRiseDecider() = default;
  bool Decider(bool first, bool second);
  void Reset();

  void SetWaitTime(const double max_time) { max_wait_time_ = max_time; }

  enum DoubleRiseState { None, ExpectFirst, ExpextSecond, ExpectEnd };

 private:
  double max_wait_time_ = 1.0;
  double wait_time_ = 0.0;
  DoubleRiseState state_ = DoubleRiseState::None;
  /* data */
};
}  // namespace mf
}  // namespace mp
}  // namespace hozon
