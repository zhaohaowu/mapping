/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 */

#pragma once

#include "localization/data_type/base.hpp"
#include "localization/localization_param.hpp"

namespace senseAD {
namespace localization {

class BaseLocator {
 public:
  BaseLocator() = default;
  virtual ~BaseLocator() = default;

  virtual adLocStatus_t Init(const LocalizationParam& param) = 0;

  virtual adLocStatus_t SetState(const NavState& nav_state) = 0;

  virtual adLocStatus_t GetState(NavState* nav_state,
                                 double* confidence = nullptr) = 0;

  virtual adLocStatus_t Restart() = 0;

  // overload this function for different data process
  // virtual adLocStatus_t Process(/* data type */) { return LOC_SUCCESS; }
};

}  // namespace localization
}  // namespace senseAD
