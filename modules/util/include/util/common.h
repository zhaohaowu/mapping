/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： common.h
 *   author     ： nihongjie
 *   date       ： 2024.01
 ******************************************************************************/

#pragma once
#include <iomanip>
#include <iostream>
#include <string>

namespace hozon {
namespace mp {
namespace util {

class common {
 public:
  common() {}
  ~common() {}
  std::string Precusion(double num, int n) {
    std::stringstream ss;
    ss << std::setprecision(n) << num;
    return ss.str();
  }
};

}  // namespace util
}  // namespace mp
}  // namespace hozon
