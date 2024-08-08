/***************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： interface_option.h
 *   author     ： hozon
 *   date       ： 2024.01
 ******************************************************************************/

#pragma once

#include <Eigen/Eigen>

namespace hozon {
namespace mp {
namespace mf {

struct BaseProcessOption {
  double timestamp = 0.0;
};

// process 函数传递参数
// 各个pipeline可以定义自己的结构
struct ProcessOption : public BaseProcessOption {
  Eigen::Affine3d T_cur_last = Eigen::Affine3d::Identity();
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
