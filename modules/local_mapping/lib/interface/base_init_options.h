// Copyright 2021 hozon Inc. All Rights Reserved.
// @author: hozon
// @file: base_init_options.h
// @brief: BaseInitOptions

#pragma once

#include <Eigen/Eigen>
#include <string>

namespace hozon {
namespace mp {
namespace lm {

struct BaseInitOptions {
  double timestamp = 0.0;
};

struct BaseProcessOption {
  double timestamp = 0.0;
};

struct ProcessInitOption : public BaseInitOptions {};
struct FilterInitOption : public BaseInitOptions {};
struct FilterOption : public BaseProcessOption {};

struct ProcessOption : public BaseProcessOption {
  Eigen::Affine3d T_cur_last = Eigen::Affine3d::Identity();
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
