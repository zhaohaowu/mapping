// Copyright 2021 hozon Inc. All Rights Reserved.
// @author: hozon
// @file: base_init_options.h
// @brief: BaseInitOptions

#pragma once

#include <string>

namespace hozon {
namespace mp {
namespace environment {

struct BaseInitOptions {
  // std::string root_dir;
  // std::string conf_file;
};

struct BaseProcessOption {
  double timestamp = 0.0;
};

struct ProcessInitOption : public BaseInitOptions {};

struct ProcessOption : public BaseProcessOption {};

}  // namespace environment
}  // namespace mp
}  // namespace hozon
