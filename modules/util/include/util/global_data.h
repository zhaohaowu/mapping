/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： global_data.h
 *   author     ： taoshaoyuan
 *   date       ： 2024.01
 ******************************************************************************/

#pragma once

#include <yaml-cpp/yaml.h>

#include <string>

namespace hozon {
namespace mp {
namespace util {

class GlobalData {
 public:
  bool Init(const YAML::Node& conf, const std::string& work_root);

  bool EnableViz();
  std::string VizAddr();
  bool VizImg();
  std::string WorkRoot();

 private:
  bool enable_viz_ = false;
  std::string viz_addr_;
  bool viz_img_ = false;
  std::string work_root_;

  // declare singleton
 private:
  GlobalData() = default;
  ~GlobalData() = default;
  GlobalData(const GlobalData&);
  GlobalData& operator=(const GlobalData&);

 public:
  static GlobalData& Instance() {
    static GlobalData instance;
    return instance;
  }
};

#define GLOBAL_DATA hozon::mp::util::GlobalData::Instance()

}  // namespace util
}  // namespace mp
}  // namespace hozon
