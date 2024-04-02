/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： global_data.cc
 *   author     ： taoshaoyuan
 *   date       ： 2024.01
 ******************************************************************************/

#include "util/global_data.h"

#include "util/mapping_log.h"

namespace hozon {
namespace mp {
namespace util {

bool GlobalData::Init(const YAML::Node& conf, const std::string& work_root) {
  std::vector<std::string> keys = {
      "enable_viz",
      "viz_addr",
      "viz_img",
  };
  for (const auto& it : keys) {
    if (!conf[it].IsDefined()) {
      HLOG_ERROR << "GlobalData conf key " << it << " not exist";
      return false;
    }
  }
  enable_viz_ = conf["enable_viz"].as<bool>();
  viz_addr_ = conf["viz_addr"].as<std::string>();
  viz_img_ = conf["viz_img"].as<bool>();
  work_root_ = work_root;

  return true;
}

bool GlobalData::EnableViz() { return enable_viz_; }

std::string GlobalData::VizAddr() { return viz_addr_; }

bool GlobalData::VizImg() { return viz_img_; }

std::string GlobalData::WorkRoot() { return work_root_; }

}  // namespace util
}  // namespace mp
}  // namespace hozon
