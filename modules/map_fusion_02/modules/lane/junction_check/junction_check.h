/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： detect_cut_pt.h
 *   author     ： pengwei
 *   date       ： 2024.05
 ******************************************************************************/

#pragma once

#include <stdlib.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/map_fusion_02/base/group.h"
#include "modules/map_fusion_02/base/interface_option.h"
#include "modules/map_fusion_02/common/common_data.h"
#include "modules/map_fusion_02/base/junction.h"

namespace hozon {
namespace mp {
namespace mf {

class JunctionCheck {
 public:
  JunctionCheck() = default;
  ~JunctionCheck() = default;
  int Init(const LaneFusionProcessOption& conf);
  void Process(const std::vector<Group::Ptr>& groups);
  JunctionInfo GetJunctionInfo();
  void Clear();

 private:
  JunctionInfo junc_info_;
  double near_junction_dis_thresh_ = 20.0;
};

using JunctionCheckPtr = std::unique_ptr<JunctionCheck>;

}  // namespace mf
}  // namespace mp
}  // namespace hozon
