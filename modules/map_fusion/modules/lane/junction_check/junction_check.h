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

#include "modules/map_fusion/base/group.h"
#include "modules/map_fusion/base/interface_option.h"
#include "modules/map_fusion/base/junction.h"
#include "modules/map_fusion/common/common_data.h"
#include "modules/map_fusion/data_manager/junction_status_manager.h"

namespace hozon {
namespace mp {
namespace mf {

const std::map<int, RoadSceneType> TypeMap = {
    {0, RoadSceneType::GENERAL_ROAD},
    {1, RoadSceneType::NEAR_JUNCTION},
    {2, RoadSceneType::IN_JUNCTION},
};

struct TypeConvertor {
 public:
  void Update(const RoadSceneType& detected_type) {
    if (!initialized) {
      initialized = true;
      cur_type = detected_type;
      return;
    }

    int next_type = (static_cast<int>(cur_type) + 1) % 3;
    if (static_cast<int>(detected_type) == next_type) {
      if (cnt == 2) {
        auto iter = TypeMap.find(next_type);
        cur_type = iter->second;
        cnt = 0;
        HLOG_INFO << "consecutive 3 next type, change to type "
                  << static_cast<int>(cur_type);
      } else {
        cnt++;
      }

      return;
    }

    cnt = 0;
  }

  void Clear() {
    cnt = 0;
    initialized = false;
  }

  RoadSceneType GetRoadSceneType() { return cur_type; }

 private:
  bool initialized = false;
  RoadSceneType cur_type = RoadSceneType::GENERAL_ROAD;
  int cnt = 0;
};

class JunctionCheck {
 public:
  JunctionCheck() = default;
  ~JunctionCheck() = default;
  int Init(const LaneFusionProcessOption& conf);
  int Process(const std::vector<Group::Ptr>& groups);
  JunctionInfo GetJunctionInfo();
  void Clear();

 private:
  JunctionInfo junc_info_;
  double near_junction_dis_thresh_ = 20.0;
  TypeConvertor type_convetor_;
};

using JunctionCheckPtr = std::unique_ptr<JunctionCheck>;

}  // namespace mf
}  // namespace mp
}  // namespace hozon
