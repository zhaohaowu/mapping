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

#include "modules/map_fusion/common/common_data.h"

namespace hozon {
namespace mp {
namespace mf {
/*
  路口场景a:

              _________
              | | | | |
              | | |↑| |
              | | | | |

  路口场景b:
              |    |    |
              |    |    |
              |    |    |
              |    |    |


               _________
               | | | | |
               | | |↑| |
               | | | | |

*/
enum class RoadSceneType {
  GENERAL_ROAD = 0,  // 普通路段
  NEAR_JUNCTION,     // 接近路口
  IN_JUNCTION,       // 在路口中
};

struct JunctionInfo {
  RoadSceneType road_scene_type = RoadSceneType::GENERAL_ROAD;  // 路口场景
  RoadSceneType last_road_scene_type =
      RoadSceneType::GENERAL_ROAD;  // 路口场景上一次状态
  Eigen::Vector3d along_path_vec;  // 自车与路口前的group的位移（向量）
  size_t cross_before_lane_num = 0;  // 路口前group的lanes数量
  size_t cross_after_lane_num = 0;   // 路口后group的lanes数量
  std::set<std::string>
      next_satisefy_lane_seg;  // 记录路口后满足连接条件的所有lane名称
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
