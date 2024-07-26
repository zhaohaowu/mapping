/******************************************************************************
 Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved
 *Author: HOZON
 *Date: 2023-08-05
 *****************************************************************************/

#pragma once

#include <cfloat>
#include <map>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "modules/local_mapping/base/location/dr.h"
#include "modules/local_mapping/base/scene/base.h"
#include "modules/local_mapping/base/scene/occedge.h"

namespace hozon {
namespace mp {
namespace lm {
// @brief lane line definition

enum class FreeSpaceCls {
  UNKNOWN = 0,          // 未知
  GROUND = 1,           // 路面
  FENCE = 2,            // 栅栏、护栏、硬隔离等
  ROADSIDE = 3,         // 路沿 (未做)
  NATURE = 4,           // 绿植
  ROADOBSTACLE = 5,     // 路面上的凸起物，不规则障碍物
  SIDEWALK = 6,         // 人行道
  VEHICLE = 7,          // 车辆
  PEDESTRIAN = 8,       // 行人
  CYCLIST = 9,          // 骑行人
  TRAFFICBARRIER = 10,  // 障碍物、锥桶、水马、石墩等
  FREE = 11,            // free
  UNFREE = 12           // unfree
};
struct OccGrid {
  enum class OccupancyState : int {
    UNKNOWN = 0,
    FREE,
    OCC,
    STATIC,
    DYNAMIC,
  };
  Eigen::Vector2f anchor_pt;
  OccupancyState state;
  FreeSpaceCls cls;
};
struct FreeSpace {
  // 接收外部freespace输入的freespace数据结构
  FreeSpaceCls cls;
  std::vector<Eigen::Vector3d> fs_points;
};
using FreeSpacePtr = std::shared_ptr<FreeSpace>;
struct FreeSpaceOutput {
  // 对外输出的freespace数据结构
  FreeSpaceCls cls = FreeSpaceCls::UNKNOWN;
  bool free = false;  // free 区域或非free区域
  float area = 0;     // 区域面积
  std::unordered_map<int, std::pair<int, int>> edges_index;  // 边界下标
  std::vector<std::vector<std::pair<int, int>>>
      vec_edges_index;  // 把相互连接的边界点归档到线
  std::vector<std::vector<Eigen::Vector3d>>
      vec_edges_points;  // 边界点下标对应的真实车系坐标值
};
struct FreeSpaces {
  double timestamp;
  DrDataConstPtr freespaces_pose;
  OccEdges edges;
};
using FreeSpaceOutputPtr = std::shared_ptr<FreeSpaceOutput>;
struct FreeSpaceOutputs {
  std::vector<FreeSpaceOutput> freespace_outputs;
};

using FreeSpacesPtr = std::shared_ptr<FreeSpaces>;
using FreeSpacesConstPtr = std::shared_ptr<const FreeSpaces>;

using FreeSpacesOutputPtr = std::shared_ptr<FreeSpaceOutputs>;
using FreeSpacesOutputConstPtr = std::shared_ptr<const FreeSpaceOutputs>;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
