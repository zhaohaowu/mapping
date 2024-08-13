/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： base_data.h
 *   author     ： hozon
 *   date       ： 2024.01
 ******************************************************************************/

#pragma once
#include <depend/map/hdmap/hdmap.h>
#include <depend/proto/local_mapping/local_map.pb.h>
#include <depend/proto/localization/localization.pb.h>
#include <depend/proto/localization/node_info.pb.h>
#include <depend/proto/map/map.pb.h>

#include <memory>
#include <vector>

#include "opencv2/core/core.hpp"
#include "opencv2/core/hal/interface.h"
#include "opencv2/features2d.hpp"
namespace hozon {
namespace mp {
namespace mf {

enum RoadScene {
  NON_JUNCTION = 0,
  BIG_JUNCTUIN = 1,
  SMALL_JUNCTION = 2,
};

class Line_kd {
 public:
  std::shared_ptr<hozon::mapping::LaneLine> line;
  // std::vector<Eigen::Vector3d> line_points;
  std::shared_ptr<cv::flann::Index> line_kdtree;
  bool store = true;  // 是否存储该线
  bool is_continue = false;  // 是否跟别的线融合了，如果被融合则不添加该线
  bool is_merge = false;  // 是否是汇入的线，如果是汇入的线则保留
  bool is_ego_road =
      true;  // 是否是主路段的线。即road_edge lanepos为-1到1之间的line
  std::vector<double> param = std::vector<double>(3, 0.0);  // 存储c0 c1 c2
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
