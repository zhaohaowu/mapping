/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： elements_filter_base.h
 *   author     ： ouyanghailin
 *   date       ： 2024.08
 ******************************************************************************/

#pragma once

#include <memory>
#include <utility>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>

#include "modules/map_fusion_02/base/element_base.h"
#include "modules/map_fusion_02/base/element_map.h"

namespace hozon {
namespace mp {
namespace mf {

struct LineInfo {
  int line_track_id;
  int flag = 2;  // 0-->最左车道线，1-->最右车道线，2-->普通车道线
  LineType line_type;
  Color color;
  bool store = true;  // 是否存储该线
  bool is_continue = false;  // 是否跟别的线融合了，如果被融合则不添加该线
  bool is_merge = false;  // 是否是汇入的线，如果是汇入的线则保留
  bool is_ego_road = true;  // 是否主路的线
  std::shared_ptr<cv::flann::Index> line_kdtree = nullptr;
  std::vector<Eigen::Vector3f> line_pts;
  std::vector<double> right_width{0.f};       // 距离右边线的距离
  std::vector<double> right_road_width{0.f};  // 距离右侧路沿距离
  std::vector<double> left_road_width{0.f};   // 距离左侧路沿距离
  std::vector<double> param = std::vector<double>(3, 0.0);  // 存储c0 c1 c2
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
