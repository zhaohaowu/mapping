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

class Line_kd {
 public:
  Line_kd() = default;
  Line_kd(Line_kd&& other) noexcept
      : lane_boundary_line(std::move(other.lane_boundary_line)),
        line_kdtree(std::move(other.line_kdtree)),
        store(other.store),
        is_continue(other.is_continue),
        is_merge(other.is_merge),
        is_ego_road(other.is_ego_road),
        param(std::move(other.param)) {}
  Line_kd& operator=(Line_kd&& other) noexcept {
    if (this != &other) {
      lane_boundary_line = std::move(other.lane_boundary_line);
      line_kdtree = std::move(other.line_kdtree);
      store = other.store;
      is_continue = other.is_continue;
      is_merge = other.is_merge;
      is_ego_road = other.is_ego_road;
      param = std::move(other.param);
    }
    return *this;
  }

 public:
  std::shared_ptr<std::pair<const Id, Boundary::Ptr>> lane_boundary_line;
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
