/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： elements_filter.h
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <map>
#include <memory>
#include <set>
#include <vector>
#include <boost/circular_buffer.hpp>

#include "depend/proto/perception/perception_obstacle.pb.h"
#include "modules/map_fusion_02/base/element_map.h"
#include "modules/map_fusion_02/base/processor.h"
#include "modules/map_fusion_02/rviz/geo_optimization_rviz.h"
#include "modules/map_fusion_02/modules/geo/elements_filter_base.h"

namespace hozon {
namespace mp {
namespace mf {
class ElementsFilter : public ProcessorBase {
 public:
  ElementsFilter() : point_num_(-1) {}
  ~ElementsFilter() = default;
  ElementsFilter(const ElementsFilter&) = delete;
  ElementsFilter& operator=(const ElementsFilter&) = delete;
  bool Init() override;
  bool Process(ElementMap::Ptr origin_element_map_ptr);
  void Clear() override;

 private:
  void FilterElementMapLines(
      const std::map<Id, Boundary::Ptr>& lane_boundaries);
  void FilterIntersectLine();

 private:
  std::map<int, std::vector<Line_kd>> all_lines_;  // 用于存储当前所有line信息
  GeoOptimizationViz geo_viz_;
  // copy from origin_element_map
  std::set<int> last_track_id_;  // 记录上一帧trackid
  int point_num_;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
