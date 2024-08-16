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
#include <unordered_map>
#include <vector>
#include <boost/circular_buffer.hpp>

#include "modules/map_fusion_02/base/element_map.h"
#include "modules/map_fusion_02/base/processor.h"
#include "modules/map_fusion_02/modules/geo/elements_filter_base.h"
#include "modules/map_fusion_02/rviz/geo_optimization_rviz.h"

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
  void CompensateElementMapLine(const std::map<Id, RoadEdge::Ptr>& road_edges);
  // void CreateLineTable();
  void MakeRoadEdgeToLaneLine(const std::map<Id, RoadEdge::Ptr>& road_edges);
  void HandleExtraWideLane();
  void CompareRoadAndLines(const std::vector<Eigen::Vector3d>& road_pts,
                           const int& road_id);

 private:
  std::unordered_map<int, LineInfo>
      line_table_;  // key: track id; value: line info
  GeoOptimizationViz geo_viz_;
  std::set<int> last_track_id_;  // 记录上一帧trackid
  int point_num_;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
