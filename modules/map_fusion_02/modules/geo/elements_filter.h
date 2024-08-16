/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： elements_filter.h
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <unordered_map>
#include <vector>

#include <boost/circular_buffer.hpp>

#include "modules/map_fusion_02/base/element_map.h"
#include "modules/map_fusion_02/base/processor.h"
#include "modules/map_fusion_02/modules/geo/elements_filter_base.h"
#include "modules/map_fusion_02/modules/geo/geo_utils.h"
#include "modules/map_fusion_02/rviz/geo_optimization_rviz.h"

namespace hozon {
namespace mp {
namespace mf {
class ElementsFilter : public ProcessorBase {
 public:
  ElementsFilter() {}
  ~ElementsFilter() = default;
  ElementsFilter(const ElementsFilter&) = delete;
  ElementsFilter& operator=(const ElementsFilter&) = delete;
  bool Init() override;
  bool Process(ElementMap::Ptr origin_element_map_ptr);
  void Clear() override;

 private:
  /*过滤不合理的车道线*/
  void FilterElementMapLines(
      const std::map<Id, Boundary::Ptr>& lane_boundaries);
  void FilterIntersectLine();

  /*参考路沿补偿或者延长车道线*/
  void CompensateElementMapLine();
  // void CreateLineTable();
  void MakeRoadEdgeToLaneLine();
  void HandleExtraWideLane();
  void CompareRoadAndLines(const std::vector<Eigen::Vector3d>& road_pts,
                           const int& road_id);
  /*识别逆向车道线*/
  void FilterReverseLine();
  void HandleOppisiteLineByObj();
  std::vector<Eigen::Vector3d> GetdRoadEdgePts();
  std::vector<Eigen::Vector3d> GetDoubleSolidYellowLine();
  std::vector<Eigen::Vector3d> FindTargetPoints(
      const std::vector<std::vector<Eigen::Vector3d>>& forward_road_edges);
  RelativePosition IsTargetOnLineRight(
      const std::vector<Eigen::Vector3d>& target_line, const LineInfo& line);
  RelativePosition IsRoadEdgeOnVehicleRight(
      const std::vector<Eigen::Vector3d>& points, const double& heading);
  void HandleOppisiteLine(const std::vector<Eigen::Vector3d>& target_line);
  void HandleOppisiteLineByObjAndYelloLine();

 private:
  Eigen::Isometry3d T_L_V_;  // 车体系在local系的位姿
  std::set<int> last_track_id_;
  std::unordered_map<int, LineInfo> line_table_;
  std::map<int, RoadEdge::Ptr> road_edge_table_;

  GeoOptimizationViz geo_viz_;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
