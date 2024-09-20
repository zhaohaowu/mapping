/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： elements_filter.h
 *   author     ： oyhl
 *   date       ： 2024.09
 ******************************************************************************/

#pragma once

#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <boost/circular_buffer.hpp>

#include "modules/map_fusion_02/base/element_base.h"
#include "modules/map_fusion_02/base/element_map.h"
#include "modules/map_fusion_02/base/processor.h"
#include "modules/map_fusion_02/modules/geo/geo_utils.h"

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
  void AddNewLineByRoadPoints(
      const std::vector<Eigen::Vector3f>& target_line_pts,
      const std::vector<Eigen::Vector3f>& road_pts, const int& road_id);
  bool GetClosestLineToRoad(const std::vector<Eigen::Vector3f>& road_pts,
                            int* target_line_track_id, double* min_dis);
  void CompareRoadAndLines(const std::vector<Eigen::Vector3f>& road_pts,
                           const int& road_id);
  void CompensatePoints(const std::vector<Eigen::Vector3f>& road_pts,
                        const int& target_line_track_id, const bool flag,
                        std::vector<Eigen::Vector3f>* target_line_pts);
  bool IsBetweenLinesMid(
      const std::vector<Eigen::Vector3f>& compensated_line_pts,
      const int& target_line_track_id, const bool& flag);
  /*识别逆向车道线*/
  void FilterReverseLine();
  void HandleOppisiteLineByObj();
  std::vector<Eigen::Vector3f> GetdRoadEdgePts();
  std::vector<Eigen::Vector3f> GetDoubleSolidYellowLine();
  std::vector<Eigen::Vector3f> FindTargetPoints(
      const std::vector<std::vector<Eigen::Vector3f>>& forward_road_edges);
  void HandleOppisiteLine(const std::vector<Eigen::Vector3f>& target_line);
  void HandleOppisiteLineByObjAndYelloLine();
  void GetHistoryObjsInMiddleOfLane(
      const boost::circular_buffer<std::shared_ptr<Object>>& history_objs,
      std::vector<Eigen::Vector3f>* obj_points);
  /*根据停止线过滤逆向车道*/
  void HandleOppisiteLineByStopline();
  void GetStopLine(
      std::vector<std::vector<Eigen::Vector2f>>* forward_stoplines);
  /*过滤非路口场景非主路车道线*/
  void FilterNoEgoLineNoCrossing();
  void HandleOppisiteLineNoCrossing(
      const std::vector<Eigen::Vector3f>& target_road_edge);
  void FindTargetPointsNoCrossing(
      const std::vector<std::vector<Eigen::Vector3f>>& nearby_road_edges,
      std::vector<Eigen::Vector3f>* target_road_edge);
  void UpdateElementMapLines(ElementMap::Ptr element_map_ptr);

 private:
  Eigen::Isometry3d T_L_V_;  // 车体系在local系的位姿
  std::set<int> last_track_id_;
  std::unordered_set<int> is_not_ego_lane_track_id_;
  std::unordered_map<int, GeoLineInfo> line_table_;
  std::map<int, RoadEdge::Ptr> road_edge_table_;
  std::map<int, StopLine::Ptr> stop_lines_talbe_;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
