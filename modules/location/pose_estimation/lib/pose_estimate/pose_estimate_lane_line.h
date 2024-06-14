/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： pose_estimate_lane_line.h
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <list>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "modules/location/pose_estimation/lib/hd_map/hd_map.h"
#include "modules/location/pose_estimation/lib/perception/perception.h"
#include "modules/location/pose_estimation/lib/perception/perception_lane_line_fitting.h"
#include "modules/location/pose_estimation/lib/pose_estimate/pose_estimate_base.h"
#include "modules/location/pose_estimation/lib/util/globals.h"
#include "modules/location/pose_estimation/lib/util/graph.h"

namespace hozon {
namespace mp {
namespace loc {

using hozon::mp::loc::ControlPoint;
using hozon::mp::loc::MapBoundaryLine;

struct ValidPose {
  bool valid = true;
  double timeStamp = -1;
  SE3 pose;
  Eigen::Vector3d velocity_vrf;
};

struct ControlPointInfo {
  V3 last_start_point_v{0.0, 0.0, 0.0};
  V3 last_end_point_v{0.0, 0.0, 0.0};
  size_t last_control_points_size = 0;
};

class MatchLaneLine {
 public:
  MatchLaneLine();
  /**
   * @brief get the match pairs
   *
   * @param hd_map : hd map message
   * @param perception : perception message
   * @param T_W_V : vehicle pose in enu
   * @return
   */
  void Match(const HdMap& hd_map, const std::shared_ptr<Perception>& perception,
             const SE3& T_W_V, const ValidPose& T_fc);

  /**
   * @brief check the solve result
   *
   * @param T : vehicle pose in enu
   * @return
   */
  bool CheckIsGoodMatch(const SE3& T);
  bool CompareLaneWidth(const SE3& T);

  /**
   * @brief filter perception lane lines
   *
   * @param lanelines : perception lane lines
   * @param out : filter result
   * @return
   */
  void FilterPercpLaneline(const std::list<LaneLinePerceptionPtr>& lanelines,
                           std::list<LaneLinePerceptionPtr>* const out);

  /**
   * @brief get match pairs
   *
   * @return match pairs
   */
  inline std::vector<PointMatchPair> get_match_pairs() {
    std::vector<PointMatchPair> match_pairs;
    for (const auto& line_match_pair : match_pairs_) {
      for (const auto& point_pair : line_match_pair)
        match_pairs.push_back(point_pair);
    }
    return match_pairs;
  }

  /**
   * @brief get match lane line size
   *
   * @return match lane line size
   */
  int get_match_line_size() { return line_match_pairs_.size(); }

  /**
   * @brief lane line connect result flag
   *
   * @return lane line connect result flag
   */
  bool get_laneline_connect_result() { return lanelineconnect_; }

  /**
   * @brief set timestamp
   * @param ts : timestamp
   *
   * @return timestamp
   */
  inline void set_ts(double ts) { ts_ = ts; }

  /**
   * @brief set ins timestamp
   * @param ins_ts : ins timestamp
   *
   * @return timestamp
   */
  void set_ins_ts(const double& ins_ts);
  void set_linear_vel(const Eigen::Vector3d& linear_vel);
  bool get_big_curvature() { return big_curvature_; }
  inline void GetError(bool* has_err, int* err_type) {
    *has_err = has_err_;
    *err_type = static_cast<int>(err_type_);
  }
  VP SetRvizMergeMapLines();
  inline std::vector<PointMatchPair> get_origin_pairs() {
    std::vector<PointMatchPair> match_pairs;
    for (const auto& line_match_pair : origin_match_pairs_) {
      for (const auto& point_pair : line_match_pair)
        match_pairs.push_back(point_pair);
    }
    return match_pairs;
  }

 private:
  /**
   * @brief line to line match
   * @param map_line_idx : map lane line id
   * @param pecep : perception lane line
   * @param min_match_x : minimum x range to select point
   * @param sample_interval : smaple distance between points
   * @param is_good_check : good match checked flag
   *
   * @return
   */
  void MatchLine2Line(const std::string& map_line_idx,
                      const LaneLinePerceptionPtr& pecep,
                      const double& min_match_x, const double& max_match_x,
                      const double& sample_interval, const bool& is_good_check);
  void CheckIsGoodMatchFCbyLine(const SE3& FC_pose,
                                const Eigen::Vector3d& FC_vel);
  void CalLinesMinDist(const LaneLinePerceptionPtr& percep,
                       const std::unordered_map<std::string, std::vector<V3>>&
                           filtered_fcmap_lines,
                       double* const near, double* const far,
                       const double& far_dis, const double& near_dis);
  bool GetEdgeFitPoints(const std::vector<ControlPoint>& points, const double x,
                        V3* pt);
  void CalPercepMinDist(const Eigen::Vector3d& FC_vel,
                        const LaneLinePerceptionPtr& line,
                        V3* percep_near_point, V3* percep_far_point);
  static bool SortPairByX(const PointMatchPair& pair1,
                          const PointMatchPair& pair2) {
    return pair1.pecep_pv.x() < pair2.pecep_pv.x();
  }

  static bool SortControlPoints(const ControlPoint& cpt1,
                                const ControlPoint& cpt2) {
    return cpt1.point.x() < cpt2.point.x();
  }
  double CalCulatePointToLineDistance(const V3& selected_point,
                                      const V3& line_point1,
                                      const V3& line_point2);
  bool CalCulatePercepAndMapLaneWidth(
      const std::vector<PointMatchPair>& match_pairs,
      const PointMatchPair& ref_pair, const SE3& T_inv, PointMatchPair* pair_v,
      double* sum_percep_lane_width, double* sum_map_lane_width, size_t* count);
  void ComputeLaneWidthDiff(
      const std::vector<PointMatchPair>& left_match_pairs,
      const std::vector<PointMatchPair>& right_match_pairs, const SE3& T_inv,
      double* lane_width_diff);
  void GetAverageDiffY(const std::vector<PointMatchPair>& match_pairs,
                       const SE3& T, double* avg_diff_y);
  bool GetNeighboringMapLines(
      const VP& target_perception_line_points,
      const std::unordered_map<std::string, std::vector<ControlPoint>>&
          map_points_cache,
      const double max_dis, const double min_dis,
      std::vector<std::pair<V3, std::string>>* nearest_line_match_pairs,
      std::vector<std::pair<V3, std::string>>* farest_line_match_pairs);
  void FilterPointPair(std::vector<PointMatchPair>* match_pairs, const SE3& T);
  std::pair<double, double> CmpWidth(const MatchMapLine& match_pair0,
                                     const MatchMapLine& match_pair1);
  /**
   * @brief line to line match
   * @param map_line_idx : map lane line id
   * @param pecep : perception lane line
   * @param min_match_x : minimum x range to select point
   * @param max_match_x : maximum x range to select point
   * @param sample_interval : smaple distance between points
   * @param is_good_check : good match checked flag
   *
   * @return
   */
  void MatchPoint2Line(const std::vector<std::string>& map_line_idxs,
                       const LaneLinePerceptionPtr& pecep,
                       const double& min_match_x, const double& max_match_x,
                       const double& sample_interval,
                       const bool& is_good_check);
  /**
   * @brief select matched lane line
   *
   * @return
   */
  void LaneLineConnect(
      const std::list<std::list<LaneLinePerceptionPtr>>& percep_lanelines,
      const std::unordered_map<std::string, std::vector<ControlPoint>>&
          boundary_lines);
  void GetMatchMapLineCache(
      const std::set<std::string>& line_match_pairs_map_line_id,
      const std::unordered_map<std::string, std::vector<ControlPoint>>&
          boundary_lines,
      const VP& perception_points, const int& percep_line_type,
      const double& max_range_x, const double& min_range_x,
      std::unordered_map<int, std::vector<MatchMapLine>>* match_mapline_cache);
  void SelectBestMatchPairs(
      std::unordered_map<int, std::vector<MatchMapLine>>* match_mapline_cache);
  /**
   * @brief get the matched pairs
   *
   * @return
   */
  void ConnectPoint(const bool& is_good_check);

  /**
   * @brief filter points that meet the x threshold
   * @param control_poins : map lane lines's control points
   * @param x : x thresold
   * @param pt : the point meet the x thresold
   * @param T_W_V : the vehicle pose
   *
   * @return true : get the fit pints; false : do not get the fit points
   */
  bool GetFitPoints(const VP& control_poins, const double x, V3* pt);
  bool GetFcFitMapPoints(const VP& control_poins, const double x, V3* pt);
  bool GetFcFitPoints(const VP& control_poins, const double x, V3* pt);
  bool GetPerceFitPoints(const VP& points, const double x, V3* pt);
  bool GetFitMapPoints(const std::vector<ControlPoint>& control_poins,
                       const double x, V3* pt);

  /**
   * @brief merge the map lane line
   * @param boundary_lines : map boundary lines
   *
   * @return
   */
  void MergeMapLines(const std::shared_ptr<MapBoundaryLine>& boundary_lines,
                     const SE3& T);
  void MergeMapEdges(const std::shared_ptr<MapRoadEdge>& road_edges,
                     const SE3& T);
  void Traversal(const V3& root_start_point, std::vector<std::string> line_ids,
                 int loop);
  void EdgesTraversal(const V3& root_start_point,
                      std::vector<std::string> line_ids, int loop);
  void NormalizeWeight(
      std::vector<std::vector<PointMatchPair>>& match_pairs);  // NOLINT
  void ComputeCurvature(const std::vector<double>& coeffs, const double x,
                        double* curvature);
  bool IsBigCurvaturePercepLine(VP perception_points);
  bool IsLowSpeed();
  void CheckTriggerBigCurv(double cur_time);

 public:
  double ts_;
  double last_ins_timestamp_ = 0.f;
  double ins_timestamp_ = 0.f;
  Eigen::Vector3d match_linear_vel_{0.0, 0.0, 0.0};
  std::vector<std::vector<PointMatchPair>> origin_match_pairs_;
  std::vector<std::vector<PointMatchPair>> match_pairs_;
  std::list<LineMatchPair> line_match_pairs_;
  std::shared_ptr<Perception> percep_;
  std::list<std::list<LaneLinePerceptionPtr>> percep_lanelines_;
  bool lanelineconnect_ = true;
  bool is_double_line_ = false;
  bool is_good_match_;
  bool has_err_;
  ERROR_TYPE err_type_;
  SE3 T_W_V_;
  SE3 T_V_W_;
  PerceptionLaneLineFitting percep_lane_line_curve_fitting_;
  using LineSegment =
      std::pair<std::string, V3>;  // LineSegment: {id, end_point}
  using MatchLaneLinePtr = std::shared_ptr<MatchLaneLine>;

 private:
  struct EigenMatrixHash {
    std::size_t operator()(const Eigen::Matrix<double, 3, 1>& matrix) const {
      return std::hash<double>()(matrix(0)) ^ std::hash<double>()(matrix(1)) ^
             std::hash<double>()(matrix(2));
    }
  };
  struct ControlPointInfoEqual {
    bool operator()(const ControlPointInfo& lhs,
                    const ControlPointInfo& rhs) const {
      return lhs.last_start_point_v.isApprox(rhs.last_start_point_v) &&
             lhs.last_end_point_v.isApprox(rhs.last_end_point_v) &&
             lhs.last_control_points_size == rhs.last_control_points_size;
    }
  };
  struct ControlPointInfoHash {
    std::size_t operator()(const ControlPointInfo& cpt_info) const {
      std::size_t hashValue = 0;
      hashValue ^= EigenMatrixHash()(cpt_info.last_start_point_v);
      hashValue ^= EigenMatrixHash()(cpt_info.last_end_point_v);
      hashValue ^= std::hash<size_t>()(cpt_info.last_control_points_size);
      return hashValue;
    }
  };

 private:
  bool big_curvature_ = false;
  bool check_error_last_ = false;
  std::unordered_set<std::string> edge_visited_id_;
  std::unordered_set<std::string> visited_id_;
  std::map<V3, std::vector<LineSegment>, PointV3Comp<V3>>
      edges_map_;  // map: {{start_point, LineSegment}}
  std::vector<std::vector<std::string>> linked_edges_id_;
  std::vector<std::vector<std::vector<std::string>>>
      multi_linked_edges_;  // 内层vector是一组前后继链接的车道线
  std::unordered_map<std::string, std::vector<ControlPoint>> merged_map_edges_;
  std::map<V3, std::vector<LineSegment>, PointV3Comp<V3>>
      lines_map_;  // map: {{start_point, LineSegment}}
  std::vector<std::vector<std::string>> linked_lines_id_;
  std::vector<std::vector<std::vector<std::string>>>
      multi_linked_lines_;  // 内层vector是一组前后继链接的车道线
  std::unordered_map<std::string, std::vector<ControlPoint>> merged_map_lines_;
  std::unordered_map<std::string, std::vector<ControlPoint>>
      merged_fcmap_lines_;
  std::unordered_set<ControlPointInfo, ControlPointInfoHash,
                     ControlPointInfoEqual>
      lane_control_pointInfo_;
  std::unordered_set<ControlPointInfo, ControlPointInfoHash,
                     ControlPointInfoEqual>
      edge_control_pointInfo_;
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
