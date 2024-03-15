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
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "modules/location/pose_estimation/lib/hd_map/hd_map.h"
#include "modules/location/pose_estimation/lib/perception/perception.h"
#include "modules/location/pose_estimation/lib/perception/perception_lane_line_fitting.h"
#include "modules/location/pose_estimation/lib/pose_estimate/frechet_distance.h"
#include "modules/location/pose_estimation/lib/pose_estimate/pose_estimate_base.h"
#include "modules/location/pose_estimation/lib/util/globals.h"
#include "modules/location/pose_estimation/lib/util/graph.h"

namespace hozon {
namespace mp {
namespace loc {

using hozon::mp::loc::ControlPoint;
using hozon::mp::loc::MapBoundaryLine;

class MatchLaneLine {
 public:
  MatchLaneLine();
  bool has_err_;
  ERROR_TYPE err_type_;
  /**
   * @brief get the match pairs
   *
   * @param hd_map : hd map message
   * @param perception : perception message
   * @param T_W_V : vehicle pose in enu
   * @return
   */
  void Match(const HdMap& hd_map, const std::shared_ptr<Perception>& perception,
             const SE3& T_W_V, const SE3& T_fc);

  /**
   * @brief check the solve result
   *
   * @param T : vehicle pose in enu
   * @return
   */
  bool CheckIsGoodMatch(const SE3& T);
  bool CompareLaneWidth(const SE3& T);
  bool is_double_line_ = false;

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
  inline std::vector<PointMatchPair> get_match_pairs() { return match_pairs_; }
  // inline std::vector<PointMatchPair> get_origin_match_pairs() { return
  // origin_match_pairs_; }

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
  inline void GetError(bool* has_err, int* err_type) {
    *has_err = has_err_;
    *err_type = static_cast<int>(err_type_);
  }
  VP SetRvizMergeMapLines();
  inline std::vector<PointMatchPair> getOriginMatchPairs() {
    return origin_match_pairs_;
  }
  std::vector<PointMatchPair> origin_match_pairs_;
  using Ptr = std::shared_ptr<MatchLaneLine>;

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
  void CheckIsGoodMatchFCbyLine(const SE3& T_fc);
  void CalLinesMinDist(const LaneLinePerceptionPtr& percep, const SE3& T_fc,
                       double* const near, double* const far,
                       const double& far_dis);

  static bool SortPairByX(const PointMatchPair& pair1,
                          const PointMatchPair& pair2) {
    return pair1.pecep_pv.x() < pair2.pecep_pv.x();
  }

  static bool SortControlPoints(const ControlPoint& cpt1,
                                const ControlPoint& cpt2) {
    return cpt1.point.x() < cpt2.point.x();
  }

  void ComputeLaneWidth(const std::vector<PointMatchPair>& left_match_pairs,
                        const std::vector<PointMatchPair>& right_match_pairs,
                        const SE3& T, double* percep_lane_width,
                        double* map_lane_width);
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

  /**
   * @brief location safety strategy about checking the match is good by fc
   * output pose
   *
   * @return
   */
  // void CheckIsGoodMatchFcByLine(const SE3& T_fc);

  /**
   * @brief get the matched pairs
   *
   * @return
   */
  void ConnectPoint(const bool& is_good_check);

  /**
   * @brief judge whether the perception lane line is curve
   *
   * @return true : yes; false : no
   */
  bool IsPercepLineCurve(const LaneLinePerceptionPtr& line);

  /**
   * @brief non maximum suppression filters out similar lane lines
   * @param map_lines : map lane lines
   *
   * @return
   */
  void NmsByLat(std::vector<std::pair<V3, std::string>>* map_lines);

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
  bool GetFcFitPoints(const VP& control_poins, const double x, V3* pt,
                      const SE3& T_W_V);
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

  /**
   * @brief filter map lane connected to specified perception lane
   * @param mlane_ids : map boundary lane lines id
   * @param plane : perception lane lines
   *
   * @return
   */
  void FilterMapLane(std::vector<std::string>* mlane_ids,
                     const LaneLinePerceptionPtr& plane);

  /**
   * @brief group map lanes by project in X axis
   * @param mlane_ids : map boundary lane lines id
   * @param groups : grouped map lanes
   *
   * @return
   */
  void GroupByXProject(const std::vector<std::string>& mlane_ids,
                       std::vector<std::list<std::string>>* groups);

  /**
   * @brief select best map lane, minimum Y distance between perception lane
   * @param mlane_ids : map boundary lane lines id
   * @param plane : grouped map lanes
   * @param best_mlane_id : best map lane line id
   *
   * @return true : best map lane line meet the condition exists; false : do not
   * exist
   */
  bool SelectBestMapLane(const std::list<std::string>& mlane_ids,
                         const LaneLinePerceptionPtr& plane,
                         std::string* best_mlane_id);

  /**
   * @brief find both side nearest perception line, max level LL or RR
   * @param lines : perception lane lines
   * @param left_id_ptr : left lane line nearest
   * @param right_id_ptr : right lane line nearest
   *
   * @return
   */
  void GetNearestPercepLinesId(const std::list<LaneLinePerceptionPtr>& lines,
                               int* const left_id_ptr, int* const right_id_ptr);

  /**
   * @brief find the nearest and the farest distance in Y axis
   * @param T_fc : fc output pose
   * @param near : nearest distance
   * @param far : farest distance
   * @param far_dis : last distance
   *
   * @return
   */
  // void CalLinesMinDist(const LaneLinePerceptionPtr& percep, const SE3& T_fc,
  //                      double* const near, double* const far,
  //                      const double& far_dis);
  void Traversal(const std::map<V3, std::vector<std::pair<std::string, V3>>,
                                PointV3Comp<V3>>& lines,
                 const V3& root_start_point,
                 std::vector<std::vector<std::string>>* linked_lines_id,
                 std::vector<std::string> line_ids, int loop);
  void AdjustWeightByLaneWidth(const std::vector<PointMatchPair>& match_pairs,
                               const SE3& T, size_t* left_pairs_count,
                               size_t* right_pairs_count,
                               bool* adjust_left_pairs_weight,
                               bool* adjust_right_pairs_weight);
  void ComputeCurvature(const std::vector<double>& coeffs, const double x,
                        double* curvature);

 public:
  V3 anchor_pt0_{2.0, 0.0, 0.0};
  V3 anchor_pt1_{22.0, 0.0, 0.0};
  static constexpr double max_match_y_thres = 2;
  double max_x_observe_thres_ = 40.f;
  double min_x_observe_thres_ = -0.f;
  double max_y_observe_thres_ = 15.f;
  double nearest_point_max_dist_thres_ = 20.f;
  double ts_;
  double last_ins_timestamp_ = 0;
  double ins_timestamp_;
  std::vector<PointMatchPair> match_pairs_;
  std::shared_ptr<Perception> percep_;
  std::list<LineMatchPair> line_match_pairs_;
  std::list<std::list<LaneLinePerceptionPtr>> percep_lanelines_;
  std::list<LaneLinePerceptionPtr>* fil_percep_laneline_;
  bool lanelineconnect_ = true;
  bool is_good_match_;
  SE3 T_W_V_;
  SE3 T_V_W_;
  PerceptionLaneLineFitting percep_lane_line_curve_fitting_;
  typedef std::pair<std::string, V3>
      LineSegment;  // LineSegment: {id, end_point}

 private:
  struct EigenMatrixHash {
    std::size_t operator()(const Eigen::Matrix<double, 3, 1>& matrix) const {
      std::size_t seed = 0;
      for (size_t i = 0; i < matrix.size(); ++i) {
        // 将每个元素的哈希值与当前种子值混合
        seed ^= std::hash<double>()(matrix[i]) + 0x9e3779b9 + (seed << 6) +
                (seed >> 2);
      }
      return seed;
    }
  };

 private:
  bool big_curvature_ = false;
  std::shared_ptr<hozon::mp::loc::FrechetDistance3D> frechet_distance3D_ =
      nullptr;
  std::unordered_set<std::string> visited_id_;
  std::vector<std::vector<std::vector<std::string>>>
      multi_linked_lines_;  // 内层vector是一组前后继链接的车道线
  std::vector<std::string> linked_line_;
  std::vector<std::string> copy_linked_line_;
  std::unordered_map<std::string, std::vector<ControlPoint>> merged_map_lines_;
  std::unordered_map<std::string, std::vector<ControlPoint>>
      merged_fcmap_lines_;
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
