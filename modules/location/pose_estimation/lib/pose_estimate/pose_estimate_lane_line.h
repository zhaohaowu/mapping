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
#include <vector>

#include "modules/location/pose_estimation/lib/hd_map/hd_map.h"
#include "modules/location/pose_estimation/lib/perception/perception.h"
#include "modules/location/pose_estimation/lib/pose_estimate/pose_estimate_base.h"
#include "modules/location/pose_estimation/lib/util/globals.h"
#include "modules/location/pose_estimation/lib/util/graph.h"

namespace hozon {
namespace mp {
namespace loc {

using hozon::mp::loc::MapBoundaryLine;

class MatchLaneLine {
 public:
  MatchLaneLine();
  /**
   * @brief get the match pairs
   *
   * @param hd_map : hd map message
   * @param perception : perception message
   * @param T_W_V : vehicle pose in enu
   * @param T_fc : fusion center output pose in enu
   * @param percep_points : perception lane lines' point
   * @param nearest_map_points : the map lane line points closest vehicle
   * @return
   */
  void Match(const HdMap &hd_map, const std::shared_ptr<Perception> &perception,
             const SE3 &T_W_V, const SE3 &T_fc, const VP &percep_points,
             const VP &nearest_map_points);

  /**
   * @brief check the solve result
   *
   * @param T : vehicle pose in enu
   * @return
   */
  bool CheckIsGoodMatch(const SE3 &T);

  /**
   * @brief filter perception lane lines
   *
   * @param lanelines : perception lane lines
   * @param out : filter result
   * @return
   */
  void FilterPercpLaneline(const std::list<LaneLinePerceptionPtr> &lanelines,
                           std::list<LaneLinePerceptionPtr> *const out);
  //   void DetectFcOffsetOneLane(
  //       const HDMap &hd_map,
  //       const std::shared_ptr<RoadMarkingPerception> &perception,
  //       const SE3 &T_W_V, VP &percep_points, VP &nearest_map_points);
  //   bool CheckIsGoodMatchFC(const SE3 &T, double fc_timestamp,
  //                           double match_dis_max);

  /**
   * @brief get match pairs
   *
   * @return match pairs
   */
  inline std::vector<PointMatchPair> get_match_pairs() { return match_pairs_; }

  /**
   * @brief get debug map lane line points and perception lane line points
   *
   * @return debug match pair points
   */
  inline std::vector<V3> get_debug_points() { return debug_points_; }

  /**
   * @brief get error flag
   * @param has_err : has error flag; err_type : error type
   *
   * @return error type
   */
  inline void get_error(bool *has_err, int *err_type) {
    (*has_err) = has_err_;
    (*err_type) = static_cast<int>(err_type_);
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
  void set_ins_ts(const double &ins_ts);
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
  void MatchLine2Line(const int &map_line_idx,
                      const LaneLinePerceptionPtr &pecep,
                      const double &min_match_x, const double &max_match_x,
                      const double &sample_interval, const bool &is_good_check);

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
  void MatchPoint2Line(const std::vector<int> &map_line_idxs,
                       const LaneLinePerceptionPtr &pecep,
                       const double &min_match_x, const double &max_match_x,
                       const double &sample_interval,
                       const bool &is_good_check);
  /**
   * @brief select matched lane line
   *
   * @return
   */
  void LaneLineConnect();

  /**
   * @brief location safety strategy about checking the match is good by fc
   * output pose
   *
   * @return
   */
  void CheckIsGoodMatchFcByLine(const SE3 &T_fc);

  /**
   * @brief get the matched pairs
   *
   * @return
   */
  void ConnectPoint(const bool &is_good_check);

  /**
   * @brief judge whether the perception lane line is curve
   *
   * @return true : yes; false : no
   */
  bool IsPercepLineCurve(const LaneLinePerceptionPtr &line);

  /**
   * @brief non maximum suppression filters out similar lane lines
   * @param map_lines : map lane lines
   *
   * @return
   */
  void NmsByLat(std::vector<AlternativeMapLine> *map_lines);

  /**
   * @brief filter points that meet the x threshold
   * @param control_poins : map lane lines's control points
   * @param x : x thresold
   * @param pt : the point meet the x thresold
   * @param T_W_V : the vehicle pose
   *
   * @return true : get the fit pints; false : do not get the fit points
   */
  bool GetFitPoints(const VP &control_poins, const double x, V3 *pt,
                    const SE3 &T_W_V);

  /**
   * @brief merge the map lane line
   * @param boundary_lines : map boundary lines
   *
   * @return
   */
  void MergeMapLines(const std::shared_ptr<MapBoundaryLine> &boundary_lines);

  /**
   * @brief filter map lane connected to specified perception lane
   * @param mlane_ids : map boundary lane lines id
   * @param plane : perception lane lines
   *
   * @return
   */
  void FilterMapLane(std::vector<int> *mlane_ids,
                     const LaneLinePerceptionPtr &plane);

  /**
   * @brief group map lanes by project in X axis
   * @param mlane_ids : map boundary lane lines id
   * @param groups : grouped map lanes
   *
   * @return
   */
  void GroupByXProject(const std::vector<int> &mlane_ids,
                       std::vector<std::list<int>> *groups);

  /**
   * @brief select best map lane, minimum Y distance between perception lane
   * @param mlane_ids : map boundary lane lines id
   * @param plane : grouped map lanes
   * @param best_mlane_id : best map lane line id
   *
   * @return true : best map lane line meet the condition exists; false : do not
   * exist
   */
  bool SelectBestMapLane(const std::list<int> &mlane_ids,
                         const LaneLinePerceptionPtr &plane,
                         int *best_mlane_id);

  /**
   * @brief find both side nearest perception line, max level LL or RR
   * @param lines : perception lane lines
   * @param left_id_ptr : left lane line nearest
   * @param right_id_ptr : right lane line nearest
   *
   * @return
   */
  void GetNearestPercepLinesId(const std::list<LaneLinePerceptionPtr> &lines,
                               int *const left_id_ptr, int *const right_id_ptr);

  /**
   * @brief find the nearest and the farest distance in Y axis
   * @param T_fc : fc output pose
   * @param near : nearest distance
   * @param far : farest distance
   * @param last_dis : last distance
   *
   * @return
   */
  void CalLinesMinDist(const LaneLinePerceptionPtr &percep, const SE3 &T_fc,
                       double *const near, double *const far,
                       const double &last_dis);

  /**
   * @brief print map lane line and perception lane line points
   * @param boundary_lines : map boundary lane lines
   * @param percep_lines : perception lane lines
   * @param KEY : string
   *
   * @return
   */
  void DebugPrintPoints(const std::shared_ptr<MapBoundaryLine> &boundary_lines,
                        const std::vector<PerceptionElement::Ptr> &percep_lines,
                        std::string KEY);
  /**
   * @brief print map lane line and perception lane line
   * @param lines : map lane lines and perception lane lines
   * @param KEY : string
   *
   * @return
   */
  void DebugPrintLines(
      const std::unordered_map<int, std::unordered_set<int>> lines,
      std::string KEY);

  /**
   * @brief print matched point pairs
   * @param pair : map lane lines and perception lane lines' matched points
   * @param stamp : process timestamp
   * @param KEY : string
   *
   * @return
   */
  void DebugPrintPair(const std::vector<V3> &pair, const double &stamp,
                      const std::string &KEY);

  /**
   * @brief print merged map lane lines
   * @param KEY : string
   *
   * @return
   */
  void DebugPrintMergedLines(const std::string &KEY);

  /**
   * @brief print matched map and perception points
   *
   * @return
   */
  void DebugPrintMapPpMatch();

  V3 anchor_pt0_{2.0, 0.0, 0.0};
  V3 anchor_pt1_{22.0, 0.0, 0.0};
  static constexpr double max_match_y_thres = 2;
  double max_x_observe_thres_ = 40.f;
  double min_x_observe_thres_ = -0.f;
  double max_y_observe_thres_ = 15.f;
  double nearest_point_max_dist_thres_ = 20.f;
  double ts_;
  double ins_timestamp_;
  std::vector<PointMatchPair> match_pairs_;
  std::vector<V3> debug_points_;
  std::unordered_map<int, std::unordered_set<int>> debug_lines_before_merge_,
      debug_lines_after_merge_;
  std::map<int, std::vector<V3>> map_merge_lines_;
  std::shared_ptr<Perception> percep_;
  std::list<LineMatchPair> line_match_pairs_;
  std::list<std::list<LaneLinePerceptionPtr>> percep_lanelines_;
  std::list<LaneLinePerceptionPtr> *fil_percep_laneline_;
  bool has_err_;
  bool lanelineconnect_ = true;
  bool is_good_match_;
  ErrorType err_type_;
  SE3 T_W_V_;
  SE3 T_V_W_;
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
