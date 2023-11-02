/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： globals.h
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

namespace hozon {
namespace mp {
namespace loc {

struct MapMatchLaneLineParams {
  // for get map element
  double map_distance;
  // perception lane config
  double lane_confidence_thre;

  // about solve weight
  double solve_weight;
  double solve_point2line_base;

  // check lane_match
  int thre_continue_badmatch;
  double thre_delta_y_diff;

  // about debug
  bool debug_plot_info;

  // rviz
  bool rviz_show_map_centerlane;
  bool rviz_show_pceplane_oriT;

  // others
  double perceplane_len_lowerbound;

  // refpoint
  bool can_ref_point_changed;
  double thre_ref_point_change;

  // temporary parameter
  bool use_ll_perceplane;
  // 130fault
  bool use_map_lane_match_fault;
  double map_lane_match_max;
  int map_lane_match_buff;
  double map_lane_match_ser_max;
  int map_lane_match_ser_buff;
  double near_dis;
  double last_straight_dis;
  double last_curve_dis;
  double curvature_thr;
  double line_error_normal_thr = 0.05;
  // 125fault
  bool use_fc_offset_onelane_fault;
  double offset_onelane_max_err;
  double offset_onelane_cnt;
  double offset_maplane_min_dis;
  double offset_maplane_max_dis;
  double offset_maxlane;
  // 123fault
  bool use_valid_pecep_lane_fault;
  int invalid_pecep_cnt_thr;
  // 124fault
  bool use_valid_map_lane_fault;
};

extern MapMatchLaneLineParams mm_params;
extern const double EPSILION;

}  // namespace loc
}  // namespace mp
}  // namespace hozon
