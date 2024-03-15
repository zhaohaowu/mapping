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
  double far_dis;
  double straight_far_dis;
  double curve_far_dis;
  // double curvature_thr;
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
  double invalid_pecep_thr;
  // 124fault
  bool use_valid_map_lane_fault;
  double invalid_map_thr;
  bool use_rviz_bridge = false;
  // double line type point weight
  float double_line_point_weight;
  bool set_point_weight_switch;
  bool recur_input_in_ins_rtk_smooth = false;
  double recur_input_x_thr = 3.0;
  // filter point pairs
  double avg_diff_offset;
  int window_size;
  // check lane width
  double lane_width_diff_thre;
  bool lane_width_check_switch;
  double lane_width_diff_thre_offset;
  int max_pair_count_thre;
  int min_pair_count_thre;
  // adjust left or right pairs weight
  bool adjust_weight_by_lane_width_check;
  double min_lane_width_check_thre;
  double max_lane_width_check_thre;
  // match perception line length
  double common_max_line_length;
  double common_min_line_length;
  // offset param
  double max_length_offset;
  // lane curvature thresold
  double curvature_thresold;
  // ins deque MAX size
  int ins_deque_max_size;
};

extern MapMatchLaneLineParams mm_params;
extern const double EPSILION;

}  // namespace loc
}  // namespace mp
}  // namespace hozon
