/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： fault.cc
 *   author     ： luoning
 *   date       ： 2024.06
 ******************************************************************************/

#include "modules/location/pose_estimation/lib/fault/fault.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

namespace hozon {
namespace mp {
namespace loc {

void MmFault::set_ins_ts(const double& ins_ts) {
  last_ins_timestamp_ = ins_timestamp_;
  HLOG_DEBUG << " ins stamp : " << ins_ts;
  ins_timestamp_ = ins_ts;
}

void MmFault::Fault(
    const std::unordered_map<std::string, std::vector<ControlPoint>>&
        merged_fcmap_lines,
    const std::unordered_map<std::string, std::vector<ControlPoint>>&
        merged_map_edges,
    const std::list<std::list<LaneLinePerceptionPtr>>& percep_lanelines,
    const ValidPose& T_fc, bool is_big_curvature_frame, bool is_ramp_road) {
  err_type_ = ERROR_TYPE::NO_ERROR;
  static double invalid_pecep_duration = 0;
  static double invalid_map_duration = 0;
  SE3 FC_pose = T_fc.pose;
  Eigen::Vector3d FC_vel = T_fc.velocity_vrf;
  if (merged_fcmap_lines.empty()) {
    if (mm_params.use_valid_map_lane_fault && last_ins_timestamp_ != 0) {
      invalid_map_duration += fabs(ins_timestamp_ - last_ins_timestamp_);
      if (invalid_map_duration > mm_params.invalid_map_thr) {
        err_type_ = ERROR_TYPE::NO_MERGE_MAP_LANE;
        invalid_map_duration = mm_params.invalid_map_thr;
      }
    }
    HLOG_ERROR << "merged_map_lines empty!!!";
    return;
  }
  if (percep_lanelines.empty()) {
    if (mm_params.use_valid_pecep_lane_fault && last_ins_timestamp_ != 0) {
      invalid_pecep_duration += fabs(ins_timestamp_ - last_ins_timestamp_);
      if (invalid_pecep_duration > mm_params.invalid_pecep_thr) {
        err_type_ = ERROR_TYPE::NO_VALID_PECEP_LANE;
        invalid_pecep_duration = mm_params.invalid_pecep_thr;
      }
    }
    HLOG_ERROR << "percep_lanelines empty!!!";
    return;
  }
  invalid_pecep_duration = 0;
  invalid_map_duration = 0;
  is_big_curvature_ = is_big_curvature_frame;
  if (mm_params.use_map_lane_match_fault && T_fc.valid) {
    CheckIsGoodMatchFCbyLine(merged_fcmap_lines, merged_map_edges,
                             percep_lanelines, FC_pose, FC_vel, is_ramp_road);
  }
}

void MmFault::CheckIsGoodMatchFCbyLine(
    const std::unordered_map<std::string, std::vector<ControlPoint>>&
        merged_fcmap_lines,
    const std::unordered_map<std::string, std::vector<ControlPoint>>&
        merged_map_edges,
    const std::list<std::list<LaneLinePerceptionPtr>>& percep_lanelines,
    const SE3& FC_pose, const Eigen::Vector3d& FC_vel, bool is_ramp_road) {
  if (merged_fcmap_lines.empty() || merged_map_edges.empty() ||
      percep_lanelines.empty()) {
    return;
  }
  const auto& fil_line_list = percep_lanelines.back();
  if (fil_line_list.empty()) {
    return;
  }
  uint32_t percep_left_cnt = 0;
  uint32_t percep_right_cnt = 0;
  CalWidthParam left_param{0, 0, 0, 0};
  CalWidthParam right_param{0, 0, 0, 0};
  CalWidthParam left_check_param{0, 0, 0, 0};
  CalWidthParam right_check_param{0, 0, 0, 0};
  std::string map_left_near_id = "0_0";
  std::string map_left_check_near_id = "0_0";
  std::string map_right_near_id = "0_0";
  std::string map_right_check_near_id = "0_0";
  V3 percep_left_target_point(0, 0, 0);
  V3 percep_right_target_point(0, 0, 0);
  V3 percep_left_check_target_point(0, 0, 0);
  V3 percep_right_check_target_point(0, 0, 0);
  double left_dist_near_v = 0.0;
  double left_dist_far_v = 0.0;
  double right_dist_near_v = 0.0;
  double right_dist_far_v = 0.0;
  double left_near_check_dist_v = 0.0;
  double right_near_check_dist_v = 0.0;
  double left_far_check_dist_v = 0.0;
  double right_far_check_dist_v = 0.0;
  double far_dis_last = 0.0;
  for (const auto& line : fil_line_list) {
    if (line->lane_position_type() == -1) {
      ++percep_left_cnt;
    }
    if (line->lane_position_type() == 1) {
      ++percep_right_cnt;
    }
  }
  std::unordered_map<std::string, std::vector<V3>> left_filtered_fcmap_lines;
  std::unordered_map<std::string, std::vector<V3>> right_filtered_fcmap_lines;
  SE3 T_V_W = FC_pose.inverse();
  for (const auto& map_line : merged_fcmap_lines) {
    const auto line_idx = map_line.first;
    std::vector<V3> map_line_points;
    for (const auto& control_point : map_line.second) {
      V3 points = T_V_W * control_point.point;
      map_line_points.emplace_back(points);
    }
    V3 map_point(0, 0, 0);
    bool flag_fit0 = GetFcFitMapPoints(map_line_points, 0, &map_point);
    if (flag_fit0) {
      if (map_point.y() >= 0) {
        left_filtered_fcmap_lines[line_idx] = map_line_points;
      } else {
        right_filtered_fcmap_lines[line_idx] = map_line_points;
      }
      if (fabs(map_point.y()) < 0.4) {
        left_filtered_fcmap_lines[line_idx] = map_line_points;
        right_filtered_fcmap_lines[line_idx] = map_line_points;
      }
    } else {
      if (map_line_points.front().x() >= 0) {
        if (map_line_points.front().y() >= 0) {
          left_filtered_fcmap_lines[line_idx] = map_line_points;
        } else {
          right_filtered_fcmap_lines[line_idx] = map_line_points;
        }
        if (fabs(map_line_points.front().y()) < 0.3) {
          left_filtered_fcmap_lines[line_idx] = map_line_points;
          right_filtered_fcmap_lines[line_idx] = map_line_points;
        }
      } else {
        if (map_line_points.back().y() >= 0) {
          left_filtered_fcmap_lines[line_idx] = map_line_points;
        } else {
          right_filtered_fcmap_lines[line_idx] = map_line_points;
        }
      }
    }
  }
  // 当车道线和路沿有前后继关系，那么进行连接
  std::unordered_map<std::string, std::vector<V3>> left_link_edge_fcmap_lines;
  std::unordered_map<std::string, std::vector<V3>> right_link_edge_fcmap_lines;
  static double left_lane_link_edge_ins_duration = 0.0;
  static double right_lane_link_edge_ins_duration = 0.0;
  LineLinkEdgeCheck(left_filtered_fcmap_lines, merged_map_edges,
                    &left_link_edge_fcmap_lines,
                    &left_lane_link_edge_ins_duration);
  LineLinkEdgeCheck(right_filtered_fcmap_lines, merged_map_edges,
                    &right_link_edge_fcmap_lines,
                    &right_lane_link_edge_ins_duration);
  if (left_lane_link_edge_ins_duration > 0.1 &&
      left_lane_link_edge_ins_duration < mm_params.quit_link_thr) {
    left_filtered_fcmap_lines = left_link_edge_fcmap_lines;
  } else {
    left_lane_link_edge_ins_duration = 0;
  }
  if (right_lane_link_edge_ins_duration > 0.1 &&
      right_lane_link_edge_ins_duration < mm_params.quit_link_thr) {
    right_filtered_fcmap_lines = right_link_edge_fcmap_lines;
  } else {
    right_lane_link_edge_ins_duration = 0;
  }
  std::pair<std::string, std::vector<ControlPoint>> nearest_left_edge;
  std::pair<std::string, std::vector<ControlPoint>> nearest_right_edge;
  uint32_t map_edge_posi_cnt = 0;
  uint32_t map_edge_nag_cnt = 0;
  V3 map_left_point(DOUBLE_MAX, DOUBLE_MAX, DOUBLE_MAX);
  V3 map_right_point(DOUBLE_MIN, DOUBLE_MIN, DOUBLE_MIN);
  for (const auto& map_edge : merged_map_edges) {
    const auto edge_idx = map_edge.first;
    // 获取当前边界的拟合点
    V3 map_point(0, 0, 0);
    bool flag_fit0 = FaultGetFitMapPoints(
        map_edge.second, std::min(FC_vel(0) * 1.5, 30.0), &map_point);
    // 比较并更新左右边界
    if (map_point.y() > mm_params.left_edge_y_err && !is_big_curvature_) {
      ++map_edge_posi_cnt;
      if (nearest_left_edge.second.empty() ||
          (map_point.y() < map_left_point.y() && map_left_point.y() > 0)) {
        map_left_point = map_point;
        nearest_left_edge = std::make_pair(edge_idx, map_edge.second);
      }
    }
    if (map_point.y() < mm_params.right_edge_y_err && !is_big_curvature_) {
      ++map_edge_nag_cnt;
      if (nearest_right_edge.second.empty() ||
          (map_point.y() > map_right_point.y() && map_right_point.y() < 0)) {
        map_right_point = map_point;
        nearest_right_edge = std::make_pair(edge_idx, map_edge.second);
      }
    }
  }
  bool exceed_map_edge_check = false;
  if ((map_edge_posi_cnt >= 2 && map_edge_nag_cnt == 0) ||
      (map_edge_posi_cnt == 0 && map_edge_nag_cnt >= 2)) {
    exceed_map_edge_check = true;
    HLOG_ERROR << "130 : vehicle exceed map edges";
  }
  std::list<std::shared_ptr<hozon::mp::loc::PerceptionLaneLine>> line_list;
  for (const auto& line : fil_line_list) {
    V3 percep_left_point(0, 0, 0);
    V3 percep_right_point(0, 0, 0);
    std::vector<V3> perce_points;
    for (const auto& point : line->points()) {
      perce_points.emplace_back(point);
    }
    if (line->lane_position_type() == -1) {
      bool flag_fit0 = GetFcFitPoints(perce_points, 0, &percep_left_point);
      if (flag_fit0 && fabs(percep_left_point.y()) < 4) {
        line_list.emplace_back(line);
      }
    }
    if (line->lane_position_type() == 1) {
      bool flag_fit1 = GetFcFitPoints(perce_points, 0, &percep_right_point);
      if (flag_fit1 && fabs(percep_right_point.y()) < 4) {
        line_list.emplace_back(line);
      }
    }
  }
  for (const auto& line : line_list) {
    auto far_dis =
        std::min(FC_vel(0) * 2.5, static_cast<double>(line->Max()) * 0.8);
    auto near_dis =
        std::min(FC_vel(0) * 2.0, static_cast<double>(line->Max()) * 0.6);
    if (is_big_curvature_ || FC_vel.norm() < mm_params.min_vel) {
      far_dis_last = mm_params.curve_far_dis;
    } else {
      far_dis_last = mm_params.straight_far_dis;
    }
    if (line->lane_position_type() == -1) {
      CalLinesMinDist(line, left_filtered_fcmap_lines, &left_dist_near_v,
                      &left_dist_far_v, far_dis, near_dis, &map_left_near_id,
                      &percep_left_target_point, &left_param);
      CalLinesMinDist(line, left_filtered_fcmap_lines, &left_near_check_dist_v,
                      &left_far_check_dist_v, far_dis_last,
                      std::max(2.0, static_cast<double>(line->Min())),
                      &map_left_check_near_id, &percep_left_check_target_point,
                      &left_check_param);
    }
    if (line->lane_position_type() == 1) {
      CalLinesMinDist(line, right_filtered_fcmap_lines, &right_dist_near_v,
                      &right_dist_far_v, far_dis, near_dis, &map_right_near_id,
                      &percep_right_target_point, &right_param);
      CalLinesMinDist(line, right_filtered_fcmap_lines,
                      &right_near_check_dist_v, &right_far_check_dist_v,
                      far_dis_last,
                      std::max(2.0, static_cast<double>(line->Min())),
                      &map_right_check_near_id,
                      &percep_right_check_target_point, &right_check_param);
    }
  }
  const double map_near_width =
      fabs(left_param.map_near_point_y) + fabs(right_param.map_near_point_y);
  const double map_far_width =
      fabs(left_param.map_far_point_y) + fabs(right_param.map_far_point_y);
  const double percep_near_width = fabs(left_param.percep_near_point_y) +
                                   fabs(right_param.percep_near_point_y);
  const double percep_far_width = fabs(left_param.percep_far_point_y) +
                                  fabs(right_param.percep_far_point_y);
  const double map_width = (map_near_width + map_far_width) * 0.5;
  const double percep_width = (percep_near_width + percep_far_width) * 0.5;
  const double width_diff = map_width - percep_width;

  HLOG_DEBUG << "map_width = " << map_width << "percep_width = " << percep_width
             << "width_diff = " << width_diff;
  HLOG_DEBUG << "left_dist_near_v = " << left_dist_near_v
             << "left_dist_far_v = " << left_dist_far_v
             << "right_dist_near_v = " << right_dist_near_v
             << "right_dist_far_v = " << right_dist_far_v;
  HLOG_DEBUG << "left_near_check_dist_v = " << left_near_check_dist_v
             << "left_far_check_dist_v = " << left_far_check_dist_v
             << "right_near_check_dist_v = " << right_near_check_dist_v
             << "right_far_check_dist_v = " << right_far_check_dist_v;
  const double left_error = (left_dist_near_v + left_dist_far_v) * 0.5;
  const double right_error = (right_dist_near_v + right_dist_far_v) * 0.5;
  const double left_check_error =
      (left_near_check_dist_v + left_far_check_dist_v) * 0.5;
  const double right_check_error =
      (right_near_check_dist_v + right_far_check_dist_v) * 0.5;

  FaultParam faultParam{left_dist_near_v,
                        left_dist_far_v,
                        right_dist_near_v,
                        right_dist_far_v,
                        left_near_check_dist_v,
                        left_far_check_dist_v,
                        right_near_check_dist_v,
                        right_far_check_dist_v,
                        left_error,
                        right_error,
                        left_check_error,
                        right_check_error};
  auto result = FaultDetected(
      faultParam, map_right_check_near_id, percep_right_target_point,
      right_filtered_fcmap_lines, map_left_check_near_id,
      percep_left_target_point, left_filtered_fcmap_lines, FC_vel, width_diff,
      is_ramp_road, map_width);

  static uint32_t match_double_err_cnt = 0;
  static uint32_t match_width_single_err_cnt = 0;
  static uint32_t match_single_err_cnt = 0;
  static uint32_t check_exceed_edge_cnt = 0;
  static uint32_t check_good_match_cnt = 0;
  static uint32_t check_good_match_nonzero_cnt = 0;
  if (exceed_map_edge_check) {
    ++check_exceed_edge_cnt;
  } else {
    check_exceed_edge_cnt = 0;
  }
  if (!std::get<1>(result)) {
    ++match_double_err_cnt;
  } else {
    match_double_err_cnt = 0;
  }
  if (!std::get<0>(result)) {
    ++match_single_err_cnt;
  } else {
    match_single_err_cnt = 0;
  }
  if (!std::get<2>(result)) {
    ++match_width_single_err_cnt;
  } else {
    match_width_single_err_cnt = 0;
  }
  if (percep_left_cnt > 0 && percep_right_cnt > 0) {
    if (match_double_err_cnt > mm_params.map_lane_match_ser_buff ||
        match_single_err_cnt > mm_params.map_lane_match_buff ||
        match_width_single_err_cnt > mm_params.map_lane_match_buff ||
        check_exceed_edge_cnt > mm_params.map_lane_match_buff) {
      err_type_ = ERROR_TYPE::MAP_LANE_MATCH_FAIL;
      check_error_last_ = true;
    }
  }
  bool fc_good_match_check_last = false;
  bool fc_good_match_check_nonzero_last = false;
  bool good_match_check_flag = false;
  // 感知车道线没有与地图车道线匹配则差值置0
  bool good_match_check_nonzero_flag = false;
  if (check_error_last_) {
    err_type_ = ERROR_TYPE::MAP_LANE_MATCH_FAIL;
    if (fabs(left_near_check_dist_v) <= mm_params.fault_restore_dis &&
        fabs(right_near_check_dist_v) <= mm_params.fault_restore_dis &&
        fabs(left_check_error) <= mm_params.fault_restore_ave_dis &&
        fabs(right_check_error) <= mm_params.fault_restore_ave_dis &&
        !exceed_map_edge_check) {
      fc_good_match_check_last = true;
    }
    if (fabs(left_near_check_dist_v) > 0 && fabs(right_near_check_dist_v) > 0 &&
        fabs(left_check_error) > 0 && fabs(right_check_error) > 0 &&
        !exceed_map_edge_check) {
      fc_good_match_check_nonzero_last = true;
    }
    if (fc_good_match_check_last) {
      ++check_good_match_cnt;
    } else {
      check_good_match_cnt = 0;
    }
    if (fc_good_match_check_nonzero_last) {
      ++check_good_match_nonzero_cnt;
    } else {
      check_good_match_nonzero_cnt = 0;
    }
    good_match_check_flag =
        static_cast<bool>(check_good_match_cnt > mm_params.fault_restore_buff);
    good_match_check_nonzero_flag = static_cast<bool>(
        check_good_match_nonzero_cnt > mm_params.fault_restore_buff);
    if (good_match_check_flag && good_match_check_nonzero_flag) {
      check_error_last_ = false;
    }
  }
  if (!check_error_last_) {
    check_good_match_cnt = 0;
    check_good_match_nonzero_cnt = 0;
  }
}

void MmFault::LineLinkEdgeCheck(
    const std::unordered_map<std::string, std::vector<V3>>&
        filtered_fcmap_lines,
    const std::unordered_map<std::string, std::vector<ControlPoint>>&
        merged_map_edges,
    std::unordered_map<std::string, std::vector<V3>>* link_edge_fcmap_lines,
    double* lane_link_edge_ins_duration) {
  if (filtered_fcmap_lines.empty() || merged_map_edges.empty() ||
      !link_edge_fcmap_lines || !lane_link_edge_ins_duration) {
    return;
  }
  std::unordered_map<std::string, std::vector<V3>> link_edge_lines;
  for (const auto& map_line : filtered_fcmap_lines) {
    const auto line_idx = map_line.first;
    link_edge_lines[line_idx] = map_line.second;
  }
  for (auto& line : link_edge_lines) {
    for (const auto& edge : merged_map_edges) {
      std::vector<V3> map_edge_points;
      for (const auto& control_point : edge.second) {
        map_edge_points.emplace_back(control_point.point);
      }
      if (fabs(edge.second.front().point.x() - line.second.back().x()) <= 0.3 &&
          fabs(edge.second.front().point.y() - line.second.back().y()) <= 0.3) {
        // 把 map_edge_points 点加到 line 里
        *lane_link_edge_ins_duration +=
            fabs(ins_timestamp_ - last_ins_timestamp_);
        line.second.insert(line.second.end(), map_edge_points.begin(),
                           map_edge_points.end());
      }
    }
  }
  *link_edge_fcmap_lines = link_edge_lines;
}

std::tuple<bool, bool, bool> MmFault::FaultDetected(
    const FaultParam& faultParam, const std::string& map_right_check_near_id,
    const V3& percep_right_target_point,
    const std::unordered_map<std::string, std::vector<V3>>&
        right_filtered_fcmap_lines,
    const std::string& map_left_check_near_id,
    const V3& percep_left_target_point,
    const std::unordered_map<std::string, std::vector<V3>>&
        left_filtered_fcmap_lines,
    const Eigen::Vector3d& FC_vel, const double& width_diff, bool is_ramp_road,
    const double& map_width) {
  const double global_error =
      (faultParam.left_error + faultParam.right_error) * 0.5;
  const double global_near_error =
      (faultParam.left_dist_near_v + faultParam.right_dist_near_v) * 0.5;
  bool fc_good_match_single_check = true;
  bool fc_good_match_double_check = true;
  bool fc_width_good_match_single_check = true;
  if (fabs(global_error) < mm_params.line_error_normal_thr &&
      fabs(global_near_error) < mm_params.line_error_normal_thr &&
      width_diff < 0) {
    return std::tuple<bool, bool, bool>{fc_good_match_single_check,
                                        fc_good_match_double_check,
                                        fc_width_good_match_single_check};
  }
  if ((faultParam.left_dist_near_v > 0) == (faultParam.right_dist_near_v > 0) &&
      (faultParam.right_dist_near_v > 0) == (faultParam.right_dist_far_v > 0) &&
      (faultParam.right_dist_far_v > 0) == (faultParam.left_dist_far_v > 0) &&
      !is_big_curvature_ &&
      (FC_vel(0) > mm_params.double_error_min_vel ||
       FC_vel(1) > mm_params.double_error_min_vel) &&
      !is_ramp_road && map_width <= 5.0) {
    if (fabs(faultParam.left_dist_near_v) >=
            mm_params.map_lane_match_double_diff &&
        fabs(faultParam.right_dist_near_v) >=
            mm_params.map_lane_match_double_diff) {
      HLOG_ERROR << "130 : ser double near distance exceed thr";
      fc_good_match_double_check = false;
    }
    if (fabs(faultParam.left_error) >= mm_params.map_lane_match_double_diff &&
        fabs(faultParam.right_error) >= mm_params.map_lane_match_double_diff) {
      HLOG_ERROR << "130 : ser double both sides distance exceed thr";
      fc_good_match_double_check = false;
    }
  }
  if (map_width <= 5.5 && width_diff > mm_params.map_percep_width_diff) {
    if (fabs(faultParam.left_dist_near_v) >= mm_params.map_lane_match_max &&
        fabs(faultParam.right_dist_near_v) >= mm_params.map_lane_match_max) {
      HLOG_ERROR << "130 : width check double near distance exceed thr";
      fc_good_match_double_check = false;
    }
    if (fabs(faultParam.left_error) >= mm_params.map_lane_match_max &&
        fabs(faultParam.right_error) >= mm_params.map_lane_match_max) {
      HLOG_ERROR << "130 : width check double both sides distance exceed thr";
      fc_good_match_double_check = false;
    }
    if (FC_vel(0) <= mm_params.single_error_min_vel &&
        FC_vel(1) <= mm_params.single_error_min_vel) {
      return std::tuple<bool, bool, bool>{fc_good_match_single_check,
                                          fc_good_match_double_check,
                                          fc_width_good_match_single_check};
    }
    if (fabs(faultParam.left_dist_near_v) >=
            mm_params.map_lane_match_single_diff ||
        fabs(faultParam.right_dist_near_v) >=
            mm_params.map_lane_match_single_diff) {
      HLOG_ERROR << "130 : width check single near distance exceed thr";
      fc_width_good_match_single_check = false;
    }
    if (fabs(faultParam.left_error) >= mm_params.map_lane_match_single_diff ||
        fabs(faultParam.right_error) >= mm_params.map_lane_match_single_diff) {
      HLOG_ERROR << "130 : width check single both sides distance exceed thr";
      fc_width_good_match_single_check = false;
    }
  } else {
    if (fabs(faultParam.left_dist_near_v) >= mm_params.map_lane_match_ser_max &&
        fabs(faultParam.right_dist_near_v) >=
            mm_params.map_lane_match_ser_max) {
      HLOG_ERROR << "130 : double near distance exceed thr";
      fc_good_match_double_check = false;
    }
    if (fabs(faultParam.left_error) >= mm_params.map_lane_match_ser_max &&
        fabs(faultParam.right_error) >= mm_params.map_lane_match_ser_max) {
      HLOG_ERROR << "130 : double both sides distance exceed thr";
      fc_good_match_double_check = false;
    }
    if (FC_vel(0) <= mm_params.single_error_min_vel &&
        FC_vel(1) <= mm_params.single_error_min_vel) {
      return std::tuple<bool, bool, bool>{fc_good_match_single_check,
                                          fc_good_match_double_check,
                                          fc_width_good_match_single_check};
    }
    if (fabs(faultParam.left_dist_near_v) >= mm_params.map_lane_match_diver ||
        fabs(faultParam.right_dist_near_v) >= mm_params.map_lane_match_diver) {
      HLOG_ERROR << "130 : single near distance exceed thr";
      fc_good_match_single_check = false;
    }
    if (fabs(faultParam.left_error) >= mm_params.map_lane_match_diver ||
        fabs(faultParam.right_error) >= mm_params.map_lane_match_diver) {
      HLOG_ERROR << "130 : single both sides distance exceed thr";
      fc_good_match_single_check = false;
    }
  }

  if (fc_good_match_single_check && fc_width_good_match_single_check) {
    return std::tuple<bool, bool, bool>{fc_good_match_single_check,
                                        fc_good_match_double_check,
                                        fc_width_good_match_single_check};
  }
  double left_near_point_distance = DOUBLE_MAX;
  double right_near_point_distance = DOUBLE_MAX;
  if ((fabs(faultParam.left_dist_near_v) >= mm_params.map_lane_match_diver ||
       fabs(faultParam.left_error) >= mm_params.map_lane_match_diver) &&
      fabs(faultParam.left_near_check_dist_v) < mm_params.ramp_judg_thre) {
    left_near_point_distance = GetDistanceBySecondFaultDetected(
        left_filtered_fcmap_lines, map_left_check_near_id,
        percep_left_target_point, FC_vel);
  }
  if ((fabs(faultParam.right_dist_near_v) >= mm_params.map_lane_match_diver ||
       fabs(faultParam.right_error) >= mm_params.map_lane_match_diver) &&
      fabs(faultParam.right_near_check_dist_v) < mm_params.ramp_judg_thre) {
       right_near_point_distance = GetDistanceBySecondFaultDetected(
        right_filtered_fcmap_lines, map_right_check_near_id,
        percep_right_target_point, FC_vel);
  }
  if ((left_near_point_distance <= mm_params.map_lane_match_diver &&
       faultParam.right_error != 0) ||
      (right_near_point_distance <= mm_params.map_lane_match_diver &&
       faultParam.left_error != 0)) {
    fc_good_match_single_check = true;
    fc_width_good_match_single_check = true;
  }
  return std::tuple<bool, bool, bool>{fc_good_match_single_check,
                                      fc_good_match_double_check,
                                      fc_width_good_match_single_check};
}

double MmFault::GetDistanceBySecondFaultDetected(
    const std::unordered_map<std::string, std::vector<V3>>&
        filtered_fcmap_lines,
    const std::string& map_check_near_id, const V3& percep_target_point,
    const Eigen::Vector3d& FC_vel) {
  double point_distance = DOUBLE_MAX;
  V3 map_line_point(0, 0, 0);
  V3 map_line_near_point(0, 0, 0);
  for (const auto& map_line : filtered_fcmap_lines) {
    const auto line_idx = map_line.first;
    const auto map_point = map_line.second;
    int point_size = static_cast<int>(map_line.second.size());
    if (point_size < 2) {
      continue;
    }
    if (line_idx != map_check_near_id) {
      continue;
    }
    if (map_line.second.back().x() < FC_vel(0) * 2.0) {
      V3 map_point1 = map_line.second.back();
      V3 map_point2 = map_line.second[point_size - 2];
      point_distance = CalCulatePointToLineDistance(percep_target_point,
                                                    map_point1, map_point2);
    } else {
      bool flag0 =
          FaultGetFitPoints(map_point, FC_vel(0) * 1.5, &map_line_near_point);
      bool flag1 =
          FaultGetFitPoints(map_point, FC_vel(0) * 2.0, &map_line_point);
      if (flag0 && flag1) {
        point_distance = CalCulatePointToLineDistance(
            percep_target_point, map_line_point, map_line_near_point);
      }
    }
  }
  return point_distance;
}
double MmFault::CalCulatePointToLineDistance(const V3& selected_point,
                                             const V3& line_point1,
                                             const V3& line_point2) {
  if ((line_point1 - line_point2).norm() < 1e-1) {
    HLOG_DEBUG << "(n1 - n2).norm() < 1e-1 !!";
    return DBL_MAX;
  }
  V3 direction_vector;
  direction_vector.x() = line_point1.x() - line_point2.x();
  direction_vector.y() = line_point1.y() - line_point2.y();
  direction_vector.z() = line_point1.z() - line_point2.z();
  V3 selected_point_to_line_point1;
  selected_point_to_line_point1.x() = selected_point.x() - line_point1.x();
  selected_point_to_line_point1.y() = selected_point.y() - line_point1.y();
  selected_point_to_line_point1.z() = selected_point.z() - line_point1.z();
  V3 cross_product = direction_vector.cross(selected_point_to_line_point1);
  double dis = cross_product.norm() / direction_vector.norm();
  return dis;
}

void MmFault::CalLinesMinDist(
    const LaneLinePerceptionPtr& percep,
    const std::unordered_map<std::string, std::vector<V3>>&
        filtered_fcmap_lines,
    double* const near, double* const far, const double& far_dis,
    const double& near_dis, std::string* cur_line_id, V3* pt,
    CalWidthParam* param) {
  if (!percep || !near || !far || !cur_line_id || !pt || !param) {
    return;
  }
  double min_y_near = DOUBLE_MAX;
  double min_y_far = DOUBLE_MAX;
  CalWidthParam widthParam{0, 0, 0, 0};
  std::string map_line_id = "0_0";
  V3 percep_target_point(0, 0, 0);
  V3 anchor_pt0(near_dis, 0, 0);
  V3 anchor_pt1(far_dis, 0, 0);
  V3 anchor_pt2(near_dis, 0, 0);
  V3 anchor_pt3(far_dis, 0, 0);
  for (const auto& map_points : filtered_fcmap_lines) {
    std::vector<V3> perce_points;
    const auto line_idx = map_points.first;
    const auto map_point = map_points.second;
    for (auto& point : percep->points()) {
      perce_points.emplace_back(point);
    }
    V3 v_p_near(0, 0, 0);
    V3 v_p_far(0, 0, 0);
    bool flag_fit0 = FaultGetFitPoints(map_point, anchor_pt0.x(), &anchor_pt0);
    if (flag_fit0) {
      v_p_near = anchor_pt0;
    }
    bool flag_fit1 = FaultGetFitPoints(map_point, anchor_pt1.x(), &anchor_pt1);
    if (flag_fit1) {
      v_p_far = anchor_pt1;
    }
    V3 w_p_near(0, 0, 0);
    bool flag_fit2 = GetFcFitPoints(perce_points, anchor_pt2.x(), &anchor_pt2);
    if (flag_fit2) {
      w_p_near = anchor_pt2;
    }
    V3 w_p_far(0, 0, 0);
    bool flag_fit3 = GetFcFitPoints(perce_points, anchor_pt3.x(), &anchor_pt3);
    if (flag_fit3) {
      w_p_far = anchor_pt3;
    }
    double y_near = 0.0;
    double y_far = 0.0;
    if (flag_fit0 && flag_fit2) {
      y_near = w_p_near.y() - v_p_near.y();
      if (fabs(y_near) < fabs(min_y_near)) {
        min_y_near = y_near;
        map_line_id = line_idx;
        percep_target_point = w_p_near;
        widthParam.percep_near_point_y = w_p_near.y();
        widthParam.map_near_point_y = v_p_near.y();
      }
    }
    if (flag_fit1 && flag_fit3) {
      y_far = w_p_far.y() - v_p_far.y();
      if (fabs(y_far) < fabs(min_y_far)) {
        min_y_far = y_far;
        widthParam.percep_far_point_y = w_p_far.y();
        widthParam.map_far_point_y = v_p_far.y();
      }
    }
  }
  if (fabs(fabs(min_y_near) - DOUBLE_MAX) < 1e-8 || percep->Min() > 8) {
    min_y_near = 0.0;
  }
  if (fabs(fabs(min_y_far) - DOUBLE_MAX) < 1e-8) {
    min_y_far = 0.0;
  }
  *near = min_y_near;
  *far = min_y_far;
  *cur_line_id = map_line_id;
  *pt = percep_target_point;
  *param = widthParam;
}

bool MmFault::FaultGetFitPoints(const VP& points, const double x, V3* pt) {
  if (pt == nullptr) {
    return false;
  }
  if (points.size() < 2) {
    return false;
  }
  auto iter = std::lower_bound(
      points.begin(), points.end(), V3({x, 0, 0}),
      [](const V3& p0, const V3& p1) { return p0(0, 0) < p1(0, 0); });
  if (iter == points.end() || iter == points.begin()) {
    return false;
  }
  auto iter_pre = std::prev(iter);
  (*pt).x() = x;
  double ratio = (x - iter_pre->x()) / (iter->x() - iter_pre->x());
  (*pt).y() = ratio * (iter->y() - iter_pre->y()) + iter_pre->y();
  (*pt).z() = ratio * (iter->z() - iter_pre->z()) + iter_pre->z();
  return true;
}

bool MmFault::FaultGetFitMapPoints(const std::vector<ControlPoint>& points,
                                   const double x, V3* pt) {
  if (pt == nullptr) {
    HLOG_DEBUG << "pt == nullptr";
    return false;
  }
  if (points.size() < 2) {
    HLOG_DEBUG << "points.size() < 2";
    return false;
  }
  auto iter =
      std::lower_bound(points.begin(), points.end(), ControlPoint(0, {x, 0, 0}),
                       [](const ControlPoint& p0, const ControlPoint& p1) {
                         return p0.point(0, 0) < p1.point(0, 0);
                       });
  if (iter == points.end() || iter == points.begin()) {
    return false;
  }
  auto iter_pre = std::prev(iter);
  if (fabs(iter->point.x() - iter_pre->point.x()) < 1e-1) {
    if (iter_pre == points.begin()) {
      *pt = 0.5 * (iter_pre->point + iter->point);
      return true;
    } else {
      iter_pre = std::prev(iter_pre);
    }
  }
  if (fabs(iter->point.x() - iter_pre->point.x()) < 1e-1) {
    *pt = 0.5 * (iter_pre->point + iter->point);
    return true;
  }
  (*pt).x() = x;
  double ratio =
      (x - iter_pre->point.x()) / (iter->point.x() - iter_pre->point.x());
  (*pt).y() =
      ratio * (iter->point.y() - iter_pre->point.y()) + iter_pre->point.y();
  (*pt).z() =
      ratio * (iter->point.z() - iter_pre->point.z()) + iter_pre->point.z();
  return true;
}

bool MmFault::GetFcFitMapPoints(const VP& points, const double x, V3* pt) {
  if (pt == nullptr) {
    HLOG_DEBUG << "pt == nullptr";
    return false;
  }
  if (points.size() < 2) {
    HLOG_DEBUG << "points.size() < 2";
    return false;
  }
  auto iter = std::lower_bound(
      points.begin(), points.end(), V3({x, 0, 0}),
      [](const V3& p0, const V3& p1) { return p0(0, 0) < p1(0, 0); });
  if (iter == points.end() || iter == points.begin()) {
    return false;
  }
  auto iter_pre = std::prev(iter);
  if (fabs(iter->x() - iter_pre->x()) < 1e-1) {
    if (iter_pre == points.begin()) {
      (*pt).x() = 0.5 * (iter->x() + iter_pre->x());
      (*pt).y() = 0.5 * (iter->y() + iter_pre->y());
      (*pt).z() = 0.5 * (iter->z() + iter_pre->z());
      return true;
    } else {
      iter_pre = std::prev(iter_pre);
    }
  }
  if (fabs(iter->x() - iter_pre->x()) < 1e-1) {
    (*pt).x() = 0.5 * (iter->x() + iter_pre->x());
    (*pt).y() = 0.5 * (iter->y() + iter_pre->y());
    (*pt).z() = 0.5 * (iter->z() + iter_pre->z());
    return true;
  }
  (*pt).x() = x;
  double ratio = (x - iter_pre->x()) / (iter->x() - iter_pre->x());
  (*pt).y() = ratio * (iter->y() - iter_pre->y()) + iter_pre->y();
  (*pt).z() = ratio * (iter->z() - iter_pre->z()) + iter_pre->z();
  return true;
}

bool MmFault::GetFcFitPoints(const VP& control_poins, const double x, V3* pt) {
  if (pt == nullptr) {
    return false;
  }
  if (control_poins.size() < 2) {
    return false;
  }
  auto iter = std::lower_bound(
      control_poins.begin(), control_poins.end(), V3({x, 0, 0}),
      [](const V3& p0, const V3& p1) { return p0(0, 0) < p1(0, 0); });
  if (iter == control_poins.end()) {
    HLOG_DEBUG << "can not find 130 points";
    return false;
  }
  if (iter == control_poins.begin()) {
    (*pt).x() = x;
    (*pt).y() = iter->y();
    (*pt).z() = iter->z();
  } else {
    auto iter_pre = std::prev(iter);
    (*pt).x() = x;
    double ratio = (x - iter_pre->x()) / (iter->x() - iter_pre->x());
    (*pt).y() = ratio * (iter->y() - iter_pre->y()) + iter_pre->y();
    (*pt).z() = ratio * (iter->z() - iter_pre->z()) + iter_pre->z();
  }
  return true;
}

}  // namespace loc
}  // namespace mp
}  // namespace hozon
