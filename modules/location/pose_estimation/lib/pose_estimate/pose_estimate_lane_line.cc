/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： pose_estimate_lane_line.cc
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#include <algorithm>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include "modules/location/pose_estimation/lib/pose_estimate/pose_estimate_lane_line.h"
#include "modules/util/include/util/orin_trigger_manager.h"

namespace hozon {
namespace mp {
namespace loc {

MatchLaneLine::MatchLaneLine() {
  match_pairs_.reserve(20);
  line_match_pairs_.clear();
  visited_id_.clear();
  multi_linked_lines_.clear();
  merged_map_lines_.clear();
  merged_fcmap_lines_.clear();
  origin_match_pairs_.clear();
  has_err_ = false;
  err_type_ = ERROR_TYPE::NO_ERROR;
  T_W_V_ = SE3();
  T_V_W_ = SE3();
  percep_ = nullptr;
}

void MatchLaneLine::set_ins_ts(const double& ins_ts) {
  last_ins_timestamp_ = ins_timestamp_;
  ins_timestamp_ = ins_ts;
}

void MatchLaneLine::Match(const HdMap& hd_map,
                          const std::shared_ptr<Perception>& perception,
                          const SE3& T_W_V, const ValidPose& T_fc) {
  percep_ = perception;
  has_err_ = false;
  err_type_ = ERROR_TYPE::NO_ERROR;
  T_W_V_ = T_W_V;
  T_V_W_ = T_W_V.inverse();
  match_pairs_.clear();
  line_match_pairs_.clear();
  percep_lanelines_.clear();
  static int invalid_pecep_cnt = 0;
  static double invalid_pecep_duration = 0;
  static double invalid_map_duration = 0;
  auto map_elment =
      hd_map.GetElement(hozon::mp::loc::HD_MAP_LANE_BOUNDARY_LINE);
  if (map_elment == nullptr) {
    if (mm_params.use_valid_map_lane_fault && last_ins_timestamp_ != 0) {
      invalid_map_duration += fabs(ins_timestamp_ - last_ins_timestamp_);
      if (invalid_map_duration > mm_params.invalid_map_thr) {
        has_err_ = true;
        err_type_ = ERROR_TYPE::NO_VALID_MAP_LANE;
        invalid_map_duration = mm_params.invalid_map_thr;
      }
    }
    return;
  }
  std::shared_ptr<MapBoundaryLine> map_boundary_lines =
      std::static_pointer_cast<MapBoundaryLine>(map_elment);
  if (map_boundary_lines->boundary_line_.size() == 0) {
    if (mm_params.use_valid_map_lane_fault && last_ins_timestamp_ != 0) {
      invalid_map_duration += fabs(ins_timestamp_ - last_ins_timestamp_);
      if (invalid_map_duration > mm_params.invalid_map_thr) {
        has_err_ = true;
        err_type_ = ERROR_TYPE::NO_MAP_BOUNDARY_LINE;
        invalid_map_duration = mm_params.invalid_map_thr;
      }
    }
    return;
  }
  for (const auto& p : percep_->GetElement(PERCEPTYION_LANE_BOUNDARY_LINE)) {
    auto lane = std::static_pointer_cast<PerceptionLaneLineList>(p);
    std::list<LaneLinePerceptionPtr> fil_percep_laneline;
    FilterPercpLaneline(lane->lane_line_list_, &fil_percep_laneline);
    if (fil_percep_laneline.size() > 0) {
      percep_lanelines_.emplace_back(fil_percep_laneline);
    }
  }
  if (percep_lanelines_.size() == 0) {
    if (mm_params.use_valid_pecep_lane_fault && last_ins_timestamp_ != 0) {
      invalid_pecep_duration += fabs(ins_timestamp_ - last_ins_timestamp_);
      if (invalid_pecep_duration > mm_params.invalid_pecep_thr) {
        has_err_ = true;
        err_type_ = ERROR_TYPE::NO_VALID_PECEP_LANE;
        invalid_pecep_duration = mm_params.invalid_pecep_thr;
      }
    }
    return;
  }
  MergeMapLines(map_boundary_lines, T_W_V);
  if (merged_map_lines_.empty()) {
    if (mm_params.use_valid_map_lane_fault && last_ins_timestamp_ != 0) {
      invalid_map_duration += fabs(ins_timestamp_ - last_ins_timestamp_);
      if (invalid_map_duration > mm_params.invalid_map_thr) {
        has_err_ = true;
        err_type_ = ERROR_TYPE::NO_MERGE_MAP_LANE;
        invalid_map_duration = mm_params.invalid_map_thr;
      }
    }
    return;
  }
  invalid_pecep_duration = 0;
  invalid_map_duration = 0;
  LaneLineConnect(percep_lanelines_, merged_map_lines_);
  // 130 fault
  if (mm_params.use_map_lane_match_fault && T_fc.valid) {
    SE3 FC_pose = T_fc.pose;
    Eigen::Vector3d FC_vel = T_fc.velocity_vrf;
    CheckIsGoodMatchFCbyLine(FC_pose, FC_vel);
  }
  for (int i = 0; i < match_pairs_.size(); i++) {
    FilterPointPair(&match_pairs_[i], T_W_V_);
  }
  bool adjust_left_pairs_weight = false;
  bool adjust_right_pairs_weight = false;
  size_t left_pairs_count = 0;
  size_t right_pairs_count = 0;
  if (!mm_params.adjust_weight_by_lane_width_check) {
    return;
  } else {
    AdjustWeightByLaneWidth(match_pairs_, T_W_V_, &left_pairs_count,
                            &right_pairs_count, &adjust_left_pairs_weight,
                            &adjust_right_pairs_weight);
    if (left_pairs_count == 0 || right_pairs_count == 0) {
      return;
    }
    double adjust_weight =
        static_cast<double>(right_pairs_count) / left_pairs_count;
    for (auto iter = match_pairs_.begin(); iter != match_pairs_.end(); ++iter) {
      for (auto point_iter = iter->begin(); point_iter != iter->end();
           ++point_iter) {
        if ((*point_iter).pecep_pv.y() < 0) {
          if (adjust_left_pairs_weight) {
            (*point_iter).weight *= adjust_weight;
          }
        } else if ((*point_iter).pecep_pv.y() > 0) {
          if (adjust_right_pairs_weight) {
            (*point_iter).weight *= (1 / adjust_weight);
          }
        }
      }
    }
  }
}

void MatchLaneLine::CheckIsGoodMatchFCbyLine(const SE3& FC_pose,
                                             const Eigen::Vector3d& FC_vel) {
  if (percep_lanelines_.empty()) {
    return;
  }
  const auto& fil_line_list = percep_lanelines_.back();
  if (fil_line_list.empty()) {
    return;
  }
  double left_dist_near_v = 0.0;
  double left_dist_far_v = 0.0;
  double right_dist_near_v = 0.0;
  double right_dist_far_v = 0.0;
  double far_dis = 0.0;
  std::vector<std::vector<V3>> map_lines_point;
  SE3 T_V_W = FC_pose.inverse();
  for (const auto& map_line : merged_fcmap_lines_) {
    const auto line_idx = map_line.first;
    std::vector<V3> map_points;
    for (auto& control_point : map_line.second) {
      V3 cur = T_V_W * control_point.point;
      map_points.emplace_back(cur);
    }
    map_lines_point.push_back(map_points);
  }

  for (auto& line : fil_line_list) {
    if (line->lane_position_type() == -1) {
      if (big_curvature_ ||
          (FC_vel(0) < mm_params.min_vel && FC_vel(1) < mm_params.min_vel)) {
        far_dis = mm_params.curve_far_dis;
      } else {
        far_dis = mm_params.straight_far_dis;
      }
      CalLinesMinDist(line, map_lines_point, &left_dist_near_v,
                      &left_dist_far_v, far_dis);
    }
    if (line->lane_position_type() == 1) {
      if (big_curvature_ ||
          (FC_vel(0) < mm_params.min_vel && FC_vel(1) < mm_params.min_vel)) {
        far_dis = mm_params.curve_far_dis;
      } else {
        far_dis = mm_params.straight_far_dis;
      }
      CalLinesMinDist(line, map_lines_point, &right_dist_near_v,
                      &right_dist_far_v, far_dis);
    }
  }
  HLOG_DEBUG << "left_dist_near_v = " << left_dist_near_v
             << "left_dist_far_v = " << left_dist_far_v
             << "right_dist_near_v = " << right_dist_near_v
             << "right_dist_far_v = " << right_dist_far_v;
  const double left_error = (left_dist_near_v + left_dist_far_v) * 0.5;
  const double right_error = (right_dist_near_v + right_dist_far_v) * 0.5;
  const double global_error = (left_error + right_error) * 0.5;
  const double global_near_error = (left_dist_near_v + right_dist_near_v) * 0.5;

  bool fc_good_match_check = true;
  bool fc_good_match_ser_check = true;
  bool fc_good_match_diversion_check = true;
  // 针对内外八场景加的global--if判断
  if (fabs(global_error) >= mm_params.line_error_normal_thr ||
      fabs(global_near_error) >= mm_params.line_error_normal_thr) {
    // normal
    if (fabs(left_dist_near_v) >= mm_params.line_error_normal_thr &&
        fabs(right_dist_near_v) >= mm_params.line_error_normal_thr) {
      HLOG_ERROR << "130 : near distance exceed thr";
      fc_good_match_check = false;
    }
    if (fabs(left_error) >= mm_params.map_lane_match_max &&
        fabs(right_error) >= mm_params.map_lane_match_max) {
      HLOG_ERROR << "130 : both sides distance exceed thr";
      fc_good_match_check = false;
    }

    // serious
    if (fabs(left_dist_near_v) >= mm_params.map_lane_match_ser_max &&
        fabs(right_dist_near_v) >= mm_params.map_lane_match_ser_max) {
      HLOG_ERROR << "130 : near distance serious exceed thr";
      fc_good_match_ser_check = false;
    }
    if (fabs(left_error) >= mm_params.map_lane_match_ser_max &&
        fabs(right_error) >= mm_params.map_lane_match_ser_max) {
      HLOG_ERROR << "130 : both sides distance serious exceed thr";
      fc_good_match_ser_check = false;
    }

    // 针对一分二改道场景
    if (fabs(left_dist_near_v) >= mm_params.map_lane_match_diver ||
        fabs(right_dist_near_v) >= mm_params.map_lane_match_diver) {
      HLOG_ERROR << "130 : diversion near distance exceed thr";
      fc_good_match_diversion_check = false;
    }
    if (fabs(left_error) >= mm_params.map_lane_match_diver ||
        fabs(right_error) >= mm_params.map_lane_match_diver) {
      HLOG_ERROR << "130 : diversion both sides distance exceed thr";
      fc_good_match_diversion_check = false;
    }
  }

  static uint32_t match_err_cnt = 0;
  static uint32_t match_err_ser_cnt = 0;
  static uint32_t match_err_diver_cnt = 0;
  if (!fc_good_match_check) {
    ++match_err_cnt;
  } else {
    match_err_cnt = 0;
  }
  if (!fc_good_match_ser_check) {
    ++match_err_ser_cnt;
  } else {
    match_err_ser_cnt = 0;
  }
  if (!fc_good_match_diversion_check) {
    ++match_err_diver_cnt;
  } else {
    match_err_diver_cnt = 0;
  }

  if (match_err_cnt > mm_params.map_lane_match_buff ||
      match_err_ser_cnt > mm_params.map_lane_match_ser_buff ||
      match_err_diver_cnt > mm_params.map_lane_match_ser_buff) {
    err_type_ = ERROR_TYPE::MAP_LANE_MATCH_FAIL;
  }
}

void MatchLaneLine::CalLinesMinDist(
    const LaneLinePerceptionPtr& percep,
    const std::vector<std::vector<V3>>& map_lines_point, double* const near,
    double* const far, const double& far_dis) {
  if (!percep || !near || !far) {
    return;
  }
  double min_y_near = DOUBLE_MAX;
  double min_y_far = DOUBLE_MAX;
  V3 anchor_pt0(mm_params.near_dis, 0, 0);
  V3 anchor_pt1(far_dis, 0, 0);
  V3 anchor_pt2(mm_params.near_dis, 0, 0);
  V3 anchor_pt3(far_dis, 0, 0);
  for (const auto& map_points : map_lines_point) {
    std::vector<V3> perce_points;
    for (auto& point : percep->points()) {
      perce_points.emplace_back(point);
    }
    V3 v_p_near(0, 0, 0);
    V3 v_p_far(0, 0, 0);
    bool flag_fit0 = GetFcFitPoints(
        map_points, std::max(2.0, static_cast<double>(percep->Min())),
        &anchor_pt0);
    if (flag_fit0) {
      v_p_near = anchor_pt0;
    }
    bool flag_fit1 = GetFcFitPoints(map_points, anchor_pt1.x(), &anchor_pt1);
    if (flag_fit1) {
      v_p_far = anchor_pt1;
    }
    V3 w_p_near(0, 0, 0);
    bool flag_fit2 = GetFcFitPoints(
        perce_points, std::max(2.0, static_cast<double>(percep->Min())),
        &anchor_pt2);
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
    if (flag_fit0 && flag_fit2 && percep->IsIn(v_p_near.x())) {
      y_near = w_p_near.y() - v_p_near.y();
      if (fabs(y_near) < fabs(min_y_near)) {
        min_y_near = y_near;
      }
    }
    if (flag_fit1 && flag_fit3 && percep->IsIn(v_p_far.x())) {
      y_far = w_p_far.y() - v_p_far.y();
      if (fabs(y_far) < fabs(min_y_far)) {
        min_y_far = y_far;
      }
    }
  }
  if (fabs(fabs(min_y_near) - DOUBLE_MAX) < 1e-8) {
    min_y_near = 0.0;
  }
  if (fabs(fabs(min_y_far) - DOUBLE_MAX) < 1e-8) {
    min_y_far = 0.0;
  }

  *near = min_y_near;
  *far = min_y_far;
}

bool MatchLaneLine::GetFcFitPoints(const VP& control_poins, const double x,
                                   V3* pt) {
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

void MatchLaneLine::GetAverageDiffY(
    const std::vector<PointMatchPair>& match_pairs, const SE3& T,
    double* avg_diff_y) {
  if (!avg_diff_y) {
    return;
  }
  if (match_pairs.size() <= 0) {
    return;
  }
  size_t match_pairs_count = match_pairs.size();
  std::vector<PointMatchPair> cal_match_pairs(match_pairs_count);
  std::copy(match_pairs.begin(), match_pairs.end(), cal_match_pairs.begin());
  std::sort(cal_match_pairs.begin(), cal_match_pairs.end(),
            [](const PointMatchPair& pair1, const PointMatchPair& pair2) {
              return pair1.pecep_pv.x() < pair2.pecep_pv.x();
            });
  const auto window_size = (cal_match_pairs.size() > mm_params.window_size)
                               ? mm_params.window_size
                               : (cal_match_pairs.size() * 0.5);
  if (window_size <= 0) {
    return;
  }
  double sum_diff_y = 0.f;
  for (size_t i = 0; i < window_size; ++i) {
    sum_diff_y += fabs((T_V_W_ * (cal_match_pairs.at(i).map_pw)).y() -
                       cal_match_pairs.at(i).pecep_pv.y());
  }
  *avg_diff_y = (sum_diff_y / window_size);
  return;
}

void MatchLaneLine::FilterPointPair(std::vector<PointMatchPair>* match_pairs,
                                    const SE3& T) {
  if (match_pairs == nullptr) {
    return;
  }
  double avg_diff_y = 0.0;
  GetAverageDiffY(*match_pairs, T, &avg_diff_y);
  HLOG_DEBUG << "avg_diff_y = " << avg_diff_y;
  avg_diff_y += mm_params.avg_diff_offset;
  match_pairs->erase(
      std::remove_if(match_pairs->begin(), match_pairs->end(),
                     [&](const PointMatchPair& pair) {
                       return fabs((T.inverse() * pair.map_pw).y() -
                                   pair.pecep_pv.y()) > avg_diff_y;
                     }),
      match_pairs->end());
}

void MatchLaneLine::FilterPercpLaneline(
    const std::list<LaneLinePerceptionPtr>& lanelines,
    std::list<LaneLinePerceptionPtr>* const out) {
  std::unordered_map<int, std::vector<LaneLinePerceptionPtr>> per_lines;
  if (out == nullptr) {
    return;
  }
  int egolane_cnt = 0;
  for (auto& line : lanelines) {
    per_lines[line->lane_position_type()].emplace_back(line);
    HLOG_DEBUG << "percep-laneinfo "
               << line->curve_vehicle_coord_.lane_position_type_
               << " confid|min|max: " << line->curve_vehicle_coord_.confidence_
               << " " << line->Min() << " " << line->Max()
               << " stamp: " << ins_timestamp_;
    HLOG_DEBUG << "per_lines size: " << per_lines.size();
    if (per_lines[line->lane_position_type()].size() < 2) {
      continue;
    }
    HLOG_DEBUG << "same lane_pose " << line->lane_position_type();
    double l0_fist_point_x = per_lines[line->lane_position_type()][0]->Min();
    double l1_fist_point_x = per_lines[line->lane_position_type()][1]->Min();
    if (fabs(l0_fist_point_x) > fabs(l1_fist_point_x)) {
      per_lines[line->lane_position_type()].erase(
          per_lines[line->lane_position_type()].begin());
    } else {
      per_lines[line->lane_position_type()].erase(
          per_lines[line->lane_position_type()].begin() + 1);
    }
  }
  for (auto line_elem : per_lines) {
    if (line_elem.second.empty()) {
      continue;
    }
    auto line = line_elem.second[0];
    if (fabs(line->lane_position_type()) == 1) {
      egolane_cnt++;
    }
  }
  for (auto line_elem : per_lines) {
    if (line_elem.second.empty()) {
      continue;
    }
    auto line = line_elem.second[0];
    if (line->curve_vehicle_coord_.confidence_ <
        mm_params.lane_confidence_thre) {
      continue;
    }
    if (line->Max() < 0.f || line->Min() > 100.f ||
        line->Max() - line->Min() < mm_params.perceplane_len_lowerbound) {
      HLOG_DEBUG << "short-percep-lane " << line->Id() << " " << ins_timestamp_;
      continue;
    }
    if (egolane_cnt == 2) {
      if (fabs(line->lane_position_type()) == 1) {
        HLOG_DEBUG << "mod_01 line->curve_vehicle_coord_.min: "
                   << line->curve_vehicle_coord_.min_;
        (*out).emplace_back(line);
      }
    } else {
      if (fabs(line->lane_position_type()) == 1 ||
          fabs(line->lane_position_type()) == 2) {
        HLOG_DEBUG << "mod_02 line->curve_vehicle_coord_.min: "
                   << line->curve_vehicle_coord_.min_;
        (*out).emplace_back(line);
      }
    }
  }
  HLOG_DEBUG << "out size: " << out->size();
}

bool MatchLaneLine::GetNeighboringMapLines(
    const VP& target_perception_line_points,
    const std::unordered_map<std::string, std::vector<ControlPoint>>&
        map_points_cache,
    const double max_dis, const double min_dis,
    std::vector<std::pair<V3, std::string>>* nearest_line_match_pairs,
    std::vector<std::pair<V3, std::string>>* farest_line_match_pairs) {
  if (nearest_line_match_pairs == nullptr ||
      farest_line_match_pairs == nullptr ||
      target_perception_line_points.empty()) {
    return false;
  }
  V3 target_perception_line_nearest_point{0, 0, 0},
      target_perception_line_farest_point{0, 0, 0};
  bool nearest_percp_pt_flag = false, farest_percp_pt_flag = false;
  bool nearest_map_pt_flag = false, farest_map_pt_flag = false;
  if (target_perception_line_points.front().x() > min_dis) {
    target_perception_line_nearest_point =
        target_perception_line_points.front();
  } else {
    nearest_percp_pt_flag = GetFitPoints(target_perception_line_points, min_dis,
                                         &target_perception_line_nearest_point);
  }
  if (target_perception_line_points.back().x() < max_dis) {
    target_perception_line_farest_point = target_perception_line_points.back();
  } else {
    farest_percp_pt_flag = GetFitPoints(target_perception_line_points, max_dis,
                                        &target_perception_line_farest_point);
  }
  for (auto& map_lane : map_points_cache) {
    auto map_line_id = map_lane.first;
    auto map_points = map_lane.second;
    V3 map_line_nearest_point;
    V3 map_line_farest_point;
    nearest_map_pt_flag =
        GetFitMapPoints(map_points, target_perception_line_nearest_point.x(),
                        &map_line_nearest_point);
    farest_map_pt_flag =
        GetFitMapPoints(map_points, target_perception_line_farest_point.x(),
                        &map_line_farest_point);
    double nearest_delta_y = 0.f, farest_delta_y = 0.f;
    if (nearest_map_pt_flag && (map_line_nearest_point).norm() > 1e-10) {
      nearest_delta_y = fabs(target_perception_line_nearest_point.y() -
                             (map_line_nearest_point).y());
      HLOG_DEBUG << "map_line_id: " << map_line_id
                 << ", nearest_delta_y: " << nearest_delta_y;
      if (nearest_delta_y < 1.5) {
        nearest_line_match_pairs->emplace_back(
            make_pair(target_perception_line_nearest_point, map_line_id));
      }
    }
    if (farest_map_pt_flag && (map_line_farest_point).norm() > 1e-10) {
      farest_delta_y = fabs(target_perception_line_farest_point.y() -
                            (map_line_farest_point).y());
      if (farest_delta_y < 1.5) {
        farest_line_match_pairs->emplace_back(
            std::make_pair(target_perception_line_farest_point, map_line_id));
      }
    }
  }
  if (nearest_line_match_pairs->empty()) {
    HLOG_ERROR << "target_perception_line get nearest match map line failed!";
    return false;
  } else if (!(nearest_line_match_pairs->empty() &&
               farest_line_match_pairs->empty())) {
    return true;
  }
  return false;
}

void MatchLaneLine::MergeMapLines(
    const std::shared_ptr<MapBoundaryLine>& boundary_lines, const SE3& T) {
  if (!lines_map_.empty()) {
    lines_map_.clear();
  }
  auto T_V_W = T.inverse();
  for (const auto& line : boundary_lines->boundary_line_) {
    if (line.second.line_type == MapLineType::LaneChangeVirtualLine) {
      continue;
    }
    auto id = line.first;
    const auto& control_points = line.second.control_point;
    if (control_points.size() <= 1) {
      HLOG_ERROR << "control_points size <= 1";
      continue;
    }
    auto start_point = control_points.front().point;
    auto start_point_v = T_V_W * start_point;
    auto end_point = control_points.back().point;
    auto end_point_v = T_V_W * end_point;
    if (start_point.norm() == end_point.norm()) {
      HLOG_ERROR << "start_point.norm() == end_point.norm()";
      continue;
    }
    LineSegment line_segment{id, end_point_v};
    lines_map_[start_point_v].emplace_back(std::move(line_segment));
  }
  if (!visited_id_.empty()) {
    visited_id_.clear();
  }
  for (auto& line : lines_map_) {
    auto root_segment_start_point = line.first;
    std::vector<std::string> line_ids;
    if (visited_id_.count(line.second.front().first) > 0) {
      continue;
    }
    int loop = 0;
    if (!linked_lines_id_.empty()) {
      linked_lines_id_.clear();
    }
    HLOG_DEBUG << "root_id: " << line.second.front().first;
    Traversal(root_segment_start_point, line_ids, loop);
    multi_linked_lines_.emplace_back(linked_lines_id_);
  }
  if (multi_linked_lines_.empty()) {
    return;
  }
  if (!merged_map_lines_.empty()) {
    merged_map_lines_.clear();
  }
  if (!merged_fcmap_lines_.empty()) {
    merged_fcmap_lines_.clear();
  }

  std::unordered_map<std::string, int> merge_lane_ids;
  for (auto& lines : multi_linked_lines_) {
    for (auto& line_ids : lines) {
      if (line_ids.empty()) {
        continue;
      }
      auto first_line_id = line_ids.front();
      auto last_line_id = line_ids.back();
      std::string merge_lane_id = first_line_id + "_" + last_line_id;
      if (merge_lane_ids.find(merge_lane_id) != merge_lane_ids.end()) {
        merge_lane_ids[merge_lane_id] += 1;
        continue;
      }
      for (auto& line_id : line_ids) {
        for (auto control_point :
             boundary_lines->boundary_line_[line_id].control_point) {
          merged_fcmap_lines_[merge_lane_id].emplace_back(control_point);
          control_point.point = T_V_W * control_point.point;
          merged_map_lines_[merge_lane_id].emplace_back(control_point);
        }
      }
      merge_lane_ids[merge_lane_id] += 1;
    }
  }
  multi_linked_lines_.clear();
  return;
}

void MatchLaneLine::Traversal(const V3& root_start_point,
                              std::vector<std::string> line_ids, int loop) {
  if (lines_map_.empty()) {
    return;
  }
  ++loop;
  if (loop > 100) {
    linked_lines_id_.emplace_back(line_ids);
    HLOG_ERROR << "loop > 40, loop: " << loop;
    return;
  }
  V3 cur_start_point = root_start_point;
  auto cur_line_infos_iter = lines_map_.find(cur_start_point);

  if (cur_line_infos_iter == lines_map_.end()) {
    if (line_ids.empty()) {
      HLOG_ERROR << "traversal loop failed " << loop;
      return;
    }
    linked_lines_id_.emplace_back(
        line_ids);  // 一侧车道线可能会有两根链接后车道线
    return;
  }
  auto successor_segments_info = cur_line_infos_iter->second;
  int size = static_cast<int>(successor_segments_info.size());
  if (size > 0) {
    for (int i = 0; i < size; ++i) {
      bool find_flag = std::binary_search(line_ids.begin(), line_ids.end(),
                                          successor_segments_info[i].first);
      if (find_flag) {
        linked_lines_id_.emplace_back(line_ids);
        return;
      }
      line_ids.emplace_back(successor_segments_info[i].first);
      visited_id_.insert(successor_segments_info[i].first);
      Traversal(successor_segments_info[i].second, line_ids, loop);
      line_ids.pop_back();
    }
  } else {
    HLOG_ERROR << "traversal loop: " << loop << " failed!";
  }
}

void MatchLaneLine::LaneLineConnect(
    const std::list<std::list<LaneLinePerceptionPtr>>& percep_lanelines,
    const std::unordered_map<std::string, std::vector<ControlPoint>>&
        boundary_lines) {
  big_curvature_ = false;
  static const double y_dist_thres = 1.f;
  double min_match_x = mm_params.common_min_line_length;
  double max_match_x = mm_params.common_max_line_length;
  auto lane_lines = percep_->GetElement(PERCEPTYION_LANE_BOUNDARY_LINE);
  for (auto& fil_line_list : percep_lanelines) {
    if (fil_line_list.size() == 0) {
      HLOG_ERROR << "fil_line_list.size() == 0";
      return;
    }
    std::unordered_map<int, std::vector<MatchMapLine>> match_mapline_cache;
    for (auto& perception_line : fil_line_list) {
      if (perception_line == nullptr) {
        continue;
      }
      auto perception_points = perception_line->points();
      auto perception_points_size = perception_points.size();
      if (perception_points_size < 2) {
        continue;
      }
      double weight = 1.f;
      PercepLineType cur_type =
          static_cast<PercepLineType>(perception_line->lane_position_type());
      size_t map_line_type = 0;
      std::vector<std::pair<V3, std::string>> nearest_line_match_pairs;
      std::vector<std::pair<V3, std::string>> farest_line_match_pairs;
      HLOG_ERROR << "cur_type: " << perception_line->lane_position_type();
      if (GetNeighboringMapLines(perception_points, boundary_lines, max_match_x,
                                 min_match_x, &nearest_line_match_pairs,
                                 &farest_line_match_pairs)) {
        for (auto& nearest_line_match_pair : nearest_line_match_pairs) {
          auto map_line_id = nearest_line_match_pair.second;
          auto iter = boundary_lines.find(map_line_id);
          if (iter == boundary_lines.end()) {
            continue;
          }
          std::vector<ControlPoint> map_points = iter->second;
          if (map_points.empty()) {
            HLOG_ERROR << "map_points is empty";
            continue;
          }
          MatchMapLine tmp_MatchMapLine;
          tmp_MatchMapLine.map_id = map_line_id;
          tmp_MatchMapLine.flag = true;
          std::vector<double> function_coeffs;
          percep_lane_line_curve_fitting_.Fitting(perception_points, 3,
                                                  &function_coeffs);
          if (function_coeffs.size() != 4) {
            HLOG_ERROR << "Fitting failed!";
            continue;
          }
          static const double unusual_avg_curvature_thre = 0.019;
          static const double unusual_max_curvature_thre = 0.039;
          int i = 0;
          double sum_curvature = 0.f;
          double max_curvature = 0.f;
          for (int i = 0; i < perception_points_size; ++i) {
            double point_curvature = 0.f;
            ComputeCurvature(function_coeffs, perception_points[i].x(),
                             &point_curvature);
            sum_curvature += point_curvature;
            max_curvature = std::max(max_curvature, point_curvature);
          }
          double avg_curvature = sum_curvature / perception_points_size;
          if (avg_curvature > unusual_avg_curvature_thre &&
              max_curvature > unusual_max_curvature_thre) {
            big_curvature_ = true;
            max_match_x -= mm_params.max_length_offset;
            HLOG_INFO << "timestamp: " << ins_timestamp_
                      << ", big curvature!!!, max_match_x: " << max_match_x;
            #ifdef ISORIN
              // mapping trigger 大曲率弯道
              CheckTriggerBigCurv(ins_timestamp_);
            #endif
          }
          for (int i = 0; i < perception_points_size; ++i) {
            V3 pt{0, 0, 0};
            if (perception_points[i].x() < min_match_x ||
                perception_points[i].x() > max_match_x) {
              continue;
            }
            bool flag_fit = GetFitMapPoints(
                map_points,
                std::max(0.0, static_cast<double>(perception_points[i].x())),
                &pt);
            weight = cos(mm_params.solve_weight * (pt.x() - min_match_x)) /
                     (max_match_x - min_match_x);
            if (cur_type == PercepLineType::LL_LINE ||
                cur_type == PercepLineType::RR_LINE) {
              weight *= 0.75;
            }
            if (flag_fit) {
              if (map_line_type) {
                weight *= mm_params.double_line_point_weight;
              }
              tmp_MatchMapLine.match_pairs.emplace_back(PointMatchPair(
                  T_W_V_ * pt, perception_points[i], cur_type, weight));
            }
          }
          match_mapline_cache[perception_line->lane_position_type()]
              .emplace_back(tmp_MatchMapLine);
          max_match_x = mm_params.common_max_line_length;
        }
      }
    }

    // 去除关联到两根地图车道线的感知车道线
    for (auto& tmp_match_mapline : match_mapline_cache) {
      int lanepose = tmp_match_mapline.first;  // perception line
      auto& candidate_match_lines = tmp_match_mapline.second;  // map lines
      HLOG_ERROR << "lanepose: " << lanepose << " ,candidate_match_lines size: "
                 << candidate_match_lines.size();
      if (candidate_match_lines.size() >= 2) {
        int best_map_id = -1;
        double min_diff = std::numeric_limits<double>::max();
        std::vector<std::pair<std::string, double>> point_diff_cache;
        for (int i = 0; i < candidate_match_lines.size(); ++i) {
          auto& candidate_match_line = candidate_match_lines[i];
          auto& match_pairs = candidate_match_line.match_pairs;
          double match_pair_diff = 0.f;
          for (auto& match_pair : match_pairs) {
            match_pair_diff +=
                match_pair.pecep_pv.y() - (T_V_W_ * match_pair.map_pw).y();
          }
          match_pair_diff = fabs(match_pair_diff);
          match_pair_diff /= match_pairs.size();
          point_diff_cache.push_back(std::pair<std::string, double>(
              candidate_match_line.map_id, match_pair_diff));
        }
        std::sort(point_diff_cache.begin(), point_diff_cache.end(),
                  [](const std::pair<std::string, double>& l0,
                     const std::pair<std::string, double>& l1) {
                    return l0.second < l1.second;
                  });
        if (point_diff_cache.size() >= 2 &&
            point_diff_cache[1].second - point_diff_cache[0].second > 2.0) {
          // 差别很大，直接按距离过滤
          HLOG_ERROR << "mod == 1";
          for (int j = 0; j < candidate_match_lines.size(); ++j) {
            if (candidate_match_lines[j].map_id != point_diff_cache[0].first) {
              candidate_match_lines[j].flag = false;
            }
          }
        } else {
          // 差别不大,查找参考线按照宽度过滤
          HLOG_ERROR << "mod == 2";
          int select_lanepose = -99;
          std::vector<std::pair<int, double>> distance_ref_curline;
          for (auto& line_match : match_mapline_cache) {
            if (line_match.first == tmp_match_mapline.first ||
                line_match.second.size() >= 2 || line_match.second.empty() ||
                tmp_match_mapline.second.empty()) {
              continue;
            }
            if (!line_match.second[0].match_pairs.empty() &&
                !tmp_match_mapline.second[0].match_pairs.empty()) {
              double distance =
                  line_match.second[0].match_pairs[0].pecep_pv.y() -
                  tmp_match_mapline.second[0].match_pairs[0].pecep_pv.y();
              distance = fabs(distance);
              distance_ref_curline.push_back(
                  std::pair<int, double>(line_match.first, distance));
            }
          }
          if (distance_ref_curline.size() > 0) {
            std::sort(distance_ref_curline.begin(), distance_ref_curline.end(),
                      [](const std::pair<int, double>& l0,
                         const std::pair<int, double>& l1) {
                        return l0.second < l1.second;
                      });
            select_lanepose = distance_ref_curline[0].first;
          }
          if (select_lanepose != -99) {
            // 找到参考线
            auto select_match_line = match_mapline_cache[select_lanepose]
                                         .front();  // 参考感知车道线一侧匹配对
            double last_width_diff = std::numeric_limits<double>::max();
            int best_index = -1;
            for (int k = 0; k < candidate_match_lines.size(); ++k) {
              auto& candidate_match_line = candidate_match_lines[k];
              std::pair<double, double> result =
                  CmpWidth(candidate_match_line,
                           select_match_line);  // first: percep; second: map
              auto width_diff = fabs(result.first - result.second);
              HLOG_DEBUG << "candidate_match_lines[" << k
                         << "] width_diff: " << width_diff;
              if (width_diff < last_width_diff) {
                last_width_diff = width_diff;
                best_index = k;
              }
            }
            for (int m = 0; m < candidate_match_lines.size(); ++m) {
              if (m != best_index) {
                candidate_match_lines[m].flag = false;
              }
            }
          } else {
            // 未找到参考线, 找最近的
            HLOG_ERROR << "not find ref line";
            for (int n = 0; n < candidate_match_lines.size(); ++n) {
              if (candidate_match_lines[n].map_id !=
                  point_diff_cache[0].first) {
                candidate_match_lines[n].flag = false;
              }
            }
          }
        }
      }
      for (int i = 0; i < candidate_match_lines.size(); ++i) {
        std::vector<PointMatchPair> line_pair_points;
        if (!candidate_match_lines[i].flag) {
          continue;
        }
        for (auto& match_pair : candidate_match_lines[i].match_pairs) {
          auto map_v = T_V_W_ * match_pair.map_pw;
          HLOG_DEBUG << "timestamp," << ins_timestamp_ << ","
                     << "after matched map point," << map_v.x() << ","
                     << map_v.y() << "," << map_v.z();
          HLOG_DEBUG << "timestamp," << ins_timestamp_ << ","
                     << "after matched percep point," << match_pair.pecep_pv.x()
                     << "," << match_pair.pecep_pv.y() << ","
                     << match_pair.pecep_pv.z();
          if (candidate_match_lines.size() >= 2) {
            match_pair.weight *= 0.3;
          }
          line_pair_points.emplace_back(match_pair);
        }
        match_pairs_.push_back(line_pair_points);
        HLOG_ERROR << "LaneLineConnect | percep line: " << lanepose
                   << ", map line: " << candidate_match_lines[i].map_id;
      }
    }
    origin_match_pairs_ = match_pairs_;
  }
  HLOG_DEBUG << "match_pairs_ size: " << match_pairs_.size();
}

void MatchLaneLine::ComputeCurvature(const std::vector<double>& coeffs,
                                     const double x, double* curvature) {
  if (curvature == NULL || coeffs.size() != 4) {
    HLOG_ERROR << "ComputeCurvature failed!";
    return;
  }
  double c_0 = coeffs[0];
  double c_1 = coeffs[1];
  double c_2 = coeffs[2];
  double c_3 = coeffs[3];
  auto second_derivative = 2 * c_2 + 6 * c_3 * x;
  auto first_derivative = c_1 + 2 * c_2 * x + 3 * c_3 * x * x;
  double a = 1 + std::pow(first_derivative, 2);
  *curvature = fabs(second_derivative) / std::pow(a, (3.0 / 2.0));
}

void MatchLaneLine::CheckTriggerBigCurv(double cur_time) {
  static bool enable_11 = true;
  static double last_time_11 = ins_timestamp_;
  if (enable_11) {
    HLOG_ERROR << "Start to trigger dc 1011";
    GLOBAL_DC_TRIGGER.TriggerCollect(1011);
    enable_11 = false;
    last_time_11 = ins_timestamp_;
  }
  enable_11 = (ins_timestamp_ - last_time_11) > 600;
}

std::pair<double, double> MatchLaneLine::CmpWidth(
    const MatchMapLine& match_pair0, const MatchMapLine& match_pair1) {
  double width_perception = 0.0, width_map = 0.0;
  auto match_pairs0 = match_pair0.match_pairs;
  auto match_pairs1 = match_pair1.match_pairs;
  if (match_pairs0.empty() || match_pairs1.empty()) {
    return std::pair<double, double>(0.f, 0.f);
  }
  std::sort(match_pairs0.begin(), match_pairs0.end(),
            [](const PointMatchPair& pair1, const PointMatchPair& pair2) {
              return pair1.pecep_pv.x() < pair2.pecep_pv.x();
            });
  std::sort(match_pairs1.begin(), match_pairs1.end(),
            [](const PointMatchPair& pair1, const PointMatchPair& pair2) {
              return pair1.pecep_pv.x() < pair2.pecep_pv.x();
            });

  auto min_size = match_pairs0.size() > match_pairs1.size()
                      ? match_pairs1.size()
                      : match_pairs0.size();
  for (int i = 0; i < min_size; ++i) {
    width_perception +=
        fabs(match_pairs0[i].pecep_pv.y() - match_pairs1[i].pecep_pv.y());
    width_map += fabs((T_V_W_ * match_pairs0[i].map_pw).y() -
                      (T_V_W_ * match_pairs1[i].map_pw).y());
  }
  width_map = width_map / static_cast<double>(min_size);
  width_perception = width_perception / static_cast<double>(min_size);
  return std::pair<double, double>(width_perception, width_map);
}

bool MatchLaneLine::GetFitPoints(const VP& points, const double x, V3* pt) {
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
    HLOG_ERROR << "can not find percep point";
    return false;
  }
  auto iter_pre = std::prev(iter);
  (*pt).x() = x;
  double ratio = (x - iter_pre->x()) / (iter->x() - iter_pre->x());
  (*pt).y() = ratio * (iter->y() - iter_pre->y()) + iter_pre->y();
  (*pt).z() = ratio * (iter->z() - iter_pre->z()) + iter_pre->z();
  return true;
}

bool MatchLaneLine::GetFitMapPoints(const std::vector<ControlPoint>& points,
                                    const double x, V3* pt) {
  if (pt == nullptr) {
    HLOG_ERROR << "pt == nullptr";
    return false;
  }
  if (points.size() < 2) {
    HLOG_ERROR << "points.size() < 2";
    return false;
  }
  auto iter =
      std::lower_bound(points.begin(), points.end(), ControlPoint(0, {x, 0, 0}),
                       [](const ControlPoint& p0, const ControlPoint& p1) {
                         return p0.point(0, 0) < p1.point(0, 0);
                       });
  if (iter == points.end() || iter == points.begin()) {
    HLOG_ERROR << "can not find map point";
    return false;
  }
  auto iter_pre = std::prev(iter);
  if (fabs(iter->point.x() - iter_pre->point.x()) < 1e-1) {
    if (iter_pre == points.begin()) {
      *pt = 0.5 * (iter_pre->point + iter->point);
      HLOG_ERROR << "iter_pre == points.begin()";
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

bool MatchLaneLine::CompareLaneWidth(const SE3& T) {
  int size = match_pairs_.size();
  if (size != 2) {
    return false;
  }
  double percep_lane_width = 0;
  double map_lane_width = 0;
  std::vector<PointMatchPair> match_pairs_0 = match_pairs_[0];
  std::vector<PointMatchPair> match_pairs_1 = match_pairs_[1];
  std::sort(match_pairs_0.begin(), match_pairs_0.end(), SortPairByX);
  std::sort(match_pairs_1.begin(), match_pairs_1.end(), SortPairByX);
  ComputeLaneWidth(match_pairs_0, match_pairs_1, T, &percep_lane_width,
                   &map_lane_width);
  const double lane_width_diff = fabs(percep_lane_width - map_lane_width);
  double lane_width_diff_thre = mm_params.lane_width_diff_thre;
  if (is_double_line_) {
    lane_width_diff_thre += mm_params.lane_width_diff_thre_offset;
  }
  return lane_width_diff >= lane_width_diff_thre;
}

void MatchLaneLine::AdjustWeightByLaneWidth(
    const std::vector<std::vector<PointMatchPair>>& match_pairs, const SE3& T,
    size_t* left_pairs_count, size_t* right_pairs_count,
    bool* adjust_left_pairs_weight, bool* adjust_right_pairs_weight) {
  if (adjust_left_pairs_weight == nullptr ||
      adjust_right_pairs_weight == nullptr || left_pairs_count == nullptr ||
      right_pairs_count == nullptr) {
    return;
  }
  std::vector<PointMatchPair> left_match_pairs;
  std::vector<PointMatchPair> right_match_pairs;
  double percep_lane_width = 0.f, map_lane_width = 0.f, lane_width_diff = 0.f;
  for (const auto& line_match_pairs : match_pairs) {
    for (const auto& match_point_pair : line_match_pairs) {
      const auto map_point = match_point_pair.map_pw;
      const auto percep_point = match_point_pair.pecep_pv;
      if ((T.inverse() * map_point).y() < 0 && percep_point.y() < 0) {
        left_match_pairs.emplace_back(match_point_pair);
      } else if ((T.inverse() * map_point).y() > 0 && percep_point.y() > 0) {
        right_match_pairs.emplace_back(match_point_pair);
      } else {
        continue;
      }
    }
  }
  *left_pairs_count = left_match_pairs.size();
  *right_pairs_count = right_match_pairs.size();
  if (*left_pairs_count < 10 || *right_pairs_count < 10) {
    return;
  }
  std::sort(left_match_pairs.begin(), left_match_pairs.end(), SortPairByX);
  std::sort(right_match_pairs.begin(), right_match_pairs.end(), SortPairByX);
  ComputeLaneWidth(left_match_pairs, right_match_pairs, T, &percep_lane_width,
                   &map_lane_width);
  lane_width_diff = fabs(percep_lane_width - map_lane_width);
  if (lane_width_diff > mm_params.min_lane_width_check_thre &&
      lane_width_diff <= mm_params.max_lane_width_check_thre) {
    if (*left_pairs_count <= *right_pairs_count) {
      *adjust_left_pairs_weight = true;
    } else if (*left_pairs_count > *right_pairs_count) {
      *adjust_right_pairs_weight = true;
    } else {
      return;
    }
  }
  return;
}

void MatchLaneLine::ComputeLaneWidth(
    const std::vector<PointMatchPair>& left_match_pairs,
    const std::vector<PointMatchPair>& right_match_pairs, const SE3& T,
    double* percep_lane_width, double* map_lane_width) {
  if (percep_lane_width == nullptr || map_lane_width == nullptr) {
    return;
  }
  if (left_match_pairs.empty() || right_match_pairs.empty()) {
    return;
  }
  size_t count = 0;
  double sum_percep_lane_width = 0.f, sum_map_lane_width = 0.f;
  for (size_t i = 0; i < left_match_pairs.size(); ++i) {
    auto left_match_pair = left_match_pairs.at(i);
    const auto left_percep_point_x = left_match_pair.pecep_pv.x();
    left_match_pair.pecep_pv.x() -= 1e-1;
    auto right_iter =
        std::lower_bound(right_match_pairs.begin(), right_match_pairs.end(),
                         left_match_pair, SortPairByX);
    if (right_iter == right_match_pairs.end()) {
      continue;
    }
    if (fabs(left_percep_point_x - right_iter->pecep_pv.x()) > 0.49) {
      continue;
    }
    count++;
    const auto left_percep_point_y = left_match_pair.pecep_pv.y();
    const auto left_map_point_y = (T.inverse() * left_match_pair.map_pw).y();
    const auto right_match_pair = *right_iter;
    const auto right_percep_point_y = right_match_pair.pecep_pv.y();
    const auto right_map_point_y = (T.inverse() * right_match_pair.map_pw).y();
    sum_percep_lane_width += fabs(left_percep_point_y - right_percep_point_y);
    sum_map_lane_width += fabs(left_map_point_y - right_map_point_y);
    if (count >= mm_params.max_pair_count_thre) {
      break;
    }
  }
  if (count < mm_params.min_pair_count_thre) {
    return;
  }
  *percep_lane_width = sum_percep_lane_width / (count + 1e-1);
  *map_lane_width = sum_map_lane_width / (count + 1e-1);
  return;
}

VP MatchLaneLine::SetRvizMergeMapLines() {
  VP control_ponits;
  for (auto& merged_map_line : merged_map_lines_) {
    auto& control_ponits_vec = merged_map_line.second;
    for (auto iter = control_ponits_vec.begin();
         iter != control_ponits_vec.end(); ++iter) {
      control_ponits.emplace_back(T_W_V_ * (*iter).point);
    }
  }
  return control_ponits;
}

}  // namespace loc
}  // namespace mp
}  // namespace hozon
