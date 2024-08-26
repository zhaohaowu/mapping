/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： pose_estimation.cc
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/location/pose_estimation/lib/map_matching.h"

#include <algorithm>
#include <cfloat>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stack>
#include <string>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Geometry/Quaternion.h"
#include "base/utils/log.h"
#include "modules/location/pose_estimation/lib/hd_map/hd_map_base.h"
#include "modules/location/pose_estimation/lib/perception/perception.h"
#include "modules/location/pose_estimation/lib/pose_estimate/pose_estimate_solver.h"
#include "modules/location/pose_estimation/lib/util/globals.h"
#include "modules/util/include/util/rviz_agent/rviz_agent.h"
namespace hozon {
namespace mp {
namespace loc {

MapMatching::MapMatching() {
  visited_id_.clear();
  edge_visited_id_.clear();
  multi_linked_edges_.clear();
  multi_linked_lines_.clear();
  map_match_ = std::make_shared<hozon::mp::loc::MapMatch>();
  mm_fault_ = std::make_shared<hozon::mp::loc::MmFault>();
}

bool MapMatching::Init(const std::string& config_file) {
  YAML::Node config = YAML::LoadFile(config_file);

  mm_params.use_map_lane_match_fault =
      config["use_map_lane_match_fault"].as<bool>();
  mm_params.min_vel = config["min_vel"].as<double>();
  mm_params.single_error_min_vel = config["single_error_min_vel"].as<double>();
  mm_params.double_error_min_vel = config["double_error_min_vel"].as<double>();
  mm_params.map_lane_match_diver = config["map_lane_match_diver"].as<double>();
  mm_params.ramp_judg_thre = config["ramp_judg_thre"].as<double>();
  mm_params.fault_restore_dis = config["fault_restore_dis"].as<double>();
  mm_params.left_edge_y_err = config["left_edge_y_err"].as<double>();
  mm_params.right_edge_y_err = config["right_edge_y_err"].as<double>();
  mm_params.fault_restore_buff = config["fault_restore_buff"].as<int>();
  mm_params.map_lane_match_buff = config["map_lane_match_buff"].as<int>();
  mm_params.map_lane_match_ser_max =
      config["map_lane_match_ser_max"].as<double>();
  mm_params.map_lane_match_max = config["map_lane_match_max"].as<double>();
  mm_params.map_lane_match_single_diff =
      config["map_lane_match_single_diff"].as<double>();
  mm_params.map_lane_match_double_diff =
      config["map_lane_match_double_diff"].as<double>();
  mm_params.map_percep_width_diff =
      config["map_percep_width_diff"].as<double>();
  mm_params.map_lane_match_ser_buff =
      config["map_lane_match_ser_buff"].as<int>();
  mm_params.fault_restore_ave_dis =
      config["fault_restore_ave_dis"].as<double>();
  mm_params.straight_far_dis = config["straight_far_dis"].as<double>();
  mm_params.curve_far_dis = config["curve_far_dis"].as<double>();
  // mm_params.curvature_thr = config["curvature_thr"].as<double>();
  mm_params.use_fc_offset_onelane_fault =
      config["use_fc_offset_onelane_fault"].as<bool>();
  mm_params.offset_onelane_max_err =
      config["offset_onelane_max_err"].as<double>();
  mm_params.double_line_point_weight =
      config["double_line_point_weight"].as<float>();
  mm_params.set_point_weight_switch =
      config["set_point_weight_switch"].as<bool>();
  mm_params.offset_onelane_cnt = config["offset_onelane_cnt"].as<double>();
  mm_params.offset_maplane_min_dis =
      config["offset_maplane_min_dis"].as<double>();
  mm_params.offset_maplane_max_dis =
      config["offset_maplane_max_dis"].as<double>();
  mm_params.offset_maxlane = config["offset_maxlane"].as<double>();
  mm_params.use_valid_pecep_lane_fault =
      config["use_valid_pecep_lane_fault"].as<bool>();
  mm_params.use_valid_map_lane_fault =
      config["use_valid_map_lane_fault"].as<bool>();
  mm_params.invalid_pecep_thr = config["invalid_pecep_thr"].as<double>();
  mm_params.invalid_map_thr = config["invalid_map_thr"].as<double>();

  // global params
  mm_params.map_distance = config["map_distance"].as<double>();
  mm_params.lane_confidence_thre = config["lane_confidence_thre"].as<double>();
  mm_params.solve_weight = config["solve_weight"].as<double>();
  mm_params.solve_point2line_base =
      config["solve_point2line_base"].as<double>();
  mm_params.thre_continue_badmatch = config["thre_continue_badmatch"].as<int>();
  mm_params.thre_delta_y_diff = config["thre_delta_y_diff"].as<double>();
  mm_params.perceplane_len_lowerbound =
      config["perceplane_len_lowerbound"].as<double>();
  mm_params.lane_width_diff_thre_offset =
      config["lane_width_diff_thre_offset"].as<double>();
  mm_params.avg_diff_offset = config["avg_diff_offset"].as<double>();
  mm_params.window_size = config["window_size"].as<int>();
  mm_params.max_pair_count_thre = config["max_pair_count_thre"].as<int>();
  mm_params.min_pair_count_thre = config["min_pair_count_thre"].as<int>();
  mm_params.lane_width_diff_thre = config["lane_width_diff_thre"].as<double>();
  mm_params.lane_width_check_switch =
      config["lane_width_check_switch"].as<bool>();
  mm_params.adjust_weight_by_lane_width_check =
      config["adjust_weight_by_lane_width_check"].as<bool>();
  mm_params.min_lane_width_check_thre =
      config["min_lane_width_check_thre"].as<double>();
  mm_params.max_lane_width_check_thre =
      config["max_lane_width_check_thre"].as<double>();
  mm_params.debug_plot_info = config["debug_plot_info"].as<bool>();
  mm_params.rviz_show_map_centerlane =
      config["rviz_show_map_centerlane"].as<bool>();
  mm_params.rviz_show_pceplane_oriT =
      config["rviz_show_pceplane_oriT"].as<bool>();
  mm_params.can_ref_point_changed = config["can_ref_point_changed"].as<bool>();
  mm_params.thre_ref_point_change =
      config["thre_ref_point_change"].as<double>();
  mm_params.use_ll_perceplane = config["use_ll_perceplane"].as<bool>();
  mm_params.line_error_normal_thr =
      config["line_error_normal_thr"].as<double>();
  mm_params.common_max_line_length =
      config["common_max_line_length"].as<double>();
  mm_params.common_min_line_length =
      config["common_min_line_length"].as<double>();
  mm_params.max_length_offset = config["max_length_offset"].as<double>();
  mm_params.curvature_thresold = config["curvature_thresold"].as<double>();
  mm_params.lane_control_pointInfo_size =
      config["lane_control_pointInfo_size"].as<int>();

  return true;
}

void MapMatching::ProcData(
    bool use_rviz, const SE3& T_input,
    const std::shared_ptr<hozon::localization::Localization>& fc,
    const hozon::perception::TransportElement& perception,
    const std::vector<hozon::hdmap::LaneInfoConstPtr>& lanes,
    const Eigen::Vector3d& ref_point, double ins_height, int sys_status) {
  // 地图数据处理
  auto hdmap = SetHdMap(lanes, ref_point);

  // 感知车道线获取
  std::shared_ptr<Perception> all_perception = std::make_shared<Perception>();
  all_perception->Set(perception);

  // 定位输入数据获取
  double cur_stamp = fc->header().data_stamp();
  int loc_state = static_cast<int>(fc->location_state());
  Eigen::Vector3d linear_vel(fc->pose().linear_velocity_vrf().x(),
                             fc->pose().linear_velocity_vrf().y(),
                             fc->pose().linear_velocity_vrf().z());
  ValidPose fc_msg;
  fc_msg.valid = true;
  fc_msg.timeStamp = cur_stamp;
  fc_msg.velocity_vrf = linear_vel;
  fc_msg.fc_loc_state = loc_state;
  Eigen::Quaterniond fc_q(
      fc->pose().quaternion().w(), fc->pose().quaternion().x(),
      fc->pose().quaternion().y(), fc->pose().quaternion().z());
  Eigen::Vector3d fc_t(fc->pose().position().x(), fc->pose().position().y(),
                       fc->pose().position().z());
  fc_msg.pose = SE3(fc_q, fc_t);

  // 过滤感知车道线
  std::list<std::list<LaneLinePerceptionPtr>> percep_lanelines;
  FilterPercpLane(all_perception, &percep_lanelines);
  // 合并地图车道线
  std::unordered_map<std::string, std::vector<ControlPoint>> merged_map_lines;
  std::unordered_map<std::string, std::vector<ControlPoint>> merged_fcmap_lines;
  std::unordered_map<std::string, std::vector<ControlPoint>> merged_map_edges;
  MergeMapLanes(hdmap, T_input, fc_msg, &merged_map_lines, &merged_fcmap_lines,
                &merged_map_edges);

  // 获取匹配对
  map_match_->SetInsTs(cur_stamp);
  map_match_->SetVel(linear_vel);
  map_match_->Match(merged_map_lines, percep_lanelines, T_input, fc_msg);
  bool big_curvature_frame = map_match_->GetMatchBigCurvature();

  // 故障检测
  hozon::common::PointENU utm_pos;
  utm_pos.set_x(fc->pose().pos_utm_01().x());
  utm_pos.set_y(fc->pose().pos_utm_01().y());
  utm_pos.set_z(0);
  bool is_ramp_road = CheckIsRampRoad(utm_pos);
  mm_fault_->set_ins_ts(cur_stamp);
  mm_fault_->Fault(merged_fcmap_lines, merged_map_edges, percep_lanelines,
                   fc_msg, big_curvature_frame, is_ramp_road);

  // 故障检测策略
  ERROR_TYPE mm_err_type{static_cast<ERROR_TYPE>(mm_fault_->GetErrorType())};
  switch (mm_err_type) {
    case ERROR_TYPE::NO_ERROR:
      mmfault_.pecep_lane_error = false;
      mmfault_.map_lane_error = false;
      mmfault_.map_lane_match_error = false;
      mmfault_.fc_offset_onelane_error = false;
      break;
    case ERROR_TYPE::NO_VALID_PECEP_LANE:
      mmfault_.pecep_lane_error = true;
      break;
    case ERROR_TYPE::NO_VALID_MAP_LANE:
    case ERROR_TYPE::NO_MAP_BOUNDARY_LINE:
    case ERROR_TYPE::NO_MERGE_MAP_LANE:
      mmfault_.map_lane_error = true;
      break;
    case ERROR_TYPE::MAP_LANE_MATCH_FAIL:
      mmfault_.map_lane_match_error = true;
      break;
    case ERROR_TYPE::OFFSET_ONELANE:
      mmfault_.fc_offset_onelane_error = true;
      break;
    default:
      break;
  }

  // ceres优化
  SE3 T_output;
  hozon::mp::loc::Connect connect = map_match_->Result();
  double line_number_factor = 0.0;
  int default_valid_number = 2;
  bool output_valid = true;
  if (!MapMatchSolver::solve2D(connect, T_input, &T_output)) {
    HLOG_ERROR << "cur_stamp " << cur_stamp << " ceres solve failed";
    output_valid = false;
  } else {
    int matching_cache_number = map_match_->GetMatchCacheNumber();
    line_number_factor = std::fabs(
        1 - 1.0 / ((std::min(default_valid_number, matching_cache_number) /
                    static_cast<double>(default_valid_number))));
  }
  // 使用输入T_input的纵向
  Eigen::Matrix4d T_diff = Eigen::Matrix4d::Identity();
  T_diff(0, 3) = (T_output.inverse() * T_input).translation().x();
  T_output = T_output * Sophus::SE3d(T_diff);

  // 感知和地图宽度校验
  double lane_width_diff_value = 0.0;
  double lane_width_check_factor = 0.0;
  if (mm_params.lane_width_check_switch) {
    bool check_failed =
        map_match_->CheckLaneWidth(T_output, &lane_width_diff_value);
    if (check_failed) {
      output_valid = false;
      HLOG_ERROR << "cur_stamp" << cur_stamp
                 << ", lane_width_check failed, lane_width_diff_value: "
                 << lane_width_diff_value;
    } else if (!check_failed &&
               lane_width_diff_value < mm_params.lane_width_diff_thre &&
               lane_width_diff_value > 0.8) {
      lane_width_check_factor = std::exp(lane_width_diff_value - 0.8);
      output_valid = true;
    }
  }
  double a = 1.5;
  double b = 10;
  double final_coeff =
      1 + a * lane_width_check_factor +
      b * line_number_factor * std::pow(2, 1 + line_number_factor);
  // 对无效位姿增加无效标记并赋为输入位姿，有效位姿增加有效标记并赋为输出位姿
  auto cur_sec = static_cast<uint64_t>(cur_stamp);
  auto cur_nsec =
      static_cast<uint64_t>((cur_stamp - static_cast<double>(cur_sec)) * 1e9);
  mm_output_lck_.lock();
  if (!output_valid) {
    mm_node_info_ =
        generateNodeInfo(T_output, 0, 0, true, ref_point, ins_height,
                         sys_status, big_curvature_frame, final_coeff);
  } else {
    mm_node_info_ = generateNodeInfo(T_output, cur_sec, cur_nsec, false,
                                     ref_point, ins_height, sys_status,
                                     big_curvature_frame, final_coeff);
  }
  mm_output_lck_.unlock();

  // 可视化
  if (RVIZ_AGENT.Ok() && use_rviz) {
    timespec cur_time{};
    clock_gettime(CLOCK_REALTIME, &cur_time);
    auto sec = cur_time.tv_sec;
    auto nsec = cur_time.tv_nsec;
    VP rviz_merged_map_lines = SetRvizMergeMapLines(merged_map_lines, T_input);
    RvizFunc(sec, nsec, connect, T_output, rviz_merged_map_lines);
  }
}

bool MapMatching::CheckIsRampRoad(const hozon::common::PointENU& utm_pos) {
  double nearest_s = 0.0;
  double nearest_l = 0.0;
  hozon::hdmap::LaneInfoConstPtr ego_lane_ptr = nullptr;
  if (!GLOBAL_HD_MAP->GetNearestLane(utm_pos, &ego_lane_ptr, &nearest_s,
                                     &nearest_l) ||
      ego_lane_ptr == nullptr) {
    return false;
  }
  if (ego_lane_ptr->IsRampRoad()) {
    return true;
  }
  return false;
}

VP MapMatching::SetRvizMergeMapLines(
    std::unordered_map<std::string, std::vector<ControlPoint>> merged_map_lines,
    const SE3& T02_W_V) {
  VP control_ponits;
  for (auto& merged_map_line : merged_map_lines) {
    auto& control_ponits_vec = merged_map_line.second;
    for (auto iter = control_ponits_vec.begin();
         iter != control_ponits_vec.end(); ++iter) {
      control_ponits.emplace_back(T02_W_V * (*iter).point);
    }
  }
  return control_ponits;
}

void MapMatching::MergeMapLanes(
    const HdMap& hd_map, const SE3& T_W_V, const ValidPose& T_fc,
    std::unordered_map<std::string, std::vector<ControlPoint>>* const
        merged_map_lines,
    std::unordered_map<std::string, std::vector<ControlPoint>>* const
        merged_fcmap_lines,
    std::unordered_map<std::string, std::vector<ControlPoint>>* const
        merged_map_edges) {
  auto map_elment =
      hd_map.GetElement(hozon::mp::loc::HD_MAP_LANE_BOUNDARY_LINE);
  if (map_elment == nullptr) {
    return;
  }
  std::shared_ptr<MapBoundaryLine> map_boundary_lines =
      std::static_pointer_cast<MapBoundaryLine>(map_elment);
  if (map_boundary_lines->boundary_line_.size() == 0) {
    return;
  }
  auto map_road_edge = hd_map.GetElement(hozon::mp::loc::HD_MAP_ROAD_EDGE);
  if (map_road_edge == nullptr) {
    return;
  }
  std::shared_ptr<MapRoadEdge> map_road_edges =
      std::static_pointer_cast<MapRoadEdge>(map_road_edge);
  if (map_road_edges->edge_line_.size() == 0) {
    return;
  }
  auto t1 = std::chrono::steady_clock::now();
  auto t2 = std::chrono::steady_clock::now();
  if (map_boundary_lines->boundary_line_.empty()) {
    lane_control_pointInfo_.clear();
  } else {
    MergeMapLines(map_boundary_lines, T_W_V, merged_fcmap_lines,
                  merged_map_lines);
  }
  if (map_road_edges->edge_line_.empty()) {
    edge_control_pointInfo_.clear();
  } else {
    MergeMapEdges(map_road_edges, T_fc.pose, merged_map_edges);
  }
  t2 = std::chrono::steady_clock::now();
  auto merge_map_lines_cost_time =
      (t2.time_since_epoch() - t1.time_since_epoch()).count() / 1e9;
  HLOG_DEBUG << "MatchLaneLine: merge_map_lines_cost_time: "
             << merge_map_lines_cost_time << " ms";
}

void MapMatching::FilterPercpLane(
    const std::shared_ptr<Perception>& perception,
    std::list<std::list<LaneLinePerceptionPtr>>* const percep_lanelines) {
  if (perception == nullptr) {
    return;
  }
  for (const auto& p : perception->GetElement(PERCEPTYION_LANE_BOUNDARY_LINE)) {
    auto lane = std::static_pointer_cast<PerceptionLaneLineList>(p);
    std::list<LaneLinePerceptionPtr> fil_percep_laneline;
    FilterPercpLaneline(lane->lane_line_list_, &fil_percep_laneline);
    if (fil_percep_laneline.size() > 0) {
      (*percep_lanelines).emplace_back(fil_percep_laneline);
    }
  }
}

void MapMatching::FilterPercpLaneline(
    const std::list<LaneLinePerceptionPtr>& lanelines,
    std::list<LaneLinePerceptionPtr>* const out) {
  std::unordered_map<int, std::vector<LaneLinePerceptionPtr>> per_lines;
  if (out == nullptr || lanelines.empty()) {
    HLOG_ERROR << "FilterPercpLaneline: out == nullptr || lanelines.empty()";
    return;
  }
  bool is_changing_lane = false;
  int egolane_cnt = 0;
  for (auto& line : lanelines) {
    per_lines[line->lane_position_type()].emplace_back(line);
    HLOG_DEBUG << "percep-laneinfo "
               << line->curve_vehicle_coord_.lane_position_type_
               << " confid|min|max: " << line->curve_vehicle_coord_.confidence_
               << " " << line->Min() << " " << line->Max();
    if (per_lines[line->lane_position_type()].size() < 2) {
      continue;
    }
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
    auto line_points = line->points();
    if (line_points.empty()) {
      continue;
    }
    if (fabs(line_points[0].y()) < 0.5) {
      is_changing_lane = true;
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
      continue;
    }
    if (is_changing_lane) {
      if (fabs(line->lane_position_type()) == 1 ||
          fabs(line->lane_position_type()) == 2) {
        (*out).emplace_back(line);
      }
    } else if (egolane_cnt == 2) {
      if (fabs(line->lane_position_type()) == 1) {
        (*out).emplace_back(line);
      }
    } else {
      if (fabs(line->lane_position_type()) == 1 ||
          fabs(line->lane_position_type()) == 2) {
        (*out).emplace_back(line);
      }
    }
  }
}

void MapMatching::MergeMapLines(
    const std::shared_ptr<MapBoundaryLine>& boundary_lines, const SE3& T,
    std::unordered_map<std::string, std::vector<ControlPoint>>*
        merged_fcmap_lines,
    std::unordered_map<std::string, std::vector<ControlPoint>>*
        merged_map_lines) {
  if (boundary_lines == nullptr) {
    return;
  }
  if (!lines_map_.empty()) {
    lines_map_.clear();
  }
  if (!lane_control_pointInfo_.empty()) {
    lane_control_pointInfo_.clear();
  }

  auto T_V_W = T.inverse();
  for (const auto& line : boundary_lines->boundary_line_) {
    if (line.second.line_type == MapLineType::LaneChangeVirtualLine) {
      continue;
    }
    auto id = line.first;
    const auto& control_points = line.second.control_point;
    auto control_points_size = control_points.size();
    if (control_points_size <= 1) {
      HLOG_DEBUG << "MergeMapLines: control_points size <= 1";
      continue;
    }
    auto start_point = control_points.front().point;
    auto start_point_v = T_V_W * start_point;
    auto end_point = control_points.back().point;
    auto end_point_v = T_V_W * end_point;
    if (start_point.norm() == end_point.norm()) {
      HLOG_DEBUG << "MergeMapLines: start_point.norm() == end_point.norm()";
      continue;
    }
    ControlPointInfo cpt_info{start_point_v, end_point_v, control_points_size};
    if (lane_control_pointInfo_.find(cpt_info) !=
        lane_control_pointInfo_.end()) {
      continue;
    }
    LineSegment line_segment{id, end_point_v};
    lines_map_[start_point_v].emplace_back(std::move(line_segment));
    lane_control_pointInfo_.insert(cpt_info);
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
    Traversal(root_segment_start_point, line_ids, loop);
    multi_linked_lines_.emplace_back(linked_lines_id_);
  }
  if (multi_linked_lines_.empty()) {
    HLOG_DEBUG << "MergeMapLines: multi_linked_lines_ empty!";
    return;
  }
  std::unordered_map<std::string, int> merge_lane_ids;
  for (auto& lines : multi_linked_lines_) {
    std::vector<std::string> last_line_ids;
    for (auto& line_ids : lines) {
      if (line_ids.empty()) {
        continue;
      }
      int left = 0, right = line_ids.size() - 1;
      auto first_line_id = line_ids[0];
      auto last_line_id = line_ids[right];
      size_t mid_index = left + (right - left) / 2;
      auto mid_line_id = line_ids[mid_index];
      std::string merge_lane_id =
          first_line_id + "_" + mid_line_id + "_" + last_line_id;
      if (merge_lane_ids.find(merge_lane_id) != merge_lane_ids.end()) {
        if (line_ids.size() == last_line_ids.size() &&
            std::equal(line_ids.begin(), line_ids.end(),
                       last_line_ids.begin())) {
          merge_lane_ids[merge_lane_id] += 1;
          continue;
        } else {
          merge_lane_id = merge_lane_id + "_00000";
        }
      }
      for (auto& line_id : line_ids) {
        for (auto control_point :
             boundary_lines->boundary_line_[line_id].control_point) {
          (*merged_fcmap_lines)[merge_lane_id].emplace_back(control_point);
          control_point.point = T_V_W * control_point.point;
          (*merged_map_lines)[merge_lane_id].emplace_back(control_point);
        }
      }
      last_line_ids = line_ids;
      merge_lane_ids[merge_lane_id] += 1;
    }
  }
  multi_linked_lines_.clear();
}

void MapMatching::Traversal(const V3& root_start_point,
                            std::vector<std::string> line_ids, int loop) {
  if (lines_map_.empty()) {
    return;
  }

  std::stack<std::tuple<V3, std::vector<std::string>, int>> stack;
  stack.push(std::make_tuple(root_start_point, line_ids, loop));

  while (!stack.empty()) {
    auto [cur_start_point, cur_line_ids, cur_loop] = stack.top();
    stack.pop();  // V3 cur_start_point = root_start_point;
    auto cur_line_infos_iter = lines_map_.find(cur_start_point);
    if (cur_line_infos_iter == lines_map_.end()) {
      if (cur_line_ids.empty()) {
        HLOG_DEBUG << "Traversal: traversal loop failed " << cur_loop;
        continue;
      }
      linked_lines_id_.emplace_back(cur_line_ids);
      continue;
    }
    auto successor_segments_info = cur_line_infos_iter->second;
    int size = static_cast<int>(successor_segments_info.size());
    if (size > 0) {
      for (int i = 0; i < size; ++i) {
        bool find_flag =
            std::binary_search(cur_line_ids.begin(), cur_line_ids.end(),
                               successor_segments_info[i].first);
        if (find_flag) {
          linked_lines_id_.emplace_back(cur_line_ids);
          continue;
        }
        cur_line_ids.emplace_back(successor_segments_info[i].first);
        cur_loop++;
        visited_id_.insert(successor_segments_info[i].first);
        stack.push(std::make_tuple(successor_segments_info[i].second,
                                   cur_line_ids, cur_loop));
        cur_line_ids.pop_back();
        cur_loop--;
      }
      if (cur_line_ids.size() > 50) {
        HLOG_ERROR << "Traversal: cur_line_ids size > 50!!!, break loop!!!";
        break;
      }
    } else {
      HLOG_DEBUG << "Traversal: traversal loop: " << cur_loop << " failed!";
    }
  }
}

void MapMatching::MergeMapEdges(
    const std::shared_ptr<MapRoadEdge>& road_edges, const SE3& T,
    std::unordered_map<std::string, std::vector<ControlPoint>>*
        merged_map_edges) {
  if (road_edges == nullptr) {
    return;
  }
  if (!edges_map_.empty()) {
    edges_map_.clear();
  }
  if (!edge_control_pointInfo_.empty()) {
    edge_control_pointInfo_.clear();
  }

  auto T_V_W = T.inverse();
  for (const auto& line : road_edges->edge_line_) {
    auto id = line.first;
    const auto& control_points = line.second.control_point;
    auto control_points_size = control_points.size();
    if (control_points_size <= 1) {
      HLOG_INFO << "MergeMapEdges: control_points size <= 1";
      continue;
    }
    auto start_point = control_points.front().point;
    auto start_point_v = T_V_W * start_point;
    auto end_point = control_points.back().point;
    auto end_point_v = T_V_W * end_point;
    if (start_point.norm() == end_point.norm()) {
      HLOG_DEBUG << "MergeMapEdges: start_point.norm() == end_point.norm()";
      continue;
    }
    ControlPointInfo cpt_info{start_point_v, end_point_v, control_points_size};
    if (edge_control_pointInfo_.find(cpt_info) !=
        edge_control_pointInfo_.end()) {
      continue;
    }
    LineSegment line_segment{id, end_point_v};
    edges_map_[start_point_v].emplace_back(std::move(line_segment));
    edge_control_pointInfo_.insert(cpt_info);
  }
  HLOG_INFO << "MergeMapEdges: reconstruct map road edges end!";
  if (!edge_visited_id_.empty()) {
    edge_visited_id_.clear();
  }
  for (auto& line : edges_map_) {
    auto root_segment_start_point = line.first;
    std::vector<std::string> line_ids;
    if (edge_visited_id_.count(line.second.front().first) > 0) {
      continue;
    }
    int loop = 0;
    if (!linked_edges_id_.empty()) {
      linked_edges_id_.clear();
    }
    EdgesTraversal(root_segment_start_point, line_ids, loop);
    multi_linked_edges_.emplace_back(linked_edges_id_);
  }
  if (multi_linked_edges_.empty()) {
    HLOG_DEBUG << "MergeMapEdges: multi_linked_edges_ empty!";
    return;
  }
  std::unordered_map<std::string, int> merge_lane_ids;
  for (auto& lines : multi_linked_edges_) {
    std::vector<std::string> last_line_ids;
    for (auto& line_ids : lines) {
      if (line_ids.empty()) {
        continue;
      }
      int left = 0, right = line_ids.size() - 1;
      auto first_line_id = line_ids[0];
      auto last_line_id = line_ids[right];
      size_t mid_index = left + (right - left) / 2;
      auto mid_line_id = line_ids[mid_index];
      std::string merge_lane_id =
          first_line_id + "_" + mid_line_id + "_" + last_line_id;
      if (merge_lane_ids.find(merge_lane_id) != merge_lane_ids.end()) {
        if (line_ids.size() == last_line_ids.size() &&
            std::equal(line_ids.begin(), line_ids.end(),
                       last_line_ids.begin())) {
          merge_lane_ids[merge_lane_id] += 1;
          continue;
        } else {
          merge_lane_id = merge_lane_id + "_00000";
        }
      }
      for (auto& line_id : line_ids) {
        for (auto control_point :
             road_edges->edge_line_[line_id].control_point) {
          control_point.point = T_V_W * control_point.point;
          (*merged_map_edges)[merge_lane_id].emplace_back(control_point);
        }
      }
      last_line_ids = line_ids;
      merge_lane_ids[merge_lane_id] += 1;
    }
  }
  multi_linked_edges_.clear();
}

void MapMatching::EdgesTraversal(const V3& root_start_point,
                                 std::vector<std::string> line_ids, int loop) {
  if (edges_map_.empty()) {
    return;
  }

  std::stack<std::tuple<V3, std::vector<std::string>, int>> stack;
  stack.push(std::make_tuple(root_start_point, line_ids, loop));

  while (!stack.empty()) {
    auto [cur_start_point, cur_line_ids, cur_loop] = stack.top();
    stack.pop();  // V3 cur_start_point = root_start_point;
    auto cur_line_infos_iter = edges_map_.find(cur_start_point);
    if (cur_line_infos_iter == edges_map_.end()) {
      if (cur_line_ids.empty()) {
        HLOG_DEBUG << "Traversal: traversal loop failed " << cur_loop;
        continue;
      }
      linked_edges_id_.emplace_back(cur_line_ids);
      continue;
    }
    auto successor_segments_info = cur_line_infos_iter->second;
    int size = static_cast<int>(successor_segments_info.size());
    if (size > 0) {
      for (int i = 0; i < size; ++i) {
        bool find_flag =
            std::binary_search(cur_line_ids.begin(), cur_line_ids.end(),
                               successor_segments_info[i].first);
        if (find_flag) {
          linked_edges_id_.emplace_back(cur_line_ids);
          continue;
        }
        cur_line_ids.emplace_back(successor_segments_info[i].first);
        cur_loop++;
        edge_visited_id_.insert(successor_segments_info[i].first);
        stack.push(std::make_tuple(successor_segments_info[i].second,
                                   cur_line_ids, cur_loop));
        cur_line_ids.pop_back();
        cur_loop--;
      }
      if (cur_line_ids.size() > 50) {
        HLOG_ERROR
            << "EDGE Traversal: cur_line_ids size > 50!!!, break loop!!!";
        break;
      }
    } else {
      HLOG_DEBUG << "EDGE Traversal: traversal loop: " << cur_loop
                 << " failed!";
    }
  }
}

void MapMatching::pubOdomPoints(const std::string& topic, const SE3& T,
                                uint64_t sec, uint64_t nsec) {
  if (!RVIZ_AGENT.Ok()) {
    return;
  }
  adsfi_proto::viz::Odometry odom;
  int curr_seq = 0;
  odom.mutable_header()->set_seq(curr_seq++);
  odom.mutable_header()->set_frameid("map");
  odom.mutable_header()->mutable_timestamp()->set_sec(sec);
  odom.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  odom.mutable_pose()->mutable_pose()->mutable_position()->set_x(
      T.translation().x());
  odom.mutable_pose()->mutable_pose()->mutable_position()->set_y(
      T.translation().y());
  odom.mutable_pose()->mutable_pose()->mutable_position()->set_z(
      T.translation().z());
  odom.mutable_pose()->mutable_pose()->mutable_orientation()->set_x(
      T.unit_quaternion().x());
  odom.mutable_pose()->mutable_pose()->mutable_orientation()->set_y(
      T.unit_quaternion().y());
  odom.mutable_pose()->mutable_pose()->mutable_orientation()->set_z(
      T.unit_quaternion().z());
  odom.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(
      T.unit_quaternion().w());
  for (int i = 0; i < 36; ++i) {
    odom.mutable_pose()->add_covariance(0.);
  }
  RVIZ_AGENT.Publish(topic, odom);
}

hozon::mp::loc::Map<hozon::hdmap::Map> MapMatching::SetHdMap(
    const std::vector<hozon::hdmap::LaneInfoConstPtr>& lane_ptr_vec,
    const Eigen::Vector3d& ref_point) {
  hozon::mp::loc::Map<hozon::hdmap::Map> hdmap;
  hdmap.Clear();
  hdmap.set_ref_point(ref_point);
  hdmap.SetMap(lane_ptr_vec, ref_point);
  return hdmap;
}

PtrNodeInfo MapMatching::getMmNodeInfo() {
  auto output_node_info =
      std::make_shared<::hozon::localization::HafNodeInfo>();
  mm_output_lck_.lock();
  output_node_info = mm_node_info_;
  mm_output_lck_.unlock();
  return output_node_info;
}

PtrNodeInfo MapMatching::generateNodeInfo(
    const Sophus::SE3d& T_W_V, uint64_t sec, uint64_t nsec, const bool& has_err,
    const Eigen::Vector3d& ref_point, double ins_height, int sys_status,
    bool is_big_curvature_frame, double lane_width_check_coeff) {
  PtrNodeInfo node_info =
      std::make_shared<::hozon::localization::HafNodeInfo>();
  auto blh = hozon::mp::util::Geo::EnuToGcj02(
      T_W_V.translation().cast<double>(), ref_point);
  node_info->set_sys_status(sys_status);
  node_info->set_type(::hozon::localization::HafNodeInfo_NodeType::
                          HafNodeInfo_NodeType_MapMatcher);
  double node_stamp =
      static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9;
  node_info->mutable_header()->set_data_stamp(node_stamp);
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
      tp = std::chrono::time_point_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now());
  node_info->mutable_header()->set_publish_stamp(
      static_cast<double>(tp.time_since_epoch().count()) * 1.0e-9);
  node_info->mutable_header()->set_frame_id("node_info_mm");
  node_info->set_is_valid(true);
  node_info->mutable_pos_gcj02()->set_x(blh.x());
  node_info->mutable_pos_gcj02()->set_y(blh.y());
  node_info->mutable_pos_gcj02()->set_z(ins_height);
  node_info->mutable_quaternion()->set_x(
      static_cast<float>(T_W_V.unit_quaternion().x()));
  node_info->mutable_quaternion()->set_y(
      static_cast<float>(T_W_V.unit_quaternion().y()));
  node_info->mutable_quaternion()->set_z(
      static_cast<float>(T_W_V.unit_quaternion().z()));
  node_info->mutable_quaternion()->set_w(
      static_cast<float>(T_W_V.unit_quaternion().w()));
  // sd_velocity 用这个字段的x 填充一下该帧是否大曲率信息： 1 是 0 否
  node_info->mutable_sd_velocity()->set_x(is_big_curvature_frame);
  // sd_velocity 用这个字段的y 填充一下该帧宽度校验系数
  node_info->mutable_sd_velocity()->set_y(lane_width_check_coeff);
  node_info->mutable_sd_velocity()->set_z(0);
  if (!has_err) {
    node_info->set_valid_estimate(true);
  } else {
    node_info->set_valid_estimate(false);
    HLOG_ERROR << "MM is not valid due to rare connect!!!";
  }
  // error code
  node_info->set_warn_info(0);
  if (mmfault_.pecep_lane_error) {
    node_info->set_warn_info(123);
    HLOG_ERROR << "mmfault:123";
  }
  if (mmfault_.map_lane_error) {
    node_info->set_warn_info(124);
    HLOG_ERROR << "mmfault:124";
  }
  if (mmfault_.map_lane_match_error) {
    node_info->set_warn_info(130);
    HLOG_ERROR << "mmfault:130";
  }
  return node_info;
}

void MapMatching::setPoints(const PerceptionLaneLineList& line_list,
                            const SE3& T_W_V, VP* points) {
  if (points == nullptr) {
    return;
  }
  (*points).clear();
  for (const auto& line : line_list.lane_line_list_) {
    if (std::fabs(line->lane_position_type()) > 2 || line->points().empty()) {
      continue;
    }
    auto line_points = line->points();
    auto point_size = std::min(500, static_cast<int>(line_points.size()));
    for (auto i = 0; i < point_size; ++i) {
      auto point = T_W_V * line_points[i];
      (*points).emplace_back(point);
    }
  }
}

void MapMatching::pubPoints(const VP& points, const uint64_t& sec,
                            const uint64_t& nsec,
                            const std::string& krviz_topic) {
  if (RVIZ_AGENT.Ok()) {
    adsfi_proto::viz::PointCloud lane_points;
    int curr_seq = 0;
    lane_points.mutable_header()->set_seq(curr_seq++);
    lane_points.mutable_header()->mutable_timestamp()->set_sec(sec);
    lane_points.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    lane_points.mutable_header()->set_frameid("map");

    auto* channels = lane_points.add_channels();
    channels->set_name("rgb");

    for (auto p : points) {
      auto* points_ = lane_points.add_points();
      points_->set_x(static_cast<float>(p.x()));
      points_->set_y(static_cast<float>(p.y()));
      points_->set_z(static_cast<float>(p.z()));
    }

    RVIZ_AGENT.Publish(krviz_topic, lane_points);
  }
}

void MapMatching::PubTfAndPath(const SE3& T, uint64_t sec, uint64_t nsec) {
  adsfi_proto::viz::TransformStamped geo_tf;
  int curr_seq = 0;
  geo_tf.mutable_header()->set_seq(curr_seq++);
  geo_tf.mutable_header()->mutable_timestamp()->set_sec(sec);
  geo_tf.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  geo_tf.mutable_header()->set_frameid("map");
  geo_tf.set_child_frame_id("vehicle");
  geo_tf.mutable_transform()->mutable_translation()->set_x(T.translation().x());
  geo_tf.mutable_transform()->mutable_translation()->set_y(T.translation().y());
  geo_tf.mutable_transform()->mutable_translation()->set_z(T.translation().z());
  geo_tf.mutable_transform()->mutable_rotation()->set_x(
      T.unit_quaternion().x());
  geo_tf.mutable_transform()->mutable_rotation()->set_y(
      T.unit_quaternion().y());
  geo_tf.mutable_transform()->mutable_rotation()->set_z(
      T.unit_quaternion().z());
  geo_tf.mutable_transform()->mutable_rotation()->set_w(
      T.unit_quaternion().w());
  RVIZ_AGENT.Publish(kTopicMmTf, geo_tf);

  adsfi_proto::viz::Path gnss_gcj02_path;
  auto* pose = gnss_gcj02_path.add_poses();
  gnss_gcj02_path.mutable_header()->set_seq(curr_seq++);
  gnss_gcj02_path.mutable_header()->mutable_timestamp()->set_sec(sec);
  gnss_gcj02_path.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  gnss_gcj02_path.mutable_header()->set_frameid("map");
  pose->mutable_header()->set_seq(curr_seq);
  pose->mutable_header()->mutable_timestamp()->set_sec(sec);
  pose->mutable_header()->mutable_timestamp()->set_nsec(nsec);
  pose->mutable_header()->set_frameid("map");
  pose->mutable_pose()->mutable_position()->set_x(T.translation().x());
  pose->mutable_pose()->mutable_position()->set_y(T.translation().y());
  pose->mutable_pose()->mutable_position()->set_z(T.translation().z());
  pose->mutable_pose()->mutable_orientation()->set_w(T.unit_quaternion().w());
  pose->mutable_pose()->mutable_orientation()->set_x(T.unit_quaternion().x());
  pose->mutable_pose()->mutable_orientation()->set_y(T.unit_quaternion().y());
  pose->mutable_pose()->mutable_orientation()->set_z(T.unit_quaternion().z());

  RVIZ_AGENT.Publish(kTopicMmCarPath, gnss_gcj02_path);
}

void MapMatching::PubLocState(const SE3& T, uint64_t sec, uint64_t nsec,
                              int loc_state) {
  adsfi_proto::viz::Marker text_marker;
  text_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
  text_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
  text_marker.set_id(0);
  text_marker.mutable_lifetime()->set_sec(0);
  text_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
  text_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  text_marker.mutable_header()->set_frameid("map");
  text_marker.mutable_pose()->mutable_position()->set_x(T.translation().x());
  text_marker.mutable_pose()->mutable_position()->set_y(T.translation().y());
  text_marker.mutable_pose()->mutable_position()->set_z(T.translation().z());

  text_marker.mutable_pose()->mutable_orientation()->set_x(0);
  text_marker.mutable_pose()->mutable_orientation()->set_y(0);
  text_marker.mutable_pose()->mutable_orientation()->set_z(0);
  text_marker.mutable_pose()->mutable_orientation()->set_w(1);

  text_marker.mutable_color()->set_r(1);
  text_marker.mutable_color()->set_g(1);
  text_marker.mutable_color()->set_b(1);
  text_marker.mutable_color()->set_a(1);

  text_marker.set_text("loc-state:" + std::to_string(loc_state));
  text_marker.mutable_scale()->set_x(0.1);
  text_marker.mutable_scale()->set_y(0);
  text_marker.mutable_scale()->set_z(0.8);
  hozon::mp::util::RvizAgent::Instance().Publish(kTopicLocstate, text_marker);
}

void MapMatching::pubTimeAndInsStatus(const SE3& T, uint64_t sec, uint64_t nsec,
                                      int ins_state) {
  adsfi_proto::viz::Marker text_marker;
  text_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
  text_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
  text_marker.set_id(0);
  text_marker.mutable_lifetime()->set_sec(0);
  text_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
  text_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  text_marker.mutable_header()->set_frameid("map");
  text_marker.mutable_pose()->mutable_position()->set_x(T.translation().x() +
                                                        1);
  text_marker.mutable_pose()->mutable_position()->set_y(T.translation().y() +
                                                        1);
  text_marker.mutable_pose()->mutable_position()->set_z(12);

  text_marker.mutable_pose()->mutable_orientation()->set_x(0);
  text_marker.mutable_pose()->mutable_orientation()->set_y(0);
  text_marker.mutable_pose()->mutable_orientation()->set_z(0);
  text_marker.mutable_pose()->mutable_orientation()->set_w(1);

  text_marker.mutable_color()->set_r(1);
  text_marker.mutable_color()->set_g(1);
  text_marker.mutable_color()->set_b(1);
  text_marker.mutable_color()->set_a(1);

  text_marker.set_text("ins-state:" + std::to_string(ins_state) + " time:" +
                       std::to_string(static_cast<float>(sec) +
                                      static_cast<float>(nsec) * 1e-9));
  text_marker.mutable_scale()->set_x(0.1);
  text_marker.mutable_scale()->set_y(0);
  text_marker.mutable_scale()->set_z(0.8);

  RVIZ_AGENT.Publish(kTopicMmTimeStamp, text_marker);
}

adsfi_proto::viz::Marker MapMatching::RoadEdgeToMarker(
    const VP& points, const std::string& id, bool is_points, bool is_center,
    uint64_t sec, uint64_t nsec, bool is_edge, float point_size) {
  adsfi_proto::viz::Marker block;
  if (is_points) {
    block.set_type(adsfi_proto::viz::MarkerType::POINTS);
  } else {
    block.set_type(adsfi_proto::viz::MarkerType::LINE_LIST);
  }
  block.set_action(adsfi_proto::viz::MarkerAction::ADD);
  const char* c_id = id.c_str();
  block.set_id(std::atoi(c_id));
  block.mutable_lifetime()->set_sec(0);
  block.mutable_lifetime()->set_nsec(0);
  block.mutable_header()->mutable_timestamp()->set_sec(sec);
  block.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  block.mutable_header()->set_frameid("map");
  std::string control = is_points ? "_control" : "";
  std::string center = is_center ? "_center" : "";
  std::string edge = is_edge ? "_edge" : "";
  block.set_ns("road" + control + center + edge);
  block.mutable_pose()->mutable_position()->set_x(0);
  block.mutable_pose()->mutable_position()->set_y(0);
  block.mutable_pose()->mutable_position()->set_z(0);
  block.mutable_pose()->mutable_orientation()->set_x(0.0);
  block.mutable_pose()->mutable_orientation()->set_y(0.0);
  block.mutable_pose()->mutable_orientation()->set_z(0.0);
  block.mutable_pose()->mutable_orientation()->set_w(1.0);
  float size = 0.05;
  if (is_points) {
    size = 0.25;
  }
  size = size * point_size;
  block.mutable_scale()->set_x(size);
  block.mutable_scale()->set_y(size);
  block.mutable_scale()->set_z(size);
  if (is_edge) {
    block.mutable_color()->set_r(0 / 255.0);
    block.mutable_color()->set_g(0 / 255.0);
    block.mutable_color()->set_b(255 / 255.0);
  }
  if (is_center) {
    block.mutable_color()->set_r(255.0 / 255.0);
    block.mutable_color()->set_g(255.0 / 255.0);
    block.mutable_color()->set_b(0.0 / 255.0);
  }
  if (!is_edge && !is_center) {
    block.mutable_color()->set_r(1.0);
    block.mutable_color()->set_g(0);
    block.mutable_color()->set_b(0);
  }
  block.mutable_color()->set_a(1.0);
  for (size_t i = 1; i < points.size(); i++) {
    auto* enu = block.add_points();
    enu->set_x(points[i - 1].x());
    enu->set_y(points[i - 1].y());
    enu->set_z(points[i - 1].z());
    auto* enu_ = block.add_points();
    enu_->set_x(points[i].x());
    enu_->set_y(points[i].y());
    enu_->set_z(points[i].z());
  }
  return block;
}

adsfi_proto::viz::Marker MapMatching::laneToMarker(
    const VP& points, const std::string& id, bool is_points, bool is_center,
    uint64_t sec, uint64_t nsec, bool is_boundary, float point_size) {
  adsfi_proto::viz::Marker block;
  if (is_points) {
    block.set_type(adsfi_proto::viz::MarkerType::POINTS);
  } else {
    block.set_type(adsfi_proto::viz::MarkerType::LINE_LIST);
  }
  block.set_action(adsfi_proto::viz::MarkerAction::ADD);
  const char* c_id = id.c_str();
  block.set_id(std::atoi(c_id));
  block.mutable_lifetime()->set_sec(0);
  block.mutable_lifetime()->set_nsec(0);
  block.mutable_header()->mutable_timestamp()->set_sec(sec);
  block.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  block.mutable_header()->set_frameid("map");
  std::string control = is_points ? "_control" : "";
  std::string center = is_center ? "_center" : "";
  std::string boundary = is_boundary ? "_boundary" : "";
  block.set_ns("lane" + control + center + boundary);
  block.mutable_pose()->mutable_position()->set_x(0);
  block.mutable_pose()->mutable_position()->set_y(0);
  block.mutable_pose()->mutable_position()->set_z(0);
  block.mutable_pose()->mutable_orientation()->set_x(0.0);
  block.mutable_pose()->mutable_orientation()->set_y(0.0);
  block.mutable_pose()->mutable_orientation()->set_z(0.0);
  block.mutable_pose()->mutable_orientation()->set_w(1.0);
  float size = 0.05;
  if (is_points) {
    size = 0.25;
  }
  size = size * point_size;
  block.mutable_scale()->set_x(size);
  block.mutable_scale()->set_y(size);
  block.mutable_scale()->set_z(size);
  if (is_boundary) {
    block.mutable_color()->set_r(229.0 / 255.0);
    block.mutable_color()->set_g(10 / 255.0);
    block.mutable_color()->set_b(245 / 255.0);
  }
  if (is_center) {
    block.mutable_color()->set_r(255.0 / 255.0);
    block.mutable_color()->set_g(255.0 / 255.0);
    block.mutable_color()->set_b(0.0 / 255.0);
  }
  if (!is_boundary && !is_center) {
    block.mutable_color()->set_r(1.0);
    block.mutable_color()->set_g(0);
    block.mutable_color()->set_b(0);
  }
  block.mutable_color()->set_a(1.0);
  for (size_t i = 1; i < points.size(); i++) {
    auto* enu = block.add_points();
    enu->set_x(points[i - 1].x());
    enu->set_y(points[i - 1].y());
    enu->set_z(points[i - 1].z());
    auto* enu_ = block.add_points();
    enu_->set_x(points[i].x());
    enu_->set_y(points[i].y());
    enu_->set_z(points[i].z());
  }
  return block;
}

adsfi_proto::viz::Marker MapMatching::lineIdToMarker(const V3& point,
                                                     const std::string& id,
                                                     uint64_t sec,
                                                     uint64_t nsec) {
  adsfi_proto::viz::Marker marker;
  marker.mutable_header()->set_frameid("map");
  marker.mutable_header()->mutable_timestamp()->set_sec(sec);
  marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  const char* c_id = id.c_str();
  marker.set_id(std::atoi(c_id));
  marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
  marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
  marker.mutable_pose()->mutable_orientation()->set_x(0.);
  marker.mutable_pose()->mutable_orientation()->set_y(0.);
  marker.mutable_pose()->mutable_orientation()->set_z(0.);
  marker.mutable_pose()->mutable_orientation()->set_w(1.);
  const double text_size = 0.6;
  marker.mutable_scale()->set_z(text_size);
  marker.mutable_lifetime()->set_sec(0);
  marker.mutable_lifetime()->set_nsec(0);
  marker.mutable_color()->set_a(1.0);
  marker.mutable_color()->set_r(0);
  marker.mutable_color()->set_g(1);
  marker.mutable_color()->set_b(0);
  marker.mutable_pose()->mutable_position()->set_x(point.x());
  marker.mutable_pose()->mutable_position()->set_y(point.y());
  marker.mutable_pose()->mutable_position()->set_z(point.z());
  auto* text = marker.mutable_text();
  *text = "ID: " + id;
  return marker;
}
void MapMatching::pubConnectPercepPoints(const VP& points, uint64_t sec,
                                         uint64_t nsec) {
  if (!RVIZ_AGENT.Ok()) {
    return;
  }
  adsfi_proto::viz::PointCloud lane_points;
  int curr_seq = 0;
  lane_points.mutable_header()->set_seq(curr_seq++);
  lane_points.mutable_header()->mutable_timestamp()->set_sec(sec);
  lane_points.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  lane_points.mutable_header()->set_frameid("map");
  auto* channels = lane_points.add_channels();
  channels->set_name("rgb");
  for (const auto& p : points) {
    auto* points_ = lane_points.add_points();
    points_->set_x(static_cast<float>(p.x()));
    points_->set_y(static_cast<float>(p.y()));
    points_->set_z(static_cast<float>(p.z()));
  }
  RVIZ_AGENT.Publish(kTopicMmConnectPercepPoints, lane_points);
}

void MapMatching::pubConnectMapPoints(const VP& points, uint64_t sec,
                                      uint64_t nsec) {
  if (RVIZ_AGENT.Ok()) {
    adsfi_proto::viz::PointCloud lane_points;
    int curr_seq = 0;
    lane_points.mutable_header()->set_seq(curr_seq++);
    lane_points.mutable_header()->mutable_timestamp()->set_sec(sec);
    lane_points.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    lane_points.mutable_header()->set_frameid("map");
    auto* channels = lane_points.add_channels();
    channels->set_name("rgb");
    for (const auto& p : points) {
      auto* points_ = lane_points.add_points();
      points_->set_x(static_cast<float>(p.x()));
      points_->set_y(static_cast<float>(p.y()));
      points_->set_z(static_cast<float>(p.z()));
    }
    RVIZ_AGENT.Publish(kTopicMmConnectMapPoints, lane_points);
  }
}

void MapMatching::pubOriginConnectPercepPoints(const VP& points, uint64_t sec,
                                               uint64_t nsec) {
  if (RVIZ_AGENT.Ok()) {
    adsfi_proto::viz::PointCloud lane_points;
    int curr_seq = 0;
    lane_points.mutable_header()->set_seq(curr_seq++);
    lane_points.mutable_header()->mutable_timestamp()->set_sec(sec);
    lane_points.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    lane_points.mutable_header()->set_frameid("map");
    auto* channels = lane_points.add_channels();
    channels->set_name("rgb");
    for (const auto& p : points) {
      auto* points_ = lane_points.add_points();
      points_->set_x(static_cast<float>(p.x()));
      points_->set_y(static_cast<float>(p.y()));
      points_->set_z(static_cast<float>(p.z()));
    }
    RVIZ_AGENT.Publish(kTopicMmOriginConnectPercepPoints, lane_points);
  }
}

void MapMatching::pubOriginConnectMapPoints(const VP& points, uint64_t sec,
                                            uint64_t nsec) {
  if (RVIZ_AGENT.Ok()) {
    adsfi_proto::viz::PointCloud lane_points;
    int curr_seq = 0;
    lane_points.mutable_header()->set_seq(curr_seq++);
    lane_points.mutable_header()->mutable_timestamp()->set_sec(sec);
    lane_points.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    lane_points.mutable_header()->set_frameid("map");
    auto* channels = lane_points.add_channels();
    channels->set_name("rgb");
    for (const auto& p : points) {
      auto* points_ = lane_points.add_points();
      points_->set_x(static_cast<float>(p.x()));
      points_->set_y(static_cast<float>(p.y()));
      points_->set_z(static_cast<float>(p.z()));
    }
    RVIZ_AGENT.Publish(kTopicMmOriginConnectMapPoints, lane_points);
  }
}

void MapMatching::setConnectPercepPoints(const Connect& connect,
                                         const SE3& T_W_V, VP* points) {
  for (const auto& lane_line_match_pair : connect.lane_line_match_pairs) {
    auto perception_point = lane_line_match_pair.pecep_pv;
    auto point = T_W_V * perception_point;
    points->emplace_back(point);
  }
}

void MapMatching::setConnectMapPoints(const Connect& connect, VP* points) {
  for (const auto& lane_line_match_pair : connect.lane_line_match_pairs) {
    auto point = lane_line_match_pair.map_pw;
    points->emplace_back(point);
  }
}

void MapMatching::setOriginConnectPercepPoints(const Connect& connect,
                                               const SE3& T_W_V, VP* points) {
  for (const auto& lane_line_match_pair : connect.lane_line_match_pairs) {
    auto perception_point = lane_line_match_pair.pecep_pv;
    auto point = T_W_V * perception_point;
    points->emplace_back(point);
  }
}

void MapMatching::setOriginConnectMapPoints(const Connect& connect,
                                            VP* points) {
  for (const auto& lane_line_match_pair : connect.lane_line_match_pairs) {
    auto point = lane_line_match_pair.map_pw;
    points->emplace_back(point);
  }
}

void MapMatching::RvizFunc(uint64_t cur_sec, uint64_t cur_nsec,
                           const hozon::mp::loc::Connect& connect,
                           const SE3& T_output,
                           const VP& rviz_merged_map_lines) {
  RVIZ_AGENT.Register<adsfi_proto::viz::TransformStamped>(kTopicMmTf);
  RVIZ_AGENT.Register<adsfi_proto::viz::Path>(kTopicMmCarPath);
  RVIZ_AGENT.Register<adsfi_proto::viz::PointCloud>(kTopicMmFrontPoints);
  RVIZ_AGENT.Register<adsfi_proto::viz::PointCloud>(
      kTopicMmMergedMapLaneLinePoints);
  RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(KTopicMmHdMap);
  RVIZ_AGENT.Register<adsfi_proto::viz::Marker>(kTopicMmTimeStamp);
  RVIZ_AGENT.Register<adsfi_proto::viz::Odometry>(kTopicInsOdom);
  RVIZ_AGENT.Register<adsfi_proto::viz::Odometry>(kTopicMmOdom);
  RVIZ_AGENT.Register<adsfi_proto::viz::Odometry>(kTopicDrOdom);
  RVIZ_AGENT.Register<adsfi_proto::viz::Odometry>(kTopicFcOdom);
  RVIZ_AGENT.Register<adsfi_proto::viz::Marker>(kTopicLocstate);
  RVIZ_AGENT.Register<adsfi_proto::viz::PointCloud>(
      kTopicMmConnectPercepPoints);
  RVIZ_AGENT.Register<adsfi_proto::viz::PointCloud>(kTopicMmConnectMapPoints);
  RVIZ_AGENT.Register<adsfi_proto::viz::PointCloud>(
      kTopicMmOriginConnectPercepPoints);
  RVIZ_AGENT.Register<adsfi_proto::viz::PointCloud>(
      kTopicMmOriginConnectMapPoints);
  RVIZ_AGENT.Register<adsfi_proto::viz::Odometry>(kTopicInputOdom);
  // merge后地图车道线
  VP merged_map_lane_lines;
  merged_map_lane_lines = rviz_merged_map_lines;
  pubPoints(merged_map_lane_lines, cur_sec, cur_nsec,
            kTopicMmMergedMapLaneLinePoints);
  // 输出位姿投影关联后的感知车道线
  std::vector<Eigen::Vector3d> connect_per_points;
  setConnectPercepPoints(connect, T_output, &connect_per_points);
  pubConnectPercepPoints(connect_per_points, cur_sec, cur_nsec);
  // 关联后的地图车道线
  std::vector<Eigen::Vector3d> connect_map_points;
  setConnectMapPoints(connect, &connect_map_points);
  pubConnectMapPoints(connect_map_points, cur_sec, cur_nsec);
  // 输出位姿投影关联后的原始感知车道线
  hozon::mp::loc::Connect origin_connect = map_match_->OriginResult();
  std::vector<Eigen::Vector3d> orin_connect_per_points;
  setOriginConnectPercepPoints(origin_connect, T_output,
                               &orin_connect_per_points);
  pubOriginConnectPercepPoints(orin_connect_per_points, cur_sec, cur_nsec);
  // 关联后的原始地图车道线
  std::vector<Eigen::Vector3d> origin_connect_map_points;
  setOriginConnectMapPoints(origin_connect, &origin_connect_map_points);
  pubOriginConnectMapPoints(origin_connect_map_points, cur_sec, cur_nsec);
  // 可视化mm的odom
  pubOdomPoints(kTopicMmOdom, T_output, cur_sec, cur_nsec);
}

}  // namespace loc
}  // namespace mp
}  // namespace hozon
