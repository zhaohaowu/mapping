/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： pose_estimate_lane_line.cc
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/location/pose_estimation/lib/pose_estimate/pose_estimate_lane_line.h"

#include <algorithm>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace hozon {
namespace mp {
namespace loc {

MatchLaneLine::MatchLaneLine() {
  match_pairs_.reserve(6 * 50);
  debug_points_.reserve(6 * 100);
  line_match_pairs_.clear();
  T_W_V_ = SE3();
  T_V_W_ = SE3();
  has_err_ = false;
  percep_ = nullptr;
  err_type_ = ErrorType::NO_ERROR;
}

void MatchLaneLine::set_ins_ts(const double &ins_ts) {
  ins_timestamp_ = ins_ts;
}

void MatchLaneLine::Match(const HdMap &hd_map,
                          const std::shared_ptr<Perception> &perception,
                          const SE3 &T_W_V, const SE3 &T_fc,
                          const VP &percep_points,
                          const VP &nearest_map_points) {
  percep_ = perception;
  has_err_ = false;
  err_type_ = ErrorType::NO_ERROR;
  T_W_V_ = T_W_V;
  T_V_W_ = T_W_V.inverse();
  match_pairs_.clear();
  debug_points_.clear();
  debug_lines_after_merge_.clear();
  debug_lines_before_merge_.clear();
  map_merge_lines_.clear();
  line_match_pairs_.clear();
  percep_lanelines_.clear();
  static int invalid_pecep_cnt = 0;

  auto map_elment =
      hd_map.GetElement(hozon::mp::loc::HD_MAP_LANE_BOUNDARY_LINE);
  if (map_elment == nullptr) {
    if (mm_params.use_valid_map_lane_fault) {
      has_err_ = true;
      err_type_ = ErrorType::NO_VALID_MAP_LANE;
    }
    HLOG_ERROR << "map_lane_line == nullptr";
    return;
  }
  std::shared_ptr<MapBoundaryLine> map_boundary_lines =
      std::static_pointer_cast<MapBoundaryLine>(map_elment);
  if (map_boundary_lines->boundary_line_.size() == 0) {
    if (mm_params.use_valid_map_lane_fault) {
      has_err_ = true;
      err_type_ = ErrorType::NO_MAP_BOUNDARY_LINE;
    }
    HLOG_ERROR << "map_boundary_lines->_boundary_line.size() == 0";
    return;
  }

  for (const auto &p : percep_->GetElement(PERCEPTYION_LANE_BOUNDARY_LINE)) {
    auto lane = std::static_pointer_cast<PerceptionLaneLineList>(p);
    (*fil_percep_laneline_).clear();
    FilterPercpLaneline(lane->lane_line_list_, fil_percep_laneline_);
    if ((*fil_percep_laneline_).size() > 0) {
      percep_lanelines_.emplace_back((*fil_percep_laneline_));
    }
  }
  if (percep_lanelines_.size() == 0) {
    if (mm_params.use_valid_pecep_lane_fault) {
      ++invalid_pecep_cnt;
      if (invalid_pecep_cnt > mm_params.invalid_pecep_cnt_thr) {
        has_err_ = true;
        err_type_ = ErrorType::NO_VALID_PECEP_LANE;
      }
    }
    HLOG_ERROR << "percep_lanelines_.size() == 0 stamp:" << SETPRECISION(15)
               << ins_timestamp_;
    return;
  } else {
    invalid_pecep_cnt = 0;
  }
  HLOG_INFO << "lane macth input valid size, map:"
            << map_boundary_lines->boundary_line_.size()
            << " percep:" << percep_lanelines_.size();

  DebugPrintPoints(map_boundary_lines,
                   percep_->GetElement(PERCEPTYION_LANE_BOUNDARY_LINE),
                   "BEFORE");
  MergeMapLines(map_boundary_lines);
  DebugPrintLines(debug_lines_before_merge_, "BEFORE");
  DebugPrintLines(debug_lines_after_merge_, "AFTER");
  DebugPrintMergedLines("");
  if (map_merge_lines_.size() == 0) {
    if (mm_params.use_valid_map_lane_fault) {
      has_err_ = true;
      err_type_ = ErrorType::NO_MERGE_MAP_LANE;
    }
    HLOG_ERROR << "map_merge_lines_.size() == 0";
    return;
  }
  // 130 fault
  // if (mm_params.use_map_lane_match_fault) {
  //   checkIsGoodMatchFCbyLine(T_fc);
  // }
  // 125 fault
  // if (mm_params.use_fc_offset_onelane_fault) {
  //   DetectfcOffsetOneLane(hd_map, perception, T_fc, percep_points,
  //                         nearest_map_points);
  // }
  // select lanelines connection
  LaneLineConnect();
  if (line_match_pairs_.size() == 0) {
    HLOG_ERROR << "line_match_pairs_.size() == 0";
    return;
  }
  // select point pairs
  ConnectPoint(false);
  DebugPrintPair(debug_points_, ins_timestamp_, "BEFORE");
}

// void MatchLaneLine::DetectfcOffsetOneLane(
//     const HDMap &hd_map,
//     const std::shared_ptr<RoadMarkingPerception> &perception, const SE3
//     &T_W_V, VP &percep_points, VP &nearest_map_points) {
//   SE3 T_V_W = T_W_V.inverse();
//   percep_points.clear();
//   nearest_map_points.clear();
//   static int offset_err_cnt = 0;
//   VP map_fit_points;
//   std::vector<VP> map_edge_format;
//   // get perception lane lines
//   auto percep_element =
//   perception->getElement(PERCEPTYION_LANE_BOUNDARY_LINE); if
//   (percep_element.size() == 0) {
//     return;
//   }
//   std::shared_ptr<RoadMarkingPerceptionLanelineList> percep_lines =
//       std::static_pointer_cast<RoadMarkingPerceptionLanelineList>(
//           percep_element[0]);
//   auto ori_percep_lines = percep_lines->_lane_line_list;
//   if (ori_percep_lines.size() == 0) {
//     return;
//   }
//   // get perception lane lines points
//   for (const auto &percep_line : ori_percep_lines) {
//     if (percep_line->id() != static_cast<size_t>(PercepLineType::L_LINE) &&
//         percep_line->id() != static_cast<size_t>(PercepLineType::R_LINE) &&
//         percep_line->id() != static_cast<size_t>(PercepLineType::LL_LINE) &&
//         percep_line->id() != static_cast<size_t>(PercepLineType::RR_LINE)) {
//       continue;
//     }
//     if (percep_line->curve_vehicle_coord_._confidence <
//         mm_params.lane_confidence_thre) {
//       continue;
//     }
//     double x = std::max(2.0, static_cast<double>(percep_line->min()));
//     if (percep_line->isIn(x)) {
//       V3 point(x, 1 * percep_line->y(x), 0);
//       if (fabs(point(1)) < mm_params.offset_maxlane) {
//         percep_points.emplace_back(point);
//       }
//     }
//   }
//   if (percep_points.size() == 0) {
//     return;
//   }
//   // get map boundary lane lines
//   auto map_elment =
//       hd_map.getElement(hozon::mp::loc::HD_MAP_LANE_BOUNDARY_LINE);
//   if (map_elment == nullptr) {
//     return;
//   }
//   double fit_min_dis = mm_params.offset_maplane_min_dis;
//   double fit_max_dis = mm_params.offset_maplane_max_dis;
//   if (map_merge_lines_.size() != 0) {
//     // get map boundary lane lines fit points
//     for (auto &line : map_merge_lines_) {
//       const auto &map_points = line.second;
//       for (double dis = fit_min_dis; dis <= fit_max_dis; ++dis) {
//         V3 map_fit_point(0, 0, 0);
//         bool flag = get_fit_points(map_points, dis, map_fit_point, T_W_V);
//         if (flag) {
//           map_fit_points.emplace_back(map_fit_point);
//         }
//       }
//     }
//   }
//   // get map road edge
//   map_elment = hd_map.getElement(hd_map::HD_MAP_ROAD_EDGE);
//   if (map_elment == nullptr) {
//     return;
//   }
//   std::shared_ptr<hd_map::MapRoadEdge> map_edges =
//       std::static_pointer_cast<hd_map::MapRoadEdge>(map_elment);
//   if (map_edges->_edge_line.size() != 0) {
//     // format road edge point and get fit point
//     for (auto &line : map_edges->_edge_line) {
//       VP temp;
//       for (auto &edge_point : line.second._control_point) {
//         temp.emplace_back(edge_point._point);
//       }
//       map_edge_format.emplace_back(temp);
//     }
//     for (auto &line_points : map_edge_format) {
//       for (double dis = fit_min_dis; dis <= fit_max_dis; ++dis) {
//         V3 map_fit_point(0, 0, 0);
//         bool flag = get_fit_points(line_points, dis, map_fit_point, T_W_V);
//         if (flag) {
//           map_fit_points.emplace_back(map_fit_point);
//         }
//       }
//     }
//   }

//   if (map_fit_points.size() == 0) {
//     return;
//   }
//   // compute max err between map lane lines and perception lane lines
//   double max_error = 0;
//   for (auto &percep_point : percep_points) {
//     double err = FLT_MAX;
//     V3 nearest_map_point;
//     for (auto &map_point : map_fit_points) {
//       double err_temp = fabs(map_point.y() - percep_point.y());
//       if (err_temp < err) {
//         err = err_temp;
//         nearest_map_point = map_point;
//         nearest_map_point = T_W_V * nearest_map_point;
//       }
//     }
//     max_error += err;
//     nearest_map_points.emplace_back(nearest_map_point);
//     percep_point = T_W_V * percep_point;
//   }

//   if (max_error > mm_params.offset_onelane_max_err) {
//     ++offset_err_cnt;
//     HLOG_ERROR << "fault(125):max_error: " << max_error;
//   } else {
//     offset_err_cnt = 0;
//   }
//   if (offset_err_cnt > mm_params.offset_onelane_cnt) {
//     err_type_ = ERROR_TYPE::OFFSET_ONELANE;
//   }
// }

// void MatchLaneLine::GetNearestPercepLinesId(
//     const std::list<PerceptionLanelinePtr> &lines, int *const left_id_ptr,
//     int *const right_id_ptr) {
//   if (!left_id_ptr || !right_id_ptr || lines.empty()) {
//     return;
//   }

//   *left_id_ptr = -1 * static_cast<int>(PercepLineType::UNKOWN);
//   *right_id_ptr = static_cast<int>(PercepLineType::UNKOWN);
//   for (const auto &line : lines) {
//     if (line->Id() <= 0 && line->Id() > *left_id_ptr) {
//       *left_id_ptr = line->Id();
//     }
//     if (line->Id() > 0 && line->Id() < *right_id_ptr) {
//       *right_id_ptr = line->Id();
//     }
//   }
//   if (*left_id_ptr < static_cast<int>(PercepLineType::LL_LINE)) {
//     *left_id_ptr = static_cast<int>(PercepLineType::UNKOWN);
//   }
//   if (*right_id_ptr > static_cast<int>(PercepLineType::RR_LINE)) {
//     *right_id_ptr = static_cast<int>(PercepLineType::UNKOWN);
//   }
// }

// void MatchLaneLine::CalLinesMinDist(const PerceptionLanelinePtr &percep,
//                                     const SE3 &T_fc, double *const near,
//                                     double *const far, const double
//                                     &last_dis) {
//   if (!percep || !near || !far) {
//     return;
//   }

//   double min_y_near = DOUBLE_MAX;
//   double min_y_far = DOUBLE_MAX;
//   V3 anchor_pt0(mm_params.near_dis, 0, 0);
//   V3 anchor_pt1(last_dis, 0, 0);

//   for (const auto &map_line : map_merge_lines_) {
//     const auto line_idx = map_line.first;
//     const auto &map_points = map_line.second;
//     V3 v_p_near(0, 0, 0);
//     V3 v_p_far(0, 0, 0);
//     bool flag_fit0 = get_fit_points(
//         map_points, std::max(2.0, static_cast<double>(percep->min())),
//         anchor_pt0, T_fc);
//     if (flag_fit0) {
//       v_p_near = anchor_pt0;
//     }
//     bool flag_fit1 =
//         get_fit_points(map_points, anchor_pt1.x(), anchor_pt1, T_fc);
//     if (flag_fit1) {
//       v_p_far = anchor_pt1;
//     }

//     double y_near = 0.0;
//     double y_far = 0.0;
//     if (flag_fit0 && percep->isIn(v_p_near.x())) {
//       y_near = percep->y(v_p_near.x()) - v_p_near.y();
//       if (fabs(y_near) < fabs(min_y_near)) {
//         min_y_near = y_near;
//       }
//     }
//     if (flag_fit1 && percep->isIn(v_p_far.x())) {
//       y_far = percep->y(v_p_far.x()) - v_p_far.y();
//       if (fabs(y_far) < fabs(min_y_far)) {
//         min_y_far = y_far;
//       }
//     }
//   }
//   if (fabs(fabs(min_y_near) - DOUBLE_MAX) < 1e-8) {
//     min_y_near = 0.0;
//   }
//   if (fabs(fabs(min_y_far) - DOUBLE_MAX) < 1e-8) {
//     min_y_far = 0.0;
//   }

//   *near = min_y_near;
//   *far = min_y_far;
// }

// void MatchLaneLine::CheckIsGoodMatchFcByLine(const SE3 &T_fc) {
//   if (percep_lanelines_.empty()) {
//     HLOG_ERROR << "can't get filtered perception lanes!";
//     return;
//   }

//   const auto &fil_line_list = percep_lanelines_.back();
//   if (fil_line_list.empty()) {
//     if (mm_params.use_valid_pecep_lane_fault) {
//       has_err_ = true;
//       err_type_ = ErrorType::NO_VALID_PECEP_LANE;
//     }
//     return;
//   }
//   HLOG_INFO << "fil_line_list.size():" << fil_line_list.size();

//   int left_id = -1 * static_cast<int>(PercepLineType::UNKOWN);
//   int right_id = static_cast<int>(PercepLineType::UNKOWN);

//   // find both side nearest perception line, max level LL or RR
//   GetNearestPercepLinesId(fil_line_list, &left_id, &right_id);
//   HLOG_INFO << "check line by fc got percep line id:" << left_id << ","
//            << right_id;
//   if (left_id == static_cast<int>(PercepLineType::UNKOWN) &&
//       right_id == static_cast<int>(PercepLineType::UNKOWN)) {
//     HLOG_ERROR << "check line by fc got no valid perception line";
//     return;
//   }

//   double left_dist_near_v = 0.0;
//   double left_dist_far_v = 0.0;
//   double right_dist_near_v = 0.0;
//   double right_dist_far_v = 0.0;
//   double last_dis = 0.0;
//   for (const auto &line : fil_line_list) {
//     if (line->Id() == left_id &&
//         left_id != static_cast<int>(PercepLineType::UNKOWN)) {
//       if (fabs(line->curve_vehicle_coord_._c2) > mm_params.curvature_thr) {
//         last_dis = mm_params.last_curve_dis;
//       } else {
//         last_dis = mm_params.last_straight_dis;
//       }
//       CalLinesMinDist(line, T_fc, &left_dist_near_v, &left_dist_far_v,
//                       last_dis);
//     }
//     if (line->Id() == right_id &&
//         right_id != static_cast<int>(PercepLineType::UNKOWN)) {
//       if (fabs(line->curve_vehicle_coord_._c2) > mm_params.curvature_thr) {
//         last_dis = mm_params.last_curve_dis;
//       } else {
//         last_dis = mm_params.last_straight_dis;
//       }
//       CalLinesMinDist(line, T_fc, &right_dist_near_v, &right_dist_far_v,
//                       last_dis);
//     }
//   }

//   HLOG_INFO << "left_dist_near_v:" << left_dist_near_v
//            << ", left_dist_far_v:" << left_dist_far_v
//            << ", right_dist_near_v:" << right_dist_near_v
//            << ", right_dist_far_v:" << right_dist_far_v;

//   const double left_error = (left_dist_near_v + left_dist_far_v) * 0.5;
//   const double right_error = (right_dist_near_v + right_dist_far_v) * 0.5;
//   const double global_error = (left_error + right_error) * 0.5;

//   HLOG_INFO << "left_error:" << left_error << ", right_error:" << right_error
//            << ", global_error:" << global_error;

//   bool fc_good_match_check = true;
//   bool fc_good_match_ser_check = true;

//   if (fabs(global_error) >= mm_params.map_lane_match_max &&
//       fabs(left_error) >= mm_params.line_error_normal_thr &&
//       fabs(right_error) >= mm_params.line_error_normal_thr) {
//     fc_good_match_check = false;
//   }
//   if (fabs(global_error) >= mm_params.map_lane_match_ser_max &&
//       fabs(left_error) >= mm_params.line_error_normal_thr &&
//       fabs(right_error) >= mm_params.line_error_normal_thr) {
//     fc_good_match_ser_check = false;
//   }

//   static uint32_t match_err_cnt = 0;
//   static uint32_t match_err_ser_cnt = 0;
//   if (!fc_good_match_check) {
//     ++match_err_cnt;
//   } else {
//     match_err_cnt = 0;
//   }
//   if (!fc_good_match_ser_check) {
//     ++match_err_ser_cnt;
//   } else {
//     match_err_ser_cnt = 0;
//   }

//   if (match_err_cnt > mm_params.map_lane_match_buff ||
//       match_err_ser_cnt > mm_params.map_lane_match_ser_buff) {
//     err_type_ = ErrorType::MAP_LANE_MATCH_FAIL;
//   }
// }

void MatchLaneLine::FilterPercpLaneline(
    const std::list<LaneLinePerceptionPtr> &lanelines,
    std::list<LaneLinePerceptionPtr> *const out) {
  if (out == nullptr) {
    return;
  }
  int egolane_cnt = 0;
  for (auto &line : lanelines) {
    HLOG_INFO << "percep-laneinfo " << SETPRECISION(15) << line->Id()
              << " confid|min|max: " << line->curve_vehicle_coord_.confidence_
              << " " << line->Min() << " " << line->Max()
              << " c0|c1|c2|c3: " << line->curve_vehicle_coord_.c0_ << " "
              << line->curve_vehicle_coord_.c1_ << " "
              << line->curve_vehicle_coord_.c2_ << " "
              << line->curve_vehicle_coord_.c3_ << " stamp: " << ins_timestamp_;
    if (line->Id() == static_cast<size_t>(PercepLineType::L_LINE) ||
        line->Id() == static_cast<size_t>(PercepLineType::R_LINE)) {
      egolane_cnt++;
    }
  }
  for (auto &line : lanelines) {
    if (line->Id() > 1000) {
      continue;
    }
    if (line->curve_vehicle_coord_.confidence_ <
        mm_params.lane_confidence_thre) {
      continue;
    }
    if (line->Max() - line->Min() < mm_params.perceplane_len_lowerbound) {
      HLOG_DEBUG << "short-percep-lane " << SETPRECISION(15) << line->Id()
                 << " " << ins_timestamp_;
      continue;
    }
    if (egolane_cnt == 2) {
      if (line->Id() == static_cast<size_t>(PercepLineType::L_LINE) ||
          line->Id() == static_cast<size_t>(PercepLineType::R_LINE)) {
        (*out).emplace_back(line);
      }
    } else {
      if (line->Id() == static_cast<size_t>(PercepLineType::L_LINE) ||
          line->Id() == static_cast<size_t>(PercepLineType::R_LINE) ||
          line->Id() == static_cast<size_t>(PercepLineType::RR_LINE) ||
          line->Id() == static_cast<size_t>(PercepLineType::LL_LINE)) {
        if ((line->Id() == static_cast<size_t>(PercepLineType::LL_LINE)) &&
            (!mm_params.use_ll_perceplane)) {
          continue;
        }
        (*out).emplace_back(line);
      }
    }
  }
}

void MatchLaneLine::NmsByLat(std::vector<AlternativeMapLine> *map_lines) {
  if (map_lines == nullptr) {
    return;
  }
  if ((*map_lines).size() <= 1) {
    return;
  }
  static const double min_lat_dist_thres = 0.1;
  std::sort((*map_lines).begin(), (*map_lines).end(),
            [](const AlternativeMapLine &l1, const AlternativeMapLine &l2) {
              return l1.ref_p.y() < l2.ref_p.y();
            });
  (*map_lines).emplace_back();
  std::vector<std::vector<AlternativeMapLine>> similar_lines;
  std::vector<AlternativeMapLine> lines;
  for (int i = 0; i < (*map_lines).size() - 1; i++) {
    if (fabs((*map_lines)[i].ref_p.y() - (*map_lines)[i + 1].ref_p.y()) <
        min_lat_dist_thres) {
      lines.emplace_back((*map_lines)[i]);
    } else {
      lines.emplace_back((*map_lines)[i]);
      similar_lines.emplace_back(lines);
      lines.clear();
    }
  }
  (*map_lines).clear();
  for (int i = 0; i < similar_lines.size(); i++) {
    std::sort(similar_lines[i].begin(), similar_lines[i].end(),
              [](const AlternativeMapLine &l1, const AlternativeMapLine &l2) {
                return l1.ref_p.x() < l2.ref_p.x();
              });
    (*map_lines).emplace_back(similar_lines[i][0]);
  }
}

void MatchLaneLine::MergeMapLines(
    const std::shared_ptr<MapBoundaryLine> &boundary_lines) {
  auto dist_comp_by_X = [&](V3 p1, V3 p2) {
    return (T_V_W_ * p1).x() < (T_V_W_ * p2).x();
  };

  typedef std::pair<int, V3> LineInfo;  // {idx, end_point}
  // lines, key:start_point, value: {(idx, end_point)}
  std::map<V3, std::vector<LineInfo>, PointV3Comp<V3>> lines;
  for (const auto &line : boundary_lines->boundary_line_) {
    const auto &control_points = line.second.control_point;
    // filter out virtual line
    if (line.second.line_type == MapLineType::LaneChangeVirtualLine) {
      continue;
    }
    // filter out invalid line
    if (control_points.size() <= 1) {
      continue;
    }
    auto start = control_points.front().point;
    auto end = control_points.back().point;
    // filter out control points with the same start and end data
    if (start.norm() == end.norm()) {
      continue;
    }
    auto start_v = T_V_W_ * start;
    if (start_v.x() > max_x_observe_thres_) {
      continue;
    }
    auto end_v = T_V_W_ * end;
    if (end_v.x() < min_x_observe_thres_) {
      continue;
    }
    int line_idx = line.first;
    debug_lines_before_merge_[line_idx].insert(line_idx);
    start = control_points.front().point;
    end = control_points.back().point;
    LineInfo line_info(line_idx, end);

    if (lines.find(start) == lines.end()) {
      std::vector<LineInfo> infos{line_info};
      lines.emplace(std::make_pair(start, infos));
    } else {
      std::vector<LineInfo> &infos = lines[start];
      infos.push_back(line_info);
    }
  }
  Graph graph(boundary_lines->boundary_line_, percep_lanelines_, T_V_W_);
  for (auto iter = lines.begin(); iter != lines.end(); ++iter) {
    V3 start = iter->first;
    auto infos = iter->second;
    for (const auto &info : infos) {
      int line_idx = info.first;
      V3 end = info.second;
      if (lines.find(end) != lines.end()) {
        auto successor_infos = lines[end];
        for (const auto &successor_info : successor_infos) {
          int successor_idx = successor_info.first;
          graph.AddEdge(line_idx, successor_idx);
        }
      } else {
        graph.AddNode(line_idx);
      }
    }
  }

  // BFS: merge only one line per branch to implement clustering
  std::vector<int> all_ids;
  graph.GetNodes(&all_ids);
  for (int idx : all_ids) {
    // get lane lines' ID belong to same line
    std::vector<int> merged_ids;
    graph.BFS(idx, &merged_ids);
    // merge
    if (merged_ids.size() > 0) {
      std::stringstream ss;
      std::vector<V3> merge_line;
      int main_idx = merged_ids[0];
      for (const auto &line_idx : merged_ids) {
        debug_lines_after_merge_[main_idx].insert(line_idx);
        ss << line_idx << " ";
        const auto &line = boundary_lines->boundary_line_[line_idx];
        for (const auto &point : line.control_point) {
          merge_line.push_back(point.point);
        }
      }
      HLOG_DEBUG << "merged_ids : " << ss.str();
      std::sort(merge_line.begin(), merge_line.end(), dist_comp_by_X);
      map_merge_lines_.emplace(main_idx, merge_line);
    }
  }
}

bool MatchLaneLine::GetFitPoints(const VP &control_poins, const double x,
                                 V3 *pt, const SE3 &T_W_V) {
  if (!pt) {
    return false;
  }
  if (control_poins.size() < 2) {
    return false;
  }
  SE3 T_V_W = T_W_V.inverse();
  V3 left(0, 0, 0), right(0, 0, 0), last = T_V_W * control_poins[0];
  for (int i = 1; i < control_poins.size(); i++) {
    V3 cur = T_V_W * control_poins[i];
    if (cur.x() >= x && last.x() < x) {
      left = last;
      right = cur;
      break;
    }
    last = cur;
  }
  if (!(left.x() < x && right.x() >= x)) {
    std::stringstream ss;
    for (auto pt : control_poins) {
      V3 cur = T_V_W * pt;
      ss << cur.x() << "," << cur.y() << "," << cur.z() << " ";
    }
    return false;
  }
  (*pt).x() = x;
  double ratio = (x - left.x()) / (right.x() - left.x());
  (*pt).y() = ratio * (right.y() - left.y()) + left.y();
  (*pt).z() = ratio * (right.z() - left.z()) + left.z();
  return true;
}

void MatchLaneLine::LaneLineConnect() {
  static const double y_dist_thres = 1.f;
  auto lane_lines = percep_->GetElement(PERCEPTYION_LANE_BOUNDARY_LINE);
  if (lane_lines.empty()) {
    HLOG_ERROR << "cant get object!";
    return;
  }
  for (auto &fil_line_list : percep_lanelines_) {
    if (fil_line_list.size() == 0) {
      if (mm_params.use_valid_pecep_lane_fault) {
        has_err_ = true;
        err_type_ = ErrorType::NO_VALID_PECEP_LANE;
      }
      HLOG_ERROR << "fil_line_list.size() == 0";
      return;
    }
    for (auto &line : fil_line_list) {
      std::vector<AlternativeMapLine> map_lines_near;
      std::vector<AlternativeMapLine> map_lines_last;
      std::pair<size_t, LaneLinePerceptionPtr> index_perception_map;
      for (const auto &map_line : map_merge_lines_) {
        const auto &line_idx = map_line.first;
        const auto &map_points = map_line.second;
        V3 v_p_near, v_p_last;
        bool flag_fit0 = GetFitPoints(
            map_points, std::max(2.0, static_cast<double>(line->Min())),
            &anchor_pt0_, T_W_V_);
        if (flag_fit0) {
          v_p_near = anchor_pt0_;
        } else {
          HLOG_INFO << "lanelineConnet search near point failed, line_idx:"
                    << line_idx;
        }
        bool flag_fit1 =
            GetFitPoints(map_points, anchor_pt1_.x(), &anchor_pt1_, T_W_V_);
        if (flag_fit1) {
          v_p_last = anchor_pt1_;
        } else {
          HLOG_INFO << "lanelineConnet search last point failed, line_idx:"
                    << line_idx;
        }
        double y_near = std::numeric_limits<double>::max();
        double y_last = std::numeric_limits<double>::max();
        if (flag_fit0 && line->IsIn(v_p_near.x())) {
          y_near = fabs(line->Y(v_p_near.x()) - v_p_near.y());
        }
        if (flag_fit1 && line->IsIn(v_p_last.x())) {
          y_last = fabs(line->Y(v_p_last.x()) - v_p_last.y());
        }
        if (y_near < y_dist_thres) {
          map_lines_near.emplace_back(v_p_near, line_idx);
        }
        if (y_last < y_dist_thres && !IsPercepLineCurve(line)) {
          map_lines_last.emplace_back(v_p_last, line_idx);
        }
      }
      if (map_lines_near.size() == 0) {
        continue;
      }
      NmsByLat(&map_lines_near);
      NmsByLat(&map_lines_last);
      std::vector<int> jacent_map_line_idxs;
      for (const auto &ele : map_lines_near) {
        if (std::find(jacent_map_line_idxs.begin(), jacent_map_line_idxs.end(),
                      ele.idx) == jacent_map_line_idxs.end()) {
          jacent_map_line_idxs.emplace_back(ele.idx);
        }
      }
      for (const auto &ele : map_lines_last) {
        if (std::find(jacent_map_line_idxs.begin(), jacent_map_line_idxs.end(),
                      ele.idx) == jacent_map_line_idxs.end()) {
          jacent_map_line_idxs.emplace_back(ele.idx);
        }
      }
      if (jacent_map_line_idxs.size() > 1) {
        FilterMapLane(&jacent_map_line_idxs, line);
      }
      if (jacent_map_line_idxs.size() > 0) {
        line_match_pairs_.emplace_back(jacent_map_line_idxs, line);
      }
    }
    if (line_match_pairs_.size() < 1) {
      lanelineconnect_ = false;
    } else {
      lanelineconnect_ = true;
    }
  }
}

void MatchLaneLine::MatchPoint2Line(const std::vector<int> &map_line_idxs,
                                    const LaneLinePerceptionPtr &pecep,
                                    const double &min_match_x,
                                    const double &max_match_x,
                                    const double &sample_interval,
                                    const bool &is_good_check) {
  std::vector<V3> percep_points;
  for (double x = min_match_x; x <= max_match_x; x += sample_interval) {
    if (!pecep->IsIn(x)) {
      continue;
    }
    percep_points.emplace_back(x, pecep->Y(x), 0);
  }
  std::vector<std::vector<V3>> map_points;
  for (auto &idx : map_line_idxs) {
    const auto &control_points = map_merge_lines_[idx];
    std::vector<V3> points;
    for (int i = 0, j = 1; i < control_points.size() - 1; i++, j++) {
      const auto &map_point = control_points[i];
      const auto &map_point_next = control_points[j];
      double norm_length = (map_point_next - map_point).norm();
      auto norm = (map_point_next - map_point) / norm_length;
      for (double length = 0; length < norm_length; length += sample_interval) {
        double min_match_y = DOUBLE_MAX;
        auto mp = map_point + norm * length;
        auto mp_v = T_V_W_ * mp;
        if (mp_v.x() < min_match_x || mp_v.x() > max_match_x) {
          continue;
        }
        points.emplace_back(mp_v);
      }
    }
    if (points.size() > 0) {
      map_points.emplace_back(points);
    }
  }
  for (const auto &pp : percep_points) {
    double min_match_y = max_match_y_thres;
    V3 mp_best = V3::Zero();
    for (int idx = 0; idx < map_points.size(); idx++) {
      for (const auto &mp : map_points[idx]) {
        if (mp.x() - pp.x() > 1) {
          break;
        }
        if (fabs(pp.x() - mp.x()) < sample_interval) {
          double y = fabs(pp.y() - mp.y());
          if (y < min_match_y) {
            min_match_y = y;
            mp_best = mp;
          }
        }
      }
    }
    if (!mp_best.isZero()) {
      V3 pp_best = V3(mp_best.x(), pecep->Y(mp_best.x()), mp_best.z());
      V3 mp_w = T_W_V_ * mp_best;
      V3 pp_w = T_W_V_ * pp_best;
      float weight = cos(mm_params.solve_weight * (pp_best.x() - min_match_x) /
                         (max_match_x - min_match_x)) *
                     mm_params.solve_point2line_base;
      PercepLineType cur_type = static_cast<PercepLineType>(pecep->Id());
      if (cur_type == PercepLineType::LL_LINE ||
          cur_type == PercepLineType::RR_LINE) {
        weight *= 0.75;
      }
      match_pairs_.emplace_back(mp_w, pp_best, cur_type, weight);
      debug_points_.emplace_back(pp_w);
      debug_points_.emplace_back(mp_w);
    }
  }
}

void MatchLaneLine::MatchLine2Line(const int &map_line_idx,
                                   const LaneLinePerceptionPtr &pecep,
                                   const double &min_match_x,
                                   const double &max_match_x,
                                   const double &sample_interval,
                                   const bool &is_good_check) {
  const auto &control_points = map_merge_lines_[map_line_idx];
  for (int i = 0, j = 1; i < control_points.size() - 1; i++, j++) {
    const auto &map_point = control_points[i];
    auto map_point_next = control_points[j];
    double norm_length = (map_point_next - map_point).norm();
    auto norm = (map_point_next - map_point) / norm_length;
    for (double length = 0; length < norm_length; length += sample_interval) {
      auto map = map_point + norm * length;
      auto map_pv = T_V_W_ * map;
      // drop map points, which x() > 20 or x() <0
      if (map_pv.x() < min_match_x || map_pv.x() > max_match_x ||
          (!pecep->IsIn(map_pv.x()))) {
        continue;
      }
      auto perception_pv = V3(map_pv.x(), pecep->Y(map_pv.x()), map_pv.z());
      auto perception_pw = T_W_V_ * perception_pv;
      if (fabs(map_pv.y() - perception_pv.y()) > max_match_y_thres) {
        continue;
      }
      float weight = cos(mm_params.solve_weight * (map_pv.x() - min_match_x) /
                         (max_match_x - min_match_x));
      PercepLineType cur_type = static_cast<PercepLineType>(pecep->Id());
      if (cur_type == PercepLineType::LL_LINE ||
          cur_type == PercepLineType::RR_LINE) {
        weight *= 0.75;
      }
      match_pairs_.emplace_back(map, perception_pv, cur_type, weight);
      debug_points_.emplace_back(perception_pw);
      debug_points_.emplace_back(map);
    }
  }
}

bool MatchLaneLine::CheckIsGoodMatch(const SE3 &T) {
  match_pairs_.clear();
  debug_points_.clear();
  T_W_V_ = T;
  T_V_W_ = T.inverse();
  static const double ll_max_lane_match_x = 15.f;
  static const double ll_min_lane_match_x = 2.f;
  static const double ll_match_sample_interval = 0.5;
  static const double pl_max_lane_match_x = 17.5f;
  static const double pl_min_lane_match_x = 2.f;
  static const double pl_match_sample_interval = 0.5;
  for (auto &p : line_match_pairs_) {
    if (p.map_line_idxs.size() == 1) {
      HLOG_INFO << "p.map_line_idxs.size() == 1";
      MatchLine2Line(p.map_line_idxs[0], p.pecep_line, ll_min_lane_match_x,
                     ll_max_lane_match_x, ll_match_sample_interval, true);
    } else if (p.map_line_idxs.size() > 1) {
      HLOG_INFO << "p.map_line_idxs.size() : " << p.map_line_idxs.size();
      MatchPoint2Line(p.map_line_idxs, p.pecep_line, pl_min_lane_match_x,
                      pl_max_lane_match_x, pl_match_sample_interval, true);
    }
  }
  DebugPrintPair(debug_points_, ins_timestamp_, "AFTER");
  int match_pair_cnt = 0;
  double match_y_total = 0.f;
  for (const auto &p : match_pairs_) {
    const auto &pp_v = p.pecep_pv;
    if (pp_v.x() < pl_max_lane_match_x) {
      V3 mp_v = T_V_W_ * p.map_pw;
      // match_y_total += fabs(mp_v.y() - pp_v.y());
      match_y_total += (mp_v.y() - pp_v.y());
      match_pair_cnt++;
    }
  }
  if (match_pair_cnt != 0) {
    double avg_match_y = fabs(match_y_total / match_pair_cnt * 1.f);
    HLOG_INFO << " avg_match_y  : " << SETPRECISION(15) << ins_timestamp_ << " "
              << avg_match_y << " ";
    if (avg_match_y < 0.06) {
      return true;
    } else {
      return false;
    }
  } else {
    HLOG_ERROR << "match_pair_cnt == 0";
    return false;
  }
}

bool MatchLaneLine::IsPercepLineCurve(const LaneLinePerceptionPtr &line) {
  static const double curvature_thres = 500.f;
  if (line->Id() != static_cast<size_t>(PercepLineType::L_LINE) &&
      line->Id() != static_cast<size_t>(PercepLineType::R_LINE)) {
    return false;
  }
  double radius = fabs(1 / (2 * line->curve_vehicle_coord_.c2_));
  return radius < curvature_thres;
}

void MatchLaneLine::ConnectPoint(const bool &is_good_check) {
  match_pairs_.clear();
  debug_points_.clear();
  static const double ll_max_lane_match_x = 30.f;
  static const double ll_min_lane_match_x = 2.f;
  static const double ll_match_sample_interval = 0.5;
  static const double pl_max_lane_match_x = 35.f;
  static const double pl_min_lane_match_x = 2.f;
  static const double pl_match_sample_interval = 0.5;
  for (auto &p : line_match_pairs_) {
    if (p.map_line_idxs.size() == 1) {
      HLOG_INFO << "p.map_line_idxs.size() == 1";
      MatchLine2Line(p.map_line_idxs[0], p.pecep_line, ll_min_lane_match_x,
                     ll_max_lane_match_x, ll_match_sample_interval, false);
    } else if (p.map_line_idxs.size() > 1) {
      HLOG_INFO << "p.map_line_idxs.size() : " << p.map_line_idxs.size();
      MatchPoint2Line(p.map_line_idxs, p.pecep_line, pl_min_lane_match_x,
                      pl_max_lane_match_x, pl_match_sample_interval, false);
    }
  }
  DebugPrintMapPpMatch();
}

void MatchLaneLine::FilterMapLane(std::vector<int> *mlane_ids,
                                  const LaneLinePerceptionPtr &plane) {
  if (mlane_ids == nullptr) {
    return;
  }
  std::vector<std::list<int>> groups;
  GroupByXProject(*mlane_ids, &groups);
  (*mlane_ids).clear();
  for (const auto &group : groups) {
    if (group.size() == 1) {
      (*mlane_ids).emplace_back(group.front());
      continue;
    }
    int best_mlane_id;
    if (group.size() > 1 && SelectBestMapLane(group, plane, &best_mlane_id)) {
      (*mlane_ids).emplace_back(best_mlane_id);
    }
  }
}

void MatchLaneLine::GroupByXProject(const std::vector<int> &mlane_ids,
                                    std::vector<std::list<int>> *groups) {
  if (!groups) {
    return;
  }
  auto comp = [&](V3 p1, V3 p2) {
    return (T_V_W_ * p1).x() < (T_V_W_ * p2).x();
  };
  typedef std::pair<double, double> Seg;  // {idx, end_point}
  // lines, key:start_point, value: {(idx, end_point)}
  std::unordered_map<int, Seg> segs;
  double min_x, max_x, left, right, interval_last, interval_cur, interval_merge;
  int seg_id;
  for (const int &mlane_id : mlane_ids) {
    const auto &control_points = map_merge_lines_[mlane_id];
    if (control_points.size() < 2) {
      continue;
    }
    min_x = (T_V_W_ * (*std::min_element(control_points.begin(),
                                         control_points.end(), comp)))
                .x();
    max_x = (T_V_W_ * (*std::max_element(control_points.begin(),
                                         control_points.end(), comp)))
                .x();
    seg_id = -1;
    for (auto &seg : segs) {
      left = seg.second.first;
      right = seg.second.second;
      interval_merge = std::max({min_x, max_x, left, right}) -
                       std::min({min_x, max_x, left, right});
      interval_last = right - left;
      interval_cur = max_x - min_x;
      if (interval_last + interval_cur - interval_merge > 1) {
        seg_id = seg.first;
        seg.second.first = std::min(min_x, left);
        seg.second.second = std::max(max_x, right);
        break;
      }
    }
    if (seg_id >= 0) {
      (*groups)[seg_id].emplace_back(mlane_id);
    } else {
      segs[segs.size()] = std::make_pair(min_x, max_x);
      (*groups).emplace_back(std::list<int>{mlane_id});
    }
  }
}

bool MatchLaneLine::SelectBestMapLane(const std::list<int> &mline_ids,
                                      const LaneLinePerceptionPtr &pline,
                                      int *best_mline_id) {
  if (best_mline_id == nullptr) {
    return false;
  }
  bool success = false;
  double sum_dist, mean_dist, best_dist = std::numeric_limits<double>::max();
  int cnt;
  for (const int &mline_id : mline_ids) {
    sum_dist = 0;
    mean_dist = 0;
    cnt = 0;
    const auto &control_points = map_merge_lines_[mline_id];
    if (control_points.size() < 2) {
      continue;
    }
    for (int i = 0, j = 1; i < control_points.size() - 1; i++, j++) {
      const auto &cur_p_w = control_points[i];
      const auto &next_p_w = control_points[j];
      double norm_length = (next_p_w - cur_p_w).norm();
      auto norm = (next_p_w - cur_p_w) / norm_length;
      for (double length = 0; length < norm_length; length += 0.5) {
        auto p_w = cur_p_w + norm * length;
        auto p_v = T_V_W_ * p_w;
        if (pline->IsIn(p_v.x())) {
          sum_dist += fabs(pline->Y(p_v.x()) - p_v.y());
          cnt += 1;
        }
      }
    }
    if (cnt > 0) {
      mean_dist = sum_dist / cnt;
      if (best_dist > mean_dist) {
        best_dist = mean_dist;
        (*best_mline_id) = mline_id;
        success = true;
      }
    }
  }
  return success;
}

void MatchLaneLine::DebugPrintPoints(
    const std::shared_ptr<MapBoundaryLine> &boundary_lines,
    const std::vector<PerceptionElement::Ptr> &percep_lines, std::string KEY) {
  if (!mm_params.debug_plot_info) {
    return;
  }
  std::stringstream ss;
  for (const auto &line : boundary_lines->boundary_line_) {
    const auto &control_points = line.second.control_point;
    // filter invalid line
    if (control_points.size() <= 1) {
      continue;
    }
    auto start = control_points.front().point;
    auto end = control_points.back().point;
    if (start.norm() == end.norm()) {
      continue;
    }
    auto start_v = T_V_W_ * start;
    if (start_v.x() > max_x_observe_thres_) {
      continue;
    }
    auto end_v = T_V_W_ * end;
    if (end_v.x() < min_x_observe_thres_) {
      continue;
    }
    int pt_idx = 0;
    ss << "ln_idx:" << line.first << "|ln_tp:" << line.second.line_type
       << "|pts:(";
    for (auto pt : control_points) {
      auto pt_v = T_V_W_ * pt.point;
      ss << "pt:" << pt_idx++ << "," << pt_v.x() << "," << pt_v.y() << ","
         << pt_v.z() << ";";
    }
    ss << ") ";
  }
  HLOG_DEBUG << KEY << " debug_map stamp:" << SETPRECISION(15) << ins_timestamp_
             << " " << ss.str();
  ss.str("");
  for (auto &p : percep_lines) {
    auto lane = std::static_pointer_cast<PerceptionLaneLineList>(p);
    for (auto &line : lane->lane_line_list_) {
      int ln_idx = line->Id();
      double c0 = line->curve_vehicle_coord_.c0_;
      double c1 = line->curve_vehicle_coord_.c1_;
      double c2 = line->curve_vehicle_coord_.c2_;
      double c3 = line->curve_vehicle_coord_.c3_;
      double min_ = line->Min(), max_ = line->Max();
      ss << "ln_idx:" << ln_idx << "|min:" << min_ << "|max:" << max_
         << "|coords:" << c0 << "," << c1 << "," << c2 << "," << c3 << " ";
    }
  }
  HLOG_DEBUG << KEY << " debug_percep stamp:" << SETPRECISION(15)
             << ins_timestamp_ << " " << ss.str();
}

void MatchLaneLine::DebugPrintLines(
    const std::unordered_map<int, std::unordered_set<int>> lines,
    std::string KEY) {
  if (!mm_params.debug_plot_info) {
    return;
  }
  std::stringstream ss;
  for (auto line : lines) {
    ss << "[" << line.first << ":(";
    for (int sub_line : line.second) {
      ss << sub_line << ",";
    }
    ss << ")] ";
  }
  HLOG_DEBUG << " debug_lines " << KEY << " :" << ss.str();
}
void MatchLaneLine::DebugPrintPair(const std::vector<V3> &pairs,
                                   const double &stamp,
                                   const std::string &KEY) {
  if (!mm_params.debug_plot_info) {
    return;
  }
  std::stringstream ss;
  for (int i = 0; i < pairs.size(); i += 2) {
    V3 pp_v = T_V_W_ * pairs[i];
    V3 map_v = T_V_W_ * pairs[i + 1];
    ss << "pp:" << pp_v.x() << "," << pp_v.y() << "," << pp_v.z();
    ss << "|mm:" << map_v.x() << "," << map_v.y() << "," << map_v.z() << ";";
  }
  HLOG_DEBUG << KEY << " debug_pairs stamp:" << SETPRECISION(15) << stamp << " "
             << ss.str();
}

void MatchLaneLine::DebugPrintMergedLines(const std::string &KEY) {
  if (!mm_params.debug_plot_info) {
    return;
  }
  std::stringstream ss;
  for (auto line : map_merge_lines_) {
    int idx = line.first;
    ss << "idx:" << idx << "|(";
    for (auto p_w : line.second) {
      V3 p_v = T_V_W_ * p_w;
      ss << "pt:" << p_v.x() << "," << p_v.y() << "," << p_v.z() << ";";
    }
    ss << ") ";
  }
  HLOG_DEBUG << KEY << " merge_lines stamp:" << SETPRECISION(15)
             << ins_timestamp_ << " " << ss.str();
}

void MatchLaneLine::DebugPrintMapPpMatch() {
  if (!mm_params.debug_plot_info) {
    return;
  }
  std::stringstream ss;
  for (auto pair : line_match_pairs_) {
    LaneLinePerceptionPtr pp_lane = pair.pecep_line;
    int pp_id = pp_lane->Id();
    double c0 = pp_lane->curve_vehicle_coord_.c0_;
    double c1 = pp_lane->curve_vehicle_coord_.c1_;
    double c2 = pp_lane->curve_vehicle_coord_.c2_;
    double c3 = pp_lane->curve_vehicle_coord_.c3_;
    double min_ = pp_lane->Min(), max_ = pp_lane->Max();
    ss << "[id:" << pp_id << ";min:" << min_ << ";max:" << max_
       << ";coords:" << c0 << "," << c1 << "," << c2 << "," << c3 << "]_[";
    auto mm_lane_ids = pair.map_line_idxs;
    for (int id : mm_lane_ids) {
      const auto &control_points = map_merge_lines_[id];
      ss << "id:" << id << "|(";
      for (auto p_w : control_points) {
        V3 p_v = T_V_W_ * p_w;
        ss << "pt:" << p_v.x() << "," << p_v.y() << "," << p_v.z() << ";";
      }
      ss << ")^";
    }
    ss << "] ";
  }
  HLOG_DEBUG << " map_pp_match stamp:" << SETPRECISION(15) << ins_timestamp_
             << " " << ss.str();
}

}  // namespace loc
}  // namespace mp
}  // namespace hozon
