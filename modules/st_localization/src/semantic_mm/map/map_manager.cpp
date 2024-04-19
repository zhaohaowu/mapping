/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */
#include "semantic_mm/map/map_manager.hpp"

#include <algorithm>
#include <limits>
#include <list>
#include <random>
#include <set>

#include "common/coordinate_converter.hpp"
#include "localization/common/log.hpp"
#include "semantic_mm/base/lane_line.hpp"
#include "semantic_mm/base/pole.hpp"
#include "semantic_mm/base/traffic_sign.hpp"
#include "semantic_mm/common/configure.hpp"
#include "semantic_mm/common/math_tools.hpp"

namespace senseAD {
namespace localization {
namespace smm {

adLocStatus_t MapManager::SwitchOriginProc() {
  // convert pose
  CoordinateConverter::GetInstance()->ConvertENUToNewOrigin(
      pose_.translation(), &pose_.translation());

  // convert laneline points
  for (auto& laneline : lanelines_) {
    for (auto& segment : laneline.second->GetLineSegments()) {
      for (auto& point : segment->GetPoints()) {
        Eigen::Vector3d pt(point.x, point.y, point.z);
        CoordinateConverter::GetInstance()->ConvertENUToNewOrigin(pt, &pt);
        point = Point3D_t(pt.x(), pt.y(), pt.z());
      }
    }
  }

  if (param_.smm_param.enable_percept_semantic) {
    // convert traffic_signs points
    for (auto& sign : traffic_signs_) {
      auto& center = sign.second->GetCenter();
      Eigen::Vector3d pt(center.x, center.y, center.z);
      CoordinateConverter::GetInstance()->ConvertENUToNewOrigin(pt, &pt);
      center = Point3D_t(pt.x(), pt.y(), pt.z());
    }

    // convert poles points
    for (auto& pole : poles_) {
      auto& bt_point = pole.second->GetBottomPoint();
      Eigen::Vector3d pt(bt_point.x, bt_point.y, bt_point.z);
      CoordinateConverter::GetInstance()->ConvertENUToNewOrigin(pt, &pt);
      bt_point = Point3D_t(pt.x(), pt.y(), pt.z());
    }
  }

  return LOC_SUCCESS;
}

adLocStatus_t MapManager::UpdateMap(const RoadStructure::Ptr& road_structure,
                                    const SE3d& pose) {
  // check if update needed
  // TODO(xxx): incremental update when vehicle is close to map boundary
  double update_map_distance = 20.0;
  SE3d pose_delta = pose_.inverse() * pose;
  if (road_structure_ != nullptr &&
      pose_delta.translation().norm() < update_map_distance) {
    return LOC_SUCCESS;
  }
  pose_ = pose;
  road_structure_ = road_structure;
  id_counter_ = 0;

  // compute ground height
  SE3d T_veh_ground = Configure::GetInstance()->GetTGround2Vehicle();
  SE3d T_world_ground = pose_ * T_veh_ground;
  ground_height_ = T_world_ground.translation().z();

  // process for each semantic type
  auto status = ProcessMapLaneLine();
  if (status != LOC_SUCCESS) {
    LC_LERROR(MAPMANAGER) << "process map laneline failed.";
    return LOC_LOCALIZATION_ERROR;
  }
  if (!param_.smm_param.enable_percept_semantic) return LOC_SUCCESS;

  status = ProcessMapTrafficSign();
  if (status != LOC_SUCCESS) {
    LC_LERROR(MAPMANAGER) << "process map traffic sign failed.";
    return LOC_LOCALIZATION_ERROR;
  }

  status = ProcessMapPole();
  if (status != LOC_SUCCESS) {
    LC_LERROR(MAPMANAGER) << "process map pole failed.";
    return LOC_LOCALIZATION_ERROR;
  }

  return LOC_SUCCESS;
}

adLocStatus_t MapManager::UpdateLocalMap(
    const SE3d& pose, const Eigen::Matrix<double, 6, 6>& pose_cov,
    const std::pair<double, double>& lateral_range,
    const std::pair<double, double>& longitudinal_range) {
  const auto& param = Configure::GetInstance()->GetLocalizationSMMParam();
  double new_ll_threshold = 2.0 * param.map_laneline_sample_dist;
  static auto PointInRange =
      [](const std::pair<double, double>& lateral_range,
         const std::pair<double, double>& longitudinal_range,
         const Point3D_t& pt) {
        return pt.x > longitudinal_range.first &&
               pt.x < longitudinal_range.second && pt.y > lateral_range.first &&
               pt.y < lateral_range.second;
      };

  // convert laneline points to query frame coordinate
  SE3d Tmain_w = pose.inverse();
  local_lanelines_.clear();
  for (const auto& item : lanelines_) {
    // get local laneline candidates
    id_t id = item.first;
    const auto& laneline_points =
        item.second->GetLineSegments().front()->GetPoints();
    std::vector<std::vector<Point3D_t>> local_points_candidates;
    Point3D_t last_pt;
    for (const auto& pt : laneline_points) {
      Point3D_t new_pt = Tmain_w * pt;
      if (PointInRange(lateral_range, longitudinal_range, new_pt)) {
        if (local_points_candidates.empty() ||
            (new_pt - last_pt).Norm2D() > new_ll_threshold) {
          local_points_candidates.emplace_back(std::vector<Point3D_t>{new_pt});
        } else {
          local_points_candidates.back().emplace_back(new_pt);
        }
        last_pt = new_pt;
      }
    }
    if (local_points_candidates.empty()) continue;

    // pick laneline with smallest heading w.r.t. vehicle
    std::vector<Point3D_t>& local_points = local_points_candidates.front();
    if (local_points_candidates.size() > 1) {
      double min_ll_heading = std::numeric_limits<double>::max();
      Point3D_t origin(0, 0, 0);
      for (auto& cand : local_points_candidates) {
        size_t i, j;
        if (!BinarySearchTwoNearestPoints(origin, cand, &i, &j)) continue;
        Point3D_t ll_direction = cand[j] - cand[i];
        double ll_heading =
            std::atan2(std::fabs(ll_direction.y), std::fabs(ll_direction.x));
        if (ll_heading < min_ll_heading) {
          min_ll_heading = ll_heading;
          local_points = cand;
        }
      }
    }

    // convert raw points to LaneLine
    if (local_points.size() > 1) {
      // check map line points sort order
      if (local_points.front().x > local_points.back().x) {
        std::reverse(local_points.begin(), local_points.end());
      }

      const auto& line_seg_from = item.second->GetLineSegments().front();
      auto line_seg_cut = std::make_shared<LineSegment>(*line_seg_from);
      line_seg_cut->SetPoints(std::move(local_points));

      std::vector<LineSegment::Ptr> line_segs{line_seg_cut};
      auto ll = std::make_shared<LaneLine>(id, line_segs);
      // dont forget linked id
      ll->SetLinkedId(item.second->GetLinkedId());
      local_lanelines_.insert(std::make_pair(id, ll));
    }
  }

  if (!param_.smm_param.enable_percept_semantic) return LOC_SUCCESS;

  // traffic signs
  std::pair<double, double> long_range_tsp = {-20, 65};
  local_traffic_signs_.clear();
  for (const auto& sign_global : traffic_signs_) {
    // TODO(xxx): transform quaterion as well if supported by HD map
    Point3D_t center_local = Tmain_w * sign_global.second->GetCenter();
    if (PointInRange(lateral_range, long_range_tsp, center_local)) {
      TrafficSign::Ptr sign_local(new TrafficSign(*sign_global.second));
      sign_local->SetCenter(center_local);
      Eigen::Matrix3d local_cov;
      CovPropagate2Local2D(pose, pose_cov, center_local, &local_cov);
      sign_local->SetCenterCov(local_cov);
      local_traffic_signs_.insert(
          std::make_pair(sign_global.first, sign_local));
    }
  }

  // poles
  local_poles_.clear();
  for (const auto& pole_global : poles_) {
    // TODO(xxx): transform top point as well if supported by HD map
    Point3D_t bottom_local = Tmain_w * pole_global.second->GetBottomPoint();
    if (PointInRange(lateral_range, long_range_tsp, bottom_local)) {
      Pole::Ptr pole_local(new Pole(*pole_global.second));
      pole_local->SetBottomPoint(bottom_local);
      Eigen::Matrix3d local_cov;
      CovPropagate2Local2D(pose, pose_cov, bottom_local, &local_cov);
      pole_local->SetPointCov(local_cov);
      local_poles_.insert(std::make_pair(pole_global.first, pole_local));
    }
  }

  // TODO(xxx): support other semantic types...

  return LOC_SUCCESS;
}

adLocStatus_t MapManager::ClearMap() {
  id_counter_ = 0;
  lanelines_.clear();
  traffic_signs_.clear();
  poles_.clear();
  return LOC_SUCCESS;
}

adLocStatus_t MapManager::ClearLocalMap() {
  local_lanelines_.clear();
  local_traffic_signs_.clear();
  local_poles_.clear();
  return LOC_SUCCESS;
}

const LaneLine::PtrUMap& MapManager::GetLaneLines() const { return lanelines_; }

const LaneLine::PtrUMap& MapManager::GetLocalLaneLines() const {
  return local_lanelines_;
}

const TrafficSign::PtrUMap& MapManager::GetTrafficSigns() const {
  return traffic_signs_;
}

const TrafficSign::PtrUMap& MapManager::GetLocalTrafficSigns() const {
  return local_traffic_signs_;
}

const Pole::PtrUMap& MapManager::GetPoles() const { return poles_; }

const Pole::PtrUMap& MapManager::GetLocalPoles() const { return local_poles_; }

adLocStatus_t MapManager::CheckMapData(const SE3d Tvw,
                                       SMMEvalData* eval_data) const {
  if (!eval_data) {
    LC_LERROR(SMM) << "evaluation data object is nullptr.";
    return LOC_NULL_PTR;
  }

  int valid_line_count = 0;
  int laneline_num = 0;
  int roadside_line_num = 0;
  // check each map lane line
  for (const auto& line : lanelines_) {
    const auto& map_points =
        line.second->GetLineSegments().front()->GetPoints();
    if (map_points.empty()) continue;
    // project points from world to bird view
    // and check data quality
    double line_min_x = std::numeric_limits<double>::max();
    double line_max_x = std::numeric_limits<double>::min();
    int point_count = 0;
    for (const auto& point : map_points) {
      Point3D_t bv_point = Tvw * point;
      // ignore point out of range
      if (bv_point.x < 0.0 || bv_point.x > 100.0 || bv_point.y > 15.0 ||
          bv_point.y < -15.0)
        continue;

      ++point_count;
      if (bv_point.x > line_max_x) line_max_x = bv_point.x;
      if (bv_point.x < line_min_x) line_min_x = bv_point.x;
    }
    if (line_max_x - line_min_x > 10.0 && point_count > 5) {
      ++valid_line_count;
      LineType line_type =
          line.second->GetLineSegments().front()->GetLineType();
      if (IsLaneMarkingLine(line_type)) {
        ++laneline_num;
      } else if (IsRoadSideLine(line_type)) {
        ++roadside_line_num;
      }
    }
  }

  if (valid_line_count > 0) {
    eval_data->is_map_valid = true;
  }
  eval_data->roadline_statistics["map"] =
      std::pair<int, int>(laneline_num, roadside_line_num);

  return LOC_SUCCESS;
}

adLocStatus_t MapManager::ProcessMapLaneLine() {
  if (!road_structure_) return LOC_LOCALIZATION_ERROR;

  const auto& param = Configure::GetInstance()->GetLocalizationSMMParam();

  // find lines intersects with given range of vehicle position
  // TODO(xxx): consider vehicle heading to limit range
  Point3D_t pst = PtEigen2Inner(pose_.translation());
  double range = 100;
  std::unordered_set<size_t> line_data_idxs;
  const auto& line_datas = road_structure_->semantic_map_data.lines;
  for (size_t i = 0; i < line_datas.size(); ++i) {
    const auto& line_data = line_datas[i];
    const auto& points = line_data.line_segments[0].points;
    if (CheckLineSegmentCircleIntersection(range, pst, points.front(),
                                           points.back())) {
      line_data_idxs.insert(i);
    }
  }

  // connect line data by check front/end pt distance
  // NOTE: connected line data has multiple line segments
  std::map<id_t, LineData> connected_line_data;
  ConnectLineDataByDistance(line_data_idxs, &connected_line_data);

  // convert connected lines to inner type
  LaneLine::PtrUMap lanelines;
  for (auto& item : connected_line_data) {
    auto& line_data = item.second;
    id_t original_id = line_data.id;
    auto is_roadside = IsRoadSideLine(line_data.line_segments[0].line_type);
    auto is_dashed = IsDashedLine(line_data.line_segments[0].line_style);
    // ensure map lines at ground height
    std::vector<Point3D_t> connected_points;
    size_t total_pt_num = 0;
    for (auto& line_seg : line_data.line_segments) {
      for (auto& pt : line_seg.points) pt.z = ground_height_;
      total_pt_num += line_seg.points.size();
    }
    connected_points.reserve(total_pt_num);
    for (const auto& line_seg : line_data.line_segments) {
      original_id = std::min(original_id, line_seg.id);
      if (is_roadside != IsRoadSideLine(line_seg.line_type) ||
          is_dashed != IsDashedLine(line_seg.line_style)) {
        LC_LDEBUG(SMM) << "why connected line segs have different type/style?";
      }
      if (line_seg.points.empty()) {
        LC_LDEBUG(SMM) << "why connected line seg empty?";
        continue;
      }
      if (!connected_points.empty()) {
        auto end = connected_points.back();
        auto start = line_seg.points.front();
        if ((start - end).Norm2D() > 0.1) {
          LC_LDEBUG(SMM) << "why connected line segs are not connected?";
        }
      }
      connected_points.insert(connected_points.end(), line_seg.points.begin(),
                              line_seg.points.end());
    }

    // convert to inner type
    // id_t reassign_id = original_id;
    id_t reassign_id = id_counter_++;
    LC_LDEBUG(SMM) << "map line original/reassign id " << original_id << "->"
                   << reassign_id;
    LineSegment::Ptr line_segment = std::make_shared<LineSegment>();
    line_segment->SetId(reassign_id);
    line_segment->SetConfidence(line_data.line_segments[0].confidence);
    line_segment->SetWidth(line_data.line_segments[0].width);
    line_segment->SetLineType(line_data.line_segments[0].line_type);
    line_segment->SetLineStyle(line_data.line_segments[0].line_style);
    std::vector<Point3D_t> upsamp_points;
    UpSamplePoints(connected_points, param.map_laneline_sample_dist,
                   &upsamp_points);
    line_segment->SetPoints(std::move(upsamp_points));

    LaneLine::Ptr laneline = std::make_shared<LaneLine>();
    laneline->SetId(reassign_id);
    laneline->AddLineSegment(line_segment);
    lanelines.insert(std::make_pair(reassign_id, laneline));
  }
  lanelines_ = std::move(lanelines);

  return LOC_SUCCESS;
}

adLocStatus_t MapManager::ProcessMapLaneLineOld() {
  const auto& map_lanes = road_structure_->route_map_data.lanes;
  const auto& map_lanelinks = road_structure_->route_map_data.lane_links;
  const auto& map_lines = road_structure_->semantic_map_data.lines;

  // for record lane/lanelink/line, <element id, index in vector>
  std::map<id_t, size_t> lane_id_idx_table;
  std::map<id_t, size_t> lanelink_id_idx_table;
  std::map<id_t, size_t> line_id_idx_table;
  for (size_t i = 0; i < map_lanes.size(); ++i) {
    lane_id_idx_table.insert(std::make_pair(map_lanes[i].id, i));
  }
  for (size_t i = 0; i < map_lanelinks.size(); ++i) {
    if (map_lanelinks[i].link_type == LinkType::Continue) {
      lanelink_id_idx_table.insert(std::make_pair(map_lanelinks[i].id, i));
    }
  }
  bool map_has_roadside = false;
  for (size_t i = 0; i < map_lines.size(); ++i) {
    line_id_idx_table.insert(std::make_pair(map_lines[i].id, i));
    LineType line_type = map_lines[i].line_segments[0].line_type;
    if (IsRoadSideLine(line_type)) map_has_roadside = true;
  }

  // make roadside for inner map
  if (!map_has_roadside) {
    MakeRoadSideForInnerMap(0.0, &line_id_idx_table);
  }

  // for record linked lane group, <group index, lane id list>
  std::map<size_t, std::list<id_t>> lane_group_ids_table;
  // for record lane id mapping, <lane id, group index>
  std::map<id_t, size_t> lane_id_group_table;

  // construct linked lane group for line connection in different sections
  size_t lane_counter = 0;
  for (size_t i = 0; i < map_lanes.size(); ++i) {
    const auto& cur_lane = map_lanes[i];
    if (lane_id_group_table.count(cur_lane.id)) continue;

    // update linked lane group for current lane
    size_t cur_lane_counter = lane_counter++;
    lane_group_ids_table[cur_lane_counter].push_back(cur_lane.id);
    lane_id_group_table[cur_lane.id] = cur_lane_counter;

    // recursive search current lane linked successor and predecessor lanes
    auto next_lane = cur_lane, pre_lane = cur_lane;
    while (true) {
      const auto& succ_lanelink_ids = next_lane.successor_link_ids;
      // check lanelink size
      if (succ_lanelink_ids.size() != 1) break;
      if (!lanelink_id_idx_table.count(succ_lanelink_ids[0])) break;
      const auto& lane_link =
          map_lanelinks.at(lanelink_id_idx_table.at(succ_lanelink_ids[0]));
      // TODO(xx): check lanelink if is continue, consider other type?
      if (lane_link.link_type != LinkType::Continue) break;
      // check if to lane id is valid
      if (!lane_id_idx_table.count(lane_link.to_lane_id)) break;
      next_lane = map_lanes.at(lane_id_idx_table.at(lane_link.to_lane_id));
      // check if has exist in lane_id_group_table
      if (lane_id_group_table.count(next_lane.id)) break;
      lane_group_ids_table[cur_lane_counter].push_back(next_lane.id);
      lane_id_group_table[next_lane.id] = cur_lane_counter;
    }
    while (true) {
      auto pred_lanelink_ids = pre_lane.predecessor_link_ids;
      // check lanelink size
      if (pred_lanelink_ids.size() != 1) break;
      if (!lanelink_id_idx_table.count(pred_lanelink_ids[0])) break;
      const auto& lane_link =
          map_lanelinks.at(lanelink_id_idx_table.at(pred_lanelink_ids[0]));
      // TODO(xx): check lanelink if is continue, consider other type?
      if (lane_link.link_type != LinkType::Continue) break;
      // check if from lane id is valid
      if (!lane_id_idx_table.count(lane_link.from_lane_id)) break;
      pre_lane = map_lanes.at(lane_id_idx_table.at(lane_link.from_lane_id));
      // check if has exist in lane_id_group_table
      if (lane_id_group_table.count(pre_lane.id)) break;
      lane_group_ids_table[cur_lane_counter].push_front(pre_lane.id);
      lane_id_group_table[pre_lane.id] = cur_lane_counter;
    }
  }

  // connect lines for linked lane group
  LaneLine::PtrUMap lanelines;
  for (const auto& group_item : lane_group_ids_table) {
    const auto& lane_ids = group_item.second;
    // group lines with same line type
    std::vector<std::vector<LineData>> left_line_groups, right_line_groups;
    LineType last_left_line_type, last_right_line_type;
    LineStyle last_left_line_style, last_right_line_style;
    for (const auto& id : lane_ids) {
      const auto& lane = map_lanes.at(lane_id_idx_table.at(id));

      // left line groups
      const auto& left_line =
          map_lines.at(line_id_idx_table.at(lane.left_line_id));
      LineType left_type = left_line.line_segments[0].line_type;
      LineStyle left_style = left_line.line_segments[0].line_style;
      if (left_line_groups.empty() || left_type != last_left_line_type ||
          left_style != last_left_line_style) {
        left_line_groups.emplace_back(std::vector<LineData>{left_line});
      } else {
        left_line_groups.back().emplace_back(left_line);
      }
      last_left_line_type = left_type;
      last_left_line_style = left_style;

      // right line groups
      const auto& right_line =
          map_lines.at(line_id_idx_table.at(lane.right_line_id));
      LineType right_type = right_line.line_segments[0].line_type;
      LineStyle right_style = right_line.line_segments[0].line_style;
      if (right_line_groups.empty() || right_type != last_right_line_type ||
          right_style != last_right_line_style) {
        right_line_groups.emplace_back(std::vector<LineData>{right_line});
      } else {
        right_line_groups.back().emplace_back(right_line);
      }
      last_right_line_type = right_type;
      last_right_line_style = right_style;
    }

    // connect lines in each line group
    for (const auto& left_lines : left_line_groups) {
      LaneLine::Ptr left_laneline = std::make_shared<LaneLine>();
      left_laneline->SetId(id_counter_++);
      ConnectLaneLineForce(left_lines, &left_laneline);
      lanelines.insert(std::make_pair(left_laneline->GetId(), left_laneline));
    }

    for (const auto& right_lines : right_line_groups) {
      LaneLine::Ptr right_laneline = std::make_shared<LaneLine>();
      right_laneline->SetId(id_counter_++);
      ConnectLaneLineForce(right_lines, &right_laneline);
      lanelines.insert(std::make_pair(right_laneline->GetId(), right_laneline));
    }
  }

  // extract curb, concrete, fence, guardrail
  std::vector<LineData> roadside_lines;
  roadside_lines.reserve(map_lines.size());
  for (const auto& line : map_lines) {
    if (IsRoadSideLine(line.line_segments[0].line_type)) {
      roadside_lines.push_back(line);
    }
  }
  std::vector<LineData> connected_roadside_lines;
  ConnectLaneLineByDistance(roadside_lines, 15.0, 15.0,
                            &connected_roadside_lines);
  std::vector<std::shared_ptr<LaneLine>> roadside_lanelines;
  MakeLaneLineData(connected_roadside_lines, id_counter_, &roadside_lanelines);
  for (const auto& laneline : roadside_lanelines) {
    lanelines.insert(std::make_pair(laneline->GetId(), laneline));
  }

  // copy lanelines at roadsides to match with perception
  std::vector<LineData> copied_laneline_data;
  copied_laneline_data = connected_roadside_lines;
  for (auto& line : copied_laneline_data) {
    for (auto& line_seg : line.line_segments) {
      line_seg.line_type = LineType::LaneMarking;
      line_seg.line_style = LineStyle::SingleSolid;
    }
  }
  std::vector<std::shared_ptr<LaneLine>> copied_lanelines;
  MakeLaneLineData(copied_laneline_data, id_counter_, &copied_lanelines);
  for (const auto& laneline : copied_lanelines) {
    lanelines.insert(std::make_pair(laneline->GetId(), laneline));
  }

  // remove redundant lanelines
  auto status = RemoveRedundantLaneLines(&lanelines);
  if (status != LOC_SUCCESS) {
    LC_LERROR(MAPMANAGER) << "remove redundant lanelines failed.";
    return status;
  }

  // re-assign laneline points height
  for (auto iter = lanelines.begin(); iter != lanelines.end(); ++iter) {
    auto& line_segments = iter->second->GetLineSegments();
    for (auto& line_segment : line_segments) {
      auto& points = line_segment->GetPoints();
      for (auto& pt : points) pt.z = ground_height_;
    }
  }

  // make the linked relation for laneline and roadside
  std::vector<id_t> laneline_ids, roadside_ids;
  for (const auto& item : lanelines) {
    LineType line_type = item.second->GetLineSegments().front()->GetLineType();
    if (IsLaneMarkingLine(line_type)) {
      laneline_ids.emplace_back(item.first);
    } else if (IsRoadSideLine(line_type)) {
      roadside_ids.emplace_back(item.first);
    }
  }
  for (const auto& rs_id : roadside_ids) {
    const auto& rs_points =
        lanelines.at(rs_id)->GetLineSegments().front()->GetPoints();
    id_t min_ll_id;
    double min_distance = std::numeric_limits<double>::max();
    for (const auto& ll_id : laneline_ids) {
      const auto& ll_points =
          lanelines.at(ll_id)->GetLineSegments().front()->GetPoints();
      int valid_num = 0;
      double dist = Line2LineDistance2D(rs_points, ll_points, &valid_num);
      if (valid_num > 5 && dist < min_distance) {
        min_distance = dist;
        min_ll_id = ll_id;
      }
    }
    if (min_distance < 0.5) {
      lanelines.at(rs_id)->SetLinkedId(min_ll_id);
    }
  }

  lanelines_ = std::move(lanelines);
  return LOC_SUCCESS;
}

adLocStatus_t MapManager::MakeRoadSideForInnerMap(
    float curb_laneline_dis, std::map<id_t, size_t>* line_id_idx_table) {
  const auto& map_lanes = road_structure_->route_map_data.lanes;
  auto& map_lines = road_structure_->semantic_map_data.lines;

  static auto RandomLineId = [](const std::map<id_t, size_t>& line_id_map) {
    static std::default_random_engine e;
    static std::uniform_int_distribution<int> u;
    int64_t val = u(e);
    while (line_id_map.count(val)) val = u(e);
    return val;
  };

  for (const auto& lane : map_lanes) {
    // each pair includes a LineData and an offset value
    std::vector<std::pair<LineData, double>> reference_lines;
    if (lane.left_lane_id == 0) {
      reference_lines.emplace_back(
          map_lines.at(line_id_idx_table->at(lane.left_line_id)),
          curb_laneline_dis);
    }
    if (lane.right_lane_id == 0) {
      reference_lines.emplace_back(
          map_lines.at(line_id_idx_table->at(lane.right_line_id)),
          -curb_laneline_dis);
    }
    for (auto& item : reference_lines) {
      LineData& reference_line = item.first;
      double offset = item.second;
      LineData road_line;
      for (const auto& line_segment : reference_line.line_segments) {
        if (line_segment.points.empty()) continue;
        LineSegmentData new_line_segment;
        new_line_segment.id = line_segment.id;
        new_line_segment.confidence = line_segment.confidence;
        new_line_segment.width = line_segment.width;
        new_line_segment.color = line_segment.color;
        new_line_segment.line_type = LineType::Curb;
        new_line_segment.line_style = LineStyle::Unknown;
        new_line_segment.points.reserve(line_segment.points.size());

        if (line_segment.points.size() == 1) {
          new_line_segment.points = line_segment.points;
        } else {
          Point3D_t last_point = line_segment.points[0];
          for (size_t i = 1; i < line_segment.points.size(); ++i) {
            const auto& point = line_segment.points[i];
            Eigen::Vector2d point_2d(point.x, point.y);
            Eigen::Vector2d last_point_2d(last_point.x, last_point.y);
            Eigen::Vector2d direction_vec = point_2d - last_point_2d;
            Eigen::Vector2d normal_vec(-direction_vec(1), direction_vec(0));
            normal_vec = normal_vec.normalized();
            Eigen::Vector2d new_point_2d = point_2d + offset * normal_vec;
            Point3D_t new_point(new_point_2d(0), new_point_2d(1),
                                (last_point.z + point.z) / 2.0);
            if (i == 1) {
              Eigen::Vector2d new_point_2d =
                  last_point_2d + offset * normal_vec;
              Point3D_t first_new_point(new_point_2d(0), new_point_2d(1),
                                        last_point.z);
              new_line_segment.points.emplace_back(first_new_point);
            }
            new_line_segment.points.emplace_back(new_point);
            last_point = point;
          }
        }

        if (!new_line_segment.points.empty()) {
          road_line.line_segments.emplace_back(new_line_segment);
        }
      }

      if (road_line.line_segments.empty()) continue;
      road_line.id = RandomLineId(*line_id_idx_table);
      map_lines.emplace_back(road_line);
      line_id_idx_table->insert(
          std::make_pair(road_line.id, map_lines.size() - 1));
    }
  }

  return LOC_SUCCESS;
}

adLocStatus_t MapManager::ConnectLaneLineForce(
    const std::vector<LineData>& lines, LaneLine::Ptr* laneline) {
  if (!laneline) {
    LC_LERROR(MAPMANAGER) << "input nullptr.";
    return LOC_NULL_PTR;
  }
  if (lines.empty()) return LOC_LOCALIZATION_ERROR;

  std::vector<Point3D_t> total_points;
  for (const auto& line_data : lines) {
    const auto& points = line_data.line_segments[0].points;
    total_points.insert(total_points.end(), points.begin(), points.end());
  }

  const auto& param = Configure::GetInstance()->GetLocalizationSMMParam();

  // smoothing lane line
  std::vector<Point3D_t> upsamp_points;  // , downsamp_points;
  UpSamplePoints(total_points, param.map_laneline_sample_dist, &upsamp_points);
  // DownSamplePointsRDPMethod(upsamp_points, 0.01, &downsamp_points);

  LineSegment::Ptr line_segment(new LineSegment);
  const auto& raw_line_seg = lines[0].line_segments[0];
  line_segment->SetId(raw_line_seg.id);
  line_segment->SetConfidence(raw_line_seg.confidence);
  line_segment->SetWidth(raw_line_seg.width);
  line_segment->SetLineType(raw_line_seg.line_type);
  line_segment->SetLineStyle(raw_line_seg.line_style);
  line_segment->SetPoints(std::move(upsamp_points));
  // line_segment->SetPoints(std::move(downsamp_points));

  (*laneline)->AddLineSegment(line_segment);

  return LOC_SUCCESS;
}

adLocStatus_t MapManager::ConnectLaneLineByDistance(
    const std::vector<LineData>& lines, float x_connect_threshold,
    float y_connect_threshold, std::vector<LineData>* new_lines) {
  if (!new_lines) {
    LC_LERROR(MAPMANAGER) << "input nullptr.";
    return LOC_NULL_PTR;
  }
  new_lines->reserve(lines.size());

  std::set<size_t> used_line_index;
  for (size_t i = 0; i < lines.size(); ++i) {
    if (used_line_index.find(i) != used_line_index.end()) {
      continue;
    }
    if (lines[i].line_segments[0].points.empty()) continue;
    auto ref_line_data = lines[i];
    auto& ref_line_segment = ref_line_data.line_segments[0];
    auto& ref_points = ref_line_segment.points;

    for (size_t j = i + 1; j < lines.size(); ++j) {
      if (used_line_index.find(j) != used_line_index.end()) {
        continue;
      }
      const auto& src_line_segment = lines[j].line_segments[0];
      if (ref_line_segment.line_type != src_line_segment.line_type) {
        continue;
      }
      const auto& src_points = src_line_segment.points;
      if (src_points.size() < 2) continue;
      Point3D_t ref_back_src_front = ref_points.back() - src_points.front();
      Point3D_t ref_front_src_back = ref_points.front() - src_points.back();
      if (std::fabs(ref_back_src_front.x) < x_connect_threshold &&
          std::fabs(ref_back_src_front.y) < y_connect_threshold) {
        // check angle
        Point3D_t src_begin = src_points[1] - src_points[0];
        Point2D_t ref_back_src_front_2d(ref_back_src_front.x,
                                        ref_back_src_front.y);
        Point2D_t src_begin_2d(src_begin.x, src_begin.y);
        double cosin = ref_back_src_front_2d.Dot(src_begin_2d) /
                       (ref_back_src_front_2d.Norm() * src_begin_2d.Norm());
        if (cosin >= -1.0 && cosin <= -0.97) {  // 166 to 180 degree
          ref_points.insert(ref_points.end(), src_points.begin(),
                            src_points.end());
          used_line_index.insert(j);
        }
      } else if (std::fabs(ref_front_src_back.x) < x_connect_threshold &&
                 std::fabs(ref_front_src_back.y) < y_connect_threshold) {
        // check angle
        Point3D_t src_back = src_points[src_points.size() - 2] -
                             src_points[src_points.size() - 1];
        Point2D_t ref_front_src_back_2d(ref_front_src_back.x,
                                        ref_front_src_back.y);
        Point2D_t src_back_2d(src_back.x, src_back.y);
        double cosin = ref_front_src_back_2d.Dot(src_back_2d) /
                       (ref_front_src_back_2d.Norm() * src_back_2d.Norm());
        if (cosin >= -1.0 && cosin <= -0.97) {  // 166 to 180 degree
          ref_points.insert(ref_points.begin(), src_points.begin(),
                            src_points.end());
          used_line_index.insert(j);
        }
      }
    }
    new_lines->push_back(std::move(ref_line_data));
    used_line_index.insert(i);
  }
  return LOC_SUCCESS;
}

adLocStatus_t MapManager::ConnectLineDataByDistance(
    const std::unordered_set<size_t>& line_data_idxs,
    std::map<id_t, LineData>* connected_line_datas) const {
  std::map<id_t, LineData> connected_line_data;
  const auto& line_datas = road_structure_->semantic_map_data.lines;
  for (const auto& i : line_data_idxs) {
    const auto& line_data = line_datas[i];
    connected_line_data.insert(std::make_pair(line_data.id, line_data));
  }

  auto iter = connected_line_data.begin();
  while (iter != connected_line_data.end()) {
    LC_LDEBUG(SMM) << "connect for line " << iter->first;
    // get unchanged attribute here
    bool is_roadside = IsRoadSideLine(iter->second.line_segments[0].line_type);
    bool is_dashed = IsDashedLine(iter->second.line_segments[0].line_style);

    auto find_iter = connected_line_data.begin();
    while (find_iter != connected_line_data.end()) {
      if (find_iter == iter) {
        ++find_iter;
        continue;
      }
      // not connect if of different type/style
      bool is_find_roadside =
          IsRoadSideLine(find_iter->second.line_segments[0].line_type);
      bool is_find_dashed =
          IsDashedLine(find_iter->second.line_segments[0].line_style);
      if (is_roadside != is_find_roadside || is_dashed != is_find_dashed) {
        ++find_iter;
        continue;
      }

      // get line start/end here, as can be changed during connection
      auto& line_segments = iter->second.line_segments;
      auto point_start = line_segments.front().points.front();
      auto point_end = line_segments.back().points.back();

      auto& find_line_segments = find_iter->second.line_segments;
      auto find_point_start = find_line_segments.front().points.front();
      auto find_point_end = find_line_segments.back().points.back();

      if ((find_point_start - point_end).Norm2D() < 0.1) {
        LC_LDEBUG(SMM) << "append find line after line: " << iter->first << "<-"
                       << find_iter->first;
        // append find line after line
        line_segments.insert(line_segments.end(), find_line_segments.begin(),
                             find_line_segments.end());
        find_iter = connected_line_data.erase(find_iter);
        find_iter--;
      } else if ((find_point_end - point_start).Norm2D() < 0.1) {
        LC_LDEBUG(SMM) << "append line after find line: " << find_iter->first
                       << "<-" << iter->first;
        // append line after find line
        find_line_segments.insert(find_line_segments.end(),
                                  line_segments.begin(), line_segments.end());
        iter = connected_line_data.erase(iter);
        iter--;
        break;  // as memory of line is already released
      }
      ++find_iter;
    }
    ++iter;
  }
  *connected_line_datas = std::move(connected_line_data);
  return LOC_SUCCESS;
}

adLocStatus_t MapManager::MakeLaneLineData(
    const std::vector<LineData>& lines, id_t start_id,
    std::vector<std::shared_ptr<LaneLine>>* lanelines) {
  if (!lanelines) {
    LC_LERROR(MAPMANAGER) << "input nullptr.";
    return LOC_NULL_PTR;
  }

  const auto& param = Configure::GetInstance()->GetLocalizationSMMParam();
  for (const auto& line_data : lines) {
    LineSegment::Ptr line_segment(new LineSegment);
    const auto& raw_line_seg = line_data.line_segments[0];
    line_segment->SetId(raw_line_seg.id);
    line_segment->SetConfidence(raw_line_seg.confidence);
    line_segment->SetWidth(raw_line_seg.width);
    line_segment->SetLineType(raw_line_seg.line_type);
    line_segment->SetLineStyle(raw_line_seg.line_style);

    // upsample and smooth lane line
    std::vector<Point3D_t> upsamp_points;
    UpSamplePoints(raw_line_seg.points, param.map_laneline_sample_dist,
                   &upsamp_points);
    line_segment->SetPoints(std::move(upsamp_points));

    LaneLine::Ptr laneline = std::make_shared<LaneLine>();
    laneline->SetId(id_counter_++);
    laneline->AddLineSegment(line_segment);
    lanelines->push_back(laneline);
  }
  return LOC_SUCCESS;
}

adLocStatus_t MapManager::RemoveRedundantLaneLines(
    LaneLine::PtrUMap* lanelines) {
  if (!lanelines) {
    LC_LERROR(MAPMANAGER) << "input nullptr.";
    return LOC_NULL_PTR;
  }

  std::unordered_map<id_t, LaneLine::Ptr> filtered_lanelines;
  for (auto iter = (*lanelines).begin(); iter != (*lanelines).end(); ++iter) {
    const auto& pts = iter->second->GetLineSegments().front()->GetPoints();
    Point3D_t base_start_pt = pts.front();
    Point3D_t base_end_pt = pts.back();
    bool base_laneline_unique = true;
    for (auto iter2 = std::next(iter); iter2 != (*lanelines).end();) {
      if (iter->second->GetLineSegments().front()->GetLineType() !=
          iter2->second->GetLineSegments().front()->GetLineType()) {
        ++iter2;
        continue;
      }

      const auto& cur_pts =
          iter2->second->GetLineSegments().front()->GetPoints();

      // check if base lane line is redundant
      double min_distance_start = std::numeric_limits<double>::max();
      double min_distance_end = std::numeric_limits<double>::max();
      for (auto pt_iter = cur_pts.begin(); pt_iter != cur_pts.end();
           ++pt_iter) {
        double dist = (base_start_pt - *pt_iter).Norm();
        if (dist < min_distance_start) {
          min_distance_start = dist;
        }
        dist = (base_end_pt - *pt_iter).Norm();
        if (dist < min_distance_end) {
          min_distance_end = dist;
        }
      }
      if (min_distance_start < 1e-6 && min_distance_end < 1e-6) {
        // base lane line is redundant, abandon it
        base_laneline_unique = false;
        break;
      } else {
        // check if current lane line is redundant
        Point3D_t cur_start_pt = cur_pts.front();
        Point3D_t cur_end_pt = cur_pts.back();
        double min_distance_start = std::numeric_limits<double>::max();
        double min_distance_end = std::numeric_limits<double>::max();
        for (auto pt_iter = pts.begin(); pt_iter != pts.end(); ++pt_iter) {
          double dist = (cur_start_pt - *pt_iter).Norm();
          if (dist < min_distance_start) {
            min_distance_start = dist;
          }
          dist = (cur_end_pt - *pt_iter).Norm();
          if (dist < min_distance_end) {
            min_distance_end = dist;
          }
        }
        if (min_distance_start < 1e-6 && min_distance_end < 1e-6) {
          // current lane line is redundant, abandon it
          iter2 = (*lanelines).erase(iter2);
        } else {
          // base lane line and current lane line are independent
          ++iter2;
        }
      }
    }
    if (base_laneline_unique) {
      // candidate lane lines is unique, save it
      filtered_lanelines.insert(std::make_pair(iter->first, iter->second));
    }
  }
  lanelines->swap(filtered_lanelines);
  return LOC_SUCCESS;
}

adLocStatus_t MapManager::ProcessMapTrafficSign() {
  TrafficSign::PtrUMap traffic_signs;
  for (const auto& traffic_sign_data :
       road_structure_->semantic_map_data.traffic_signs) {
    Point3D_t center = traffic_sign_data.centroid;
    center.z = ground_height_;
    id_t id = id_counter_++;
    traffic_signs.insert({id, std::make_shared<TrafficSign>(
                                  id, center, traffic_sign_data.type)});
  }
  traffic_signs_ = std::move(traffic_signs);
  return LOC_SUCCESS;
}

adLocStatus_t MapManager::ProcessMapPole() {
  Pole::PtrUMap poles;
  for (auto& pole_data : road_structure_->semantic_map_data.poles) {
    Point3D_t bottom_point = pole_data.bottom_point;
    bottom_point.z = ground_height_;
    id_t id = id_counter_++;
    poles.insert(
        {id, std::make_shared<Pole>(id, bottom_point, pole_data.type)});
  }
  poles_ = std::move(poles);
  return LOC_SUCCESS;
}

adLocStatus_t MapManager::CovPropagate2Local2D(
    const SE3d& pose, const Eigen::Matrix<double, 6, 6>& pose_cov,
    const Point3D_t& local_point, Eigen::Matrix3d* local_cov) const {
  if (!local_cov) return LOC_NULL_PTR;

  // propagate position covariance
  Eigen::Matrix3d R_wv = pose.so3().matrix();
  Eigen::Matrix3d local_trans_cov =
      R_wv.transpose() * pose_cov.topLeftCorner(3, 3) * R_wv;
  Eigen::Matrix2d local_trans_cov_2d = local_trans_cov.topLeftCorner(2, 2);
  if (local_trans_cov_2d(0, 0) > 1e4 || local_trans_cov_2d(1, 1) > 1e4 ||
      local_trans_cov_2d(0, 0) <= 0.0 || local_trans_cov_2d(1, 1) <= 0.0) {
    // position covariance is invalid
    local_trans_cov_2d = Eigen::Matrix2d::Identity();
    local_trans_cov_2d *= 1e4;
  }

  // propagate heading covariance
  double heading_cov = pose_cov(5, 5);
  if (heading_cov <= 0.0 || heading_cov > 0.1) {
    // heading covariance is invalid
    heading_cov = 0.1;
  }
  Eigen::Matrix2d local_heading_cov;
  local_heading_cov(0, 0) = heading_cov * local_point.y * local_point.y;
  local_heading_cov(1, 1) = heading_cov * local_point.x * local_point.x;
  local_heading_cov(0, 1) = -heading_cov * local_point.x * local_point.y;
  local_heading_cov(1, 0) = local_heading_cov(0, 1);

  Eigen::Matrix2d local_cov2d = local_trans_cov_2d + local_heading_cov;
  local_cov->topLeftCorner(2, 2).noalias() = local_cov2d;
  (*local_cov)(2, 2) = 1e8;
  return LOC_SUCCESS;
}

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
