/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： elements_filter.cc
 *   author     ： oyhl
 *   date       ： 2024.09
 ******************************************************************************/

#include "modules/map_fusion/modules/geo/elements_filter.h"

#include <limits>
#include <memory>
#include <numeric>
#include <utility>

#include "modules/map_fusion/common/calc_util.h"
#include "modules/map_fusion/data_convert/data_convert.h"
#include "modules/map_fusion/data_manager/location_data_manager.h"
#include "modules/map_fusion/data_manager/percep_obj_manager.h"
#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace mf {

bool ElementsFilter::Init() {
  HLOG_WARN << "inital success!";
  return true;
}

bool ElementsFilter::Process(ElementMap::Ptr origin_element_map_ptr) {
  road_edge_table_ = origin_element_map_ptr->road_edges;
  stop_lines_talbe_ = origin_element_map_ptr->stop_lines;
  LocInfo::ConstPtr T_ptr = LOCATION_MANAGER->GetLocationByTimeStamp(
      origin_element_map_ptr->map_info.stamp);
  T_L_V_ = T_ptr->pose.matrix().cast<double>();
  FilterElementMapLines(origin_element_map_ptr->lane_boundaries);
  FilterIntersectLine();
  // 补缺失的线
  CompensateElementMapLine();
  // 处理路口场景逆向车道车道线
  FilterReverseLine();
  // 过滤非路口场景非主路车道线
  FilterNoEgoLineNoCrossing();
  UpdateElementMapLines(origin_element_map_ptr);
  return true;
}

void ElementsFilter::FilterElementMapLines(
    const std::map<Id, Boundary::Ptr>& lane_boundaries) {
  if (lane_boundaries.empty()) {
    return;
  }
  if (!line_table_.empty()) {
    line_table_.clear();
  }
  std::set<int> duplicate_last_track_id;
  if (!last_track_id_.empty()) {
    duplicate_last_track_id = last_track_id_;
    last_track_id_.clear();
  }
  for (const auto& lane_boundary : lane_boundaries) {
    auto track_id = lane_boundary.first;
    const auto& lane_boundary_ptr = lane_boundary.second;
    const auto& lane_boundary_nodes = lane_boundary_ptr->nodes;
    const auto& line_type = lane_boundary_ptr->linetype;
    auto lane_boundary_nodes_size = lane_boundary_nodes.size();
    if (lane_boundary_nodes_size < 2) {
      continue;
    }
    if (duplicate_last_track_id.find(track_id) !=
            duplicate_last_track_id.end() &&
        lane_boundary_nodes_size < 25) {
      last_track_id_.insert(track_id);  // last_track_id_: 不进行处理的车道线
      continue;
    }
    std::vector<cv::Point2f> kdtree_points;
    std::vector<Eigen::Vector3f> local_points;
    for (const auto& node_ptr : lane_boundary_nodes) {
      const auto& point = node_ptr->point;
      kdtree_points.emplace_back(point.x(), point.y());
      local_points.emplace_back(point);
    }
    if (kdtree_points.size() < 2) {
      continue;
    }
    cv::flann::KDTreeIndexParams index_param(1);
    std::shared_ptr<cv::flann::Index> kdtree_ptr =
        std::make_shared<cv::flann::Index>(cv::Mat(kdtree_points).reshape(1),
                                           index_param);
    GeoLineInfo line_info;
    line_info.line_kdtree = kdtree_ptr;
    line_info.line_track_id = track_id;
    line_info.line_pts = local_points;
    line_info.line_type = line_type;
    line_table_[track_id] = line_info;
  }
  return;
}

void ElementsFilter::FilterIntersectLine() {
  int local_lines_size = static_cast<int>(line_table_.size());
  if (local_lines_size < 3) {
    return;
  }
  std::vector<int> track_ids;
  track_ids.reserve(local_lines_size);
  for (const auto& line : line_table_) {
    track_ids.emplace_back(line.first);
  }
  for (int i = 0; i < local_lines_size - 1; ++i) {
    auto& selected_line = line_table_[track_ids[i]];
    auto selected_line_pts = selected_line.line_pts;
    if (selected_line_pts.size() > 25 || selected_line_pts.size() < 0) {
      continue;
    }
    for (int j = i + 1; j < local_lines_size; ++j) {
      auto& candidated_line = line_table_[track_ids[j]];
      auto candidated_line_pts = candidated_line.line_pts;
      if (candidated_line_pts.size() > 25 || selected_line_pts.size() < 0) {
        continue;
      }
      if (math::DropIntersectLine(selected_line_pts, candidated_line_pts)) {
        selected_line.store = false;
        candidated_line.store = false;
      }
    }
  }
  // 短线跟长线交叉 删除短线
  for (int i = 0; i < local_lines_size - 1; ++i) {
    auto& selected_line = line_table_[track_ids[i]];
    auto selected_line_pts = selected_line.line_pts;
    if (!selected_line.store || selected_line_pts.size() < 2) {
      continue;
    }
    for (int j = i + 1; j < local_lines_size; ++j) {
      auto& candidated_line = line_table_[track_ids[j]];
      auto candidated_line_pts = candidated_line.line_pts;
      if (!candidated_line.store || candidated_line_pts.size() < 2) {
        continue;
      }
      if (selected_line_pts.back().y() < candidated_line_pts.front().y() ||
          candidated_line_pts.back().y() < selected_line_pts.front().y()) {
        continue;
      }
      if (selected_line_pts.size() > 25 && candidated_line_pts.size() <= 25) {
        if (math::DropIntersectLine(selected_line_pts, candidated_line_pts)) {
          candidated_line.store = false;
        }
      } else if (selected_line_pts.size() <= 25 &&
                 candidated_line_pts.size() > 25) {
        if (math::DropIntersectLine(selected_line_pts, candidated_line_pts)) {
          selected_line.store = false;
        }
      }
    }
  }
  for (auto& line : line_table_) {
    if (!line.second.store) {
      last_track_id_.insert(line.first);
    }
  }
}

void ElementsFilter::CompensateElementMapLine() {
  if (line_table_.empty() || road_edge_table_.empty()) {
    return;
  }
  // MakeRoadEdgeToLaneLine();
  HandleExtraWideLane();
}

void ElementsFilter::MakeRoadEdgeToLaneLine() {
  if (road_edge_table_.empty()) {
    return;
  }
  for (const auto& road_edge : road_edge_table_) {
    std::vector<Eigen::Vector3f> road_pts;
    for (const auto& road_pt : road_edge.second->points) {
      road_pts.emplace_back(road_pt);
    }
    if (road_pts.empty()) {
      continue;
    }
    CompareRoadAndLines(road_pts, road_edge.first);
  }
}

void ElementsFilter::CompensatePoints(
    const std::vector<Eigen::Vector3f>& road_pts,
    const int& target_line_track_id, const bool flag,
    std::vector<Eigen::Vector3f>* target_line_pts) {
  if (target_line_pts == nullptr) {
    return;
  }
  Eigen::Vector3f target_line_heading(0.0, 0.0, 0.0);
  ComputeLaneLineHeading(*target_line_pts, &target_line_heading);
  std::vector<Eigen::Vector3f> compensated_pts;
  math::GetCompensatePoints(road_pts, *target_line_pts, flag, &compensated_pts);
  if (compensated_pts.empty()) {
    return;
  }
  if (IsBetweenLinesMid(compensated_pts, target_line_track_id, flag)) {
    return;
  }
  std::vector<Eigen::Vector3f> retained_pts;
  if (!flag) {  // 补后向
    for (const auto& pt : *target_line_pts) {
      retained_pts.emplace_back(pt);
    }
    if (!target_line_pts->empty()) {
      target_line_pts->clear();
    }
  }
  for (int i = 0; i < static_cast<int>(compensated_pts.size()) - 1; ++i) {
    auto heading = compensated_pts[i + 1] - compensated_pts[i];
    float angle = 0.0;
    ComputeAngleBetweenVectors(target_line_heading, heading, &angle);
    if (angle > 15) {
      continue;
    }
    target_line_pts->emplace_back(compensated_pts[i]);
  }
  if (!flag && !retained_pts.empty()) {  // 补后向
    for (const auto& retained_pt : retained_pts) {
      target_line_pts->emplace_back(retained_pt);
    }
  }
  return;
}

bool ElementsFilter::GetClosestLineToRoad(
    const std::vector<Eigen::Vector3f>& road_pts, int* target_line_track_id,
    double* min_dis) {
  if (target_line_track_id == nullptr || min_dis == nullptr) {
    return false;
  }
  // 比较路沿和车道线
  *min_dis = std::numeric_limits<double>::max();
  for (const auto& line : line_table_) {
    if (line.second.line_pts.empty()) {
      continue;
    }
    double avg_road_line_distance = 0.0;
    if (!math::ComputerLineDis(road_pts, line.second.line_pts,
                               &avg_road_line_distance)) {
      continue;
    }
    if (avg_road_line_distance < *min_dis) {
      *min_dis = avg_road_line_distance;
      *target_line_track_id = line.first;
    }
  }
  if (line_table_.find(*target_line_track_id) == line_table_.end()) {
    return false;
  }
  return true;
}

void ElementsFilter::AddNewLineByRoadPoints(
    const std::vector<Eigen::Vector3f>& target_line_pts,
    const std::vector<Eigen::Vector3f>& road_pts, const int& road_id) {
  Eigen::Vector3f target_line_heading(0.0, 0.0, 0.0);
  ComputeLaneLineHeading(target_line_pts, &target_line_heading);
  GeoLineInfo new_line;
  for (int i = 0; i < static_cast<int>(road_pts.size()) - 1; ++i) {
    auto road_heading = road_pts[i + 1] - road_pts[i];
    float angle = 0.0;
    ComputeAngleBetweenVectors(target_line_heading, road_heading, &angle);
    if (angle > 15) {
      continue;
    }
    new_line.line_pts.emplace_back(road_pts[i]);
  }
  new_line.line_track_id = road_id + 1000;
  new_line.line_type = LineType::LaneType_SOLID;
  new_line.color = Color::WHITE;
  std::vector<cv::Point2f> kdtree_points;
  for (const auto& road_pt : road_pts) {
    kdtree_points.emplace_back(road_pt.x(), road_pt.y());
  }
  cv::flann::KDTreeIndexParams index_param(1);
  std::shared_ptr<cv::flann::Index> kdtree_ptr =
      std::make_shared<cv::flann::Index>(cv::Mat(kdtree_points).reshape(1),
                                         index_param);
  new_line.line_kdtree = kdtree_ptr;
  line_table_[new_line.line_track_id] = new_line;
  return;
}

void ElementsFilter::CompareRoadAndLines(
    const std::vector<Eigen::Vector3f>& road_pts, const int& road_id) {
  if (road_pts.empty()) {
    return;
  }
  int target_line_track_id;
  double min_dis;
  if (!GetClosestLineToRoad(road_pts, &target_line_track_id, &min_dis)) {
    return;
  }
  std::vector<Eigen::Vector3f> target_line_pts =
      line_table_[target_line_track_id].line_pts;
  LineType target_line_type = line_table_[target_line_track_id].line_type;
  if (min_dis < 1.0) {
    // 路沿离车道线的距离小于1米,对离路沿最近的车道线进行增补
    if (road_pts.back().x() > target_line_pts.back().x()) {
      // 对车道线往前补,直至和路沿远端对齐
      CompensatePoints(road_pts, target_line_track_id, true, &target_line_pts);
    }
    if (road_pts.front().x() < target_line_pts.front().x()) {
      // 对车道线往后补,直至和路沿远端对齐
      CompensatePoints(road_pts, target_line_track_id, false, &target_line_pts);
    }
  }
  if (!(((target_line_type == LineType::LaneType_DASHED ||
          target_line_type == LineType::LaneType_FISHBONE_DASHED ||
          target_line_type == LineType::LaneType_DOUBLE_DASHED ||
          target_line_type == LineType::LaneType_SHORT_DASHED) &&
         min_dis > 2.0) ||
        ((target_line_type == LineType::LaneType_SOLID ||
          target_line_type == LineType::LaneType_DOUBLE_SOLID ||
          target_line_type == LineType::LaneType_FISHBONE_SOLID) &&
         min_dis > 3.0))) {
    return;
  }
  AddNewLineByRoadPoints(target_line_pts, road_pts, road_id);
}

bool ElementsFilter::IsBetweenLinesMid(
    const std::vector<Eigen::Vector3f>& compensated_line_pts,
    const int& target_line_track_id, const bool& flag) {
  if (compensated_line_pts.empty() || line_table_.empty()) {
    return false;
  }
  int left_count = 0, right_count = 0;
  for (const auto& line : line_table_) {
    if (line.second.line_pts.empty() || line.first == target_line_track_id) {
      continue;
    }
    if (flag) {
      // 只判断前向
      for (const auto& line_pt : line.second.line_pts) {
        if (line_pt.x() < compensated_line_pts.front().x()) {
          continue;
        }
        // 只判断第一个点
        if (line_pt.y() >= compensated_line_pts.front().y()) {
          left_count += 1;
        } else {
          right_count += 1;
        }
        break;
      }
      // 判断最后一个点
      if (line.second.line_pts.back().y() >= compensated_line_pts.back().y()) {
        left_count += 1;
      } else {
        right_count += 1;
      }
    }
    if (!flag) {
      // 只判断后向
      for (const auto& line_pt : line.second.line_pts) {
        if (line_pt.x() > compensated_line_pts.back().x()) {
          continue;
        }
        // 只判断最后点
        if (line_pt.y() >= compensated_line_pts.back().y()) {
          left_count += 1;
        } else {
          right_count += 1;
        }
        break;
      }
      // 判断第一个点
      if (line.second.line_pts.front().y() >=
          compensated_line_pts.front().y()) {
        left_count += 1;
      } else {
        right_count += 1;
      }
    }
  }
  HLOG_DEBUG << "left_count: " << left_count
             << ", right_count: " << right_count;
  return (left_count != 0 && right_count != 0);
}

void ElementsFilter::UpdateElementMapLines(ElementMap::Ptr element_map_ptr) {
  if (line_table_.empty()) {
    return;
  }
  if (!element_map_ptr->lane_boundaries.empty()) {
    element_map_ptr->lane_boundaries.clear();
  }
  int loop = 0;
  for (auto& line : line_table_) {
    if (line.second.line_pts.empty()) {
      HLOG_WARN << "line_pts empty, line id: " << line.first;
      continue;
    }
    Boundary::Ptr boundary_added = std::make_shared<Boundary>();
    DataConvert::CvtLine2Boundary(line.second, boundary_added);
    for (const auto& road : road_edge_table_) {
      double avg_dis = 0.0;
      if (!math::ComputerLineDis(line.second.line_pts, road.second->points,
                                 &avg_dis)) {
        continue;
      }
      if (avg_dis < 1.0) {
        boundary_added->is_near_road_edge = true;
        break;
      }
    }
    if (element_map_ptr->lane_boundaries.find(line.first) !=
        element_map_ptr->lane_boundaries.end()) {
      continue;
    }
    element_map_ptr->lane_boundaries[line.first] = boundary_added;
  }
  return;
}

void ElementsFilter::HandleExtraWideLane() {}

void ElementsFilter::Clear() {
  line_table_.clear();
  road_edge_table_.clear();
  stop_lines_talbe_.clear();
  is_not_ego_lane_track_id_.clear();
}

void ElementsFilter::FilterReverseLine() {
  // 处理路口逆向车道的车道线
  /*
    路口场景
              |    |    |
              |    |    |
              |  ↓ |  ↑ |
              |    |    |
              /         \
             /           \



             \           /
              \         /
               | | | | |
               | | |↑| |
               | | | | |
  */
  if (line_table_.empty()) {
    HLOG_WARN << "line_table_ is empty";
    return;
  }
  // 根据障碍物过滤对向车道线
  HandleOppisiteLineByObj();
  std::vector<Eigen::Vector3f> road_edge_pts = GetdRoadEdgePts();
  std::vector<Eigen::Vector3f> double_solid_yellow_pts =
      GetDoubleSolidYellowLine();
  HandleOppisiteLineByStopline();

  // 根据路沿,双黄线,停止线过滤逆向车道,优先用路沿，其次用双黄线，再次用停止线
  /*
    1.两侧有车道的路沿作为正逆向的分隔线
    2.若有双黄线，将双黄线左侧的车道线做标记
    3.和停止线关联的线
  */
  if (!road_edge_pts.empty()) {
    HandleOppisiteLine(road_edge_pts);
  } else {
    if (!double_solid_yellow_pts.empty()) {
      HandleOppisiteLine(double_solid_yellow_pts);
    } else {
      // HandleOppisiteLineByObjAndYelloLine();  // 融合障碍物和黄线过滤对向车道
    }
  }
}

// 获取前向stopline集合
void ElementsFilter::GetStopLine(
    std::vector<std::vector<Eigen::Vector2f>>* forward_stoplines) {
  if (forward_stoplines == nullptr || stop_lines_talbe_.empty()) {
    HLOG_WARN << "stop line is empty!!";
    return;
  }
  for (const auto& stop_line : stop_lines_talbe_) {
    if (!stop_line.second->points.size() < 2 ||
        stop_line.second->points[0].x() < 0 ||
        stop_line.second->points[1].x() < 0) {
      continue;
    }
    Eigen::Vector2f pt_left(stop_line.second->points[0].x(),
                            stop_line.second->points[0].y());
    Eigen::Vector2f pt_right(stop_line.second->points[1].x(),
                             stop_line.second->points[1].y());
    std::vector<Eigen::Vector2f> stopline_pts{pt_left, pt_right};
    forward_stoplines->emplace_back(stopline_pts);
  }
  return;
}

void ElementsFilter::HandleOppisiteLineByStopline() {
  if (line_table_.empty()) {
    HLOG_ERROR << "local_map_ is nullptr";
    return;
  }
  // 获取前向stopline集合
  std::vector<std::vector<Eigen::Vector2f>> forward_stoplines;
  GetStopLine(&forward_stoplines);
  std::unordered_set<int> line_track_ids;
  for (auto& line : line_table_) {
    line_track_ids.insert(line.first);
  }
  for (auto it = is_not_ego_lane_track_id_.begin();
       it != is_not_ego_lane_track_id_.end();) {
    if (line_track_ids.find(*it) != line_track_ids.end()) {
      it++;
    } else {
      it = is_not_ego_lane_track_id_.erase(it);
    }
  }
  for (auto& line : line_table_) {
    if (line.second.line_pts.size() < 2 ||
        (!line.second.line_pts.empty() && line.second.line_pts[0].x() < 0)) {
      continue;
    }
    if (is_not_ego_lane_track_id_.find(line.first) !=
        is_not_ego_lane_track_id_.end()) {
      line.second.is_ego = IsEgo::Other_Road;
      continue;
    }
    auto line_pts = line.second.line_pts;
    // 前向车道线的第一个点能投影在前向停止线上,且投影距离<10,且停止线的右边点在车道线的右侧1.5米以外，
    // 且车道线的终点要在停止线的左边5m以外，认为是对向车道线
    Eigen::Vector2f line_first_point(line_pts[0].x(), line_pts[0].y());
    Eigen::Vector2f line_end_point(line_pts.back().x(), line_pts.back().y());
    for (auto& forward_stopline : forward_stoplines) {
      if (forward_stopline.size() < 2) {
        continue;
      }
      auto point_it = std::find_if(
          line_pts.begin() + 1, line_pts.end(),
          [&](const Eigen::Vector3f& point) {
            return PointInVectorSide(forward_stopline[0], forward_stopline[1],
                                     Eigen::Vector2f(point.x(), point.y())) < 0;
          });
      Eigen::Vector2f line_second_point;
      if (point_it == line_pts.end()) {
        line_second_point << line_pts.back().x(), line_pts.back().y();
      } else {
        line_second_point << point_it->x(), point_it->y();
      }

      if (PointToVectorDist(forward_stopline[0], forward_stopline[1],
                            line_first_point) < 10 &&
          PointInVectorSide(line_first_point, line_second_point,
                            forward_stopline[1]) > 0 &&
          PointToVectorDist(line_first_point, line_second_point,
                            forward_stopline[1]) > 1.5 &&
          PointInVectorSide(forward_stopline[0], forward_stopline[1],
                            line_end_point) < 0 &&
          PointToVectorDist(forward_stopline[0], forward_stopline[1],
                            line_end_point) > 5.0f) {
        line.second.is_ego = IsEgo::Other_Road;
        if (line.first < 1000) {
          is_not_ego_lane_track_id_.insert(line.first);
        }
      }
    }
  }
}

void ElementsFilter::GetHistoryObjsInMiddleOfLane(
    const boost::circular_buffer<std::shared_ptr<Object>>& history_objs,
    std::vector<Eigen::Vector3f>* obj_points) {
  if (obj_points == nullptr) {
    return;
  }
  for (const auto& object : history_objs) {
    Eigen::Vector3d p_local(object->position.x(), object->position.y(),
                            object->position.z());
    auto p_v_tmp = T_L_V_.inverse() * p_local;
    Eigen::Vector3f p_veh(static_cast<float>(p_v_tmp.x()),
                          static_cast<float>(p_v_tmp.y()),
                          static_cast<float>(p_v_tmp.z()));
    if (object->type != ObjType::VEHICLE || p_veh.x() <= 0 ||
        object->velocity.x() > 0 ||
        (-M_PI * 0.75 < object->heading && object->heading < M_PI * 0.75)) {
      continue;
    }
    // 如果是自车右侧逆向障碍物，且是静止状态(速度在正负１m/s之间，则不用)
    if (p_veh.y() < -2.5 && std::fabs(object->velocity.x()) < 1) {
      continue;
    }
    // 筛选出在车道线中间的object
    bool obj_on_line_left = false;
    bool obj_on_line_right = false;
    for (const auto& line_elem : line_table_) {
      auto line = line_elem.second;
      auto line_pts = line.line_pts;
      int line_size = line_pts.size();
      if (line_size < 2 || (line_size >= 2 && line_pts.at(0).x() < 0)) {
        continue;
      }
      if (p_veh.x() < line_pts.front().x() || p_veh.x() > line_pts.back().x()) {
        continue;
      }
      int line_index = 0;
      for (; line_index < line_size - 1; line_index++) {
        if (line_pts[line_index].x() < p_veh.x() &&
            line_pts[line_index + 1].x() > p_veh.x()) {
          break;
        }
      }
      auto linel1 = line_pts[line_index];
      auto linel2 = line_pts[line_index + 1];
      if (math::IsRight(p_veh, linel1, linel2)) {
        obj_on_line_right = true;
      } else {
        obj_on_line_left = true;
      }
    }
    if (obj_on_line_left && obj_on_line_right) {
      obj_points->emplace_back(p_veh);
    }
  }
  return;
}

void ElementsFilter::HandleOppisiteLineByObj() {
  boost::circular_buffer<std::shared_ptr<Object>> history_objs =
      OBJECT_MANAGER->GetHistoryObjs();
  std::vector<Eigen::Vector3f> obj_points;
  GetHistoryObjsInMiddleOfLane(history_objs, &obj_points);
  if (obj_points.empty()) {
    return;
  }
  for (auto& line_elem : line_table_) {
    auto& line = line_elem.second;
    auto line_pts = line.line_pts;
    int line_size = line_pts.size();
    if (line_size < 2 || (line_size >= 2 && line_pts[0].x() < 0)) {
      continue;
    }
    if (CheckOppositeLineByObj(line_pts, obj_points)) {
      line.is_ego = IsEgo::Other_Road;
      continue;
    }
  }
}

std::vector<Eigen::Vector3f> ElementsFilter::GetdRoadEdgePts() {
  std::vector<std::vector<Eigen::Vector3f>> forward_road_edges;
  for (const auto& road_edge : road_edge_table_) {
    auto local_road = road_edge.second;
    if (local_road->points.size() < 2 ||
        (!local_road->points.empty() && local_road->points[0].x() < 0)) {
      continue;
    }
    std::vector<Eigen::Vector3f> pts;
    for (const auto& it : local_road->points) {
      Eigen::Vector3f pt(it.x(), it.y(), it.z());
      pts.emplace_back(pt);
    }
    forward_road_edges.emplace_back(pts);
  }
  std::vector<Eigen::Vector3f> road_edge_pts;
  road_edge_pts = FindTargetPoints(forward_road_edges);
  return road_edge_pts;
}

std::vector<Eigen::Vector3f> ElementsFilter::FindTargetPoints(
    const std::vector<std::vector<Eigen::Vector3f>>& forward_road_edges) {
  std::vector<Eigen::Vector3f> res;
  for (const auto& road_edge : forward_road_edges) {
    bool have_left_line = false;
    bool have_right_line = false;
    for (const auto& line_elem : line_table_) {
      auto line = line_elem.second;
      auto line_pts = line.line_pts;
      if (line_pts.size() < 2 || (!line_pts.empty() && line_pts[0].x() < 0)) {
        continue;
      }
      double avg_width = 0.0;
      if (!math::ComputerLineDis(road_edge, line_pts, &avg_width)) {
        continue;
      }
      if (math::IsTargetOnLineRight(road_edge, line_pts) ==
              RelativePosition::RIGHT &&
          avg_width > 3.0) {
        have_left_line = true;
      } else if (math::IsTargetOnLineRight(road_edge, line_pts) ==
                     RelativePosition::LEFT &&
                 avg_width > 3.0) {
        have_right_line = true;
      }
      if (have_left_line && have_right_line) {
        double road_edge_heading = math::CalMeanLineHeading(road_edge);
        if (math::IsRoadEdgeOnVehicleRight(road_edge, road_edge_heading) ==
            RelativePosition::LEFT) {
          res = road_edge;
          return res;
        }
      }
    }
  }
  return res;
}

std::vector<Eigen::Vector3f> ElementsFilter::GetDoubleSolidYellowLine() {
  std::vector<std::vector<Eigen::Vector3f>> double_solid_yellow_line;
  for (const auto& line_elem : line_table_) {
    auto line = line_elem.second;
    if (line.line_type == LineType::LaneType_DOUBLE_SOLID &&
        line.color == Color::YELLOW && line.line_pts.size() > 1 &&
        line.line_pts.at(0).x() > 0) {
      std::vector<Eigen::Vector3f> pts;
      for (const auto& it : line.line_pts) {
        Eigen::Vector3f pt(it.x(), it.y(), it.z());
        pts.emplace_back(pt);
      }
      double_solid_yellow_line.emplace_back(pts);
    }
  }
  // 确定双黄线的左右侧是否有车道线
  std::vector<Eigen::Vector3f> double_solid_yellow_pts;
  double_solid_yellow_pts = FindTargetPoints(double_solid_yellow_line);
  return double_solid_yellow_pts;
}

void ElementsFilter::HandleOppisiteLine(
    const std::vector<Eigen::Vector3f>& target_line) {
  if (target_line.empty()) {
    return;
  }
  for (auto& line_elem : line_table_) {
    auto line = line_elem.second;
    if (line.line_pts.size() < 2 ||
        (!line.line_pts.empty() && line.line_pts.at(0).x() < 0)) {
      continue;
    }
    double avg_width = 0.0;
    if (!math::ComputerLineDis(target_line, line.line_pts, &avg_width)) {
      continue;
    }
    if (math::IsTargetOnLineRight(target_line, line.line_pts) ==
            RelativePosition::RIGHT &&
        avg_width > 2.0) {
      line.is_ego = IsEgo::Other_Road;
    }
  }
}

void ElementsFilter::HandleOppisiteLineByObjAndYelloLine() {
  double max_y = -DBL_MAX;
  for (const auto& line_elem : line_table_) {
    auto line = line_elem.second;

    if (line.is_ego == IsEgo::Other_Road) {
      continue;
    }
    if ((line.line_type == LineType::LaneType_DOUBLE_SOLID ||
         line.line_type == LineType::LaneType_SOLID) &&
        line.color == Color::YELLOW && line.line_pts.size() > 1 &&
        line.line_pts.at(0).x() > 0) {
      if (line.line_pts.at(0).y() > max_y) {
        max_y = line.line_pts.at(0).y();
      }
    }
  }

  if (max_y == -DBL_MAX) {
    return;
  }
  for (auto& line_elem : line_table_) {
    auto line = line_elem.second;
    if (line.is_ego == IsEgo::Other_Road) {
      continue;
    }
    if (line.line_pts.size() > 1 && line.line_pts.at(0).x() > 0 &&
        line.line_pts.at(0).y() > max_y) {
      line.is_ego = IsEgo::Other_Road;
    }
  }
}

void ElementsFilter::FilterNoEgoLineNoCrossing() {
  if (road_edge_table_.empty()) {
    HLOG_WARN << "road_edge_table_ is nullptr";
    return;
  }
  // 模型路沿
  std::vector<std::vector<Eigen::Vector3f>> nearby_road_edges;
  for (const auto& road_edge : road_edge_table_) {
    if (road_edge.second->points.empty()) {
      continue;
    }
    std::vector<Eigen::Vector3f> pts;
    for (const auto& it : road_edge.second->points) {
      pts.emplace_back(it.x(), it.y(), it.z());
    }
    nearby_road_edges.emplace_back(pts);
  }
  std::vector<Eigen::Vector3f> road_edge_pts;
  FindTargetPointsNoCrossing(nearby_road_edges, &road_edge_pts);
  if (!road_edge_pts.empty()) {
    HandleOppisiteLineNoCrossing(road_edge_pts);
  }
}

void ElementsFilter::FindTargetPointsNoCrossing(
    const std::vector<std::vector<Eigen::Vector3f>>& nearby_road_edges,
    std::vector<Eigen::Vector3f>* target_road_edge) {
  if (target_road_edge == nullptr || nearby_road_edges.empty()) {
    return;
  }
  for (const auto& road_edge : nearby_road_edges) {
    if (road_edge.size() < 2 ||
        (!road_edge.empty() && road_edge.at(0).x() > 0)) {
      continue;
    }
    bool have_left_line = false;
    bool have_right_line = false;
    for (const auto& line : line_table_) {
      if (line.second.line_pts.size() < 2 ||
          (!line.second.line_pts.empty() && line.second.line_pts[0].x() > 0)) {
        continue;
      }
      double avg_width = 0.0;
      if (!math::ComputerLineDis(road_edge, line.second.line_pts, &avg_width)) {
        continue;
      }
      if (avg_width < 3.0) {
        continue;
      }
      RelativePosition relative_pos =
          math::IsTargetOnLineRight(road_edge, line.second.line_pts);
      have_left_line = (relative_pos == RelativePosition::RIGHT) ? true : false;
      have_right_line = (relative_pos == RelativePosition::LEFT) ? true : false;
      if (!(have_left_line && have_right_line)) {
        continue;
      }
      double road_edge_heading = math::CalMeanLineHeading(road_edge);
      if (math::IsRoadEdgeOnVehicleRight(road_edge, road_edge_heading) !=
          RelativePosition::LEFT) {
        continue;
      }
      // 比较res和road_edge哪个离车更近——x最小的点的y值大小
      if (target_road_edge->empty()) {
        *target_road_edge = road_edge;
      } else {
        Eigen::Vector3f res_point = (*target_road_edge)[0];
        Eigen::Vector3f road_edge_point = road_edge[0];
        for (auto point : *target_road_edge) {
          if (std::abs(point.x()) < std::abs(res_point.x())) {
            res_point = point;
          }
        }
        for (auto point : road_edge) {
          if (std::abs(point.x()) < std::abs(road_edge_point.x())) {
            road_edge_point = point;
          }
        }
        if (res_point.y() > road_edge_point.y()) {
          *target_road_edge = road_edge;
        }
      }
    }
  }
}

void ElementsFilter::HandleOppisiteLineNoCrossing(
    const std::vector<Eigen::Vector3f>& target_road_edge) {
  if (target_road_edge.empty()) {
    return;
  }
  for (auto& line : line_table_) {
    if (line.second.line_pts.empty()) {
      continue;
    }
    if (line.second.line_pts.size() < 2 ||
        (!line.second.line_pts.empty() && line.second.line_pts[0].x() > 0)) {
      continue;
    }
    double avg_width = 0.0;
    if (!math::ComputerLineDis(target_road_edge, line.second.line_pts,
                               &avg_width)) {
      continue;
    }
    if (math::IsTargetOnLineRight(target_road_edge, line.second.line_pts) ==
            RelativePosition::RIGHT &&
        avg_width > 2.0) {
      line.second.is_ego = IsEgo::Other_Road;
    }
  }
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
