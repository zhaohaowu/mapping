/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： elements_filter.h
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/map_fusion_02/modules/geo/elements_filter.h"

#include <limits>
#include <memory>
#include <numeric>
#include <utility>

#include "modules/map_fusion_02/common/calc_util.h"
#include "modules/map_fusion_02/data_convert/data_convert.h"
#include "modules/map_fusion_02/data_manager/dr_data_manager.h"
#include "modules/map_fusion_02/modules/geo/elements_filter_base.h"
#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace mf {

bool ElementsFilter::Init() {
  history_objs_.set_capacity(10);
  history_objs_.clear();
  HLOG_WARN << "inital success!";
  return true;
}

bool ElementsFilter::Process(ElementMap::Ptr origin_element_map_ptr) {
  HLOG_INFO << "ElementsFilter Process";
  // get road edge
  std::map<Id, RoadEdge::Ptr> road_edges = origin_element_map_ptr->road_edges;
  // get pose
  LocInfo::ConstPtr T_ptr =
      DR_MANAGER->GetDrPoseByTimeStamp(origin_element_map_ptr->map_info.stamp);
  // filter element map lines
  FilterElementMapLines(origin_element_map_ptr->lane_boundaries);
  // filter intersect line
  FilterIntersectLine();
  // ContinueLocalMapUseLine();
  // 补缺失的线
<<<<<<< HEAD
  CompensateElementMapLine(road_edges);
=======
  CompleteElementMapLine();

  // 处理路口场景逆向车道车道线
  road_edge_table_ = origin_element_map_ptr->road_edges;
  FilterReverseLine();

>>>>>>> mf-geo:移植过滤逆向车道模块
  // rviz
  geo_viz_.VizElementMap(origin_element_map_ptr, T_ptr->pose);
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
    auto lane_boundary_nodes_size = lane_boundary_nodes.size();
    if (lane_boundary_nodes_size < 2) {
      continue;
    }
    if (!duplicate_last_track_id.empty() &&
        duplicate_last_track_id.find(track_id) !=
            duplicate_last_track_id.end() &&
        lane_boundary_nodes_size < 25) {
      last_track_id_.insert(track_id);
      continue;
    }
    std::vector<cv::Point2f> kdtree_points;
    std::vector<Eigen::Vector3f> local_points;
    for (const auto& node_ptr : lane_boundary_nodes) {
      const auto& point = node_ptr->point;
      if (std::isnan(point.x()) || std::isnan(point.y()) ||
          std::isnan(point.z())) {
        HLOG_ERROR << "found nan point in local map lane line";
        continue;
      }
      kdtree_points.emplace_back(point.x(), point.y());
      local_points.emplace_back(point.x(), point.y(), point.z());
    }
    if (kdtree_points.size() < 2) {
      continue;
    }
    cv::flann::KDTreeIndexParams index_param(1);
    std::shared_ptr<cv::flann::Index> kdtree_ptr =
        std::make_shared<cv::flann::Index>(cv::Mat(kdtree_points).reshape(1),
                                           index_param);
    LineInfo line_info;
    line_info.line_kdtree = kdtree_ptr;
    line_info.line_track_id = track_id;
    line_info.line_pts = local_points;
<<<<<<< HEAD
    line_table_[track_id] = line_info;
=======
    local_line_table_[track_id] = line_info;
>>>>>>> mf-geo:移植过滤逆向车道模块
  }
  return;
}

void ElementsFilter::FilterIntersectLine() {
  auto local_lines_size = line_table_.size();
  if (local_lines_size < 3) {
    return;
  }
  std::vector<int> track_ids;
  track_ids.reserve(local_lines_size);
  for (const auto& line : line_table_) {
    track_ids.emplace_back(line.first);
  }
  for (int i = 0, j = 1; i < local_lines_size - 1, j < local_lines_size;
       ++i, j = i + 1) {
<<<<<<< HEAD
    auto& selected_line = line_table_[track_ids[i]];
    if (selected_line.line_pts.size() > 25 ||
        selected_line.line_pts.size() < 0) {
      continue;
    }
    for (int j = i + 1; j < local_lines_size; ++j) {
      auto& candidated_line = line_table_[track_ids[j]];
      if (candidated_line.line_pts.size() > 25 ||
          selected_line.line_pts.size() < 0) {
=======
    auto& selected_line = local_line_table_[i];
    if (selected_line.line_pts.size() > 25) {
      continue;
    }
    for (int j = i + 1; j < local_lines_size; ++j) {
      auto& candidated_line = local_line_table_[j];
      if (candidated_line.line_pts.size() > 25) {
>>>>>>> mf-geo:移植过滤逆向车道模块
        continue;
      }
      Eigen::Vector2f intersect_point{0.f, 0.f};
      Eigen::Vector2f a(static_cast<float>(selected_line.line_pts.front().x()),
                        static_cast<float>(selected_line.line_pts.front().y()));
      Eigen::Vector2f b(static_cast<float>(selected_line.line_pts.back().x()),
                        static_cast<float>(selected_line.line_pts.back().y()));
      Eigen::Vector2f c(
          static_cast<float>(candidated_line.line_pts.front().x()),
          static_cast<float>(candidated_line.line_pts.front().y()));
      Eigen::Vector2f d(
          static_cast<float>(candidated_line.line_pts.back().x()),
          static_cast<float>(candidated_line.line_pts.back().y()));
      if (SegmentIntersection(a, b, c, d, &intersect_point)) {
        selected_line.store = false;
        candidated_line.store = false;
      }
    }
  }
  // 短线跟长线交叉 删除短线
  for (int i = 0, j = 1; i < local_lines_size - 1, j < local_lines_size;
       ++i, j = i + 1) {
<<<<<<< HEAD
    auto& selected_line = line_table_[track_ids[i]];
=======
    auto& selected_line = local_line_table_[i];
>>>>>>> mf-geo:移植过滤逆向车道模块
    if (!selected_line.store || selected_line.line_pts.size() < 2) {
      continue;
    }
    for (int j = i + 1; j < local_lines_size; ++j) {
<<<<<<< HEAD
      auto& candidated_line = line_table_[track_ids[j]];
=======
      auto& candidated_line = local_line_table_[j];
>>>>>>> mf-geo:移植过滤逆向车道模块
      if (!candidated_line.store || candidated_line.line_pts.size() < 2) {
        continue;
      }
      if (selected_line.line_pts.back().y() <
              candidated_line.line_pts.front().y() ||
          candidated_line.line_pts.back().y() <
              selected_line.line_pts.front().y()) {
        continue;
      }
      if (selected_line.line_pts.size() > 25 &&
          candidated_line.line_pts.size() <= 25) {
        Eigen::Vector2f intersect_point{0.f, 0.f};
        Eigen::Vector2f a(
            static_cast<float>(selected_line.line_pts.front().x()),
            static_cast<float>(selected_line.line_pts.front().y()));
        Eigen::Vector2f b(
            static_cast<float>(selected_line.line_pts.back().x()),
            static_cast<float>(selected_line.line_pts.back().y()));
        Eigen::Vector2f c(
            static_cast<float>(candidated_line.line_pts.front().x()),
            static_cast<float>(candidated_line.line_pts.front().y()));
        Eigen::Vector2f d(
            static_cast<float>(candidated_line.line_pts.back().x()),
            static_cast<float>(candidated_line.line_pts.back().y()));
        if (SegmentIntersection(a, b, c, d, &intersect_point)) {
          candidated_line.store = false;
        }
      } else if (selected_line.line_pts.size() <= 25 &&
                 candidated_line.line_pts.size() > 25) {
        Eigen::Vector2f intersect_point{0.f, 0.f};
        Eigen::Vector2f a(
            static_cast<float>(selected_line.line_pts.front().x()),
            static_cast<float>(selected_line.line_pts.front().y()));
        Eigen::Vector2f b(
            static_cast<float>(selected_line.line_pts.back().x()),
            static_cast<float>(selected_line.line_pts.back().y()));
        Eigen::Vector2f c(
            static_cast<float>(candidated_line.line_pts.front().x()),
            static_cast<float>(candidated_line.line_pts.front().y()));
        Eigen::Vector2f d(
            static_cast<float>(candidated_line.line_pts.back().x()),
            static_cast<float>(candidated_line.line_pts.back().y()));
        if (SegmentIntersection(a, b, c, d, &intersect_point)) {
          selected_line.store = false;
        }
      }
    }
  }
}

void ElementsFilter::CompensateElementMapLine(
    const std::map<Id, RoadEdge::Ptr>& road_edges) {
  if (line_table_.empty() || road_edges.empty()) {
    return;
  }
  // CreateLineTable();
  MakeRoadEdgeToLaneLine(road_edges);
  HandleExtraWideLane();
}

void ElementsFilter::MakeRoadEdgeToLaneLine(
    const std::map<Id, RoadEdge::Ptr>& road_edges) {
  if (road_edges.empty()) {
    return;
  }
  for (const auto& road_edge : road_edges) {
    std::vector<Eigen::Vector3d> road_pts;
    for (const auto& road_pt : road_edge.second->points) {
      if (std::isnan(road_pt.x()) || std::isnan(road_pt.y())) {
        continue;
      }
      road_pts.emplace_back(road_pt.x(), road_pt.y(), road_pt.z());
    }
    if (road_pts.empty()) {
      continue;
    }
    CompareRoadAndLines(road_pts, road_edge.first);
  }
}

void ElementsFilter::CompareRoadAndLines(
    const std::vector<Eigen::Vector3d>& road_pts, const int& road_id) {
  if (road_pts.empty()) {
    return;
  }
  // 比较路沿和车道线
  LineInfo target_lane_boundary_line;
  double min_dis = std::numeric_limits<double>::max();
  for (const auto& line : line_table_) {
    if (line.second.line_pts.empty()) {
      continue;
    }
    std::vector<Eigen::Vector3d> line_pts;
    for (const auto& line_pt : line.second.line_pts) {
      line_pts.emplace_back(line_pt.x(), line_pt.y(), line_pt.z());
    }
    std::vector<double> road_line_distance;
    math::ComputerLineDis(road_pts, line_pts, &road_line_distance);
    if (road_line_distance.empty()) {
      continue;
    }
    auto avg_road_line_distance =
        std::accumulate(road_line_distance.begin(), road_line_distance.end(),
                        0.0) /
        static_cast<double>(road_line_distance.size());
    if (avg_road_line_distance < min_dis) {
      min_dis = avg_road_line_distance;
      target_lane_boundary_line = line.second;
    }
  }
  if (min_dis < 1.0) {
    // 路沿离车道线的距离小于1米,对离路沿最近的车道线进行增补
    auto target_line_pt_size = target_lane_boundary_line.line_pts.size();
    if (road_pts.back().x() > target_lane_boundary_line.line_pts.back().x()) {
      // 对车道线往前补,直至和路沿远端对齐
      for (auto& line : line_table_) {
        if (line.second.line_track_id !=
            target_lane_boundary_line.line_track_id) {
          continue;
        }
        Eigen::Vector3d fit_point;
        for (const auto& road_pt : road_pts) {
          if (road_pt.x() > target_lane_boundary_line.line_pts.back().x()) {
            Eigen::Vector3d target_pt(
                target_lane_boundary_line.line_pts.back().x(),
                target_lane_boundary_line.line_pts.back().y(),
                target_lane_boundary_line.line_pts.back().z());
            fit_point = target_pt - road_pt;
            break;
          }
        }
        for (const auto& road_pt : road_pts) {
          if (road_pt.x() < target_lane_boundary_line.line_pts.back().x()) {
            continue;
          }
          double fit_dis = fit_point.norm();
          auto fit_point_normlized = fit_point.normalized();
          auto fit_road_pt = road_pt + fit_point_normlized * fit_dis;
          // 简单策略:先根据横向距离对点进行调整,防止车道线跟路沿连接时不平滑
          // 后续可以严格计算点线距离
          line.second.line_pts.emplace_back(fit_road_pt.x(), fit_road_pt.y(),
                                            fit_road_pt.z());
        }
        break;
      }
    }
    if (road_pts.front().x() < target_lane_boundary_line.line_pts.front().x()) {
      for (auto& line : line_table_) {
        if (line.second.line_track_id !=
            target_lane_boundary_line.line_track_id) {
          continue;
        }
        std::vector<Eigen::Vector3f> retained_pts;
        for (const auto& pt : line.second.line_pts) {
          retained_pts.emplace_back(pt);
        }
        line.second.line_pts.clear();
        Eigen::Vector3d fit_point;
        for (const auto& road_pt : road_pts) {
          if (road_pt.x() < target_lane_boundary_line.line_pts.front().x()) {
            continue;
          }
          Eigen::Vector3d target_pt(
              target_lane_boundary_line.line_pts.front().x(),
              target_lane_boundary_line.line_pts.front().y(),
              target_lane_boundary_line.line_pts.front().z());
          fit_point = target_pt - road_pt;
          break;
        }
        for (const auto& road_pt : road_pts) {
          if (road_pt.x() > target_lane_boundary_line.line_pts.front().x()) {
            continue;
          }
          double fit_dis = fit_point.norm();
          auto fit_point_normlized = fit_point.normalized();
          auto fit_road_pt = road_pt + fit_point_normlized * fit_dis;
          line.second.line_pts.emplace_back(fit_road_pt.x(), fit_road_pt.y(),
                                            fit_road_pt.z());
        }
        for (const auto& retained_pt : retained_pts) {
          line.second.line_pts.emplace_back(retained_pt.x(), retained_pt.y(),
                                            retained_pt.z());
        }
        break;
      }
    }
  }
  if (((target_lane_boundary_line.line_type == LineType::LaneType_DASHED ||
        target_lane_boundary_line.line_type ==
            LineType::LaneType_FISHBONE_DASHED ||
        target_lane_boundary_line.line_type ==
            LineType::LaneType_DOUBLE_DASHED ||
        target_lane_boundary_line.line_type ==
            LineType::LaneType_SHORT_DASHED) &&
       min_dis > 2.0) ||
      ((target_lane_boundary_line.line_type == LineType::LaneType_SOLID ||
        target_lane_boundary_line.line_type ==
            LineType::LaneType_DOUBLE_SOLID ||
        target_lane_boundary_line.line_type ==
            LineType::LaneType_FISHBONE_SOLID) &&
       min_dis > 3.0)) {
    LineInfo new_line;
    new_line.line_track_id = road_id + 1000;
    new_line.line_type = LineType::LaneType_SOLID;
    new_line.color = Color::WHITE;
    for (const auto& pt : road_pts) {
      new_line.line_pts.emplace_back(pt.x(), pt.y(), pt.z());
    }
    std::vector<cv::Point2f> kdtree_points;
    for (const auto& road_pt : road_pts) {
      kdtree_points.emplace_back(static_cast<float>(road_pt.x()),
                                 static_cast<float>(road_pt.y()));
    }
    cv::flann::KDTreeIndexParams index_param(1);
    std::shared_ptr<cv::flann::Index> kdtree_ptr =
        std::make_shared<cv::flann::Index>(cv::Mat(kdtree_points).reshape(1),
                                           index_param);
    new_line.line_kdtree = kdtree_ptr;
    line_table_[new_line.line_track_id] = new_line;
  }
}

void ElementsFilter::HandleExtraWideLane() {}
<<<<<<< HEAD
void ElementsFilter::Clear() { line_table_.clear(); }
=======

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
  if (local_line_table_.empty()) {
    HLOG_WARN << "local_line_table_ is empty";
    return;
  }
  // 根据障碍物过滤对向车道线
  HandleOppisiteLineByObj();
  std::vector<Eigen::Vector3d> road_edge_pts = GetdRoadEdgePts();
  std::vector<Eigen::Vector3d> double_solid_yellow_pts =
      GetDoubleSolidYellowLine();

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
      // HandleOppisiteLineByStopline();
      HandleOppisiteLineByObjAndYelloLine();  // 融合障碍物和黄线过滤对向车道
    }
  }
}

void ElementsFilter::HandleOppisiteLineByObj() {
  std::vector<Eigen::Vector3d> obj_points;
  for (const auto& history_obj : history_objs_) {
    for (const auto& object : history_obj->perception_obstacle()) {
      Eigen::Vector3d p_local(object.position().x(), object.position().y(),
                              object.position().z());
      Eigen::Vector3d p_veh = T_U_V_.inverse() * p_local;

      if (object.type() != hozon::perception::PerceptionObstacle::VEHICLE ||
          p_veh.x() <= 0 || object.velocity().x() > 0 ||
          (-M_PI * 0.75 < object.theta() && object.theta() < M_PI * 0.75)) {
        continue;
      }
      // 筛选出在车道线中间的object
      bool obj_on_line_left = false;
      bool obj_on_line_right = false;
      for (const auto& line_elem : local_line_table_) {
        auto line = line_elem.second;
        int line_size = line.line_pts.size();
        if (line_size < 2 ||
            (line_size >= 2 && line.line_pts.at(0).x() < 0)) {
          continue;
        }
        int line_index = 0;
        if (p_veh.x() < line.line_pts[0].x()) {
          line_index = 0;
        } else if (p_veh.x() > line.line_pts[line_size - 1].x()) {
          line_index = line_size - 2;
        } else {
          for (; line_index < line_size - 1; line_index++) {
            if (line.line_pts[line_index].x() < p_veh.x() &&
                line.line_pts[line_index + 1].x() > p_veh.x()) {
              break;
            }
          }
        }
        Eigen::Vector3d linel1(line.line_pts[line_index].x(),
                               line.line_pts[line_index].y(),
                               line.line_pts[line_index].z());
        Eigen::Vector3d linel2(line.line_pts[line_index + 1].x(),
                               line.line_pts[line_index + 1].y(),
                               line.line_pts[line_index + 1].z());
        if (math::IsRight(p_veh, linel1, linel2)) {
          obj_on_line_right = true;
        } else {
          obj_on_line_left = true;
        }
      }
      if (obj_on_line_left && obj_on_line_right) {
        obj_points.emplace_back(p_veh);
      }
    }
  }
  if (obj_points.empty()) {
    return;
  }
  for (auto& line_elem : local_line_table_) {
    auto& line = line_elem.second;
    int line_size = line.line_pts.size();
    if (line_size < 2 || (line_size>=2 && line.line_pts[0].x() < 0)) {
      continue;
    }
    for (auto& obj_point : obj_points) {
      bool stop_loop = false;
      if (obj_point.x() > line.line_pts[line_size - 1].x() ||
          obj_point.x() < line.line_pts[0].x()) {
        continue;
      }
      for (int line_index = 0; line_index < line_size - 1; line_index++) {
        if (line.line_pts[line_index].x() < obj_point.x() &&
            line.line_pts[line_index + 1].x() > obj_point.x()) {
          Eigen::Vector3d linel1(line.line_pts[line_index].x(),
                                 line.line_pts[line_index].y(),
                                 line.line_pts[line_index].z());
          Eigen::Vector3d linel2(line.line_pts[line_index + 1].x(),
                                 line.line_pts[line_index + 1].y(),
                                 line.line_pts[line_index + 1].z());
          if (math::IsRight(obj_point, linel1, linel2)) {
            line.is_ego_road = false;
            stop_loop = true;
            break;
          }
        }
      }
      if (stop_loop) {
        break;
      }
    }
  }
}

std::vector<Eigen::Vector3d> ElementsFilter::GetdRoadEdgePts() {
  std::vector<std::vector<Eigen::Vector3d>> forward_road_edges;
  for (const auto& road_edge : road_edge_table_) {
    auto local_road = road_edge.second;
    if (local_road->points.size() < 2 ||
        (!local_road->points.empty() && local_road->points[0].x() < 0)) {
      continue;
    }
    std::vector<Eigen::Vector3d> pts;
    for (const auto& it : local_road->points) {
      Eigen::Vector3d pt(it.x(), it.y(), it.z());
      pts.emplace_back(pt);
    }
    forward_road_edges.emplace_back(pts);
  }
  std::vector<Eigen::Vector3d> road_edge_pts;
  road_edge_pts = FindTargetPoints(forward_road_edges);
  return road_edge_pts;
}


std::vector<Eigen::Vector3d> ElementsFilter::FindTargetPoints(
    const std::vector<std::vector<Eigen::Vector3d>>& forward_road_edges) {
  std::vector<Eigen::Vector3d> res;
  for (const auto& road_edge : forward_road_edges) {
    bool have_left_line = false;
    bool have_right_line = false;
    for (const auto& line_elem : local_line_table_) {
      auto line = line_elem.second;
      if (line.line_pts.size() < 2 ||
          (!line.line_pts.empty() && line.line_pts[0].x() < 0)) {
        continue;
      }
      std::vector<double> widths;
      std::vector<Eigen::Vector3d> line_pts;
      for (const auto& it : line.line_pts) {
        Eigen::Vector3d pt(it.x(), it.y(), it.z());
        line_pts.emplace_back(pt);
      }
      math::ComputerLineDis(road_edge, line_pts, &widths);
      double avg_width = std::accumulate(widths.begin(), widths.end(), 0.0) /
                         static_cast<double>(widths.size());

      if (IsTargetOnLineRight(road_edge, line) == RelativePosition::RIGHT &&
          avg_width > 3.0) {
        have_left_line = true;
      } else if (IsTargetOnLineRight(road_edge, line) ==
                     RelativePosition::LEFT &&
                 avg_width > 3.0) {
        have_right_line = true;
      }
      if (have_left_line && have_right_line) {
        double road_edge_heading = math::CalMeanLineHeading(road_edge);
        if (IsRoadEdgeOnVehicleRight(road_edge, road_edge_heading) ==
            RelativePosition::LEFT) {
          res = road_edge;
          return res;
        }
      }
    }
  }
  return res;
}


RelativePosition ElementsFilter::IsTargetOnLineRight(
    const std::vector<Eigen::Vector3d>& target_line, const LineInfo& line) {
  int line_size = line.line_pts.size();
  if (line_size < 2 || target_line.size() < 1) {
    return RelativePosition::UNCERTAIN;
  }
  if (target_line.front().x() >= line.line_pts[line_size - 1].x() &&
      line.line_pts[0].x() >= target_line.back().x()) {
    return RelativePosition::UNCERTAIN;
  }
  int num_thresh = 0, num_calculate = 0;
  for (int target_index = 0, line_index = 0; target_index < target_line.size();
       target_index++) {
    if (std::isnan(target_line.at(target_index).x()) ||
        std::isnan(target_line.at(target_index).y()) ||
        std::isnan(target_line.at(target_index).z())) {
      HLOG_WARN
          << "found nan point in double_solid_yellow_line or road_edge_line";
      continue;
    }
    Eigen::Vector3d target_point(target_line.at(target_index).x(),
                                 target_line.at(target_index).y(),
                                 target_line.at(target_index).z());
    if (target_point.x() < line.line_pts[0].x()) {
      continue;
    }
    while (line_index < line_size - 1 &&
           line.line_pts[line_index + 1].x() < target_point.x()) {
      line_index++;
    }
    if (line_index == line_size - 1) {
      break;
    }
    Eigen::Vector3d linel1(line.line_pts[line_index].x(),
                           line.line_pts[line_index].y(),
                           line.line_pts[line_index].z());
    Eigen::Vector3d linel2(line.line_pts[line_index + 1].x(),
                           line.line_pts[line_index + 1].y(),
                           line.line_pts[line_index + 1].z());
    num_calculate++;
    if (math::IsRight(target_point, linel1, linel2)) {
      num_thresh++;
    }
  }
  if (num_thresh < 1 ||
      (num_calculate != 0 && (static_cast<double>(num_thresh) /
                              static_cast<double>(num_calculate)) < 0.5)) {
    return RelativePosition::LEFT;
  }
  return RelativePosition::RIGHT;
}


RelativePosition ElementsFilter::IsRoadEdgeOnVehicleRight(
    const std::vector<Eigen::Vector3d>& points, const double& heading) {
  if (points.empty() || std::abs(heading) > M_PI) {
    return RelativePosition::UNCERTAIN;
  }
  Eigen::Vector3d p1(0, 0, 0);
  Eigen::Vector3d p2(cos(heading), sin(heading), 0);
  int num_thresh = 0;
  int num_calculate = 0;
  for (const auto& point : points) {
    if (std::isnan(point.x()) || std::isnan(point.y()) ||
        std::isnan(point.z())) {
      HLOG_WARN << "found nan point in road_edge_line";
      continue;
    }
    num_calculate++;
    if (math::IsRight(point, p1, p2)) {
      num_thresh++;
    }
  }
  if (num_thresh < 1 ||
      (num_calculate != 0 && (static_cast<double>(num_thresh) /
                              static_cast<double>(num_calculate)) < 0.5)) {
    return RelativePosition::LEFT;
  }
  return RelativePosition::RIGHT;
}


std::vector<Eigen::Vector3d> ElementsFilter::GetDoubleSolidYellowLine() {
  std::vector<std::vector<Eigen::Vector3d>> double_solid_yellow_line;
  for (const auto& line_elem : local_line_table_) {
    auto line = line_elem.second;
    if (line.line_type == hozon::mapping::LaneType::LaneType_DOUBLE_SOLID &&
        line.color == hozon::mapping::Color::YELLOW &&
        line.line_pts.size() > 1 && line.line_pts.at(0).x() > 0) {
      std::vector<Eigen::Vector3d> pts;
      for (const auto& it : line.line_pts) {
        Eigen::Vector3d pt(it.x(), it.y(), it.z());
        pts.emplace_back(pt);
      }
      double_solid_yellow_line.emplace_back(pts);
    }
  }
  // 确定双黄线的左右侧是否有车道线
  std::vector<Eigen::Vector3d> double_solid_yellow_pts;
  double_solid_yellow_pts = FindTargetPoints(double_solid_yellow_line);
  return double_solid_yellow_pts;
}

void ElementsFilter::HandleOppisiteLine(
    const std::vector<Eigen::Vector3d>& target_line) {
  if (target_line.empty()) {
    return;
  }
  for (auto& line_elem : local_line_table_) {
    auto line = line_elem.second;
    if (line.line_pts.size() < 2 ||
        (!line.line_pts.empty() && line.line_pts.at(0).x() < 0)) {
      continue;
    }
    std::vector<double> widths;
    std::vector<Eigen::Vector3d> line_pts;
    for (const auto& it : line.line_pts) {
      Eigen::Vector3d pt(it.x(), it.y(), it.z());
      line_pts.emplace_back(pt);
    }
    math::ComputerLineDis(target_line, line_pts, &widths);
    double avg_width = std::accumulate(widths.begin(), widths.end(), 0.0) /
                       (static_cast<double>(widths.size()) + 1e-3);
    if (IsTargetOnLineRight(target_line, line) == RelativePosition::RIGHT &&
        avg_width > 2.0) {
      line.is_ego_road = false;
    }
  }
}

void ElementsFilter::HandleOppisiteLineByObjAndYelloLine() {
  double max_y = -DBL_MAX;
  for (const auto& line_elem : local_line_table_) {
    auto line = line_elem.second;

    if (!line.is_ego_road) {
      continue;
    }
    if ((line.line_type == hozon::mapping::LaneType::LaneType_DOUBLE_SOLID ||
         line.line_type == hozon::mapping::LaneType::LaneType_SOLID) &&
        line.color == hozon::mapping::Color::YELLOW &&
        line.line_pts.size() > 1 && line.line_pts.at(0).x() > 0) {
      if (line.line_pts.at(0).y() > max_y) {
        max_y = line.line_pts.at(0).y();
      }
    }
  }

  if (max_y == -DBL_MAX) {
    return;
  }
  for (auto& line_elem : local_line_table_) {
    auto line = line_elem.second;
    if (!line.is_ego_road) {
      continue;
    }
    if (line.line_pts.size() > 1 && line.line_pts.at(0).x() > 0 &&
        line.line_pts.at(0).y() > max_y) {
      line.is_ego_road = false;
    }
  }
}

void ElementsFilter::Clear() { local_line_table_.clear(); }
>>>>>>> mf-geo:移植过滤逆向车道模块

}  // namespace mf
}  // namespace mp
}  // namespace hozon
