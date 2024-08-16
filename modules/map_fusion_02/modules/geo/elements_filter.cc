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
  CompensateElementMapLine(road_edges);
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
    line_table_[track_id] = line_info;
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
    auto& selected_line = line_table_[track_ids[i]];
    if (selected_line.line_pts.size() > 25 ||
        selected_line.line_pts.size() < 0) {
      continue;
    }
    for (int j = i + 1; j < local_lines_size; ++j) {
      auto& candidated_line = line_table_[track_ids[j]];
      if (candidated_line.line_pts.size() > 25 ||
          selected_line.line_pts.size() < 0) {
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
    auto& selected_line = line_table_[track_ids[i]];
    if (!selected_line.store || selected_line.line_pts.size() < 2) {
      continue;
    }
    for (int j = i + 1; j < local_lines_size; ++j) {
      auto& candidated_line = line_table_[track_ids[j]];
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
void ElementsFilter::Clear() { line_table_.clear(); }

}  // namespace mf
}  // namespace mp
}  // namespace hozon
