/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： elements_filter.h
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/map_fusion_02/modules/geo/elements_filter.h"

#include <memory>
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
  // get pose
  LocInfo::ConstPtr T_ptr =
      DR_MANAGER->GetDrPoseByTimeStamp(origin_element_map_ptr->map_info.stamp);
  // filter element map lines
  FilterElementMapLines(origin_element_map_ptr->lane_boundaries);
  // filter intersect line
  FilterIntersectLine();
  // ContinueLocalMapUseLine();
  // 补缺失的线
  CompleteElementMapLine();
  // rviz
  geo_viz_.VizElementMap(origin_element_map_ptr, T_ptr->pose);
  return true;
}

void ElementsFilter::FilterElementMapLines(
    const std::map<Id, Boundary::Ptr>& lane_boundaries) {
  if (lane_boundaries.empty()) {
    return;
  }
  if (!local_line_table_.empty()) {
    local_line_table_.clear();
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
    std::vector<cv::Point2d> kdtree_points;
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
    local_line_info line_info;
    line_info.line_kdtree = kdtree_ptr;
    line_info.line_track_id = track_id;
    line_info.local_line_pts = local_points;
    local_line_table_[track_id] = line_info;
  }
  return;
}

void ElementsFilter::FilterIntersectLine() {
  auto local_lines_size = local_line_table_.size();
  if (local_lines_size < 3) {
    return;
  }
  for (int i = 0, j = 1; i < local_lines_size - 1, j < local_lines_size;
       ++i, j = i + 1) {
    auto& selected_line = local_line_table_[i];
    if (selected_line.local_line_pts.size() > 25) {
      continue;
    }
    for (int j = i + 1; j < local_lines_size; ++j) {
      auto& candidated_line = local_line_table_[j];
      if (candidated_line.local_line_pts.size() > 25) {
        continue;
      }
      Eigen::Vector2f intersect_point{0.f, 0.f};
      Eigen::Vector2f a(
          static_cast<float>(selected_line.local_line_pts.front().x()),
          static_cast<float>(selected_line.local_line_pts.front().y()));
      Eigen::Vector2f b(
          static_cast<float>(selected_line.local_line_pts.back().x()),
          static_cast<float>(selected_line.local_line_pts.back().y()));
      Eigen::Vector2f c(
          static_cast<float>(candidated_line.local_line_pts.front().x()),
          static_cast<float>(candidated_line.local_line_pts.front().y()));
      Eigen::Vector2f d(
          static_cast<float>(candidated_line.local_line_pts.back().x()),
          static_cast<float>(candidated_line.local_line_pts.back().y()));
      if (SegmentIntersection(a, b, c, d, &intersect_point)) {
        selected_line.store = false;
        candidated_line.store = false;
      }
    }
  }
  // 短线跟长线交叉 删除短线
  for (int i = 0, j = 1; i < local_lines_size - 1, j < local_lines_size;
       ++i, j = i + 1) {
    auto& selected_line = local_line_table_[i];
    if (!selected_line.store || selected_line.local_line_pts.size() < 2) {
      continue;
    }
    for (int j = i + 1; j < local_lines_size; ++j) {
      auto& candidated_line = local_line_table_[j];
      if (!candidated_line.store || candidated_line.local_line_pts.size() < 2) {
        continue;
      }
      if (selected_line.local_line_pts.back().y() <
              candidated_line.local_line_pts.front().y() ||
          candidated_line.local_line_pts.back().y() <
              selected_line.local_line_pts.front().y()) {
        continue;
      }
      if (selected_line.local_line_pts.size() > 25 &&
          candidated_line.local_line_pts.size() <= 25) {
        Eigen::Vector2f intersect_point{0.f, 0.f};
        Eigen::Vector2f a(
            static_cast<float>(selected_line.local_line_pts.front().x()),
            static_cast<float>(selected_line.local_line_pts.front().y()));
        Eigen::Vector2f b(
            static_cast<float>(selected_line.local_line_pts.back().x()),
            static_cast<float>(selected_line.local_line_pts.back().y()));
        Eigen::Vector2f c(
            static_cast<float>(candidated_line.local_line_pts.front().x()),
            static_cast<float>(candidated_line.local_line_pts.front().y()));
        Eigen::Vector2f d(
            static_cast<float>(candidated_line.local_line_pts.back().x()),
            static_cast<float>(candidated_line.local_line_pts.back().y()));
        if (SegmentIntersection(a, b, c, d, &intersect_point)) {
          candidated_line.store = false;
        }
      } else if (selected_line.local_line_pts.size() <= 25 &&
                 candidated_line.local_line_pts.size() > 25) {
        Eigen::Vector2f intersect_point{0.f, 0.f};
        Eigen::Vector2f a(
            static_cast<float>(selected_line.local_line_pts.front().x()),
            static_cast<float>(selected_line.local_line_pts.front().y()));
        Eigen::Vector2f b(
            static_cast<float>(selected_line.local_line_pts.back().x()),
            static_cast<float>(selected_line.local_line_pts.back().y()));
        Eigen::Vector2f c(
            static_cast<float>(candidated_line.local_line_pts.front().x()),
            static_cast<float>(candidated_line.local_line_pts.front().y()));
        Eigen::Vector2f d(
            static_cast<float>(candidated_line.local_line_pts.back().x()),
            static_cast<float>(candidated_line.local_line_pts.back().y()));
        if (SegmentIntersection(a, b, c, d, &intersect_point)) {
          selected_line.store = false;
        }
      }
    }
  }
}

void ElementsFilter::CompleteElementMapLine() {
  if (used_lines_.empty()) {
    return;
  }
  CreateLineTable();
  MakeRoadEdgeToLaneLine();
  HandleExtraWideLane();
}

void ElementsFilter::CreateLineTable() {}

void ElementsFilter::MakeRoadEdgeToLaneLine() {}

void ElementsFilter::HandleExtraWideLane() {}
void ElementsFilter::Clear() { local_line_table_.clear(); }

}  // namespace mf
}  // namespace mp
}  // namespace hozon
