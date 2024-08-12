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
  // rviz
  geo_viz_.VizElementMap(origin_element_map_ptr, T_ptr->pose);
  return true;
}

void ElementsFilter::FilterElementMapLines(
    const std::map<Id, Boundary::Ptr>& lane_boundaries) {
  if (lane_boundaries.empty()) {
    return;
  }
  if (!all_lines_.empty()) {
    all_lines_.clear();
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
    Line_kd line_kd;
    for (const auto& node_ptr : lane_boundary_nodes) {
      const auto& point = node_ptr->point;
      if (std::isnan(point.x()) || std::isnan(point.y()) ||
          std::isnan(point.z())) {
        HLOG_ERROR << "found nan point in local map lane line";
        continue;
      }
      Eigen::Vector3d point_local(point.x(), point.y(), point.z());
      kdtree_points.emplace_back(point_local.x(), point_local.y());
    }
    if (kdtree_points.size() < 2) {
      continue;
    }
    cv::flann::KDTreeIndexParams index_param(1);
    std::shared_ptr<cv::flann::Index> kdtree_ptr =
        std::make_shared<cv::flann::Index>(cv::Mat(kdtree_points).reshape(1),
                                           index_param);
    line_kd.line_kdtree = kdtree_ptr;
    line_kd.lane_boundary_line =
        std::make_shared<std::pair<const Id, Boundary::Ptr>>(lane_boundary);
    all_lines_[static_cast<int>(lane_boundary_ptr->lanepos)].emplace_back(
        std::move(line_kd));
  }
  return;
}

void ElementsFilter::FilterIntersectLine() {
  // auto all_lines_size = all_lines_.size();
  // if (all_lines_size < 3) {
  //   return;
  // }
  // std::vector<int> all_lane_poses;
  // all_lane_poses.resize(all_lines_size);
  // if (all_lane_poses.size() != all_lines_size) {
  //   return;
  // }
  // int index = 0;
  // for (const auto& line : all_lines_) {
  //   all_lane_poses[index++] = line.first;
  // }
  // // 长线不判断
  // for (int i = 0, j = 1; i < all_lines_size - 1, j < all_lines_size;
  //      ++i, j = i + 1) {
  //   for (auto& selected_line : all_lines_[all_lane_poses[i]]) { // Line_kd vector
  //     // 点数 > 25的长线不判断
  //     if (selected_line.lane_boundary_line->second->nodes.size() > 25) {
  //       continue;
  //     }
  //     for (auto& candidated_line : all_lines_[all_lane_poses[j]]) {
  //       if (candidated_line.lane_boundary_line->second->nodes.size() > 25 ||
  //           j >= all_lines_size) {
  //         continue;
  //       }
  //       Eigen::Vector2d intersect_point;
  //       Eigen::Vector2d a =
  //           (selected_line.lane_boundary_line->second->nodes.front()
  //                ->get()
  //                ->point.x(),
  //            selected_line.lane_boundary_line->second->nodes.front()
  //                ->get()
  //                ->point.y());
  //       Eigen::Vector2d b =
  //           (selected_line.lane_boundary_line->second->nodes.back()
  //                ->get()
  //                ->point.x(),
  //            selected_line.lane_boundary_line->second->nodes.back()
  //                ->get()
  //                ->point.y());
  //       Eigen::Vector2d c =
  //           (selected_line.lane_boundary_line->second->nodes.front()
  //                ->get()
  //                ->point.x(),
  //            selected_line.lane_boundary_line->second->nodes.front()
  //                ->get()
  //                ->point.y());
  //       Eigen::Vector2d d =
  //           (selected_line.lane_boundary_line->second->nodes.back()
  //                ->get()
  //                ->point.x(),
  //            selected_line.lane_boundary_line->second->nodes.back()
  //                ->get()
  //                ->point.y());
  //       if (SegmentIntersection(a, b, c, d)) {
  //         selected_line.store = false;
  //         candidated_line.store = false;
  //       }
  //       ++j;
  //     }
  //   }
  // }
  // // 短线与长线交叉 删除短线
  // for (int i = 0, j = 1; i < all_lines_size - 1, j < all_lines_size;
  //      ++i, j = i + 1) {
  //   for (auto& selected_line : all_lines_[all_lane_poses[i]]) {
  //     if (!selected_line.store) {
  //       continue;
  //     }
  //     for (auto& candidated_line : all_lines_[all_lane_poses[j]]) {
  //       if (!candidated_line.store) {
  //         continue;
  //       }
  //       if (selected_line.lane_boundary_line->second->nodes.size() < 2 ||
  //           candidated_line.lane_boundary_line->second->nodes.size() < 2) {
  //         continue;
  //       }
  //       if ((selected_line.lane_boundary_line->second->nodes.back()
  //                .get()
  //                ->point.y() <
  //            candidated_line.lane_boundary_line->second->nodes.front()
  //                .get()
  //                ->point.y()) ||
  //           (candidated_line.lane_boundary_line->second->nodes.back()
  //                .get()
  //                ->point.y() <
  //            selected_line.lane_boundary_line->second->nodes.front()
  //                .get()
  //                ->point.y())) {
  //         continue;
  //       }
  //       if (selected_line.lane_boundary_line->second->nodes.size() > 25 ||
  //           candidated_line.lane_boundary_line->second->nodes.size() <= 25) {
  //         Eigen::Vector2d a =
  //             (selected_line.lane_boundary_line->second->nodes.front()
  //                  ->get()
  //                  ->point.x(),
  //              selected_line.lane_boundary_line->second->nodes.front()
  //                  ->get()
  //                  ->point.y());
  //         Eigen::Vector2d b =
  //             (selected_line.lane_boundary_line->second->nodes.back()
  //                  ->get()
  //                  ->point.x(),
  //              selected_line.lane_boundary_line->second->nodes.back()
  //                  ->get()
  //                  ->point.y());
  //         Eigen::Vector2d c =
  //             (selected_line.lane_boundary_line->second->nodes.front()
  //                  ->get()
  //                  ->point.x(),
  //              selected_line.lane_boundary_line->second->nodes.front()
  //                  ->get()
  //                  ->point.y());
  //         Eigen::Vector2d d =
  //             (selected_line.lane_boundary_line->second->nodes.back()
  //                  ->get()
  //                  ->point.x(),
  //              selected_line.lane_boundary_line->second->nodes.back()
  //                  ->get()
  //                  ->point.y());
  //         if (SegmentIntersection(a, b, c, d)) {
  //           candidated_line.store = false;
  //         }
  //       } else if (selected_line.lane_boundary_line->second->nodes.size() <=
  //                      25 ||
  //                  candidated_line.lane_boundary_line->second->nodes.size() >
  //                      25) {
  //         Eigen::Vector2d a =
  //             (selected_line.lane_boundary_line->second->nodes.front()
  //                  ->get()
  //                  ->point.x(),
  //              selected_line.lane_boundary_line->second->nodes.front()
  //                  ->get()
  //                  ->point.y());
  //         Eigen::Vector2d b =
  //             (selected_line.lane_boundary_line->second->nodes.back()
  //                  ->get()
  //                  ->point.x(),
  //              selected_line.lane_boundary_line->second->nodes.back()
  //                  ->get()
  //                  ->point.y());
  //         Eigen::Vector2d c =
  //             (selected_line.lane_boundary_line->second->nodes.front()
  //                  ->get()
  //                  ->point.x(),
  //              selected_line.lane_boundary_line->second->nodes.front()
  //                  ->get()
  //                  ->point.y());
  //         Eigen::Vector2d d =
  //             (selected_line.lane_boundary_line->second->nodes.back()
  //                  ->get()
  //                  ->point.x(),
  //              selected_line.lane_boundary_line->second->nodes.back()
  //                  ->get()
  //                  ->point.y());
  //         if (SegmentIntersection(a, b, c, d)) {
  //           selected_line.store = false;
  //         }
  //       }
  //     }
  //   }
  // }
}
void ElementsFilter::Clear() { all_lines_.clear(); }

}  // namespace mf
}  // namespace mp
}  // namespace hozon
