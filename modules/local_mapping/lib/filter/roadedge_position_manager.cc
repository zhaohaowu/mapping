// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Hozon (hozon@hozon.com)
// @file: mapping_position_manager.cc
// @brief: mapping_position for local map

#include "modules/local_mapping/lib/filter/roadedge_position_manager.h"

#include <algorithm>

#include "modules/local_mapping/utils/lane_utils.h"

namespace hozon {
namespace mp {
namespace lm {

void RoadEdgePositionManager::Init() { inited_ = true; }
void RoadEdgePositionManager::Process(const RoadEdgesPtr& roadedges_ptr) {
  std::map<double, RoadEdgePtr> left_road_edges;
  std::map<double, RoadEdgePtr, std::greater<>> right_road_edges;
  std::vector<RoadEdgePtr> unknown_positionroad_edges;
  for (auto& road_edge : roadedges_ptr->road_edges) {
    if (road_edge->vehicle_points.empty()) {
      continue;
    }
    if (road_edge->vehicle_points.back().x() -
                road_edge->vehicle_points.front().x() <
            10 ||
        road_edge->vehicle_points.back().x() < 0 ||
        road_edge->vehicle_points.front().x() > 60) {
      road_edge->position = LaneLinePosition::OTHER;
      unknown_positionroad_edges.emplace_back(road_edge);
      continue;
    }

    std::vector<Eigen::Vector3d> points;
    for (const auto& point : road_edge->vehicle_points) {
      if (point.x() > -20 && point.x() < 20) {
        points.emplace_back(point);
      }
    }
    if (points.size() < 4) {
      road_edge->position = LaneLinePosition::OTHER;
      unknown_positionroad_edges.emplace_back(road_edge);
      continue;
    }

    if (road_edge->vehicle_curve.coeffs[0] > 0) {
      left_road_edges[road_edge->vehicle_curve.coeffs[0]] = road_edge;
    } else if (road_edge->vehicle_curve.coeffs[0] < 0) {
      right_road_edges[road_edge->vehicle_curve.coeffs[0]] = road_edge;
    }
  }
  roadedges_ptr->road_edges.clear();
  roadedges_ptr->road_edges = unknown_positionroad_edges;

  auto pre_it = left_road_edges.begin();
  if (pre_it != left_road_edges.end()) {
    pre_it->second->position = static_cast<LaneLinePosition>(-1);
    roadedges_ptr->road_edges.emplace_back(pre_it->second);
    int left_index = -2;
    for (auto it = std::next(pre_it); it != left_road_edges.end(); it++) {
      if (it->first - pre_it->first > 3) {
        it->second->position = static_cast<LaneLinePosition>(left_index--);
        roadedges_ptr->road_edges.emplace_back(it->second);
      } else {
        it->second->position = pre_it->second->position;
        roadedges_ptr->road_edges.emplace_back(it->second);
      }
      pre_it = it;
    }
  }
  pre_it = right_road_edges.begin();
  if (pre_it != right_road_edges.end()) {
    pre_it->second->position = static_cast<LaneLinePosition>(1);
    roadedges_ptr->road_edges.emplace_back(pre_it->second);
    int right_index = 2;
    for (auto it = std::next(pre_it); it != right_road_edges.end(); it++) {
      if (pre_it->first - it->first > 3) {
        it->second->position = static_cast<LaneLinePosition>(right_index++);
        roadedges_ptr->road_edges.emplace_back(it->second);
      } else {
        it->second->position = pre_it->second->position;
        roadedges_ptr->road_edges.emplace_back(it->second);
      }
      pre_it = it;
    }
  }
}
}  // namespace lm
}  // namespace mp
}  // namespace hozon
