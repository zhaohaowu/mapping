// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Hozon (hozon@hozon.com)
// @file: mapping_position_manager.cc
// @brief: mapping_position for local map

#include "modules/local_mapping/lib/filter/mapping_position_manager.h"

#include <algorithm>
#include <utility>
#include "base/utils/log.h"
#include "modules/local_mapping/lib/datalogger/map_manager.h"
#include "modules/local_mapping/utils/lane_utils.h"

namespace hozon {
namespace mp {
namespace lm {

bool MappingPositionManager::Init() {
  inited_ = true;
  return true;
}

bool MappingPositionManager::IsUnknownLaneline(
    const LaneLinePtr& laneline_ptr) {
  // 对于长度小于10米或者在50米外的车道线，
  // lane_pos设置为OTHER;(注意分合流场景等特殊情况的处理) TODO(陈安猛)
  if (laneline_ptr->vehicle_points.back().x() -
              laneline_ptr->vehicle_points.front().x() <
          10 ||
      laneline_ptr->vehicle_points.back().x() < -10 ||
      laneline_ptr->vehicle_points.front().x() > 50) {
    return true;
  }
  return false;
}

void MappingPositionManager::Process(const LaneLinesPtr& laneline_ptrs) {
  HLOG_DEBUG << "***MappingPositionManager Process start***";

  // 给所有车道线打上一个是否位于路口前后的标记
  SetJunction(laneline_ptrs);
  SetC0(laneline_ptrs);
  SetReferC0(laneline_ptrs);
  IfCross(laneline_ptrs);

  std::vector<LaneLinePtr> forward_lanelines;  // 路口外侧
  std::vector<LaneLinePtr> behind_lanelines;   // 路口内侧
  std::vector<LaneLinePtr> unknown_lanelines;
  std::vector<LaneLinePtr> normal_lanelines;

  for (const auto& laneline : laneline_ptrs->lanelines) {
    if (IsUnknownLaneline(laneline)) {
      unknown_lanelines.emplace_back(laneline);
      laneline->position = LaneLinePosition::OTHER;
    } else {
      normal_lanelines.emplace_back(laneline);
    }
  }
  DivideLaneLines(normal_lanelines, &forward_lanelines, &behind_lanelines);
  SetLaneLinePosition(forward_lanelines);
  SetLaneLinePosition(behind_lanelines);
}

void MappingPositionManager::SetJunction(const LaneLinesPtr& laneline_ptrs) {
  const auto& last_local_map = MAP_MANAGER->GetLocalMap();
  double min_intersection_x = FLT_MAX;
  if (last_local_map->stop_lines_ptr != nullptr) {
    for (const auto& stop_line_ptr :
         last_local_map->stop_lines_ptr->stoplines) {
      if (stop_line_ptr->center_point.x() < min_intersection_x) {
        min_intersection_x = stop_line_ptr->center_point.x();
      }
    }
  }
  if (last_local_map->zebra_crossings_ptr != nullptr) {
    for (const auto& zebra_crossing_ptr :
         last_local_map->zebra_crossings_ptr->zebra_crossings) {
      if (zebra_crossing_ptr->center_point.x() < min_intersection_x) {
        min_intersection_x = zebra_crossing_ptr->center_point.x();
      }
    }
  }
  for (auto& lane_line : laneline_ptrs->lanelines) {
    double mid_point_x = (lane_line->vehicle_points.front().x() +
                          lane_line->vehicle_points.back().x()) /
                         2.0;
    lane_line->after_intersection = mid_point_x > min_intersection_x;
  }
}

void MappingPositionManager::SetLaneLinePosition(
    const std::vector<LaneLinePtr>& lane_lines) {
  std::vector<std::pair<int, float>> left_lane_index;
  std::vector<std::pair<int, float>> right_lane_index;
  int size = lane_lines.size();

  for (int i = 0; i < size; ++i) {
    float d = 0.f;
    // 理论c0相距过近则用参考c0
    if (!lane_lines[i]->cross) {
      d = lane_lines[i]->theory_c0;
    } else {
      d = lane_lines[i]->refer_c0;
    }
    if (d > 0) {
      left_lane_index.push_back(std::pair<int, float>(i, d));
    } else {
      right_lane_index.push_back(std::pair<int, float>(i, d));
    }
  }
  // left_lane: sort by decrease
  std::sort(left_lane_index.begin(), left_lane_index.end(),
            [](std::pair<int, float>& a, std::pair<int, float>& b) {
              return a.second < b.second;
            });
  // right_lane: sort by increase
  std::sort(right_lane_index.begin(), right_lane_index.end(),
            [](std::pair<int, float>& a, std::pair<int, float>& b) {
              return a.second > b.second;
            });

  // set left lane pos_type
  for (int i = 0; i < left_lane_index.size(); ++i) {
    int lane_index = left_lane_index[i].first;
    int temp = 0 - i - 1;
    if (kIndex2LanePosMap.count(temp) > 0) {
      lane_lines[lane_index]->position = kIndex2LanePosMap.at(temp);
    } else {
      lane_lines[lane_index]->position = LaneLinePosition::FOURTH_LEFT;
    }
  }

  // set right lane pos_type
  for (int i = 0; i < right_lane_index.size(); ++i) {
    int lane_index = right_lane_index[i].first;
    int temp = i + 1;
    if (kIndex2LanePosMap.count(temp) > 0) {
      lane_lines[lane_index]->position = kIndex2LanePosMap.at(temp);
    } else {
      lane_lines[lane_index]->position = LaneLinePosition::FOURTH_RIGHT;
    }
  }
  //   for (const auto& lane : lane_lines) {
  //     HLOG_INFO << "cam lane id:" << lane->id
  //               << " lane pos:" << static_cast<int>(lane->position);
  //   }
}

void MappingPositionManager::SetC0(const LaneLinesPtr& laneline_ptrs) {
  for (auto& lane_line : laneline_ptrs->lanelines) {
    double init_c0 = FLT_MAX;
    double final_y = 0.0;
    for (auto& point : lane_line->vehicle_points) {
      double distance = sqrt(point.x() * point.x() + point.y() * point.y());
      if (distance < init_c0) {
        init_c0 = distance;
        final_y = point.y();
      }
    }
    lane_line->theory_c0 = final_y;
    // HLOG_INFO << "cam lane_line id" << lane_line->id
    //           << ", lane_line->theory_c0:" << lane_line->theory_c0;
  }
}

void MappingPositionManager::SetReferC0(const LaneLinesPtr& laneline_ptrs) {
  for (auto& lane_line : laneline_ptrs->lanelines) {
    double init_c0 = FLT_MAX;
    double avg_y = 0.0;
    double sum_y = 0.0;
    int use_point_num =
        static_cast<int>(0.2 * lane_line->vehicle_points.size());

    // 只选择车距离本车最近的20%的点
    std::vector<std::pair<Eigen::Vector3d, double>> dist_points_pairs;
    for (auto& point : lane_line->vehicle_points) {
      double distance = sqrt(point.x() * point.x() + point.y() * point.y());
      dist_points_pairs.emplace_back(
          std::pair<Eigen::Vector3d, double>(point, distance));
    }
    std::sort(dist_points_pairs.begin(), dist_points_pairs.end(),
              [](std::pair<Eigen::Vector3d, double>& a,
                 std::pair<Eigen::Vector3d, double>& b) {
                return a.second < b.second;
              });
    int select_num = 0;
    for (int i = 0; i < dist_points_pairs.size(); ++i) {
      if (select_num < use_point_num) {
        sum_y += dist_points_pairs[i].first.y();

        select_num += 1;
      }
    }
    avg_y = sum_y / (use_point_num + 0.001);
    lane_line->refer_c0 = avg_y;
    // HLOG_INFO << "cam lane_line id" << lane_line->id
    //           << ", lane_line->refer_c0:" << lane_line->refer_c0;
  }
}
void MappingPositionManager::IfCross(const LaneLinesPtr& laneline_ptrs) {
  //   HLOG_INFO << "***************laneline_ptrs->lanelines.size():"
  //             << laneline_ptrs->lanelines.size();
  for (int i = 0; i < laneline_ptrs->lanelines.size() - 1; ++i) {
    // HLOG_INFO << "=========== i:" << i;
    double c0 = laneline_ptrs->lanelines[i]->theory_c0;
    for (int j = i + 1; j < laneline_ptrs->lanelines.size(); ++j) {
      double tmp_c0 = laneline_ptrs->lanelines[j]->theory_c0;
      if (abs(c0 - tmp_c0) < 0.5) {
        laneline_ptrs->lanelines[i]->cross = true;
        laneline_ptrs->lanelines[j]->cross = true;
      }
    }
  }
}
void MappingPositionManager::DivideLaneLines(
    const std::vector<LaneLinePtr>& normal_lanelines,
    std::vector<LaneLinePtr>* forward_lanelines,
    std::vector<LaneLinePtr>* behind_lanelines) {
  for (const auto& lane_line : normal_lanelines) {
    if (lane_line->vehicle_points.empty()) {
      continue;
    }
    // 将车道线分为路口前（外侧），路口后（内侧）。
    if (lane_line->after_intersection) {
      forward_lanelines->emplace_back(lane_line);

    } else {
      behind_lanelines->emplace_back(lane_line);
    }
  }
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
