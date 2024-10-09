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

bool MappingPositionManager::IsUnknownLaneline(const LaneLinePtr& laneline_ptr,
                                               bool has_main_line_flag) {
  if (laneline_ptr->vehicle_points.size() == 0) {
    return true;
  }
  // 对于长度小于10米或者在50米外的车道线，
  // lane_pos设置为OTHER;(注意分合流场景等特殊情况的处理) TODO(陈安猛)
  if (laneline_ptr->vehicle_points.back().x() -
              laneline_ptr->vehicle_points.front().x() <
          10 ||
      laneline_ptr->vehicle_points.back().x() < 0 ||
      laneline_ptr->vehicle_points.front().x() > 50) {
    return true;
  }
  // flag_near是指与自车左右车道线中有横向距离小于2m,且没有纵向重叠区域的线的标志位
  // over_lap_flag与自车左右车道线纵向重叠区域大于0.5
  bool flag_near = false;
  bool over_lap_flag = false;
  for (auto& line : lane_lines_ego_) {
    float avg_dist = GetDistBetweenTwoLane(laneline_ptr->vehicle_points,
                                           line->vehicle_points);
    double over_lay_ratio = GetOverLayRatioBetweenTwoLane(laneline_ptr, line);
    if (avg_dist < 2 && over_lay_ratio == 0) {
      flag_near = true;
    }
    if (over_lay_ratio > 0.5) {
      over_lap_flag = true;
    }
  }

  if (flag_near) {
    over_lap_flag = false;
  }
  HLOG_DEBUG << "has_main_line_flag:" << has_main_line_flag
             << "flag_near:" << flag_near
             << " ,over_lap_flag:" << over_lap_flag;
  // 如果本车左右近处有车道线，且和它们没有大于0.5的重叠区域，且前面车道线的起始端点在车前方，则归为unknown
  // 如果本车左右近处有车道线，且和它们横向区域距离较近的前方车道线，也归为unknown
  if (has_main_line_flag && laneline_ptr->vehicle_points.front().x() > 4 &&
          !over_lap_flag ||
      has_main_line_flag && flag_near) {
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
  bool has_main_line_flag = false;
  lane_lines_ego_.clear();
  for (const auto& laneline : laneline_ptrs->lanelines) {
    if (laneline->vehicle_points.back().x() < 0 ||
        laneline->vehicle_points.front().x() > 4) {
      continue;
    }

    double min_dist = std::numeric_limits<double>::max();
    Eigen::Vector3d A = {0, 0, 0};
    Eigen::Vector3d B = laneline->vehicle_points[0];
    Eigen::Vector3d C = laneline->vehicle_points[1];
    for (int j = 0; j < static_cast<int>(laneline->vehicle_points.size());
         ++j) {
      double dist = (A - laneline->vehicle_points[j]).norm();
      if (dist < min_dist) {
        min_dist = dist;
        B = C;
        C = laneline->vehicle_points[j];
      }
    }
    double dist = GetDistPointLane(A, B, C);
    if (dist < 2) {
      has_main_line_flag = true;
      lane_lines_ego_.emplace_back(laneline);
    }
  }
  for (const auto& laneline : laneline_ptrs->lanelines) {
    if (IsUnknownLaneline(laneline, has_main_line_flag)) {
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
// void MappingPositionManager::Process(const LaneLinesPtr& laneline_ptrs) {
//   HLOG_DEBUG << "***MappingPositionManager Process start***";

//   SetC0(laneline_ptrs);
//   SetReferC0(laneline_ptrs);
//   IfCross(laneline_ptrs);

//   std::vector<LaneLinePtr> selected_lanelines;  // 被选中需要排序的线
//   std::vector<LaneLinePtr> unknown_lanelines;
//   std::vector<LaneLinePtr> normal_lanelines;
//   selected_lanelines.clear();
//   unknown_lanelines.clear();
//   normal_lanelines.clear();
//   for (const auto& laneline : laneline_ptrs->lanelines) {
//     if (IsUnknownLaneline(laneline)) {
//       unknown_lanelines.emplace_back(laneline);
//       laneline->position = LaneLinePosition::OTHER;
//     } else {
//       normal_lanelines.emplace_back(laneline);
//     }
//   }
//   // DivideLaneLines(normal_lanelines, &forward_lanelines,
//   &behind_lanelines); SelectLaneLines(normal_lanelines, &selected_lanelines);
//   SetLaneLinePosition(selected_lanelines);
// }
void MappingPositionManager::SetJunction(const LaneLinesPtr& laneline_ptrs) {
  const auto& last_local_map = MAP_MANAGER->GetLocalMap();
  double min_intersection_x = FLT_MAX;
  bool zebra_enable = false;
  int cross_num = 0;
  if (last_local_map->stop_lines_ptr != nullptr) {
    for (const auto& stop_line_ptr :
         last_local_map->stop_lines_ptr->stoplines) {
      if (stop_line_ptr->center_point.x() < min_intersection_x &&
          (stop_line_ptr->center_point.x() >= -10)) {
        min_intersection_x = stop_line_ptr->center_point.x();
        zebra_enable = true;
      }
    }
  }
  if (last_local_map->zebra_crossings_ptr != nullptr) {
    for (const auto& zebra_crossing_ptr :
         last_local_map->zebra_crossings_ptr->zebra_crossings) {
      if (zebra_crossing_ptr->center_point.x() < min_intersection_x &&
          (zebra_crossing_ptr->center_point.x() >= -10)) {
        min_intersection_x = zebra_crossing_ptr->center_point.x();
        zebra_enable = true;
      }
    }
  }
  for (auto& laneline : laneline_ptrs->lanelines) {
    if (laneline->vehicle_points.size() == 0) {
      continue;
    }
    // 防止误检斑马线场景，车道线整个穿过斑马线，且起点在车后面，不用此斑马线。
    if ((laneline->vehicle_points.front().x() < min_intersection_x) &&
        (laneline->vehicle_points.back().x() > min_intersection_x)) {
      cross_num++;
    }
  }
  // 穿过的条数要满足大于50%,防止检测车道线的跳变，还有两边路沿边车道线的影响。
  if (cross_num > 0.5 * laneline_ptrs->lanelines.size()) {
    zebra_enable = false;
  }
  HLOG_DEBUG << "min_intersection_x:" << min_intersection_x
             << " ,cross_num: " << cross_num
             << " ,zebra_enable:" << zebra_enable;
  for (auto& lane_line : laneline_ptrs->lanelines) {
    if (lane_line->vehicle_points.size() == 0) {
      continue;
    }
    double start_point_x = lane_line->vehicle_points.front().x();
    HLOG_DEBUG << "start_point_x:" << start_point_x;
    if (zebra_enable) {
      // 如果 start_point_x 小于等于 min_intersection_x 且 min_intersection_x
      // 非负， 则设置 after_intersection 为
      // false,即属于过路口前的线，否则设置为
      // true。2.5(斑马线的宽度5m的一半)+4m的车辆后轴到前面的长度=6.5
      lane_line->after_intersection =
          !(start_point_x <= min_intersection_x && min_intersection_x >= 6.5 &&
            start_point_x <= 4);
    } else {
      lane_line->after_intersection = false;
    }
    HLOG_DEBUG << "lane id:" << lane_line->id
               << " ,after_intersection:" << lane_line->after_intersection;
  }
}

void MappingPositionManager::SetLaneLinePosition(
    const std::vector<LaneLinePtr>& lane_lines) {
  std::vector<std::pair<int, float>> left_lane_index;
  std::vector<std::pair<int, float>> right_lane_index;
  int size = lane_lines.size();
  static int maintain_num = 0;
  static float ref_thresh = 0;
  for (int i = 0; i < size; ++i) {
    float d = 0.f;
    // 理论c0相距过近则用参考c0,如果参考和理论符号相反，相信理论cO
    if (lane_lines[i]->theory_c0 * lane_lines[i]->refer_c0 < 0) {
      lane_lines[i]->refer_c0 = lane_lines[i]->theory_c0;
    }
    if (!lane_lines[i]->cross) {
      d = lane_lines[i]->theory_c0;
    } else {
      d = lane_lines[i]->refer_c0;
    }

    if ((lane_lines[i]->history_line_pos.size() == 2) && maintain_num == 0) {
      int first_pos = static_cast<int>(lane_lines[i]->history_line_pos[0]);
      int sec_pos = static_cast<int>(lane_lines[i]->history_line_pos[1]);
      HLOG_DEBUG << "id:" << lane_lines[i]->id << " ,d:" << d
                 << " ,first_pos:" << first_pos << " ,sec_pos" << sec_pos;
      if ((first_pos + sec_pos == 0) && first_pos > 0) {
        ref_thresh = -0.5;
        maintain_num++;

      } else if ((first_pos + sec_pos == 0) && first_pos < 0) {
        ref_thresh = 0.5;
        maintain_num++;
      }
    }
    // 阈值保持5帧（两次调用）
    if (maintain_num > 10) {
      ref_thresh = 0;
      maintain_num = 0;
    }
    HLOG_DEBUG << "ref_thresh:" << ref_thresh
               << " ,maintain_num:" << maintain_num;
    if (d > ref_thresh) {
      left_lane_index.push_back(std::pair<int, float>(i, d));
    } else {
      right_lane_index.push_back(std::pair<int, float>(i, d));
    }
  }
  if (maintain_num > 0) {
    maintain_num++;
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
      lane_lines[lane_index]->history_line_pos.push_back(
          lane_lines[lane_index]->position);
    } else {
      lane_lines[lane_index]->position = LaneLinePosition::FOURTH_LEFT;
      lane_lines[lane_index]->history_line_pos.push_back(
          lane_lines[lane_index]->position);
    }
  }

  // set right lane pos_type
  for (int i = 0; i < right_lane_index.size(); ++i) {
    int lane_index = right_lane_index[i].first;
    int temp = i + 1;
    if (kIndex2LanePosMap.count(temp) > 0) {
      lane_lines[lane_index]->position = kIndex2LanePosMap.at(temp);
      lane_lines[lane_index]->history_line_pos.push_back(
          lane_lines[lane_index]->position);
    } else {
      lane_lines[lane_index]->position = LaneLinePosition::FOURTH_RIGHT;
      lane_lines[lane_index]->history_line_pos.push_back(
          lane_lines[lane_index]->position);
    }
  }
  // for (const auto& lane : lane_lines) {
  //   HLOG_DEBUG << "cam lane id:" << lane->id
  //              << " lane pos:" << static_cast<int>(lane->position);
  // }
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
    HLOG_DEBUG << "cam lane_line id" << lane_line->id
               << ", lane_line->theory_c0:" << lane_line->theory_c0;
  }
}

void MappingPositionManager::SetReferC0(const LaneLinesPtr& laneline_ptrs) {
  for (auto& lane_line : laneline_ptrs->lanelines) {
    double init_c0 = FLT_MAX;
    double avg_y = 0.0;
    double sum_y = 0.0;
    int use_point_num =
        static_cast<int>(0.2 * lane_line->vehicle_points.size());
    double avg_interval_xy = 0.0;
    double sum_interval_xy_pt = 0.0;
    // 只选择车距离本车最近的20%的点
    std::vector<std::pair<Eigen::Vector3d, double>> dist_points_pairs;
    dist_points_pairs.clear();
    if (lane_line->vehicle_points.size() == 0) {
      continue;
    }
    for (auto& point : lane_line->vehicle_points) {
      if (lane_line->vehicle_points.back().x() > 10 && point.x() < 0) {
        HLOG_DEBUG << "lane_line id:" << lane_line->id
                   << "lane_line->vehicle_points.back().x():"
                   << lane_line->vehicle_points.back().x();
        continue;
      }
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
    auto ref_point = dist_points_pairs.front().first;
    for (int i = 1; i < dist_points_pairs.size(); ++i) {
      float diff_pt =
          (dist_points_pairs[i].first - dist_points_pairs[i - 1].first).norm();
      sum_interval_xy_pt += diff_pt;
    }
    // 计算xy方向平均间隔
    avg_interval_xy =
        sum_interval_xy_pt / (lane_line->vehicle_points.size() - 1 + 0.00001);
    for (int i = 1; i < dist_points_pairs.size() - 1; ++i) {
      float diff = (dist_points_pairs[i].first - ref_point).norm();
      float diff_y = dist_points_pairs[i].first.y() - ref_point.y();
      HLOG_DEBUG << "x:" << dist_points_pairs[i].first.x()
                 << " ,y:" << dist_points_pairs[i].first.y()
                 << " ,ref x:" << ref_point.x() << " ,ref y:" << ref_point.y()
                 << " ,diff:" << diff
                 << " ,avg_interval_xy:" << avg_interval_xy;
      // diff需要大于xy方向平均间隔，且横向有10cm的差距才加入计算（防止聚集在一起的点团点）
      if ((select_num < use_point_num) &&
          (abs(diff) >= (abs(avg_interval_xy))) && (abs(diff_y) > 0.1)) {
        sum_y += dist_points_pairs[i].first.y();
        ref_point = dist_points_pairs[i].first;
        select_num += 1;
      }
    }
    if (dist_points_pairs.size() == 1) {
      sum_y += dist_points_pairs[0].first.y();
      select_num += 1;
    }
    if (select_num == 0) {
      avg_y = ref_point.y();
      select_num = 1;
      sum_y = avg_y;
    }
    avg_y = sum_y / (select_num + 0.00001);
    lane_line->refer_c0 = avg_y;
    HLOG_DEBUG << "cam lane_line id" << lane_line->id
               << ", lane_line->refer_c0:" << lane_line->refer_c0;
  }
}
void MappingPositionManager::IfCross(const LaneLinesPtr& laneline_ptrs) {
  //   HLOG_INFO << "***************laneline_ptrs->lanelines.size():"
  //             << laneline_ptrs->lanelines.size();
  for (int i = 0; i < laneline_ptrs->lanelines.size() - 1; ++i) {
    // HLOG_INFO << "=========== i:" << i;
    if (laneline_ptrs->lanelines[i]->vehicle_points.back().x() < -10) {
      laneline_ptrs->lanelines[i]->cross = false;
      continue;
    }
    double c0 = laneline_ptrs->lanelines[i]->theory_c0;
    for (int j = i + 1; j < laneline_ptrs->lanelines.size(); ++j) {
      if (laneline_ptrs->lanelines[j]->vehicle_points.back().x() < -10) {
        laneline_ptrs->lanelines[j]->cross = false;
        continue;
      }
      double tmp_c0 = laneline_ptrs->lanelines[j]->theory_c0;
      if (abs(c0 - tmp_c0) < 0.5) {
        HLOG_DEBUG << "cross_id: " << laneline_ptrs->lanelines[i]->id
                   << " ,cross_id:" << laneline_ptrs->lanelines[j]->id;
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

void MappingPositionManager::SelectLaneLines(
    const std::vector<LaneLinePtr>& normal_lanelines,
    std::vector<LaneLinePtr>* selected_lanelines) {
  for (const auto& lane_line : normal_lanelines) {
    if (lane_line->vehicle_points.empty()) {
      continue;
    }
    if (lane_line->vehicle_points.front().x() < 0 &&
        lane_line->vehicle_points.back().x() > 0) {
      selected_lanelines->emplace_back(lane_line);
    } else {
      lane_line->position = LaneLinePosition::OTHER;
    }
  }
}
}  // namespace lm
}  // namespace mp
}  // namespace hozon
