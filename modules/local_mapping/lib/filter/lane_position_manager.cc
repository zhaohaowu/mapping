// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Hozon (hozon@hozon.com)
// @file: lane_position_manager.cc
// @brief: lane pos for lane tracker

#include "modules/local_mapping/lib/filter/lane_position_manager.h"

#include <algorithm>
#include <cmath>

#include "modules/local_mapping/utils/lane_utils.h"

namespace hozon {
namespace mp {
namespace lm {

void LanePositionManager::Init() { inited_ = true; }
bool LanePositionManager::IsUnknownLaneline(const LaneLinePtr& laneline_ptr) {
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
  return false;
}
void LanePositionManager::Process(const LaneLinesPtr& laneline_ptrs) {
  HLOG_DEBUG << "***LanePositionManager Process start***";

  SetC0(laneline_ptrs);
  SetReferC0(laneline_ptrs);
  IfCross(laneline_ptrs);

  std::vector<LaneLinePtr> selected_lanelines;  // 被选中需要排序的线
  std::vector<LaneLinePtr> unknown_lanelines;
  std::vector<LaneLinePtr> normal_lanelines;
  selected_lanelines.clear();
  unknown_lanelines.clear();
  normal_lanelines.clear();
  for (const auto& laneline : laneline_ptrs->lanelines) {
    if (IsUnknownLaneline(laneline)) {
      unknown_lanelines.emplace_back(laneline);
      laneline->mf_position = LaneLinePosition::OTHER;
    } else {
      normal_lanelines.emplace_back(laneline);
    }
  }
  SelectLaneLines(normal_lanelines, &selected_lanelines);
  SetLaneLinePosition(selected_lanelines);
}
void LanePositionManager::SetLaneLinePosition(
    const std::vector<LaneLinePtr>& lane_lines) {
  std::vector<std::pair<int, float>> left_lane_index;
  std::vector<std::pair<int, float>> right_lane_index;
  int size = lane_lines.size();
  static int maintain_num_mf = 0;
  static float ref_thresh_mf = 0;
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

    if ((lane_lines[i]->history_mf_line_pos.size() == 2) &&
        maintain_num_mf == 0) {
      int first_pos = static_cast<int>(lane_lines[i]->history_mf_line_pos[0]);
      int sec_pos = static_cast<int>(lane_lines[i]->history_mf_line_pos[1]);
      HLOG_DEBUG << "id:" << lane_lines[i]->id << " ,d:" << d
                 << " ,first_pos:" << first_pos << " ,sec_pos" << sec_pos;
      if ((first_pos + sec_pos == 0) && first_pos > 0) {
        ref_thresh_mf = -0.5;
        maintain_num_mf++;

      } else if ((first_pos + sec_pos == 0) && first_pos < 0) {
        ref_thresh_mf = 0.5;
        maintain_num_mf++;
      }
    }
    // 阈值保持5帧（两次调用）
    if (maintain_num_mf > 10) {
      ref_thresh_mf = 0;
      maintain_num_mf = 0;
    }
    HLOG_DEBUG << "ref_thresh_mf:" << ref_thresh_mf
               << " ,maintain_num_mf:" << maintain_num_mf;
    if (d > ref_thresh_mf) {
      left_lane_index.push_back(std::pair<int, float>(i, d));
    } else {
      right_lane_index.push_back(std::pair<int, float>(i, d));
    }
  }
  if (maintain_num_mf > 0) {
    maintain_num_mf++;
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
      lane_lines[lane_index]->mf_position = kIndex2LanePosMap.at(temp);
      lane_lines[lane_index]->history_mf_line_pos.push_back(
          lane_lines[lane_index]->mf_position);
    } else {
      lane_lines[lane_index]->mf_position = LaneLinePosition::FOURTH_LEFT;
      lane_lines[lane_index]->history_mf_line_pos.push_back(
          lane_lines[lane_index]->mf_position);
    }
  }

  // set right lane pos_type
  for (int i = 0; i < right_lane_index.size(); ++i) {
    int lane_index = right_lane_index[i].first;
    int temp = i + 1;
    if (kIndex2LanePosMap.count(temp) > 0) {
      lane_lines[lane_index]->mf_position = kIndex2LanePosMap.at(temp);
      lane_lines[lane_index]->history_mf_line_pos.push_back(
          lane_lines[lane_index]->mf_position);
    } else {
      lane_lines[lane_index]->mf_position = LaneLinePosition::FOURTH_RIGHT;
      lane_lines[lane_index]->history_mf_line_pos.push_back(
          lane_lines[lane_index]->mf_position);
    }
  }
}

void LanePositionManager::SetC0(const LaneLinesPtr& laneline_ptrs) {
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

void LanePositionManager::SetReferC0(const LaneLinesPtr& laneline_ptrs) {
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
void LanePositionManager::IfCross(const LaneLinesPtr& laneline_ptrs) {
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
void removeDuplicatesWithSet(std::vector<LaneLinePtr>* vec_lane) {
  std::unordered_set<LaneLinePtr> seen;
  std::vector<LaneLinePtr> result;

  for (auto& lane : *vec_lane) {
    if (seen.find(lane) == seen.end()) {
      seen.insert(lane);
      result.push_back(lane);
    }
  }

  vec_lane->swap(result);
}
void LanePositionManager::SelectLaneLines(
    const std::vector<LaneLinePtr>& normal_lanelines,
    std::vector<LaneLinePtr>* selected_lanelines) {
  bool left_valid = false;
  bool right_valid = false;
  // 判断自车左右是否有主车道线
  for (const auto& lane_line : normal_lanelines) {
    if (lane_line->vehicle_points.empty()) {
      continue;
    }
    float d = 0.f;
    // 理论c0相距过近则用参考c0,如果参考和理论符号相反，相信理论cO
    if (lane_line->theory_c0 * lane_line->refer_c0 < 0) {
      d = lane_line->theory_c0;
    }
    if (!lane_line->cross) {
      d = lane_line->theory_c0;
    } else {
      d = lane_line->refer_c0;
    }
    if (lane_line->vehicle_points.front().x() < 0 &&
        lane_line->vehicle_points.back().x() > 0) {
      // d>0 左边,d<0，右边
      if (d > 0 && std::fabs(d) < 4) {
        left_valid = true;
        selected_lanelines->emplace_back(lane_line);
      }
      if (d < 0 && std::fabs(d) < 4) {
        right_valid = true;
        selected_lanelines->emplace_back(lane_line);
      }
    }
    // else {
    //   lane_line->mf_position = LaneLinePosition::OTHER;
    // }
  }
  // 选择需要排序的主车道线
  for (const auto& lane_line : normal_lanelines) {
    if (lane_line->vehicle_points.empty()) {
      continue;
    }
    float d = 0.f;
    // 理论c0相距过近则用参考c0,如果参考和理论符号相反，相信理论cO
    if (lane_line->theory_c0 * lane_line->refer_c0 < 0) {
      d = lane_line->theory_c0;
    }
    if (!lane_line->cross) {
      d = lane_line->theory_c0;
    } else {
      d = lane_line->refer_c0;
    }
    if (left_valid && right_valid) {
      if (lane_line->vehicle_points.front().x() < 0 &&
          lane_line->vehicle_points.back().x() > 0) {
        selected_lanelines->emplace_back(lane_line);
      } else {
        lane_line->mf_position = LaneLinePosition::OTHER;
      }
    } else if (left_valid && !right_valid) {
      if (d < 0 && std::fabs(d) < 4 &&
          lane_line->vehicle_points.front().x() > 0) {
        selected_lanelines->emplace_back(lane_line);
      }
    } else if (!left_valid && right_valid) {
      if (d > 0 && std::fabs(d) < 4 &&
          lane_line->vehicle_points.front().x() > 0) {
        selected_lanelines->emplace_back(lane_line);
      }
    } else if (!left_valid && !right_valid) {
      // 如果左右都没有，则排前方最多4条车道线
      if (std::fabs(d) < 8 && lane_line->vehicle_points.front().x() > 0) {
        selected_lanelines->emplace_back(lane_line);
      }
    }
  }
  // 对选择的线进行去重
  removeDuplicatesWithSet(selected_lanelines);
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
