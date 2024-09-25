/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： occ_process.cc
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/map_fusion_02/modules/geo/occ_process.h"

#include <algorithm>
#include <limits>
#include <numeric>

#include "modules/map_fusion_02/common/calc_util.h"
#include "modules/map_fusion_02/data_manager/location_data_manager.h"
#include "modules/map_fusion_02/data_manager/percep_obj_manager.h"
#include "modules/map_fusion_02/modules/geo/geo_utils.h"
#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace mf {

bool OccProcessor::Init() { return true; }

void OccProcessor::CalDistNearOcc(bool left, const OccRoad::Ptr& occ_road_ptr,
                                  int* index, double* distance) {
  // 从左边线或者右边线找最近的点
  int occ_id = left ? occ_road_ptr->left_occ_id : occ_road_ptr->right_occ_id;
  int find_index = -1;
  double find_dist = std::numeric_limits<double>::max();
  while (occ_id != -1) {
    bool find_flag = false;
    auto iter = std::find_if(
        find_occ_.begin(), find_occ_.end(),
        [occ_id](const auto& elem) { return occ_id == elem->track_id; });
    if (iter != find_occ_.end()) {
      auto occ = *iter;
      const auto& reft_pt = occ->road_points.at(occ->guide_index);
      for (int i = 0; i < static_cast<int>(occ_road_ptr->road_points.size());
           ++i) {
        const auto& pt = occ_road_ptr->road_points[i];
        if ((reft_pt - pt).norm() < find_dist) {
          find_flag = true;
          find_dist = (reft_pt - pt).norm();
          find_index = i;
        }
      }
      // 找不到的话继续往下一个去找
      occ_id = left ? occ->left_occ_id : occ->right_occ_id;
    }
    if (find_flag) {
      break;
    }
  }
  // 至少保留两个点
  if (find_index > static_cast<int>(occ_road_ptr->road_points.size() - 2)) {
    find_index = static_cast<int>(occ_road_ptr->road_points.size() - 2);
  }
  *index = find_index;
  *distance = find_dist;
}

bool OccProcessor::GetFirstNearIndex(const OccRoad::Ptr& occ_road_ptr,
                                     int* first_point_index) {
  if (occ_road_ptr->road_points.size() < 6) {
    return false;
  }
  int left_index = -1;
  double left_dist = 0.0;
  CalDistNearOcc(true, occ_road_ptr, &left_index, &left_dist);
  int right_index = -1;
  double right_dist = 0.0;
  CalDistNearOcc(false, occ_road_ptr, &right_index, &right_dist);
  if (left_index == -1 && right_index == -1) {
    return false;
  }
  if (left_index != -1 && right_index == -1) {
    *first_point_index = left_index;
  } else if (left_index == -1 && right_index != -1) {
    *first_point_index = right_index;
  } else {
    *first_point_index = left_dist < right_dist ? left_index : right_index;
  }
  return true;
}

bool OccProcessor::GetFirstOCCPoints(const OccRoad::Ptr& occ_road_ptr,
                                     int* first_point_index) {
  if (occ_road_ptr->road_points.size() < 6) {
    return false;
  }
  std::vector<float> heading_vec;
  for (const auto& pt : occ_road_ptr->road_points) {
    auto heading =
        math::evaluateHeadingDiff(pt.x(), occ_road_ptr->curve_params.coefs);
    HLOG_DEBUG << "GetFirstOCCPoints track_id: " << occ_road_ptr->track_id
               << ", x: " << pt.x() << ", " << heading;
    heading_vec.push_back(heading);
  }
  int count_low_slope = 0;
  for (int i = 0; i < static_cast<int>(heading_vec.size()) - 5; ++i) {
    if (std::abs(heading_vec[i]) < 0.02) {
      count_low_slope++;
      if (count_low_slope >= 3) {
        *first_point_index = i - 2;
        break;
      }
    } else {
      count_low_slope = 0;
    }
  }
  HLOG_DEBUG << "first_point_index: " << *first_point_index;
  return count_low_slope >= 3;
}

bool OccProcessor::FindOCCGuidePoint() {
  for (auto& occ_road : vec_occ_line_) {
    if (occ_road.second == nullptr) {
      continue;
    }
    if ((!occ_road.second->is_forward) ||
        (occ_road.second->road_points.empty())) {
      continue;
    }
    int point_index = -1;
    if (GetFirstOCCPoints(occ_road.second, &point_index)) {
      occ_road.second->guide_index = point_index;
      find_occ_.push_back(occ_road.second);
    } else {
      // lost_occ_.push_back(occ_road.second);
      occ_road.second->is_forward = false;
    }
  }
  if (find_occ_.empty()) {
    return false;
  }
  // 斜率找不到的用已经找到的线来找第一个点
  for (const auto& occ : lost_occ_) {
    int point_index = -1;
    GetFirstNearIndex(occ, &point_index);
    occ->guide_index = point_index;
  }
  for (auto& occ_road : vec_occ_line_) {
    HLOG_DEBUG << "FindOCCGuidePoint track_id:" << occ_road.second->track_id
               << ", guide_index :" << occ_road.second->guide_index
               << ", group_id:" << occ_road.second->group_id
               << ", left_occ_id:" << occ_road.second->left_occ_id
               << ", right_occ_id" << occ_road.second->right_occ_id;
  }
  return true;
}

void OccProcessor::CompareGroupLines() {
  // 组与组进行比较，返回id对
  if (grouped_lines_.empty()) {
    return;
  }
  std::vector<std::pair<int, OccRoad::Ptr>> curr_group;
  for (auto& group : grouped_lines_) {
    // 对group中的line按照x的从小到大排序
    std::sort(group.begin(), group.end(), [](const auto& a, const auto& b) {
      return a.second->road_points.front().x() <
             b.second->road_points.front().x();
    });
    if (curr_group.empty()) {
      curr_group = group;
      continue;
    }
    // 计算curr_group与group中成对的路沿
    for (auto& occ1 : curr_group) {
      bool flag = false;
      for (auto& occ2 : group) {
        const auto& occ_l1 = occ1.second->road_points;
        const auto& occ_l2 = occ2.second->road_points;
        double ave_width = 0.0;
        if (!math::ComputerLineDis(occ_l1, occ_l2, &ave_width)) {
          continue;
        }
        HLOG_DEBUG << "ComputerLineDis occ1: " << occ1.first
                   << ", occ2: " << occ2.first << ", " << ave_width;
        if (ave_width < 5) {
          continue;
        }
        auto overlay_ratio =
            math::GetOverLayRatioBetweenTwoLane(occ_l1, occ_l2);
        HLOG_DEBUG << "GetOverLayRatioBetweenTwoLane occ1: " << occ1.first
                   << ", y: " << occ_l1.back().y() << ", occ2: " << occ2.first
                   << ", y: " << occ_l2.back().y() << ", " << overlay_ratio;
        if (overlay_ratio < 0.2) {
          continue;
        }
        // 满足要求的occ line
        occ1.second->left_occ_id = occ2.first;
        occ2.second->right_occ_id = occ1.first;
        occ1.second->is_forward = true;
        occ2.second->is_forward = true;
        line_pairs_.emplace_back(occ1.first, occ2.first);
        flag = true;
        break;
      }
      if (flag) {
        break;
      }
    }
    curr_group = group;
  }
}

void OccProcessor::UpdateOCCRoadPoints() {
  for (auto& occ_road : occ_roads_) {
    if (occ_road.second == nullptr) {
      occ_road.second->is_forward = false;
      continue;
    }
    auto guide_index = occ_road.second->guide_index;
    if (guide_index >
        static_cast<int>(occ_road.second->road_points.size()) - 1) {
      occ_road.second->is_forward = false;
      continue;
    }
    if (guide_index == -1) {
      occ_road.second->is_forward = false;
      continue;
    }
    // 保留至少两个点
    occ_road.second->guide_index = std::min(
        guide_index, static_cast<int>(occ_road.second->road_points.size() - 2));
    occ_road.second->road_points.erase(
        occ_road.second->road_points.begin(),
        occ_road.second->road_points.begin() + occ_road.second->guide_index);
  }
}

void OccProcessor::ConstructLinePairs() {
  // 按照纵向距离对occ_road进行分组
  // 对occ_line按照每根线的y值平均值大小进行排序
  std::sort(vec_occ_line_.begin(), vec_occ_line_.end(),
            [](const auto& a, const auto& b) {
              double sumY_A = 0.0;
              double sumY_B = 0.0;
              for (const auto& vec : a.second->road_points) {
                sumY_A += vec.y();
              }
              double avgY_A = sumY_A / a.second->road_points.size();
              for (const auto& vec : b.second->road_points) {
                sumY_B += vec.y();
              }
              double avgY_B = sumY_B / b.second->road_points.size();
              return avgY_A < avgY_B;
            });

  // 遍历vec_occ_line，计算平均距离，判断是否是一个包络，并判断能否构成道
  if (!vec_occ_line_.empty()) {
    // 同组的线
    std::vector<std::pair<int, OccRoad::Ptr>> currentGroup;
    for (const auto& line : vec_occ_line_) {
      double ave_width = std::numeric_limits<double>::max();
      if (!currentGroup.empty()) {
        double ave_width = 0.0;
        if (!math::ComputerLineDis(line.second->road_points,
                                   currentGroup.back().second->road_points,
                                   &ave_width)) {
          continue;
        }
      }
      if (currentGroup.empty() || std::abs(ave_width) <= 1.0) {
        currentGroup.push_back(line);
      } else {
        grouped_lines_.push_back(currentGroup);
        currentGroup.clear();
        currentGroup.push_back(line);
      }
    }
    if (!currentGroup.empty()) {
      grouped_lines_.push_back(currentGroup);
    }
    // 组与组进行比较,返回满足条件的id对
    CompareGroupLines();
  }
  if (FindOCCGuidePoint()) {
    UpdateOCCRoadPoints();
  }
}
bool OccProcessor::Process(ElementMap::Ptr element_map_ptr) {
  occ_roads_ = element_map_ptr->occ_roads;
  for (const auto& occ_pair : occ_roads_) {
    OccRoad::Ptr occ = occ_pair.second;
    if (OccLineFitError(occ) > 2.0) {
      continue;
    }
    std::vector<Eigen::Vector3f> obj_points;
    std::vector<Eigen::Vector3f> occ_road_points;
    const auto& cur_T_w_v_ = LOCATION_MANAGER->GetCurrentPose();
    for (const auto& object : OBJECT_MANAGER->GetInverseHistoryObjs()) {
      Eigen::Vector3d p_local(object->position);
      Eigen::Vector3f p_veh(
          static_cast<float>((cur_T_w_v_.inverse() * p_local).x()),
          static_cast<float>((cur_T_w_v_.inverse() * p_local).y()),
          static_cast<float>((cur_T_w_v_.inverse() * p_local).z()));
      obj_points.emplace_back(p_veh);
    }
    if (obj_points.empty()) {
      continue;
    }
    for (const auto& occ_point : occ->road_points) {
      Eigen::Vector3f occ_road_point(static_cast<float>(occ_point.x()),
                                     static_cast<float>(occ_point.y()),
                                     static_cast<float>(occ_point.z()));
      occ_road_points.emplace_back(occ_road_point);
    }
    if (occ_road_points.empty()) {
      continue;
    }
    if (CheckOppositeLineByObj(occ_road_points, obj_points)) {
      HLOG_DEBUG << "CheckOppositeLineByObj id: " << occ->track_id
                 << ", detect_id: " << occ->detect_id;
      continue;
    }
    vec_occ_line_.emplace_back(occ->track_id, occ);
  }
  return true;
}
void OccProcessor::Clear() {
  line_pairs_.clear();
  vec_occ_line_.clear();
  grouped_lines_.clear();
  occ_roads_.clear();
  find_occ_.clear();
  lost_occ_.clear();
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
