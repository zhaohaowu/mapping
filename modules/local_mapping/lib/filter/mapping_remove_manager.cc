// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Hozon (hozon@hozon.com)
// @file: mapping_filter_manager.cc
// @brief: delete outlier local map

#include "modules/local_mapping/lib/filter/mapping_remove_manager.h"

#include <algorithm>
#include <utility>

#include "base/utils/log.h"
#include "modules/local_mapping/lib/datalogger/map_manager.h"
#include "modules/local_mapping/utils/lane_utils.h"

namespace hozon {
namespace mp {
namespace lm {

bool MappingRemoveManager::Init() { return true; }

// 合流分流判断
bool MappingRemoveManager::IsForkConvergelike(
    const LaneTargetConstPtr& left_line, const LaneTargetConstPtr& right_line) {
  const std::vector<Eigen::Vector3d>& point_set1 =
      left_line->GetConstTrackedObject()->vehicle_points;

  const std::vector<Eigen::Vector3d>& point_set2 =
      right_line->GetConstTrackedObject()->vehicle_points;

  if (point_set1.empty() || point_set2.empty()) {
    return false;
  }

  // 分合流线场景
  float line1_length = GetLength(point_set1);
  float line2_length = GetLength(point_set2);
  HLOG_DEBUG << "line1_length:" << line1_length;
  float short_line_length = std::min(line1_length, line2_length);

  std::vector<double> dist_list;
  std::vector<Eigen::Vector3d> point_list;
  dist_list.clear();
  point_list.clear();
  // 前提两条线的车辆系下的点已经从近到远排好序
  float overlay_min = std::max(point_set1.front().x(), point_set2.front().x());
  float overlay_max = std::min(point_set1.back().x(), point_set2.back().x());

  std::vector<Eigen::Vector3d> overlay_point_set1;
  std::vector<Eigen::Vector3d> overlay_point_set2;
  overlay_point_set1.clear();
  overlay_point_set2.clear();

  for (const auto& point : point_set1) {
    if (point.x() < overlay_min || point.x() > overlay_max) {
      continue;
    }
    overlay_point_set1.push_back(point);
  }

  for (const auto& point : point_set2) {
    if (point.x() < overlay_min || point.x() > overlay_max) {
      continue;
    }
    overlay_point_set2.push_back(point);
  }

  if (overlay_point_set1.size() < 2 || overlay_point_set2.size() < 2) {
    return false;
  }

  Eigen::Vector3d A, B, C;
  for (int i = 0, j = 0;
       i < overlay_point_set1.size() && j < overlay_point_set2.size(); ++i) {
    A = overlay_point_set1[i];
    if (A.x() <= overlay_point_set2[0].x()) {
      B = overlay_point_set2[0];
      C = overlay_point_set2[1];
    } else if (A.x() >= overlay_point_set2.back().x()) {
      B = overlay_point_set2[overlay_point_set2.size() - 2];
      C = overlay_point_set2[overlay_point_set2.size() - 1];
    } else if (A.x() < overlay_point_set2[j].x()) {
      B = overlay_point_set2[j - 1];
      C = overlay_point_set2[j];
    } else {
      ++j;
      --i;
      continue;
    }
    double dist = GetDistPointLane(A, B, C);
    dist_list.push_back(dist);
    point_list.push_back(A);
  }

  int order_times = 0;
  int reorder_times = 0;
  int ambiguous_times = 0;
  float equal_length = 0.0;
  int bins = 0;
  for (int i = 0, j = 0; i < point_list.size() && j < point_list.size();) {
    double point_dis = (point_list[i].head(2) - point_list[j].head(2)).norm();
    if (point_dis > 4.0) {
      HLOG_DEBUG << "TEST i:" << i << ",j:" << j << ", point_dis:" << point_dis
                 << ",dis:" << dist_list[j] - dist_list[i];
      bins++;
      if (dist_list[j] - dist_list[i] > 0.15) {
        order_times++;
      } else if (dist_list[j] - dist_list[i] < -0.15) {
        reorder_times++;
      } else if (abs(dist_list[j] - dist_list[i]) < 0.03) {
        ambiguous_times++;
      } else if (abs(dist_list[j] - dist_list[i]) < 0.08) {
        // 横向距离在8cm内认为是重叠区域
        equal_length += point_dis;
      }
      i = j;
      j++;
    } else {
      j++;
    }
  }

  HLOG_DEBUG << "[IsForkConvergelike], bins:" << bins << "order_rate:"
             << 1.0 * order_times / (bins - ambiguous_times + 0.001)
             << "reorder_rate:"
             << 1.0 * reorder_times / (bins - ambiguous_times + 0.001)
             << "same pos length:" << equal_length;

  if (bins <= 2) {
    return false;
  }

  // 以下是判定两条线是否是分合流场景还是一条线存在误检的情况，重叠区域小于20m
  if ((1.0 * order_times / (bins - ambiguous_times + 0.001) > 0.65 ||
       1.0 * reorder_times / (bins - ambiguous_times + 0.001) > 0.65) &&
      equal_length < 15) {
    return true;
  }

  return false;
}

// 路口前后判断
void MappingRemoveManager::SetIntersection(
    std::vector<LaneTrackerPtr>* trackers) {
  const auto& last_local_map = MAP_MANAGER->GetLocalMap();
  double min_intersection_x = FLT_MAX;
  if (last_local_map->stop_lines_ptr != nullptr) {
    for (const auto& stop_line_ptr :
         last_local_map->stop_lines_ptr->stoplines) {
      if (stop_line_ptr->center_point.x() < min_intersection_x &&
          (stop_line_ptr->center_point.x() >= -10)) {
        min_intersection_x = stop_line_ptr->center_point.x();
      }
    }
  }
  if (last_local_map->zebra_crossings_ptr != nullptr) {
    for (const auto& zebra_crossing_ptr :
         last_local_map->zebra_crossings_ptr->zebra_crossings) {
      if (zebra_crossing_ptr->center_point.x() < min_intersection_x &&
          (zebra_crossing_ptr->center_point.x() >= -10)) {
        min_intersection_x = zebra_crossing_ptr->center_point.x();
      }
    }
  }

  for (int i = 0; i < trackers->size(); ++i) {
    const auto& proposal_line = trackers->at(i)->GetConstTarget();
    const auto& lane_line = proposal_line->GetConstTrackedObject();
    double mid_point_x = (lane_line->vehicle_points.front().x() +
                          lane_line->vehicle_points.back().x()) /
                         2.0;
    // lane_line->after_intersection为true则为路口后
    lane_line->after_intersection = mid_point_x > min_intersection_x;
  }
}

// 与参考线可构成车道的非参考线
std::map<int, std::vector<LaneTrackerPtr>>
MappingRemoveManager::GenerateLanesFromRefLines(
    const LaneTrackerPtr& ref_track,
    const std::vector<LaneTrackerPtr>* remove_schedule_lanes) {
  std::map<int, std::vector<LaneTrackerPtr>> laneTrackerMap;
  for (int i = 0; i < remove_schedule_lanes->size(); ++i) {
    const auto& remove_line = remove_schedule_lanes->at(i)->GetConstTarget();
    const auto& ref_line = ref_track->GetConstTarget();
    // 只选择overlay部分来计算距离
    float avg_dist = 0;
    double over_lay_ratio =
        GetOverLayRatioBetweenTwoLane(ref_line->GetConstTrackedObject(),
                                      remove_line->GetConstTrackedObject());
    if (over_lay_ratio > 0.3) {
      avg_dist = GetAvgDistBetweenTwoLane(ref_line->GetConstTrackedObject(),
                                          remove_line->GetConstTrackedObject());
    }
    // HLOG_DEBUG << "lane_1 id:" << ref_line->Id()
    //            << "propoal_lane id:" <<
    //            remove_line->GetConstTrackedObject()->id
    //            << ",avg_dist:" << avg_dist;
    // 与参考线的平均距离大于2.5，认为可构成车道
    if (avg_dist > 2.5f) {
      laneTrackerMap[ref_line->Id()].emplace_back(remove_schedule_lanes->at(i));
    } else {
      // 小于2.5m的，分两种情况：
      if (over_lay_ratio > 0.3) {
        // 1、overlay大于0.3，且非分合流场景的则直接删除待删除线；
        if (!IsForkConvergelike(ref_line, remove_line)) {
          remove_index_.insert(remove_line->Id());
        }

      } else {
        // 2、overlay小于0.3的则判断是否在马路对面，且lost_age大于3；
        if (isOnOpposite(remove_line->GetConstTrackedObject()) &&
            remove_line->GetConstTrackedObject()->lost_age > 3) {
          remove_index_.insert(remove_line->Id());
        }
      }
    }
  }

  return laneTrackerMap;
}

// 选点,overlay 范围内选20%的点
void MappingRemoveManager::SelectPoints(const LaneLinePtr& laneline,
                                        std::vector<Eigen::Vector3d>* point_set,
                                        int min_x, int max_x) {
  point_set->clear();
  // 只选择车距离本车最近的20%的点
  std::vector<std::pair<Eigen::Vector3d, double>> dist_points_pairs;

  int use_point_num = static_cast<int>(0.2 * laneline->vehicle_points.size());

  for (auto& point : laneline->vehicle_points) {
    if (point.x() > max_x || point.x() < min_x) {
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
  for (int i = 0; i < dist_points_pairs.size(); ++i) {
    if (select_num < use_point_num) {
      point_set->emplace_back(dist_points_pairs[i].first);

      select_num += 1;
    }
  }
}
// 计算两车道线的距离（只取overlay部分的距离本车最近的20%作为计算参考）
float MappingRemoveManager::GetAvgDistBetweenTwoLane(
    const LaneLinePtr& laneline_left, const LaneLinePtr& laneline_right) {
  std::vector<Eigen::Vector3d> point_set_left;
  std::vector<Eigen::Vector3d> point_set_right;
  int left_size = laneline_left->vehicle_points.size();
  int right_size = laneline_right->vehicle_points.size();
  if (left_size == 0 || right_size == 0) {
    return FLT_MAX;
  }
  float min_x = std::max(laneline_left->vehicle_points[0].x(),
                         laneline_right->vehicle_points[0].x());
  float max_x = std::min(laneline_left->vehicle_points[left_size - 1].x(),
                         laneline_right->vehicle_points[right_size - 1].x());
  // 选点，选择overlay部分的距离本车最近的20%的点来计算距离
  SelectPoints(laneline_left, &point_set_left, min_x, max_x);
  SelectPoints(laneline_right, &point_set_right, min_x, max_x);

  HLOG_DEBUG << "point_set_left:" << point_set_left.size()
             << "point_set_right:" << point_set_right.size();
  if (point_set_left.size() == 0 || point_set_right.size() == 0) {
    HLOG_ERROR << "LocanMapping: point_set size == 0.";
    return FLT_MAX;
  }
  float avg_dist = GetDistBetweenTwoLane(point_set_left, point_set_right);

  return avg_dist;
}

// 如果没有参考线，待删除线之间的博弈删除
bool MappingRemoveManager::DeleteLaneisTooNear(
    const LaneTrackerPtr& lanetarget, std::vector<LaneTrackerPtr>* trackers) {
  for (int i = 0; i < trackers->size(); ++i) {
    const auto& proposal_line = trackers->at(i)->GetConstTarget();
    const auto& laneline =
        lanetarget->GetConstTarget()->GetConstTrackedObject();
    int repete_flag = 0;
    // 已删除的线不参与比较
    for (auto& id : remove_index_) {
      if (proposal_line->Id() == id ||
          lanetarget->GetConstTarget()->Id() == id) {
        repete_flag = 1;
      }
    }
    if (lanetarget->GetConstTarget()->Id() == proposal_line->Id() ||
        repete_flag == 1) {
      continue;
    }
    int total_tracked_count = 0;
    int max_continue_count = 0;
    TrackedStatic(proposal_line, &total_tracked_count, &max_continue_count);
    double over_lay_ratio = GetOverLayRatioBetweenTwoLane(
        laneline, proposal_line->GetConstTrackedObject());
    // 比较的两条线需要有overlay,且大于0.3
    if (over_lay_ratio > 0.3) {
      // 车道距离小于2.5m
      float avg_dist = GetAvgDistBetweenTwoLane(
          laneline, proposal_line->GetConstTrackedObject());
      // HLOG_DEBUG << "DeleteLaneisTooNear lane_1 id:" << laneline->id
      //            << "propoal_lane id:"
      //            << proposal_line->GetConstTrackedObject()->id
      //            << ",avg_dist:" << avg_dist;
      // -------------------------------------分割线-------------------------------------
      // 待删除线要删除需要满足条件：
      // 1、两条线之间距离小于2.5m;
      // 2、当前线的lost_age要大于对比的线；
      // 3、状态==0 （条件可删除）；
      // 4、对比线的必须要成熟的，track_count>4,否则存在删除后，没有替代的线；
      // 5、对比线的连续跟踪次数大于1；
      // 6、对比线的长度大于30m；
      // 7、非分合流场景

      if (avg_dist < 2.5f &&
          (laneline->lost_age >
           proposal_line->GetConstTrackedObject()->lost_age) &&
          (static_cast<int>(laneline->state) == 0) &&
          (proposal_line->GetConstTrackedObject()->tracked_count > 4) &&
          (max_continue_count > 1) &&
          (GetLength(proposal_line->GetConstTrackedObject()->vehicle_points) >
           30.0) &&
          (!IsForkConvergelike(lanetarget->GetConstTarget(), proposal_line))) {
        return true;
      }
    }
  }

  return false;
}
// 判断车道线是否是参考线
bool MappingRemoveManager::isReference(const LaneTargetConstPtr& target,
                                       std::vector<LaneTrackerPtr>* trackers) {
  // 长度条件，长度大于10m；跟踪状态，未lost,点稳定性好,路口前的线（路口对面的一般质量不太好，不作为参考线）
  auto laneline = target->GetConstTrackedObject();
  // HLOG_DEBUG << "lane id:" << laneline->id << ", tracked_count"
  //            << laneline->tracked_count;
  // HLOG_DEBUG << "lane id:" << laneline->id
  //            << ",state:" << static_cast<int>(laneline->state)
  //            << ",lost_age:" << laneline->lost_age;
  // HLOG_DEBUG << ",lane_length:" << GetLength(laneline->vehicle_points);

  int total_tracked_count = 0;
  int max_continue_count = 0;
  TrackedStatic(target, &total_tracked_count, &max_continue_count);
  // HLOG_DEBUG << "target->IsTracked():" << target->IsTracked()
  //            << "total_tracked_count:" << total_tracked_count
  //            << " ,max_continue_count:" << max_continue_count;

  // 非路口：
  // 1、跟踪状态；
  // 2、长度大于50m；
  // 3、最大连续跟踪帧数大于3帧；
  // 4、总跟踪帧数大于5帧；
  // 5、非路口对面；
  if (target->IsTracked() && GetLength(laneline->vehicle_points) > 50.0 &&
      max_continue_count > 3 && total_tracked_count > 5 &&
      (!isOnOpposite(laneline))) {
    // HLOG_DEBUG << "OK isReference";
    return true;
  }
  // 路口对面：
  // 1、跟踪状态；
  // 2、长度大于20m；
  // 3、最大连续跟踪帧数大于1帧；
  // 4、总跟踪帧数大于3帧；
  // 5、是路口对面；
  if (target->IsTracked() && GetLength(laneline->vehicle_points) > 20.0 &&
      max_continue_count > 2 && total_tracked_count > 4 &&
      (isOnOpposite(laneline))) {
    // HLOG_DEBUG << "OK isReference";
    return true;
  }

  return false;
}
// 判断是路口对面，且localmap的起始点大于10m
bool MappingRemoveManager::isOnOpposite(const LaneLinePtr& laneline) {
  bool pos_flag = false;
  if (laneline->vehicle_points.size() > 0) {
    if (laneline->vehicle_points[0].x() > 10 && laneline->after_intersection) {
      pos_flag = true;
    }
  }
  return pos_flag;
}
// 统计跟踪状态：10帧内的跟踪上的帧数，以及最近连续跟踪上的帧数
void MappingRemoveManager::TrackedStatic(const LaneTargetConstPtr& target,
                                         int* true_count,
                                         int* max_continue_count) {
  auto tracked_state = target->lastest_n_tracked_state_;
  *true_count = 0;
  int curr_continue_count = 0;
  *max_continue_count = 0;
  for (int i = tracked_state.size() - 1; i > 0; --i) {
    if (tracked_state[i]) {
      (*true_count)++;
      curr_continue_count++;
      *max_continue_count = std::max(curr_continue_count, *max_continue_count);

    } else {
      curr_continue_count = -10;
    }
  }
}

void MappingRemoveManager::Process(std::vector<LaneTrackerPtr>* trackers) {
  HLOG_DEBUG << "***MappingRemoveManager Process start***";
  remove_index_.clear();
  // 设置路口前后的标志
  SetIntersection(trackers);
  // 分成三部分（参考线、车后的线和待删除线）。
  std::vector<LaneTrackerPtr> reference_lanes;        // 参考线
  std::vector<LaneTrackerPtr> behind_vehicle_lanes;   // 车后面的线
  std::vector<LaneTrackerPtr> remove_schedule_lanes;  // 待删除线
  reference_lanes.clear();
  behind_vehicle_lanes.clear();
  remove_schedule_lanes.clear();
  for (int i = 0; i < trackers->size(); ++i) {
    const auto& proposal_line = trackers->at(i)->GetConstTarget();
    const auto& laneline = proposal_line->GetConstTrackedObject();
    // 如果满足参考线条件，加入参考线列表
    if (isReference(proposal_line, trackers)) {
      reference_lanes.emplace_back(trackers->at(i));
    } else if (laneline->vehicle_points.size() > 0) {
      // 如果不满足参考线，则进行如下处理：

      // 1、车道线完全在本车后面，不再删除
      // 2、其它的归为待删除线

      if (laneline->vehicle_points[laneline->vehicle_points.size() - 1].x() <
          0) {
        behind_vehicle_lanes.emplace_back(trackers->at(i));
      } else {
        remove_schedule_lanes.emplace_back(trackers->at(i));
      }
    }
  }

  // 如果没有参考线，则待删除之间进行判断是否要删除。（有风险，暂停）
  // if (reference_lanes.size() == 0) {
  //   for (auto& remove_lane : remove_schedule_lanes) {
  //     if (DeleteLaneisTooNear(remove_lane, &remove_schedule_lanes)) {
  //       remove_index_.insert(remove_lane->GetConstTarget()->Id());
  //     }
  //   }
  // }

  // 参考线不再删除，待删除线参考删除线来删除。
  // 1、不能与参考线构成车道的待删除线，直接删除；
  // 2、和参考线可以构成车道的待删除线之间再进行判断是否需要删除；
  for (auto& ref_lane : reference_lanes) {
    // 可与参考线生成车道的待删除线map
    auto ref_delete_lane_map =
        GenerateLanesFromRefLines(ref_lane, &remove_schedule_lanes);

    // 有参考线时,待删除线之间的博弈(有风险，暂停)

    // for (auto& remove_lane :
    // ref_delete_lane_map[ref_lane->GetConstTarget()->Id()]) {
    //   if (DeleteLaneisTooNear(remove_lane,
    //                           &ref_delete_lane_map[ref_lane->GetConstTarget()->Id()]))
    //                           {
    //     remove_index_.insert(remove_lane->GetConstTarget()->Id());
    //   }
    // }
  }
  trackers->erase(std::remove_if(trackers->begin(), trackers->end(),
                                 [&](const auto& tracker) {
                                   return remove_index_.count(
                                              tracker->GetConstTarget()->Id()) >
                                          0;
                                 }),
                  trackers->end());
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
