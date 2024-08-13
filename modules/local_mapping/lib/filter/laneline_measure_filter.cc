// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: hozon
// @file: simple_lane_tracker_pipeline.cc
// @brief: associate history lane_track to current detected lane　object
#include "modules/local_mapping/lib/filter/laneline_measure_filter.h"
#include <math.h>

#include <limits>
#include <unordered_set>
#include <utility>

#include "modules/local_mapping/utils/lane_utils.h"
#include "perception-lib/lib/config_manager/config_manager.h"
namespace hozon {
namespace mp {
namespace lm {

namespace perception_lib = hozon::perception::lib;

bool LaneMeasurementFilter::Init() {
  auto* config_manager = perception_lib::ConfigManager::Instance();
  const perception_lib::ModelConfig* model_config = nullptr;
  if (!config_manager->GetModelConfig(Name(), &model_config)) {
    HLOG_ERROR << "Parse config failed! Name: " << Name();
    return false;
  }

  if (!model_config->get_value("short_lane_length_thresh",
                               &short_lane_length_thresh_)) {
    HLOG_ERROR << "get short_lane_length_thresh failed...";
    return false;
  }

  if (!model_config->get_value("same_pos_distance_thresh",
                               &same_pos_distance_thresh_)) {
    HLOG_ERROR << "get same_pos_distance_thresh failed...";
    return false;
  }

  if (!model_config->get_value("hight_overlay_length_ratio_thresh",
                               &hight_overlay_length_ratio_thresh_)) {
    HLOG_ERROR << "get hight_overlay_length_ratio_thresh failed...";
    return false;
  }

  if (!model_config->get_value("low_overlay_length_ratio_thresh",
                               &low_overlay_length_ratio_thresh_)) {
    HLOG_ERROR << "get low_overlay_length_ratio_thresh failed ...";
    return false;
  }

  if (!model_config->get_value("length_ratio_thresh", &length_ratio_thresh_)) {
    HLOG_ERROR << "get length_ratio_thresh failed...";
    return false;
  }

  if (!model_config->get_value("far_distance_thresh", &far_distance_thresh_)) {
    HLOG_ERROR << "get far_distance_thresh failed...";
    return false;
  }

  if (!model_config->get_value("interval_diff_ratio_thresh",
                               &interval_diff_ratio_thresh_)) {
    HLOG_ERROR << "get interval_diff_ratio_thresh failed...";
    return false;
  }

  if (!model_config->get_value("min_interval_thresh", &min_interval_thresh_)) {
    HLOG_ERROR << "get min_interval_thresh failed...";
    return false;
  }

  if (!model_config->get_value("anomaly_inter_ratio_thresh",
                               &anomaly_inter_ratio_thresh_)) {
    HLOG_ERROR << "get anomaly_inter_ratio_thresh failed...";
    return false;
  }
  return true;
}  // namespace LaneMeasurementFilter::Init()

bool LaneMeasurementFilter::Process(
    const std::vector<LaneLinePtr>& input_measurements,
    std::vector<LaneLinePtr>* output_measurements) {
  // PERF_FUNCTION();
  // PERF_BLOCK_START();
  // // 1.1 检测NMS过滤
  // std::stringstream time_ss;
  // time_ss << std::fixed << std::setprecision(10) << options.timestamp;
  // 0. 分流合流线都不过滤。
  // 1. 重叠比例非常大（包含关系时）, 同时长度比例大的时候, 删除短线。
  // 2. 重叠比例非常大（包含关系时）, 同时长度比例较小的时候，删除置信度低的线。
  // 3. 重叠比例一般时， 删除远处的线。

  // 4. 车子身后的线， 删除。

  output_measurements->clear();

  auto fork_merge_lanelines = GetForkOrMergeLaneLine(input_measurements);
  auto key_lanelines = GetUnForkOrMergeLaneLine(input_measurements);

  HLOG_DEBUG << "[LaneMeasurementFilter], origin measurement lanes size:"
             << key_lanelines.size();
  HLOG_DEBUG
      << "[LaneMeasurementFilter], before DelHighlyOverlayShortLaneLine size:"
      << key_lanelines.size();
  DelHighlyOverlayShortLaneLine(&key_lanelines);
  HLOG_DEBUG
      << "[LaneMeasurementFilter], before DelHighlyOverlayLowConfLaneLine size:"
      << key_lanelines.size();
  DelHighlyOverlayLowConfLaneLine(&key_lanelines);
  HLOG_DEBUG
      << "[LaneMeasurementFilter], before DelMiddleOverlayFarLaneline size:"
      << key_lanelines.size();
  DelMiddleOverlayFarLaneline(&key_lanelines);
  HLOG_DEBUG
      << "[LaneMeasurementFilter], before DeleteBehindVehicleLaneLine size:"
      << key_lanelines.size();
  DeleteBehindVehicleLaneLine(&key_lanelines);
  HLOG_DEBUG << "[LaneMeasurementFilter], before DelFarShortLaneLine size:"
             << key_lanelines.size();
  DelFarShortLaneLine(&key_lanelines);
  HLOG_DEBUG
      << "[LaneMeasurementFilter], before DelAnomalyIntervalLaneLine size:"
      << key_lanelines.size();
  DelAnomalyIntervalLaneLine(&key_lanelines);
  HLOG_DEBUG << "[LaneMeasurementFilter], output measurement lanes size:"
             << key_lanelines.size();
  for (auto& line : fork_merge_lanelines) {
    output_measurements->push_back(line);
  }

  for (auto& line : key_lanelines) {
    output_measurements->push_back(line);
  }

  // PERF_BLOCK_END("lane_filter");
  return true;
}

bool FindFarGroup(std::vector<LaneTrackerPtr>* lane_trackers,
                  std::pair<double, std::vector<LaneLinePtr>>* vec_lines) {
  std::vector<std::pair<double, std::vector<LaneLinePtr>>> group_pts;
  for (const auto& tracker : *lane_trackers) {
    if (!tracker->GetConstTarget()->IsTracked()) {
      continue;
    }
    const auto& line = tracker->GetConstTarget()->GetConstTrackedObject();
    // 能否找到分组标志
    bool find_flag = false;
    double start_pt_x = line->vehicle_points.front().x();
    if (start_pt_x > 5.0 && start_pt_x < 80.0) {
      for (auto& range_elem : group_pts) {
        // 端点范围4m内认为是一组
        if (std::abs(start_pt_x - range_elem.first) < 4) {
          range_elem.second.push_back(line);
          // 近似平均
          range_elem.first = (range_elem.first + start_pt_x) / 2.0;
          find_flag = true;
        }
      }
      // 没找到分组则插入
      if (!find_flag) {
        std::vector<LaneLinePtr> vec{line};
        group_pts.emplace_back(start_pt_x, vec);
      }
    }
  }
  // 找到远端目标group
  bool group_flag = false;
  int count_group = 0;
  for (const auto& range_elem : group_pts) {
    if (range_elem.second.size() >= 2) {
      // 检查任意两条线间的宽度
      count_group++;
      bool find_group_flag = true;
      for (size_t i = 0; i < range_elem.second.size(); ++i) {
        const auto& left_line = range_elem.second[i];
        for (size_t j = i + 1; j < range_elem.second.size(); ++j) {
          const auto& right_line = range_elem.second[j];
          float length = GetDistBetweenTwoLane(left_line->vehicle_points,
                                               right_line->vehicle_points);
          // 任意两条间距小于1.5米则不成立
          if (length < 1.5) {
            find_group_flag = false;
          }
        }
      }
      // 找到一个group
      if (find_group_flag) {
        group_flag = true;
        *vec_lines = range_elem;
      }
    }
  }
  if (count_group > 1) {
    group_flag = false;
  }
  return group_flag;
}

bool LaneMeasurementFilter::SetLostTrackerTruncation(
    std::vector<LaneLinePtr>* measurements,
    std::vector<LaneTrackerPtr>* lane_trackers) {
  // 结合measurements的确定点删除策略(目前改成用已有trackers，检测有抖动)
  // 有些场景由于模型脑补问题导致local_map的线长度一直错误
  // 当这些线后续没有匹配到时由于其一直存在地图中，长度是错误的
  // 少变多的场景case较多
  // 策略：根据检测线确定远方车道线的起点,tracker没有匹配上时根据起点做删除
  // | | | | |
  // | | | | |
  // |       |
  // |       |
  //  |  |  |
  //  |  |  |
  // 先对匹配上tracker清空
  const int lost_age_threshold = 5;
  for (auto& tracker : *lane_trackers) {
    if (tracker->GetConstTarget()->GetLostAge() <= lost_age_threshold) {
      tracker->GetTarget()->SetTruncation(std::numeric_limits<double>::max());
    } else {
      double last_truncation = tracker->GetTarget()->GetTruncation();
      // 映射到当前帧迭代更新
      if (last_truncation < 1.0) {
        const Eigen::Affine3d T_cur_last_ = POSE_MANAGER->GetDeltaPose();
        Eigen::Vector3d truncation_pt(last_truncation, 0.0, 0.0);
        truncation_pt = T_cur_last_ * truncation_pt;
        tracker->GetTarget()->SetTruncation(truncation_pt.x());
      }
    }
  }
  std::pair<double, std::vector<LaneLinePtr>> vec_lines;
  if (!FindFarGroup(lane_trackers, &vec_lines)) {
    return false;
  }
  for (auto& tracker : *lane_trackers) {
    // 匹配到了不做删除点
    if (tracker->GetConstTarget()->GetLostAge() <= lost_age_threshold) {
      continue;
    }
    // 已经确定删除了不再更新截断阈值
    if (tracker->GetConstTarget()->GetTruncation() <= 1.0) {
      continue;
    }
    auto pts =
        tracker->GetConstTarget()->GetConstTrackedObject()->vehicle_points;
    pts.erase(std::remove_if(
                  pts.begin(), pts.end(),
                  [&](const auto& pt) { return pt.x() < vec_lines.first; }),
              pts.end());
    // 没有重叠不做删除点
    if (pts.empty()) {
      continue;
    }
    for (const auto& line : vec_lines.second) {
      if (tracker->GetConstTarget()->GetConstTrackedObject()->id == line->id) {
        continue;
      }
      double dist = GetDistBetweenTwoLane(pts, line->vehicle_points);
      auto truncation = tracker->GetTarget()->GetTruncation();
      // 需要删除
      HLOG_DEBUG << tracker->GetTarget()->ToStr() << ", detect id: " << line->id
                 << ", " << dist << ", truncation: " << truncation;
      if (dist < 2.0) {
        tracker->GetTarget()->SetTruncation(0.0);
        break;
      }
    }
  }
  return true;
}

std::vector<LaneLinePtr> LaneMeasurementFilter::GetUnForkOrMergeLaneLine(
    const std::vector<LaneLinePtr>& measure_lanelines) {
  std::vector<LaneLinePtr> output_lanelines;
  output_lanelines.clear();

  for (auto& line : measure_lanelines) {
    if ((line->scene_type != LaneLineSceneType::FORK) &&
        (line->scene_type != LaneLineSceneType::CONVERGE)) {
      output_lanelines.push_back(line);
    }
  }

  return output_lanelines;
}

std::vector<LaneLinePtr> LaneMeasurementFilter::GetForkOrMergeLaneLine(
    const std::vector<LaneLinePtr>& measure_lanelines) {
  std::vector<LaneLinePtr> output_lanelines;
  output_lanelines.clear();

  for (auto& line : measure_lanelines) {
    if ((line->scene_type == LaneLineSceneType::FORK) ||
        (line->scene_type == LaneLineSceneType::CONVERGE)) {
      output_lanelines.push_back(line);
    }
  }

  return output_lanelines;
}

bool LaneMeasurementFilter::DeleteBehindVehicleLaneLine(
    std::vector<LaneLinePtr>* measure_lanelines) {
  for (auto iter = measure_lanelines->begin();
       iter != measure_lanelines->end();) {
    if ((*iter)->vehicle_points.rbegin()->x() < 2.0) {
      measure_lanelines->erase(iter);
    } else {
      ++iter;
    }
  }

  return true;
}

bool LaneMeasurementFilter::DelFarShortLaneLine(
    std::vector<LaneLinePtr>* measure_lanelines) {
  for (auto iter = measure_lanelines->begin();
       iter != measure_lanelines->end();) {
    auto length = GetLength((*iter)->vehicle_points);
    bool is_short = length < short_lane_length_thresh_;

    bool is_far = (*iter)->vehicle_points.begin()->x() > far_distance_thresh_;
    if (is_short && is_far) {
      // 是否需要通过heading再约束一下？ TODO(张文海)
      measure_lanelines->erase(iter);
    } else {
      ++iter;
    }
  }

  return true;
}

bool CompareLaneLineFunc(const LaneLinePtr& left, const LaneLinePtr& right) {
  float avg_left = 0.0;
  float avg_right = 0.0;
  const auto& left_pts = left->vehicle_points;
  for (int i = 0; i < left_pts.size(); ++i) {
    avg_left += left_pts[i].y();
  }
  avg_left = avg_left / left_pts.size();
  const auto& right_pts = right->vehicle_points;
  for (int i = 0; i < right_pts.size(); ++i) {
    avg_right += right_pts[i].y();
  }
  avg_right = avg_right / right_pts.size();
  return avg_left < avg_right;
}

bool LaneMeasurementFilter::IsSamePosLaneLine(
    const LaneLineConstPtr& ori_lanelane,
    const LaneLineConstPtr& compare_laneline) {
  float avg_dist = GetDistBetweenTwoLane(ori_lanelane->vehicle_points,
                                         compare_laneline->vehicle_points);
  return avg_dist < same_pos_distance_thresh_;
}

bool LaneMeasurementFilter::DelHighlyOverlayShortLaneLine(
    std::vector<LaneLinePtr>* measure_lanelines) {
  if (measure_lanelines->size() == 0) {
    return true;
  }

  std::vector<int> remove_index;
  HLOG_DEBUG << "measure_lanelines_nums:" << measure_lanelines->size();
  for (int i = 0; i < measure_lanelines->size() - 1; ++i) {
    HLOG_DEBUG << "measure_lanelines_idxs:" << i;
    auto left_lane = measure_lanelines->at(i);
    float left_lane_length = GetLength(left_lane->vehicle_points);

    for (int j = i + 1; j < measure_lanelines->size(); ++j) {
      auto right_lane = measure_lanelines->at(j);
      if (!IsSamePosLaneLine(left_lane, right_lane)) {
        continue;
      }
      float right_lane_length = GetLength(right_lane->vehicle_points);
      float overlay_ratio =
          GetOverLayRatioBetweenTwoLane(left_lane, right_lane);
      float length_ratio = GetLengthRatioBetweenTwoLane(left_lane, right_lane);
      if ((overlay_ratio > hight_overlay_length_ratio_thresh_) &&
          (length_ratio < length_ratio_thresh_)) {
        if (left_lane_length < right_lane_length) {
          remove_index.push_back(i);
        } else {
          remove_index.push_back(j);
        }
      }
    }
  }

  // 排序，去重
  sort(remove_index.begin(), remove_index.end());
  remove_index.erase(std::unique(remove_index.begin(), remove_index.end()),
                     remove_index.end());
  for (int idx = remove_index.size() - 1; idx >= 0; --idx) {
    measure_lanelines->erase(measure_lanelines->begin() + remove_index[idx]);
  }
  return true;
}
bool LaneMeasurementFilter::DelHighlyOverlayLowConfLaneLine(
    std::vector<LaneLinePtr>* measure_lanelines) {
  std::vector<int> remove_index;
  if (measure_lanelines->size() == 0) {
    return true;
  }

  for (int i = 0; i < measure_lanelines->size() - 1; ++i) {
    auto left_lane = measure_lanelines->at(i);
    float left_lane_length = GetLength(left_lane->vehicle_points);

    for (int j = i + 1; j < measure_lanelines->size(); ++j) {
      auto right_lane = measure_lanelines->at(j);
      if (!IsSamePosLaneLine(left_lane, right_lane)) {
        continue;
      }
      float right_lane_length = GetLength(right_lane->vehicle_points);
      float overlay_ratio =
          GetOverLayRatioBetweenTwoLane(left_lane, right_lane);
      float length_ratio = GetLengthRatioBetweenTwoLane(left_lane, right_lane);
      if ((overlay_ratio > hight_overlay_length_ratio_thresh_) &&
          (length_ratio >= length_ratio_thresh_)) {
        if (left_lane->score < right_lane->score) {
          remove_index.push_back(i);
        } else {
          remove_index.push_back(j);
        }
      }
    }
  }

  // 排序，去重
  sort(remove_index.begin(), remove_index.end());
  remove_index.erase(std::unique(remove_index.begin(), remove_index.end()),
                     remove_index.end());
  for (int idx = remove_index.size() - 1; idx >= 0; --idx) {
    measure_lanelines->erase(measure_lanelines->begin() + remove_index[idx]);
  }
  return true;
}

bool LaneMeasurementFilter::DelMiddleOverlayFarLaneline(
    std::vector<LaneLinePtr>* measure_lanelines) {
  if (measure_lanelines->size() == 0) {
    return true;
  }

  std::vector<int> remove_index;
  for (int i = 0; i < measure_lanelines->size() - 1; ++i) {
    auto left_lane = measure_lanelines->at(i);
    float left_lane_length = GetLength(left_lane->vehicle_points);

    for (int j = i + 1; j < measure_lanelines->size(); ++j) {
      auto right_lane = measure_lanelines->at(j);
      if (!IsSamePosLaneLine(left_lane, right_lane)) {
        continue;
      }
      float right_lane_length = GetLength(right_lane->vehicle_points);
      float overlay_ratio =
          GetOverLayRatioBetweenTwoLane(left_lane, right_lane);
      float length_ratio = GetLengthRatioBetweenTwoLane(left_lane, right_lane);

      if (overlay_ratio <= low_overlay_length_ratio_thresh_) {
        continue;
      }
      if (length_ratio >= length_ratio_thresh_) {
        // 长度比较接近的话， 删除远的
        if (left_lane->vehicle_points.front().x() >
            right_lane->vehicle_points.front().x()) {
          remove_index.push_back(i);
        } else {
          remove_index.push_back(j);
        }
      } else {
        // 长度相差大的话， 删除短的
        if (left_lane_length < right_lane_length) {
          remove_index.push_back(i);
        } else {
          remove_index.push_back(j);
        }
      }
    }
  }

  // 排序，去重
  sort(remove_index.begin(), remove_index.end());
  remove_index.erase(std::unique(remove_index.begin(), remove_index.end()),
                     remove_index.end());
  for (int idx = remove_index.size() - 1; idx >= 0; --idx) {
    measure_lanelines->erase(measure_lanelines->begin() + remove_index[idx]);
  }
  return true;
}

// bool LaneMeasurementFilter::AssginForkOrConvergeType(
//     std::vector<LaneLinePtr>* out_measurements,
//     std::vector<LaneTrackerPtr>* lane_trackers) {

//     }

bool LaneMeasurementFilter::DelAnomalyIntervalLaneLine(
    std::vector<LaneLinePtr>* measure_lanelines) {
  HLOG_DEBUG << "DelAnomalyIntervalLaneLine start";
  if (measure_lanelines->size() == 0) {
    return true;
  }
  std::vector<int> remove_index;
  for (int i = 0; i < measure_lanelines->size(); ++i) {
    auto laneline = measure_lanelines->at(i);
    if (laneline->vehicle_points.empty()) {
      continue;
    }
    if (laneline->score > 0.5) {
      continue;
    }
    // 计算点的平均距离
    float laneline_length = GetLength(laneline->vehicle_points);
    float avg_dist = laneline_length / laneline->vehicle_points.size();
    float variance = 0.f;
    HLOG_DEBUG << "index: " << i << " ,laneline_length: " << laneline_length;

    // 计算两点距离
    int anomaly_inter_flag = 0;
    for (int j = 1; j < laneline->vehicle_points.size() - 1; ++j) {
      float dist = sqrt((laneline->vehicle_points[j - 1].x() -
                         laneline->vehicle_points[j].x()) *
                            (laneline->vehicle_points[j - 1].x() -
                             laneline->vehicle_points[j].x()) +
                        (laneline->vehicle_points[j - 1].y() -
                         laneline->vehicle_points[j].y()) *
                            (laneline->vehicle_points[j - 1].y() -
                             laneline->vehicle_points[j].y()));
      variance += pow(dist - avg_dist, 2);
      if (dist / avg_dist > interval_diff_ratio_thresh_ ||
          avg_dist / dist > interval_diff_ratio_thresh_) {
        anomaly_inter_flag++;
      }
    }
    variance = variance / laneline->vehicle_points.size();
    HLOG_DEBUG << " ,avg_dist:" << avg_dist << " ,dist variance :" << variance
               << " ,anomaly_inter_flag: " << anomaly_inter_flag;
    if (anomaly_inter_flag / (laneline->vehicle_points.size() + 0.000001) >
            anomaly_inter_ratio_thresh_ ||
        avg_dist < min_interval_thresh_) {
      remove_index.push_back(i);
    }
  }

  // 排序，去重
  sort(remove_index.begin(), remove_index.end());
  remove_index.erase(std::unique(remove_index.begin(), remove_index.end()),
                     remove_index.end());
  for (int idx = remove_index.size() - 1; idx >= 0; --idx) {
    measure_lanelines->erase(measure_lanelines->begin() + remove_index[idx]);
  }
  return true;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
