// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: hozon
// @file: simple_lane_tracker_pipeline.cc
// @brief: associate history lane_track to current detected lane　object
#include "modules/local_mapping/lib/filter/laneline_measure_filter.h"
#include <math.h>

#include <unordered_set>

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

bool LaneMeasurementFilter::DelNearTrackers(
    std::vector<LaneLinePtr>* out_measurements,
    const std::vector<MatchScoreTuple>& match_score_list,
    std::vector<LaneTrackerPtr>* lane_trackers) {
  // 结合tracker的过滤策略

  if (match_score_list.size() < 2) {
    return true;
  }
  std::unordered_set<int> remove_indexs;
  size_t track_id2 = 0;
  size_t detect_id2 = 0;
  size_t track_id1 = 0;
  size_t detect_id1 = 0;
  float score_1 = 0.0;
  float score_2 = 0.0;

  std::unordered_set<int> track_id_set;
  for (auto& lane_tracker : *lane_trackers) {
    track_id_set.insert(lane_tracker->GetConstTarget()->Id());
  }

  for (int i = 0; i < match_score_list.size() - 1; ++i) {
    track_id1 = std::get<0>(match_score_list[i]);
    if (track_id_set.count(track_id1) == 0) {
      continue;
    }

    detect_id1 = std::get<1>(match_score_list[i]);
    score_1 = std::get<2>(match_score_list[i]);
    for (int j = i + 1; j < match_score_list.size(); ++j) {
      track_id2 = std::get<0>(match_score_list[j]);
      if (track_id_set.count(track_id2) == 0) {
        continue;
      }
      detect_id2 = std::get<1>(match_score_list[j]);
      score_2 = std::get<2>(match_score_list[j]);
      if (detect_id1 == detect_id2) {
        if (score_1 <= score_2) {
          remove_indexs.insert(track_id1);
        } else {
          remove_indexs.insert(track_id2);
        }
      }
    }
  }

  lane_trackers->erase(
      std::remove_if(lane_trackers->begin(), lane_trackers->end(),
                     [&](LaneTrackerPtr& track) {
                       return remove_indexs.count(
                                  track->GetConstTarget()->Id()) > 0;
                     }),
      lane_trackers->end());
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
