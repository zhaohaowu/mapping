// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: simple_lane_tracker_pipeline.cc
// @brief: associate history lane_track to current detected lane　object
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/pipeline/lane_measurements_filter.h"

#include <math.h>

#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/datalogger/load_data_singleton.h"
#include "modules/laneline_postprocess/lib/laneline/utils/lane_utils.h"
#include "perception-base/base/laneline/base_laneline.h"
// #include "perception-common/common/performance/perf_util.h"
#include "perception-lib/lib/config_manager/config_manager.h"
#include "perception-lib/lib/io/file_util.h"
#include "perception-lib/lib/io/protobuf_util.h"
#include "perception-lib/lib/location_manager/location_manager.h"

namespace hozon {
namespace mp {
namespace environment {
using base::LaneLine;
using base::LaneLinePoint;
using base::LaneLinePosition;
using base::LaneLinePtr;
using base::Location;
using lib::FileUtil;
using lib::LocationManager;
using lib::ParseProtobufFromFile;

bool LaneMeasurementFilter::Init(const AnomalyFilterInitOptions &options) {
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig *model_config = nullptr;
  if (!config_manager->GetModelConfig(Name(), &model_config)) {
    HLOG_ERROR << "Parse config failed! Name: " << Name();
    return false;
  }

  const std::string work_root = config_manager->work_root();
  std::string config_file;

  if (!model_config->get_value("config_file", &config_file)) {
    HLOG_ERROR << "Get root path failed!";
    return false;
  }

  LanePostProcessParam postprocess_param;
  config_file = lib::FileUtil::GetAbsolutePath(work_root, config_file);
  CHECK(ParseProtobufFromFile<LanePostProcessParam>(config_file,
                                                    &postprocess_param))
      << "Read config failed: " << config_file;

  lane_meas_filter_param_ = postprocess_param.lane_measurement_filter_param();
  HLOG_DEBUG << "load lane_meas_filter parameters from " << config_file
             << " \nParams: " << lane_meas_filter_param_.DebugString();

  return true;
}

bool CompareLaneLineFunc(const base::LaneLineMeasurementPtr &left,
                         const base::LaneLineMeasurementPtr &right) {
  float avg_left = 0.0, avg_right = 0.0;
  const auto &left_pt = left->point_set;
  for (int i = 0; i < left_pt.size(); ++i) {
    avg_left += left_pt[i].vehicle_point.y;
  }
  avg_left = avg_left / left_pt.size();
  const auto &right_pt = right->point_set;
  for (int i = 0; i < right_pt.size(); ++i) {
    avg_right += right_pt[i].vehicle_point.y;
  }
  avg_right = avg_right / right_pt.size();
  return avg_left < avg_right;
}

bool LaneMeasurementFilter::IsSamePosLaneLine(
    const base::LaneLineMeasurementConstPtr &ori_lanelane,
    const base::LaneLineMeasurementConstPtr &compare_laneline) {
  float avg_dist = GetDistBetweenTwoLane(ori_lanelane->point_set,
                                         compare_laneline->point_set);
  return avg_dist < lane_meas_filter_param_.same_pos_distance_thresh();
}

bool LaneMeasurementFilter::DelHighlyOverlayShortLaneLine(
    std::vector<base::LaneLineMeasurementPtr> *measure_lanelines) {
  if (measure_lanelines->size() == 0) {
    return true;
  }

  std::vector<int> remove_index;
  HLOG_INFO << "measure_lanelines_nums:" << measure_lanelines->size();
  for (int i = 0; i < measure_lanelines->size() - 1; ++i) {
    HLOG_INFO << "measure_lanelines_idxs:" << i;
    auto left_lane = measure_lanelines->at(i);
    float left_lane_length = GetLength(left_lane->point_set);

    for (int j = i + 1; j < measure_lanelines->size(); ++j) {
      auto right_lane = measure_lanelines->at(j);
      if (!IsSamePosLaneLine(left_lane, right_lane)) {
        continue;
      }
      float right_lane_length = GetLength(right_lane->point_set);
      float overlay_ratio =
          GetOverLayRatioBetweenTwoLane(left_lane, right_lane);
      float length_ratio = GetLengthRatioBetweenTwoLane(left_lane, right_lane);
      if ((overlay_ratio >
           lane_meas_filter_param_.hight_overlay_length_ratio_thresh()) &&
          (length_ratio < lane_meas_filter_param_.length_ratio_thresh())) {
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
    std::vector<base::LaneLineMeasurementPtr> *measure_lanelines) {
  std::vector<int> remove_index;
  if (measure_lanelines->size() == 0) {
    return true;
  }

  for (int i = 0; i < measure_lanelines->size() - 1; ++i) {
    auto left_lane = measure_lanelines->at(i);
    float left_lane_length = GetLength(left_lane->point_set);

    for (int j = i + 1; j < measure_lanelines->size(); ++j) {
      auto right_lane = measure_lanelines->at(j);
      if (!IsSamePosLaneLine(left_lane, right_lane)) {
        continue;
      }
      float right_lane_length = GetLength(right_lane->point_set);
      float overlay_ratio =
          GetOverLayRatioBetweenTwoLane(left_lane, right_lane);
      float length_ratio = GetLengthRatioBetweenTwoLane(left_lane, right_lane);
      if ((overlay_ratio >
           lane_meas_filter_param_.hight_overlay_length_ratio_thresh()) &&
          (length_ratio >= lane_meas_filter_param_.length_ratio_thresh())) {
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
    std::vector<base::LaneLineMeasurementPtr> *measure_lanelines) {
  if (measure_lanelines->size() == 0) {
    return true;
  }

  std::vector<int> remove_index;
  for (int i = 0; i < measure_lanelines->size() - 1; ++i) {
    auto left_lane = measure_lanelines->at(i);
    float left_lane_length = GetLength(left_lane->point_set);

    for (int j = i + 1; j < measure_lanelines->size(); ++j) {
      auto right_lane = measure_lanelines->at(j);
      if (!IsSamePosLaneLine(left_lane, right_lane)) {
        continue;
      }
      float right_lane_length = GetLength(right_lane->point_set);
      float overlay_ratio =
          GetOverLayRatioBetweenTwoLane(left_lane, right_lane);
      float length_ratio = GetLengthRatioBetweenTwoLane(left_lane, right_lane);

      if (overlay_ratio <=
          lane_meas_filter_param_.low_overlay_length_ratio_thresh()) {
        continue;
      }
      if (length_ratio >= lane_meas_filter_param_.length_ratio_thresh()) {
        // 长度比较接近的话， 删除远的
        if (left_lane->point_set.begin()->vehicle_point.x >
            right_lane->point_set.begin()->vehicle_point.x) {
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
bool LaneMeasurementFilter::DelAnomalyIntervalLaneLine(
    std::vector<base::LaneLineMeasurementPtr> *measure_lanelines) {
  HLOG_DEBUG << "DelAnomalyIntervalLaneLine start";
  if (measure_lanelines->size() == 0) {
    return true;
  }
  std::vector<int> remove_index;
  for (int i = 0; i < measure_lanelines->size(); ++i) {
    auto laneline = measure_lanelines->at(i);
    if (!laneline->point_set.size() > 0) {
      continue;
    }
    if (laneline->score > 0.5) {
      continue;
    }
    // 计算点的平均距离
    float laneline_length = GetLength(laneline->point_set);
    float avg_dist = laneline_length / laneline->point_set.size();
    float variance = 0.f;
    HLOG_DEBUG << "index: " << i << " ,laneline_length: " << laneline_length;

    // 计算两点距离
    int anomaly_inter_flag = 0;
    for (int j = 1; j < laneline->point_set.size() - 1; ++j) {
      float dist = sqrt((laneline->point_set[j - 1].vehicle_point.x -
                         laneline->point_set[j].vehicle_point.x) *
                            (laneline->point_set[j - 1].vehicle_point.x -
                             laneline->point_set[j].vehicle_point.x) +
                        (laneline->point_set[j - 1].vehicle_point.y -
                         laneline->point_set[j].vehicle_point.y) *
                            (laneline->point_set[j - 1].vehicle_point.y -
                             laneline->point_set[j].vehicle_point.y));
      variance += pow(dist - avg_dist, 2);
      if (dist / avg_dist >
              lane_meas_filter_param_.interval_diff_ratio_thresh() ||
          avg_dist / dist >
              lane_meas_filter_param_.interval_diff_ratio_thresh()) {
        anomaly_inter_flag++;
      }
      HLOG_DEBUG << "dist: " << dist;
    }
    variance = variance / laneline->point_set.size();
    HLOG_DEBUG << " ,avg_dist:" << avg_dist << " ,dist variance :" << variance
               << " ,anomaly_inter_flag: " << anomaly_inter_flag;
    if (anomaly_inter_flag / (laneline->point_set.size() + 0.000001) >
            lane_meas_filter_param_.anomaly_inter_ratio_thresh() ||
        avg_dist < lane_meas_filter_param_.min_interval_thresh()) {
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
bool LaneMeasurementFilter::Filter(
    const AnomalyFilterOptions &options,
    const base::LaneLinesMeasurementConstPtr &input_measurements,
    const base::LaneLinesMeasurementPtr &output_measurements) {
  // PERF_FUNCTION();
  // PERF_BLOCK_START();

  std::vector<base::LaneLineMeasurementPtr> local_measurements =
      input_measurements->lanelines;
  // // 1.1 检测NMS过滤
  // std::stringstream time_ss;
  // time_ss << std::fixed << std::setprecision(10) << options.timestamp;
  // FilterMeasurements(detect_measurements, 1.0);

  // 0. 分流合流线都不过滤。
  // 1. 重叠比例非常大（包含关系时）, 同时长度比例大的时候, 删除短线。
  // 2. 重叠比例非常大（包含关系时）, 同时长度比例较小的时候，删除置信度低的线。
  // 3. 重叠比例一般时， 删除远处的线。

  // 4. 车子身后的线， 删除。

  output_measurements->lanelines.clear();

  auto fork_merge_lanelines = GetForkOrMergeLaneLine(local_measurements);
  auto key_lanelines = GetUnForkOrMergeLaneLine(local_measurements);

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
  for (auto &line : fork_merge_lanelines) {
    output_measurements->lanelines.push_back(line);
  }

  for (auto &line : key_lanelines) {
    output_measurements->lanelines.push_back(line);
  }

  // PERF_BLOCK_END("lane_filter");
  return true;
}

std::string LaneMeasurementFilter::Name() const {
  return "LaneMeasurementFilter";
}

std::vector<base::LaneLineMeasurementPtr>
LaneMeasurementFilter::GetUnForkOrMergeLaneLine(
    const std::vector<base::LaneLineMeasurementPtr> &measure_lanelines) {
  std::vector<base::LaneLineMeasurementPtr> output_lanelines;
  output_lanelines.clear();

  for (auto &line : measure_lanelines) {
    if ((line->split != base::LaneLineSceneType::FORK) &&
        (line->split != base::LaneLineSceneType::CONVERGE)) {
      output_lanelines.push_back(line);
    }
  }

  return output_lanelines;
}

std::vector<base::LaneLineMeasurementPtr>
LaneMeasurementFilter::GetForkOrMergeLaneLine(
    const std::vector<base::LaneLineMeasurementPtr> &measure_lanelines) {
  std::vector<base::LaneLineMeasurementPtr> output_lanelines;
  output_lanelines.clear();

  for (auto &line : measure_lanelines) {
    if ((line->split == base::LaneLineSceneType::FORK) ||
        (line->split == base::LaneLineSceneType::CONVERGE)) {
      output_lanelines.push_back(line);
    }
  }

  return output_lanelines;
}

bool LaneMeasurementFilter::DeleteBehindVehicleLaneLine(
    std::vector<base::LaneLineMeasurementPtr> *measure_lanelines) {
  for (auto iter = measure_lanelines->begin();
       iter != measure_lanelines->end();) {
    if ((*iter)->point_set.rbegin()->vehicle_point.x < 2.0) {
      measure_lanelines->erase(iter);
    } else {
      ++iter;
    }
  }

  return true;
}

bool LaneMeasurementFilter::DelFarShortLaneLine(
    std::vector<base::LaneLineMeasurementPtr> *measure_lanelines) {
  for (auto iter = measure_lanelines->begin();
       iter != measure_lanelines->end();) {
    auto length = GetLength((*iter)->point_set);
    bool is_short = length < lane_meas_filter_param_.short_lane_length_thresh();

    bool is_far = (*iter)->point_set.begin()->vehicle_point.x >
                  lane_meas_filter_param_.far_distance_thresh();
    if (is_short && is_far) {
      // 是否需要通过heading再约束一下？ TODO(张文海)
      measure_lanelines->erase(iter);
    } else {
      ++iter;
    }
  }

  return true;
}
// Register plugin.
// REGISTER_LANE_TRACKER(LaneMeasurementFilter);

}  // namespace environment
}  // namespace mp
}  // namespace hozon
