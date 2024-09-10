/*================================================================
*   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
*   file       ：lane_post_process.h
*   author     ：Chenanmeng
*   date       ：2023.02.28
================================================================*/
#include "modules/laneline_postprocess/app/laneline_postprocess.h"
#include <float.h>
#include <sys/time.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "modules/laneline_postprocess/lib/laneline/proto/lane_postprocess.pb.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/datalogger/load_data_singleton.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/pipeline/lane_measurements_filter.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/pipeline/lane_point_tracker_pipeline.h"
#include "perception-base/base/scene/laneline.h"
#include "perception-base/base/utils/log.h"
// #include "perception-common/common/performance/perf_util.h"
#include "perception-lib/lib/config_manager/config_manager.h"
#include "perception-lib/lib/io/file_util.h"
#include "perception-lib/lib/io/protobuf_util.h"
#include "perception-lib/lib/location_manager/location_manager.h"
#include "perception-lib/lib/singleton/singleton.h"

namespace hozon {
namespace mp {
namespace environment {

bool LanePostProcess::Init(const ProcessInitOption& init_option) {
  // std::cout << "LanePostProcess Init==========" << std::endl;
  auto config_manager = perception_lib::ConfigManager::Instance();
  const perception_lib::ModelConfig* model_config = nullptr;
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

  config_file =
      perception_lib::FileUtil::GetAbsolutePath(work_root, config_file);
  if (!perception_lib::GetProtoFromFile(config_file, &config_)) {
    HLOG_ERROR << "Lane Post Process Get Proto File Failed !";
    return false;
  }

  // std::cout << "***lane config file***:" << config_file << std::endl;

  // 车道线观测线过滤模块初始化
  lane_measurements_filter_.reset(
      BaseLaneMeasurementFilterRegisterer::GetInstanceByName(
          "LanePostMeasurementFilter"));
  CHECK(lane_measurements_filter_ != nullptr);
  AnomalyFilterInitOptions lane_anomaly_filter_init_options;
  CHECK(lane_measurements_filter_->Init(lane_anomaly_filter_init_options));
  HLOG_DEBUG << "init lane detect filter name: "
             << lane_measurements_filter_->Name();

  // 车道线跟踪器模块初始化
  lane_tracker_ = std::make_unique<LanePointFilterTrackerPipeline>();
  CHECK(lane_tracker_ != nullptr);
  ProcessInitOption lane_tracker_init_options;
  CHECK(lane_tracker_->Init(lane_tracker_init_options));
  HLOG_DEBUG << "init lane tracker name: " << lane_tracker_->Name();

  last_track_lanelines_.clear();
  // std::cout << "LanePostProcess Init end==========" << std::endl;
  return true;
}

bool LanePostProcess::Process(
    perception_base::MeasurementFramePtr measurement_ptr,
    perception_base::FusionFramePtr fusion_ptr) {
  HLOG_DEBUG << "Start LanePostProcess Working...";
  if (!fusion_ptr->scene_) {
    fusion_ptr->scene_ = std::make_shared<perception_base::Scene>();
  }
  if (!fusion_ptr->scene_->lane_lines) {
    fusion_ptr->scene_->lane_lines =
        std::make_shared<perception_base::LaneLines>();
  }

  InputDataSingleton* local_data = InputDataSingleton::Instance();
  if (local_data->IsStaticState(config_.static_strategy_param()) &&
      !last_track_lanelines_.empty()) {
    TransTrackerLocal2Vehicle(&last_track_lanelines_);
    // 暂不对三次方程系数做更新（方程系数只影响渲染）
    fusion_ptr->scene_->lane_lines->lanelines = last_track_lanelines_;

    HLOG_DEBUG << "Use Static strategy !!!";
    return true;
  }
  local_data->IsTurnState(config_.static_strategy_param());

  ProcessOption options;
  options.timestamp = measurement_ptr->header.timestamp;
  // 记录结果更新时的pose
  auto& dr_datas = InputDataSingleton::Instance()->dr_data_buffer_;
  local_data->SetMapUpdatePose(dr_datas.back()->pose);
  Process(options, measurement_ptr->lanelines_measurement_,
          fusion_ptr->scene_->lane_lines);
  last_track_lanelines_ = fusion_ptr->scene_->lane_lines->lanelines;

  HLOG_DEBUG << "End LanePostProcess Working...";
  return true;
}

void LanePostProcess::TransTrackerLocal2Vehicle(
    std::vector<perception_base::LaneLinePtr>* tracked_outputs) {
  auto& dr_datas = InputDataSingleton::Instance()->dr_data_buffer_;
  auto novatel2world_pose_ = dr_datas.back()->pose;
  for (auto& tracked_output : *tracked_outputs) {
    auto& points = tracked_output->point_set;
    Eigen::Vector3d vehicle_pt(0.0, 0.0, 0.0);
    Eigen::Vector3d local_pt(0.0, 0.0, 0.0);

    for (auto& lane_line_point : points) {
      auto& local_point = lane_line_point.local_point;
      local_pt[0] = local_point.x;
      local_pt[1] = local_point.y;

      vehicle_pt = novatel2world_pose_.inverse() * local_pt;
      auto& vehicle_point = lane_line_point.vehicle_point;
      vehicle_point.x = vehicle_pt[0];
      vehicle_point.y = vehicle_pt[1];
    }
  }
}

bool LanePostProcess::Process(
    const ProcessOption& options,
    perception_base::LaneLinesMeasurementConstPtr detect_measurements,
    const perception_base::LaneLinesPtr track_lanelines) {
  // PERF_FUNCTION();
  // PERF_BLOCK_START();

  environment::ProcessOption track_options;
  track_options.timestamp = options.timestamp;
  environment::AnomalyFilterOptions lane_anomaly_filter_options;
  lane_anomaly_filter_options.timestamp = options.timestamp;

  perception_base::LaneLinesMeasurementPtr filtered_measurement =
      std::make_shared<perception_base::LaneLinesMeasurement>();
  if (!lane_measurements_filter_->Filter(lane_anomaly_filter_options,
                                         detect_measurements,
                                         filtered_measurement)) {
    HLOG_ERROR << "Do laneline anomaly filter failed...";
    return false;
  }

  HLOG_DEBUG << "before filter amony measurement laneline:"
             << detect_measurements->lanelines.size();
  HLOG_DEBUG << "after filter amony measurement laneline:"
             << filtered_measurement->lanelines.size();
  if (!lane_tracker_->Track(track_options, filtered_measurement,
                            track_lanelines)) {
    HLOG_ERROR << "Do laneline tracking failed...";
    return false;
  }
  // PERF_BLOCK_END("PointLaneTracker Used Time");

  return true;
}

void LanePostProcess::Reset() {}
}  // namespace environment
}  // namespace mp
}  // namespace hozon
