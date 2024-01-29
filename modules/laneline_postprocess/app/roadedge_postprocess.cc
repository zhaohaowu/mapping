/*================================================================
*   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
*   file       ：lane_post_process.h
*   author     ：Chenanmeng
*   date       ：2023.02.28
================================================================*/
#include "modules/laneline_postprocess/app/roadedge_postprocess.h"
#include <float.h>
#include <sys/time.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/datalogger/load_data_singleton.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/pipeline/roadedge_point_tracker_pipeline.h"
#include "perception-base/base/scene/roadedges.h"
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

bool RoadEdgePostProcess::Init(const ProcessInitOption& init_option) {
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
    HLOG_ERROR << "Roadedge Post Process Get Proto File Failed !";
    return false;
  }

  // 车道线跟踪器模块初始化
  roadedge_tracker_ = std::make_unique<RoadEdgePointFilterTrackerPipeline>();
  CHECK(roadedge_tracker_ != nullptr);
  ProcessInitOption roadedge_tracker_init_options;
  CHECK(roadedge_tracker_->Init(roadedge_tracker_init_options));
  HLOG_DEBUG << "init lane tracker name: " << roadedge_tracker_->Name();

  last_track_roadedges_.clear();

  return true;
}

bool RoadEdgePostProcess::Process(
    perception_base::MeasurementFramePtr measurement_ptr,
    perception_base::FusionFramePtr fusion_ptr) {
  HLOG_DEBUG << "Start RoadEdgePostProcess Working...";
  if (!fusion_ptr->scene_) {
    fusion_ptr->scene_ = std::make_shared<perception_base::Scene>();
  }
  if (!fusion_ptr->scene_->road_edges) {
    fusion_ptr->scene_->road_edges =
        std::make_shared<perception_base::RoadEdges>();
  }

  InputDataSingleton* local_data = InputDataSingleton::Instance();
  if (local_data->IsStaticState(config_.static_strategy_param()) &&
      !last_track_roadedges_.empty()) {
    fusion_ptr->scene_->road_edges->road_edges = last_track_roadedges_;
    HLOG_DEBUG << "Use Static strategy !!!";
    return true;
  }

  ProcessOption options;
  options.timestamp = measurement_ptr->header.timestamp;
  Process(options, measurement_ptr->roadedges_measurement_,
          fusion_ptr->scene_->road_edges);

  last_track_roadedges_ = fusion_ptr->scene_->road_edges->road_edges;

  HLOG_DEBUG << "End RoadEdgePostProcess Working...";
  return true;
}

bool RoadEdgePostProcess::Process(
    const ProcessOption& options,
    perception_base::RoadEdgesMeasurementConstPtr detect_measurements,
    const perception_base::RoadEdgesPtr track_outputs) {
  // PERF_FUNCTION();
  // PERF_BLOCK_START();

  environment::ProcessOption track_options;
  if (!roadedge_tracker_->Track(track_options, detect_measurements,
                                track_outputs)) {
    HLOG_ERROR << "Do laneline tracking failed...";
    return false;
  }
  // PERF_BLOCK_END("PointLaneTracker Used Time");

  return true;
}

void RoadEdgePostProcess::Reset() {}
}  // namespace environment
}  // namespace mp
}  // namespace hozon
