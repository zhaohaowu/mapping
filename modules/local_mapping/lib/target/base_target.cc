// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: chenlongxi
// @file: base_target.cc
// @brief: base lane element target head file

#include "modules/local_mapping/lib/target/base_target.h"

#include "perception-lib/lib/config_manager/config_manager.h"
namespace hozon {
namespace mp {
namespace lm {

namespace perception_lib = hozon::perception::lib;

bool LaneTarget::Init(const ProcessOption& options,
                      const LaneLinePtr& detected_lane_line) {
  auto* config_manager = perception_lib::ConfigManager::Instance();
  const perception_lib::ModelConfig* model_config = nullptr;
  if (!config_manager->GetModelConfig("LaneLineTargetParam", &model_config)) {
    HLOG_ERROR << "Parse config failed! Name: LaneLineTargetParam";
    return false;
  }
  if (!model_config->get_value("tracked_init_life", &tracked_init_life_)) {
    HLOG_ERROR << "Get laneline target tracked_init_life failed!";
    return false;
  }
  if (!model_config->get_value("reserve_age", &reserve_age_)) {
    HLOG_ERROR << "Get laneline target reserve_age failed!";
    return false;
  }

  // HLOG_DEBUG << "laneline reserve_age:" << reserve_age_;
  // HLOG_DEBUG << "laneline tracked_init_life:" << tracked_init_life_;

  InitBase(options, detected_lane_line);
  tracked_element_ = std::make_shared<LaneLine>();
  tracked_element_->vehicle_points = detected_lane_line->vehicle_points;
  tracked_element_->world_points = detected_lane_line->world_points;
  tracked_element_->id = id_;
  tracked_element_->tracked_count++;
  // 缓存N帧跟踪状态
  lastest_n_tracked_state_.set_capacity(10);
  history_line_pos_.set_capacity(2);
  history_mf_line_pos_.set_capacity(2);
  return true;
}

bool RoadEdgeTarget::Init(const ProcessOption& options,
                          const RoadEdgePtr& detected_road_edge) {
  auto* config_manager = perception_lib::ConfigManager::Instance();
  const perception_lib::ModelConfig* model_config = nullptr;
  if (!config_manager->GetModelConfig("RoadEdgeTargetParam", &model_config)) {
    HLOG_ERROR << "Parse config failed! Name: RoadEdgeTargetParam";
    return false;
  }
  if (!model_config->get_value("tracked_init_life", &tracked_init_life_)) {
    HLOG_ERROR << "Get roadedge target tracked_init_life failed!";
    return false;
  }
  if (!model_config->get_value("reserve_age", &reserve_age_)) {
    HLOG_ERROR << "Get roadedge target reserve_age failed!";
    return false;
  }
  InitBase(options, detected_road_edge);
  tracked_element_ = std::make_shared<RoadEdge>();
  tracked_element_->vehicle_points = detected_road_edge->vehicle_points;
  tracked_element_->world_points = detected_road_edge->world_points;
  tracked_element_->id = id_;
  tracked_element_->tracked_count++;
  return true;
}

bool OccEdgeTarget::Init(const ProcessOption& options,
                         const OccEdgePtr& detected_occ_edge) {
  auto* config_manager = perception_lib::ConfigManager::Instance();
  const perception_lib::ModelConfig* model_config = nullptr;
  if (!config_manager->GetModelConfig("OccEdgeTargetParam", &model_config)) {
    HLOG_ERROR << "Parse config failed! Name: OccEdgeTargetParam";
    return false;
  }
  if (!model_config->get_value("tracked_init_life", &tracked_init_life_)) {
    HLOG_ERROR << "Get occedge target tracked_init_life failed!";
    return false;
  }
  if (!model_config->get_value("reserve_age", &reserve_age_)) {
    HLOG_ERROR << "Get occedge target reserve_age failed!";
    return false;
  }
  InitBase(options, detected_occ_edge);
  tracked_element_ = std::make_shared<OccEdge>();
  tracked_element_->vehicle_points = detected_occ_edge->vehicle_points;
  tracked_element_->fit_points = detected_occ_edge->fit_points;
  tracked_element_->world_points = detected_occ_edge->world_points;
  tracked_element_->id = id_;
  tracked_element_->tracked_count++;
  tracked_element_->vehicle_curve = detected_occ_edge->vehicle_curve;
  occ_detect_id = detected_occ_edge->detect_id;
  return true;
}

bool StopLineTarget::Init(const ProcessOption& options,
                          const StopLinePtr& detected_stopline_ptr) {
  auto* config_manager = perception_lib::ConfigManager::Instance();
  const perception_lib::ModelConfig* model_config = nullptr;
  if (!config_manager->GetModelConfig("StopLineTargetParam", &model_config)) {
    HLOG_ERROR << "Parse config failed! Name: StopLineTargetParam";
    return false;
  }
  if (!model_config->get_value("tracked_init_life", &tracked_init_life_)) {
    HLOG_ERROR << "Get stopline target tracked_init_life failed!";
    return false;
  }
  if (!model_config->get_value("reserve_age", &reserve_age_)) {
    HLOG_ERROR << "Get stopline target reserve_age failed!";
    return false;
  }
  InitBase(options, detected_stopline_ptr);
  tracked_element_ = std::make_shared<StopLine>();
  *tracked_element_ = *detected_stopline_ptr;
  tracked_element_->id = id_;
  tracked_element_->tracked_count++;
  return true;
}

bool ArrowTarget::Init(const ProcessOption& options,
                       const ArrowPtr& detected_Arrow_ptr) {
  auto* config_manager = perception_lib::ConfigManager::Instance();
  const perception_lib::ModelConfig* model_config = nullptr;
  if (!config_manager->GetModelConfig("ArrowTargetParam", &model_config)) {
    HLOG_ERROR << "Parse config failed! Name: ArrowTargetParam";
    return false;
  }
  if (!model_config->get_value("tracked_init_life", &tracked_init_life_)) {
    HLOG_ERROR << "Get arrow target tracked_init_life failed!";
    return false;
  }
  if (!model_config->get_value("reserve_age", &reserve_age_)) {
    HLOG_ERROR << "Get arrow target reserve_age failed!";
    return false;
  }
  InitBase(options, detected_Arrow_ptr);
  tracked_element_ = std::make_shared<Arrow>();
  *tracked_element_ = *detected_Arrow_ptr;
  tracked_element_->id = id_;
  tracked_element_->tracked_count++;
  return true;
}
bool ZebraCrossingTarget::Init(
    const ProcessOption& options,
    const ZebraCrossingPtr& detected_zebracrossing_ptr) {
  auto* config_manager = perception_lib::ConfigManager::Instance();
  const perception_lib::ModelConfig* model_config = nullptr;
  if (!config_manager->GetModelConfig("ZebraCrossingTargetParam",
                                      &model_config)) {
    HLOG_ERROR << "Parse config failed! Name: ZebraCrossingTargetParam";
    return false;
  }
  if (!model_config->get_value("tracked_init_life", &tracked_init_life_)) {
    HLOG_ERROR << "Get zebracrossing target tracked_init_life  failed!";
    return false;
  }
  if (!model_config->get_value("reserve_age", &reserve_age_)) {
    HLOG_ERROR << "Get zebracrossing target reserve_age  failed!";
    return false;
  }

  // HLOG_DEBUG << "zebracrossing reserve_age:" << reserve_age_;
  // HLOG_DEBUG << "zebracrossing tracked_init_life:" << tracked_init_life_;
  InitBase(options, detected_zebracrossing_ptr);
  tracked_element_ = std::make_shared<ZebraCrossing>();
  *tracked_element_ = *detected_zebracrossing_ptr;
  tracked_element_->id = id_;
  tracked_element_->tracked_count++;
  return true;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
