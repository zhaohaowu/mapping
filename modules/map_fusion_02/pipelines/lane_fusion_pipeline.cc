/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： lane_fusion_pipeline.cc
 *   author     ： chenlongxi
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/map_fusion_02/pipelines/lane_fusion_pipeline.h"

namespace hozon {
namespace mp {
namespace mf {

std::string LaneFusionPipeline::Name() const { return "LaneFusionPipeline"; }

bool LaneFusionPipeline::Init() {
  auto* config_manager = hozon::perception::lib::ConfigManager::Instance();
  const hozon::perception::lib::ModelConfig* model_config = nullptr;
  if (!config_manager->GetModelConfig(Name(), &model_config)) {
    HLOG_ERROR << "Parse config failed! Name: " << Name();
    return false;
  }

  if (!model_config->get_value("path_predict_range",
                               &options_.path_predict_range)) {
    HLOG_ERROR << "Get path_predict_range failed!";
    return false;
  }

  if (!model_config->get_value("path_back_range", &options_.path_back_range)) {
    HLOG_ERROR << "Get path_back_range failed!";
    return false;
  }

  if (!model_config->get_value("path_interval", &options_.path_interval)) {
    HLOG_ERROR << "Get path_interval failed!";
    return false;
  }
  if (!model_config->get_value("state_detect_range",
                               &options_.state_detect_range)) {
    HLOG_ERROR << "Get state_detect_range failed!";
    return false;
  }

  if (!model_config->get_value("lane_line_interp_dist",
                               &options_.lane_line_interp_dist)) {
    HLOG_ERROR << "Get lane_line_interp_dist failed!";
    return false;
  }

  if (!model_config->get_value("half_slice_length",
                               &options_.half_slice_length)) {
    HLOG_ERROR << "Get half_slice_length failed!";
    return false;
  }

  if (!model_config->get_value("min_lane_width", &options_.min_lane_width)) {
    HLOG_ERROR << "Get min_lane_width failed!";
    return false;
  }

  if (!model_config->get_value("max_lane_width", &options_.max_lane_width)) {
    HLOG_ERROR << "Get max_lane_width failed!";
    return false;
  }

  if (!model_config->get_value("lane_speed_limit_kmph",
                               &options_.lane_speed_limit_kmph)) {
    HLOG_ERROR << "Get lane_speed_limit_kmph failed!";
    return false;
  }
  if (!model_config->get_value("road_min_max_speed_kmph",
                               &options_.road_min_max_speed_kmph)) {
    HLOG_ERROR << "Get road_min_max_speed_kmph failed!";
    return false;
  }
  if (!model_config->get_value("road_max_max_speed_kmph",
                               &options_.road_max_max_speed_kmph)) {
    HLOG_ERROR << "Get road_max_max_speed_kmph failed!";
    return false;
  }

  mf_rviz_ = std::unique_ptr<MapFusionRviz>();
  auto ret = mf_rviz_->Init();
  if (!ret) {
    HLOG_FATAL << "RvizAgent init failed";
  } else {
    HLOG_INFO << "RvizAgent init success";
  }

  path_manager_ = std::make_shared<PathManager>();
  path_manager_->Init(options_);

  // broken_pt_search_ = std::make_unique<BrokenPointSearch>();

  road_constructor_ = std::make_unique<RoadConstruct>();
  road_constructor_->Init(options_);

  initialized_ = true;

  return true;
}

void LaneFusionPipeline::Clear() {}

void LaneFusionPipeline::InsertPose(const LocInfo::Ptr& pose) {
  if (path_manager_) {
    HLOG_ERROR << "Path manager is nullptr";
    return;
  }

  KinePose curr;
  curr.stamp = pose->timestamp;
  curr.pos << static_cast<float>(pose->translation.x()),
      static_cast<float>(pose->translation.y()),
      static_cast<float>(pose->translation.z());
  curr.quat.w() = static_cast<float>(pose->quaternion.w());
  curr.quat.x() = static_cast<float>(pose->quaternion.x());
  curr.quat.y() = static_cast<float>(pose->quaternion.y());
  curr.quat.z() = static_cast<float>(pose->quaternion.z());
  // 注意：上游发过来的速度、加速度、角速度都是车体系下的
  curr.vel << static_cast<float>(pose->linear_velocity.x()),
      static_cast<float>(pose->linear_velocity.y()),
      static_cast<float>(pose->linear_velocity.z());
  curr.acc << static_cast<float>(pose->acceleration.x()),
      static_cast<float>(pose->acceleration.y()),
      static_cast<float>(pose->acceleration.z());
  curr.ang_vel << static_cast<float>(pose->angular_velocity.x()),
      static_cast<float>(pose->angular_velocity.y()),
      static_cast<float>(pose->angular_velocity.z());
  path_manager_->AddPose(curr);
}

bool LaneFusionPipeline::Process(const ElementMap::Ptr& element_map_ptr) const {
  if (!initialized_) {
    HLOG_ERROR << "Lane fusion pipline not initialized";
    return false;
  }

  mf_rviz_->VizEleMap(element_map_ptr);

  // 获取历史和预测轨迹
  auto path = std::make_shared<std::vector<KinePosePtr>>();
  path_manager_->GetPath(path.get());
  auto curr_pose = path_manager_->LatestPose();
  mf_rviz_->VizPath(*path, *curr_pose);

  // 计算切分点和切分线
  BrokenPointSearch bps;
  auto ret_bps = bps.SearchCtp(path, curr_pose, element_map_ptr);
  if (!ret_bps) {
    HLOG_ERROR << "Search broken point failed";
    return false;
  }

  std::vector<CutPoint> cut_points;
  bps.GetCutPoints(&cut_points);
  mf_rviz_->VizCutpoint(cut_points, element_map_ptr->map_info.stamp);

  std::deque<Line::Ptr> lines;
  bps.GetLines(&lines);

  // 构建车道
  auto ret_rc = road_constructor_->ConstructLane(cut_points, lines, path,
                                                 curr_pose, element_map_ptr);
  if (!ret_rc) {
    HLOG_ERROR << "Construct road failed";
    return false;
  }

  std::vector<Eigen::Vector3f> distpoints;
  road_constructor_->GetDistPoints(&distpoints);
  mf_rviz_->VizDistpoint(distpoints, element_map_ptr->map_info.stamp);

  std::vector<Group::Ptr> groups;
  road_constructor_->GetGroups(&groups);
  mf_rviz_->VizGroup(groups, element_map_ptr->map_info.stamp);

  return true;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
