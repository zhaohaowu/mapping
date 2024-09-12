/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： lane_fusion_pipeline.cc
 *   author     ： chenlongxi
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/map_fusion_02/pipelines/lane_fusion_pipeline.h"

#include <memory>

#include "base/utils/log.h"
#include "common/calc_util.h"
#include "modules/lane/road_topo_builder/topo_construct.h"
#include "modules/lane_loc/lane_loc.h"
#include "rviz/map_fusion_rviz.h"

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

  if (!model_config->get_value("near_junction_dis_thresh",
                               &options_.near_junction_dis_thresh)) {
    HLOG_ERROR << "Get near_junction_dis_thresh failed!";
    return false;
  }

  if (!model_config->get_value("junction_heading_diff",
                               &options_.junction_heading_diff)) {
    HLOG_ERROR << "Get junction_heading_diff failed!";
    return false;
  }

  if (!model_config->get_value("next_group_max_distance",
                               &options_.next_group_max_distance)) {
    HLOG_ERROR << "Get next_group_max_distance failed!";
    return false;
  }

  if (!model_config->get_value("use_occ", &options_.use_occ)) {
    HLOG_ERROR << "Get use_occ failed!";
    return false;
  }

  if (!model_config->get_value("junction_predict_distance",
                               &options_.junction_predict_distance)) {
    HLOG_ERROR << "Get junction_predict_distance failed!";
    return false;
  }

  if (!model_config->get_value("predict_farthest_dist",
                               &options_.predict_farthest_dist)) {
    HLOG_ERROR << "Get predict_farthest_dist failed!";
    return false;
  }

  if (!model_config->get_value("min_predict_interval",
                               &options_.min_predict_interval)) {
    HLOG_ERROR << "Get min_predict_interval failed!";
    return false;
  }

  if (!model_config->get_value("robust_percep_dist",
                               &options_.robust_percep_dist)) {
    HLOG_ERROR << "Get robust_percep_dist failed!";
    return false;
  }

  path_manager_ = std::make_shared<PathManager>();
  path_manager_->Init(options_);

  broken_pt_search_ = std::make_unique<BrokenPointSearch>();
  broken_pt_search_->Init();

  road_constructor_ = std::make_unique<RoadConstruct>();
  road_constructor_->Init(options_);

  lane_loc_ = std::make_unique<lane_loc::LaneLoc>();

  road_topo_constructor_ = std::make_unique<RoadTopoConstruct>();
  road_topo_constructor_->Init(options_);

  junction_check_ = std::make_unique<JunctionCheck>();
  junction_check_->Init(options_);

  lane_prediction_ = std::make_unique<LanePrediction>();
  lane_prediction_->Init(options_);
  virtual_line_gen_ = std::make_unique<VirtualLineGen>();
  virtual_line_gen_->Init(options_);

  initialized_ = true;

  return true;
}

void LaneFusionPipeline::Clear() {
  broken_pt_search_->Clear();
  road_constructor_->Clear();
  lane_prediction_->Clear();
  road_topo_constructor_->Clear();
  virtual_line_gen_->Clear();
}

void LaneFusionPipeline::InsertPose(const LocInfo::Ptr& pose) {
  if (!path_manager_) {
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

  MF_RVIZ->VizEleMap(element_map_ptr);

  // 获取历史和预测轨迹
  auto path = std::make_shared<std::vector<KinePosePtr>>();
  path_manager_->GetPath(path.get());
  auto curr_pose = path_manager_->LatestPose();
  MF_RVIZ->VizPath(*path, *curr_pose);

  // 计算切分点和切分线
  auto ret_bps = broken_pt_search_->SearchCtp(path, curr_pose, element_map_ptr);
  if (!ret_bps) {
    HLOG_ERROR << "Search broken point failed";
    return false;
  }

  std::vector<CutPoint> cut_points;
  cut_points = broken_pt_search_->GetCutPoints();
  MF_RVIZ->VizCutpoint(cut_points, element_map_ptr->map_info.stamp);

  std::deque<Line::Ptr> lines;
  lines = broken_pt_search_->GetLines();

  // 构建车道
  auto ret_rc = road_constructor_->ConstructRoad(cut_points, lines, path,
                                                 curr_pose, element_map_ptr);
  if (!ret_rc) {
    HLOG_ERROR << "Construct road failed";
    return false;
  }

  std::vector<Eigen::Vector3f> distpoints;
  distpoints = road_constructor_->GetDistPoints();
  MF_RVIZ->VizDistpoint(distpoints, element_map_ptr->map_info.stamp);

  std::vector<Group::Ptr> groups;
  groups = road_constructor_->GetGroups();

  std::deque<Line::Ptr> occ_lines;
  occ_lines = road_constructor_->GetFitOcc();
  MF_RVIZ->VizFitOcc(occ_lines, element_map_ptr->map_info.stamp);

  bool lane_loc_state = lane_loc_->UpdateGroups(&groups);
  if (!lane_loc_state) {
    HLOG_WARN << "update group failed";
  }

  // 路口判断
  int status = junction_check_->Process(groups);
  MF_RVIZ->VizJunctionStatus(status, element_map_ptr->map_info.stamp);
  // 虚拟线构建
  virtual_line_gen_->ConstructVirtualLine(&groups);
  // 构造拓扑
  road_topo_constructor_->ConstructTopology(element_map_ptr->map_info.stamp,
                                            &groups, path, curr_pose);

  // 车道线预测
  lane_prediction_->SetEgoLaneId();
  lane_prediction_->ComputeHeadingCompensation(curr_pose);
  lane_prediction_->LaneForwardPredict(&groups,
                                       element_map_ptr->map_info.stamp);
  MF_RVIZ->VizGroup(groups, element_map_ptr->map_info.stamp);
  MF_RVIZ->VizGuidePoint(groups, element_map_ptr->map_info.stamp);

  return true;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
