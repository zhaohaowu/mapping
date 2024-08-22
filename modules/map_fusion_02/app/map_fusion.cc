/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_fusion.cc
 *   author     ： chenlongxi
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/map_fusion_02/app/map_fusion.h"

#include <tuple>

#include "modules/map_fusion_02/data_manager/ins_data_manager.h"
#include "modules/map_fusion_02/data_manager/location_data_manager.h"
#include "modules/map_fusion_02/data_manager/percep_obj_manager.h"
#include "util/mapping_log.h"
#include "util/rate.h"
#include "util/tic_toc.h"

namespace hozon {
namespace mp {
namespace mf {

// 各模块自行添加
// mapfusion内部各模块间的数据接口自行对齐
// 需要定义新的数据结构放在base文件夹下，可添加头文件

int MapFusion::Init(const YAML::Node& conf) {
  cur_loc_info_ = std::make_shared<LocInfo>();
  cur_elem_map_ = std::make_shared<ElementMap>();
  cur_group_ = std::make_shared<Group>();
  HLOG_INFO << "Create loc info & element map";

  // geometry map processing
  geo_optimization_ptr_ = std::make_unique<GeoOptimizationPipeline>();
  geo_optimization_ptr_->Init();
  // road construction
  lane_fusion_ptr_ = std::make_unique<LaneFusionPipeline>();
  lane_fusion_ptr_->Init();
  // rviz init执行,没有构造执行的原因是单例的初始化顺序不确定
  if (!MF_RVIZ->Init()) {
    HLOG_FATAL << "RvizAgent init failed";
  } else {
    HLOG_INFO << "RvizAgent init success";
  }
  // TODO(a): map service

  // TODO(a): ld map matching & prediction

  // TODO(a): topo generation

  return 0;
}

void MapFusion::Stop() {
  HLOG_ERROR << "try stopping map prediction";

  HLOG_ERROR << "done stopping map prediction";
  HLOG_ERROR << "done stopping map fusion";
}

int MapFusion::OnLocalization(
    const std::shared_ptr<hozon::localization::Localization>& loc_msg) {
  if (!DataConvert::Localization2LocInfo(loc_msg, cur_loc_info_)) {
    HLOG_ERROR << "Localization2LocInfo error";
    return -1;
  }

  if (!LOCATION_MANAGER->PushOriginLocData(cur_loc_info_)) {
    HLOG_ERROR << "localization timestamp error";
    return -1;
  }

  lane_fusion_ptr_->InsertPose(cur_loc_info_);

  return 0;
}

int MapFusion::OnInsPlugin(
    const std::shared_ptr<const hozon::localization::HafNodeInfo>& ins_msg) {
  if (ins_msg == nullptr) {
    return -1;
  }
  INS_MANAGER->PushIns(*ins_msg);
  return 0;
}

int MapFusion::ProcPercep(
    const std::shared_ptr<hozon::localization::Localization>& curr_loc,
    const std::shared_ptr<hozon::mapping::LocalMap>& curr_local_map,
    const std::shared_ptr<hozon::perception::PerceptionObstacles>& curr_obj,
    hozon::hdmap::Map* fusion_map, hozon::routing::RoutingResponse* routing) {
  if (curr_loc == nullptr || curr_local_map == nullptr ||
      fusion_map == nullptr || routing == nullptr) {
    HLOG_ERROR << "nullptr input input";
    return -1;
  }
  ProcessOption option;
  option.timestamp = curr_local_map->header().data_stamp();
  // 数据输入处理转换
  if (!InDataMapping(curr_local_map, curr_obj)) {
    HLOG_ERROR << "Input data mapping failed";
    return -1;
  }
  // 各个pipeline处理
  HLOG_DEBUG << "Proc Pilot start!";
  geo_optimization_ptr_->Process(option, cur_elem_map_);
  lane_fusion_ptr_->Process(cur_elem_map_);
  HLOG_DEBUG << "Proc Pilot end!";
  // 输出数据处理转换
  if (!OutDataMapping(fusion_map, routing)) {
    HLOG_ERROR << "Output data mapping failed";
    return -1;
  }
  // 单帧处理结束各个pipeline清理内部元素
  Clear();
  return 0;
}

bool MapFusion::InDataMapping(
    const std::shared_ptr<hozon::mapping::LocalMap>& map_msg,
    const std::shared_ptr<hozon::perception::PerceptionObstacles>& obj_msg) {
  cur_elem_map_->map_info.stamp = map_msg->header().data_stamp();
  if (!DataConvert::LocalMap2ElmentMap(map_msg, cur_elem_map_)) {
    return false;
  }

  if (!OBJECT_MANAGER->PushObjects(obj_msg)) {
    HLOG_ERROR << "push perception object failed";
  }

  LocInfo::ConstPtr perception_pose =
      LOCATION_MANAGER->GetLocationByTimeStamp(map_msg->header().data_stamp());
  if (perception_pose == nullptr) {
    HLOG_ERROR << "map_msg time is:"
               << std::to_string(map_msg->header().data_stamp());
    HLOG_ERROR << "map_msg is nullptr";
    return false;
  }
  LOCATION_MANAGER->SetTimeStampLocation(perception_pose);
  LOCATION_MANAGER->PushLocalMapLocData(map_msg->header().data_stamp(),
                                        perception_pose);

  return true;
}

bool MapFusion::OutDataMapping(
    hozon::hdmap::Map* percep_map,
    hozon::routing::RoutingResponse* percep_routing) {
  // TODO(a): map从对应模块的对象中获取,指针结构自行调整
  // std::shared_ptr<hozon::hdmap::Map> map;
  // if (map == nullptr) {
  //   HLOG_ERROR << "get nullptr percep map";
  //   return false;
  // }
  // percep_map->CopyFrom(*map);

  // TODO(a): routing从对应模块的对象中获取,指针结构自行调整
  // std::shared_ptr<hozon::routing::RoutingResponse> routing;
  // if (routing == nullptr) {
  //   HLOG_ERROR << "get nullptr routing!";
  //   return false;
  // }
  // percep_routing->CopyFrom(*routing);

  return true;
}

void MapFusion::Clear() {
  cur_elem_map_->Clear();
  cur_group_->Clear();
  geo_optimization_ptr_->Clear();
  lane_fusion_ptr_->Clear();
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
