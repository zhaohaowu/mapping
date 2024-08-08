/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_fusion.cc
 *   author     ： chenlongxi
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/map_fusion_02/app/map_fusion.h"

#include <tuple>

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
  HLOG_INFO << "Create loc info & element map";

  // geometry map processing

  // road construction
  lane_fusion_ = std::make_unique<LaneFusionPipeline>();

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

int MapFusion::ProcPercep(
    const std::shared_ptr<hozon::localization::Localization>& curr_loc,
    const std::shared_ptr<hozon::mapping::LocalMap>& curr_local_map,
    const std::shared_ptr<hozon::perception::PerceptionObstacles>& curr_obj,
    hozon::hdmap::Map* percep_map,
    hozon::routing::RoutingResponse* percep_routing) {
  if (curr_loc == nullptr || curr_local_map == nullptr ||
      percep_map == nullptr || percep_routing == nullptr) {
    HLOG_ERROR << "nullptr input input";
    return -1;
  }

  if (!InDataMapping(curr_loc, curr_local_map, curr_obj)) {
    HLOG_ERROR << "Input data mapping failed";
    return -1;
  }

  HLOG_DEBUG << "Proc Pilot start!";
  GeoMapProcess();
  ElementFusionProcess();
  FillMapProcess();
  HLOG_DEBUG << "Proc Pilot end!";

  if (!OutDataMapping(percep_map, percep_routing)) {
    HLOG_ERROR << "Output data mapping failed";
    return -1;
  }

  return 0;
}

bool MapFusion::InDataMapping(
    const std::shared_ptr<hozon::localization::Localization>& loc_msg,
    const std::shared_ptr<hozon::mapping::LocalMap>& map_msg,
    const std::shared_ptr<hozon::perception::PerceptionObstacles>& obj_msg) {
  if (!DataConvert::Localization2LocInfo(loc_msg, cur_loc_info_)) {
    return false;
  }

  if (!DR_MANAGER->PushDrData(cur_loc_info_)) {
    HLOG_ERROR << "localization timestamp error";
  }

  if (!DataConvert::LocalMap2ElmentMap(map_msg, obj_msg, cur_elem_map_)) {
    return false;
  }

  return true;
}

void MapFusion::GeoMapProcess() {}

void MapFusion::ElementFusionProcess() {}

void MapFusion::FillMapProcess() {}

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

}  // namespace mf
}  // namespace mp
}  // namespace hozon
