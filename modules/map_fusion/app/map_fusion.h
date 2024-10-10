/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_fusion.h
 *   author     ： chenlongxi
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <depend/proto/local_mapping/local_map.pb.h>
#include <depend/proto/localization/localization.pb.h>
#include <depend/proto/localization/node_info.pb.h>
#include <depend/proto/map/map.pb.h>
#include <depend/proto/planning/planning.pb.h>
#include <depend/proto/routing/routing.pb.h>
#include <yaml-cpp/yaml.h>

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "base/utils/log.h"
#include "modules/map_fusion/base/element_map.h"
#include "modules/map_fusion/data_convert/data_convert.h"
#include "modules/map_fusion/pipelines/geo_optimization_pipeline.h"
#include "modules/map_fusion/pipelines/lane_fusion_pipeline.h"
#include "modules/map_fusion/pipelines/tlr_fusion_pipeline.h"

namespace hozon {
namespace mp {
namespace mf {

class MapFusion {
 public:
  MapFusion() = default;
  ~MapFusion() = default;
  int Init(const YAML::Node& conf);
  int OnLocalization(
      const std::shared_ptr<hozon::localization::Localization>& msg);
  int OnInsPlugin(
      const std::shared_ptr<const hozon::localization::HafNodeInfo>& ins_msg);
  void Stop();

  int ProcPercep(
      const std::shared_ptr<hozon::localization::Localization>& curr_loc,
      const std::shared_ptr<hozon::mapping::LocalMap>& curr_local_map,
      const std::shared_ptr<hozon::perception::PerceptionObstacles>& curr_obj,
      hozon::hdmap::Map* fusion_map, hozon::routing::RoutingResponse* routing);

 private:
  LocInfo::Ptr cur_loc_info_ = nullptr;
  ElementMap::Ptr cur_elem_map_ = nullptr;
  Group::Ptr cur_group_ = nullptr;
  std::unique_ptr<LaneFusionPipeline> lane_fusion_ptr_ = nullptr;
  std::unique_ptr<GeoOptimizationPipeline> geo_optimization_ptr_ = nullptr;
  HistoryId history_id_;
  bool InDataMapping(
      const std::shared_ptr<hozon::mapping::LocalMap>& map_msg,
      const std::shared_ptr<hozon::perception::PerceptionObstacles>& obj_msg);
  bool OutDataMapping(hozon::hdmap::Map* percep_map,
                      hozon::routing::RoutingResponse* percep_routing,
                      const std::vector<Group::Ptr>& groups,
                      const ElementMap::Ptr& element_map_ptr,
                      const std::map<Id, Zebra::Ptr>& zebra,
                      const std::map<Id, Stpl::Ptr>& stopline);
  void Clear();
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
