/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_fusion.cc
 *   author     ： taoshaoyuan
 *   date       ： 2023.08
 ******************************************************************************/

#include "map_fusion/map_fusion.h"

#include <tuple>

#include "map_fusion/map_prediction/map_prediction.h"
#include "map_fusion/map_service/map_service.h"
#include "map_fusion/map_service/map_table.h"
#include "map_fusion/topo_assignment/topo_assignment.h"
#include "util/mapping_log.h"
#include "util/rate.h"
#include "util/tic_toc.h"

DECLARE_bool(output_hd_map);

namespace hozon {
namespace mp {
namespace mf {

int MapFusion::Init(const std::string& conf) {
  (void)(conf);
  map_service_ = std::make_shared<MapService>();
  if (!map_service_->Init()) {
    HLOG_ERROR << "Init MapService failed";
    return -1;
  }

  topo_ = std::make_shared<TopoAssignment>();
  int ret = topo_->Init();
  if (ret < 0) {
    HLOG_ERROR << "Init TopoAssignment failed";
    return -1;
  }

  pred_ = std::make_shared<MapPrediction>();
  ret = pred_->Init();
  if (ret < 0) {
    HLOG_ERROR << "Init MapPrediction failed";
    return -1;
  }
  map_table_ = std::make_shared<MapTable>();

  return 0;
}

void MapFusion::Stop() {
  HLOG_ERROR << "try stopping map prediction";
  if (pred_) {
    pred_->Stop();
  }
  HLOG_ERROR << "done stopping map prediction";
  HLOG_ERROR << "done stopping map fusion";
}

int MapFusion::ProcService(
    const std::shared_ptr<hozon::localization::HafNodeInfo>& curr_node_info,
    const std::shared_ptr<hozon::planning::ADCTrajectory>& curr_planning,
    hozon::routing::RoutingResponse* routing) {
  if (routing == nullptr) {
    HLOG_ERROR << "nullptr input routing";
    return -1;
  }

  if (!curr_node_info) {
    HLOG_ERROR << "nullptr current plugin node info";
    return -1;
  }

  if (!map_service_) {
    HLOG_ERROR << "nullptr map service";
    return -1;
  }

  if (FLAGS_ehp_monitor == 2 || FLAGS_ehp_monitor == 0) {
    if (!curr_planning) {
      HLOG_ERROR << "nullptr current planning";
      return -1;
    }
    map_service_->OnInsAdcNodeInfo(*curr_node_info, *curr_planning);
  } else {
    hozon::planning::ADCTrajectory empty_planning;
    map_service_->OnInsAdcNodeInfo(*curr_node_info, empty_planning);
  }

  auto rout = map_service_->GetRouting();
  if (rout) {
    routing->CopyFrom(*rout);
  }
  return 0;
}

int MapFusion::ProcFusion(
    const std::shared_ptr<hozon::localization::Localization>& curr_loc,
    const std::shared_ptr<hozon::mapping::LocalMap>& curr_local_map,
    bool need_update_global_hd, hozon::hdmap::Map* fusion_map,
    hozon::routing::RoutingResponse* routing) {
  if (fusion_map == nullptr) {
    HLOG_ERROR << "input nullptr fusion map";
    return -1;
  }

  if (!curr_loc) {
    HLOG_ERROR << "input nullptr current localization";
    return -1;
  }

  if (!curr_local_map) {
    HLOG_ERROR << "input nullptr current local map";
    return -1;
  }

  if (!topo_) {
    HLOG_ERROR << "nullptr topo assignment";
    return -1;
  }

  if (!pred_) {
    HLOG_ERROR << "nullptr map prediction";
    return -1;
  }

  if (!routing) {
    HLOG_ERROR << "nullptr routing";
    return -1;
  }
  util::TicToc global_tic;
  util::TicToc local_tic;
  std::shared_ptr<hozon::hdmap::Map> map;
  if (!FLAGS_output_hd_map) {
    map_table_->OnLocalization(curr_loc);
    auto map_info = map_table_->GetMapTable();
    topo_->OnLocalization(curr_loc);
    HLOG_INFO << "topo OnLocalization cost " << local_tic.Toc();
    local_tic.Tic();
    topo_->OnLocalMap(curr_local_map, std::get<0>(map_info));
    HLOG_INFO << "topo OnLocalMap cost " << local_tic.Toc();
    local_tic.Tic();
    topo_->TopoAssign();
    HLOG_INFO << "topo TopoAssign cost " << local_tic.Toc();
    local_tic.Tic();
    //! 注意：TopoAssignment内部保证每次返回的都是全新的ptr，
    //! 不会存在两次调用得到的ptr指向同一片空间;
    auto topo_map = topo_->GetTopoMap();
    HLOG_INFO << "topo GetTopoMap cost " << local_tic.Toc();
    local_tic.Tic();

    pred_->OnLocalization(curr_loc);
    pred_->OnTopoMap(topo_map, map_info);
    pred_->Prediction();

    //! 注意：MapPrediction内部保证每次返回的都是全新的ptr，
    //! 不会存在两次调用得到的ptr指向同一片空间;
    map = pred_->GetPredictionMap();
  } else {
    pred_->OnLocalization(curr_loc);
    map = pred_->GetHdMap(need_update_global_hd, routing);
  }

  if (!map) {
    HLOG_ERROR << "get nullptr prediction map";
    return -1;
  }
  HLOG_INFO << "pred cost " << local_tic.Toc();
  local_tic.Tic();
  fusion_map->CopyFrom(*map);
  HLOG_INFO << "copy fusion_map cost " << local_tic.Toc();
  HLOG_INFO << "topo+pred cost " << global_tic.Toc();

  return 0;
}

MapServiceFault MapFusion::GetMapServiceFault() {
  if (map_service_ != nullptr) {
    return map_service_->GetFault();
  }
  return MS_NO_ERROR;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
