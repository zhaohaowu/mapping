/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_fusion.cc
 *   author     ： taoshaoyuan
 *   date       ： 2023.08
 ******************************************************************************/

#include "map_fusion/map_fusion.h"

#include "map_fusion/map_prediction/map_prediction.h"
#include "map_fusion/topo_assignment/topo_assignment.h"

namespace hozon {
namespace mp {
namespace mf {

int MapFusion::Init(const std::string& conf) {
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

  return 0;
}

void MapFusion::OnInsNodeInfo(
    const std::shared_ptr<hozon::localization::HafNodeInfo>& msg) {
  if (!msg) {
    HLOG_ERROR << "nullptr ins node info";
    return;
  }

  topo_->OnInsNodeInfo(msg);
  pred_->OnInsNodeInfo(msg);
}

void MapFusion::OnHQMap(const std::shared_ptr<hozon::hdmap::Map>& msg) {
  if (!msg) {
    HLOG_ERROR << "nullptr HQ map";
    return;
  }

  // topo_->OnHQMap(msg);
  pred_->OnHqMap(msg);
}

void MapFusion::OnLocalMap(
    const std::shared_ptr<hozon::mapping::LocalMap>& msg) {
  if (!msg) {
    HLOG_ERROR << "nullptr local map";
    return;
  }

  topo_->OnLocalMap(msg);
  auto topo_map = topo_->GetTopoMap();

  pred_->OnTopoMap(topo_map);
  std::shared_ptr<hdmap::Map> map = nullptr;
  map = pred_->GetPredictionMap();
  if (map) {
    std::lock_guard<std::mutex> lock(mtx_);
    pred_map_ = std::make_shared<hdmap::Map>();
    pred_map_->CopyFrom(*map);
  }
}

void MapFusion::OnLocalMapLocation(
    const std::shared_ptr<hozon::localization::Localization>& msg) {
  if (!msg) {
    HLOG_ERROR << "nullptr local map location";
    return;
  }

  topo_->OnLocalMapLocation(msg);
}

std::shared_ptr<hozon::hdmap::Map> MapFusion::GetMap() {
  std::lock_guard<std::mutex> lock(mtx_);
  return pred_map_;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
