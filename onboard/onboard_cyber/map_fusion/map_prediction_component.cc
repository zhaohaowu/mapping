/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_prediction_component.cc
 *   author     ： zhangshuo
 *   date       ： 2023.09
 ******************************************************************************/

#include "onboard/onboard_cyber/map_fusion/map_prediction_component.h"
#include <gflags/gflags.h>

#include <memory>

#include "map_fusion/map_prediction/map_prediction.h"
#include "proto/localization/node_info.pb.h"
#include "util/mapping_log.h"

DEFINE_string(channel_pred_map, "mf/pred_map",
              "channel of pred map from map prediction");
DEFINE_string(channel_ins_node_info_mpc, "/PluginNodeInfo",
              "channel of ins msg from ins fusion");
DEFINE_string(channel_topo_map_mpc, "mf/topo_map",
              "channel of topo map from topo assignment");
DEFINE_string(channel_hqMap_node_info, "mf/prior_map",
              "channel of hqMap msg from hqMap");
DEFINE_string(channel_LocalMap_node_info, "mf/local_map_provider",
              "channel of hq map msg from prior provider");

namespace hozon {
namespace mp {
namespace mf {

bool MapPredictionComponent::Init() {
  prediction_ = std::make_shared<MapPrediction>();
  prediction_->Init();
  pred_writer_ = node_->CreateWriter<hozon::hdmap::Map>(FLAGS_channel_pred_map);
  ins_reader_ = node_->CreateReader<hozon::localization::HafNodeInfo>(
      FLAGS_channel_ins_node_info_mpc,
      [this](const std::shared_ptr<hozon::localization::HafNodeInfo>& msg) {
        OnInsNodeInfo(msg);
      });
  hqMap_reader_ = node_->CreateReader<hozon::hdmap::Map>(
      FLAGS_channel_hqMap_node_info,
      [this](const std::shared_ptr<hozon::hdmap::Map>& msg) { OnHqMap(msg); });
  topo_reader_ = node_->CreateReader<hozon::hdmap::Map>(
      FLAGS_channel_topo_map_mpc,
      [this](const std::shared_ptr<hozon::hdmap::Map>& msg) {
        OnTopoMap(msg);
      });
  return true;
}

void MapPredictionComponent::Clear() {
  if (prediction_) {
    prediction_->Stop();
  }
}

void MapPredictionComponent::OnInsNodeInfo(
    const std::shared_ptr<hozon::localization::HafNodeInfo>& msg) {
  if (!msg) {
    HLOG_ERROR << "message ins node info is null";
    return;
  }
  if (!prediction_) {
    HLOG_ERROR << "nullptr prediction";
    return;
  }
  prediction_->OnInsNodeInfo(msg);
}

void MapPredictionComponent::OnHqMap(
    const std::shared_ptr<hozon::hdmap::Map>& msg) {
  if (!msg) {
    HLOG_ERROR << "message hq map is null";
    return;
  }
  if (!prediction_) {
    HLOG_ERROR << "nullptr prediction";
    return;
  }
  prediction_->OnHqMap(msg);
}

void MapPredictionComponent::OnTopoMap(
    const std::shared_ptr<hozon::hdmap::Map>& msg) {
  if (!msg) {
    HLOG_ERROR << "message topo map is null";
    return;
  }
  if (!prediction_) {
    HLOG_ERROR << "nullptr prediction";
    return;
  }
  prediction_->OnTopoMap(msg);

  // 发送pred地图
  auto map = prediction_->GetPredictionMap();
  pred_writer_->Write(map);
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
