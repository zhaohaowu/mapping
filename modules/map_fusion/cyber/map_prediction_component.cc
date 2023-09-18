/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_prediction_component.cc
 *   author     ： zhangshuo
 *   date       ： 2023.09
 ******************************************************************************/

#include "cyber/map_prediction_component.h"

#include <gflags/gflags.h>

#include <memory>

#include "map_fusion/map_prediction/map_prediction.h"
#include "util/temp_log.h"

DEFINE_string(channel_ins_node_info_mpc, "ins_node_info",
              "channel of ins msg from ins fusion");
// DEFINE_string(channel_localMap_node_info, "localMap_node_info",
//               "channel of localMap msg from topo");
DEFINE_string(channel_hqMap_node_info, "hqMap_node_info",
              "channel of hqMap msg from hqMap");
DEFINE_string(channel_LocalMap_node_info, "lmp/local_map_provider",
              "channel of hq map msg from prior provider");

namespace hozon {
namespace mp {
namespace mf {

bool MapPredictionComponent::Init() {
  prediction_ = std::make_shared<MapPrediction>();
  ins_reader_ = node_->CreateReader<adsfi_proto::internal::HafNodeInfo>(
      FLAGS_channel_ins_node_info_mpc,
      [this](const std::shared_ptr<adsfi_proto::internal::HafNodeInfo>& msg) {
        OnInsNodeInfo(msg);
      });
  // localMap_reader_ = node_->CreateReader<hozon::hdmap::Map>(
  //     FLAGS_channel_localMap_node_info,
  //     [this](const std::shared_ptr<hozon::hdmap::Map>& msg) {
  //       OnLocalMap(msg);
  //     });
  hqMap_reader_ = node_->CreateReader<hozon::hdmap::Map>(
      FLAGS_channel_hqMap_node_info,
      [this](const std::shared_ptr<hozon::hdmap::Map>& msg) { OnHqMap(msg); });
  lm_reader_ = node_->CreateReader<hozon::mapping::LocalMap>(
      FLAGS_channel_LocalMap_node_info,
      [this](const std::shared_ptr<hozon::mapping::LocalMap>& msg) {
        OnLocalMap(msg);
      });
  return true;
}

void MapPredictionComponent::OnInsNodeInfo(
    const std::shared_ptr<adsfi_proto::internal::HafNodeInfo>& msg) {
  if (!prediction_) {
    HLOG_ERROR << "nullptr prediction";
  }
  prediction_->OnInsNodeInfo(msg);
}

// void MapPredictionComponent::OnLocalMap(
//     const std::shared_ptr<const hozon::hdmap::Map>& msg) {
//   if (!prediction_) {
//     HLOG_ERROR << "nullptr prediction";
//   }
//   prediction_->OnLocalMap(msg);
// }

void MapPredictionComponent::OnHqMap(
    const std::shared_ptr<const hozon::hdmap::Map>& msg) {
  if (!prediction_) {
    HLOG_ERROR << "nullptr prediction";
  }
  prediction_->OnHqMap(msg);
}

void MapPredictionComponent::OnLocalMap(
    const std::shared_ptr<hozon::mapping::LocalMap>& msg) {
  if (!prediction_) {
    HLOG_ERROR << "nullptr tppo map assignment";
    return;
  }
  prediction_->OnLocalMap(msg);
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
