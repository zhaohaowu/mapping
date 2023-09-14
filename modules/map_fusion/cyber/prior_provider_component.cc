/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： prior_provider_component.cc
 *   author     ： taoshaoyuan/zuodongsheng
 *   date       ： 2023.09
 ******************************************************************************/

#include "cyber/prior_provider_component.h"

#include <gflags/gflags.h>

#include <memory>

#include "map_fusion/prior_provider/prior_provider.h"
#include "util/temp_log.h"

DEFINE_string(channel_prior, "mf/prior_map",
              "channel of prior map from prior provider");
DEFINE_string(conf_provider, "conf/mapping/map_fusion/map_fusion.yaml",
              "config file path for prior provider");
DEFINE_string(channel_ins_node_info_ppc, "ins_node_info",
              "channel of ins msg from ins fusion");

namespace hozon {
namespace mp {
namespace mf {

bool PriorProviderComponent::Init() {
  prior_writer_ = node_->CreateWriter<hozon::hdmap::Map>(FLAGS_channel_prior);

  ins_reader_ = node_->CreateReader<adsfi_proto::internal::HafNodeInfo>(
      FLAGS_channel_ins_node_info_ppc,
      [this](const std::shared_ptr<adsfi_proto::internal::HafNodeInfo>& msg) {
        OnInsNodeInfo(msg);
      });

  provider_ = std::make_shared<PriorProvider>();
  int ret = provider_->Init(FLAGS_conf_provider);
  if (ret < 0) {
    HLOG_ERROR << "init provider failed";
    return false;
  }

  return true;
}

void PriorProviderComponent::OnInsNodeInfo(
    const std::shared_ptr<adsfi_proto::internal::HafNodeInfo>& msg) {
  if (!provider_ || !prior_writer_) {
    HLOG_ERROR << "nullptr provider or prior writer";
    return;
  }
  auto map = provider_->GetPrior();

  prior_writer_->Write(map);
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
