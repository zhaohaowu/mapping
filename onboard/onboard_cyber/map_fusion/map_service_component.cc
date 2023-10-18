/*
 * Copyright (c) Hozon Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  ehp
 */
#include "onboard/onboard_cyber/map_fusion/map_service_component.h"

#include <cstdlib>
#include <iomanip>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "amap/GHDMapDefines.h"
#include "amap/GHDMapService.h"
#include "common/adapters/adapter_gflags.h"
#include "common/configs/config_gflags.h"
#include "common/file/file.h"
#include "common/math/vec2d.h"
#include "common/time/clock.h"
#include "cyber/common/log.h"
#include "map/hdmap/hdmap_util.h"
#include "proto/localization/node_info.pb.h"
#include "proto/map/ehp.pb.h"
#include "proto/map/map.pb.h"
#include "proto/routing/routing.pb.h"
#include "util/temp_log.h"

DEFINE_string(channel_prior, "mf/prior_map",
              "channel of prior map from prior provider");
DEFINE_string(channel_ins_node_info, "/PluginNodeInfo",
              "channel of ins msg from ins fusion");

namespace hozon {
namespace mp {
namespace mf {

using hozon::planning::ADCTrajectory;

bool MapServiceComponent::Init() {
  ins_reader_ = node_->CreateReader<hozon::localization::HafNodeInfo>(
      FLAGS_channel_ins_node_info);
  map_service_ = std::make_shared<MapService>();
  map_service_->Init();
  // apollo::cyber::ReaderConfig localization_reader_config;
  // localization_reader_config.channel_name = FLAGS_localization_topic;  //
  // NOLINT localization_reader_config.pending_queue_size = 10;

  // localization_reader_ = node_->CreateReader<Localization>(
  //     localization_reader_config, nullptr);
  // ACHECK(localization_reader_ != nullptr);

  apollo::cyber::ReaderConfig planning_reader_config;
  planning_reader_config.channel_name =
      FLAGS_planning_trajectory_topic;  // NOLINT
  planning_reader_config.pending_queue_size = 10;

  planning_reader_ =
      node_->CreateReader<ADCTrajectory>(planning_reader_config, nullptr);
  ACHECK(planning_reader_ != nullptr);

  prior_writer_ =
      node_->CreateWriter<hozon::hdmap::Map>(FLAGS_channel_prior);  // NOLINT
  ACHECK(prior_writer_ != nullptr);
  routing_response_writer_ =
      node_->CreateWriter<hozon::routing::RoutingResponse>(
          FLAGS_routing_response_topic);
  ACHECK(routing_response_writer_ != nullptr);

  return true;
}

bool MapServiceComponent::Proc() {
  ins_reader_->Observe();
  auto ins_msg = ins_reader_->GetLatestObserved();
  if (ins_msg == nullptr) {
    HLOG_ERROR << "localization msg is not ready!";
    return false;
  }
  latest_localization_ = *ins_msg;

  if (FLAGS_ehp_monitor == 2 || FLAGS_ehp_monitor == 0) {
    planning_reader_->Observe();
    auto adc_msg = planning_reader_->GetLatestObserved();  // NOLINT
    if (adc_msg == nullptr) {
      HLOG_ERROR << "adc msg is not ready!";
      return false;
    }
    latest_planning_ = *adc_msg;
  }
  map_service_->OnInsAdcNodeInfo(latest_localization_, latest_planning_);
  return true;
}

void MapServiceComponent::PubData() {
  if (FLAGS_map_service_mode == 0) {
    prior_writer_->Write(map_service_->GetCropMap());
  } else if (FLAGS_map_service_mode == 1) {
    prior_writer_->Write(map_service_->GetCropMap());
    routing_response_writer_->Write(map_service_->GetRouting());
  }
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
