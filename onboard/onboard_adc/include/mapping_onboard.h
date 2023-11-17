/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: shenlianchen
 *Date: 2023-08-31
 *****************************************************************************/
#pragma once

#include <adsf/node/node_base.h>
#include <adsfi/adb/include/core/core.h>

#include <memory>
#include <sstream>
#include <string>

#include "map_fusion/map_fusion.h"
#include "modules/dr/include/dr.h"
#include "modules/local_mapping/local_mapping.h"
#include "modules/location/location.h"
// #include
// "modules/map_fusion/include/map_fusion/map_prediction/map_prediction.h"
// #include
// "modules/map_fusion/include/map_fusion/topo_assignment/topo_assignment.h"
#include "onboard/onboard_adc/include/data_board.h"
#include "proto/dead_reckoning/dr.pb.h"

namespace hozon {
namespace mp {

class MappingAdc : public hz_Adsfi::NodeBase {
 public:
  int32_t AlgInit() override;
  int32_t ChassisImuCallBack(
      hz_Adsfi::NodeBundle* input);  // chassis和imu回调，回调中notify_one
                                     // dr线程
  int32_t LaneCallBack(
      hz_Adsfi::NodeBundle* input);  // lane回调，回调中notify_one
                                     // local_mapping线程
  int32_t PluginCallback(hz_Adsfi::NodeBundle* input);

  int32_t MapServiceCycleCallback(hz_Adsfi::NodeBundle* input);
  int32_t MapFusionCycleCallback(hz_Adsfi::NodeBundle* input);

  void AlgRelease() override;

 private:
  template <typename T>
  std::string Node2Xyz(const T& p);
  template <typename T>
  std::string Node2Xyzw(const T& p);

  std::shared_ptr<hozon::localization::Localization> GetLatestLoc();
  std::shared_ptr<hozon::mapping::LocalMap> GetLatestLocalMap();
  std::shared_ptr<hozon::routing::RoutingResponse> GetLatestRouting();
  int SendFusionResult(const std::shared_ptr<hozon::localization::Localization>& loc,
      const std::shared_ptr<hozon::hdmap::Map>& map,
                        const std::shared_ptr<hozon::routing::RoutingResponse>& routing);

 private:
  std::unique_ptr<std::thread> dr_thread_ptr_;
  std::unique_ptr<std::thread> local_mapping_thread_ptr_;
  std::unique_ptr<dr::DRInterface> dr_ = nullptr;
  std::unique_ptr<lm::LMapApp> lmap_ = nullptr;
  std::unique_ptr<loc::Localization> loc_ = nullptr;
  std::unique_ptr<mf::MapFusion> mf_ = nullptr;
  std::mutex plugin_mtx_;
  std::shared_ptr<hozon::localization::HafNodeInfo> curr_plugin_ = nullptr;
  std::mutex routing_mtx_;
  std::shared_ptr<hozon::routing::RoutingResponse> curr_routing_ = nullptr;
  std::mutex loc_mtx_;
  std::shared_ptr<hozon::localization::Localization> curr_loc_ = nullptr;
  std::mutex local_map_mtx_;
  std::shared_ptr<hozon::mapping::LocalMap> curr_local_map_ = nullptr;

  std::shared_ptr<hozon::dead_reckoning::DeadReckoning> dr_data_ptr_;

  DataBoard board_;
};

}  // namespace mp
}  // namespace hozon
