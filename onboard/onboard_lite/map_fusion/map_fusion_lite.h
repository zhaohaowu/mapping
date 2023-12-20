/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_fusion_lite.h
 *   author     ： xuliang
 *   date       ： 2023.11
 ******************************************************************************/
#pragma once

#include <adf-lite/include/base.h>
#include <depend/perception-base/base/fault/fault_info.h>
#include <depend/perception-lib/lib/fault_manager/fault_manager.h>
#include <depend/perception-lib/lib/health_manager/health_manager.h>

#include <memory>
#include <mutex>

#include "depend/nos/x86_2004/include/adf-lite/include/executor.h"
#include "depend/nos/x86_2004/include/adf/include/node_proto_register.h"
#include "modules/map_fusion/include/map_fusion/map_fusion.h"

namespace hozon {
namespace perception {
namespace common_onboard {

using hozon::mp::mf::MapFusion;
using hozon::netaos::adf_lite::Bundle;

class MapFusionLite : public hozon::netaos::adf_lite::Executor {
 public:
  MapFusionLite() = default;
  ~MapFusionLite() = default;

  int32_t AlgInit() override;
  void AlgRelease() override;

 private:
  void RegistMessageType();
  void RegistProcessFunc();

  int32_t OnLocation(Bundle* input);
  int32_t OnLocalMap(Bundle* input);
  int32_t OnLocPlugin(Bundle* input);

  int32_t MapFusionOutput(Bundle* output);

 private:
  std::shared_ptr<hozon::localization::Localization> GetLatestLoc();
  std::shared_ptr<hozon::mapping::LocalMap> GetLatestLocalMap();
  std::shared_ptr<hozon::localization::HafNodeInfo> GetLatestLocPlugin();
  int SendFusionResult(
      const std::shared_ptr<hozon::localization::Localization>& location,
      const std::shared_ptr<hozon::hdmap::Map>& map,
      hozon::routing::RoutingResponse* routing);

 private:
  std::unique_ptr<MapFusion> mf_ = nullptr;

  std::mutex plugin_mtx_;
  std::shared_ptr<hozon::localization::HafNodeInfo> curr_plugin_ = nullptr;
  std::mutex loc_mtx_;
  std::shared_ptr<hozon::localization::Localization> curr_loc_ = nullptr;
  std::mutex local_map_mtx_;
  std::shared_ptr<hozon::mapping::LocalMap> curr_local_map_ = nullptr;
};

REGISTER_ADF_CLASS(MapFusionLite, MapFusionLite);

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
