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
#include "modules/map_fusion/include/map_fusion/map_select/map_select_lite.h"
#include "onboard/onboard_lite/phm_comment_lite/proto/running_mode.pb.h"

namespace hozon {
namespace perception {
namespace common_onboard {

using hozon::mp::mf::MapFusion;
using hozon::mp::mf::select::MapSelectLite;
using hozon::netaos::adf_lite::Bundle;

enum WorkMode { PerceptMap, FusionMap, Normal };

class MapFusionLite : public hozon::netaos::adf_lite::Executor {
 public:
  MapFusionLite() = default;
  ~MapFusionLite() = default;

  int32_t AlgInit() override;
  void AlgRelease() override;
  int32_t OnRunningMode(hozon::netaos::adf_lite::Bundle* input);
  static Eigen::Vector3d Qat2EulerAngle(const Eigen::Quaterniond& q);

 private:
  void RegistMessageType();
  void RegistProcessFunc();

  int32_t OnLocation(Bundle* input);
  int32_t OnLocalMap(Bundle* input);
  int32_t OnLocPlugin(Bundle* input);
  int32_t OnFCTIn(Bundle* input);
  int32_t OnObj(Bundle* input);
  int DebugSelectMap();
  int32_t MapFusionOutput(Bundle* output);

 private:
  std::shared_ptr<hozon::localization::Localization> GetLatestLoc();
  std::shared_ptr<hozon::mapping::LocalMap> GetLatestLocalMap();
  std::shared_ptr<hozon::localization::HafNodeInfo> GetLatestLocPlugin();
  std::shared_ptr<hozon::perception::PerceptionObstacles> GetLatestObj();
  std::shared_ptr<hozon::functionmanager::FunctionManagerIn> GetLatestFCTIn();
  int SendFusionResult(
      const std::shared_ptr<hozon::localization::Localization>& location,
      const std::shared_ptr<hozon::hdmap::Map>& map,
      mp::mf::select::MapSelectResult select,
      hozon::routing::RoutingResponse* routing);
  int SendPercepResult(
      const std::shared_ptr<hozon::localization::Localization>& location,
      const std::shared_ptr<hozon::hdmap::Map>& map,
      mp::mf::select::MapSelectResult select,
      const std::shared_ptr<hozon::routing::RoutingResponse>& routing);

  int MapFusionOutputEvaluation(
      const std::shared_ptr<hozon::localization::Localization>& location);
  int MapServiceFaultOutput(const hozon::mp::mf::MapServiceFault& fault);

 private:
  WorkMode work_mode_;
  std::shared_ptr<hozon::routing::RoutingResponse> curr_routing_ = nullptr;
  std::unique_ptr<MapFusion> mf_ = nullptr;
  std::unique_ptr<MapSelectLite> map_select_ = nullptr;

  std::mutex plugin_mtx_;
  std::shared_ptr<hozon::localization::HafNodeInfo> curr_plugin_ = nullptr;
  std::mutex loc_mtx_;
  std::shared_ptr<hozon::localization::Localization> curr_loc_ = nullptr;
  std::mutex local_map_mtx_;
  std::shared_ptr<hozon::mapping::LocalMap> curr_local_map_ = nullptr;
  int pre_fault_value_ = -1;
  std::mutex percep_obj_mtx_;
  std::shared_ptr<hozon::perception::PerceptionObstacles> curr_obj_ = nullptr;
  std::mutex fct_mtx_;
  std::shared_ptr<hozon::functionmanager::FunctionManagerIn> curr_fct_in_ =
      nullptr;
  std::mutex process_mtx_;
  mp::mf::select::MapSelectResult curr_map_type_ = {
      hozon::navigation_hdmap::MapMsg_MapType_INVALID, false, 2};
  bool select_debug_ = true;

  double frame_proc_maxtime_ = 0.0;
  int frame_proc_num = 0;
  int frame_overtime_nums = 0;
};

REGISTER_ADF_CLASS(MapFusionLite, MapFusionLite);

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
