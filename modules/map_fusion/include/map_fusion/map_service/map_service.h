/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_service_component.h
 *   author     ： mashaoping
 *   date       ： 2023.10
 ******************************************************************************/

#pragma once

#include <depend/proto/localization/node_info.pb.h>

#include <memory>
#include <queue>
#include <string>

#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "common/util/macros.h"
#include "common/util/message_util.h"
#include "common/util/util.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "cyber/node/writer.h"
#include "cyber/timer/timer.h"
#include "map/ehr/ehr_factory.h"
#include "map/hdmap/hdmap.h"
#include "map_fusion/map_service/ehp/amap_core.h"
#include "modules/map_fusion/include/map_fusion/map_service/ehp/amap_core.h"
#include "proto/localization/localization.pb.h"
#include "proto/map/adasisv3.pb.h"
#include "proto/map/ehp.pb.h"
#include "proto/map/map.pb.h"
#include "proto/planning/planning.pb.h"
#include "proto/routing/routing.pb.h"
#include "util/geo.h"

namespace hozon {
namespace mp {
namespace mf {
/**
 * @class Ehp
 *
 * @brief ehp module main class, it processes localization, chassis, and
 * pad data to compute throttle, brake and steer values.
 */
class MapService {
  // friend class EhpTestBase;

 public:
  MapService() = default;
  ~MapService() = default;
  bool Init();

  void OnInsAdcNodeInfo(const hozon::localization::HafNodeInfo& ins_msg,
                        const hozon::planning::ADCTrajectory& adc_msg);

  std::shared_ptr<hozon::hdmap::Map> GetCropMap() const { return cro_map_; }

  std::shared_ptr<hozon::routing::RoutingResponse> GetRouting() const {
    return routing_;
  }

 private:
  bool EhpProc(const hozon::localization::HafNodeInfo& ins_msg,
               const hozon::planning::ADCTrajectory& adc_msg,
               const std::shared_ptr<hozon::routing::RoutingResponse>& routing);
  bool BinProc(const hozon::localization::HafNodeInfo& ins_msg,
               const std::shared_ptr<hozon::hdmap::Map>& map);

  void MergeMap(const hozon::hdmap::Map& extend_map,
                const hozon::hdmap::Map& shrink_map);
  std::shared_ptr<hozon::hdmap::Map> cro_map_ = nullptr;
  std::shared_ptr<hozon::routing::RoutingResponse> routing_ = nullptr;

  std::unique_ptr<hozon::ehr::Ehr> ehr_ = nullptr;
  std::shared_ptr<hdmap::HDMap> hd_map_ = nullptr;
  hozon::common::math::Vec2d last_pose_;

  std::chrono::steady_clock::time_point last_send_time_;
  hozon::mp::mf::AmapAdapter amap_adapter_;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
