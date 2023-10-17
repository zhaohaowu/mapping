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
class MapServiceComponent final : public apollo::cyber::TimerComponent {
  // friend class EhpTestBase;

 public:
  MapServiceComponent();
  bool Init() override;

  bool Proc() override;
  bool EhpProc(hozon::routing::RoutingResponse* routing);
  bool BinProc(hozon::hdmap::Map* map);

  void MergeMap(const hozon::hdmap::Map& extend_map,
                const hozon::hdmap::Map& shrink_map);

 private:
  hozon::localization::HafNodeInfo latest_localization_;
  hozon::planning::ADCTrajectory latest_planning_;

  std::mutex mutex_;

  std::shared_ptr<apollo::cyber::Reader<hozon::planning::ADCTrajectory>>
      planning_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<hozon::localization::Localization>>
      localization_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<hozon::localization::HafNodeInfo>>
      ins_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<hozon::hdmap::Map>> prior_writer_ =
      nullptr;
  std::shared_ptr<apollo::cyber::Writer<hozon::routing::RoutingResponse>>
      routing_response_writer_ = nullptr;
  std::unique_ptr<hozon::ehr::Ehr> ehr_ = nullptr;
  std::shared_ptr<hdmap::HDMap> hd_map_ = nullptr;
  hozon::common::math::Vec2d last_pose_;

  std::chrono::steady_clock::time_point last_send_time_;
  hozon::mp::mf::AmapAdapter amap_adapter_;
};

CYBER_REGISTER_COMPONENT(MapServiceComponent)  // NOLINT
}  // namespace mf
}  // namespace mp
}  // namespace hozon
