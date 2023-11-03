/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_fusion_component.h
 *   author     ： taoshaoyuan
 *   date       ： 2023.08
 ******************************************************************************/

#pragma once

#include <cyber/component/timer_component.h>
#include <cyber/cyber.h>
#include <depend/proto/local_mapping/local_map.pb.h>
#include <depend/proto/localization/localization.pb.h>
#include <depend/proto/localization/node_info.pb.h>
#include <depend/proto/map/map.pb.h>
#include <depend/proto/planning/planning.pb.h>

#include <memory>

namespace hozon {
namespace mp {
namespace mf {

class MapFusion;

class MapFusionComponent final : public apollo::cyber::TimerComponent {
 public:
  MapFusionComponent() = default;
  ~MapFusionComponent() override = default;

  bool Init() override;
  void Clear() override;
  bool Proc() override;

 private:
  void ProcForService();
  std::shared_ptr<apollo::cyber::Writer<hozon::hdmap::Map>> map_writer_ =
      nullptr;
  std::shared_ptr<apollo::cyber::Writer<hozon::routing::RoutingResponse>>
      routing_writer_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<hozon::localization::Localization>>
      localization_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<hozon::localization::HafNodeInfo>>
      plugin_node_info_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<hozon::mapping::LocalMap>>
      local_map_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<hozon::planning::ADCTrajectory>>
      planning_reader_ = nullptr;

  std::shared_ptr<MapFusion> mf_ = nullptr;
  std::mutex mtx_;
  std::atomic_bool running_proc_service_ = {false};
  std::shared_ptr<std::thread> proc_service_ = nullptr;
};

CYBER_REGISTER_COMPONENT(MapFusionComponent);

}  // namespace mf
}  // namespace mp
}  // namespace hozon
