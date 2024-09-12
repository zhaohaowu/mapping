/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_service_component.h
 *   author     ： mashaoping
 *   date       ： 2023.10
 ******************************************************************************/

#pragma once

#include <depend/proto/localization/node_info.pb.h>

#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "base/utils/log.h"
#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "common/util/macros.h"
#include "common/util/message_util.h"
#include "common/util/util.h"
#include "map/ehr/ehr_factory.h"
#include "map/hdmap/hdmap.h"
#include "map_fusion/map_service/baidu_map.h"
#include "map_fusion/map_service/ehp/amap_core.h"
#include "map_fusion/map_service/map_service_fault.h"
#include "modules/map_fusion/include/map_fusion/map_service/ehp/amap_core.h"
#include "proto/localization/localization.pb.h"
#include "proto/map/adasisv3.pb.h"
#include "proto/map/ehp.pb.h"
#include "proto/map/map.pb.h"
#include "proto/planning/planning.pb.h"
#include "proto/routing/nav_data.pb.h"
#include "proto/routing/routing.pb.h"
#include "util/geo.h"

namespace hozon {
namespace mp {
namespace mf {

struct IdHash {
  size_t operator()(const hozon::hdmap::Id& id) const {
    return std::hash<std::string>()(id.id());
  }
};

struct IdEqual {
  bool operator()(const hozon::hdmap::Id& a, const hozon::hdmap::Id& b) const {
    return a.id() == b.id();
  }
};
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
  ~MapService();
  bool Init();

  void OnInsAdcNodeInfo(const hozon::localization::HafNodeInfo& ins_msg,
                        const hozon::planning::ADCTrajectory& adc_msg);

  std::shared_ptr<hozon::routing::RoutingResponse> GetRouting() const {
    return routing_;
  }
  std::shared_ptr<std::vector<uint32_t>> GetLDRoutingRoadId() const {
    return routing_road_id_;
  }
  MapServiceFault GetFault();
  void UpdateHMINavService(
      const std::shared_ptr<hozon::hmi::NAVDataService>& nav_data);

 private:
  void GetUidThread();
  void BaiduProc();
  bool EhpProc(const hozon::localization::HafNodeInfo& ins_msg,
               const hozon::planning::ADCTrajectory& adc_msg,
               const std::shared_ptr<hozon::routing::RoutingResponse>& routing);
  bool BinProc(const hozon::localization::HafNodeInfo& ins_msg,
               const std::shared_ptr<hozon::hdmap::Map>& map);
  void SetCurrentPathId(const hozon::common::PointENU& utm_pos,
                        std::string* current_pathid);
  static void SetLaneIdsPool(
      const hozon::hdmap::LaneInfoConstPtr& current_lane,
      std::unordered_set<hozon::hdmap::Id, IdHash, IdEqual>* lane_id_pool,
      std::unordered_set<std::string>* last_roadid_pool);
  static void GetAroundId(
      const ::google::protobuf::RepeatedPtrField<::hozon::hdmap::Id>& ids,
      std::unordered_set<hozon::hdmap::Id, IdHash, IdEqual>* lane_id_pool);
  void SetFautl();
  std::shared_ptr<hozon::routing::RoutingResponse> routing_ = nullptr;
  std::shared_ptr<std::vector<uint32_t>> routing_road_id_ = nullptr;
  hozon::common::math::Vec2d last_pose_;
  std::chrono::steady_clock::time_point last_send_time_{};
  std::unique_ptr<hozon::ehr::Ehr> ehr_ = nullptr;
  std::unique_ptr<hozon::mp::mf::BaiDuMapEngine> baidu_map_ = nullptr;

  hozon::mp::mf::AmapAdapter amap_adapter_;

  std::future<void> amap_tsp_proc_{};
  std::mutex mutex_{};

  std::atomic<bool> is_amap_tsp_thread_stop_{false};
  std::string uuid_{};
  MapServiceFault fault_{MS_NO_ERROR};
  std::mutex fault_mutex_{};
  std::thread tmp_thread_;
  hozon::localization::HafNodeInfo ins_msg_;
  std::mutex ins_msg_thread_;
  std::mutex ms_nav_mtx_;
  std::mutex ld_routing_mtx_;
  std::shared_ptr<hozon::hmi::NAVDataService> hmi_nav_data_ = nullptr;
  std::atomic<bool> bd_thread_flag_{false};
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
