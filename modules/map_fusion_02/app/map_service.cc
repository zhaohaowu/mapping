/*
 * Copyright (c) Hozon Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  ehp
 */
#include "modules/map_fusion_02/app/map_service.h"
#include <unistd.h>
#include <algorithm>
#include <cstdlib>
#include <iomanip>
#include <list>
#include <memory>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "amap/GHDMapDefines.h"
#include "amap/GHDMapService.h"
#include "base/utils/log.h"
#include "common/adapters/adapter_gflags.h"
#include "common/configs/config_gflags.h"
#include "common/file/file.h"
#include "common/math/vec2d.h"
#include "common/time/clock.h"
#include "https/include/tsp_comm.h"
#include "lib/environment/environment.h"
#include "map/hdmap/hdmap.h"
#include "map/hdmap/hdmap_util.h"
#include "modules/map_fusion_02/data_manager/ins_data_manager.h"
#include "modules/map_fusion_02/modules/map_hd/include/global_hd_map.h"
#include "proto/localization/node_info.pb.h"
#include "proto/map/ehp.pb.h"
#include "proto/map/map.pb.h"
#include "proto/map/map_id.pb.h"
#include "proto/routing/routing.pb.h"
#include "util/mapping_log.h"
#include "util/tic_toc.h"
// NOLINTBEGIN
// DEFINE_double(radius, 500, "radius of the vehicle position");
// DEFINE_double(transform_distance, 200, "distance to update the map");
// NOLINTEND

namespace hozon {
namespace mp {
namespace mf {
std::string MapService::Name() const { return "MapServiceConf"; }
bool MapService::Init() {
  auto* config_manager = hozon::perception::lib::ConfigManager::Instance();
  const hozon::perception::lib::ModelConfig* model_config = nullptr;
  if (!config_manager->GetModelConfig(Name(), &model_config)) {
    HLOG_INFO << "Parse config failed! Name: " << Name();
    return false;
  }
  if (!model_config->get_value("map_service_mode",
                               &ms_option_.map_service_mode)) {
    HLOG_INFO << "Get map_service_mode failed!";
    return false;
  }
  if (!model_config->get_value("map_dir", &ms_option_.map_dir)) {
    HLOG_INFO << "Get map_dir failed!";
    return false;
  }

  routing_ = std::make_shared<hozon::routing::RoutingResponse>();
  auto* global_hd_map = hozon::mp::GlobalHdMap::Instance();
  if (ms_option_.map_service_mode == 0) {
    global_hd_map->GetHdMap()->LoadMapFromFile(ms_option_.map_dir +
                                               "/base_map.bin");
  } else if (ms_option_.map_service_mode == 1) {
    amap_tsp_proc_ = std::async(&MapService::GetUidThread, this);
    ehr_ = std::make_unique<hozon::ehr::AmapEhrImpl>();
    amap_adapter_ = std::make_unique<hozon::mp::mf::AmapAdapter>();
    amap_adapter_->Init();

    baidu_map_ = std::make_unique<hozon::mp::mf::BaiDuMapEngine>();
    baidu_map_->AlgInit();
    ld_thread_ = std::thread(&MapService::BaiduProc, this);
    hd_thread_ = std::thread(&MapService::AMapProc, this);

  } else if (FLAGS_map_service_mode == 2) {
    // todo map api
  }

  return true;
}

void MapService::GetUidThread() {
  static constexpr double kGetUUIDIntervalMs = 1000;
  std::ifstream ifs;
  ifs.open("/cfg/hd_uuid/uid.txt", std::ios::in);
  if (ifs.is_open()) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::getline(ifs, uuid_);
    HLOG_INFO << "uid from uid.txt: " << uuid_;
  }
  ifs.close();
  if (uuid_.empty()) {
    auto& tsp_client = hozon::netaos::https::TspComm::GetInstance();
    tsp_client.Init();
    int counter = 0;
    while (!is_amap_tsp_thread_stop_) {
      const double start_time = hozon::common::Clock::NowInMicroseconds();
      if (counter % 60 == 50) {
        std::future<hozon::netaos::https::TspComm::TspResponse> uuid_result =
            tsp_client.RequestHdUuid();
        auto uid_result = uuid_result.get();
        if (uid_result.result_code ==
            hozon::netaos::https::HttpsResult_Success) {
          HLOG_INFO << "uid: " << uid_result.response;
          std::ofstream ofs("/cfg/hd_uuid/uid.txt");
          std::lock_guard<std::mutex> lock(mutex_);
          uuid_ = uid_result.response;
          ofs << uuid_ << std::endl;
          ofs.close();
        } else {
          HLOG_ERROR << "uid code: " << uid_result.result_code;
          HLOG_ERROR << "uid: " << uid_result.response;
        }
      }
      ++counter;
      const double sleep_time =
          kGetUUIDIntervalMs -
          (hozon::common::Clock::NowInMicroseconds() - start_time);
      if (sleep_time > 0) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>(sleep_time)));
      }
    }
    tsp_client.Deinit();
  }
}

void MapService::AMapProc() {
  while (true) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!uuid_.empty() && !is_amap_tsp_thread_stop_) {
        amap_adapter_->SetUUID(uuid_);
        is_amap_tsp_thread_stop_ = true;
      }
    }
    auto ins_ptr = INS_MANAGER->GetIns();
    if (ins_ptr != nullptr) {
      EhpProc(*ins_ptr, routing_);
      SetFautl();
    } else {
      HLOG_INFO << "ins_ptr is nullptr when load hd map";
    }
    usleep(1e6);
  }
}

void MapService::BaiduProc() {
  while (true) {
    auto ins_ptr = INS_MANAGER->GetIns();
    if (ins_ptr != nullptr) {
      INSPos ins_pos;
      ins_pos.x = ins_ptr->pos_gcj02().x();
      ins_pos.y = ins_ptr->pos_gcj02().y();
      baidu_map_->UpdateBaiDuMap(ins_pos);
      // hozon::hdmap::Map test_map;
      // GLOBAL_LD_MAP->GetMap(&test_map);
      // HLOG_ERROR << "ld  lane size :" << test_map.lane_size();
    } else {
      HLOG_WARN << "ins_ptr is nullptr when load ld map";
    }
    // hozon::hdmap::Map hd_map;
    // GLOBAL_HD_MAP->GetMap(&hd_map);
    // HLOG_ERROR << "hd lane size :" << hd_map.lane_size();
    usleep(1e6);
  }
}

bool MapService::EhpProc(
    const hozon::localization::HafNodeInfo& ins_msg,
    const std::shared_ptr<hozon::routing::RoutingResponse>& routing) {
  std::vector<EhpData> ehp_data_list;
  amap_adapter_->Process(ins_msg, &ehp_data_list);
  hozon::ehp::EHP message;
  int receive_counter(0);
  hozon::hdmap::Map deleted_map;
  bool reset_map(false);

  hozon::hdmap::Map extend_map;
  std::string current_pathid;
  std::string to_pathid;
  auto x = ins_msg.pos_gcj02().x();
  auto y = ins_msg.pos_gcj02().y();
  hozon::common::coordinate_convertor::GCS2UTM(51, &y, &x);
  hozon::common::PointENU utm_pos;
  utm_pos.set_x(y);
  utm_pos.set_y(x);
  utm_pos.set_z(0.0);

  SetCurrentPathId(utm_pos, &current_pathid);

  if (!ehp_data_list.empty()) {
    for (const auto& one_ehp_data : ehp_data_list) {
      auto* ehp_data = message.add_ehp_data();
      ehp_data->set_send_counter(one_ehp_data.first);

      for (const auto& one_data : one_ehp_data.second) {
        ehp_data->add_data(one_data);
      }
    }
    hozon::common::util::FillHeader("ehp_msg", &message);
  }
  ehr_->CollectData(message, &receive_counter, &deleted_map, &reset_map);
  ehr_->StartBuild(&extend_map, routing.get(), current_pathid, to_pathid, 51);
  if (reset_map) {
    auto hd_map = std::make_shared<hdmap::HDMap>();
    hozon::mp::GlobalHdMap::Instance()->ResetHdMap(hd_map);
  }
  std::list<hozon::hdmap::Map> extended_map_protos;
  std::list<hozon::hdmap::Map> shrinked_map_protos;
  if (extend_map.lane_size() > 0) {
    extended_map_protos.emplace_back(extend_map);
  }
  if (deleted_map.road_size() > 0) {
    shrinked_map_protos.emplace_back(deleted_map);
  }
  if (!extended_map_protos.empty() || !shrinked_map_protos.empty()) {
    GLOBAL_HD_MAP->UpdateMapFromProto(extended_map_protos, shrinked_map_protos);
  }

  return true;
}

void MapService::SetCurrentPathId(const hozon::common::PointENU& utm_pos,
                                  std::string* current_pathid) {
  static std::unordered_set<hozon::hdmap::Id, IdHash, IdEqual> last_laneid_pool;
  static std::unordered_set<std::string> last_roadid_pool;
  hozon::hdmap::LaneInfoConstPtr nearst_lane = nullptr;
  double s(0.0);
  double l(0.0);
  GLOBAL_HD_MAP->GetNearestLane(utm_pos, &nearst_lane, &s, &l);
  if (nearst_lane != nullptr && l < 5.0) {
    if (!last_laneid_pool.empty()) {
      if (last_laneid_pool.count(nearst_lane->id()) == 0) {
        std::vector<hdmap::LaneInfoConstPtr> lanes;
        GLOBAL_HD_MAP->GetLanes(utm_pos, 5, &lanes);
        if (lanes.empty()) {
          current_pathid->clear();
          last_laneid_pool.clear();
          last_roadid_pool.clear();
        } else if (lanes.size() == 1) {
          *current_pathid = lanes.front()->road_id().id();
          SetLaneIdsPool(lanes.front(), &last_laneid_pool, &last_roadid_pool);
        } else if (lanes.size() >= 2) {
          common::math::Vec2d point(utm_pos.x(), utm_pos.y());
          std::sort(lanes.begin(), lanes.end(),
                    [&](const auto& u, const auto& v) {
                      return u->DistanceTo(point) < v->DistanceTo(point);
                    });
          if (lanes.at(0)->road_id().id() == lanes.at(1)->road_id().id()) {
            *current_pathid = lanes.at(0)->road_id().id();
            SetLaneIdsPool(lanes.at(0), &last_laneid_pool, &last_roadid_pool);
          } else {
            if (last_roadid_pool.count(lanes.at(0)->road_id().id()) != 0) {
              *current_pathid = lanes.at(0)->road_id().id();
              SetLaneIdsPool(lanes.at(0), &last_laneid_pool, &last_roadid_pool);
            } else if (last_roadid_pool.count(lanes.at(1)->road_id().id()) !=
                       0) {
              *current_pathid = lanes.at(1)->road_id().id();
              SetLaneIdsPool(lanes.at(1), &last_laneid_pool, &last_roadid_pool);
            } else {
              current_pathid->clear();
              last_laneid_pool.clear();
              last_roadid_pool.clear();
            }
          }
        }
      } else {
        *current_pathid = nearst_lane->road_id().id();
        SetLaneIdsPool(nearst_lane, &last_laneid_pool, &last_roadid_pool);
      }
    } else {
      *current_pathid = nearst_lane->road_id().id();
      SetLaneIdsPool(nearst_lane, &last_laneid_pool, &last_roadid_pool);
    }
  } else {
    current_pathid->clear();
    last_laneid_pool.clear();
    last_roadid_pool.clear();
  }
}

void MapService::SetLaneIdsPool(
    const hozon::hdmap::LaneInfoConstPtr& current_lane,
    std::unordered_set<hozon::hdmap::Id, IdHash, IdEqual>* lane_id_pool,
    std::unordered_set<std::string>* last_roadid_pool) {
  const int kSearchLevel = 2;
  lane_id_pool->clear();
  lane_id_pool->insert(current_lane->id());
  GetAroundId(current_lane->lane().left_neighbor_forward_lane_id(),
              lane_id_pool);

  GetAroundId(current_lane->lane().right_neighbor_forward_lane_id(),
              lane_id_pool);
  last_roadid_pool->insert(current_lane->road_id().id());
  int i = 0;
  std::unordered_set<hozon::hdmap::Id, IdHash, IdEqual> current_level(
      *lane_id_pool);
  while (i < kSearchLevel) {
    std::unordered_set<hozon::hdmap::Id, IdHash, IdEqual> tmp;
    for (const auto& lane_id : current_level) {
      auto lane_prt = GLOBAL_HD_MAP->GetLaneById(lane_id);
      if (lane_prt != nullptr) {
        last_roadid_pool->insert(lane_prt->road_id().id());
        GetAroundId(lane_prt->lane().successor_id(), lane_id_pool);
        GetAroundId(lane_prt->lane().successor_id(), &tmp);
      }
    }
    current_level = tmp;
    i++;
  }
}

void MapService::GetAroundId(
    const ::google::protobuf::RepeatedPtrField<::hozon::hdmap::Id>& ids,
    std::unordered_set<hozon::hdmap::Id, IdHash, IdEqual>* lane_id_pool) {
  for (const auto& id : ids) {
    lane_id_pool->insert(id);
  }
}

void MapService::SetFautl() {
  std::lock_guard<std::mutex> lock(fault_mutex_);
  if (!amap_adapter_->GetInitState()) {
    fault_ = MS_EHP_INIT_ERROR;
    return;
  }
  ::hdmap::service::DiagnosisState state(
      ::hdmap::service::DIAGNOSIS_STATUS_OFF);
  auto type(static_cast<::hdmap::service::DiagnosisType>(0));
  std::string detailMessage;
  amap_adapter_->GetDiagnose(&state, &type, &detailMessage);

  if (static_cast<::hdmap::service::DiagnosisType>(0) == type) {
    fault_ = MS_NO_ERROR;
    return;
  }

  if (::hdmap::service::DIAGNOSIS_TYPE_SYSTEM_FAULT == type) {
    fault_ = MS_SDK_INNER_ERROR;
  } else if (::hdmap::service::DIAGNOSIS_TYPE_UID_FAULT == type) {
    fault_ = MS_UUID_ERROR;
  } else if (::hdmap::service::DIAGNOSIS_TYPE_ROM_ERROR == type) {
    fault_ = MS_MAP_DATA_ROM_ERROR;
  } else if (::hdmap::service::DIAGNOSIS_TYPE_PATH_ERROR == type) {
    fault_ = MS_MAP_DATA_PATH_ERROR;
  } else if (::hdmap::service::DIAGNOSIS_TYPE_FILE_AUTH_FAULT == type) {
    fault_ = MS_PATH_RW_ERROR;
  } else if (::hdmap::service::DIAGNOSIS_TYPE_HDMAP_FAULT == type) {
    fault_ = MS_SDK_INIT_FAIL;
  } else if (::hdmap::service::DIAGNOSIS_TYPE_ACTIVATION_FAULT == type) {
    fault_ = MS_SDK_ACTIVE_ERROR;
  } else if (::hdmap::service::DIAGNOSIS_TYPE_POS_SIGNAL_FAULT == type ||
             ::hdmap::service::DIAGNOSIS_TYPE_GNSS_LOST_FAULT == type ||
             ::hdmap::service::DIAGNOSIS_TYPE_GNSS_INVALID_FAULT == type) {
    fault_ = MS_INPUT_LOC_ERROR;
  } else {
    fault_ = MS_NO_ERROR;
  }
  // HLOG_INFO << "GD:" << static_cast<int>(type) << " HZ"
  //           << static_cast<int>(fault_);
}

MapServiceFault MapService::GetFault() {
  std::lock_guard<std::mutex> lock(fault_mutex_);
  return fault_;
}

MapService::~MapService() {
  is_amap_tsp_thread_stop_ = true;
  amap_tsp_proc_.get();
  if (ld_thread_.joinable()) {
    ld_thread_.join();
  }
  if (hd_thread_.joinable()) {
    hd_thread_.join();
  }
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
