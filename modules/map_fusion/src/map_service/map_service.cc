/*
 * Copyright (c) Hozon Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  ehp
 */
#include "map_fusion/map_service/map_service.h"

#include <cstdlib>
#include <iomanip>
#include <list>
#include <memory>
#include <string>
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
#include "map/hdmap/hdmap.h"
#include "map/hdmap/hdmap_util.h"
#include "map_fusion/map_service/global_hd_map.h"
#include "proto/localization/node_info.pb.h"
#include "proto/map/ehp.pb.h"
#include "proto/map/map.pb.h"
#include "proto/map/map_id.pb.h"
#include "proto/routing/routing.pb.h"
#include "util/mapping_log.h"

// NOLINTBEGIN
DEFINE_double(radius, 500, "radius of the vehicle position");
DEFINE_double(transform_distance, 200, "distance to update the map");
// NOLINTEND

namespace hozon {
namespace mp {
namespace mf {

bool MapService::Init() {
  amap_adapter_.Init();

  routing_ = std::make_shared<hozon::routing::RoutingResponse>();
  auto* global_hd_map = hozon::mp::GlobalHdMap::Instance();

  if (FLAGS_map_service_mode == 0) {
    global_hd_map->GetHdMap()->LoadMapFromFile(FLAGS_map_dir + "/base_map.bin");
  } else if (FLAGS_map_service_mode == 1) {
    amap_tsp_proc_ = std::async(&MapService::GetUidThread, this);
    ehr_ = std::make_unique<hozon::ehr::AmapEhrImpl>();
  } else if (FLAGS_map_service_mode == 2) {
    // todo map api
  }

  return true;
}

void MapService::GetUidThread() {
  static constexpr double kGetUUIDIntervalMs = 1000;
  std::ifstream ifs;
  ifs.open("/hd_map/uid.txt", std::ios::in);
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
          std::ofstream ofs("/hd_map/uid.txt");
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

void MapService::OnInsAdcNodeInfo(
    const hozon::localization::HafNodeInfo& ins_msg,
    const hozon::planning::ADCTrajectory& adc_msg) {
  // double x = ins_msg.pos_gcj02().y();
  // double y = ins_msg.pos_gcj02().x();
  // hozon::common::coordinate_convertor::GCS2UTM(51, &x, &y);

  // hozon::common::PointENU enupos;
  // enupos.set_x(x);
  // enupos.set_y(y);
  // enupos.set_z(0);
  // std::vector<hozon::hdmap::LaneInfoConstPtr> lanes;

  if (FLAGS_map_service_mode == 0) {
    // GLOBAL_HD_MAP->GetLanes(enupos, 10, &lanes);
    // HLOG_ERROR << "mspppppppppplane size: " << lanes.size();
    // do nothing
    //  hd_map_ = hozon::hdmap::HDMapUtil::LoadLocalMapPtr();
    //  BinProc(ins_msg, cro_map_);
    //  if (!cro_map_->lane().empty()) {
    //    HLOG_DEBUG << "crop map lane size:" << cro_map_->lane_size();
    //  }
    //  if (!hozon::mp::GlobalHdMap::Instance()->SetHdMap(hd_map_)) {
    //    HLOG_ERROR << "hdmap set error";
    //  }
  } else if (FLAGS_map_service_mode == 1) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!uuid_.empty() && !is_amap_tsp_thread_stop_) {
        amap_adapter_.SetUUID(uuid_);
        is_amap_tsp_thread_stop_ = true;
      }
    }
    EhpProc(ins_msg, adc_msg, routing_);
    SetFautl();
  }
}

bool MapService::EhpProc(
    const hozon::localization::HafNodeInfo& ins_msg,
    const hozon::planning::ADCTrajectory& adc_msg,
    const std::shared_ptr<hozon::routing::RoutingResponse>& routing) {
  std::vector<EhpData> ehp_data_list;

  // hozon::localization::HafNodeInfo tem_loc(ins_msg);
  // auto* gcj = tem_loc.mutable_pos_gcj02();
  // gcj->set_x(31.258064753);
  // gcj->set_y(121.175229170);
  // tem_loc.set_heading(304.610);

  amap_adapter_.Process(ins_msg, adc_msg, &ehp_data_list);

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
  HLOG_DEBUG << "currentpathid: " << current_pathid;

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
  if (amap_adapter_.GetInitState()) {
    fault_ = MS_EHP_INIT_ERROR;
    return;
  }
  ::hdmap::service::DiagnosisState state(
      ::hdmap::service::DIAGNOSIS_STATUS_OFF);
  auto type(static_cast<::hdmap::service::DiagnosisType>(0));
  std::string detailMessage;
  amap_adapter_.GetDiagnose(&state, &type, &detailMessage);

  if (static_cast<::hdmap::service::DiagnosisType>(0) == type) {
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
  }
}

MapServiceFault MapService::GetFault() {
  std::lock_guard<std::mutex> lock(fault_mutex_);
  return fault_;
}

MapService::~MapService() {
  is_amap_tsp_thread_stop_ = true;
  amap_tsp_proc_.get();
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
