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
#include <vector>

#include "amap/GHDMapDefines.h"
#include "amap/GHDMapService.h"
#include "common/adapters/adapter_gflags.h"
#include "common/configs/config_gflags.h"
#include "common/file/file.h"
#include "common/math/vec2d.h"
#include "common/time/clock.h"
#include "map/hdmap/hdmap.h"
#include "map/hdmap/hdmap_util.h"
#include "map_fusion/map_service/global_hd_map.h"
#include "proto/localization/node_info.pb.h"
#include "proto/map/ehp.pb.h"
#include "proto/map/map.pb.h"
#include "proto/routing/routing.pb.h"
#include "util/temp_log.h"

DEFINE_double(radius, 500, "radius of the vehicle position");
DEFINE_double(transform_distance, 200, "distance to update the map");

namespace hozon {
namespace mp {
namespace mf {

bool MapService::Init() {
  amap_adapter_.Init();
  amap_adapter_.SetUUID("LUZHRQASDHNVQCQNIH4CLM5OTA7VQ489");
  routing_ = std::make_shared<hozon::routing::RoutingResponse>();
  auto global_hd_map = hozon::mp::GlobalHdMap::Instance();

  if (FLAGS_map_service_mode == 0) {
    global_hd_map->GetHdMap()->LoadMapFromFile(FLAGS_map_dir + "/base_map.bin");
  } else if (FLAGS_map_service_mode == 1) {
    ehr_ = std::make_unique<hozon::ehr::AmapEhrImpl>();
  } else if (FLAGS_map_service_mode == 2) {
    // todo map api
  }

  return true;
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
    EhpProc(ins_msg, adc_msg, routing_);
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
  extended_map_protos.emplace_back(extend_map);
  shrinked_map_protos.emplace_back(deleted_map);

  GLOBAL_HD_MAP->UpdateMapFromProto(extended_map_protos, shrinked_map_protos);
  return true;
}

// bool MapService::BinProc(const hozon::localization::HafNodeInfo& ins_msg,
//                          const std::shared_ptr<hozon::hdmap::Map>& map) {
//   double x = ins_msg.pos_gcj02().y();
//   double y = ins_msg.pos_gcj02().x();
//   hozon::common::coordinate_convertor::GCS2UTM(51, &x, &y);

//   last_send_time_ = std::chrono::steady_clock::now();
//   hozon::common::math::Vec2d loc{x, y};
//   double distance = loc.DistanceTo(last_pose_);
//   HLOG_DEBUG << "distance: " << distance;

//   if (distance >= FLAGS_transform_distance) {
//     map->Clear();
//     hozon::common::PointENU enupos;
//     enupos.set_x(x);
//     enupos.set_y(y);
//     enupos.set_z(0);

//     std::vector<hozon::hdmap::JunctionInfoConstPtr> junctions;
//     int result = hd_map_->GetJunctions(enupos, FLAGS_radius, &junctions);
//     if (result == 0) {
//       for (const auto& junction : junctions) {
//         *map->add_junction() = junction->junction();
//       }
//     } else {
//       HLOG_DEBUG << "get junctions failed";
//     }

//     std::vector<hozon::hdmap::RoadInfoConstPtr> roads;
//     result = hd_map_->GetRoads(enupos, FLAGS_radius, &roads);
//     if (result == 0) {
//       for (const auto& road : roads) {
//         *map->add_road() = road->road();
//       }
//     } else {
//       HLOG_DEBUG << "get roads failed";
//     }
//     std::vector<hozon::hdmap::SignalInfoConstPtr> signals;
//     result = hd_map_->GetSignals(enupos, FLAGS_radius, &signals);
//     if (result == 0) {
//       for (const auto& signal : signals) {
//         *map->add_signal() = signal->signal();
//       }
//     } else {
//       HLOG_DEBUG << "get signals failed";
//     }

//     std::vector<hozon::hdmap::LaneInfoConstPtr> lanes;
//     result = hd_map_->GetLanes(enupos, FLAGS_radius, &lanes);
//     if (result == 0) {
//       for (const auto& lane : lanes) {
//         *map->add_lane() = lane->lane();
//       }
//     } else {
//       HLOG_DEBUG << "get lanes failed";
//     }

//     std::vector<hozon::hdmap::CrosswalkInfoConstPtr> crosswalks;
//     result = hd_map_->GetCrosswalks(enupos, FLAGS_radius, &crosswalks);
//     if (result == 0) {
//       for (const auto& crosswalk : crosswalks) {
//         *map->add_crosswalk() = crosswalk->crosswalk();
//       }
//     } else {
//       HLOG_DEBUG << "get crosswalks failed";
//     }

//     std::vector<hozon::hdmap::StopSignInfoConstPtr> stop_signs;
//     result = hd_map_->GetStopSigns(enupos, FLAGS_radius, &stop_signs);
//     if (result == 0) {
//       for (const auto& stopsign : stop_signs) {
//         *map->add_stop_sign() = stopsign->stop_sign();
//       }
//     } else {
//       HLOG_DEBUG << "get stop_signs failed";
//     }

//     std::vector<hozon::hdmap::YieldSignInfoConstPtr> yield_signs;
//     result = hd_map_->GetYieldSigns(enupos, FLAGS_radius, &yield_signs);
//     if (result == 0) {
//       for (const auto& yieldsign : yield_signs) {
//         *map->add_yield() = yieldsign->yield_sign();
//       }
//     } else {
//       HLOG_DEBUG << "get yield_signs failed";
//     }

//     std::vector<hozon::hdmap::ClearAreaInfoConstPtr> clear_areas;
//     result = hd_map_->GetClearAreas(enupos, FLAGS_radius, &clear_areas);
//     if (result == 0) {
//       for (const auto& cleararea : clear_areas) {
//         *map->add_clear_area() = cleararea->clear_area();
//       }
//     } else {
//       HLOG_DEBUG << "get clear_areas failed";
//     }

//     std::vector<hozon::hdmap::SpeedBumpInfoConstPtr> speed_bumps;
//     result = hd_map_->GetSpeedBumps(enupos, FLAGS_radius, &speed_bumps);
//     if (result == 0) {
//       for (const auto& speedbump : speed_bumps) {
//         *map->add_speed_bump() = speedbump->speed_bump();
//       }
//     } else {
//       HLOG_DEBUG << "get speed_bumps failed";
//     }

//     // 更新位置
//     last_pose_ = loc;
//   }

//   // if (prior_writer_) {
//   //   prior_writer_->Write(crop_map_);
//   // }
//   return true;
// }

}  // namespace mf
}  // namespace mp
}  // namespace hozon
