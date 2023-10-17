/*
 * Copyright (c) Hozon Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  ehp
 */
#include "onboard/onboard_cyber/map_fusion/map_service_component.h"

#include <cstdlib>
#include <iomanip>
#include <list>
#include <string>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "amap/GHDMapDefines.h"
#include "amap/GHDMapService.h"
#include "common/adapters/adapter_gflags.h"
#include "common/configs/config_gflags.h"
#include "common/file/file.h"
#include "common/math/vec2d.h"
#include "common/time/clock.h"
#include "map/hdmap/hdmap_util.h"
#include "proto/localization/node_info.pb.h"
#include "proto/map/ehp.pb.h"
#include "proto/map/map.pb.h"
#include "proto/routing/routing.pb.h"

DEFINE_string(channel_prior, "mf/prior_map",
              "channel of prior map from prior provider");
DEFINE_string(channel_ins_node_info, "/PluginNodeInfo",
              "channel of ins msg from ins fusion");
DEFINE_double(radius, 500, "radius of the vehicle position");
DEFINE_double(transform_distance, 200, "distance to update the map");

namespace hozon {
namespace mp {
namespace mf {

using hozon::planning::ADCTrajectory;

MapServiceComponent::MapServiceComponent() = default;

bool MapServiceComponent::Init() {
  ins_reader_ = node_->CreateReader<hozon::localization::HafNodeInfo>(
      FLAGS_channel_ins_node_info);
  amap_adapter_.Init();
  // apollo::cyber::ReaderConfig localization_reader_config;
  // localization_reader_config.channel_name = FLAGS_localization_topic;  //
  // NOLINT localization_reader_config.pending_queue_size = 10;

  // localization_reader_ = node_->CreateReader<Localization>(
  //     localization_reader_config, nullptr);
  // ACHECK(localization_reader_ != nullptr);

  if (FLAGS_map_service_mode == 0) {
    hd_map_ = hozon::hdmap::HDMapUtil::LoadLocalMapPtr();
  } else if (FLAGS_map_service_mode == 1) {
    hd_map_ = std::make_shared<hdmap::HDMap>();
    apollo::cyber::ReaderConfig planning_reader_config;
    planning_reader_config.channel_name =
        FLAGS_planning_trajectory_topic;  // NOLINT
    planning_reader_config.pending_queue_size = 10;

    planning_reader_ =
        node_->CreateReader<ADCTrajectory>(planning_reader_config, nullptr);
    ACHECK(planning_reader_ != nullptr);

    ehr_ = std::make_unique<hozon::ehr::AmapEhrImpl>();
  } else if (FLAGS_map_service_mode == 2) {
    // todo map api
  }

  prior_writer_ =
      node_->CreateWriter<hozon::hdmap::Map>(FLAGS_channel_prior);  // NOLINT
  ACHECK(prior_writer_ != nullptr);
  routing_response_writer_ =
      node_->CreateWriter<hozon::routing::RoutingResponse>(
          FLAGS_routing_response_topic);
  ACHECK(routing_response_writer_ != nullptr);

  return true;
}

bool MapServiceComponent::Proc() {
  // localization_reader_->Observe();
  // const auto& localization_msg = localization_reader_->GetLatestObserved();
  // if (localization_msg == nullptr &&
  //     !FLAGS_enable_planning_self_simulator) {             // NOLINT
  //   HLOG_ERROR_EVERY(10) << "localization msg is not ready!";  // NOLINT
  //   return false;
  // }
  ins_reader_->Observe();
  const auto& localization_msg = ins_reader_->GetLatestObserved();
  if (localization_msg != nullptr) {
    latest_localization_ = *localization_msg;
  } else {
    HLOG_ERROR << "loc isnull";
    return false;
  }
  hozon::hdmap::Map map;
  hozon::routing::RoutingResponse routing;

  if (FLAGS_map_service_mode == 0) {
    BinProc(&map);

    if (!map.lane().empty()) {
      prior_writer_->Write(map);
      HLOG_DEBUG << "crop map lane size:" << map.lane_size();
    }

  } else if (FLAGS_map_service_mode == 1) {
    EhpProc(&routing);
    hd_map_->GetMap(&map);
    prior_writer_->Write(map);
    routing_response_writer_->Write(routing);
  }

  return true;
}

bool MapServiceComponent::EhpProc(
    hozon::routing::RoutingResponse* const routing) {
  planning_reader_->Observe();
  const auto& planning_msg = planning_reader_->GetLatestObserved();
  if (planning_msg == nullptr || !planning_msg->has_header()) {
    if (FLAGS_ehp_monitor == 2 || FLAGS_ehp_monitor == 0) {   // NOLINT
      AERROR_EVERY(10) << "ADCTrajectory msg is not ready!";  // NOLINT
      return false;
    }
  } else {
    latest_planning_ = *planning_msg;
  }

  std::vector<EhpData> ehp_data_list;

  // hozon::localization::HafNodeInfo tem_loc(latest_localization_);
  // auto* gcj = tem_loc.mutable_pos_gcj02();
  // gcj->set_x(121.175229170);
  // gcj->set_y(31.258064753);
  // tem_loc.set_heading(2.537517423);

  amap_adapter_.Process(latest_localization_, latest_planning_, &ehp_data_list);

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
  ehr_->StartBuild(&extend_map, routing, current_pathid, to_pathid, 51);
  if (reset_map) {
    hd_map_ = std::make_shared<hdmap::HDMap>();
  }
  std::list<hozon::hdmap::Map> extended_map_protos;
  std::list<hozon::hdmap::Map> shrinked_map_protos;
  extended_map_protos.emplace_back(extend_map);
  shrinked_map_protos.emplace_back(deleted_map);

  hd_map_->UpdateMapFromProto(extended_map_protos, shrinked_map_protos);
  return true;
}

bool MapServiceComponent::BinProc(hozon::hdmap::Map* const map) {
  double x = latest_localization_.pos_gcj02().y();
  double y = latest_localization_.pos_gcj02().x();
  hozon::common::coordinate_convertor::GCS2UTM(51, &x, &y);

  last_send_time_ = std::chrono::steady_clock::now();
  hozon::common::math::Vec2d loc{x, y};
  double distance = loc.DistanceTo(last_pose_);
  HLOG_DEBUG << "distance: " << distance;

  if (distance >= FLAGS_transform_distance) {
    map->Clear();
    hozon::common::PointENU enupos;
    enupos.set_x(x);
    enupos.set_y(y);
    enupos.set_z(0);

    std::vector<hozon::hdmap::JunctionInfoConstPtr> junctions;
    int result = hd_map_->GetJunctions(enupos, FLAGS_radius, &junctions);
    if (result == 0) {
      for (const auto& junction : junctions) {
        *map->add_junction() = junction->junction();
      }
    } else {
      HLOG_DEBUG << "get junctions failed";
    }

    std::vector<hozon::hdmap::RoadInfoConstPtr> roads;
    result = hd_map_->GetRoads(enupos, FLAGS_radius, &roads);
    if (result == 0) {
      for (const auto& road : roads) {
        *map->add_road() = road->road();
      }
    } else {
      HLOG_DEBUG << "get roads failed";
    }
    std::vector<hozon::hdmap::SignalInfoConstPtr> signals;
    result = hd_map_->GetSignals(enupos, FLAGS_radius, &signals);
    if (result == 0) {
      for (const auto& signal : signals) {
        *map->add_signal() = signal->signal();
      }
    } else {
      HLOG_DEBUG << "get signals failed";
    }

    std::vector<hozon::hdmap::LaneInfoConstPtr> lanes;
    result = hd_map_->GetLanes(enupos, FLAGS_radius, &lanes);
    if (result == 0) {
      for (const auto& lane : lanes) {
        *map->add_lane() = lane->lane();
      }
    } else {
      HLOG_DEBUG << "get lanes failed";
    }

    std::vector<hozon::hdmap::CrosswalkInfoConstPtr> crosswalks;
    result = hd_map_->GetCrosswalks(enupos, FLAGS_radius, &crosswalks);
    if (result == 0) {
      for (const auto& crosswalk : crosswalks) {
        *map->add_crosswalk() = crosswalk->crosswalk();
      }
    } else {
      HLOG_DEBUG << "get crosswalks failed";
    }

    std::vector<hozon::hdmap::StopSignInfoConstPtr> stop_signs;
    result = hd_map_->GetStopSigns(enupos, FLAGS_radius, &stop_signs);
    if (result == 0) {
      for (const auto& stopsign : stop_signs) {
        *map->add_stop_sign() = stopsign->stop_sign();
      }
    } else {
      HLOG_DEBUG << "get stop_signs failed";
    }

    std::vector<hozon::hdmap::YieldSignInfoConstPtr> yield_signs;
    result = hd_map_->GetYieldSigns(enupos, FLAGS_radius, &yield_signs);
    if (result == 0) {
      for (const auto& yieldsign : yield_signs) {
        *map->add_yield() = yieldsign->yield_sign();
      }
    } else {
      HLOG_DEBUG << "get yield_signs failed";
    }

    std::vector<hozon::hdmap::ClearAreaInfoConstPtr> clear_areas;
    result = hd_map_->GetClearAreas(enupos, FLAGS_radius, &clear_areas);
    if (result == 0) {
      for (const auto& cleararea : clear_areas) {
        *map->add_clear_area() = cleararea->clear_area();
      }
    } else {
      HLOG_DEBUG << "get clear_areas failed";
    }

    std::vector<hozon::hdmap::SpeedBumpInfoConstPtr> speed_bumps;
    result = hd_map_->GetSpeedBumps(enupos, FLAGS_radius, &speed_bumps);
    if (result == 0) {
      for (const auto& speedbump : speed_bumps) {
        *map->add_speed_bump() = speedbump->speed_bump();
      }
    } else {
      HLOG_DEBUG << "get speed_bumps failed";
    }

    // 更新位置
    last_pose_ = loc;
  }

  // if (prior_writer_) {
  //   prior_writer_->Write(crop_map_);
  // }
  return true;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
