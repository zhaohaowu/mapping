/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： routing_generation.cc
 *   author     ： zhangzhike
 *   date       ： 2024.1
 ******************************************************************************/

#include "map_fusion/road_recognition/routing_generation.h"
#include <gflags/gflags.h>

#include <limits>

#include "depend/common/utm_projection/coordinate_convertor.h"
#include "map_fusion/fusion_common/calc_util.h"
#include "map_fusion/map_service/global_hd_map.h"
#include "map_fusion/road_recognition/geo_optimization.h"
#include "map_fusion/road_recognition/topo_generation.h"
#include "util/mapping_log.h"
#include "util/tic_toc.h"

namespace hozon {
namespace mp {
namespace mf {

int RoutingGeneration::Init() {
  routing_ = std::make_shared<hozon::routing::RoutingResponse>();
  percep_map_ = std::make_shared<hozon::hdmap::Map>();
  return 0;
}

template <typename T, typename std::enable_if<
                          std::is_base_of<google::protobuf::Message, T>::value,
                          int>::type = 0>
void RoutingGeneration::FillHeader(const std::string& module_name, T* msg) {
  static std::atomic<uint64_t> sequence_num = {0};
  auto* header = msg->mutable_header();
  double timestamp = ::hozon::common::Clock::NowInSeconds();
  header->set_frame_id(module_name);
  header->set_publish_stamp(timestamp);
  header->set_seq(static_cast<unsigned int>(sequence_num.fetch_add(1)));
}

void RoutingGeneration::SetRouting() {
  int lane_num = percep_map_->lane_size();
  if (lane_num < 1) {
    HLOG_ERROR << "empty lane in percep_map";
    return;
  }

  // <lane_id, lane_length>
  std::map<std::string, double> lane_length_hash;
  for (const auto& lane : percep_map_->lane()) {
    lane_length_hash.insert_or_assign(lane.id().id(), lane.length());
  }

  for (const auto& road : percep_map_->road()) {
    for (const auto& section : road.section()) {
      if (section.lane_id().empty()) {
        continue;
      }

      auto* routing_road = routing_->add_road();
      routing_road->set_id(section.id().id());
      routing_road->mutable_passage()->Reserve(section.lane_id_size());
      for (const auto& lane_id : section.lane_id()) {
        double length = 0;
        if (lane_length_hash.find(lane_id.id()) == lane_length_hash.end()) {
          continue;
        }
        length = lane_length_hash[lane_id.id()];
        auto* passage = routing_road->add_passage();
        passage->set_can_exit(true);
        passage->set_change_lane_type(routing::ChangeLaneType::FORWARD);
        auto* segment = passage->add_segment();
        segment->set_id(lane_id.id());
        segment->set_start_s(0.0);
        segment->set_end_s(length);
      }
    }
  }

  auto adc_lane_segment_points = percep_map_->lane(lane_num - 1)
                                     .central_curve()
                                     .segment()[0]
                                     .line_segment()
                                     .point();
  common::PointENU start_point = adc_lane_segment_points[0];
  int max_index = static_cast<int>(adc_lane_segment_points.size()) - 1;
  common::PointENU end_point = adc_lane_segment_points[max_index];
  auto* routing_request = routing_->mutable_routing_request();
  routing::LaneWaypoint waypoint;
  waypoint.set_id(percep_map_->lane(lane_num - 1).id().id());
  waypoint.mutable_pose()->set_x(start_point.x());
  waypoint.mutable_pose()->set_y(start_point.y());
  waypoint.set_s(0.0);
  routing_request->add_waypoint()->CopyFrom(waypoint);
  waypoint.set_s(percep_map_->lane(lane_num - 1).length());
  waypoint.mutable_pose()->set_x(end_point.x());
  waypoint.mutable_pose()->set_y(end_point.y());
  routing_request->add_waypoint()->CopyFrom(waypoint);

  // auto* routing_request = routing_->mutable_routing_request();
  FillHeader("from_adaptive_cruise_routingrequest", routing_request);
  FillHeader("from_adaptive_cruise_routing", &(*routing_));
}

void RoutingGeneration::Generate(
    const std::shared_ptr<hozon::hdmap::Map>& percep_map) {
  if (percep_map == nullptr) {
    HLOG_WARN << "get nullptr percep_map!";
    return;
  }
  routing_ = std::make_shared<hozon::routing::RoutingResponse>();
  // percep_map_ = std::make_shared<hozon::hdmap::Map>();
  // percep_map_->CopyFrom(*percep_map);

  // 不进行复制，直接传递指针进行效率优化。
  percep_map_ = percep_map;

  SetRouting();
}

std::shared_ptr<hozon::routing::RoutingResponse>
RoutingGeneration::GetRouting() {
  return routing_;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
