/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： road_recognition.cc
 *   author     ： zhangzhike
 *   date       ： 2023.12
 ******************************************************************************/

#include "map_fusion/road_recognition/road_recognition.h"
#include <gflags/gflags.h>

#include <limits>

#include "depend/common/utm_projection/coordinate_convertor.h"
#include "map_fusion/map_service/global_hd_map.h"
#include "map_fusion/road_recognition/geo_optimization.h"
#include "map_fusion/road_recognition/routing_generation.h"
#include "map_fusion/road_recognition/topo_generation.h"
#include "util/mapping_log.h"
#include "util/tic_toc.h"

namespace hozon {
namespace mp {
namespace mf {

bool RoadRecognition::Init(const YAML::Node& conf) {
  geo_ = std::make_shared<GeoOptimization>();
  geo_->Init();

  if (!conf["TopoGeneration"].IsDefined()) {
    HLOG_ERROR << "no config for TopoGeneration";
    return false;
  }
  topo_ = std::make_shared<TopoGeneration>();
  if (!topo_->Init(conf["TopoGeneration"])) {
    HLOG_ERROR << "TopoGeneration Init failed";
    return false;
  }
  rout_ = std::make_shared<RoutingGeneration>();
  rout_->Init();

  return true;
}

void RoadRecognition::OnLocalization(
    const std::shared_ptr<hozon::localization::Localization>& msg) {
  if (geo_ == nullptr) {
    return;
  }

  geo_->OnLocalization(msg);
  topo_->OnLocalization(msg);
}

void RoadRecognition::OnLocalMap(
    const std::shared_ptr<hozon::mapping::LocalMap>& msg,
    const std::shared_ptr<hozon::perception::PerceptionObstacles>& obj_msg) {
  if (geo_ == nullptr) {
    return;
  }

  geo_->OnLocalMap(msg, obj_msg);
  auto ele = geo_->GetElemMap();
  topo_->OnElementMap(ele);
  percep_map_ = topo_->GetPercepMap();
  ele_map_ = topo_->GetEleMap();
  // // 需要topo_给到map.proto形式的数据
  AddRoadEdge(msg, percep_map_);  // 将road edge透传给下游
  rout_->Generate(percep_map_);
  routingresponse_ = rout_->GetRouting();
}

void RoadRecognition::AddRoadEdge(
    const std::shared_ptr<hozon::mapping::LocalMap>& msg,
    const std::shared_ptr<hozon::hdmap::Map>& percep_map) {
  if (msg == nullptr || percep_map == nullptr) {
    return;
  }
  if (percep_map->road_size() > 0 && percep_map->road(0).section_size() > 0 &&
      msg->road_edges_size() > 0) {
    auto* first_section = percep_map->mutable_road(0)->mutable_section(0);
    for (const auto& local_road_edge : msg->road_edges()) {
      if (local_road_edge.points().empty()) {
        continue;
      }
      if (static_cast<int>(local_road_edge.lanepos()) != -1 &&
          static_cast<int>(local_road_edge.lanepos()) != 1) {
        continue;
      }
      auto* road_edge = first_section->mutable_boundary()
                            ->mutable_outer_polygon()
                            ->add_edge();
      road_edge->mutable_id()->set_id(
          std::to_string(local_road_edge.track_id()));
      if (static_cast<int>(local_road_edge.lanepos()) == -1) {
        road_edge->set_type(
            ::hozon::hdmap::BoundaryEdge_Type::BoundaryEdge_Type_LEFT_BOUNDARY);
      } else {
        road_edge->set_type(::hozon::hdmap::BoundaryEdge_Type::
                                BoundaryEdge_Type_RIGHT_BOUNDARY);
      }
      auto* segment =
          road_edge->mutable_curve()->add_segment()->mutable_line_segment();
      for (const auto& point : local_road_edge.points()) {
        if (std::isnan(point.x()) || std::isnan(point.y()) ||
            std::isnan(point.z())) {
          continue;
        }
        auto* point_proto = segment->add_point();
        point_proto->set_x(point.x());
        point_proto->set_y(point.y());
        point_proto->set_z(point.z());
      }
    }
  }
}

std::shared_ptr<hozon::hdmap::Map> RoadRecognition::GetPercepMap() {
  return percep_map_;
}

std::shared_ptr<hozon::routing::RoutingResponse> RoadRecognition::GetRouting() {
  // std::cout << "TBD " << __FUNCTION__ << std::endl;
  // return nullptr;
  // HLOG_ERROR<<"get routingrespose";
  return routingresponse_;
}

std::shared_ptr<hozon::mp::mf::em::ElementMapOut>
RoadRecognition::GetElementMap() {
  return ele_map_;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
