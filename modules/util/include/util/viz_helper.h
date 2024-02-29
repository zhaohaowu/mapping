/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： viz_helper.h
 *   author     ： taoshaoyuan
 *   date       ： 2024.02
 ******************************************************************************/
#pragma once

#include <adsfi_proto/viz/geometry_msgs.pb.h>
#include <adsfi_proto/viz/nav_msgs.pb.h>
#include <adsfi_proto/viz/sensor_msgs.pb.h>
#include <adsfi_proto/viz/tf2_msgs.pb.h>
#include <adsfi_proto/viz/visualization_msgs.pb.h>
#include <proto/local_mapping/local_map.pb.h>
#include <proto/localization/localization.pb.h>
#include <proto/map/map.pb.h>
#include <proto/map/navigation.pb.h>
#include <proto/perception/transport_element.pb.h>
#include <proto/soc/sensor_image.pb.h>

#include <memory>
#include <string>

#include <opencv2/opencv.hpp>

namespace hozon {
namespace mp {
namespace util {

static const char kFrameVehicle[] = "vehicle";
static const char kFrameLocal[] = "local";
static const char kFrameLocalEnu[] = "local_enu";

struct Rgb {
  float r = 0;
  float g = 0;
  float b = 0;
};

enum Color {
  WHITE = 0,
  GREY,
  BLACK,
  RED,
  ORANGE,
  YELLOW,
  GREEN,
  BLUE,
  CYAN,
  PURPLE,
};

Rgb ColorRgb(Color color);

void SplitStamp(double secs, uint32_t* sec, uint32_t* nsec);

std::shared_ptr<adsfi_proto::viz::CompressedImage> YUVNV12ImageToVizImage(
    const std::shared_ptr<hozon::soc::Image>& yuv_image, int quality,
    double resize_factor);

void MapCurveToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                       const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                       const std::string& ns,
                       const adsfi_proto::viz::ColorRGBA& rgba,
                       double line_width, const hozon::hdmap::Curve& curve,
                       adsfi_proto::viz::MarkerArray* ma, bool is_dashed);
void MapLaneInfoToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                          const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                          const std::string& ns,
                          const adsfi_proto::viz::ColorRGBA& rgba,
                          const hozon::hdmap::Lane& lane,
                          adsfi_proto::viz::MarkerArray* ma);
void MapLaneBoundaryToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                              const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                              const std::string& ns,
                              const adsfi_proto::viz::ColorRGBA& rgba,
                              const hozon::hdmap::LaneBoundary& boundary,
                              adsfi_proto::viz::MarkerArray* ma,
                              bool auto_color = false);
void MapLaneToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                      const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                      const hozon::hdmap::Lane& lane,
                      adsfi_proto::viz::MarkerArray* ma);
void MapRoadToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                      const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                      const hozon::hdmap::Road& road,
                      adsfi_proto::viz::MarkerArray* ma);
void MapCrosswalkToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                           const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                           const hozon::hdmap::Crosswalk& crosswalk,
                           adsfi_proto::viz::MarkerArray* ma);
void MapArrowToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                       const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                       const hozon::hdmap::ArrowData& arrow,
                       adsfi_proto::viz::MarkerArray* ma);
void MapStopLineToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                          const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                          const hozon::hdmap::StopLine& stop_line,
                          adsfi_proto::viz::MarkerArray* ma);
void MapToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                  const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                  const hozon::hdmap::Map& map,
                  adsfi_proto::viz::MarkerArray* ma);
void RoutingToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                      const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                      const hozon::routing::RoutingResponse& routing,
                      const hozon::hdmap::Map& map,
                      adsfi_proto::viz::MarkerArray* ma);
void MapMsgStatusToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                           const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                           const hozon::navigation_hdmap::MapMsg& map_msg,
                           adsfi_proto::viz::MarkerArray* ma);
void MapMsgToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                     const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                     const hozon::navigation_hdmap::MapMsg& map_msg,
                     adsfi_proto::viz::MarkerArray* map_ma,
                     adsfi_proto::viz::MarkerArray* routing_ma,
                     adsfi_proto::viz::MarkerArray* status_ma);

void TeLaneToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                     const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                     const hozon::perception::LaneInfo& lane,
                     const std::string& ns_prefix,
                     const adsfi_proto::viz::ColorRGBA& rgba,
                     adsfi_proto::viz::MarkerArray* ma);
void TeRoadEdgeToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                         const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                         const hozon::perception::RoadEdge& edge,
                         const std::string& ns_prefix,
                         const adsfi_proto::viz::ColorRGBA& rgba,
                         adsfi_proto::viz::MarkerArray* ma);
void TeArrowToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                      const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                      const hozon::perception::Arrow& arrow,
                      const std::string& ns_prefix,
                      const adsfi_proto::viz::ColorRGBA& rgba,
                      adsfi_proto::viz::MarkerArray* ma);
void TeStopLineToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                         const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                         const hozon::perception::StopLine& stop_line,
                         const std::string& ns_prefix,
                         const adsfi_proto::viz::ColorRGBA& rgba,
                         adsfi_proto::viz::MarkerArray* ma);
void TeCrossWalkToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                          const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                          const hozon::perception::ZebraCrossing& cross_walk,
                          const std::string& ns_prefix,
                          const adsfi_proto::viz::ColorRGBA& rgba,
                          adsfi_proto::viz::MarkerArray* ma);
void TransportElementToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                               const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                               const hozon::perception::TransportElement& te,
                               adsfi_proto::viz::MarkerArray* te_ma);

void LmLaneLineToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                         const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                         const hozon::mapping::LaneLine& line,
                         const std::string& ns_prefix,
                         const adsfi_proto::viz::ColorRGBA& rgba,
                         adsfi_proto::viz::MarkerArray* ma);
void LmStopLineToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                         const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                         const hozon::mapping::StopLine& stop_line,
                         const std::string& ns_prefix,
                         const adsfi_proto::viz::ColorRGBA& rgba,
                         adsfi_proto::viz::MarkerArray* ma);
void LmCrossWalkToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                          const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                          const hozon::mapping::CrossWalk& cross_walk,
                          const std::string& ns_prefix,
                          const adsfi_proto::viz::ColorRGBA& rgba,
                          adsfi_proto::viz::MarkerArray* ma);
void LmArrowToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                      const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                      const hozon::mapping::Arrow& arrow,
                      const std::string& ns_prefix,
                      const adsfi_proto::viz::ColorRGBA& rgba,
                      adsfi_proto::viz::MarkerArray* ma);
void LocalMapToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                       const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                       const hozon::mapping::LocalMap& local_map,
                       adsfi_proto::viz::MarkerArray* ma);

}  // namespace util
}  // namespace mp
}  // namespace hozon
