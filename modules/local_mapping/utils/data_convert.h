/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: shenliangchen
 *Date: 2023-09-05
 *****************************************************************************/
#pragma once

#include <algorithm>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "depend/proto/local_mapping/local_map.pb.h"
#include "depend/proto/localization/node_info.pb.h"
#include "depend/proto/map/map.pb.h"
#include "depend/proto/map/navigation.pb.h"
#include "depend/proto/perception/perception_measurement.pb.h"
#include "modules/local_mapping/types/types.h"
#include "modules/local_mapping/utils/common.h"
#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace lm {
class DataConvert {
 public:
  DataConvert() = default;

  static void SetObj(const hozon::perception::measurement::MeasurementPb& msg,
                     std::shared_ptr<Objects> objects);

  static void SetLocalization(const hozon::localization::Localization& msg,
                              Localization* localization);

  static void SetPerception(const hozon::perception::TransportElement& msg,
                            Perception* perception);

  static void SetIns(const hozon::localization::HafNodeInfo& msg, InsData* ins);

  static void SetLaneLine(const hozon::perception::TransportElement& msg,
                          std::vector<LaneLine>* lane_lines);

  static void SetRoadEdge(const hozon::perception::TransportElement& msg,
                          std::vector<RoadEdge>* road_edges);

  static void SetStopLine(const hozon::perception::TransportElement& msg,
                          std::vector<StopLine>* stop_lines);

  static void SetArrow(const hozon::perception::TransportElement& msg,
                       std::vector<Arrow>* arrows);

  static void SetZebraCrossing(const hozon::perception::TransportElement& msg,
                               std::vector<ZebraCrossing>* zebra_crossing);

  static void ConvertProtoLanePos(
      const hozon::perception::LanePositionType& raw_lanepos,
      LanePositionType* lanepos);

  static void ConvertProtoLaneType(
      const hozon::perception::LaneType& raw_lanetype, LaneType* lanetype);

  static void ConvertProtoEdgeType(
      const hozon::perception::RoadEdge::RoadEdgeType& raw_edgetype,
      EdgeType* edgetype);

  static void ConvertProtoArrowType(
      const hozon::perception::ArrowType& raw_arrowtype, ArrowType* arrowtype);

  static void ConvertProtoColor(const hozon::perception::Color& raw_color,
                                Color* color);

  static void ConvertInnerLanePos(const LanePositionType& inner_lanepos,
                                  hozon::mapping::LanePositionType* lanepos);

  static void ConvertInnerLaneType(const LaneType& inner_lanetype,
                                   hozon::mapping::LaneType* lanetype);

  static void ConvertInnerColor(const Color& inner_color,
                                hozon::mapping::Color* color);

  static void ConvertInnerMapLaneType(
      const Color& color, const bool& is_left, const LaneType& inner_lanetype,
      hozon::hdmap::LaneBoundaryType::Type* lanetype);

  static void ConvertInnerArrowType(const ArrowType& inner_arrowtype,
                                    hozon::hdmap::ArrowData::Type* arrowtype);

  static void ConvertMultiLaneLinesToPb(
      const std::vector<LaneLine>& lane_lines,
      const std::shared_ptr<hozon::mapping::LocalMap>& localmap);

  static void ConvertMultiRoadEdgesToPb(
      const std::vector<RoadEdge>& road_edges,
      const std::shared_ptr<hozon::mapping::LocalMap>& localmap);

  static void ConvertMultiStopLinesToPb(
      const std::vector<StopLine>& stop_lines,
      const std::shared_ptr<hozon::mapping::LocalMap>& localmap);

  static void ConvertMultiArrowsToPb(
      const std::vector<Arrow>& arrows,
      const std::shared_ptr<hozon::mapping::LocalMap>& localmap);

  static void ConvertMultiZebraCrossingsToPb(
      const std::vector<ZebraCrossing>& zebra_crossings,
      const std::shared_ptr<hozon::mapping::LocalMap>& localmap);
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
