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

#include "base/scene/arrow.h"
#include "base/scene/zebra_crossing.h"
#include "depend/perception-base/base/frame/fusion_frame.h"
#include "depend/perception-base/base/frame/measurement_frame.h"
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

  static void SetLocalization(const hozon::localization::Localization& msg,
                              Localization* localization);

  static void SetPerceptionEnv(
      const std::shared_ptr<hozon::perception::base::FusionFrame>& msg,
      Perception* perception);
  static void SetPerceptionObj(
      const std::shared_ptr<hozon::perception::base::MeasurementFrame>& msg,
      Perception* perception);
  static void SetIns(const hozon::localization::HafNodeInfo& msg, InsData* ins);

  static void SetLaneLine(
      std::vector<hozon::perception::base::LaneLinePtr> perception_lanelines,
      std::vector<LaneLine>* lane_lines);

  static void SetRoadEdge(
      std::vector<hozon::perception::base::RoadEdgePtr> perception_roadedges,
      std::vector<RoadEdge>* road_edges);

  static void SetStopLine(
      std::vector<hozon::perception::base::StopLinePtr> perception_stoplines,
      std::vector<StopLine>* stop_lines);

  static void SetArrow(
      std::vector<hozon::perception::base::ArrowPtr> perception_arrows,
      std::vector<Arrow>* arrows);

  static void SetZebraCrossing(
      std::vector<hozon::perception::base::ZebraCrossingPtr>
          perception_zebracrosswalks,
      std::vector<ZebraCrossing>* zebra_crossing);

  static void ConvertStructLanePos(
      const hozon::perception::base::LaneLinePosition& raw_lanepos,
      LanePositionType* lanepos);

  static void ConvertProtoLanePos(
      const hozon::perception::LanePositionType& raw_lanepos,
      LanePositionType* lanepos);

  static void ConvertStructLaneType(
      const hozon::perception::base::LaneLineType& raw_lanetype,
      LaneType* lanetype);

  static void ConvertProtoLaneType(
      const hozon::perception::LaneType& raw_lanetype, LaneType* lanetype);

  static void ConvertStrutEdgeType(
      const hozon::perception::base::RoadEdgeType& raw_edgetype,
      EdgeType* edgetype);

  static void ConvertProtoEdgeType(
      const hozon::perception::RoadEdge::RoadEdgeType& raw_edgetype,
      EdgeType* edgetype);

  static void ConvertStructArrowType(
      const hozon::perception::base::ArrowType& raw_arrowtype,
      ArrowType* arrowtype);

  static void ConvertProtoArrowType(
      const hozon::perception::ArrowType& raw_arrowtype, ArrowType* arrowtype);

  static void ConvertStructColor(
      const hozon::perception::base::LaneLineColor& raw_color, Color* color);

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
