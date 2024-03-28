/********************************************************
 * Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 * Author: mq
 *******************************************************/

#pragma once
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "base/utils/log.h"
#include "modules/local_mapping/base/location/Ins.h"
#include "modules/local_mapping/base/location/location.h"
#include "modules/local_mapping/base/object/object.h"
#include "modules/local_mapping/base/scene/arrow.h"
#include "modules/local_mapping/base/scene/laneline.h"
#include "modules/local_mapping/base/scene/localmap.h"
#include "modules/local_mapping/base/scene/noparking.h"
#include "modules/local_mapping/base/scene/roadedge.h"
#include "modules/local_mapping/base/scene/slowdown.h"
#include "modules/local_mapping/base/scene/stopline.h"
#include "modules/local_mapping/base/scene/waitzone.h"
#include "modules/local_mapping/base/scene/zebracrossing.h"
#include "proto/dead_reckoning/dr.pb.h"
#include "proto/local_mapping/local_map.pb.h"
#include "proto/localization/localization.pb.h"
#include "proto/localization/node_info.pb.h"
#include "proto/perception/perception_measurement.pb.h"
#include "proto/perception/transport_element.pb.h"
#include "proto/soc/radar.pb.h"

namespace hozon {
namespace mp {
namespace lm {
namespace data_mapping {

#define NAME_SHARED_PTR(type)                         \
  using type##ConstPtr = std::shared_ptr<const type>; \
  using type##Ptr = std::shared_ptr<type>;

using NetaTransportElement = hozon::perception::TransportElement;
NAME_SHARED_PTR(NetaTransportElement)

using NetaDeadReckoning = hozon::dead_reckoning::DeadReckoning;
NAME_SHARED_PTR(NetaDeadReckoning)

using NetaLoaction = hozon::localization::Localization;
NAME_SHARED_PTR(NetaLoaction)

using NetaIns = hozon::localization::HafNodeInfo;
NAME_SHARED_PTR(NetaIns)

using NetaPerceptionObstacles = hozon::perception::PerceptionObstacles;
NAME_SHARED_PTR(NetaPerceptionObstacles)

using NetaPerceptionObstacle = hozon::perception::PerceptionObstacle;
NAME_SHARED_PTR(NetaPerceptionObstacle)

class DataMapping {
 public:
  DataMapping() = default;

  ~DataMapping() = default;

  // 感知pb数据到内部数据结构体的转换
  static bool CvtPb2Measurement(
      const std::shared_ptr<hozon::perception::measurement::MeasurementPb>&
          measurepb,
      const std::shared_ptr<MeasurementFrame>& measure_frame);

  static bool CvtPb2LaneLineMeasurement(
      const hozon::perception::LaneInfo& laneinfo, LaneLinePtr laneptr);

  static bool CvtPb2CrossPointMeasurement(
      const hozon::perception::CrossPoint& crosspoint_info,
      CrossPointPtr cpptr);

  static bool CvtPb2RoadEdgeMeasurement(
      const hozon::perception::RoadEdge& roadedge, RoadEdgePtr roadedgeptr);

  static bool CvtPb2ZebraCrossingMeasurement(
      const hozon::perception::ZebraCrossing& zebracrossing,
      ZebraCrossingPtr zebracrossingptr);

  static bool CvtPb2StopLineMeasurement(
      const hozon::perception::StopLine& stopline, StopLinePtr stoplineptr);

  static bool CvtPb2ArrowMeasurement(const hozon::perception::Arrow& arrow,
                                     ArrowPtr arrowptr);

  static bool CvtPbLocation2Location(
      const NetaLoactionPtr& pb_location,
      std::shared_ptr<lm::Location> location_ptr);

  static bool CvtPbIns2Ins(const NetaInsPtr& pb_ins,
                           std::shared_ptr<lm::InsData> ins_ptr);

  static bool CvtPb2Object(const NetaPerceptionObstacle& pb_object,
                           ObjectPtr object_msg);

  // 内部数据结构体到局部建图pb的转换
  static bool CvtLocalMap2Pb(
      const std::shared_ptr<LocalMapFrame>& measure_frame_ptr,
      const std::shared_ptr<hozon::mapping::LocalMap>& localmap_pb);

  static bool CvtLaneLine2Pb(const LaneLinePtr& lane_msg,
                             hozon::mapping::LaneLine* pb_lane);

  static bool CvtRoadEdge2Pb(const RoadEdgePtr& roadedge_msg,
                             hozon::mapping::RoadEdge* pb_roadedge);

  static bool CvtArrow2Pb(const ArrowPtr& arrow_msg,
                          hozon::mapping::Arrow* pb_arrow);

  static bool CvtZebraCrossing2Pb(const ZebraCrossingPtr& zebracrossing_msg,
                                  hozon::mapping::CrossWalk* pb_zebracrossing);

  static bool CvtStopLine2Pb(const StopLinePtr& stopline_msg,
                             hozon::mapping::StopLine* pb_stopline);

  // 内部数据结构体到静态元素后处理pb的转换
  static bool CvtLocalMap2TePb(
      const std::shared_ptr<LocalMapFrame>& measure_frame_ptr,
      const std::shared_ptr<hozon::perception::TransportElement>& te_pb);

  static bool CvtLaneLine2TePb(const LaneLinePtr& lane_msg,
                               hozon::perception::LaneInfo* pb_lane);

  static bool CvtRoadEdge2TePb(const RoadEdgePtr& roadedge_msg,
                               hozon::perception::RoadEdge* pb_roadedge);
};

}  // namespace data_mapping
}  // namespace lm
}  // namespace mp
}  //  namespace hozon
