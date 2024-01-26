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

#include "base/frame/camera_frame.h"
#include "base/frame/measurement_frame.h"
#include "base/location/location.h"
#include "base/measurement/arrow_measurement.h"
#include "base/measurement/laneline_measurement.h"
#include "base/measurement/noparking_measurement.h"
#include "base/measurement/roadedges_measurement.h"
#include "base/measurement/slowdown_measurement.h"
#include "base/measurement/stopline_measurement.h"
#include "base/measurement/waitzone_measurement.h"
#include "base/measurement/zebra_crossing_measurement.h"
#include "base/occupancy/occupancy.h"
#include "base/point/radar_point.h"
#include "base/scene/arrow.h"
#include "base/scene/laneline.h"
#include "base/scene/noparking.h"
#include "base/scene/roadedges.h"
#include "base/scene/slowdown.h"
#include "base/scene/stopline.h"
#include "base/scene/waitzone.h"
#include "base/scene/zebra_crossing.h"
#include "base/state_machine/state_machine_info.h"
#include "base/utils/log.h"
#include "proto/dead_reckoning/dr.pb.h"
#include "proto/localization/localization.pb.h"
#include "proto/perception/perception_measurement.pb.h"
#include "proto/soc/radar.pb.h"

namespace hozon {
namespace mp {
namespace common_onboard {

#define NAME_SHARED_PTR(type)                         \
  using type##ConstPtr = std::shared_ptr<const type>; \
  using type##Ptr = std::shared_ptr<type>;

using NetaTransportElement = hozon::perception::TransportElement;
NAME_SHARED_PTR(NetaTransportElement)

using NetaMeasurementPb = hozon::perception::measurement::MeasurementPb;

using NetaDeadReckoning = hozon::dead_reckoning::DeadReckoning;
NAME_SHARED_PTR(NetaDeadReckoning)

using NetaLoaction = hozon::localization::Localization;
NAME_SHARED_PTR(NetaLoaction)

using namespace hozon::perception;
/**
 * @brief The DataMapping class provides functions for converting data between
 * different formats.
 */
class DataMapping {
 public:
  /**
   * @brief Default constructor for the DataMapping class.
   */
  DataMapping() = default;
  /**
   * @brief Default destructor for the DataMapping class.
   */
  ~DataMapping() = default;
  /**
   * @brief Converts a protobuf fisheye image to a frame message.
   * @param pb_Image The protobuf fisheye image.
   * @param framemsg The frame message to be populated.
   * @param cameraid The ID of the camera.
   * @return True if the conversion is successful, false otherwise.
   */
  /**
   * @brief Converts lanelines data to a protobuf format.
   * @param lanes_msg The laneline data.
   * @param pb_objects The frame message to be populated.
   * @return True if the conversion is successful, false otherwise.
   */
  using LaneLinePtrVec = std::vector<base::LaneLinePtr>;
  static bool CvtMultiLanesToPb(const LaneLinePtrVec &lanes_msg,
                                NetaTransportElementPtr pb_objects);
  static bool CvtLaneToPb(const base::LaneLinePtr &lane_msg,
                          LaneInfo *pb_object);

  using RoadEdgePtrVec = std::vector<base::RoadEdgePtr>;
  static bool CvtMultiRoadEdgesToPb(const RoadEdgePtrVec &roadedges_msg,
                                    NetaTransportElementPtr pb_objects);
  static bool CvtRoadEdgeToPb(const base::RoadEdgePtr &roadedge_msg,
                              RoadEdge *pb_object);

  static bool CvtPb2Measurement(
      const std::shared_ptr<measurement::MeasurementPb> &measurepb,
      std::shared_ptr<base::MeasurementFrame> measure_frame);

  /**
   * @brief convert lane measurement to proto
   *
   * @param laneline_measure
   * @param transport_element
   * @return true
   * @return false
   */
  static bool CvtLaneMeasurementToPb(
      const base::LaneLinesMeasurementPtr &laneline_measures,
      hozon::perception::TransportElement *transport_element);
  /**
   * @brief convert roadedge measurement to proto
   *
   * @param roadedge_measure
   * @param transport_element
   * @return true
   * @return false
   */
  static bool CvtRoadEdgeMeasurementToPb(
      const std::vector<base::RoadEdgeMeasurementPtr> &roadedge_measures,
      hozon::perception::TransportElement *transport_element);

  /**
   * @brief convert lane line pb to measurement
   *
   * @param laneinfo
   * @param laneptr
   * @return true
   * @return false
   */
  static bool CvtPb2LaneLineMeasurement(
      const hozon::perception::LaneInfo &laneinfo,
      base::LaneLineMeasurementPtr laneptr);

  /**
   * @brief convert roadedge pb to measurement
   *
   * @param roadedge
   * @param roadedgeptr
   * @return true
   * @return false
   */
  static bool CvtPb2RoadEdgeMeasurement(
      const hozon::perception::RoadEdge &roadedge,
      base::RoadEdgeMeasurementPtr roadedgeptr);

  /**
   * @brief convert zebracrossing pb to measurement
   *
   * @param zebracrossing
   * @param zebracrossingptr
   * @return true
   * @return false
   */
  static bool CvtPb2ZebraCrossingMeasurement(
      const hozon::perception::ZebraCrossing &zebracrossing,
      base::ZebraCrossingMeasurementPtr zebracrossingptr);
  /**
   * @brief convert stopline pb to measurement
   *
   * @param stopline
   * @param stoplineptr
   * @return true
   * @return false
   */
  static bool CvtPb2StopLineMeasurement(
      const hozon::perception::StopLine &stopline,
      base::StopLineMeasurementPtr stoplineptr);
  /**
   * @brief convert arrow pb to measurement
   *
   * @param arrow
   * @param arrowptr
   * @return true
   * @return false
   */
  static bool CvtPb2ArrowMeasurement(const hozon::perception::Arrow &arrow,
                                     base::ArrowMeasurementPtr arrowptr);

  /**
   * @brief convert arrow measurement to proto
   *
   * @param arrow_measure
   * @param transport_element
   * @return true
   * @return false
   */
  static bool CvtArrowMeasurementToPb(
      const std::vector<base::ArrowMeasurementPtr> &arrow_measure,
      hozon::perception::TransportElement *transport_element);

  /**
   * @brief convert zebracrossing measurement to proto
   *
   * @param zebracrossing_measure
   * @param transport_element
   * @return true
   * @return false
   */
  static bool CvtZebraCrossingMeasurementToPb(
      const std::vector<base::ZebraCrossingMeasurementPtr>
          &zebracrossing_measure,
      hozon::perception::TransportElement *transport_element);

  /**
   * @brief convert waitzone measurement to proto
   *
   * @param waitzone_measure
   * @param transport_element
   * @return true
   * @return false
   */
  static bool CvtWaitZoneMeasurementToPb(
      const std::vector<base::WaitZoneMeasurementPtr> &waitzone_measure,
      hozon::perception::TransportElement *transport_element);

  /**
   * @brief convert noparking measurement to proto
   *
   * @param noparking_measure
   * @param transport_element
   * @return true
   * @return false
   */
  static bool CvtNoParkingMeasurementToPb(
      const std::vector<base::NoparkingMeasurementPtr> &noparking_measure,
      hozon::perception::TransportElement *transport_element);

  /**
   * @brief convert slowdown measurement to proto
   *
   * @param slowdown_measure
   * @param transport_element
   * @return true
   * @return false
   */
  static bool CvtSlowDownMeasurementToPb(
      const std::vector<base::SlowdownMeasurementPtr> &slowdown_measure,
      hozon::perception::TransportElement *transport_element);

  /**
   * @brief convert stopline measurement to proto
   *
   * @param stopline_measure
   * @param transport_element
   * @return true
   * @return false
   */
  static bool CvtStopLineMeasurementToPb(
      const std::vector<base::StopLineMeasurementPtr> &stopline_measure,
      hozon::perception::TransportElement *transport_element);

  using ArrowPtrVec = std::vector<base::ArrowPtr>;
  static bool CvtMultiArrowsToPb(const ArrowPtrVec &arrows_msg,
                                 NetaTransportElementPtr pb_objects);

  static bool CvtArrowToPb(const base::ArrowPtr &arrow_msg, Arrow *pb_arrow);

  using ZebraCrossingPtrVec = std::vector<base::ZebraCrossingPtr>;
  static bool CvtMultiZebraCrossingsToPb(
      const ZebraCrossingPtrVec &zebracrossings_msg,
      NetaTransportElementPtr pb_objects);

  static bool CvtZebraCrossingToPb(
      const base::ZebraCrossingPtr &zebracrossing_msg,
      ZebraCrossing *pb_zebracrossing);

  using WaitZonePtrVec = std::vector<base::WaitZonePtr>;
  static bool CvtMultiWaitZonesToPb(const WaitZonePtrVec &waitzones_msg,
                                    NetaTransportElementPtr pb_objects);

  static bool CvtWaitZoneToPb(const base::WaitZonePtr &waitzone_msg,
                              TurnWaitingZone *pb_waitzone);

  using NoParkingPtrVec = std::vector<base::NoParkingPtr>;
  static bool CvtMultiNoParkingsToPb(const NoParkingPtrVec &noparkings_msg,
                                     NetaTransportElementPtr pb_objects);

  static bool CvtNoParkingToPb(const base::NoParkingPtr &noparking_msg,
                               NoParkingZone *pb_waitzone);

  using SlowDownPtrVec = std::vector<base::SlowDownPtr>;
  static bool CvtMultiSlowDownsToPb(const SlowDownPtrVec &slowdowns_msg,
                                    NetaTransportElementPtr pb_objects);

  static bool CvtSlowDownToPb(const base::SlowDownPtr &slowdown_msg,
                              SlowDown *pb_waitzone);

  using StopLinePtrVec = std::vector<base::StopLinePtr>;
  static bool CvtMultiStopLinesToPb(const StopLinePtrVec &stoplines_msg,
                                    NetaTransportElementPtr pb_objects);

  static bool CvtStopLineToPb(const base::StopLinePtr &stopline_msg,
                              StopLine *pb_stopline);

  static bool CvtPbLocation2Location(
      const NetaLoactionPtr &pb_location,
      std::shared_ptr<base::Location> location_ptr);

  static bool CvtPbDR2Location(const NetaDeadReckoningPtr &pb_dr,
                               std::shared_ptr<base::Location> location_ptr);
};

}  //  namespace common_onboard
}  // namespace mp
}  //  namespace hozon
