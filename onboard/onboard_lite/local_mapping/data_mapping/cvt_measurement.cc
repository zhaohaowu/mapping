/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-10-16
 *****************************************************************************/

#include "onboard/onboard_lite/local_mapping/data_mapping/data_mapping.h"

namespace hozon {
namespace mp {
namespace common_onboard {

bool DataMapping::CvtPb2Measurement(
    const std::shared_ptr<hozon::perception::measurement::MeasurementPb>&
        measurepb,
    std::shared_ptr<hozon::perception::base::MeasurementFrame> measure_frame) {
  measure_frame->header.timestamp = measurepb->header().data_stamp();
  measure_frame->header.sequence_num = measurepb->header().seq();
  if (!measure_frame->objects_measurement_) {
    measure_frame->objects_measurement_ =
        std::make_shared<hozon::perception::base::ObjectMeasurement>();
  }
  measure_frame->objects_measurement_->objects_.clear();
  CvtPb2MultiObjects(measurepb->obstacles(),
                     measure_frame->objects_measurement_->objects_);

  measure_frame->lanelines_measurement_ =
      std::make_shared<hozon::perception::base::LaneLinesMeasurement>();
  for (auto& item : measurepb->transport_element().lane()) {
    hozon::perception::base::LaneLineMeasurementPtr laneptr =
        std::make_shared<hozon::perception::base::LaneLineMeasurement>();
    CvtPb2LaneLineMeasurement(item, laneptr);
    measure_frame->lanelines_measurement_->lanelines.push_back(laneptr);
  }
  measure_frame->roadedges_measurement_ =
      std::make_shared<hozon::perception::base::RoadEdgesMeasurement>();
  for (auto& item : measurepb->transport_element().road_edges()) {
    hozon::perception::base::RoadEdgeMeasurementPtr roadedgeptr =
        std::make_shared<hozon::perception::base::RoadEdgeMeasurement>();
    CvtPb2RoadEdgeMeasurement(item, roadedgeptr);
    measure_frame->roadedges_measurement_->road_edges.push_back(roadedgeptr);
  }
  measure_frame->arrows_measurement_ =
      std::make_shared<hozon::perception::base::ArrowsMeasurement>();
  for (auto& item : measurepb->transport_element().arrow()) {
    hozon::perception::base::ArrowMeasurementPtr arrowsptr =
        std::make_shared<hozon::perception::base::ArrowMeasurement>();
    CvtPb2ArrowMeasurement(item, arrowsptr);
    measure_frame->arrows_measurement_->arrows.push_back(arrowsptr);
  }
  measure_frame->zebracrossings_measurement_ =
      std::make_shared<hozon::perception::base::ZebraCrossingsMeasurement>();
  for (auto& item : measurepb->transport_element().zebra_crossing()) {
    hozon::perception::base::ZebraCrossingMeasurementPtr zebra_crossingptr =
        std::make_shared<hozon::perception::base::ZebraCrossingMeasurement>();
    CvtPb2ZebraCrossingMeasurement(item, zebra_crossingptr);
    measure_frame->zebracrossings_measurement_->crosswalks.push_back(
        zebra_crossingptr);
  }
  measure_frame->stoplines_measurement_ =
      std::make_shared<hozon::perception::base::StopLinesMeasurement>();
  for (auto& item : measurepb->transport_element().stopline()) {
    hozon::perception::base::StopLineMeasurementPtr stoplineptr =
        std::make_shared<hozon::perception::base::StopLineMeasurement>();
    CvtPb2StopLineMeasurement(item, stoplineptr);
    measure_frame->stoplines_measurement_->stoplines.push_back(stoplineptr);
  }
  return true;
}

}  //  namespace common_onboard
}  // namespace mp
}  //  namespace hozon
