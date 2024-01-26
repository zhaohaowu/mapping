/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-10-16
 *****************************************************************************/

#include "onboard/onboard_lite/laneline_postprocess/data_mapping/data_mapping.h"

namespace hozon {
namespace mp {
namespace common_onboard {

bool DataMapping::CvtPb2Measurement(
    const std::shared_ptr<measurement::MeasurementPb> &measurepb,
    std::shared_ptr<base::MeasurementFrame> measure_frame) {
  measure_frame->header.timestamp = measurepb->header().data_stamp();
  measure_frame->header.sequence_num = measurepb->header().seq();
  if (!measure_frame->objects_measurement_) {
    measure_frame->objects_measurement_ =
        std::make_shared<base::ObjectMeasurement>();
  }
  measure_frame->lanelines_measurement_ =
      std::make_shared<base::LaneLinesMeasurement>();
  for (auto &item : measurepb->transport_element().lane()) {
    base::LaneLineMeasurementPtr laneptr =
        std::make_shared<base::LaneLineMeasurement>();
    CvtPb2LaneLineMeasurement(item, laneptr);
    measure_frame->lanelines_measurement_->lanelines.push_back(laneptr);
  }
  measure_frame->roadedges_measurement_ =
      std::make_shared<base::RoadEdgesMeasurement>();
  for (auto &item : measurepb->transport_element().road_edges()) {
    base::RoadEdgeMeasurementPtr roadedgeptr =
        std::make_shared<base::RoadEdgeMeasurement>();
    CvtPb2RoadEdgeMeasurement(item, roadedgeptr);
    measure_frame->roadedges_measurement_->road_edges.push_back(roadedgeptr);
  }

  return true;
}

}  //  namespace common_onboard
}  // namespace mp
}  //  namespace hozon
