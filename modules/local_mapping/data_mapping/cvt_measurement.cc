/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-10-16
 *****************************************************************************/

#include "modules/local_mapping/data_mapping/data_mapping.h"

namespace hozon {
namespace mp {
namespace lm {
namespace data_mapping {

void PrintMeasurementStopLine(StopLinePtr stopline_ptr) {
  std::cout << "stopline-leftpoint:" << stopline_ptr->left_point << std::endl;
  std::cout << "stopline-right_point:" << stopline_ptr->right_point
            << std::endl;
  std::cout << "stopline-center_point:" << stopline_ptr->center_point
            << std::endl;
  std::cout << "stopline-heading:" << stopline_ptr->heading << std::endl;
  std::cout << "stopline-length:" << stopline_ptr->length << std::endl;
}

bool DataMapping::CvtPb2Measurement(
    const std::shared_ptr<hozon::perception::measurement::MeasurementPb>&
        measurepb,
    const std::shared_ptr<MeasurementFrame>& measure_frame) {
  measure_frame->header.timestamp = measurepb->header().data_stamp();
  measure_frame->header.sequence_num = measurepb->header().seq();

  // 障碍物观测数据转换，待补充 TODO(张文海)
  // if (!measure_frame->objects_ptr) {
  //   measure_frame->objects_ptr = std::make_shared<Objects>();
  // }
  // measure_frame->objects_ptr->objects.clear();
  // CvtPb2MultiObjects(measurepb->obstacles(),
  //                    measure_frame->objects_ptr->objects);

  measure_frame->lane_lines_ptr = std::make_shared<LaneLines>();
  HLOG_DEBUG << "TEST pb lane_lines nums:"
             << measurepb->transport_element().lane().size();
  for (const auto& item : measurepb->transport_element().lane()) {
    if (item.points_size() != 20) {
      continue;
    }
    LaneLinePtr laneptr = std::make_shared<LaneLine>();
    CvtPb2LaneLineMeasurement(item, laneptr);
    measure_frame->lane_lines_ptr->lanelines.push_back(laneptr);
  }
  measure_frame->road_edges_ptr = std::make_shared<RoadEdges>();
  HLOG_DEBUG << "TEST pb road_edges nums:"
             << measurepb->transport_element().road_edges().size();
  for (const auto& item : measurepb->transport_element().road_edges()) {
    if (item.points_size() != 20) {
      continue;
    }
    RoadEdgePtr roadedgeptr = std::make_shared<RoadEdge>();
    CvtPb2RoadEdgeMeasurement(item, roadedgeptr);
    measure_frame->road_edges_ptr->road_edges.push_back(roadedgeptr);
  }
  measure_frame->road_arrows_ptr = std::make_shared<Arrows>();
  HLOG_DEBUG << "TEST pb road_arrows nums:"
             << measurepb->transport_element().arrow().size();
  for (const auto& item : measurepb->transport_element().arrow()) {
    ArrowPtr arrowptr = std::make_shared<Arrow>();
    CvtPb2ArrowMeasurement(item, arrowptr);
    measure_frame->road_arrows_ptr->arrows.push_back(arrowptr);
  }
  measure_frame->zebra_crossings_ptr = std::make_shared<ZebraCrossings>();
  HLOG_DEBUG << "TEST pb zebra_crossings nums:"
             << measurepb->transport_element().zebra_crossing().size();
  for (const auto& item : measurepb->transport_element().zebra_crossing()) {
    ZebraCrossingPtr zebra_crossingptr = std::make_shared<ZebraCrossing>();
    if (CvtPb2ZebraCrossingMeasurement(item, zebra_crossingptr)) {
      measure_frame->zebra_crossings_ptr->zebra_crossings.push_back(
          zebra_crossingptr);
    }
  }
  measure_frame->stop_lines_ptr = std::make_shared<StopLines>();
  HLOG_DEBUG << "TEST pb stoplines nums:"
             << measurepb->transport_element().stopline().size();
  for (const auto& item : measurepb->transport_element().stopline()) {
    StopLinePtr stoplineptr = std::make_shared<StopLine>();
    CvtPb2StopLineMeasurement(item, stoplineptr);
    PrintMeasurementStopLine(stoplineptr);
    measure_frame->stop_lines_ptr->stoplines.push_back(stoplineptr);
  }
  return true;
}

}  // namespace data_mapping
}  // namespace lm
}  // namespace mp
}  //  namespace hozon
