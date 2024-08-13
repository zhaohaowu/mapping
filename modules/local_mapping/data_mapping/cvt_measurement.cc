/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-10-16
 *****************************************************************************/

#include "base/utils/log.h"
#include "modules/local_mapping/data_mapping/data_mapping.h"
#include "modules/local_mapping/lib/datalogger/pose_manager.h"

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

bool DataMapping::CvtPbFreeSpaces2FreeSpaces(
    const std::shared_ptr<hozon::perception::FreeSpaceOutArray> fs_pb,
    std::shared_ptr<FreeSpaces> freespaces_ptr) {
  // 包含occ的数据转换过程
  freespaces_ptr->timestamp = fs_pb->header().data_stamp();
  // 时间戳递推当前时刻occ自车在local系位姿
  freespaces_ptr->freespaces_pose =
      POSE_MANAGER->GetDrPoseByTimeStamp(freespaces_ptr->timestamp);
  if (freespaces_ptr->freespaces_pose == nullptr) {
    HLOG_ERROR << "occ time is:" << std::to_string(freespaces_ptr->timestamp);
    HLOG_ERROR << "freespace_pose is nullptr";
    return false;
  }
  HLOG_DEBUG << "TEST pb lane_lines nums:"
             << fs_pb->freespace_out_vrf()[0].grid().type().size();
  bool first_flag = false;
  for (const auto& item : fs_pb->freespace_out_vrf()) {
    // freespace_out_vrf[0]是网格数据，跳过
    if (!first_flag) {
      first_flag = true;
      continue;
    }
    if (!(item.freespace_point().size() > 0)) {
      HLOG_ERROR << "freespace_point().size()<0";
      continue;
    }
    OccEdgePtr occedge_ptr = std::make_shared<OccEdge>();
    // 上游会把一个包络切成两个，处理时合并成一个，local_map后续处理时切
    if (CvtPbFreeSpace2FreeSpace(item, occedge_ptr)) {
      bool find_flag = false;
      for (const auto& occ : freespaces_ptr->edges.occ_edges) {
        if (occ->detect_id == occedge_ptr->detect_id) {
          occ->vehicle_points.insert(occ->vehicle_points.end(),
                                     occedge_ptr->vehicle_points.begin(),
                                     occedge_ptr->vehicle_points.end());
          find_flag = true;
          break;
        }
      }
      if (!find_flag) {
        freespaces_ptr->edges.occ_edges.push_back(occedge_ptr);
      }
    } else {
      HLOG_ERROR << "freespace measurement data has nan value...";
    }
  }

  return true;
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
    if (CvtPb2LaneLineMeasurement(item, laneptr)) {
      measure_frame->lane_lines_ptr->lanelines.push_back(laneptr);
    } else {
      HLOG_ERROR << "laneline measurement data has nan value...";
    }
  }
  measure_frame->road_edges_ptr = std::make_shared<RoadEdges>();
  HLOG_DEBUG << "TEST pb road_edges nums:"
             << measurepb->transport_element().road_edges().size();
  for (const auto& item : measurepb->transport_element().road_edges()) {
    if (item.points_size() != 20) {
      continue;
    }
    RoadEdgePtr roadedgeptr = std::make_shared<RoadEdge>();
    if (CvtPb2RoadEdgeMeasurement(item, roadedgeptr)) {
      measure_frame->road_edges_ptr->road_edges.push_back(roadedgeptr);
    } else {
      HLOG_ERROR << "roadedge measurement data has nan value...";
    }
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
    // PrintMeasurementStopLine(stoplineptr);
    measure_frame->stop_lines_ptr->stoplines.push_back(stoplineptr);
  }
  return true;
}

}  // namespace data_mapping
}  // namespace lm
}  // namespace mp
}  //  namespace hozon
