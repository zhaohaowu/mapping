/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-10-16
 *****************************************************************************/

#include <cstdint>

#include "modules/local_mapping/data_mapping/data_mapping.h"

namespace hozon {
namespace mp {
namespace lm {
namespace data_mapping {

bool DataMapping::CvtLocalMap2Pb(
    const std::shared_ptr<LocalMapFrame>& localmap_frame_ptr,
    const std::shared_ptr<hozon::mapping::LocalMap>& localmap_pb) {
  localmap_pb->set_init_timestamp(localmap_frame_ptr->header.timestamp);
  localmap_pb->mutable_header()->set_gnss_stamp(
      localmap_frame_ptr->header.timestamp);
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
      tp = std::chrono::time_point_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now());
  localmap_pb->mutable_header()->set_publish_stamp(
      static_cast<double>(tp.time_since_epoch().count()) * 1.0e-9);
  localmap_pb->mutable_header()->set_data_stamp(
      localmap_frame_ptr->header.timestamp);
  localmap_pb->mutable_header()->set_seq(
      static_cast<int32_t>(localmap_frame_ptr->header.sequence_num));

  if (localmap_frame_ptr->lane_lines_ptr != nullptr) {
    for (auto& item : localmap_frame_ptr->lane_lines_ptr->lanelines) {
      auto pb_laneline_ptr = localmap_pb->add_lane_lines();
      CvtLaneLine2Pb(item, pb_laneline_ptr);
    }
  }

  if (localmap_frame_ptr->road_edges_ptr != nullptr) {
    for (auto& item : localmap_frame_ptr->road_edges_ptr->road_edges) {
      auto pb_roadedge_ptr = localmap_pb->add_road_edges();
      CvtRoadEdge2Pb(item, pb_roadedge_ptr);
    }
  }

  if (localmap_frame_ptr->road_arrows_ptr != nullptr) {
    for (auto& item : localmap_frame_ptr->road_arrows_ptr->arrows) {
      auto pb_arrow_ptr = localmap_pb->add_arrows();
      CvtArrow2Pb(item, pb_arrow_ptr);
    }
  }

  if (localmap_frame_ptr->zebra_crossings_ptr != nullptr) {
    for (auto& item :
         localmap_frame_ptr->zebra_crossings_ptr->zebra_crossings) {
      auto pb_crosswalk_ptr = localmap_pb->add_cross_walks();
      CvtZebraCrossing2Pb(item, pb_crosswalk_ptr);
    }
  }

  if (localmap_frame_ptr->stop_lines_ptr != nullptr) {
    for (auto& item : localmap_frame_ptr->stop_lines_ptr->stoplines) {
      auto pb_stopline_ptr = localmap_pb->add_stop_lines();
      CvtStopLine2Pb(item, pb_stopline_ptr);
    }
  }

  if (localmap_frame_ptr->occ_edges_ptr != nullptr) {
    for (auto& item : localmap_frame_ptr->occ_edges_ptr->occ_edges) {
      auto pb_occ_edge_ptr = localmap_pb->add_occs();
      CvtOccEdge2Pb(item, pb_occ_edge_ptr);
    }
  }

  return true;
}

}  // namespace data_mapping
}  // namespace lm
}  // namespace mp
}  //  namespace hozon
