/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-10-16
 *****************************************************************************/

#include <cstdint>

#include "modules/local_mapping/data_mapping/data_mapping.h"
#include "proto/perception/transport_element.pb.h"

namespace hozon {
namespace mp {
namespace lm {
namespace data_mapping {

bool DataMapping::CvtLocalMap2TePb(
    const std::shared_ptr<LocalMapFrame>& localmap_frame_ptr,
    const std::shared_ptr<hozon::perception::TransportElement>&
        transport_element_pb) {
  transport_element_pb->mutable_header()->set_gnss_stamp(
      localmap_frame_ptr->header.timestamp);
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
      tp = std::chrono::time_point_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now());
  transport_element_pb->mutable_header()->set_publish_stamp(
      static_cast<double>(tp.time_since_epoch().count()) * 1.0e-9);
  transport_element_pb->mutable_header()->set_data_stamp(
      localmap_frame_ptr->header.timestamp);
  transport_element_pb->mutable_header()->set_seq(
      static_cast<int32_t>(localmap_frame_ptr->header.sequence_num));

  if (localmap_frame_ptr->lane_lines_ptr != nullptr) {
    for (auto& item : localmap_frame_ptr->lane_lines_ptr->lanelines) {
      if (item->send_postlane) {
        auto pb_laneline_ptr = transport_element_pb->add_lane();
        CvtLaneLine2TePb(item, pb_laneline_ptr);
      }
    }
  }

  if (localmap_frame_ptr->road_edges_ptr != nullptr) {
    for (auto& item : localmap_frame_ptr->road_edges_ptr->road_edges) {
      if (item->send_postlane) {
        auto pb_roadedge_ptr = transport_element_pb->add_road_edges();
        CvtRoadEdge2TePb(item, pb_roadedge_ptr);
      }
    }
  }

  return true;
}

}  // namespace data_mapping
}  // namespace lm
}  // namespace mp
}  //  namespace hozon
