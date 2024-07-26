/********************************************************
 * Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 * Author: Hozon
 *******************************************************/
#include <cmath>

#include "base/scene/occedge.h"
#include "modules/local_mapping/data_mapping/data_mapping.h"
#include "proto/local_mapping/local_map.pb.h"

namespace hozon {
namespace mp {

namespace lm {
namespace data_mapping {

static hozon::mapping::LanePositionType CvtOccEdgePosType2Pb(
    LaneLinePosition type) {
  switch (type) {
    case LaneLinePosition::OTHER:
      return hozon::mapping::LanePositionType::LanePositionType_OTHER;
    case LaneLinePosition::FOURTH_LEFT:
      return hozon::mapping::LanePositionType::LanePositionType_FOURTH_LEFT;
    case LaneLinePosition::THIRD_LEFT:
      return hozon::mapping::LanePositionType::LanePositionType_THIRD_LEFT;
    case LaneLinePosition::ADJACENT_LEFT:
      return hozon::mapping::LanePositionType::LanePositionType_ADJACENT_LEFT;
    case LaneLinePosition::EGO_LEFT:
      return hozon::mapping::LanePositionType::LanePositionType_EGO_LEFT;
    case LaneLinePosition::EGO_RIGHT:
      return hozon::mapping::LanePositionType::LanePositionType_EGO_RIGHT;
    case LaneLinePosition::ADJACENT_RIGHT:
      return hozon::mapping::LanePositionType::LanePositionType_ADJACENT_RIGHT;
    case LaneLinePosition::THIRD_RIGHT:
      return hozon::mapping::LanePositionType::LanePositionType_THIRD_RIGHT;
    case LaneLinePosition::FOURTH_RIGHT:
      return hozon::mapping::LanePositionType::LanePositionType_FOURTH_RIGHT;
    default:
      return hozon::mapping::LanePositionType::LanePositionType_OTHER;
  }
}

bool DataMapping::CvtOccEdge2Pb(const OccEdgePtr& occedge_msg,
                                hozon::mapping::Occ* pb_occedge) {
  if (nullptr == occedge_msg || nullptr == pb_occedge) {
    HLOG_ERROR << "occedge msg  or pb_occedge is nullptr.";
    return false;
  }
  pb_occedge->set_track_id(occedge_msg->id);
  pb_occedge->set_detect_id(occedge_msg->detect_id);
  pb_occedge->set_confidence(occedge_msg->type_confidence);

  for (auto& item_pt : occedge_msg->vehicle_points) {
    auto pb_pt = pb_occedge->add_points();
    pb_pt->set_x(item_pt.x());
    pb_pt->set_y(item_pt.y());
    pb_pt->set_z(item_pt.z());
  }
  hozon::mapping::LanePositionType send_pos_type =
      CvtOccEdgePosType2Pb(occedge_msg->position);
  // HLOG_INFO << "=====******====roadedge send_pos_type:"
  //           << static_cast<int>(send_pos_type);
  auto* curve_lane = pb_occedge->mutable_lane_param()->add_cubic_curve_set();
  // pb_occedge->set_lanepos(send_pos_type);
  // pb_occedge->set_confidence(1.0);
  // pb_occedge->set_lost_age(occedge_msg->lost_age);
  // pb_occedge->set_tracked_age(occedge_msg->tracked_count);
  // pb_occedge->set_latest_tracked_time(occedge_msg->latest_tracked_time);
  if (occedge_msg->vehicle_curve.coeffs.size() == 4) {
    // 车道线三次方程系数跟踪方案发送字段
    curve_lane->set_c0(occedge_msg->vehicle_curve.coeffs[0]);
    curve_lane->set_c1(occedge_msg->vehicle_curve.coeffs[1]);
    curve_lane->set_c2(occedge_msg->vehicle_curve.coeffs[2]);
    curve_lane->set_c3(occedge_msg->vehicle_curve.coeffs[3]);
    curve_lane->set_start_point_x(occedge_msg->vehicle_curve.min);
    curve_lane->set_end_point_x(occedge_msg->vehicle_curve.max);
  }

  return true;
}

}  // namespace data_mapping
}  // namespace lm
}  // namespace mp
}  //  namespace hozon
