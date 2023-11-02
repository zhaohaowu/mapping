/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: shenliangchen
 *Date: 2023-09-05
 *****************************************************************************/
#pragma once

#include <memory>
#include <vector>

#include "modules/local_mapping/types/common.h"
#include "modules/util/include/util/temp_log.h"
#include "proto/perception/transport_element.pb.h"

namespace hozon {
namespace mp {
namespace lm {
class DataConvert {
 public:
  DataConvert() = default;

  /**
   * @brief convert location message into internal class
   *
   * @param msg : location message
   * @return
   */
  static void SetLocation(const hozon::localization::Localization msg,
                          std::shared_ptr<Location> dr_location);

  /**
   * @brief convert dr message into internal class
   *
   * @param msg : dr message
   * @return
   */
  static void SetDr(const hozon::dead_reckoning::DeadReckoning msg,
                    std::shared_ptr<Location> dr_location);

  /**
   * @brief convert laneline message into internal class
   *
   * @param msg : laneline message
   * @return
   */
  static void SetLaneLine(const hozon::perception::TransportElement& msg,
                          std::shared_ptr<Lanes> lanes);

  /**
   * @brief convert laneline message into internal class
   *
   * @param msg : laneline message
   * @return
   */
  static void SetEdgeLine(const hozon::perception::TransportElement& msg,
                          std::shared_ptr<Lanes> lanes);

  /**
   * @brief convert road edge message into internal class
   *
   * @param msg : road edge message
   * @return
   */
  static void SetRoadEdge(const hozon::perception::TransportElement& msg);

  /**
   * @brief convert LanePositionType from proto to inner type
   *
   * @param raw_lane_pose_type : proto type
   * @param lane_pose_type : inner type
   * @return
   */
  static void ConvertProtoLanePos(
      const hozon::perception::LanePositionType& raw_lanepos,
      LanePositionType* lanepos);

  /**
   * @brief convert LanePositionType from proto to inner type
   *
   * @param raw_lane_pose_type : proto type
   * @param lane_pose_type : inner type
   * @return
   */
  static void ConvertProtoLaneType(
      const hozon::perception::LaneType& raw_lanetype, LaneType* lanetype);

  /**
   * @brief convert LanePositionType from inerr to proto type
   *
   * @param lane_pose_type : inner type
   * @return proto type
   */
  static void ConvertInnerLanePos(const LanePositionType& inner_lanepos,
                                  hozon::mapping::LanePositionType* lanepos);

  /**
   * @brief convert LanePositionType from inerr to proto type
   *
   * @param lane_pose_type : inner type
   * @return proto type
   */
  static void ConvertInnerLaneType(const LaneType& inner_lanetype,
                                   hozon::mapping::LaneType* lanetype);
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
