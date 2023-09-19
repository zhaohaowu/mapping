/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: shenliangchen
 *Date: 2023-09-05
 *****************************************************************************/
#pragma once

#include <memory>
#include <vector>

#include "interface/adsfi_proto/location/location.pb.h"
#include "interface/adsfi_proto/perception/lanes.pb.h"
#include "modules/local_mapping/lib/types/common.h"
#include "proto/local_mapping/local_map.pb.h"

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
  static void SetLocation(const adsfi_proto::hz_Adsfi::AlgLocation msg,
                          std::shared_ptr<Location> dr_location);

  /**
   * @brief convert dr message into internal class
   *
   * @param msg : dr message
   * @return
   */
  static void SetDr(const adsfi_proto::hz_Adsfi::AlgLocation msg,
                    std::shared_ptr<Location> dr_location);

  /**
   * @brief convert laneline message into internal class
   *
   * @param msg : laneline message
   * @return
   */
  static void SetLaneLine(
      const adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray& msg,
      std::shared_ptr<Lanes> lanes);

  /**
   * @brief convert road edge message into internal class
   *
   * @param msg : road edge message
   * @return
   */
  static void SetRoadEdge(
      const adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray& msg);
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
