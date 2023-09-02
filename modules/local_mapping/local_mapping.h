/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-08-31
 *****************************************************************************/
#pragma once

#include <iostream>
#include <memory>

#include "interface/adsfi_proto/location/location.pb.h"
#include "interface/adsfi_proto/map/local_map.pb.h"
#include "interface/adsfi_proto/perception/lanes.pb.h"
#include "modules/local_mapping/local_mapping.h"

namespace hozon {
namespace mp {
namespace lm {
class LMapApp {
 public:
  LMapApp() = default;

  /**
   * @brief receive location message
   *
   * @param msg : location message
   * @return
   */
  void OnLocation(
      const std::shared_ptr<const adsfi_proto::hz_Adsfi::AlgLocation> &msg);

  /**
   * @brief receive dr message
   *
   * @param msg : dr message
   * @return
   */
  void OnDr(
      const std::shared_ptr<const adsfi_proto::hz_Adsfi::AlgLocation> &msg);

  /**
   * @brief receive laneline message
   *
   * @param msg : laneline message
   * @return
   */
  void OnLaneLine(const std::shared_ptr<
                  const adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray> &msg);

  /**
   * @brief receive road edge message
   *
   * @param msg : road edge message
   * @return
   */
  void OnRoadEdge(const std::shared_ptr<
                  const adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray> &msg);

  /**
   * @brief fetch local_map at current timestamp
   *
   * @param local_map : local_map at current timestamp
   * @return `true` for fetching success, `false` for failed
   */
  bool FetchLocalMap(std::shared_ptr<LocalMap> local_map);
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
