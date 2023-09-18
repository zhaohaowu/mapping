/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-08-31
 *****************************************************************************/
#pragma once

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "modules/local_mapping/lib/datalogger/load_data_singleton.h"
#include "modules/local_mapping/lib/ops/lane/lane_op.h"
#include "modules/local_mapping/lib/types/common.h"
#include "modules/local_mapping/lib/utils/common.h"
#include "modules/local_mapping/lib/utils/data_convert.h"
#include "modules/local_mapping/lib/utils/lane_filter.h"
#include "modules/local_mapping/lib/utils/map_manager.h"
#include "util/temp_log.h"
namespace hozon {
namespace mp {
namespace lm {
class LMapApp {
 public:
  explicit LMapApp(const std::string& config_file);

  /**
   * @brief receive location message
   *
   * @param msg : location message
   * @return
   */
  void OnLocation(
      const std::shared_ptr<const adsfi_proto::hz_Adsfi::AlgLocation>& msg);

  /**
   * @brief receive dr message
   *
   * @param msg : dr message
   * @return
   */
  void OnDr(
      const std::shared_ptr<const adsfi_proto::hz_Adsfi::AlgLocation>& msg);

  /**
   * @brief receive laneline message
   *
   * @param msg : laneline message
   * @return
   */
  void OnLaneLine(const std::shared_ptr<
                  const adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray>& msg);

  /**
   * @brief receive road edge message
   *
   * @param msg : road edge message
   * @return
   */
  void OnRoadEdge(const std::shared_ptr<
                  const adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray>& msg);

  /**
   * @brief fetch local_map at current timestamp
   *
   * @return `true` for fetching success, `false` for failed
   */
  bool FetchLocalMap(std::shared_ptr<hozon::mapping::LocalMap> local_map);

  ConstDrDataPtr GetDrPoseForTime(double timestamp);

 private:
  std::shared_ptr<LaneOp> laneOp_;
  std::shared_ptr<MapManager> mmgr_;

  std::shared_ptr<std::vector<LocalMapLane>> map_lanes_;
  std::shared_ptr<Location> latest_location_;
  std::shared_ptr<Location> latest_dr_;
  std::shared_ptr<Lanes> latest_lanes_;
  std::shared_ptr<std::vector<LaneMatchInfo>> lane_matches_;
  std::shared_ptr<std::vector<Eigen::Vector3d>> new_lane_pts_;
  double map_init_timestamp_;
  Eigen::Matrix4d init_T_, lasted_T_, T_V_W_;

  LaneFilter lane_filter_;
  std::mutex localmap_mutex_;
  bool use_rviz;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
