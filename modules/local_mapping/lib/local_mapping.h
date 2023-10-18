/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-08-31
 *****************************************************************************/
#pragma once

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Sophus/se3.hpp>
#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "depend/map/hdmap/hdmap.h"
#include "depend/proto/localization/node_info.pb.h"
#include "depend/proto/map/map.pb.h"
#include "modules/local_mapping/lib/datalogger/load_data_singleton.h"
#include "modules/local_mapping/lib/ops/lane/lane_op.h"
#include "modules/local_mapping/lib/types/common.h"
#include "modules/local_mapping/lib/utils/common.h"
#include "modules/local_mapping/lib/utils/compute_loss.h"
#include "modules/local_mapping/lib/utils/data_convert.h"
#include "modules/local_mapping/lib/utils/fetch_hq.h"
#include "modules/local_mapping/lib/utils/lane_filter.h"
#include "modules/local_mapping/lib/utils/map_manager.h"
#include "modules/util/include/util/temp_log.h"
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
      const std::shared_ptr<const hozon::localization::Localization>& msg);

  /**
   * @brief receive dr message
   *
   * @param msg : dr message
   * @return
   */
  void OnDr(
      const std::shared_ptr<const hozon::dead_reckoning::DeadReckoning>& msg);

  /**
   * @brief receive ins message
   *
   * @param msg : ins message
   * @return
   */
  void OnIns(
      const std::shared_ptr<const hozon::localization::HafNodeInfo>& msg);

  /**
   * @brief receive laneline message
   *
   * @param msg : laneline message
   * @return
   */
  void OnLaneLine(
      const std::shared_ptr<const hozon::perception::TransportElement>& msg);

  /**
   * @brief receive road edge message
   *
   * @param msg : road edge message
   * @return
   */
  void OnRoadEdge(
      const std::shared_ptr<const hozon::perception::TransportElement>& msg);

  /**
   * @brief fetch local_map at current timestamp
   *
   * @return `true` for fetching success, `false` for failed
   */
  bool FetchLocalMap(std::shared_ptr<hozon::mapping::LocalMap> local_map);

  /**
   * @brief fetch local_map location at current timestamp
   *
   * @return `true` for fetching success, `false` for failed
   */
  bool FetchLocalMapLocation(
      std::shared_ptr<hozon::localization::Localization> local_map_location);

  ConstDrDataPtr GetDrPoseForTime(double timestamp);

  void SetLaneTimestamp(const double timestamp) {
    last_lane_timestamp_ = timestamp;
  }

 private:
  std::shared_ptr<LaneOp> laneOp_;
  std::shared_ptr<MapManager> mmgr_;

  std::shared_ptr<std::vector<LocalMapLane>> map_lanes_;
  std::shared_ptr<Location> latest_location_;
  std::shared_ptr<Location> latest_dr_;
  std::shared_ptr<Lanes> latest_lanes_;
  std::shared_ptr<std::vector<LaneMatchInfo>> lane_matches_;
  std::shared_ptr<hozon::hdmap::HDMap> hdmap_;
  std::shared_ptr<hozon::hdmap::Map> crop_map_;
  std::shared_ptr<PriorProvider> provider_;
  double map_init_timestamp_;
  Sophus::SE3d init_T_, lasted_T_, T_W_V_, T_V_W_, T_G_V_, T_W_UTM_,
      last_T_W_V_;

  std::mutex localmap_mutex_;
  bool use_perception_match_;
  bool use_bipartite_assoc_match_;
  bool use_rviz_;
  bool dr_inited_;
  bool laneline_inited_;
  std::shared_ptr<PtFilter> lane_filter_;

  double last_lane_timestamp_;
  Loss loss_;
  bool compute_error;
  double sample_interval;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
