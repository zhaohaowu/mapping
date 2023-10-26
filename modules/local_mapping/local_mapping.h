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
#include <list>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Sophus/se3.hpp>

#include "depend/common/utm_projection/coordinate_convertor.h"
#include "depend/map/hdmap/hdmap.h"
#include "depend/proto/localization/node_info.pb.h"
#include "depend/proto/map/map.pb.h"
#include "depend/proto/soc/sensor_image.pb.h"
#include "modules/local_mapping/datalogger/load_data_singleton.h"
#include "modules/local_mapping/ops/association/bipartite_match.h"
#include "modules/local_mapping/ops/lane/lane_op.h"
#include "modules/local_mapping/types/common.h"
#include "modules/local_mapping/utils/common.h"
#include "modules/local_mapping/utils/compute_loss.h"
#include "modules/local_mapping/utils/data_convert.h"
#include "modules/local_mapping/utils/fetch_hq.h"
#include "modules/local_mapping/utils/lane_filter.h"
#include "modules/local_mapping/utils/map_manager.h"
#include "modules/util/include/util/geo.h"
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
   * @brief receive image message
   *
   * @param msg : image message
   * @return
   */
  void OnImage(const std::shared_ptr<const hozon::soc::CompressedImage>& msg);

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

  std::shared_ptr<hozon::hdmap::HDMap> hdmap_;
  std::shared_ptr<hozon::hdmap::Map> crop_map_;
  std::shared_ptr<PriorProvider> provider_;
  std::shared_ptr<const hozon::soc::CompressedImage> image_msg = nullptr;
  Sophus::SE3d init_T_, lasted_T_, T_W_V_, T_G_V_;

  std::mutex localmap_mutex_;
  bool use_perception_match_;
  bool use_bipartite_assoc_match_;
  bool use_rviz_;
  bool use_filter_;
  bool dr_inited_;
  bool laneline_inited_;
  std::shared_ptr<PtFilter> lane_filter_;
  LocalMap local_map_tmp_;

  double last_lane_timestamp_;
  Loss loss_;
  bool compute_error;
  double sample_interval;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
