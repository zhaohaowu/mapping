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
#include "depend/proto/local_mapping/local_map.pb.h"
#include "depend/proto/localization/node_info.pb.h"
#include "depend/proto/map/map.pb.h"
#include "depend/proto/soc/sensor_image.pb.h"
#include "modules/local_mapping/datalogger/load_data_singleton.h"
#include "modules/local_mapping/ops/association/horizon_assoc.h"
#include "modules/local_mapping/ops/lane/lane_op.h"
#include "modules/local_mapping/types/types.h"
#include "modules/local_mapping/utils/common.h"
#include "modules/local_mapping/utils/compute_loss.h"
#include "modules/local_mapping/utils/data_convert.h"
#include "modules/local_mapping/utils/fetch_hq.h"
#include "modules/local_mapping/utils/map_manager.h"
#include "modules/map_fusion/include/map_fusion/map_service/global_hd_map.h"
#include "modules/util/include/util/geo.h"
#include "modules/util/include/util/mapping_log.h"
namespace hozon {
namespace mp {
namespace lm {
class LMapApp {
 public:
  explicit LMapApp(const std::string& mapping_path,
                   const std::string& config_file);

  ~LMapApp();

  LMapApp(const LMapApp& other);

  LMapApp& operator=(const LMapApp& other);

  LMapApp(LMapApp&& other) noexcept;

  LMapApp& operator=(LMapApp&& other) noexcept;

  /**
   * @brief receive localization message
   *
   * @param msg : localization message
   * @return
   */
  void OnLocalization(
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
  void OnImage(
      const std::shared_ptr<const hozon::soc::CompressedImage>& msg) const;

  /**
   * @brief fetch local_map at current timestamp
   *
   * @return `true` for fetching success, `false` for failed
   */
  bool FetchLocalMap(
      const std::shared_ptr<hozon::mapping::LocalMap>& local_map);

  /**
   * @brief fetch local_map at current timestamp
   *
   * @return `true` for fetching success, `false` for failed
   */
  bool FetchLocalMap(const std::shared_ptr<hozon::hdmap::Map>& local_map);

  void RvizFunc();

 private:
  std::shared_ptr<LaneOp> laneOp_;
  std::shared_ptr<MapManager> mmgr_;
  std::shared_ptr<hozon::hdmap::HDMap> hdmap_ = nullptr;
  std::shared_ptr<hozon::hdmap::Map> hqmap_ = nullptr;
  std::shared_ptr<PriorProvider> provider_;
  std::shared_ptr<LocalMap> local_map_ptr_;
  LocalMap local_map_output_;
  Perception perception_;
  Sophus::SE3d T_W_V_, T_G_V_;
  std::mutex localmap_mutex_;
  std::mutex perception_mutex_;
  std::mutex T_mutex_;
  std::string map_file_;
  bool use_point_tracking_;
  bool use_perception_match_;
  bool use_horizon_assoc_match_;
  bool use_rviz_;
  std::atomic<bool> localization_inited_ = false;
  std::atomic<bool> dr_inited_ = false;
  std::atomic<bool> laneline_inited_ = false;
  std::atomic<bool> ins_inited_ = false;
  std::thread rviz_thread_;
  std::atomic<bool> stop_rviz_thread_ = true;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
