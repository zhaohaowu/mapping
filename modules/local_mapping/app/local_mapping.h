/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-08-31
 *****************************************************************************/
#pragma once

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <algorithm>
#include <functional>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/src/Geometry/Transform.h"
#include "depend/common/utm_projection/coordinate_convertor.h"
#include "depend/map/hdmap/hdmap.h"
#include "depend/proto/local_mapping/local_map.pb.h"
#include "depend/proto/localization/node_info.pb.h"
#include "depend/proto/map/map.pb.h"
#include "depend/proto/map/navigation.pb.h"
#include "depend/proto/perception/perception_measurement.pb.h"
#include "depend/proto/soc/sensor_image.pb.h"
#include "modules/local_mapping/base/location/Ins.h"
#include "modules/local_mapping/base/location/location.h"
#include "modules/local_mapping/base/scene/localmap.h"
#include "modules/local_mapping/core/map_worker.h"
#include "modules/local_mapping/utils/common.h"
#include "modules/local_mapping/utils/fetch_hq.h"
#include "modules/local_mapping/utils/rviz_common.h"
#include "modules/map_fusion/modules/map_hd/global_hd_map.h"
#include "modules/util/include/util/geo.h"
#include "modules/util/include/util/mapping_log.h"
#include "modules/util/include/util/tic_toc.h"
#include "perception-lib/lib/location_manager/location_manager.h"
namespace hozon {
namespace mp {
namespace lm {

class LocalMapApp {
 public:
  bool Init();

  bool OnPerception(const MeasurementFrameConstPtr& measurement_frame_ptr);

  void OnLocalization(const LocationConstPtr& localization_frame_ptr);

  void OnFreeSpace(const FreeSpacesConstPtr& freespace_frame_ptr);

  void OnIns(const InsDataConstPtr& ins_msg_ptr);

  static std::string Name() { return "LocalMapApp"; }

  LocalMapApp() = default;

  ~LocalMapApp();

  LocalMapApp(const LocalMapApp& other);

  LocalMapApp& operator=(const LocalMapApp& other);

  LocalMapApp(LocalMapApp&& other) noexcept;

  LocalMapApp& operator=(LocalMapApp&& other) noexcept;

  void RvizFunc();

 private:
  // 地图管理器，整个的localmap的管理器
  std::shared_ptr<MapWorker> mmgr_ptr_;
  // 计数使用
  int seq_ = 0;
  // 可视化用到的变量
  bool use_rviz_ = false;
  bool stop_rviz_thread_ = false;
  std::thread rviz_thread_;
  std::mutex rviz_mutex_;
  Eigen::Affine3d T_W_V_;
  Eigen::Affine3d globle_transfor_mat_;
  MeasurementFrame perception_;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
