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

#include <Sophus/se3.hpp>

#include "depend/common/utm_projection/coordinate_convertor.h"
#include "depend/map/hdmap/hdmap.h"
#include "depend/perception-base/base/frame/measurement_frame.h"
#include "depend/proto/local_mapping/local_map.pb.h"
#include "depend/proto/localization/node_info.pb.h"
#include "depend/proto/map/map.pb.h"
#include "depend/proto/map/navigation.pb.h"
#include "depend/proto/perception/perception_measurement.pb.h"
#include "depend/proto/soc/sensor_image.pb.h"
#include "modules/local_mapping/app/laneline_postprocess.h"
#include "modules/local_mapping/app/roadedge_postprocess.h"
#include "modules/local_mapping/app/roadmark_postprocess.h"
#include "modules/local_mapping/datalogger/load_data_singleton.h"
#include "modules/local_mapping/types/types.h"
#include "modules/local_mapping/utils/common.h"
#include "modules/local_mapping/utils/data_convert.h"
#include "modules/local_mapping/utils/fetch_hq.h"
#include "modules/local_mapping/utils/map_manager.h"
#include "modules/local_mapping/utils/rviz_common.h"
#include "modules/map_fusion/include/map_fusion/map_service/global_hd_map.h"
#include "modules/util/include/util/geo.h"
#include "modules/util/include/util/mapping_log.h"
#include "modules/util/include/util/tic_toc.h"
#include "perception-lib/lib/location_manager/location_manager.h"
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

  void OnLocalization(
      const std::shared_ptr<const Localization>& latest_localization);

  void OnDr(const std::shared_ptr<hozon::perception::base::Location>&
                latest_localization);

  void DoBuildMap(const std::shared_ptr<Perception>& perception);

  bool DoPostProcess(
      hozon::perception::base::MeasurementFramePtr measurement_frame,
      hozon::perception::base::FusionFramePtr fusion_frame);

  void OnIns(const std::shared_ptr<const InsData>& msg);

  bool FetchLocalMap(
      const std::shared_ptr<hozon::mapping::LocalMap>& local_map);

  void RvizFunc();

 protected:
  void ProcLaneLine(const std::shared_ptr<const Perception>& perception);
  void ProcRoadEdge(const std::shared_ptr<const Perception>& perception);
  void ProcStopLine(const std::shared_ptr<const Perception>& perception);
  void ProcZebraCrossing(const std::shared_ptr<const Perception>& perception);
  void ProcArrow(const std::shared_ptr<const Perception>& perception);
  static void PreProcArrow(const std::shared_ptr<Perception>& perception);

 private:
  std::unique_ptr<environment::RoadMarkPostProcess> roadmark_postprocessor_;
  std::unique_ptr<environment::LanePostProcess> lane_postprocessor_;
  std::unique_ptr<environment::RoadEdgePostProcess> roadedge_postprocessor_;

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
  bool use_rviz_;
  std::atomic<bool> localization_inited_ = false;
  std::atomic<bool> perception_inited_ = false;
  std::atomic<bool> ins_inited_ = false;
  std::thread rviz_thread_;
  std::atomic<bool> stop_rviz_thread_ = true;

  Sophus::SE3d last_twv_;
  Sophus::SE3d cur_twv_;

  int seq_ = 0;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
