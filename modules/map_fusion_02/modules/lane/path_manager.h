/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： path_manager.h
 *   author     ： taoshaoyuan
 *   date       ： 2024.01
 ******************************************************************************/

#pragma once

#include <algorithm>
#include <deque>
#include <memory>
#include <vector>

#include "modules/map_fusion_02/base/interface_option.h"
#include "modules/map_fusion_02/common/calc_util.h"
#include "modules/map_fusion_02/common/common_data.h"
#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace mf {

struct PathManagerConf {
  float back_range = 0;
  float interval = 0;
  int state_detect_range = 0;
};

class PathManager {
 public:
  PathManager() { latest_pose_.Reset(-1); }
  void Init(const LaneFusionProcessOption& options);
  void AddPose(const KinePose& pose);
  void GetPath(std::vector<KinePosePtr>* path);
  void GetPath(std::vector<KinePosePtr>* path,
               const std::vector<double>& line_param);
  KinePosePtr LatestPose();
  bool CheckTurnBack(const std::vector<Pose>& poses);
  void CheckPoseState(const KinePosePtr& cur_p);

 private:
  LaneFusionProcessOption conf_;
  float back_length_ = 0.;
  KinePose latest_pose_;
  std::deque<KinePosePtr> poses_;
};

using PathManagerPtr = std::shared_ptr<PathManager>;

}  // namespace mf
}  // namespace mp
}  // namespace hozon
