/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： path_manager.h
 *   author     ： taoshaoyuan
 *   date       ： 2024.01
 ******************************************************************************/

#pragma once

#include <deque>
#include <vector>

#include "map_fusion/fusion_common/common_data.h"

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
  explicit PathManager(const PathManagerConf& conf)
      : conf_(conf), back_length_(0) {
    latest_pose_.Reset(-1);
  }
  void AddPose(const KinePose& pose);
  void GetPath(std::vector<KinePose::Ptr>* path, float predict_range = -1);
  void GetPath(std::vector<KinePose::Ptr>* path, float predict_range,
               const std::vector<double>& line_param);
  KinePose::Ptr LatestPose();
  bool CheckTurnBack(const std::vector<Pose>& poses);
  void CheckPoseState(const KinePose::Ptr& cur_p);

 private:
  PathManagerConf conf_;
  float back_length_ = 0.;
  KinePose latest_pose_;
  std::deque<KinePose::Ptr> poses_;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
