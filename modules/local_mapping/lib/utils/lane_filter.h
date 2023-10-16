/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: luerwei
 *Date: 2023-09-011
 *****************************************************************************/
#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <vector>

#include "modules/local_mapping/lib/datalogger/load_data_singleton.h"
#include "modules/local_mapping/lib/types/common.h"
#include "modules/local_mapping/lib/utils/common.h"
#include "modules/local_mapping/lib/utils/kalman_filter.h"
#include "modules/util/include/util/temp_log.h"

namespace hozon {
namespace mp {
namespace lm {

class PtFilter {
 public:
  PtFilter() = default;

  void Init(const LaneCubicSpline& lane);

  void Predict(double theta, const Eigen::Vector2d& T,
               std::shared_ptr<std::vector<Eigen::Vector2d>> pts);

  void Update(const std::vector<Eigen::Vector2d>& measure_pts,
              std::shared_ptr<std::vector<Eigen::Vector3d>> pts);

 private:
  std::vector<KFFilter> filter_;
};

class LaneFilter {
 public:
  LaneFilter() = default;
  bool LaneProcess(std::shared_ptr<const Lane> cur_lane,
                   std::shared_ptr<LaneCubicSpline> filtered_lane_func);
  void Init(std::shared_ptr<const Lane> cur_lane);
  Eigen::Vector3d GetRelativePose(const Eigen::Vector3d& pose0,
                                  const Eigen::Vector3d& pose1);

  void SetCurLanePose(const Eigen::Vector3d& pose) { cur_lane_pose_ = pose; }

  void SetLastLanePose(const Eigen::Vector3d& pose) { last_lane_pose_ = pose; }

 private:
  void GetCurLanePts(const std::vector<Eigen::Vector2d>& predicted_pts,
                     const LaneCubicSpline& cubic_curve,
                     std::shared_ptr<std::vector<Eigen::Vector2d>> pts);

 private:
  PtFilter kf_;
  bool init_ = true;
  LaneCubicSpline last_lane_func_;
  std::vector<Eigen::Vector2d> last_lane_pts_;
  Eigen::Vector3d cur_lane_pose_;
  Eigen::Vector3d last_lane_pose_;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
