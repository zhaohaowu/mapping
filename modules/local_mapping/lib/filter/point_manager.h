// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: lane_point_filter.h
// @brief: filter 3d points

#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Dense"
#include "boost/circular_buffer.hpp"
#include "modules/local_mapping/lib/datalogger/pose_manager.h"
#include "modules/local_mapping/lib/target/base_target.h"

namespace hozon {
namespace mp {
namespace lm {

class AdaptorPointManager {
 public:
  struct InnerPoint {
    // 车体系下的坐标点
    Eigen::Vector3d vehicle_point;
    // local系下的坐标点
    Eigen::Vector3d local_point;
  };

 public:
  AdaptorPointManager() {
    latest_measurement_lines_.set_capacity(3);
    latest_measurement_lines_.clear();
  }

  ~AdaptorPointManager() = default;

  AdaptorPointManager(const AdaptorPointManager&) = delete;
  AdaptorPointManager& operator=(const AdaptorPointManager&) = delete;

  void CalThreshold(Eigen::Matrix<double, 40, 1>* XPtr,
                    Eigen::Matrix<double, 40, 40>* PPtr);

  void CopyMatrix(Eigen::Matrix<double, 40, 1>* XPtr,
                  Eigen::Matrix<double, 40, 40>* PPtr);
  template <typename Element>
  void AddObservePoints(const std::shared_ptr<Element>& measurement);

  void Process(Eigen::Matrix<double, 40, 1>* X,
               Eigen::Matrix<double, 40, 40>* P);

 private:
  void UpdatePointsNear(const std::vector<InnerPoint>& measurement);

  void UpdatePointsFar(const std::vector<InnerPoint>& measurement);

  void DelPointsFar(const std::vector<InnerPoint>& measurement);

  // local_measurement_lines_ 用于储存当前帧观测数据
  boost::circular_buffer<std::vector<InnerPoint>> latest_measurement_lines_;

  Eigen::Matrix<double, 40, 1> X_, X_SWAP_;   // 跟踪点
  Eigen::Matrix<double, 40, 40> P_, P_SWAP_;  // 状态方差

  float init_p_ = 1;

  Eigen::Vector3d fastest_track_pt_;
  Eigen::Vector3d near_track_pt_;
  Eigen::Vector3d near_measure_pt_;
  Eigen::Vector3d far_measure_pt_;
  Eigen::Affine3d pose_;
  int64_t pt_size_ = 20;
  double threshold_ = 0.0;

  int update_point_max_num_ = 2;

  float dis_gap_thresh_ = 0.5;
};

template <typename Element>
void AdaptorPointManager::AddObservePoints(
    const std::shared_ptr<Element>& measurement) {
  std::vector<InnerPoint> vec_pts;
  for (int64_t i = 0; i < measurement->vehicle_points.size(); ++i) {
    vec_pts.push_back(
        {measurement->vehicle_points[i], measurement->world_points[i]});
  }
  latest_measurement_lines_.push_back(vec_pts);
  const auto& delta_pose = PoseManager::Instance()->GetDeltaPose();
  for (auto& lane_line : latest_measurement_lines_) {
    for (auto& inner_pt : lane_line) {
      inner_pt.vehicle_point = delta_pose * inner_pt.vehicle_point;
    }
    if (lane_line.front().vehicle_point.x() >
        lane_line.back().vehicle_point.x()) {
      std::reverse(lane_line.begin(), lane_line.end());
    }
  }
}

using AdaptorPointManagerPtr = std::shared_ptr<AdaptorPointManager>;
using AdaptorPointManagerConstPtr = std::shared_ptr<const AdaptorPointManager>;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
