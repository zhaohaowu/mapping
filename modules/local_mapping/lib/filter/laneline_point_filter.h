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

#include "boost/circular_buffer.hpp"
#include "depend/proto/local_mapping/local_map.pb.h"
#include "modules/local_mapping/lib/filter/point_manager.h"
#include "modules/local_mapping/lib/interface/base_init_options.h"
#include "modules/local_mapping/lib/target/base_target.h"
#include "modules/local_mapping/utils/curve_fitter.h"

namespace hozon {
namespace mp {
namespace lm {

using CalculateStatus = mapping::LaneStabilityError::CalculateStatus;
struct LaneLineStabilityData {
  CalculateStatus status;
  double offset;
  double heading;
};

class LaneLinePointFilter {
 public:
  explicit LaneLinePointFilter(LaneTargetPtr lane_target)
      : target_ref_(std::move(lane_target)), curve_fit_(3, 40) {
    latest_stability_data_.set_capacity(3);
  }
  virtual ~LaneLinePointFilter() = default;
  LaneLinePointFilter(const LaneLinePointFilter&) = delete;
  LaneLinePointFilter& operator=(const LaneLinePointFilter&) = delete;

  bool Init(const FilterInitOption& init_options);

  void UpdateWithMeasurement(const FilterOption& filter_options,
                             const LaneLinePtr& measurement);

  void UpdateWithoutMeasurement(const FilterOption& filter_options);

  bool IsAbnormalPose(const Eigen::Affine3d& novatel2world_pose);

  void Reset();

 private:
  void UpdateStage(const LaneLinePtr& measurement_line);

  void PredictStage();

  void UpdateResult(bool match_flag);

  bool ConvertPointSet2Eigen(const std::vector<Eigen::Vector3d>& point_set,
                             Eigen::Matrix<double, 40, 1>* eigen_vector);

  bool ConvertEigen2PointSet(const Eigen::Matrix<double, 40, 1>& eigen_vector,
                             const LaneLinePtr& track_lane);
  void MergeMapTrackLanePoints();

  void CalculateNormal();
  void CalculateNormalV2();

  void CreateAMatrix();

  void RevisePredictFit();
  // 基于local系下的点做卡尔曼更新
  void KalmanUpdate(const std::vector<Eigen::Vector3d>& measurement_points);
  // 计算车道线的heading、offset误差
  void calculate_stability_error(
      const std::vector<Eigen::Vector3d>& fit_points);

 private:
  LaneTargetPtr target_ref_;
  AdaptorPointManagerPtr point_manager_;

  // todo 配置化参数
  // LaneLinePointFilterParam lane_point_filter_param_;
  Eigen::Matrix<double, 40, 1> X_;                   // 跟踪点
  Eigen::Matrix<double, 40, 1> X_NORMAL_;            // 跟踪点法向量
  Eigen::Matrix<double, 40, 40> P_;                  // 状态方差
  Eigen::Matrix<double, 40, 40> A_;                  // 状态转移矩阵
  Eigen::Matrix<double, 40, 40> B_;                  // A_的权重矩阵
  Eigen::Matrix<double, 20, 20> R_;                  // 测量方差
  Eigen::Matrix<double, 40, 40> Q_;                  // 预测方差
  Eigen::Matrix<double, 20, 40> H_;                  // 观测矩阵
  Eigen::Matrix<double, 20, 1> HZ_;                  // 实际观测输出
  Eigen::Matrix<double, 40, 40> A_update_;           // 更新A_
  std::vector<Eigen::Vector3d> target_vehicle_pts_;  // 临时保存后处理点

  CurveFitter curve_fit_;

  float init_p_ = 1;
  int pt_size_ = 20;
  float point_sigma_ = 0.3;
  // 记录连续3帧线的拟合状态、offset、heading
  boost::circular_buffer<LaneLineStabilityData> latest_stability_data_;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
