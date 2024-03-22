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
#include "modules/local_mapping/lib/filter/point_manager.h"
#include "modules/local_mapping/lib/interface/base_init_options.h"
#include "modules/local_mapping/lib/target/base_target.h"
#include "modules/local_mapping/utils/curve_fitter.h"

namespace hozon {
namespace mp {
namespace lm {

class RoadEdgePointFilter {
 public:
  explicit RoadEdgePointFilter(RoadEdgeTargetPtr lane_target)
      : target_ref_(std::move(lane_target)), curve_fit_(3, 40) {}
  virtual ~RoadEdgePointFilter() = default;
  RoadEdgePointFilter(const RoadEdgePointFilter&) = delete;
  RoadEdgePointFilter& operator=(const RoadEdgePointFilter&) = delete;

  bool Init(const FilterInitOption& init_options);

  void UpdateWithMeasurement(const FilterOption& filter_options,
                             const RoadEdgePtr& measurement);

  void UpdateWithoutMeasurement(const FilterOption& filter_options);

  bool IsAbnormalPose(const Eigen::Affine3d& novatel2world_pose);

  void Reset();

 private:
  void UpdateStage(const RoadEdgePtr& measurement_line);

  void PredictStage();

  void UpdateResult(bool match_flag);

  bool ConvertPointSet2Eigen(const std::vector<Eigen::Vector3d>& point_set,
                             Eigen::Matrix<double, 40, 1>* eigen_vector);

  bool ConvertEigen2PointSet(const Eigen::Matrix<double, 40, 1>& eigen_vector,
                             const RoadEdgePtr& track_roadedge);
  void MergeMapTrackLanePoints();

  void CalculateNormal();
  void CalculateNormalV2();

  void CreateAMatrix();

  void RevisePredictFit();

  void KalmanUpdate(const std::vector<Eigen::Vector3d>& measurement_points);

 private:
  RoadEdgeTargetPtr target_ref_;
  AdaptorPointManagerPtr point_manager_;

  // todo 配置化参数
  // LanePointFilterParam lane_point_filter_param_;
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
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
