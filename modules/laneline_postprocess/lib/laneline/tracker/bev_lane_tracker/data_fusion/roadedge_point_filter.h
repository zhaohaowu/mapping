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
#include "modules/laneline_postprocess/lib/laneline/curve_fitter/base_curve_fitter.h"
#include "modules/laneline_postprocess/lib/laneline/proto/lane_postprocess.pb.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/base/lane_filter.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/base/roadedge_target.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/data_fusion/laneline_point_filter.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/data_fusion/point_manager.h"
#include "modules/laneline_postprocess/lib/laneline/utils/laneline_polynomial.h"
#include "perception-base/base/laneline/base_laneline.h"

namespace hozon {
namespace mp {
namespace environment {

namespace perception_base = hozon::perception::base;

class RoadEdgePointFilter : public BaseRoadEdgeFilter {
 public:
  explicit RoadEdgePointFilter(RoadEdgeTargetPtr lane_target)
      : BaseRoadEdgeFilter(lane_target) {}
  virtual ~RoadEdgePointFilter() {}
  RoadEdgePointFilter(const RoadEdgePointFilter&) = delete;
  RoadEdgePointFilter& operator=(const RoadEdgePointFilter&) = delete;

  bool Init(const LanePointFilterInitOptions& init_options);

  void UpdateWithMeasurement(
      const LaneFilterOptions& filter_options,
      const perception_base::RoadEdgeMeasurementPtr& measurement);

  void UpdateWithoutMeasurement(const LaneFilterOptions& filter_options);

  bool IsAbnormalPose(const Eigen::Affine3d& novatel2world_pose);

  void Reset();

 private:
  void UpdateStage(
      const perception_base::RoadEdgeMeasurementPtr& measurement_line);

  void PredictStage();

  void UpdateResult();

  bool ConvertPointSet2Eigen(
      const std::vector<perception_base::LaneLinePoint>& point_set,
      Eigen::Matrix<double, 40, 1>* eigen_vector);

  bool ConvertEigen2PointSet(
      const Eigen::Matrix<double, 40, 1>& eigen_vector,
      std::vector<perception_base::LaneLinePoint>* point_set);

  void CalculateNormal();
  void CalculateNormalV2();

  void CreateAMatrix();

  void RevisePredictFit();

  void KalmanUpdate(
      const std::vector<perception_base::LaneLinePoint>& measurement_points);

 private:
  AdaptorPointManagerPtr point_manager_;

  // base::LaneLineMeasurementPtr local_measurement_lines_;
  LanePointFilterParam lane_point_filter_param_;
  Eigen::Matrix<double, 40, 1> X_;          // 跟踪点
  Eigen::Matrix<double, 40, 1> X_NORMAL_;   // 跟踪点法向量
  Eigen::Matrix<double, 40, 40> P_;         // 状态方差
  Eigen::Matrix<double, 40, 40> A_;         // 状态转移矩阵
  Eigen::Matrix<double, 40, 40> B_;         // A_的权重矩阵
  Eigen::Matrix<double, 20, 20> R_;         // 测量方差
  Eigen::Matrix<double, 40, 40> Q_;         // 预测方差
  Eigen::Matrix<double, 20, 40> H_;         // 观测矩阵
  Eigen::Matrix<double, 20, 1> HZ_;         // 实际观测输出
  Eigen::Matrix<double, 40, 40> A_update_;  // 更新A_

  std::shared_ptr<BaseCurveFitter> pyfilt_ = nullptr;
  LaneLinePolynomialPtr polynomial_ = nullptr;

  float init_p_ = 1;
  int pt_size_ = 20;
  float point_sigma_ = 0.3;
};

}  // namespace environment
}  // namespace mp
}  // namespace hozon
