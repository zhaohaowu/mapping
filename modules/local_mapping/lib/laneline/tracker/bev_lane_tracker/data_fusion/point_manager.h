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
#include "modules/local_mapping/lib/laneline/curve_fitter/base_curve_fitter.h"
#include "modules/local_mapping/lib/laneline/proto/lane_postprocess.pb.h"
#include "modules/local_mapping/lib/laneline/tracker/bev_lane_tracker/base/lane_filter.h"
#include "modules/local_mapping/lib/laneline/utils/laneline_polynomial.h"
#include "perception-base/base/laneline/base_laneline.h"

namespace hozon {
namespace mp {
namespace environment {

class AdaptorPointManager {
 public:
  AdaptorPointManager() {
    latest_measurement_lines_.set_capacity(5);
    latest_measurement_lines_.clear();
  }

  ~AdaptorPointManager() {}

  AdaptorPointManager(const AdaptorPointManager&) = delete;
  AdaptorPointManager& operator=(const AdaptorPointManager&) = delete;

  void init(Eigen::Matrix<double, 40, 1>* XPtr,
            Eigen::Matrix<double, 40, 40>* PPtr);

  void CopyMatrix(Eigen::Matrix<double, 40, 1>* XPtr,
                  Eigen::Matrix<double, 40, 40>* PPtr);

  void AddObservePoints(
      const std::vector<perception_base::LaneLinePoint>& point_set);

  void Process(Eigen::Matrix<double, 40, 1>* X,
               Eigen::Matrix<double, 40, 40>* P);

 private:
  void UpdatePointsNear(
      const std::vector<perception_base::LaneLinePoint>& measurement_points);

  void UpdatePointsFar(
      const std::vector<perception_base::LaneLinePoint>& measurement_points);

  void DelPointsFar(
      const std::vector<perception_base::LaneLinePoint>& measurement_points);

  // local_measurement_lines_ 用于储存当前帧观测数据
  boost::circular_buffer<std::vector<perception_base::LaneLinePoint>>
      latest_measurement_lines_;

  Eigen::Matrix<double, 40, 1> X_, X_SWAP_;   // 跟踪点
  Eigen::Matrix<double, 40, 40> P_, P_SWAP_;  // 状态方差

  float init_p_ = 1;

  Eigen::Vector3d fastest_track_pt_;
  Eigen::Vector3d near_track_pt_;
  perception_base::Point3DF near_measure_pt_;
  perception_base::Point3DF far_measure_pt_;
  Eigen::Affine3d pose_;
  int pt_size_ = 20;
  double threshold_ = 0.0;

  int update_point_max_num_ = 2;

  float dis_gap_thresh_ = 0.5;
};

typedef std::shared_ptr<AdaptorPointManager> AdaptorPointManagerPtr;
typedef std::shared_ptr<const AdaptorPointManager> AdaptorPointManagerConstPtr;

}  // namespace environment
}  // namespace mp
}  // namespace hozon
