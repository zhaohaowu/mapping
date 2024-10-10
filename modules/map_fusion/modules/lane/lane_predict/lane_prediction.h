/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： lane_prediction.h
 *   author     ： xuliang
 *   date       ： 2024.08
 ******************************************************************************/
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/map_fusion/base/element_base.h"
#include "modules/map_fusion/base/group.h"
#include "modules/map_fusion/base/processor.h"
#include "modules/map_fusion/common/common_data.h"

namespace hozon {
namespace mp {
namespace mf {

class LanePrediction : ProcessorBase {
 public:
  LanePrediction() = default;

  ~LanePrediction() = default;

  bool Init(const LaneFusionProcessOption& conf);

  void SetEgoLaneId();

  void ComputeHeadingCompensation(const KinePosePtr& curr_pose);

  bool LaneForwardPredict(std::vector<Group::Ptr>* groups, const double& stamp);

  void Clear() override;

 private:
  bool LaneLineNeedToPredict(const LineSegment& line, bool check_back = true);
  void ComputeLineHeadingPredict(
      std::vector<Group::Ptr>* groups,
      std::vector<LineSegment::Ptr>* lines_need_pred);
  void HeadingCluster(const std::vector<Lane::Ptr>& lanes_need_pred,
                      std::vector<LineSegment::Ptr>* lines_need_pred,
                      const std::string& last_grp_lines_id, double threshold,
                      bool need_pred_kappa);
  void PredictLaneLine(std::vector<Lane::Ptr>* pred_lane,
                       const Lane::Ptr curr_lane);
  float PointToLaneDis(const Lane::Ptr& lane_ptr, Eigen::Vector3f point);

  LaneFusionProcessOption conf_;
  double delta_pose_heading_ = 0.;
  EgoLane ego_lane_id_;

  KinePosePtr last_pose_ = nullptr;
};

using LanePredictionPtr = std::unique_ptr<LanePrediction>;

}  // namespace mf
}  // namespace mp
}  // namespace hozon
