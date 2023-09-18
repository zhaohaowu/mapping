/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-09-05
 *****************************************************************************/
#include "modules/local_mapping/lib/ops/lane/lane_op.h"

#include <unordered_map>

namespace hozon {
namespace mp {
namespace lm {

void LaneOp::Match(ConstDrDataPtr dr_ptr,
                   std::shared_ptr<const Lanes> cur_lanes,
                   std::shared_ptr<const std::vector<LocalMapLane>> map_lanes,
                   std::shared_ptr<std::vector<LaneMatchInfo>> match_info) {
  HLOG_INFO << "GetMatches";
  match_info->clear();
  if (map_lanes == nullptr || map_lanes->size() == 0) {
    for (auto cur_lane : cur_lanes->front_lanes_) {
      LaneMatchInfo match;
      match.frame_lane_ = std::make_shared<Lane>(cur_lane);
      match.update_type_ = ObjUpdateType::ADD_NEW;
      match_info->emplace_back(match);
    }
    return;
  }

  std::vector<LanePointsPtr> lanes_points;
  for (size_t i = 0; i < cur_lanes->front_lanes_.size(); ++i) {
    LanePointsPtr pts = std::make_shared<std::vector<Eigen::Vector3d>>();
    CommonUtil::SampleLanePointsInLocal(cur_lanes->front_lanes_[i], pts);
    lanes_points.emplace_back(pts);
  }

  Eigen::Vector3d lane_pose = CommonUtil::Get2DPose(dr_ptr);
  LaneAssocOptions lane_assoc_options;
  lane_assoc_.reset(new LaneAssoc(lane_assoc_options));
  std::unordered_map<int, int> map_det_lm =
      lane_assoc_->Process(lanes_points, *map_lanes, lane_pose);

  for (size_t i = 0; i < cur_lanes->front_lanes_.size(); ++i) {
    LaneMatchInfo match;
    match.frame_lane_ = std::make_shared<Lane>(cur_lanes->front_lanes_[i]);
    if (map_det_lm.find(i) != map_det_lm.end()) {
      match.map_lane_ = std::make_shared<LocalMapLane>((*map_lanes)[map_det_lm[i]]);
      match.update_type_ = ObjUpdateType::MERGE_OLD;
    } else {
      match.update_type_ = ObjUpdateType::ADD_NEW;
    }
    match_info->emplace_back(match);
  }
}

void LaneOp::FilterCurve(
    std::shared_ptr<const LocalMapLane> map_lane,
    std::shared_ptr<const Lane> cur_lane,
    std::shared_ptr<std::vector<Eigen::Vector3d>> new_pts) {
  HLOG_INFO << "FilterCurve";
  new_pts->clear();
  CommonUtil::SampleLanePoints(cur_lane, Eigen::Matrix4d::Identity(), new_pts);
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
