/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-08-31
 *****************************************************************************/
#include "modules/local_mapping/lib/local_mapping.h"

namespace hozon {
namespace mp {
namespace lm {

LMapApp::LMapApp() {
  laneOp_ = std::make_shared<LaneOp>();
  mmgr_ = std::make_shared<MapManager>();
  map_lanes_ = std::make_shared<std::vector<LocalMapLane>>();
  latest_location_ = std::make_shared<Location>();
  latest_dr_ = std::make_shared<Location>();
  latest_lanes_ = std::make_shared<Lanes>();
  lane_matches_ = std::make_shared<std::vector<LaneMatchInfo>>();
  new_lane_pts_ = std::make_shared<std::vector<Eigen::Vector3d>>();
  map_init_timestamp_ = 0;
  init_T_ = Eigen::Matrix4d::Identity();
  lasted_T_ = Eigen::Matrix4d::Identity();
  mmgr_->Init();
}

void LMapApp::OnLocation(
    const std::shared_ptr<const adsfi_proto::hz_Adsfi::AlgLocation> &msg) {
  DataConvert::SetLocation(*msg, latest_dr_);
}

void LMapApp::OnDr(
    const std::shared_ptr<const adsfi_proto::hz_Adsfi::AlgLocation> &msg) {
  DataConvert::SetDr(*msg, latest_location_);
  if (map_init_timestamp_ < 1e-5) {
    map_init_timestamp_ = latest_location_->timestamp_;
    init_T_.block<3, 3>(0, 0) =
        latest_location_->quaternion_.toRotationMatrix();
    init_T_.block<3, 1>(0, 3) = latest_location_->position_;
  }
  lasted_T_.block<3, 3>(0, 0) =
      latest_location_->quaternion_.toRotationMatrix();
  lasted_T_.block<3, 1>(0, 3) = latest_location_->position_;
  T_V_W_ = init_T_.inverse() * lasted_T_;
}

void LMapApp::OnLaneLine(
    const std::shared_ptr<const adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray>
        &msg) {
  HLOG_INFO << "OnLaneLine";

  DataConvert::SetLaneLine(*msg, latest_lanes_);
  mmgr_->GetLanes(map_lanes_);
  // match current frame lanes to map lanes
  laneOp_->Match(latest_lanes_, map_lanes_, lane_matches_);
  for (auto match : *lane_matches_) {
    if (match.update_type_ == ObjUpdateType::ADD_NEW) {
      CommonUtil::SampleLanePoints(match.frame_lane_, T_V_W_, new_lane_pts_);
      mmgr_->AppendLanePoints(match.frame_lane_->lane_id_, *new_lane_pts_);
    } else if (match.update_type_ == ObjUpdateType::MERGE_OLD) {
      laneOp_->FilterCurve(match.map_lane_, match.frame_lane_, new_lane_pts_);
      mmgr_->AppendLanePoints(match.map_lane_->track_id_, *new_lane_pts_);
    }
  }
}

void LMapApp::OnRoadEdge(
    const std::shared_ptr<const adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray>
        &msg) {}

bool LMapApp::FetchLocalMap(std::shared_ptr<LocalMap> local_map) {
  return true;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
