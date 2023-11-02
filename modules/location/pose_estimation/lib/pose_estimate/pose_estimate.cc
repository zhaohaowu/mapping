/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： pose_estimate.cc
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/location/pose_estimation/lib/pose_estimate/pose_estimate.h"

#include <memory>
#include <vector>

namespace hozon {
namespace mp {
namespace loc {

MapMatch::MapMatch() {
  lane_line_ = std::make_shared<MatchLaneLine>();
  //   road_edge = std::make_shared<MatchRoadEdge>();
  //   pole = std::make_shared<MatchPole>();
  //   traffic_sign = std::make_shared<MatchTrafficSign>();
}

void MapMatch::SetInsTs(const double &ins_ts) {
  ins_timestamp_ = ins_ts;
  // road_edge->SetTimeStamp(ins_timestamp_);
}

bool MapMatch::GoodMatchCheck(const SE3 &T) {
  return lane_line_->CheckIsGoodMatch(T);
}

void MapMatch::Match(const HdMap &hd_map,
                     const std::shared_ptr<Perception> &perception,
                     const SE3 &T02_W_V) {
  lane_line_->set_ins_ts(ins_timestamp_);
  lane_line_->Match(hd_map, perception, T02_W_V);
  connect_.lane_line_match_pairs = lane_line_->get_match_pairs();
  HLOG_INFO << "CONNECT LANE LINE SIZE: "
            << connect_.lane_line_match_pairs.size();
  // road_edge->match(hd_map, perception, T02_W_V);
}

// void MapMatch::Match3D(const HDMap &hd_map, const Tracking &track,
//                        const SE3 &T02_W_V) {
//   auto pole3d = track.pole3D();
//   auto traffic_sign_3d = track.trafficSign3D();

//   connect_._pole = pole->match3D(hd_map, pole3d, T02_W_V);
//   HLOG_INFO << "CONNECT POLE SIZE: " << connect_._pole.size();
//   connect_._traffic_sign =
//       traffic_sign->match3D(hd_map, traffic_sign_3d, T02_W_V);
//   HLOG_INFO << "CONNECT TSR SIZE: " << connect_._traffic_sign.size() << " "
//            << traffic_sign_3d.size();
// }

Connect MapMatch::Result(void) { return connect_; }

// CommonState MapMatch::IsInRoadedge(const SE3 &pose, const HDMap &hd_map) {
//   return road_edge->IsInRoadEdge(pose, hd_map);
// }

}  // namespace loc
}  // namespace mp
}  // namespace hozon
