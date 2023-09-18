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
  has_err_ = false;
  err_type_ = static_cast<int>(ErrorType::NO_ERROR);
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
                     const SE3 &T02_W_V, const SE3 &T_fc,
                     const VP &percep_points, const VP &nearest_map_points) {
  lane_line_->set_ins_ts(ins_timestamp_);
  lane_line_->Match(hd_map, perception, T02_W_V, T_fc, percep_points,
                    nearest_map_points);
  lane_line_->get_error(&has_err_, &err_type_);
  connect_.lane_line_match_pairs = lane_line_->get_match_pairs();
  HLOG_INFO << "CONNECT LANE LINE SIZE: "
            << connect_.lane_line_match_pairs.size();
  // road_edge->match(hd_map, perception, T02_W_V);
  // road_edge->getError(has_err_, err_type_);
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

std::vector<V3> MapMatch::Debug() {
  std::vector<V3> ret;
  ret = lane_line_->get_debug_points();
  if (ret.size() > 0) {
    HLOG_INFO << "get_debug_points size : " << ret.size();
  }

  /*
  // road edge
  auto road_edge_match = road_edge->getDebugPoints();
  if (!road_edge_match.empty())
    ret.insert(ret.end(), road_edge_match.begin(), road_edge_match.end());
  // pole
  auto pole_match = pole->debug3DConnect();
  if (!pole_match.empty())
    ret.insert(ret.end(), pole_match.begin(), pole_match.end());
  // traffic_sign
  auto traffic_sign_match = traffic_sign->debug3DConnect();
  if (!traffic_sign_match.empty())
    ret.insert(ret.end(), traffic_sign_match.begin(), traffic_sign_match.end());
  //
  */

  return ret;
}

std::vector<V3> MapMatch::DebugConnect() {
  std::vector<V3> ret;
  ret = lane_line_->get_debug_points();
  if (ret.size() > 0) {
    HLOG_INFO << "get_connect_debug_points size : " << ret.size();
  }

  return ret;
}

Connect MapMatch::Result(void) { return connect_; }

// CommonState MapMatch::IsInRoadedge(const SE3 &pose, const HDMap &hd_map) {
//   return road_edge->IsInRoadEdge(pose, hd_map);
// }

}  // namespace loc
}  // namespace mp
}  // namespace hozon
