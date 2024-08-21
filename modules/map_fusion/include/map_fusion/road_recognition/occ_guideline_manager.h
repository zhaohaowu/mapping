/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： group_map.h
 *   author     ： taoshaoyuan
 *   date       ： 2024.01
 ******************************************************************************/

#pragma once

#include <depend/common/math/vec2d.h>

#include <deque>
#include <map>
#include <memory>
#include <queue>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "boost/circular_buffer.hpp"
#include "map_fusion/fusion_common/common_data.h"
#include "map_fusion/fusion_common/element_map.h"
#include "map_fusion/road_recognition/base_data.h"
#include "opencv2/core/core.hpp"

using hozon::common::math::Vec2d;

namespace hozon {
namespace mp {
namespace mf {

class OccGuideLineManager {
 public:
  OccGuideLineManager() {
    input_ele_map_ = std::make_shared<em::ElementMap>();
    history_n_best_occs_.set_capacity(history_measure_size_);
    history_n_best_occs_.clear();
  }

  void Process(const em::ElementMap::Ptr& ele_map,
               std::map<int, std::vector<Line_kd>>* bev_lanelines,
               const KinePose& curr_pose, const RoadScene& road_scene);

  void Reset() {
    is_occ_stable_state_ = false;
    history_n_best_occs_.clear();
    default_assume_lane_width_ = 3.5;
    averge_exit_lane_width_ = 3.5;
    averge_entrance_lane_width_ = 3.5;
    assume_entrancelane_nums_by_exitlane_ = -1;
    assume_entrancelane_nums_by_entrancelane_ = -1;
    occ_width_ = 0.0;
    occ_dir_vec_.setZero();
    stable_occ_datas_.clear();
  }

 protected:
  void ReplaceOccLocByBevRoadEdge();
  std::vector<em::Boundary::Ptr> GetBevLaneLineInOcc();
  std::vector<em::Boundary> FineTuneGuideLine(
      const std::vector<em::Boundary::Ptr>& bev_linelines);

  void FineTuneOccPair(std::vector<em::Boundary::Ptr> best_occ_pair);

  bool IsLineBetweenOcc(const em::Boundary::Ptr& line);

  std::vector<em::Boundary> OccRegionSplit(
      const std::vector<em::Boundary::Ptr>& best_occ_pair);

  Eigen::Vector3f CalcuDirectionVec(const em::Boundary::Ptr& right_occ,
                                    const em::Boundary::Ptr& left_occ);

  Eigen::Vector3f CalcuDirectionVecV2(em::Boundary::Ptr right_occ,
                                      em::Boundary::Ptr left_occ);

  void SetStableOcc();
  std::vector<em::Boundary::Ptr> GetStableOcc();
  void AddCurrentMeasurementOcc();
  bool CheckOccWhetherStable();
  std::vector<em::Boundary> InferGuideLineOnlyByOcc();
  std::vector<std::pair<em::Id, em::OccRoad::Ptr>> GetFrontOccRoadPair();
  std::vector<std::pair<em::Id, em::OccRoad::Ptr>> GetBestOccPair(
      const std::vector<std::pair<em::Id, em::OccRoad::Ptr>>& front_occ_set);

  double GetOccWidth(const em::OccRoad::Ptr& right_occ,
                     const em::OccRoad::Ptr& left_occ);

  float GetExitLaneWidth();

  float GetEntranceLaneWidth(
      const std::vector<em::Boundary::Ptr>& bev_lanelines);

  em::Boundary::Ptr TransformPbLine2Boundary(
      const std::shared_ptr<hozon::mapping::LaneLine>& pb_line);
  std::shared_ptr<hozon::mapping::LaneLine> TransformBoundary2PbLine(
      const em::Boundary::Ptr& line);

  float GetTwoBoundayDis(const em::Boundary::Ptr& left_boundary,
                         const em::Boundary::Ptr& right_boundary);

 private:
  bool ComputerPointIsInLine(const Eigen::Vector3f& P, const Eigen::Vector3f& A,
                             const Eigen::Vector3f& B);

  float perpendicular_distance(const Eigen::Vector3f& A,
                               const Eigen::Vector3f& B,
                               const Eigen::Vector3f& P);

  bool IsPointOnOccLeftSide(const em::BoundaryNode::Ptr& node,
                            const em::Boundary::Ptr& occ_data);

  bool IsPointOnOccRightSide(const em::BoundaryNode::Ptr& node,
                             const em::Boundary::Ptr& occ_data);

  em::ElementMap::Ptr input_ele_map_ = nullptr;
  std::map<int, std::vector<Line_kd>>* bevlanelines_ = nullptr;
  std::vector<std::pair<int, em::Boundary::Ptr>> bev_laneline_boundarys_;
  bool is_occ_stable_state_ = false;  //
  float safe_distance_ = 0.2;
  float distance_error_thresh_ = 0.8;
  KinePose last_pose_;
  KinePose curr_pose_;

  boost::circular_buffer<std::vector<em::Boundary::Ptr>> history_n_best_occs_;
  int history_measure_size_ = 4;

  float default_assume_lane_width_ = 3.5;
  float averge_exit_lane_width_ = 3.5;
  float averge_entrance_lane_width_ = 3.5;
  int assume_entrancelane_nums_by_exitlane_ = -1;
  int assume_entrancelane_nums_by_entrancelane_ = -1;
  Eigen::Vector3f occ_dir_vec_;
  float occ_width_ = 0.0;
  std::vector<em::Boundary::Ptr> stable_occ_datas_;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
