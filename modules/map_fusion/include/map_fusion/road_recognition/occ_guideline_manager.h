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

    exit_lane_info_.exist = false;
  }

  void Process(const em::ElementMap::Ptr& ele_map,
               std::map<int, std::vector<Line_kd>>* bev_lanelines,
               const KinePose& curr_pose, const RoadScene& road_scene);

  inline em::ExitLaneInfo GetExitLaneInfo() { return exit_lane_info_; }
  inline std::map<int, em::Boundary::Ptr> GetStableOccRoads() {
    return stable_occ_roads_;
  }

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
    ego_bevlines_x_dis_.clear();
    exit_lane_info_.exist = false;
  }

 protected:
  void ReplaceOccLocByBevRoadEdge();
  std::vector<em::Boundary::Ptr> GetBevLaneLineInOcc(
      const std::vector<em::Boundary::Ptr>& stable_occ);
  std::vector<em::Boundary> FineTuneGuideLine(
      const std::vector<em::Boundary::Ptr>& bev_linelines,
      const std::vector<em::Boundary::Ptr>& stable_occ);
  void FineTuneOccPair(std::vector<em::Boundary::Ptr> best_occ_pair);

  bool IsLineBetweenOcc(const em::Boundary::Ptr& line,
                        const std::vector<em::Boundary::Ptr>& stable_occ);
  bool IsNearbyBevLane(const em::Boundary::Ptr& virtual_line);
  std::vector<em::Boundary> OccRegionSplit(
      const std::vector<em::Boundary::Ptr>& best_occ_pair);

  Eigen::Vector3f CalcuDirectionVec(em::Boundary::Ptr right_occ,
                                    em::Boundary::Ptr left_occ);
  double ComputeCurvature(const em::Boundary::Ptr& boundary_ptr);
  void RecordExitLaneInfos();
  void SetStableOcc();
  std::vector<em::Boundary::Ptr> GetStableOcc();
  std::vector<std::vector<em::Boundary::Ptr>> GetStableOccGroups();
  void AddCurrentMeasurementOcc();
  bool CheckOccWhetherStable();
  std::vector<em::Boundary> InferGuideLineOnlyByOcc(
      const std::vector<em::Boundary::Ptr>& stable_occ);
  std::vector<std::pair<em::Id, em::OccRoad::Ptr>> GetFrontOccRoadPair();
  std::vector<std::pair<em::Id, em::OccRoad::Ptr>> GetBestOccPair(
      const std::vector<std::pair<em::Id, em::OccRoad::Ptr>>& front_occ_set);

  std::vector<std::vector<bool>> ConstructPairTable();
  std::vector<std::pair<em::Id, em::OccRoad::Ptr>> GetBestOccPair(
      std::vector<std::vector<bool>>* pair_table);
  bool CanOccPairOtherOcc(const em::OccRoad::Ptr& occ_i,
                          const em::OccRoad::Ptr& occ_j);
  double GetOccWidth(const em::OccRoad::Ptr& right_occ,
                     const em::OccRoad::Ptr& left_occ);
  void FilterBadOccByVertical(std::vector<std::vector<bool>>* pair_table);
  void FilterBadOccByHorizon(std::vector<std::vector<bool>>* pair_table);
  void CloseOccPairStatus(std::vector<std::vector<bool>>* pair_table,
                          int occ_index);
  std::vector<std::pair<em::Id, em::OccRoad::Ptr>> SelectBestOccPair(
      const std::vector<std::vector<bool>>* pair_table);

  float GetExitLaneWidth();

  std::vector<float> GetBevLineXdisOfExitAndEntrance();

  float GetEntranceLaneWidth(
      const std::vector<em::Boundary::Ptr>& bev_lanelines);

  void ClearBadOccByBevLinePosition();

  em::Boundary::Ptr TransformPbLine2Boundary(
      const std::shared_ptr<hozon::mapping::LaneLine>& pb_line);
  std::shared_ptr<hozon::mapping::LaneLine> TransformBoundary2PbLine(
      const em::Boundary::Ptr& line);

  float GetTwoBoundayDis(const em::Boundary::Ptr& left_boundary,
                         const em::Boundary::Ptr& right_boundary);
  float GetTwoBoundaryMinDis(const em::Boundary::Ptr& left_boundary,
                             const em::Boundary::Ptr& right_boundary);
  float GetTwoBoundayDis(const em::OccRoad::Ptr& left_occ,
                         const em::OccRoad::Ptr& right_occ);

  void DebugPrint(const std::string tag,
                  const std::vector<std::vector<bool>>& pair_table, bool show);

 private:
  bool ComputerPointIsInLine(const Eigen::Vector3f& P, const Eigen::Vector3f& A,
                             const Eigen::Vector3f& B);

  bool IsPointOnOccLeftSide(const em::BoundaryNode::Ptr& node,
                            const em::Boundary::Ptr& occ_data);

  bool IsPointOnOccRightSide(const em::BoundaryNode::Ptr& node,
                             const em::Boundary::Ptr& occ_data);

  double CalcLaneLength(const em::OccRoad::Ptr& lane);

  em::Boundary::Ptr TransVehicleToLocal(const em::OccRoad::Ptr& occ);

  double GetCurrentTime() {
    std::chrono::time_point<std::chrono::system_clock,
                            std::chrono::microseconds>
        tpMicro = std::chrono::time_point_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now());

    // (微秒精度的)时间点 => (微秒精度的)时间戳
    time_t totalMicroSeconds = tpMicro.time_since_epoch().count();
    return totalMicroSeconds;
  }

  em::ElementMap::Ptr input_ele_map_ = nullptr;
  std::map<int, std::vector<Line_kd>>* bevlanelines_ = nullptr;
  std::vector<std::pair<int, em::Boundary::Ptr>> bev_laneline_boundarys_;
  std::vector<em::OccRoad::Ptr> input_occ_set_;
  bool is_occ_stable_state_ = false;  //
  float safe_distance_ = 0.2;
  float x_distance_error_thresh_ = 4.0;
  float y_distance_error_thresh_ = 0.8;
  float virtual_occ_line_length_thresh_ = 10.0;
  float occ_pair_dis_thresh_ = 5.8;
  KinePose last_pose_;
  KinePose curr_pose_;

  // 超过 20条就要考虑移除部分. 
  struct OccInfo{
    em::Boundary::Ptr boundary_ptr;
    double timestamp;
    double length;
    int num;
    em::OccRoad::Ptr occ_ptr;

    bool operator()(OccInfo a, OccInfo b) {
        return a.timestamp < b.timestamp;
    }
  };
  std::vector<OccInfo> pre_occ_buffer_;

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
  std::vector<std::vector<em::Boundary::Ptr>> stable_occ_groups_;

  std::vector<float> ego_bevlines_x_dis_;
  em::ExitLaneInfo exit_lane_info_;
  std::map<int, em::Boundary::Ptr> stable_occ_roads_;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
