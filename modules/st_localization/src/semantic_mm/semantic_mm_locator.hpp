/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */
#pragma once

#include <Eigen/Core>
#include <deque>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Sophus/se3.hpp>
#include <boost/circular_buffer.hpp>
#include <opencv2/opencv.hpp>

#include "base_locator/base_locator.hpp"
#include "eval/evaluator_smm.hpp"
#include "localization/data_type/base.hpp"
#include "localization/data_type/percept_data.hpp"
#include "localization/data_type/road_structure.hpp"

namespace senseAD {
namespace localization {
namespace smm {

class FramePackage;
class MapManager;
class MatchingManager;
class TrackingManager;
class Visualization;

class SemanticMMLocator : public BaseLocator {
 public:
  SemanticMMLocator() = default;
  virtual ~SemanticMMLocator() = default;

  adLocStatus_t Init(const LocalizationParam& param) final;

  adLocStatus_t SetState(const NavState& car_state) final;

  adLocStatus_t GetState(NavState* car_state,
                         double* confidence = nullptr) final;

  adLocStatus_t Restart() final { return LOC_SUCCESS; }

  adLocStatus_t SwitchOriginProc();

  adLocStatus_t SetLocalMap(const RoadStructure::Ptr& road_structure,
                            const NavState& car_state);

  adLocStatus_t Process(
      const std::vector<std::tuple<PerceptData::Ptr, NavState, OdomState>>&
          percept_data,
      bool is_reloc_mode = false);

  // @brief: get smm image for visualization
  cv::Mat GetMMVisImage() const;

  // @brief: set GT(INS) state for visualization
  adLocStatus_t SetGtWithTime(const NavState& nav_state);

  // @brief: set GNSS state for visualization
  adLocStatus_t SetGNSSWithTime(const NavState& nav_state);

 private:
  // @brief: check if need relocalize
  void CheckRelocalizationMode(
      const std::shared_ptr<FramePackage>& frame_package);

  // @brief: main locator process, single frame version
  adLocStatus_t ProcessMainSingleFrame(
      const std::shared_ptr<FramePackage>& frame_package);

  // @brief: main locator process, temporal fusion version
  adLocStatus_t ProcessMainTemporalFusion(
      const std::shared_ptr<FramePackage>& frame_package);

  // @brief: construct eval data
  SMMEvalData ConstructEvalData(
      const std::shared_ptr<FramePackage>& frame_package,
      adLocStatus_t smm_status, double time_cost);

  // @brief: set gt data to frame package for visualizer
  void SetGTDataForVisual(uint64_t timestamp,
                          std::shared_ptr<FramePackage>* frame_package);

  // @brief: linearly interpolate NavState
  bool NavStateInterp(const NavState& s_ns, const NavState& e_ns, double factor,
                      NavState* ns);

  // @brief: query a state in the buffer by timestamp
  adLocStatus_t QueryPoseByTime(uint64_t query_timestamp_ns,
                                const std::deque<NavState>& state_buffer,
                                NavState* output_state);

  // @brief: transfrom GNSS pose from (latitude,longitude,altitude) to (x,y,z)
  SE3d TransformGNSSPose(const SE3d& gnss_pose_lla, const SE3d& Twv);

  // @brief: transform nav/odom state from SE3d to SE2
  void TransformNavStateSE3ToSE2(const NavState& nav_state_se3,
                                 NavState* nav_state_se2);
  void TransformOdomStateSE3ToSE2(const OdomState& odom_state_se3,
                                  OdomState* odom_state_se2);

 private:
  uint64_t main_timestamp_;   // timestamp(ns) for main frame
  NavState init_pose_state_;  // init pose state(car-center) for main frame
  NavState pose_state_;       // refined pose state(car-center) for main frame
  id_t last_odom_origin_id_ = -1;  // dr origin id record

  std::deque<NavState> gnss_state_buffer_;  // gnss pose buffer
  std::deque<NavState> gt_state_buffer_;    // gt (ins) pose buffer

  // buffer map-matching result for reloc check
  boost::circular_buffer<std::pair<uint64_t, Eigen::Vector2d>>
      optim_buffer_;  // optim delta buffer
  boost::circular_buffer<std::pair<uint64_t, SE3d>>
      optim_pose_buffer_;  // optim pose buffer

  const size_t optim_buffer_size_{20};  // optim buffer size
  bool in_mismatch_reloc_{false};       // whether in reloc as map mismatch

  cv::Mat mm_vis_image_;  // map matching visual image
  double map_process_cost_{0};

  // module instance
  std::shared_ptr<MapManager> map_manager_;
  std::shared_ptr<MatchingManager> matching_manager_;
  std::shared_ptr<TrackingManager> tracking_manager_;
  std::shared_ptr<Visualization> visualizer_;

  // evaluator
  std::shared_ptr<EvaluatorSMM> evaluator_ = nullptr;
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
