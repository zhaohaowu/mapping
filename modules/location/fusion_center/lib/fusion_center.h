/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： fusion_center.h
 *   author     ： lilanxing
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <deque>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <Sophus/se3.hpp>

#include "Eigen/src/Core/Matrix.h"
#include "modules/location/fusion_center/lib/eskf.h"
#include "modules/location/fusion_center/lib/kalman_filter.h"
#include "modules/location/fusion_center/lib/monitor.h"
#include "proto/dead_reckoning/dr.pb.h"
#include "proto/localization/localization.pb.h"
#include "proto/localization/node_info.pb.h"
#include "proto/soc/sensor_imu_ins.pb.h"

namespace hozon {
namespace mp {
namespace loc {
namespace fc {

using hozon::localization::HafNodeInfo;
using hozon::localization::Localization;
using hozon::soc::ImuIns;

class FusionCenter {
 public:
  FusionCenter() = default;
  ~FusionCenter();

  bool Init(const std::string& configfile, const std::string& filterconf,
            const std::string& eskfconf, const std::string& monitorconf);
  void OnImu(const ImuIns& imuins);
  void OnIns(const HafNodeInfo& ins);
  void OnDR(const hozon::dead_reckoning::DeadReckoning& dr);
  void OnInitDR(const Node& initdr);
  void OnPoseEstimate(const HafNodeInfo& pe);
  void SetEhpCounter(int32_t counter);
  int32_t GetEhpCounter() const;
  bool GetCurrentOutput(Localization* const location);

 private:
  bool LoadParams(const std::string& configfile);
  template <typename T>
  void ShrinkQueue(T* const deque, uint32_t maxsize);
  bool ExtractBasicInfo(const HafNodeInfo& msg, Node* const node);
  bool DrToBasicInfo(const hozon::dead_reckoning::DeadReckoning& msg,
                     Node* const node);
  void SetRefpoint(const Eigen::Vector3d& blh);
  Eigen::Vector3d Refpoint();
  void Node2Localization(const Context& ctx, Localization* const location);
  void RunFusion();
  bool PoseInit(const Eigen::Vector3d& refpoint);
  bool GenerateNewESKFPre();   // 用于收集融合的预测
  bool GenerateNewESKFMeas(
      const Eigen::Vector3d& refpoint);  // 用于收集融合的观测
  Node State2Node(const State& state, const Eigen::Vector3d& refpoint);
  void InsertESKFFusionNode(const Node& node);
  void RunESKFFusion(const Eigen::Vector3d& refpoint);
  bool AllowInsMeas(uint32_t sys_status, uint32_t rtk_status);
  bool InsertESKFMeasDR();  // 用于插入DR相对测量值
  bool PredictMMMeas();     // 当无MM测量时，用INS递推MM
  void PruneDeques();
  bool AllowInit(uint32_t sys_status, uint32_t rtk_status);
  bool GetCurrentContext(Context* const ctx);
  static bool IsInterpolable(const std::shared_ptr<Node>& n1,
                             const std::shared_ptr<Node>& n2,
                             double dis_tol = 5.0, double ang_tol = 0.3,
                             double time_tol = 0.5);
  static bool Interpolate(double ticktime,
                          const std::deque<std::shared_ptr<Node>>& d,
                          Node* const node, double dis_tol = 5.0,
                          double ang_tol = 0.3, double time_tol = 0.5);
  static Sophus::SE3d Node2SE3(const Node& node);
  static Sophus::SE3d Node2SE3(const std::shared_ptr<Node>& node);
  void KalmanFiltering(Node* const node, const Eigen::Vector3d& ref_point);

  template <typename T>
  void CutoffDeque(double timestamp, std::deque<std::shared_ptr<T>>* const d);

  // Get blh pose to ctx.global_node
  bool GetGlobalPose(Context* const ctx);
  uint32_t GetGlobalLocationState();

  // Get local pose in local mapping coord to ctx.local_node
  bool GetLocalPose(Context* const ctx);
  std::string GetHdCurrLaneId(const Eigen::Vector3d& utm, double heading);
  uint32_t FaultCodeAssign(uint32_t state);
  void DebugDiffTxt(std::ofstream& diff_file, const Node& compare_node,
                    const Node& ins_node);
  double OrientationToHeading(const Eigen::Vector3d& orientation);
  bool FilterPoseEstimation(const Node& node);
  // mapping trigger
  void CheckTriggerInsStatePose(const std::shared_ptr<Node> i_node);
  bool IsInsDrift(const std::shared_ptr<Node> ins_node);
  bool IsInsStateChange(const std::shared_ptr<Node> node);
  void CheckTriggerLocState(Context* const ctx);

 private:
  Params params_;
  int32_t seq_ = 0;
  int32_t ehp_counter_ = 0;
  Node init_dr_node_;

  std::atomic<bool> fusion_run_{false};
  std::atomic<bool> filter_valid_pe_{false};
  std::atomic<bool> ref_init_{false};

  std::mutex refpoint_mutex_;
  Eigen::Vector3d init_refpoint_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d refpoint_ = Eigen::Vector3d::Zero();

  std::mutex imuins_deque_mutex_;
  std::deque<std::shared_ptr<ImuIns>> imuins_deque_;

  std::mutex ins_deque_mutex_;
  std::deque<std::shared_ptr<Node>> ins_deque_;

  std::mutex dr_deque_mutex_;
  std::deque<std::shared_ptr<Node>> dr_deque_;

  // pe means pose estimation
  std::mutex pe_deque_mutex_;
  std::deque<std::shared_ptr<Node>> pe_deque_;

  std::mutex fusion_deque_mutex_;
  std::deque<std::shared_ptr<Node>> fusion_deque_;

  std::mutex lastest_valid_pe_mutex_;
  Node lastest_valid_pe_;

  std::mutex latest_ins_mutex_;
  HafNodeInfo latest_ins_data_;

  ImuIns prev_imuins_;
  ImuIns curr_imuins_;
  hozon::dead_reckoning::DeadReckoning prev_raw_dr_;
  HafNodeInfo prev_raw_pe_;

  std::shared_ptr<std::thread> th_fusion_ = nullptr;
  std::shared_ptr<ESKF> eskf_ = nullptr;
  KalmanFilter kalman_filter_;
  std::shared_ptr<Monitor> monitor_ = nullptr;

  bool init_ins_ = false;
  bool can_output_ = false;
  double last_meas_time_ = 0.0;
  std::atomic<bool> prev_global_valid_{false};
  std::mutex prev_global_node_mutex_;
  Node prev_global_node_;
  std::atomic<bool> init_dr_{false};

  std::deque<std::shared_ptr<Node>> pre_deque_;
  std::deque<std::shared_ptr<Node>> meas_deque_;
  std::ofstream fc_ins_diff_file_;
  std::ofstream mm_ins_diff_file_;
  std::deque<std::pair<double, double>> lateral_error_compensation_;
  bool lateral_error_compensation_state_ = false;
  double x_lateral_error_ = 0;
  double y_lateral_error_ = 0;
  bool init_dr_ins_ = false;
  Node init_ins_node_;
  Eigen::Vector3d kalman_blh_;

  // mapping trigger
  std::deque<std::shared_ptr<Node>> ins_trig_deque_;
};

}  // namespace fc
}  // namespace loc
}  // namespace mp
}  // namespace hozon
