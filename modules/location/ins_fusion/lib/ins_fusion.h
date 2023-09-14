/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： ins_fusion.h
 *   author     ： nihongjie
 *   date       ： 2023.09
 ******************************************************************************/
#pragma once

#include <Eigen/Eigen>
#include <deque>
#include <future>
#include <iomanip>
#include <memory>
#include <string>

#include "adsfi_proto/sensors/sensors_ins.pb.h"
#include "depend/proto/localization/localization.pb.h"
#include "interface/adsfi_proto/internal/node_info.pb.h"
#include "modules/location/ins_fusion/lib/defines.h"
#include "modules/location/ins_fusion/lib/smoother.h"

namespace hozon {
namespace mp {
namespace loc {

enum InsStateEnum { NORMAL, MILD, SERIOUSLY };

class InsFusion {
 public:
  InsFusion() = default;
  ~InsFusion();

  InsInitStatus Init(const std::string& configfile);
  void OnOriginIns(const adsfi_proto::hz_Adsfi::AlgInsInfo& origin_ins);
  void OnInspva(const adsfi_proto::internal::HafNodeInfo& inspva);
  bool GetResult(adsfi_proto::internal::HafNodeInfo* const node_info);
  bool GetLocalizationEstimate(
      hozon::localization::LocalizationEstimate* const estimate);
  void SetRefpoint(const Eigen::Vector3d& blh);
  Eigen::Vector3d GetRefpoint() const;
  void ProcessMonitorIns();

 private:
  void LoadConfigParams(const std::string& configfile);
  void AccumulateGpsStatus(const adsfi_proto::internal::HafNodeInfo& inspva);
  double ToSeconds(const uint32_t& sec, const uint32_t& nsec);
  bool Extract02InsNode(const adsfi_proto::internal::HafNodeInfo& inspva,
                        InsNode* const node);
  bool Extract84InsNode(const adsfi_proto::hz_Adsfi::AlgInsInfo& ins,
                        InsNode* const node);
  bool CoordSameInPlanar(const Eigen::Vector3d& c1, const Eigen::Vector3d& c2);
  bool FixDeflectionRepeat(const InsNode& prev_node, InsNode* const curr_node);
  bool SimultaneousWgs84With02(const InsNode& gcj02, InsNode* const wgs84);
  Eigen::Vector3d PredictEN(const double& t, const InsNode& node) const;

  bool SmoothProc(InsNode* const node);
  bool PublishTopic();

 private:
  Config config_;
  uint32_t seq_ = 0;
  bool ref_init_ = false;
  Eigen::Vector3d refpoint_;

  std::mutex inspva_mutex_;
  adsfi_proto::internal::HafNodeInfo latest_inspva_data_;
  std::mutex origin_ins_mutex_;
  adsfi_proto::hz_Adsfi::AlgInsInfo latest_origin_ins_;
  adsfi_proto::internal::HafNodeInfo curr_output_;

  Eigen::Vector3d pos_china_ref_ = Eigen::Vector3d::Zero();
  std::unique_ptr<Smoother> smoother_ = nullptr;

  double gps_1_last_ = 0.0;
  double gps_2_last_ = 0.0;
  double gps_3_last_ = 0.0;
  double gps_5_last_ = 0.0;
  double gps_1_last_with_mm_ = 0.0;
  double gps_2_last_with_mm_ = 0.0;
  double gps_3_last_with_mm_ = 0.0;
  double gps_5_last_with_mm_ = 0.0;

  InsNode last_node_;
  InsNode curr_node_;

  // store origin ins from IMU/INS message
  std::mutex ins84_deque_mutex_;
  std::deque<InsNode> ins84_deque_;
  std::chrono::steady_clock::time_point last_timestamp_;
  InsStateEnum ins_state_enum_{InsStateEnum::NORMAL};
  unsigned int loss_ins_frame_id_ = 0;
  std::mutex inspva_deque_mutex_;
  std::deque<adsfi_proto::internal::HafNodeInfo> inspva_deque_;
  std::future<void> monitor_ins_proc_;
  std::atomic<bool> monitor_ins_proc_run_{false};
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
