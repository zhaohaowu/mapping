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

#include "modules/location/ins_fusion/lib/defines.h"
#include "modules/location/ins_fusion/lib/smoother.h"
#include "proto/localization/node_info.pb.h"
#include "proto/soc/sensor_imu_ins.pb.h"
#include <Sophus/se3.hpp>

namespace hozon {
namespace mp {
namespace loc {


class InsFusion {
 public:
  InsFusion() = default;
  ~InsFusion();

  InsInitStatus Init(const std::string& configfile);
  bool OnOriginIns(const hozon::soc::ImuIns& origin_ins,
                   hozon::localization::HafNodeInfo* const node_info);
  bool OnInspva(const hozon::localization::HafNodeInfo& inspva,
                hozon::localization::HafNodeInfo* const node_info);
  void SetRefpoint(const Eigen::Vector3d& blh);
  Eigen::Vector3d GetRefpoint() const;
  bool InsFusionState();
  void Convert(const hozon::soc::ImuIns& origin_ins, hozon::localization::HafNodeInfo* const node_info);

 private:
  void LoadConfigParams(const std::string& configfile);
  void AccumulateGpsStatus(const hozon::localization::HafNodeInfo& inspva);
  bool Extract02InsNode(const hozon::localization::HafNodeInfo& inspva,
                        InsNode* const node);
  bool Extract84InsNode(const hozon::soc::ImuIns& ins, InsNode* const node);
  bool CoordSameInPlanar(const Eigen::Vector3d& c1, const Eigen::Vector3d& c2);
  bool FixDeflectionRepeat(const InsNode& prev_node, InsNode* const curr_node);
  bool SimultaneousWgs84With02(const InsNode& gcj02, InsNode* const wgs84);
  Eigen::Vector3d PredictEN(const double& t, const InsNode& node) const;

  bool SmoothProc(InsNode* const node);
  bool PublishTopic();
  void CheckTriggerInsTime(const hozon::soc::ImuIns& cur_ins);
  double QuaternionToHeading(const Eigen::Quaterniond& q);

 private:
  Config config_;
  bool ref_init_ = false;
  Eigen::Vector3d refpoint_;

  std::mutex inspva_mutex_;
  hozon::soc::ImuIns latest_origin_ins_;
  hozon::localization::HafNodeInfo latest_inspva_data_;
  std::mutex origin_ins_mutex_;
  std::mutex origin_ins_mutex2_;
  std::deque<hozon::soc::ImuIns> latest_origin_ins_deque_;
  hozon::localization::HafNodeInfo curr_output_;

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
  bool ins_node_is_valid_ = false;
  bool inspva_node_is_valid_ = false;
  bool init_send_ins_fusion_ = false;

  // store origin ins from IMU/INS message
  std::mutex ins84_deque_mutex_;
  std::deque<InsNode> ins84_deque_;
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
