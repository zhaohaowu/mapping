/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： fusion_center.h
 *   author     ： lilanxing
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <string>
#include <memory>
#include <deque>

#include "proto/soc/sensor_imu_ins.pb.h"
#include "proto/localization/node_info.pb.h"
#include "proto/localization/localization.pb.h"
#include "modules/location/fusion_center/lib/defines.h"

namespace hozon {
namespace mp {
namespace loc {
namespace fc {

using hozon::soc::ImuIns;
using hozon::localization::HafNodeInfo;
using hozon::localization::Localization;

class FusionCenter {
 public:
  FusionCenter() = default;
  ~FusionCenter() = default;

  bool Init(const std::string& configfile);
  void OnImu(const ImuIns& imuins);
  void OnIns(const HafNodeInfo& ins);
  void OnDR(const HafNodeInfo& dr);
  void OnPoseEstimate(const HafNodeInfo& pe);
  void SetEhpCounter(uint32_t counter);
  uint32_t GetEhpCounter() const;
  void SetCoordInitTimestamp(double t);
  bool GetCurrentOutput(Localization* const location);

 private:
  bool LoadParams(const std::string& configfile);
  template <typename T>
  void ShrinkQueue(T* const deque, uint32_t maxsize);
  bool ExtractBasicInfo(const HafNodeInfo& msg, Node* const node);
  void SetRefpoint(const Eigen::Vector3d& blh);
  const Eigen::Vector3d Refpoint();
  void Node2AlgLocation(const Context& ctx, Localization* const location);

 private:
  Params params_;
  uint32_t seq_ = 0;
  uint32_t ehp_counter_ = 0;
  double coord_init_timestamp_ = -1;

  std::mutex refpoint_mutex_;
  Eigen::Vector3d init_refpoint_ = Eigen::Vector3d::Zero();
  bool ref_init_ = false;
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

  ImuIns prev_imuins_;
  ImuIns curr_imuins_;
  HafNodeInfo prev_raw_ins_;
  HafNodeInfo curr_raw_ins_;
  HafNodeInfo prev_raw_dr_;
  HafNodeInfo prev_raw_pe_;
};

}  // namespace fc
}  // namespace loc
}  // namespace mp
}  // namespace hozon
