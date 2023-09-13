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

#include "adsfi_proto/internal/node_info.pb.h"
#include "adsfi_proto/location/location.pb.h"
#include "adsfi_proto/sensors/sensors_imu.pb.h"
#include "modules/location/fusion_center/lib/defines.h"

namespace hozon {
namespace mp {
namespace loc {
namespace fc {

using adsfi_proto::hz_Adsfi::AlgIMU;
using adsfi_proto::hz_Adsfi::AlgLocation;
using adsfi_proto::internal::HafNodeInfo;

class FusionCenter {
 public:
  FusionCenter() = default;
  ~FusionCenter();

  bool Init(const std::string& configfile);
  void OnImu(const AlgIMU& imu);
  void OnIns(const HafNodeInfo& ins);
  void OnDR(const HafNodeInfo& dr);
  void OnPoseEstimate(const HafNodeInfo& pe);
  void SetEhpCounter(uint32_t counter);
  uint32_t GetEhpCounter() const;
  void SetCoordInitTimestamp(double t);
  bool GetCurrentOutput(AlgLocation* const location);

 private:
  bool LoadParams(const std::string& configfile);
  template <typename T>
  void ShrinkQueue(T* const deque, uint32_t maxsize);
  bool ExtractBasicInfo(const HafNodeInfo& msg, Node* const node);
  void SetRefpoint(const Eigen::Vector3d& blh);
  const Eigen::Vector3d Refpoint();
  void Node2AlgLocation(const Context& ctx, AlgLocation* const location);

 private:
  Params params_;
  uint32_t seq_ = 0;
  uint32_t ehp_counter_ = 0;
  double coord_init_timestamp_ = -1;

  std::mutex refpoint_mutex_;
  Eigen::Vector3d init_refpoint_ = Eigen::Vector3d::Zero();
  bool ref_init_ = false;
  Eigen::Vector3d refpoint_ = Eigen::Vector3d::Zero();

  std::mutex imu_deque_mutex_;
  std::deque<std::shared_ptr<AlgIMU>> imu_deque_;

  std::mutex ins_deque_mutex_;
  std::deque<std::shared_ptr<Node>> ins_deque_;

  std::mutex dr_deque_mutex_;
  std::deque<std::shared_ptr<Node>> dr_deque_;

  // pe means pose estimation
  std::mutex pe_deque_mutex_;
  std::deque<std::shared_ptr<Node>> pe_deque_;

  AlgIMU prev_imu_;
  AlgIMU curr_imu_;
  HafNodeInfo prev_raw_ins_;
  HafNodeInfo curr_raw_ins_;
  HafNodeInfo prev_raw_dr_;
  HafNodeInfo prev_raw_pe_;
};

}  // namespace fc
}  // namespace loc
}  // namespace mp
}  // namespace hozon
