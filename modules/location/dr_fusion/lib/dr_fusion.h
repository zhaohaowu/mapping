/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： dr_fusion.h
 *   author     ： nihongjie
 *   date       ： 2023.09
 ******************************************************************************/
#pragma once
#include <Eigen/Eigen>
#include <deque>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "modules/location/ins_fusion/lib/ins_fusion.h"
#include "proto/dead_reckoning/dr.pb.h"
#include "proto/localization/node_info.pb.h"

namespace hozon {
namespace mp {
namespace loc {

class DrFusion {
 public:
  DrFusion() = default;
  ~DrFusion();
  hozon::mp::loc::InsInitStatus Init(const std::string& dr_configfile);
  void LoadConfigParams(const std::string& configfile);
  void OnInspva(const hozon::localization::HafNodeInfo& inspva_node);
  void OnDr(const hozon::dead_reckoning::DeadReckoning& dr_node);
  bool GetResult(hozon::localization::HafNodeInfo* const node);
  void RunFusion();
  int DrFusionState();

 private:
  bool PublishTopic();
  bool Extract02InsNode(const hozon::localization::HafNodeInfo& origin_node,
                        InsNode* const node);
  bool DrNode2DrFusionNode(
      const hozon::dead_reckoning::DeadReckoning& origin_node,
      hozon::localization::HafNodeInfo* const node);
  void SetRefpoint(const Eigen::Vector3d& blh);
  Eigen::Vector3d GetRefpoint() const;

 private:
  std::unique_ptr<hozon::mp::loc::InsFusion> ins_ = nullptr;
  std::mutex ins_mutex_;
  hozon::localization::HafNodeInfo latest_ins_node_;
  std::mutex dr_mutex_;
  hozon::localization::HafNodeInfo latest_dr_node_;

  bool use_rviz_bridge_ = false;
  bool ref_inspva_node_init_ = false;
  bool use_inspva_ = false;
  bool use_dr_ = false;

  InsNode ref_ins_node_;
  Eigen::Vector3d refpoint_;
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
