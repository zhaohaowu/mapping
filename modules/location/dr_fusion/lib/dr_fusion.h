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

enum class DrInitStatus : uint32_t { OK = 0, CONFIG_NOT_FOUND = 1 };

class DrFusion {
 public:
  DrFusion() = default;
  ~DrFusion() = default;
  hozon::mp::loc::DrInitStatus Init(const std::string& dr_configfile);
  void LoadConfigParams(const std::string& configfile);
  void OnInsFusion(const hozon::localization::HafNodeInfo& ins_fusion_node);
  void OnDr(const hozon::dead_reckoning::DeadReckoning& dr_node);
  bool GetResult(hozon::localization::HafNodeInfo* const node);
  void RunFusion();
  int DrFusionState() const;

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
  std::mutex ins_fusion_mutex_;
  hozon::localization::HafNodeInfo latest_ins_fusion_node_;
  std::mutex dr_mutex_;
  hozon::localization::HafNodeInfo latest_dr_node_;

  bool use_rviz_bridge_ = false;
  bool ref_ins_fusion_node_init_ = false;
  bool use_ins_fusion_ = false;
  bool use_dr_ = false;

  InsNode ref_ins_fusion_node_;
  Eigen::Vector3d refpoint_;
  bool init_ = false;
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
