/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： dr_fusion.h
 *   author     ： nihongjie
 *   date       ： 2023.09
 ******************************************************************************/
#pragma once
#include <Eigen/Eigen>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "interface/adsfi_proto/internal/node_info.pb.h"
#include "modules/location/ins_fusion/lib/ins_fusion.h"

namespace hozon {
namespace mp {
namespace loc {

class DrFusion {
 public:
  DrFusion() = default;
  ~DrFusion();
  hozon::mp::loc::InsInitStatus Init(const std::string& dr_configfile);
  void LoadConfigParams(const std::string& configfile);
  void OnInspva(const adsfi_proto::internal::HafNodeInfo& inspva_node);
  void OnDr(const adsfi_proto::internal::HafNodeInfo& dr_node);
  bool GetResult(adsfi_proto::internal::HafNodeInfo* const dr_node);
  void RunFusion();

 private:
  bool PublishTopic();
  double ToSeconds(const uint32_t& sec, const uint32_t& nsec);
  bool Extract02InsNode(const adsfi_proto::internal::HafNodeInfo& dr_node,
                        InsNode* const node);

 private:
  std::unique_ptr<hozon::mp::loc::InsFusion> ins_ = nullptr;
  std::mutex ins_mutex_;
  adsfi_proto::internal::HafNodeInfo latest_ins_node_;
  std::mutex dr_mutex_;
  adsfi_proto::internal::HafNodeInfo latest_dr_node_;
  std::mutex loc_dr_mutex_;
  adsfi_proto::internal::HafNodeInfo latest_loc_dr_node_;
  bool use_rviz_bridge_{false};
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
