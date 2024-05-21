/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： location.cc
 *   author     ： lilanxing
 *   date       ： 2023.10
 ******************************************************************************/
#include <perception-lib/lib/environment/environment.h>
#include <string>
#include "modules/location/location.h"

namespace hozon {
namespace mp {
namespace loc {

const char* const kCoordAdapterConf =
    "runtime_service/mapping/conf/mapping/location/coord_adapter/config.yaml";
const char* const kDrFusionConf =
    "runtime_service/mapping/conf/mapping/location/dr_fusion/dr_config.yaml";
const char* const kInsFusionConf =
    "runtime_service/mapping/conf/mapping/location/ins_fusion/ins_config.yaml";
const char* const kFcConf =
    "runtime_service/mapping/conf/mapping/location/fusion_center/fc_config.yaml";
const char* const kFcKfConf =
    "runtime_service/mapping/conf/mapping/location/fusion_center/kalman.yaml";
const char* const kFcEskfConf =
    "runtime_service/mapping/conf/mapping/location/fusion_center/eskf.yaml";
const char* const kFcMonitorConf =
    "runtime_service/mapping/conf/mapping/location/fusion_center/monitor.yaml";

bool Localization::Init() {
  const std::string ws = hozon::perception::lib::GetEnv(
      "DEBUG_MAPPING_WORK_ROOT", "/opt/app/1") + "/";

  const std::string ins_fusion_conf(ws + kInsFusionConf);
  ins_fusion_ = std::make_unique<InsFusion>();
  if (ins_fusion_->Init(ins_fusion_conf) != InsInitStatus::OK) {
    return false;
  }

  const std::string fc_conf(ws + kFcConf);
  const std::string fc_kf_conf(ws + kFcKfConf);
  const std::string fc_eskf_conf(ws + kFcEskfConf);
  const std::string fc_monitor_conf(ws + kFcMonitorConf);
  fc_ = std::make_unique<FusionCenter>();
  if (!fc_->Init(fc_conf, fc_kf_conf, fc_eskf_conf, fc_monitor_conf)) {
    return false;
  }

  return true;
}

void Localization::OnDr(const hozon::dead_reckoning::DeadReckoning& dr) {
  // dr_fusion_->OnDr(dr);
  DrFusionPoseProcess(1);
}

void Localization::OnOriginIns(const hozon::soc::ImuIns& origin_ins) {
  hozon::localization::HafNodeInfo ins_fusion_ret;
  bool flag = ins_fusion_->OnOriginIns(origin_ins, &ins_fusion_ret);
  if (!flag) {
    return;
  }
}

void Localization::OnInspva(
    const hozon::localization::HafNodeInfo& inspva_node) {
  hozon::localization::HafNodeInfo ins_fusion_ret;
  bool flag = ins_fusion_->OnInspva(inspva_node, &ins_fusion_ret);
  if (!flag) {
    return;
  }
}

void Localization::OnImu(const hozon::soc::ImuIns& imuins) {
  fc_->OnImu(imuins);
}

void Localization::OnLocalMap(const hozon::mapping::LocalMap& local_map) {
  // wait udpate
}

void Localization::DrFusionPoseProcess(int dr_state) {
  // wait udpate
}

bool Localization::GetCurrentLocalization(
    hozon::localization::Localization* const location) {
  if (!location) {
    return false;
  }
  return fc_->GetCurrentOutput(location);
}

}  // namespace loc
}  // namespace mp
}  // namespace hozon
