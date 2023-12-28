/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-11-29
 *****************************************************************************/

#include "onboard/onboard_lite/phm_comment_lite/phm_commenet.h"
#include "lib/fault_manager/fault_manager.h"
#include "lib/health_manager/health_manager.h"
#include "lib/io/file_util.h"

namespace hozon {
namespace perception {
namespace common_onboard {

PhmComponent::PhmComponent() {
  phm_client_ = std::make_shared<PHMClient>();
}

PhmComponent::~PhmComponent() {
  if (!faultmap_.empty()) {
    faultmap_.clear();
  }
  if (!trigger_.empty()) {
    trigger_.clear();
  }
  if (phm_client_) {
    phm_client_->Stop();
    phm_client_->Deinit();
  }
}

bool PhmComponent::Init() {
  HLOG_INFO << "[DEBUG_PHM] ======================================: " << Name();
  config_manager_ = lib::ConfigManager::Instance();
  if (config_manager_ != nullptr) {
    if (!config_manager_->GetModelConfig(Name(), &model_config_)) {
      HLOG_ERROR << "[DEBUG_PHM] Parse config failed! Name: " << Name();
      return false;
    }
  }
  work_root_ = config_manager_->work_root();
  // 初始化故障管理
  InitFault();
  // 初始化健康管理
  InitHealth();
  std::string yaml_file = "yaml_file";
  GetFilePath(yaml_file);
  // 开启平台phm健康监控
  if (phm_client_) {
    phm_client_->Init(yaml_file,
                      std::bind(&PhmComponent::ServiceAvailableCallback, this,
                                std::placeholders::_1),
                      std::bind(&PhmComponent::FaultReceiveCallback, this,
                                std::placeholders::_1));
    phm_client_->Start();
  }
  return true;
}

void PhmComponent::GetFilePath(std::string& path) {
  if (!model_config_->get_value(path, &path)) {
    HLOG_ERROR << "[DEBUG_PHM] Get root path failed!";
    return;
  }

  path = lib::FileUtil::GetAbsolutePath(work_root_, path);
  HLOG_INFO << "[DEBUG_PHM], path: " << path;
}

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
