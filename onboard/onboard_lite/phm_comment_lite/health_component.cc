/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-11-29
 *****************************************************************************/

#include "lib/health_manager/health_manager.h"
#include "lib/io/file_util.h"
#include "lib/io/protobuf_util.h"
#include "lib/proto/health_config.pb.h"
#include "onboard/onboard_lite/phm_comment_lite/phm_commenet.h"
#include "proto/perception/perception_state_machine.pb.h"

namespace hozon {
namespace perception {
namespace common_onboard {
using hozon::perception::base::RunningMode;
using hozon::perception::lib::HealthConfig;
using hozon::perception::lib::HealthManager;
using hozon::perception::parking::PSMState;

bool PhmComponent::InitHealth() {
  std::string hm_config_file = "hm_config_file";
  GetFilePath(hm_config_file);
  // 初始化健康管理
  HLOG_INFO << "[DEBUG_PHM] -------------hm_config_file-----: "
            << hm_config_file;
  HealthManager::Instance()->Init(hm_config_file);

  HealthConfig health_config;
  if (!hozon::perception::lib::GetProtoFromFile(hm_config_file,
                                                &health_config)) {
    HLOG_ERROR << "[DEBUG_PHM] Parse failed! path: " << hm_config_file;
    return false;
  }

  for (const auto& iter : health_config.topic_name()) {
    trigger_.emplace_back(iter);
  }

  // 注册健康管理上报接口
  HealthManager::Instance()->SetCallBackHealthReport(
      [this](const int& checkpointid) -> bool {
        return ReportCheckPointId(checkpointid);
      });

  HealthManager::Instance()->SetCallBackNotifySmInfo(
      [this](const RunningMode& state) { NotifySmInfo(state); });
  return true;
}

void PhmComponent::NotifySmInfo(const RunningMode& state) {
  HLOG_DEBUG << "[DEBUG_HM] state " << static_cast<int32_t>(state);
  if (state == base::RunningMode::DRIVING) {
    ResumeTrigger();
  } else if (state == base::RunningMode::PARKING) {
    PauseTrigger();
  }
}

void PhmComponent::BindResumeTrigger(
    const std::function<int32_t(const std::string& trigger)>& resumeTrigger) {
  ResumeTrigger_ = resumeTrigger;
}

void PhmComponent::BindPauseTrigger(
    const std::function<int32_t(const std::string& trigger)>& pauseTrigger) {
  PauseTrigger_ = pauseTrigger;
}

void PhmComponent::PauseTrigger() {
  HLOG_INFO << "[DEBUG_HM] PauseTrigger ";
  phm_client_->Stop();
  for (const auto& iter : trigger_) {
    PauseTrigger_(iter);
    HLOG_INFO << "[DEBUG_HM] PauseTrigger iter" << iter;
  }
}

void PhmComponent::ResumeTrigger() {
  for (const auto& iter : trigger_) {
    ResumeTrigger_(iter);
    HLOG_INFO << "[DEBUG_HM] ResumeTrigger iter" << iter;
  }
  if (phm_client_) {
    phm_client_->Start();
  }
}

bool PhmComponent::ReportCheckPointId(const int& reportid) {
  HLOG_DEBUG << "[DEBUG_PHM] reportid " << reportid;
  if (phm_client_) {
    phm_client_->ReportCheckPoint(reportid);
  }
  return true;
}

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
