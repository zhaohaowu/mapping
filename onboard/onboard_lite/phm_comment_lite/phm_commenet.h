/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-11-29
 *****************************************************************************/
#pragma once
#include <functional>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "phm/include/phm_client.h"
#include "base/state_machine/state_machine_info.h"
#include "lib/config_manager/config_manager.h"
#include "onboard/onboard_lite/flags/mapping_flags.h"

namespace hozon {
namespace perception {
namespace common_onboard {

using hozon::netaos::phm::PHMClient;
using hozon::netaos::phm::ReceiveFault_t;
using hozon::netaos::phm::SendFault_t;

class PhmComponent {
 public:
  PhmComponent();
  ~PhmComponent();
  bool Init();
  bool FaultReport(const int32_t& faultid, const int32_t& objid,
                   const int32_t& status, const int32_t& debounceCount = 0,
                   const int32_t& debounceTime = 0);
  bool ReportCheckPointId(const int& reportid);
  void BindResumeTrigger(
      const std::function<int32_t(const std::string& trigger)>& resumeTrigger);
  void BindPauseTrigger(
      const std::function<int32_t(const std::string& trigger)>& pauseTrigger);

 private:
  bool InitHealth();
  bool InitFault();
  void ResetFault();
  void ServiceAvailableCallback(const bool bResult);
  void FaultReceiveCallback(const ReceiveFault_t& fault);
  void GetFilePath(std::string& path);  // NOLINT
  void NotifySmInfo(const hozon::perception::base::RunningMode& state);
  void PauseTrigger();
  void ResumeTrigger();
  std::string Name() const {
    return "MappingComponent";
  }

 private:
  std::shared_ptr<PHMClient> phm_client_;
  std::unordered_map<std::string, int16_t> faultmap_;
  lib::ConfigManager* config_manager_;
  const lib::ModelConfig* model_config_;
  std::string work_root_;
  bool suspend_ = false;
  bool cur_suspend_ = false;
  std::vector<std::string> trigger_;
  std::function<int32_t(const std::string& trigger)> PauseTrigger_;
  std::function<int32_t(const std::string& trigger)> ResumeTrigger_;
};
}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
