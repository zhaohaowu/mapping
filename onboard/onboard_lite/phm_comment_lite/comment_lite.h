/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-11-29
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>

#include "adf-lite/include/adf_lite_logger.h"
#include "adf-lite/include/base.h"
#include "adf-lite/include/bundle.h"
#include "adf-lite/include/executor.h"
#include "base/state_machine/state_machine_info.h"
#include "base/utils/log.h"
#include "lib/config_manager/config_manager.h"
#include "onboard/onboard_lite/phm_comment_lite/phm_commenet.h"
#include "phm/include/phm_client.h"

namespace hozon {
namespace perception {
namespace common_onboard {

using hozon::netaos::phm::PHMClient;
using hozon::netaos::phm::ReceiveFault_t;
using hozon::netaos::phm::SendFault_t;

using hozon::netaos::adf_lite::Bundle;
class PhmComponentOnboard : public hozon::netaos::adf_lite::Executor {
 public:
  PhmComponentOnboard() = default;
  ~PhmComponentOnboard() = default;
  int32_t AlgInit() override;
  void AlgRelease() override;

 private:
  int32_t send_running_mode(Bundle* input);

 private:
  std::unique_ptr<PhmComponent> phm_component_;
  uint8_t runmode_ = 0;
};

REGISTER_ADF_CLASS(PhmComponentOnboard, PhmComponentOnboard);
}  //  namespace common_onboard
}  //  namespace perception
}  //  namespace hozon
