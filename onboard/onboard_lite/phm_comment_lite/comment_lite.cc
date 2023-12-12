/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-11-29
 *****************************************************************************/
#include "onboard/onboard_lite/phm_comment_lite/comment_lite.h"
#include <gflags/gflags.h>

#include "base/utils/log.h"
#include "yaml-cpp/yaml.h"

namespace hozon {
namespace perception {
namespace common_onboard {

int32_t PhmComponentOnboard::AlgInit() {
  // phm
  HLOG_INFO << "AlgInit ===================================== ";
  phm_component_ = std::make_unique<PhmComponent>();
  phm_component_->Init();
  phm_component_->BindPauseTrigger(
      std::bind((int32_t(PhmComponentOnboard::*)(const std::string& trigger)) &
                    PhmComponentOnboard::PauseTrigger,
                this, std::placeholders::_1));
  phm_component_->BindResumeTrigger(
      std::bind((int32_t(PhmComponentOnboard::*)(const std::string& trigger)) &
                    PhmComponentOnboard::ResumeTrigger,
                this, std::placeholders::_1));

  HLOG_INFO << "AlgInit successfully ";

  return 0;
}

void PhmComponentOnboard::AlgRelease() {}
}  //  namespace common_onboard
}  //  namespace perception
}  //  namespace hozon
