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
  phm_component_ = std::make_unique<PhmComponent>();
  phm_component_->Init();

  HLOG_INFO << "AlgInit successfully ";

  return 0;
}

void PhmComponentOnboard::AlgRelease() {}
}  //  namespace common_onboard
}  //  namespace perception
}  //  namespace hozon
