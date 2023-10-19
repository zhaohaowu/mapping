/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-09-09
 *****************************************************************************/

#pragma once

#include <memory>
#include "modules/local_mapping/lib/local_mapping.h"
#include "common_onboard/adapter/onboard_lite/onboard_lite.h"

namespace hozon {
namespace perception {
namespace common_onboard {

using hozon::mp::lm::LMapApp;

class LocalMappingOnboard : public OnboardLite {
 public:
  LocalMappingOnboard() = default;
  ~LocalMappingOnboard() = default;

  int32_t AlgInit() override;
  void AlgRelease() override;

  int32_t OnLaneLine(Bundle* input);

  int32_t OnDr(Bundle* input);

  // int32_t OnLocation(Bundle* input);

  int32_t OnIns(Bundle* input);

  // int32_t OnRoadEdge(Bundle* input);

  int32_t LocalMapPublish(Bundle* output);

  int32_t LocalMapLocationPublish(Bundle* output);

 private:
  std::shared_ptr<LMapApp> lmap_;
};

REGISTER_EXECUTOR_CLASS("LocalMappingOnboard", LocalMappingOnboard);

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
