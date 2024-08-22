/*
 * Copyright (c) Hozon Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  ehp
 */
#pragma once
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "amap/GHDMapDefines.h"
#include "amap/GHDMapService.h"
#include "common/time/clock.h"
#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace mf {
// 系统接口 demo
class OtaDeviceImp : public ::hdmap::service::IDOTAReceiver {
 public:
  OtaDeviceImp();
  OtaDeviceImp(const OtaDeviceImp& ada) = delete;
  OtaDeviceImp& operator=(const OtaDeviceImp& ada) = delete;
  OtaDeviceImp(OtaDeviceImp&& ada) = delete;
  OtaDeviceImp& operator=(const OtaDeviceImp&& ada) = delete;
  ~OtaDeviceImp() override;

  bool isHotSwapEnabled() override;
  void receiveDOTAStatus(uint32_t status) override;
  void setDOTAListener(::hdmap::service::IDOTAListener* listener) override;
  void confirmDOTAUpdateAuthorization() override;

  void UpdateMapData();

  void HotSwap();

 private:
  ::hdmap::service::IDOTAListener* listener_;
  uint32_t ota_status_;
  bool update_map_;
  bool hot_swap_;
};
}  // namespace mf

}  // namespace mp
}  // namespace hozon
