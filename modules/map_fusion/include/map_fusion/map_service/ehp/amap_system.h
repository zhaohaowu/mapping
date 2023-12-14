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
#include "proto/localization/localization.pb.h"
#include "util/mapping_log.h"

namespace hozon {
namespace mp {
namespace mf {
// 系统接口 demo
class SystemDeviceImp : public ::hdmap::service::ISystemDevice {
 public:
  SystemDeviceImp();
  SystemDeviceImp(const SystemDeviceImp& ada) = delete;
  SystemDeviceImp& operator=(const SystemDeviceImp& ada) = delete;
  SystemDeviceImp(SystemDeviceImp&& ada) = delete;
  SystemDeviceImp& operator=(const SystemDeviceImp&& ada) = delete;
  ~SystemDeviceImp() override;

 public:
  const char* getUid() override;
  ::hdmap::service::SystemInfo getSystemInfo() override;
  ::hdmap::service::NetworkState getNetworkState() override;
  int64_t getDiskFreeSize() override;
  void appearUnrecoverableError() override;
  void setSystemDeviceListener(
      ::hdmap::service::ISystemDeviceListener* systemDeviceListener) override;
  /**
   * @brief Set UUID to amap sdk
   *
   * @param uuid
   */
  void SetUUID(const std::string& uuid);

 private:
  ::hdmap::service::ISystemDeviceListener* listener_;
  std::string uuid_;
  std::mutex mutex_;
};

}  // namespace mf

}  // namespace mp
}  // namespace hozon
