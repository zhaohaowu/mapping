/*
 * Copyright (c) Hozon Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  ehp
 */
#include "map_fusion/map_service/ehp/amap_system.h"
#include <memory.h>
#include <sys/statfs.h>

#include <cmath>
#include <cstring>
#include <ctime>
#include <iostream>

#include "glog/logging.h"
#include "util/temp_log.h"

namespace hozon {
namespace mp {
namespace mf {
// 系统接口 demo
SystemDeviceImp::SystemDeviceImp() : listener_(nullptr) {}

SystemDeviceImp::~SystemDeviceImp() { listener_ = nullptr; }

const char* SystemDeviceImp::getUid() {
  std::lock_guard<std::mutex> lock(mutex_);
  HLOG_INFO << "uuid: " << uuid_;
  return uuid_.c_str();
}

::hdmap::service::NetworkState SystemDeviceImp::getNetworkState() {
  HLOG_INFO << "NETWORK_STATE_4G";
  return ::hdmap::service::NETWORK_STATE_4G;
}

::hdmap::service::SystemInfo SystemDeviceImp::getSystemInfo() {
  static ::hdmap::service::SystemInfo info;
  info.systemName = "linux";
  info.systemVersion = "2.0.0";
  return info;
}

int64_t SystemDeviceImp::getDiskFreeSize() {
#ifdef ISMDC
  struct statfs diskInfo {};
  statfs("/opt/usr/hd_map", &diskInfo);
  uint64_t blocksize = diskInfo.f_bsize;
  // uint64_t totalsize = blocksize * diskInfo.f_blocks;
  auto availableDisk = static_cast<int64_t>(diskInfo.f_bavail * blocksize);
  return availableDisk;
#endif
  return -1;
}

void SystemDeviceImp::SetUUID(const std::string& uuid) {
  std::lock_guard<std::mutex> lock(mutex_);
  uuid_ = uuid;
}

void SystemDeviceImp::appearUnrecoverableError() {}

void SystemDeviceImp::setSystemDeviceListener(
    ::hdmap::service::ISystemDeviceListener* systemDeviceListener) {
  listener_ = systemDeviceListener;
  // 测试代码,系统方主动调用一次网络状态更新
  // if (systemDeviceListener) {
  //   systemDeviceListener->onNetworkStateChanged(NETWORK_STATE_3G);
  // }
}
}  // namespace mf

}  // namespace mp
}  // namespace hozon
