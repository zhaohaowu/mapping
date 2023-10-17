/*
 * Copyright (c) Hozon Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  ehp
 */
#include "map_fusion/map_service/ehp/amap_ota.h"

#include <memory.h>

#include <cmath>
#include <ctime>
#include <iostream>

#include "common/configs/config_gflags.h"

namespace hozon {
namespace mp {
namespace mf {
// 系统接口 demo
OtaDeviceImp::OtaDeviceImp()
    : listener_(nullptr),
      ota_status_(0U),
      update_map_(true),
      hot_swap_(false) {}

OtaDeviceImp::~OtaDeviceImp() { listener_ = nullptr; }

bool OtaDeviceImp::isHotSwapEnabled() { return false; }

void OtaDeviceImp::receiveDOTAStatus(uint32_t status) { ota_status_ = status; }

void OtaDeviceImp::confirmDOTAUpdateAuthorization() {
  update_map_ = true;
  if (listener_ != nullptr) {
    listener_->setDOTAUpdateEnabled(true);
  }
}

void OtaDeviceImp::UpdateMapData() {
  if (update_map_ && (listener_ != nullptr)) {
    listener_->setDOTAUpdateEnabled(true);
  }
}

void OtaDeviceImp::HotSwap() {
  if (hot_swap_) {
    listener_->setHotSwapEnabled(false);
  }
}

void OtaDeviceImp::setDOTAListener(::hdmap::service::IDOTAListener* listener) {
  listener_ = listener;
}
}  // namespace mf

}  // namespace mp
}  // namespace hozon
