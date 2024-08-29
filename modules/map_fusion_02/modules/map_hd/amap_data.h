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
#include <utility>
#include <vector>

#include "amap/GHDMapDefines.h"
#include "amap/GHDMapService.h"
#include "common/time/clock.h"

namespace hozon {
namespace mp {
namespace mf {
using EhpData = std::pair<int32_t, std::vector<std::string>>;

// adasis V3 协议透出 demo
class AdasisV3DataListenerImp
    : public ::hdmap::service::IAdasisV3MessageReceiver {
 public:
  AdasisV3DataListenerImp();
  AdasisV3DataListenerImp(const AdasisV3DataListenerImp& ada) = delete;
  AdasisV3DataListenerImp& operator=(const AdasisV3DataListenerImp& ada) =
      delete;
  AdasisV3DataListenerImp(AdasisV3DataListenerImp&& ada) = delete;
  AdasisV3DataListenerImp& operator=(const AdasisV3DataListenerImp&& ada) =
      delete;
  ~AdasisV3DataListenerImp() override;

 public:
  bool shouldReceiveMessage() override;
  bool receiveGlobalData(
      const ::hdmap::service::GlobalData& globalData) override;
  bool receivePosition(
      const ::hdmap::service::PositionData& positionData) override;
  bool receivePathControl(
      const ::hdmap::service::PathControlData& pathControlData) override;
  bool receiveProfileControl(
      const ::hdmap::service::ProfileControlData& profileControlData) override;
  bool receiveProfile(
      const ::hdmap::service::ProfileData& profileData) override;
  bool receiveReason(const ::hdmap::service::ReasonData& reasonData) override;
  void setAdasisV3Listener(
      ::hdmap::service::IAdasisV3MessageListener* adasisV3Listener) override;
  bool GetEhpData(std::vector<EhpData>* ehp_data);
  void DeleteDataByCounter(int min_counter);
  ::hdmap::service::IAdasisV3MessageListener* GetListener();

  bool ReInit();

 private:
  ::hdmap::service::IAdasisV3MessageListener* listener_;
  int counter_;
  std::mutex mutex_;
  std::vector<std::string> ehp_data_;
  std::vector<EhpData> ehp_data_list_;
};
}  // namespace mf

}  // namespace mp
}  // namespace hozon
