/*
 * Copyright (c) Hozon Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  ehp
 */
#pragma once
#include <depend/proto/localization/node_info.pb.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "amap/GHDAv3Service.h"
#include "amap/GHDMapDefines.h"
#include "amap/GHDMapService.h"
#include "common/time/clock.h"
#include "lib/environment/environment.h"
#include "modules/map_fusion_02/base/interface_option.h"
#include "modules/map_fusion_02/modules/map_hd/include/ehp/amap_data.h"
#include "modules/map_fusion_02/modules/map_hd/include/ehp/amap_diagnose.h"
#include "modules/map_fusion_02/modules/map_hd/include/ehp/amap_ota.h"
#include "modules/map_fusion_02/modules/map_hd/include/ehp/amap_system.h"
#include "proto/localization/localization.pb.h"
#include "proto/planning/planning.pb.h"
namespace hozon {
namespace mp {
namespace mf {
class AmapAdapter {
 public:
  AmapAdapter();
  AmapAdapter(const AmapAdapter& amap_adapter) = default;
  AmapAdapter& operator=(const AmapAdapter& amap_adapter) = default;
  AmapAdapter(AmapAdapter&& amap_daapter) = default;
  AmapAdapter& operator=(AmapAdapter&& amap_adapter) = default;
  ~AmapAdapter();
  /**
   * @brief inital amap sdk
   *when init succ, is_init_ is true; if not,  is_init_ is false
   * @return 0
   */
  bool Init();

  // v3data process
  bool Process(const hozon::localization::HafNodeInfo& localization_input,
               std::vector<hozon::mp::mf::EhpData>* ehp_data);
  bool GetInitState() const;

  bool IsOverTime(double new_time) const;
  bool MessageControl();

  void MessageClose() const;
  void MessageOpen() const;

  // diagnos process
  bool GetDiagnose(::hdmap::service::DiagnosisState* state,
                   ::hdmap::service::DiagnosisType* type,
                   std::string* detailMessage);
  /**
   * @brief Set UUID
   *
   * @param uuid
   */
  void SetUUID(const std::string& uuid);

 private:
  bool is_init_;
  std::shared_ptr<hozon::mp::mf::AdasisV3DataListenerImp> ehp_data_listener_;
  std::shared_ptr<hozon::mp::mf::DiagnosisImp> ehp_diagnose_;
  std::shared_ptr<hozon::mp::mf::SystemDeviceImp> ehp_system_;
  std::shared_ptr<hozon::mp::mf::OtaDeviceImp> ehp_ota_;
  double start_timestamp_;
  bool message_state_;
  double current_time_;
  bool message_open_;
  std::string uid_;
};
}  // namespace mf

}  // namespace mp
}  // namespace hozon
