/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <list>
#include <utility>

#include "ins/ins_locator.hpp"
#include "localization/data_type/base.hpp"
#include "localization/data_type/imu.hpp"
#include "localization/data_type/ins.hpp"
#include "localization/data_type/smart_ptr.hpp"
#include "localization/localization_param.hpp"
#include "system/localization_frontend.hpp"

namespace senseAD {
namespace localization {

class LocalizationFrontendINS : public LocalizationFrontend,
                                public FrontendAdapter<INSLocator, Ins> {
 public:
  DEFINE_PTR(LocalizationFrontendINS)

  LocalizationFrontendINS() = default;
  LocalizationFrontendINS(const LocalizationFrontendINS&) = delete;
  LocalizationFrontendINS& operator=(const LocalizationFrontendINS&) = delete;
  virtual ~LocalizationFrontendINS() = default;

  void SetImuData(uint64_t timestamp, const Imu& imu);

 protected:
  // @brief: check whether need system restart
  bool CheckFrontendNeedRestart() { return false; }

  adLocStatus_t InitLocator() final;

  adLocStatus_t InitInitializer() final;

  adLocStatus_t InitDataBuffer() final;

  void SetThreadName() final;

  adLocStatus_t ResetSubData() final;

  adLocStatus_t SwitchOriginSubProc() final;

  InitStage Initialize() final;

  NavStatus GetCurrentNavStatus(const NavState& nav_state) final;

  adLocStatus_t LocatorProcess(NavState* nav_state,
                               bool* continue_to_process) final;

 private:
  // buffer imu datas, for locator process
  std::mutex imu_mutex_;
  boost::circular_buffer<std::pair<uint64_t, Imu>> imu_data_list_;
};

}  // namespace localization
}  // namespace senseAD
