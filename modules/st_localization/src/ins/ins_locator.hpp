/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Chen Shengjie <chenshengjie@sensetime.com>
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 */
#pragma once

#include <Eigen/Core>
#include <list>
#include <memory>
#include <string>

#include <Sophus/se3.hpp>

#include "base_locator/base_locator.hpp"
#include "localization/data_type/base.hpp"
#include "localization/data_type/gnss.hpp"
#include "localization/data_type/imu.hpp"
#include "localization/data_type/ins.hpp"
#include "localization/localization_param.hpp"

namespace senseAD {
namespace localization {

class INSLocator : public BaseLocator {
 public:
  INSLocator() = default;

  virtual ~INSLocator() = default;

  adLocStatus_t Init(const LocalizationParam& param) final;

  adLocStatus_t SetState(const NavState& nav_state) final {
    return LOC_SUCCESS;
  }

  adLocStatus_t GetState(NavState* nav_state,
                         double* confidence = nullptr) final;

  adLocStatus_t Restart() final { return LOC_SUCCESS; }

  adLocStatus_t Process(const uint64_t timestamp,
                        std::shared_ptr<Ins> fused_ins,
                        std::shared_ptr<Imu> raw_imu = nullptr);

  static SE3d ParsePose(const Eigen::Vector3d& ypr, const Eigen::Vector3d& t,
                        const std::string& ins_device, bool init = false);

 private:
  std::string ins_device_;
  Eigen::Vector3d gravity_;
  Eigen::Matrix<double, 6, 6> ins_measure_cov_;

  NavState pose_state_;  // refined pose state
};

}  // namespace localization
}  // namespace senseAD
