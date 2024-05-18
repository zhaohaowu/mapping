/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-08-28
 *****************************************************************************/
#pragma once

#include <iomanip>
#include <memory>
#include <string>

#include "base/utils/log.h"
#include "modules/dr/include/odometry2D.h"
#include "modules/dr/include/odometry_base.h"
#include "modules/dr/include/slam_data_types.h"
#include "modules/dr/include/wheel_odom.h"
#include "modules/util/include/util/mapping_log.h"
#include "proto/common/header.pb.h"
#include "proto/dead_reckoning/dr.pb.h"
#include "proto/soc/chassis.pb.h"
#include "proto/soc/sensor_imu_ins.pb.h"

namespace hozon {
namespace mp {
namespace dr {

class DRInterface {
 public:
  explicit DRInterface(const std::string& conf_path);
  ~DRInterface() {}

  void AddImuData(const std::shared_ptr<const hozon::soc::ImuIns>& imu_proto);

  void AddChassisData(
      const std::shared_ptr<const hozon::soc::Chassis>& chassis_proto);

  bool Process();
  bool GetLatestPose(
      double timestamp,
      std::shared_ptr<hozon::dead_reckoning::DeadReckoning> locationDataPtr,
      double sync_time);

 private:
  double GetCurrentNsecTime() {
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
        tp = std::chrono::time_point_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now());

    time_t time = tp.time_since_epoch().count();
    double result = time * 1e-9;
    return result;
  }

  void SetLocationData(
      std::shared_ptr<hozon::dead_reckoning::DeadReckoning> locationDataPtr,
      const OdometryData& latest_odom);

  static void ConvertImuData(
      const std::shared_ptr<const hozon::soc::ImuIns>& imu_proto,
      ImuDataHozon& imu_data);  // NOLINT

  static void ConvertChassisData(
      const std::shared_ptr<const hozon::soc::Chassis>& chassis_proto,
      WheelDataHozon& wheel_data);  // NOLINT

  static Eigen::Vector3d Qat2EulerAngle(const Eigen::Quaterniond& q);

 private:
  std::shared_ptr<OdometryBase> dr_estimator_ = nullptr;
};

}  // namespace dr
}  // namespace mp
}  // namespace hozon
