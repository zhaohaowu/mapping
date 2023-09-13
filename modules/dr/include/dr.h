/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-08-28
 *****************************************************************************/
#pragma once

#include <memory>
#include <string>

#include "base/utils/log.h"
#include "modules/dr/include/odometry2D.h"
#include "modules/dr/include/odometry_base.h"
#include "modules/dr/include/slam_data_types.h"
#include "modules/dr/include/wheel_odom.h"
#include "proto/canbus/chassis.pb.h"
#include "proto/drivers/location.pb.h"
#include "proto/drivers/sensor_imu_ins.pb.h"

namespace hozon {
namespace mp {
namespace dr {

class DRInterface {
 public:
  DRInterface();
  ~DRInterface() {}

  void AddImuData(std::shared_ptr<hozon::drivers::imuIns::ImuIns> imu_proto);

  void AddChassisData(std::shared_ptr<hozon::canbus::Chassis> chassis_proto);

  void SetLocation(std::shared_ptr<hozon::perception::datacollection::Location>
                       locationDataPtr);

 private:
  double GetCurrentNsecTime() {
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
        tp = std::chrono::time_point_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now());

    time_t time = tp.time_since_epoch().count();
    return time;
  }

  void SetInsData2Location(
      std::shared_ptr<hozon::perception::datacollection::Location>
          locationDataPtr,
      const OdometryData &odom_data);

  void SetLocationData(
      std::shared_ptr<hozon::perception::datacollection::Location>
          locationDataPtr,
      OdometryData &latest_odom, Eigen::Vector3d &eulerAngle);  // NOLINT

  void ConvertImuData(std::shared_ptr<hozon::drivers::imuIns::ImuIns> imu_proto,
                      ImuDataHozon &imu_data);  // NOLINT

  void ConvertChassisData(std::shared_ptr<hozon::canbus::Chassis> chassis_proto,
                          WheelDataHozon &wheel_data);  // NOLINT

  Eigen::Vector3d Qat2EulerAngle(const Eigen::Quaterniond &q);

 private:
  std::shared_ptr<OdometryBase> dr_estimator_;
};

}  // namespace dr
}  // namespace mp
}  // namespace hozon
