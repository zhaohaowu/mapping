/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: shenlianchen
 *Date: 2023-10-08
 *****************************************************************************/
#pragma once

#include <memory>

#include "depend/ap-release/include/adsfi/include/data_types/perception/lanes.h"
#include "depend/ap-release/include/adsfi/include/data_types/sensors/sensors_imu.h"
#include "depend/ap-release/include/adsfi/include/data_types/vehicle/chassis_info.h"
#include "depend/proto/perception/transport_element.pb.h"
#include "proto/soc/chassis.pb.h"
#include "proto/dead_reckoning/dr.pb.h"
#include "proto/soc/sensor_imu_ins.pb.h"

namespace hozon {
namespace mp {
struct DataBoard {
  std::shared_ptr<hozon::perception::TransportElement> adsfi_lane_proto =
      nullptr;
  std::shared_ptr<hozon::soc::ImuIns> imu_proto = nullptr;
  std::shared_ptr<hozon::soc::Chassis> chassis_proto = nullptr;

  DataBoard();

  static void Adsfi2Proto(const hz_Adsfi::AlgLaneDetectionOutArray &stu,
                          hozon::perception::TransportElement *proto);

  void Adsfi2Proto(
      const std::shared_ptr<hz_Adsfi::AlgImuIns> &imuinsDataPtr_,
      std::shared_ptr<hozon::soc::ImuIns> &imu_proto);  // NOLINT

  void Adsfi2Proto(
      const std::shared_ptr<hz_Adsfi::AlgChassisInfo> &chassisDataPtr_,
      std::shared_ptr<hozon::soc::Chassis> &chassis_proto);  // NOLINT
};

}  // namespace mp
}  // namespace hozon
