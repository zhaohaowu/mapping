/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: zhuxiaolin
 *Date: 2023-08-31
 *****************************************************************************/
#pragma once

#include <memory>

// #include "interface/adsfi_proto/sensors/sensors_imu.pb.h"
// #include "interface/adsfi_proto/sensors/sensors_ins.pb.h"
// #include "interface/adsfi_proto/vehicle/chassis_info.pb.h"
#include "proto/soc/chassis.pb.h"
#include "proto/soc/sensor_imu_ins.pb.h"
#include "depend/proto/dead_reckoning/dr.pb.h"
#include "cyber/component/component.h"
#include "modules/dr/include/dr.h"

namespace hozon {
namespace perception {
namespace common_onboard {

class DeadReckoningComponent : public apollo::cyber::Component<> {
 public:
  DeadReckoningComponent() = default;

  /**
   * @brief cyber init
   *
   * @return `true` for success, `false` for failed
   */
  bool Init() override;

  /**
   * @brief receive imuins message
   *
   * @param msg : imuins message
   * @return `true` for receiveing and processing success, `false` for failed
   */
  bool OnImuIns(const std::shared_ptr<const hozon::soc::ImuIns>& msg);

  /**
   * @brief receive chassis message
   *
   * @param msg : chassis message
   * @return `true` for receiveing and processing success, `false` for failed
   */
  bool OnChassis(
      const std::shared_ptr<const hozon::soc::Chassis>& msg);

  /**
   * @brief local map publish
   *
   * @return
   */
  void DeadReckoningPublish();

 private:
  std::thread dr_publish_thread_;
  std::shared_ptr<hozon::mp::dr::DRInterface> dr_;
  std::shared_ptr<apollo::cyber::Reader<hozon::soc::ImuIns>>
      imuins_listener_;
  std::shared_ptr<apollo::cyber::Reader<hozon::soc::Chassis>>
      chassis_listener_;
  std::shared_ptr<apollo::cyber::Writer<hozon::dead_reckoning::DeadReckoning>> result_talker_;
};

CYBER_REGISTER_COMPONENT(DeadReckoningComponent);

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
