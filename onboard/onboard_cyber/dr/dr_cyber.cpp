/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: zhuxiaolin
 *Date: 2023-08-31
 *****************************************************************************/

#include "onboard/onboard_cyber/dr/dr_cyber.h"

DEFINE_string(imuins_topic, "/ImuIns", "ImuInfo topic");
DEFINE_string(chassis_topic, "/Chassis", "Chassis topic");
DEFINE_string(output_topic, "/mapping/dr", "dr topic");
DEFINE_string(config_yaml, "conf/mapping/dr/dr_conf.yaml",
              "path to dr conf yaml");

namespace hozon {
namespace perception {
namespace common_onboard {

bool DeadReckoningComponent::Init() {
  dr_ = std::make_shared<hozon::mp::dr::DRInterface>();  // FLAGS_config_yaml

  dr_publish_thread_ =
      std::thread(&DeadReckoningComponent::DeadReckoningPublish, this);

  imuins_listener_ = node_->CreateReader<hozon::soc::ImuIns>(
      FLAGS_imuins_topic,
      [this](const std::shared_ptr<const hozon::soc::ImuIns>& msg) {
        OnImuIns(msg);
      });

  chassis_listener_ = node_->CreateReader<hozon::soc::Chassis>(
      FLAGS_chassis_topic,
      [this](const std::shared_ptr<const hozon::soc::Chassis>& msg) {
        OnChassis(msg);
      });

  result_talker_ = node_->CreateWriter<hozon::dead_reckoning::DeadReckoning>(
      FLAGS_output_topic);

  return true;
}

bool DeadReckoningComponent::OnImuIns(
    const std::shared_ptr<const hozon::soc::ImuIns>& msg) {
  if (!dr_) {
    return false;
  }
  dr_->AddImuData(msg);
  return true;
}

bool DeadReckoningComponent::OnChassis(
    const std::shared_ptr<const hozon::soc::Chassis>& msg) {
  if (!dr_) {
    return false;
  }
  dr_->AddChassisData(msg);
  return true;
}

void DeadReckoningComponent::DeadReckoningPublish() {
  while (apollo::cyber::OK()) {
    std::shared_ptr<hozon::dead_reckoning::DeadReckoning> result =
        std::make_shared<hozon::dead_reckoning::DeadReckoning>();
    if (dr_->SetLocation(result) && result_talker_ != nullptr) {
      result_talker_->Write(result);
    }
    usleep(9 * 1e3);
  }
}

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
