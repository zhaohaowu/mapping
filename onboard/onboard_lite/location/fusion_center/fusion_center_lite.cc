/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： fusion_center_lite.cc
 *   author     ： lilanxing
 *   date       ： 2023.09
 ******************************************************************************/

#include "onboard/onboard_lite/location/fusion_center/fusion_center_lite.h"
#include <gflags/gflags.h>
#include <base/utils/log.h>

DEFINE_string(fc_config,
              "/data/code/mapping/conf/mapping/location/fusion_center/"
              "fc_config.yaml",
              "fusion center core config file");

namespace hozon {
namespace perception {
namespace common_onboard {

const char* const kImuTopic = "imu_ins";
const char* const kInsFusionTopic = "/location/ins_fusion";
const char* const kFcTopic = "localization";

int32_t FusionCenterLite::AlgInit() {
  fusion_center_ = std::make_unique<FusionCenter>();
  if (!fusion_center_->Init(FLAGS_fc_config)) {
    return -1;
  }

  RegistLog();
  RegistMessageType();
  RegistProcessFunc();

  return 0;
}

void FusionCenterLite::AlgRelease() {}

void FusionCenterLite::RegistLog() const {
  hozon::netaos::log::InitLogging("loc_fc", "fusion_center",
                                  hozon::netaos::log::LogLevel::kInfo,
                                  HZ_LOG2CONSOLE, "./", 10, (20));
  hozon::netaos::adf::NodeLogger::GetInstance().CreateLogger(
      "loc_fc", "fusion_center", hozon::netaos::log::LogLevel::kInfo);
}

void FusionCenterLite::RegistMessageType() const {
  REGISTER_MESSAGE_TYPE(kImuTopic, hozon::soc::ImuIns);
  REGISTER_MESSAGE_TYPE(kInsFusionTopic, hozon::localization::HafNodeInfo);
  REGISTER_MESSAGE_TYPE(kFcTopic, hozon::localization::Localization);
}

void FusionCenterLite::RegistProcessFunc() {
  RegistAlgProcessFunc(
      "recv_ins_fusion",
      std::bind(&FusionCenterLite::OnInsFusion, this, std::placeholders::_1));
}

int32_t FusionCenterLite::OnInsFusion(Bundle* input) {
  if (!input) {
    return -1;
  }

  BaseDataTypePtr p_ins_fusion = input->GetOne(kInsFusionTopic);
  if (!p_ins_fusion) {
    return -1;
  }

  const auto ins_fusion =
      std::static_pointer_cast<hozon::localization::HafNodeInfo>(
          p_ins_fusion->proto_msg);
  if (!ins_fusion) {
    return -1;
  }

  fusion_center_->OnIns(*ins_fusion);

  // send output
  auto localization = std::make_shared<hozon::localization::Localization>();
  if (!fusion_center_->GetCurrentOutput(localization.get())) {
    HLOG_ERROR << "onboard get localization result error";
    return -1;
  }

  BaseDataTypePtr localization_pack =
      std::make_shared<hozon::netaos::adf_lite::BaseData>();
  localization_pack->proto_msg = localization;
  SendOutput(kFcTopic, localization_pack);

  return 0;
}

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
