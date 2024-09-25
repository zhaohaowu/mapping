/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： fusion_center_lite.cc
 *   author     ： lilanxing
 *   date       ： 2023.09
 ******************************************************************************/
#include <base/utils/log.h>
#include <gflags/gflags.h>
#include <perception-lib/lib/environment/environment.h>

#include <string>
#include <utility>

#include "depend/nos/x86_2004/include/adf/include/node_proto_register.h"
#include "depend/perception-lib/lib/fault_manager/fault_manager.h"
#include "depend/proto/soc/chassis.pb.h"
#include "onboard/onboard_lite/location/fusion_center/fusion_center_lite.h"
#include "onboard/onboard_lite/phm_comment_lite/proto/running_mode.pb.h"
#include "perception-base/base/state_machine/state_machine_info.h"
#include "proto/soc/sensor_imu_ins.pb.h"

namespace hozon {
namespace mp {
namespace loc {

const char* const kImuTopic = "imu_ins";
const char* const kChassisTopic = "chassis";
const char* const kInsFusionTopic = "/location/ins_fusion";
const char* const kDrTopic = "dr";
const char* const kMmTopic = "/location/pose_estimation";
const char* const kFcTopic = "localization";
const char* const kRunningModeTopic = "running_mode";
const char* const kFcConfSuffix =
    "runtime_service/mapping/conf/mapping/"
    "location/fusion_center/fc_config.yaml";

int32_t FusionCenterLite::AlgInit() {
  const std::string adflite_root_path =
      hozon::perception::lib::GetEnv("ADFLITE_ROOT_PATH", ".");
  const std::string fc_config = adflite_root_path + "/" + kFcConfSuffix;

  fusion_center_ = std::make_unique<FusionCenter>();
  if (!fusion_center_->Init(fc_config)) {
    return -1;
  }
  RegistMessageType();
  RegistProcessFunc();

  return 0;
}

void FusionCenterLite::AlgRelease() {}

void FusionCenterLite::RegistMessageType() const {
  REGISTER_PROTO_MESSAGE_TYPE(kImuTopic, hozon::soc::ImuIns);
  REGISTER_PROTO_MESSAGE_TYPE(kChassisTopic, hozon::soc::Chassis);
  REGISTER_PROTO_MESSAGE_TYPE(kInsFusionTopic,
                              hozon::localization::HafNodeInfo);
  REGISTER_PROTO_MESSAGE_TYPE(kDrTopic, hozon::dead_reckoning::DeadReckoning);
  REGISTER_PROTO_MESSAGE_TYPE(kMmTopic, hozon::localization::HafNodeInfo);
  REGISTER_PROTO_MESSAGE_TYPE(kFcTopic, hozon::localization::Localization);
  REGISTER_PROTO_MESSAGE_TYPE(kRunningModeTopic,
                              hozon::perception::common_onboard::running_mode);
}

void FusionCenterLite::RegistProcessFunc() {
  RegistAlgProcessFunc("recv_ins_fusion", [this](auto&& PH1) {
    return OnInsFusion(std::forward<decltype(PH1)>(PH1));
  });
  RegistAlgProcessFunc("recv_dr", [this](auto&& PH1) {
    return OnDr(std::forward<decltype(PH1)>(PH1));
  });
  RegistAlgProcessFunc("recv_mm", [this](auto&& PH1) {
    return OnMm(std::forward<decltype(PH1)>(PH1));
  });
  RegistAlgProcessFunc("recv_imu", [this](auto&& PH1) {
    return OnImu(std::forward<decltype(PH1)>(PH1));
  });
  RegistAlgProcessFunc("recv_chassis", [this](auto&& PH1) {
    return OnChassis(std::forward<decltype(PH1)>(PH1));
  });
  RegistAlgProcessFunc("recv_running_mode", [this](auto&& PH1) {
    return OnRunningMode(std::forward<decltype(PH1)>(PH1));
  });
}

int32_t FusionCenterLite::OnInsFusion(Bundle* input) {
  auto* phm_fault = hozon::perception::lib::FaultManager::Instance();
  static bool input_data_init_error_flag = false;
  if (input == nullptr) {
    return -1;
  }
  auto p_ins_fusion = input->GetOne(kInsFusionTopic);
  if (!p_ins_fusion) {
    HLOG_WARN << "ins get_one failed";
    return -1;
  }

  const auto ins_fusion =
      std::static_pointer_cast<hozon::localization::HafNodeInfo>(
          p_ins_fusion->proto_msg);
  if (!ins_fusion) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::LOCALIZATION_CAN_NOT_INIT,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 1, 1000));
    input_data_init_error_flag = true;
    HLOG_ERROR << "Location init failed!!!";
    return -1;
  }
  if (input_data_init_error_flag) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::LOCALIZATION_CAN_NOT_INIT,
        hozon::perception::base::FaultStatus::RESET,
        hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
    input_data_init_error_flag = false;
  }
  fusion_center_->OnIns(*ins_fusion);

  static int ins_fusion_count = 0;
  ++ins_fusion_count;
  if (ins_fusion_count >= 100) {
    ins_fusion_count = 0;
    HLOG_INFO << "rev ins_fusion lite heartbeat";
  }
  return 0;
}

int32_t FusionCenterLite::OnDr(Bundle* input) {
  auto* phm_fault = hozon::perception::lib::FaultManager::Instance();
  if (input == nullptr) {
    return -1;
  }
  auto p_dr = input->GetOne(kDrTopic);
  if (!p_dr) {
    HLOG_WARN << "dr get_one failed";
    return -1;
  }
  const auto dr =
      std::static_pointer_cast<hozon::dead_reckoning::DeadReckoning>(
          p_dr->proto_msg);
  if (!dr) {
    return -1;
  }
  fusion_center_->OnDr(*dr);

  static int dr_count = 0;
  ++dr_count;
  if (dr_count >= 100) {
    dr_count = 0;
    HLOG_INFO << "rev dr lite begin heartbeat";
  }
  return 0;
}

int32_t FusionCenterLite::OnMm(Bundle* input) {
  if (input == nullptr) {
    return -1;
  }
  auto p_mm = input->GetOne(kMmTopic);
  if (!p_mm) {
    HLOG_WARN << "mm get_one failed";
    return -1;
  }
  const auto mm = std::static_pointer_cast<hozon::localization::HafNodeInfo>(
      p_mm->proto_msg);
  if (!mm) {
    return -1;
  }
  fusion_center_->OnMm(*mm);

  static int mm_count = 0;
  ++mm_count;
  if (mm_count >= 100) {
    mm_count = 0;
    HLOG_INFO << "rev mm lite heartbeat";
  }
  return 0;
}

int32_t FusionCenterLite::OnImu(Bundle* input) {
  static int imu_count = 0;
  if (input == nullptr) {
    return -1;
  }
  auto p_imu = input->GetOne(kImuTopic);
  if (!p_imu) {
    HLOG_WARN << "imu get_one failed";
    return -1;
  }
  const auto imu =
      std::static_pointer_cast<hozon::soc::ImuIns>(p_imu->proto_msg);
  if (!imu) {
    return -1;
  }
  fusion_center_->OnImu(*imu);
  auto localization = fusion_center_->GetFcOutput();
  if (!localization) {
    HLOG_ERROR << "pub fc failed";
    return -1;
  }
  auto localization_pack =
      std::make_shared<hozon::netaos::adf_lite::BaseData>();
  localization_pack->proto_msg = localization;
  SendOutput(kFcTopic, localization_pack);

  ++imu_count;
  if (imu_count >= 100) {
    imu_count = 0;
    HLOG_INFO << "rev imu lite heartbeat";
  }
  return 0;
}

int32_t FusionCenterLite::OnChassis(Bundle* input) {
  static int chassis_count = 0;
  if (input == nullptr) {
    return -1;
  }
  auto p_chassis = input->GetOne(kChassisTopic);
  if (!p_chassis) {
    HLOG_ERROR << "chassis get_one failed";
    return -1;
  }
  const auto chassis =
      std::static_pointer_cast<hozon::soc::Chassis>(p_chassis->proto_msg);
  if (!chassis) {
    return -1;
  }
  fusion_center_->OnChassis(*chassis);

  ++chassis_count;
  if (chassis_count >= 100) {
    chassis_count = 0;
    HLOG_INFO << "rev chassis lite heartbeat";
  }
  return 0;
}

int32_t FusionCenterLite::OnRunningMode(Bundle* input) {
  static int running_mode_count = 0;
  auto rm_msg = input->GetOne("running_mode");
  if (rm_msg == nullptr) {
    HLOG_ERROR << "nullptr rm_msg plugin";
    return -1;
  }
  auto msg =
      std::static_pointer_cast<hozon::perception::common_onboard::running_mode>(
          rm_msg->proto_msg);
  if (msg == nullptr) {
    HLOG_ERROR << "nullptr rm_msg->proto_msg";
    return -1;
  }
  int runmode = msg->mode();
  // HLOG_ERROR << "!!!!!!!!!!get run mode : " << runmode;
  static int last_runmode =
      static_cast<int>(hozon::perception::base::RunningMode::DRIVING);
  if (runmode ==
      static_cast<int>(hozon::perception::base::RunningMode::PARKING)) {
    if (last_runmode != runmode) {
      PauseTrigger("recv_ins_fusion");
      PauseTrigger("recv_pose_estimation");
      PauseTrigger("recv_dr");
      HLOG_INFO << "!!!!!!!!!!get run mode PARKING";
      last_runmode = runmode;
    }
    // HLOG_ERROR << "!!!!!!!!!!get run mode PARKING";
  } else if (runmode == static_cast<int>(
                            hozon::perception::base::RunningMode::DRIVING) ||
             runmode ==
                 static_cast<int>(hozon::perception::base::RunningMode::ALL)) {
    if (last_runmode != runmode) {
      ResumeTrigger("recv_ins_fusion");
      ResumeTrigger("recv_pose_estimation");
      ResumeTrigger("recv_dr");
      HLOG_INFO << "!!!!!!!!!!get run mode DRIVER & UNKNOWN";
      last_runmode = runmode;
    }
    // HLOG_ERROR << "!!!!!!!!!!get run mode DRIVER & ALL";
  }
  ++running_mode_count;
  if (running_mode_count >= 100) {
    running_mode_count = 0;
    HLOG_INFO << "on running model heartbeat";
  }
  return 0;
}

}  // namespace loc
}  // namespace mp
}  // namespace hozon
