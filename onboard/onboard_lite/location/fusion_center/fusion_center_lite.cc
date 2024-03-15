/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： fusion_center_lite.cc
 *   author     ： lilanxing
 *   date       ： 2023.09
 ******************************************************************************/
#include <base/utils/log.h>
#include <gflags/gflags.h>
#include <perception-lib/lib/environment/environment.h>

#include "depend/proto/local_mapping/local_map.pb.h"
#include "onboard/onboard_lite/location/fusion_center/fusion_center_lite.h"
#include "perception-base/base/state_machine/state_machine_info.h"

namespace hozon {
namespace perception {
namespace common_onboard {

const char* const kImuTopic = "imu_ins";
const char* const kInsFusionTopic = "/location/ins_fusion";
const char* const kDrFusionTopic = "/location/dr_fusion";
const char* const kPoseEstimationTopic = "/location/pose_estimation";
const char* const kLocalMapTopic = "local_map";
const char* const kFcTopic = "localization";
const char* const kRunningModeTopic = "running_mode";
const char* const kFcConfSuffix =
    "runtime_service/mapping/conf/mapping/"
    "location/fusion_center/fc_config.yaml";
const char* const kFcKfConfSuffix =
    "runtime_service/mapping/conf/mapping/location/fusion_center/kalman.yaml";
const char* const kFcEskfConfSuffix =
    "runtime_service/mapping/conf/mapping/location/fusion_center/eskf.yaml";
const char* const kFcMonitorConfSuffix =
    "runtime_service/mapping/conf/mapping/location/fusion_center/monitor.yaml";
const char* const kCoordAdapterConf =
    "runtime_service/mapping/conf/mapping/location/coord_adapter/config.yaml";

int32_t FusionCenterLite::AlgInit() {
  const std::string adflite_root_path =
      hozon::perception::lib::GetEnv("ADFLITE_ROOT_PATH", ".");
  const std::string fc_config = adflite_root_path + "/" + kFcConfSuffix;
  const std::string fc_kf_config = adflite_root_path + "/" + kFcKfConfSuffix;
  const std::string fc_eskf_config =
      adflite_root_path + "/" + kFcEskfConfSuffix;
  const std::string fc_monitor_config =
      adflite_root_path + "/" + kFcMonitorConfSuffix;

  fusion_center_ = std::make_unique<FusionCenter>();
  if (!fusion_center_->Init(fc_config, fc_kf_config, fc_eskf_config,
                            fc_monitor_config)) {
    return -1;
  }
  const std::string coord_adapter_conf =
      adflite_root_path + "/" + kCoordAdapterConf;
  coord_adapter_ = std::make_unique<CoordAdapter>();
  if (!coord_adapter_->Init(coord_adapter_conf)) {
    return -1;
  }
  RegistMessageType();
  RegistProcessFunc();

  return 0;
}

void FusionCenterLite::AlgRelease() {}

void FusionCenterLite::RegistMessageType() const {
  REGISTER_PROTO_MESSAGE_TYPE(kImuTopic, hozon::soc::ImuIns);
  REGISTER_PROTO_MESSAGE_TYPE(kInsFusionTopic,
                              hozon::localization::HafNodeInfo);
  REGISTER_PROTO_MESSAGE_TYPE(kDrFusionTopic, hozon::localization::HafNodeInfo);
  REGISTER_PROTO_MESSAGE_TYPE(kPoseEstimationTopic,
                              hozon::localization::HafNodeInfo);
  REGISTER_PROTO_MESSAGE_TYPE(kFcTopic, hozon::localization::Localization);
  REGISTER_PROTO_MESSAGE_TYPE(kRunningModeTopic,
                              hozon::perception::common_onboard::running_mode);
}

void FusionCenterLite::RegistProcessFunc() {
  RegistAlgProcessFunc(
      "recv_ins_fusion",
      std::bind(&FusionCenterLite::OnInsFusion, this, std::placeholders::_1));
  RegistAlgProcessFunc(
      "recv_dr_fusion",
      std::bind(&FusionCenterLite::OnDrFusion, this, std::placeholders::_1));
  // temp remove
  // RegistAlgProcessFunc(
  //     "recv_local_map",
  //     std::bind(&FusionCenterLite::OnLocalMap, this, std::placeholders::_1));
  RegistAlgProcessFunc("recv_pose_estimation",
                       std::bind(&FusionCenterLite::OnPoseEstimation, this,
                                 std::placeholders::_1));
  RegistAlgProcessFunc(
      "recv_running_mode",
      std::bind(&FusionCenterLite::OnRunningMode, this, std::placeholders::_1));
}

int32_t FusionCenterLite::OnInsFusion(Bundle* input) {
  auto phm_fault = hozon::perception::lib::FaultManager::Instance();
  static bool input_data_init_error_flag = false;
  if (!input) {
    return -1;
  }

  auto p_ins_fusion = input->GetOne(kInsFusionTopic);
  if (!p_ins_fusion) {
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
  } else {
    if (input_data_init_error_flag) {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::LOCALIZATION_CAN_NOT_INIT,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
      input_data_init_error_flag = false;
    }
  }

  fusion_center_->OnIns(*ins_fusion);
  return 0;
}

int32_t FusionCenterLite::OnDrFusion(Bundle* input) {
  auto phm_fault = hozon::perception::lib::FaultManager::Instance();
  static bool input_data_value_error_flag = false;
  if (!input) {
    return -1;
  }
  auto p_dr_fusion = input->GetOne(kDrFusionTopic);
  if (!p_dr_fusion) {
    return -1;
  }
  const auto dr_fusion =
      std::static_pointer_cast<hozon::localization::HafNodeInfo>(
          p_dr_fusion->proto_msg);
  if (!dr_fusion) {
    return -1;
  }
  if (!coord_adapter_->IsCoordInitSucc()) {
    fusion_center_->OnInitDR(*dr_fusion);
    coord_adapter_->OnDrFusion(*dr_fusion);
  }
  fusion_center_->OnDR(*dr_fusion);

  // send output
  auto localization = std::make_shared<hozon::localization::Localization>();
  if (!fusion_center_->GetCurrentOutput(localization.get())) {
    HLOG_ERROR << "onboard get localization result error";
    return -1;
  }
  double pose_x = localization->pose_local().position().x();
  double pose_y = localization->pose_local().position().y();
  double qua_w = localization->pose_local().quaternion().w();
  double qua_x = localization->pose_local().quaternion().x();
  double qua_y = localization->pose_local().quaternion().y();
  double qua_z = localization->pose_local().quaternion().z();
  if (std::isnan(pose_x) || std::isnan(pose_y) ||
      (qua_w == 0 && qua_x == 0 && qua_y == 0 && qua_z == 0)) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
                        LOCALIZATION_POSE_AND_ATTITUDE_CRITICAL_ABNORMALITY,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 3, 50));
    input_data_value_error_flag = true;
    HLOG_ERROR << "Location: output Nan";
  } else {
    if (input_data_value_error_flag) {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::
                          LOCALIZATION_POSE_AND_ATTITUDE_CRITICAL_ABNORMALITY,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
      input_data_value_error_flag = false;
    }
  }
  auto localization_pack =
      std::make_shared<hozon::netaos::adf_lite::BaseData>();
  localization_pack->proto_msg = localization;
  SendOutput(kFcTopic, localization_pack);

  return 0;
}

int32_t FusionCenterLite::OnLocalMap(Bundle* input) {
  if (!input) {
    return -1;
  }
  if (init_dr_) {
    return 0;
  }
  static double last_lm_time = -1.0;
  auto phm_fault = hozon::perception::lib::FaultManager::Instance();
  auto p_local_map = input->GetOne(kLocalMapTopic);
  if (!p_local_map) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
            MULTI_FRAME_LOCALMAPPING_INPUT_DATA_LOSS,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 3, 500));
    HLOG_ERROR << "Location:localmap input data loss";
    return -1;
  }
  phm_fault->Report(
      MAKE_FM_TUPLE(hozon::perception::base::FmModuleId::MAPPING,
                    hozon::perception::base::FaultType::
                        MULTI_FRAME_LOCALMAPPING_INPUT_DATA_LOSS,
                    hozon::perception::base::FaultStatus::RESET,
                    hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
  const auto local_map = std::static_pointer_cast<hozon::mapping::LocalMap>(
      p_local_map->proto_msg);
  if (!local_map) {
    return -1;
  }
  double cur_lm_time = local_map->header().data_stamp();
  if (last_lm_time > 0) {
    if (last_lm_time - cur_lm_time > 0) {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::
              MULTI_FRAME_LOCALMAPPING_INPUT_TIME_ERROR,
          hozon::perception::base::FaultStatus::OCCUR,
          hozon::perception::base::SensorOrientation::UNKNOWN, 3, 500));
      HLOG_ERROR << "Location:receieve localmap time error";
    } else {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::
              MULTI_FRAME_LOCALMAPPING_INPUT_TIME_ERROR,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
    }
  }
  last_lm_time = cur_lm_time;
  // coord_adapter_->OnLocalMap(*local_map);
  // if (!coord_adapter_->IsCoordInitSucc()) {
  //   return -1;
  // }
  // const auto& init_dr = coord_adapter_->GetSysInitDrFusion();
  // fusion_center_->OnInitDR(init_dr);
  // if (coord_adapter_->IsCoordInitSucc()) {
  //   init_dr_ = true;
  // } else {
  //   HLOG_ERROR << "OnInitDR Failed";
  // }

  return 0;
}

int32_t FusionCenterLite::OnPoseEstimation(Bundle* input) {
  if (!input) {
    return -1;
  }
  auto p_pose_estimation = input->GetOne(kPoseEstimationTopic);
  if (!p_pose_estimation) {
    HLOG_ERROR << "FC !p_pose_estimation";
    return -1;
  }

  const auto pose_estimation =
      std::static_pointer_cast<hozon::localization::HafNodeInfo>(
          p_pose_estimation->proto_msg);
  if (!pose_estimation) {
    return -1;
  }

  fusion_center_->OnPoseEstimate(*pose_estimation);
  return 0;
}

int32_t FusionCenterLite::OnRunningMode(Bundle* input) {
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
  if (runmode ==
      static_cast<int>(hozon::perception::base::RunningMode::PARKING)) {
    PauseTrigger("recv_ins_fusion");
    PauseTrigger("recv_pose_estimation");
    PauseTrigger("recv_dr_fusion");
    // HLOG_ERROR << "!!!!!!!!!!get run mode PARKING";
  } else if (runmode == static_cast<int>(
                            hozon::perception::base::RunningMode::DRIVING) ||
             runmode ==
                 static_cast<int>(hozon::perception::base::RunningMode::ALL)) {
    ResumeTrigger("recv_ins_fusion");
    ResumeTrigger("recv_pose_estimation");
    ResumeTrigger("recv_dr_fusion");
    // HLOG_ERROR << "!!!!!!!!!!get run mode DRIVER & ALL";
  }
  return 0;
}

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
