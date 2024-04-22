/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： ins_fusion.cc
 *   author     ： nihongjie
 *   date       ： 2023.09
 ******************************************************************************/
#include "onboard/onboard_lite/location/ins_fusion/ins_fusion_lite.h"
#include <gflags/gflags.h>
#include <perception-lib/lib/environment/environment.h>

#include "base/utils/log.h"
#include "depend/proto/localization/node_info.pb.h"
#include "depend/proto/perception/transport_element.pb.h"
#include "depend/proto/soc/sensor_imu_ins.pb.h"
#include "modules/util/include/util/mapping_log.h"
#include "modules/util/include/util/rviz_agent/rviz_agent.h"
#include "yaml-cpp/yaml.h"
#include "perception-base/base/state_machine/state_machine_info.h"

namespace hozon {
namespace perception {
namespace common_onboard {

const char* const kInsFusionConfSuffix =
    "runtime_service/mapping/conf/mapping/location/ins_fusion/ins_config.yaml";
const char* const KInsFusionLiteConfig =
    "runtime_service/mapping/conf/mapping/location/ins_fusion/"
    "ins_fusion_lite_config.yaml";

int32_t InsFusionLite::AlgInit() {
  const std::string adflite_root_path =
      hozon::perception::lib::GetEnv("ADFLITE_ROOT_PATH", ".");
  const std::string ins_fusion_config =
      adflite_root_path + "/" + kInsFusionConfSuffix;
  const std::string ins_fusion_lite_config =
      adflite_root_path + "/" + KInsFusionLiteConfig;
  ins_fusion_ = std::make_unique<hozon::mp::loc::InsFusion>();
  if (ins_fusion_->Init(ins_fusion_config) !=
      hozon::mp::loc::InsInitStatus::OK) {
    return -1;
  }

  REGISTER_PROTO_MESSAGE_TYPE("inspva", hozon::localization::HafNodeInfo);
  REGISTER_PROTO_MESSAGE_TYPE("imu_ins", hozon::soc::ImuIns);
  REGISTER_PROTO_MESSAGE_TYPE("gnssinfo", hozon::soc::gnss::GnssInfo);
  REGISTER_PROTO_MESSAGE_TYPE("/location/ins_fusion",
                              hozon::localization::HafNodeInfo);
  REGISTER_PROTO_MESSAGE_TYPE("running_mode",
                              hozon::perception::common_onboard::running_mode);
  HLOG_INFO << "RegistAlgProcessFunc";
  // 接收数据线程
  RegistAlgProcessFunc("receive_ins", std::bind(&InsFusionLite::receive_ins,
                                                this, std::placeholders::_1));
  RegistAlgProcessFunc("receive_gnss", std::bind(&InsFusionLite::receive_gnss,
                                                this, std::placeholders::_1));
  RegistAlgProcessFunc(
      "receive_inspva",
      std::bind(&InsFusionLite::receive_inspva, this, std::placeholders::_1));
  RegistAlgProcessFunc(
      "recv_running_mode",
      std::bind(&InsFusionLite::OnRunningMode, this, std::placeholders::_1));

  HLOG_INFO << "AlgInit successfully ";
  YAML::Node config = YAML::LoadFile(ins_fusion_lite_config);
  auto use_rviz_bridge = config["use_rviz_bridge"].as<bool>();
  if (use_rviz_bridge) {
    auto viz_addr = config["viz_addr"].as<std::string>();
    int ret = mp::util::RvizAgent::Instance().Init(viz_addr);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent init failed:" << viz_addr;
    }
  }
  return 0;
}

void InsFusionLite::AlgRelease() {
  if (mp::util::RvizAgent::Instance().Ok()) {
    mp::util::RvizAgent::Instance().Term();
  }
}

// recieve in-process data and interprocess data
int32_t InsFusionLite::receive_ins(Bundle* input) {
  static double last_ins_time = -1.0;
  auto ptr_rec_ins = input->GetOne("imu_ins");
  auto phm_fault = hozon::perception::lib::FaultManager::Instance();
  static bool input_data_loss_error_flag = false;
  static bool input_data_value_error_flag = false;
  static bool input_data_time_error_flag = false;
  if (!ptr_rec_ins) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
                        INS_DATA_ERROR_MUL_FPS,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 3, 50));
        input_data_loss_error_flag = true;
    HLOG_ERROR << "INS: receive ins is null";
    return -1;
  } else {
    if (input_data_loss_error_flag) {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::INS_DATA_ERROR_MUL_FPS,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
      input_data_loss_error_flag = false;
    }
  }
  std::shared_ptr<hozon::soc::ImuIns> ins_proto =
      std::static_pointer_cast<hozon::soc::ImuIns>(ptr_rec_ins->proto_msg);
  if (!ins_proto) {
    return -1;
  }
  auto ins_workflow = std::make_shared<hozon::netaos::adf_lite::BaseData>();
  std::shared_ptr<hozon::localization::HafNodeInfo> msg(
      new hozon::localization::HafNodeInfo);
  bool flag = ins_fusion_->OnOriginIns(*ins_proto.get(), msg.get());
  if (flag) {
    ins_workflow->proto_msg = msg;
    SendOutput("/location/ins_fusion", ins_workflow);
  }
  double cur_ins_time =
      ins_proto->header().sensor_stamp().imuins_stamp();
  if (last_ins_time > 0) {
    if (cur_ins_time - last_ins_time < 0) {
      phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
                        INS_DATA_TIME_ERROR_MUL_FPS,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 3, 50));
        input_data_time_error_flag = true;
      HLOG_ERROR << "INS: receive ins data stamp is error";
    } else {
      if (input_data_time_error_flag) {
        phm_fault->Report(MAKE_FM_TUPLE(
            hozon::perception::base::FmModuleId::MAPPING,
            hozon::perception::base::FaultType::INS_DATA_TIME_ERROR_MUL_FPS,
            hozon::perception::base::FaultStatus::RESET,
            hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
        input_data_time_error_flag = false;
      }
    }
    if (cur_ins_time - last_ins_time >= 0.03) {
      phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
                        INS_DATA_TIME_ERROR_MUL_FPS,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 3, 50));
        input_data_time_error_flag = true;
      HLOG_ERROR << "INS: receive ins data stamp is delay";
    } else {
      if (input_data_time_error_flag) {
        phm_fault->Report(MAKE_FM_TUPLE(
            hozon::perception::base::FmModuleId::MAPPING,
            hozon::perception::base::FaultType::INS_DATA_TIME_ERROR_MUL_FPS,
            hozon::perception::base::FaultStatus::RESET,
            hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
        input_data_time_error_flag = false;
      }
    }
  }
  if (isNan(ins_proto->ins_info().linear_acceleration()) ||
      isNan(ins_proto->ins_info().augular_velocity())) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::INS_DATA_VALUE_ERROR_MUL_FPS,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 3, 50));
        input_data_value_error_flag = true;
    HLOG_ERROR << "INS: receive ins data Nan";
  } else {
    if (input_data_value_error_flag) {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::INS_DATA_VALUE_ERROR_MUL_FPS,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
      input_data_value_error_flag = false;
    }
  }
  last_ins_time = cur_ins_time;
  return 0;
}

int32_t InsFusionLite::receive_gnss(Bundle* input) {
  static double last_gnss_time = -1.0;
  auto ptr_rec_gnss = input->GetOne("gnssinfo");
  auto phm_fault = hozon::perception::lib::FaultManager::Instance();
  if (!ptr_rec_gnss) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
                        GNSS_DATA_ERROR_MUL_FPS,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 3, 500));
    HLOG_ERROR << "INS: receive gnss is null";
    return -1;
  } else {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
                        GNSS_DATA_ERROR_MUL_FPS,
        hozon::perception::base::FaultStatus::RESET,
        hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
  }
  std::shared_ptr<hozon::soc::gnss::GnssInfo> gnss_proto =
      std::static_pointer_cast<hozon::soc::gnss::GnssInfo>(ptr_rec_gnss->proto_msg);
  if (!gnss_proto) {
    return -1;
  }
  double cur_gnss_time =
      gnss_proto->header().sensor_stamp().gnss_stamp();
  if (last_gnss_time > 0) {
    if (cur_gnss_time - last_gnss_time < 0) {
      phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
                        GNSS_DATA_TIME_ERROR_MUL_FPS,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 3, 500));
      HLOG_ERROR << "INS: receive gnss data stamp is error";
    } else {
      phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
                        GNSS_DATA_TIME_ERROR_MUL_FPS,
        hozon::perception::base::FaultStatus::RESET,
        hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
    }
    if (cur_gnss_time - last_gnss_time >= 0.3) {
      phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
                        GNSS_DATA_TIME_ERROR_MUL_FPS,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 3, 500));
      HLOG_ERROR << "INS: receive gnss data stamp is delay";
    } else {
      phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
                        GNSS_DATA_TIME_ERROR_MUL_FPS,
        hozon::perception::base::FaultStatus::RESET,
        hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
    }
  }
  double longi = gnss_proto->gnss_pos().longitude();
  double lati = gnss_proto->gnss_pos().latitude();
  if (std::isnan(longi) || std::isnan(lati)) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::GNSS_DATA_VALUE_ERROR_MUL_FPS,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 3, 500));
    HLOG_ERROR << "GNSS: receive gnss data Nan";
  } else {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::GNSS_DATA_VALUE_ERROR_MUL_FPS,
        hozon::perception::base::FaultStatus::RESET,
        hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
  }
  last_gnss_time = cur_gnss_time;
  double diff_age = gnss_proto->gnss_pos().diff_age();
  if (std::isnan(diff_age)) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::CORRECTION_SERVICE_ABNORMAL_FOR_LONG_PERIOD,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 3, 500));
    HLOG_ERROR << "GNSS: receive diff age Nan";
  } else {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::CORRECTION_SERVICE_ABNORMAL_FOR_LONG_PERIOD,
        hozon::perception::base::FaultStatus::RESET,
        hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
  }
  return 0;
}
int32_t InsFusionLite::receive_inspva(Bundle* input) {
  static int inspva_count = 0;
  auto ptr_rec_inspva = input->GetOne("inspva");
  if (!ptr_rec_inspva) {
    HLOG_INFO << "Not receive inspva";
    return -1;
  }

  std::shared_ptr<hozon::localization::HafNodeInfo> inspva_proto =
      std::static_pointer_cast<hozon::localization::HafNodeInfo>(
          ptr_rec_inspva->proto_msg);
  auto ins_workflow = std::make_shared<hozon::netaos::adf_lite::BaseData>();
  std::shared_ptr<hozon::localization::HafNodeInfo> msg(
      new hozon::localization::HafNodeInfo);
  bool flag = ins_fusion_->OnInspva(*inspva_proto.get(), msg.get());
  if (flag) {
    ins_workflow->proto_msg = msg;
    SendOutput("/location/ins_fusion", ins_workflow);
  }
  ++inspva_count;
  if (inspva_count >= 100) {
    inspva_count = 0;
    HLOG_ERROR << "rev plugin ins lite heartbeat";
  }
  return 0;
}

int32_t InsFusionLite::OnRunningMode(Bundle* input) {
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
    PauseTrigger("send_ins_proc");
    PauseTrigger("receive_inspva");
    PauseTrigger("receive_ins");
    PauseTrigger("receive_gnss");
    // HLOG_ERROR << "!!!!!!!!!!get run mode PARKING";
  } else if (runmode == static_cast<int>(
                            hozon::perception::base::RunningMode::DRIVING) ||
             runmode ==
                 static_cast<int>(hozon::perception::base::RunningMode::ALL)) {
    ResumeTrigger("send_ins_proc");
    ResumeTrigger("receive_inspva");
    ResumeTrigger("receive_ins");
    ResumeTrigger("receive_gnss");
    // HLOG_ERROR << "!!!!!!!!!!get run mode DRIVER & ALL";
  }
  return 0;
}

template<typename T>
bool InsFusionLite::isNan(const T& t) {
  return std::isnan(t.x()) | std::isnan(t.y()) | std::isnan(t.z());
}

}  //  namespace common_onboard
}  //  namespace perception
}  //  namespace hozon
