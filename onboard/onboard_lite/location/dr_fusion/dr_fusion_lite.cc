/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： dr_fusion.cc
 *   author     ： nihongjie
 *   date       ： 2023.09
 ******************************************************************************/
#include "onboard/onboard_lite/location/dr_fusion/dr_fusion_lite.h"
#include <gflags/gflags.h>
#include <perception-lib/lib/environment/environment.h>

#include "base/utils/log.h"
#include "depend/proto/localization/node_info.pb.h"
#include "modules/util/include/util/mapping_log.h"
#include "modules/util/include/util/rviz_agent/rviz_agent.h"
#include "yaml-cpp/yaml.h"
#include "perception-base/base/state_machine/state_machine_info.h"

namespace hozon {
namespace perception {
namespace common_onboard {

const char* const kDrFusionConfSuffix =
    "runtime_service/mapping/conf/mapping/location/dr_fusion/dr_config.yaml";

int32_t DrFusionLite::AlgInit() {
  const std::string adflite_root_path =
      hozon::perception::lib::GetEnv("ADFLITE_ROOT_PATH", ".");
  const std::string dr_fusion_config =
      adflite_root_path + "/" + kDrFusionConfSuffix;
  dr_fusion_ = std::make_unique<hozon::mp::loc::DrFusion>();

  REGISTER_PROTO_MESSAGE_TYPE("/location/ins_fusion",
                              hozon::localization::HafNodeInfo);
  REGISTER_PROTO_MESSAGE_TYPE("dr", hozon::localization::HafNodeInfo);
  REGISTER_PROTO_MESSAGE_TYPE("/location/dr_fusion",
                              hozon::localization::HafNodeInfo);
  REGISTER_PROTO_MESSAGE_TYPE("running_mode",
                              hozon::perception::common_onboard::running_mode);
  HLOG_INFO << "RegistAlgProcessFunc";
  // 接收数据线程
  RegistAlgProcessFunc("receive_dr", std::bind(&DrFusionLite::receive_dr, this,
                                               std::placeholders::_1));
  RegistAlgProcessFunc(
      "recv_running_mode",
      std::bind(&DrFusionLite::OnRunningMode, this, std::placeholders::_1));

  HLOG_INFO << "AlgInit successfully ";
  return 0;
}

void DrFusionLite::AlgRelease() {
  if (mp::util::RvizAgent::Instance().Ok()) {
    mp::util::RvizAgent::Instance().Term();
  }
}

// recieve in-process data and interprocess data
int32_t DrFusionLite::receive_dr(Bundle* input) {
  static double last_dr_time = -1.0;
  auto phm_fault = hozon::perception::lib::FaultManager::Instance();
  auto ptr_rec_dr = input->GetOne("dr");
  static bool input_data_loss_error_flag = false;
  static bool input_data_value_error_flag = false;
  static bool input_data_time_error_flag = false;
  if (!ptr_rec_dr) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
                        MULTI_FRAME_DR_INPUT_DATA_LOSS,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 3, 50));
        input_data_loss_error_flag = true;
    HLOG_ERROR << "Location: receive dr is null";
    return -1;
  } else {
    if (input_data_loss_error_flag) {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::MULTI_FRAME_DR_INPUT_DATA_LOSS,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
      input_data_loss_error_flag = false;
    }
  }
  std::shared_ptr<hozon::dead_reckoning::DeadReckoning> dr_proto =
      std::static_pointer_cast<hozon::dead_reckoning::DeadReckoning>(
          ptr_rec_dr->proto_msg);

  {
    static double last_log_time = -1.0;
    double curr_log_time = dr_proto->header().data_stamp();
    if (curr_log_time - last_log_time >= 1) {
      HLOG_INFO << "Location: receiving dr normaly";
      last_log_time = curr_log_time;
    }
  }

  if (!dr_proto) {
    return -1;
  }
  double cur_dr_time = dr_proto->header().data_stamp();
  if (last_dr_time > 0) {
    if (cur_dr_time - last_dr_time < 0) {
      phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
                        MULTI_FRAME_DR_INPUT_TIME_ERROR,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 3, 50));
        input_data_time_error_flag = true;
      HLOG_ERROR << "Location: receive dr data stamp is error";
    } else {
      if (input_data_time_error_flag) {
        phm_fault->Report(MAKE_FM_TUPLE(
            hozon::perception::base::FmModuleId::MAPPING,
            hozon::perception::base::FaultType::MULTI_FRAME_DR_INPUT_TIME_ERROR,
            hozon::perception::base::FaultStatus::RESET,
            hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
        input_data_time_error_flag = false;
      }
    }
    if (cur_dr_time - last_dr_time >= 0.03) {
      phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
                        MULTI_FRAME_DR_INPUT_TIME_ERROR,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 3, 50));
        input_data_time_error_flag = true;
      HLOG_ERROR << "Location: receive dr data stamp is delay";
    } else {
      if (input_data_time_error_flag) {
        phm_fault->Report(MAKE_FM_TUPLE(
            hozon::perception::base::FmModuleId::MAPPING,
            hozon::perception::base::FaultType::MULTI_FRAME_DR_INPUT_TIME_ERROR,
            hozon::perception::base::FaultStatus::RESET,
            hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
        input_data_time_error_flag = false;
      }
    }
  }
  last_dr_time = cur_dr_time;
  double pose_x = dr_proto->pose().pose_local().position().x();
  double qua_w = dr_proto->pose().pose_local().quaternion().w();
  double qua_x = dr_proto->pose().pose_local().quaternion().x();
  double qua_y = dr_proto->pose().pose_local().quaternion().y();
  double qua_z = dr_proto->pose().pose_local().quaternion().z();
  double magnitude = std::sqrt(qua_w *qua_w + qua_x *qua_x + qua_y *qua_y + qua_z *qua_z);
  if (std::isnan(pose_x) || (abs(magnitude) < 1e-7)) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::MULTI_FRAME_DR_INPUT_VALUE_ERROR,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 3, 50));
        input_data_value_error_flag = true;
    HLOG_ERROR << "Location: receive dr data Nan";
  } else {
    if (input_data_value_error_flag) {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::MULTI_FRAME_DR_INPUT_VALUE_ERROR,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
      input_data_value_error_flag = false;
    }
  }
  auto dr_workflow = std::make_shared<hozon::netaos::adf_lite::BaseData>();

  std::shared_ptr<hozon::localization::HafNodeInfo> msg(
      new hozon::localization::HafNodeInfo);
  if (!dr_fusion_->OnDr(*dr_proto.get(), msg.get())) {
    return -1;
  }
  dr_workflow->proto_msg = msg;
  SendOutput("/location/dr_fusion", dr_workflow);

  return 0;
}

int32_t DrFusionLite::OnRunningMode(Bundle* input) {
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
    PauseTrigger("receive_dr");
    // HLOG_ERROR << "!!!!!!!!!!get run mode PARKING";
  } else if (runmode == static_cast<int>(
                            hozon::perception::base::RunningMode::DRIVING) ||
             runmode ==
                 static_cast<int>(hozon::perception::base::RunningMode::ALL)) {
    ResumeTrigger("receive_dr");
    // HLOG_ERROR << "!!!!!!!!!!get run mode DRIVER & ALL";
  }
  return 0;
}

}  //  namespace common_onboard
}  //  namespace perception
}  //  namespace hozon
