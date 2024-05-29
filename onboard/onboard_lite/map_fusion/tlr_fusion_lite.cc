/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： tlr_fusion_lite.cc
 *   author     ： taoshaoyuan
 *   date       ： 2023.12
 ******************************************************************************/

#include "onboard/onboard_lite/map_fusion/tlr_fusion_lite.h"

#include <memory>

#include "base/utils/log.h"
#include "modules/util/include/util/mapping_log.h"
#include "perception-base/base/state_machine/state_machine_info.h"

namespace hozon {
namespace perception {
namespace common_onboard {

int32_t TlrFusionLite::AlgInit() {
  REGISTER_PROTO_MESSAGE_TYPE("tlr_percep",
                              hozon::perception::TrafficLightDetection);
  REGISTER_PROTO_MESSAGE_TYPE("tlr_result", hozon::hdmap::JunctionPassable);
  REGISTER_PROTO_MESSAGE_TYPE("running_mode",
                              hozon::perception::common_onboard::running_mode);
  tlr_fusion_ = std::make_shared<hozon::mp::mf::TlrFusion>();

  RegistAlgProcessFunc("recv_tlr_percep", std::bind(&TlrFusionLite::OnTlr, this,
                                                    std::placeholders::_1));
  RegistAlgProcessFunc(
      "recv_running_mode",
      std::bind(&TlrFusionLite::OnRunningMode, this, std::placeholders::_1));
  return 0;
}

void TlrFusionLite::AlgRelease() {}

int32_t TlrFusionLite::OnTlr(hozon::netaos::adf_lite::Bundle* input) {
  if (tlr_fusion_ == nullptr) {
    HLOG_ERROR << "nullptr TlrFusion";
    return -1;
  }

  if (input == nullptr) {
    HLOG_ERROR << "nullptr input Bundle";
    return -1;
  }

  auto input_msg = input->GetOne("tlr_percep");
  if (input_msg == nullptr) {
    HLOG_ERROR << "GetOne tlr_percep nullptr";
    return -1;
  }

  const auto tlr_percep =
      std::static_pointer_cast<hozon::perception::TrafficLightDetection>(
          input_msg->proto_msg);
  if (tlr_percep == nullptr) {
    HLOG_ERROR << "proto_msg is nullptr in Bundle input";
    return -1;
  }

  auto tlr_result = std::make_shared<hozon::hdmap::JunctionPassable>();
  if (tlr_fusion_->Proc(tlr_percep, tlr_result.get()) < 0) {
    HLOG_ERROR << "TlrFusion::Proc failed";
    return -1;
  }

  auto output_msg = std::make_shared<hozon::netaos::adf_lite::BaseData>();
  output_msg->proto_msg = tlr_result;
  SendOutput("tlr_fusion", output_msg);
  HLOG_DEBUG << "SendOutput tlr_fusion:\n" << tlr_result->DebugString();

  return 0;
}

int32_t TlrFusionLite::OnRunningMode(hozon::netaos::adf_lite::Bundle* input) {
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
      PauseTrigger("recv_tlr_percep");
      last_runmode = runmode;
      HLOG_INFO << "!!!!!!!!!!get run mode PARKING";
    }
    // HLOG_ERROR << "!!!!!!!!!!get run mode PARKING";
  } else if (runmode == static_cast<int>(
                            hozon::perception::base::RunningMode::DRIVING) ||
             runmode ==
                 static_cast<int>(hozon::perception::base::RunningMode::ALL)) {
    if (last_runmode != runmode) {
      ResumeTrigger("recv_tlr_percep");
      last_runmode = runmode;
      HLOG_INFO << "!!!!!!!!!!get run mode DRIVER & UNKNOWN";
    }
    // HLOG_ERROR << "!!!!!!!!!!get run mode DRIVER & UNKNOWN";
  }
  return 0;
}

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
