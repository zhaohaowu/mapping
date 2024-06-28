/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-09-09
 *****************************************************************************/
#include "onboard/onboard_lite/laneline_postprocess/env_bevfusion_onboard_lite.h"

#include "base/state_machine/state_machine_info.h"
#include "modules/laneline_postprocess/lib/laneline/interface/base_lane_process.h"
#include "modules/laneline_postprocess/lib/laneline/interface/base_roadedge_process.h"
#include "onboard/onboard_lite/laneline_postprocess/data_mapping/data_mapping.h"
#include "onboard/onboard_lite/laneline_postprocess/measurement_message.h"
#include "onboard/onboard_lite/phm_comment_lite/proto/running_mode.pb.h"
#include "perception-lib/lib/environment/environment.h"
#include "perception-lib/lib/fault_manager/fault_manager.h"
#include "perception-lib/lib/health_manager/health_manager.h"
#include "perception-lib/lib/state_manager/state_manager.h"

namespace hozon {
namespace mp {
namespace environment_onboard {

// EnvBevfusionOnboard::EnvBevfusionOnboard() {}
// EnvBevfusionOnboard::~EnvBevfusionOnboard() {}

int32_t EnvBevfusionOnboard::AlgInit() {
  HLOG_INFO << "EnvBevfusionOnboard AlgInit Start...";

  perception_lib::LocationManager::Instance()->Init(100, 0.5);

  // HLOG_DEBUG << "common_onboard::FLAGS_env_laneline_using_recv_proto: "
  //           << common_onboard::FLAGS_env_laneline_using_recv_proto;
  REGISTER_PROTO_MESSAGE_TYPE("percep_detection",
                              hozon::perception::measurement::MeasurementPb);

  REGISTER_PROTO_MESSAGE_TYPE("dr", hozon::dead_reckoning::DeadReckoning);
  REGISTER_PROTO_MESSAGE_TYPE("running_mode",
                              hozon::perception::common_onboard::running_mode);
  RegistAlgProcessFunc("recv_detection_bevfusion_lane_proto",
                       std::bind(&EnvBevfusionOnboard::ReceiveDetectLaneLine,
                                 this, std::placeholders::_1));

  RegistAlgProcessFunc("recv_dr", std::bind(&EnvBevfusionOnboard::ReceiveDr,
                                            this, std::placeholders::_1));
  RegistAlgProcessFunc("recv_running_mode",
                       std::bind(&EnvBevfusionOnboard::OnRunningMode, this,
                                 std::placeholders::_1));
  // std::string default_work_root = "/app/";
  // std::string work_root = lib::GetEnv("ADFLITE_ROOT_PATH",
  // default_work_root); if (work_root == "") {
  //   HLOG_ERROR << "ENV: ADFLITE_ROOT_PATH is not set.";
  //   return false;
  // }

  // 车道线后处理初始化
  lane_postprocessor_ = std::make_unique<environment::LanePostProcess>();
  environment::ProcessInitOption init_option;
  CHECK(lane_postprocessor_->Init(init_option));
  HLOG_DEBUG << "lane_postprocessor_ init successfully";

  // 路沿后处理初始化
  roadedge_postprocessor_ =
      std::make_unique<environment::RoadEdgePostProcess>();
  CHECK(roadedge_postprocessor_->Init(init_option));
  HLOG_DEBUG << "roadedge_postprocessor_ init successfully";
  HLOG_INFO << "EnvBevfusionOnboard AlgInit End...";

  return 0;
}

void EnvBevfusionOnboard::AlgRelease() {}
// // 状态机行泊信号
int32_t EnvBevfusionOnboard::OnRunningMode(adf_lite_Bundle* input) {
  static int running_mode_count = 0;
  auto rm_msg = input->GetOne("running_mode");
  if (rm_msg == nullptr) {
    HLOG_ERROR << "nullptr rm_msg plugin";
    return -1;
  }
  auto msg =
      std::static_pointer_cast<hozon::perception::common_onboard::running_mode>(
          rm_msg->proto_msg);
  int runmode = msg->mode();
  HLOG_DEBUG << " *** get run mode : *** " << runmode;
  static int last_runmode =
      static_cast<int>(hozon::perception::base::RunningMode::DRIVING);
  if (runmode ==
      static_cast<int>(hozon::perception::base::RunningMode::PARKING)) {
    if (last_runmode != runmode) {
      PauseTrigger("recv_detection_bevfusion_lane_proto");
      PauseTrigger("recv_dr");
      last_runmode = runmode;
      HLOG_INFO << "!!!!!!!!!!get run mode PARKING";
    }
    HLOG_DEBUG << "!!!!!!!!!!get run mode PARKING";
  } else if (runmode == static_cast<int>(
                            hozon::perception::base::RunningMode::DRIVING) ||
             runmode ==
                 static_cast<int>(hozon::perception::base::RunningMode::ALL)) {
    if (last_runmode != runmode) {
      ResumeTrigger("recv_detection_bevfusion_lane_proto");
      ResumeTrigger("recv_dr");
      last_runmode = runmode;
      HLOG_INFO << "!!!!!!!!!!get run mode DRIVER & UNKNOWN";
    }
    // HLOG_DEBUG << "!!!!!!!!!!get run mode DRIVER & UNKNOWN";
  }

  ++running_mode_count;
  if (running_mode_count >= 100) {
    running_mode_count = 0;
    HLOG_INFO << "on running model heartbeat";
  }
  return 0;
}
int32_t EnvBevfusionOnboard::ReceiveDr(adf_lite_Bundle* input) {
  static int running_mode_count = 0;
  ++running_mode_count;
  if (running_mode_count >= 100) {
    running_mode_count = 0;
    HLOG_INFO << " * **RECEIVE DR DATA * **";
  }

  auto dr_msg = input->GetOne("dr");
  if (dr_msg == nullptr) {
    HLOG_ERROR << " dr_msg is nullptr. ";
    return -1;
  }

  auto pb_location =
      std::static_pointer_cast<hozon::dead_reckoning::DeadReckoning>(
          dr_msg->proto_msg);

  location_msg_ = std::make_shared<perception_base::Location>();
  common_onboard::DataMapping::CvtPbDR2Location(pb_location, location_msg_);
  perception_lib::LocationManager::Instance()->Push(*location_msg_);
  return 0;
}

int32_t EnvBevfusionOnboard::ReceiveDetectLaneLine(adf_lite_Bundle* input) {
  HLOG_INFO << "LaneLine PostProcess Lite start!!!";
  if (nullptr == input) {
    HLOG_ERROR << "input bundle is nullptr.";
    return -1;
  }

  auto percept_detection = input->GetOne("percep_detection");
  if (!percept_detection) {
    HLOG_ERROR << "perception detection is nullptr";
    return -1;
  }

  auto pbdata =
      std::static_pointer_cast<hozon::perception::measurement::MeasurementPb>(
          percept_detection->proto_msg);

  std::shared_ptr<common_onboard::MeasurementMessage> data_msg;

  data_msg = std::make_shared<common_onboard::MeasurementMessage>();
  data_msg->measurement_frame =
      std::make_shared<perception_base::MeasurementFrame>();
  if (!common_onboard::DataMapping::CvtPb2Measurement(
          pbdata, data_msg->measurement_frame)) {
    HLOG_ERROR << "DataMapping::CvtPb2Measurement failed";
    return -1;
  }

  if (!data_msg->measurement_frame) {
    HLOG_ERROR << "data_msg->measurement_frame is nullptr";
    return -1;
  }

  auto transport_element =
      std::make_shared<common_onboard::NetaTransportElement>();
  auto fusion_msg = std::make_shared<common_onboard::FusionMessage>();
  fusion_msg->fusion_frame = std::make_shared<perception_base::FusionFrame>();

  // 输入数据放入输入数据管理模块中。
  double ts = data_msg->measurement_frame->header.timestamp;
  perception_base::LocationPtr cur_location =
      std::make_shared<perception_base::Location>();
  bool ret =
      perception_lib::LocationManager::Instance()->GetLocationByTimestamp(
          ts, cur_location.get());
  if (!ret) {
    HLOG_ERROR << "get location msg failed...";
    return false;
  }

  environment::InputDataSingleton* local_data_ =
      environment::InputDataSingleton::Instance();
  local_data_->dr_data_buffer_.push_new_message(ts, cur_location);
  local_data_->roadedges_buffer_.push_new_message(
      ts, data_msg->measurement_frame->roadedges_measurement_->road_edges);
  local_data_->lanes_buffer_.push_new_message(
      ts, data_msg->measurement_frame->lanelines_measurement_->lanelines);

  // 车道线后处理和结果转换
  HLOG_INFO << "do laneline postprocess...";
  if (!lane_postprocessor_) {
    HLOG_ERROR << "lane_postprocessor_ is nullptr";
    return -1;
  }
  lane_postprocessor_->Process(data_msg->measurement_frame,
                               fusion_msg->fusion_frame);
  HLOG_DEBUG << "output track laneline nums trans to pb:"
             << fusion_msg->fusion_frame->scene_->lane_lines->lanelines.size();

  if (!common_onboard::DataMapping::CvtMultiLanesToPb(
          fusion_msg->fusion_frame->scene_->lane_lines->lanelines,
          transport_element)) {
    HLOG_ERROR << "multi laneline convert to proto struct failed.";
    // return -1;
  }

  // 路沿后处理和结果转换
  HLOG_INFO << "do roadedge_postprocess...";
  if (!roadedge_postprocessor_) {
    HLOG_ERROR << "roadedge_postprocessor_ is nullptr";
    return -1;
  }
  roadedge_postprocessor_->Process(data_msg->measurement_frame,
                                   fusion_msg->fusion_frame);
  HLOG_DEBUG << "output track road_edges nums trans to pb:"
             << fusion_msg->fusion_frame->scene_->road_edges->road_edges.size();

  if (!common_onboard::DataMapping::CvtMultiRoadEdgesToPb(
          fusion_msg->fusion_frame->scene_->road_edges->road_edges,
          transport_element)) {
    HLOG_ERROR << "multi laneline convert to proto struct failed.";
    // return -1;
  }

  // 箭头、斑马线、停止线等其他路面元素透传
  *(transport_element->mutable_arrow()) = pbdata->transport_element().arrow();
  *(transport_element->mutable_zebra_crossing()) =
      pbdata->transport_element().zebra_crossing();
  *(transport_element->mutable_stopline()) =
      pbdata->transport_element().stopline();
  *(transport_element->mutable_slow_downs()) =
      pbdata->transport_element().slow_downs();
  *(transport_element->mutable_cross_points()) =
      pbdata->transport_element().cross_points();
  *(transport_element->mutable_turn_waiting_zone()) =
      pbdata->transport_element().turn_waiting_zone();
  *(transport_element->mutable_no_parking_zone()) =
      pbdata->transport_element().no_parking_zone();

  transport_element->mutable_header()->set_data_stamp(
      data_msg->measurement_frame->header.timestamp);
  transport_element->mutable_header()->set_seq(
      data_msg->measurement_frame->header.sequence_num);
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
      tp = std::chrono::time_point_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now());
  transport_element->mutable_header()->set_publish_stamp(
      tp.time_since_epoch().count() * 1.0e-9);
  auto EnvDataPtr = std::make_shared<hozon::netaos::adf_lite::BaseData>();
  EnvDataPtr->proto_msg = transport_element;

  adf_lite_Bundle lane_node_bundle;
  lane_node_bundle.Add("percep_transport", EnvDataPtr);
  SendOutput(&lane_node_bundle);

  HLOG_INFO << "LaneLine PostProcess Lite End...";
  return 0;
}

// REGISTER_EXECUTOR_CLASS(EnvBevfusionOnboard, EnvBevfusionOnboard);

}  // namespace environment_onboard
}  // namespace mp
}  // namespace hozon
