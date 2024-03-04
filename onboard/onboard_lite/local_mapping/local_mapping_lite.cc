/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： prior_provider_lite.cc
 *   author     ： taoshaoyuan
 *   date       ： 2023.09
 ******************************************************************************/
#include "onboard/onboard_lite/local_mapping/local_mapping_lite.h"
#include <adf-lite/include/base.h>
#include <base/utils/log.h>
#include <gflags/gflags.h>
#include <proto/localization/node_info.pb.h>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "base/state_machine/state_machine_info.h"
#include "depend/proto/perception/perception_measurement.pb.h"
#include "modules/local_mapping/lib/laneline/interface/base_lane_process.h"
#include "modules/local_mapping/lib/laneline/interface/base_roadedge_process.h"
#include "onboard/onboard_lite/local_mapping/data_mapping/data_mapping.h"
#include "onboard/onboard_lite/local_mapping/measurement_message.h"
#include "onboard/onboard_lite/phm_comment_lite/proto/running_mode.pb.h"
#include "perception-base/base/state_machine/state_machine_info.h"
#include "perception-lib/lib/environment/environment.h"
#include "perception-lib/lib/fault_manager/fault_manager.h"
#include "perception-lib/lib/health_manager/health_manager.h"
#include "perception-lib/lib/state_manager/state_manager.h"
namespace hozon {
namespace mp {
namespace lm {

int32_t LocalMappingOnboard::AlgInit() {
  perception_lib::LocationManager::Instance()->Init(100, 0.5);

  REGISTER_PROTO_MESSAGE_TYPE("percep_detection",
                              hozon::perception::measurement::MeasurementPb);
  // REGISTER_PROTO_MESSAGE_TYPE("percep_transport",
  //                             hozon::perception::TransportElement);
  REGISTER_PROTO_MESSAGE_TYPE("dr", hozon::dead_reckoning::DeadReckoning);
  REGISTER_PROTO_MESSAGE_TYPE("lm_image", hozon::soc::Image);
  REGISTER_PROTO_MESSAGE_TYPE("running_mode",
                              hozon::perception::common_onboard::running_mode);
  // REGISTER_PROTO_MESSAGE_TYPE("/localization/deadreckoning",
  //                             hozon::dead_reckoning::DeadReckoning);

  std::string default_work_root = "/app/";
  std::string work_root =
      hozon::perception::lib::GetEnv("ADFLITE_ROOT_PATH", default_work_root);
  if (work_root.empty()) {
    HLOG_ERROR << "ENV: ADFLITE_ROOT_PATH is not set.";
    return -1;
  }
  std::string config_file = work_root +
                            "/runtime_service/mapping/conf/mapping/"
                            "local_mapping/local_mapping_conf.yaml";
  std::string mapping_path = work_root + "/runtime_service/mapping/";

  YAML::Node config = YAML::LoadFile(config_file);
  if (config["use_rviz"].as<bool>()) {
    HLOG_DEBUG << "Start RvizAgent!!!";
    int ret = RVIZ_AGENT.Init(config["rviz_addr"].as<std::string>());
    if (ret < 0) {
      HLOG_ERROR << "RvizAgent start failed";
    }
  }

  lmap_ = std::make_shared<LMapApp>(mapping_path, config_file);

  RegistAlgProcessFunc("recv_detection",
                       std::bind(&LocalMappingOnboard::OnPerception, this,
                                 std::placeholders::_1));

  RegistAlgProcessFunc("recv_dr", std::bind(&LocalMappingOnboard::OnDr, this,
                                            std::placeholders::_1));
  // NOLINTBEGIN
  RegistAlgProcessFunc("recv_localization",
                       std::bind(&LocalMappingOnboard::Onlocalization, this,
                                 std::placeholders::_1));
  RegistAlgProcessFunc("recv_running_mode",
                       std::bind(&LocalMappingOnboard::OnRunningMode, this,
                                 std::placeholders::_1));
  if (RVIZ_AGENT.Ok()) {
    RegistAlgProcessFunc("recv_ins", std::bind(&LocalMappingOnboard::OnIns,
                                               this, std::placeholders::_1));
    RVIZ_AGENT.Register<adsfi_proto::viz::CompressedImage>("/localmap/image");
    RegistAlgProcessFunc("recv_image", std::bind(&LocalMappingOnboard::OnImage,
                                                 this, std::placeholders::_1));
  }
  // NOLINTEND
  return 0;
}

void LocalMappingOnboard::AlgRelease() {
  RVIZ_AGENT.Term();
  if (hozon::perception::lib::FaultManager::Instance() != nullptr) {
    hozon::perception::lib::FaultManager::Instance()->Reset();
  }
}

int32_t LocalMappingOnboard::OnRunningMode(adf_lite_Bundle* input) {
  auto rm_msg = input->GetOne("running_mode");
  if (rm_msg == nullptr) {
    HLOG_ERROR << "nullptr rm_msg plugin";
    return -1;
  }
  auto msg =
      std::static_pointer_cast<hozon::perception::common_onboard::running_mode>(
          rm_msg->proto_msg);
  int runmode = msg->mode();
  // HLOG_ERROR << "!!!!!!!!!!get run mode : " << runmode;
  if (runmode ==
      static_cast<int>(hozon::perception::base::RunningMode::PARKING)) {
    PauseTrigger("recv_detection");
    PauseTrigger("recv_localization");
    PauseTrigger("recv_dr");
    if (RVIZ_AGENT.Ok()) {
      PauseTrigger("recv_ins");
      PauseTrigger("recv_image");
    }
    // HLOG_ERROR << "!!!!!!!!!!get run mode PARKING";
  } else if (runmode == static_cast<int>(
                            hozon::perception::base::RunningMode::DRIVING) ||
             runmode ==
                 static_cast<int>(hozon::perception::base::RunningMode::ALL)) {
    ResumeTrigger("recv_dr");
    ResumeTrigger("recv_detection");
    ResumeTrigger("recv_localization");
    if (RVIZ_AGENT.Ok()) {
      ResumeTrigger("recv_ins");
      ResumeTrigger("recv_image");
    }
    // HLOG_ERROR << "!!!!!!!!!!get run mode DRIVER & UNKNOWN";
  }
  return 0;
}

int32_t LocalMappingOnboard::OnDr(adf_lite_Bundle* input) {
  HLOG_DEBUG << "*** RECEIVE DR DATA ***";

  auto dr_msg = input->GetOne("dr");
  if (dr_msg == nullptr) {
    HLOG_ERROR << " dr_msg is nullptr. ";
    return -1;
  }

  auto pb_location =
      std::static_pointer_cast<hozon::dead_reckoning::DeadReckoning>(
          dr_msg->proto_msg);

  std::shared_ptr<perception_base::Location> location_msg =
      std::make_shared<perception_base::Location>();
  common_onboard::DataMapping::CvtPbDR2Location(pb_location, location_msg);

  lmap_->OnDr(location_msg);

  return 0;
}

int32_t LocalMappingOnboard::OnPerception(adf_lite_Bundle* input) {
  HLOG_INFO << "*** LocalMappingOnboard Run Start ***";
  if (nullptr == input) {
    HLOG_ERROR << "input is nullptr.";
    return -1;
  }

  auto* phm_fault = hozon::perception::lib::FaultManager::Instance();
  static double last_percep_time = -1.0;
  HLOG_DEBUG << "receive perception data...";
  auto percept_detection = input->GetOne("percep_detection");
  if (!percept_detection) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
            LOCALMAPPING_MUL_FRAM_PERCEPTION_INPUT_DATA_LOSS,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 3, 500));
    HLOG_ERROR << "nullptr detect lane plugin";
    return -1;
  }

  phm_fault->Report(
      MAKE_FM_TUPLE(hozon::perception::base::FmModuleId::MAPPING,
                    hozon::perception::base::FaultType::
                        LOCALMAPPING_MUL_FRAM_PERCEPTION_INPUT_DATA_LOSS,
                    hozon::perception::base::FaultStatus::RESET,
                    hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));

  if (!lmap_) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::LOCALMAPPING_CANNOT_INIT,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 1, 1000));
    HLOG_ERROR << "LMapApp init failed!!!";
    return -1;
  }
  phm_fault->Report(MAKE_FM_TUPLE(
      hozon::perception::base::FmModuleId::MAPPING,
      hozon::perception::base::FaultType::LOCALMAPPING_CANNOT_INIT,
      hozon::perception::base::FaultStatus::RESET,
      hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));

  auto measure_pbdata =
      std::static_pointer_cast<hozon::perception::measurement::MeasurementPb>(
          percept_detection->proto_msg);

  double cur_percep_time = measure_pbdata->header().data_stamp();
  if (last_percep_time > 0) {
    if (cur_percep_time - last_percep_time <= 0 ||
        cur_percep_time - last_percep_time > 0.4) {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::
              LOCALMAPPING_MUL_FRAM_PERCEPTION_INPUT_TIME_ERROR,
          hozon::perception::base::FaultStatus::OCCUR,
          hozon::perception::base::SensorOrientation::UNKNOWN, 3, 500));
      HLOG_ERROR << "receieve laneline time error";
      // std::cout << "last_percep_time: " << std::setprecision(20)
      //           << last_percep_time;
      // std::cout << "cur_percep_time: " << std::setprecision(20)
      //           << cur_percep_time;
    } else {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::
              LOCALMAPPING_MUL_FRAM_PERCEPTION_INPUT_TIME_ERROR,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
    }
  }
  last_percep_time = cur_percep_time;

  // 感知车道线模型输出结果pb转为内部数据结构
  hozon::perception::base::MeasurementFramePtr measurement_frame =
      std::make_shared<hozon::perception::base::MeasurementFrame>();
  if (!common_onboard::DataMapping::CvtPb2Measurement(measure_pbdata,
                                                      measurement_frame)) {
    HLOG_ERROR << "DataMapping::CvtPb2Measurement failed";
    return -1;
  }

  if (!measurement_frame) {
    HLOG_ERROR << "measurement_frame is nullptr";
    return -1;
  }

  util::TicToc global_tic;
  global_tic.Tic();
  hozon::perception::base::FusionFramePtr fusion_frame =
      std::make_shared<hozon::perception::base::FusionFrame>();
  util::TicToc local_tic;
  local_tic.Tic();
  // 后处理模块功能实现（车道线、路沿等元素）
  if (!lmap_->DoPostProcess(measurement_frame, fusion_frame)) {
    // 如果后处理失败， 直接返回。
    return 0;
  }
  HLOG_INFO << "后处理耗时 " << local_tic.Toc() << "ms";
  // 中间模块，为了临时支持其他第三方模块功能联调。未来会做废弃处理。
  PublishPostLaneLine(measure_pbdata, fusion_frame);

  // 将后处理数据结构转换为内部数据结构
  HLOG_DEBUG << "start do localmap work job...";
  std::shared_ptr<Perception> perception = std::make_shared<Perception>();
  DataConvert::SetPerceptionEnv(fusion_frame, perception.get());
  DataConvert::SetPerceptionObj(measurement_frame, perception.get());

  // 建图模块功能实现
  lmap_->DoBuildMap(perception);
  HLOG_DEBUG << "finish do localmap work job...";

  // 发送localmap地图给下游
  PublishLocalMap();

  HLOG_INFO << "总耗时 " << global_tic.Toc() << "ms";
  static double max_time = 0;
  max_time = global_tic.Toc() > max_time ? global_tic.Toc() : max_time;
  HLOG_INFO << "最大耗时 " << max_time << "ms";
  HLOG_INFO << "*** LocalMappingOnboard Run End ***";
  return 0;
}

int32_t LocalMappingOnboard::Onlocalization(adf_lite_Bundle* input) {
  auto* phm_fault = hozon::perception::lib::FaultManager::Instance();
  static double last_localization_time = -1.0;
  static bool input_data_loss_error_flag = false;
  static bool input_data_value_error_flag = false;
  static bool input_data_time_error_flag = false;
  HLOG_DEBUG << "receive localization data...";
  auto localization_msg = input->GetOne("localization");
  if (localization_msg == nullptr) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
            LOCALMAPPING_MUL_FRAM_LOCALIZATION_INPUT_DATA_LOSS,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 3, 50));
    input_data_loss_error_flag = true;
    HLOG_ERROR << "nullptr localization plugin";
    return -1;
  } else {
    if (input_data_loss_error_flag) {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::
              LOCALMAPPING_MUL_FRAM_LOCALIZATION_INPUT_DATA_LOSS,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
      input_data_loss_error_flag = false;
    }
  }

  auto msg = std::static_pointer_cast<hozon::localization::Localization>(
      localization_msg->proto_msg);
  double pose_x = msg->pose_local().position().x();
  double pose_y = msg->pose_local().position().y();
  double qua_w = msg->pose_local().quaternion().w();
  double qua_x = msg->pose_local().quaternion().x();
  double qua_y = msg->pose_local().quaternion().y();
  double qua_z = msg->pose_local().quaternion().z();
  if (std::isnan(pose_x) || std::isnan(pose_y) ||
      (qua_w == 0 && qua_x == 0 && qua_y == 0 && qua_z == 0)) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
            LOCALMAPPING_MUL_FRAM_LOCALIZATION_INPUT_VALUE_ERROR,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 3, 50));
    input_data_value_error_flag = true;
    HLOG_ERROR << "localmapping input localization value error";
  } else {
    if (input_data_value_error_flag) {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::
              LOCALMAPPING_MUL_FRAM_LOCALIZATION_INPUT_VALUE_ERROR,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
      input_data_value_error_flag = false;
    }
  }
  double cur_localization_time = msg->header().data_stamp();
  if (last_localization_time > 0) {
    if (cur_localization_time - last_localization_time <= 0 ||
        cur_localization_time - last_localization_time > 0.2) {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::
              LOCALMAPPING_MUL_FRAM_LOCALIZATION_INPUT_TIME_ERROR,
          hozon::perception::base::FaultStatus::OCCUR,
          hozon::perception::base::SensorOrientation::UNKNOWN, 3, 50));
      input_data_time_error_flag = true;
      // std::cout << "last_localization_time: " << std::setprecision(20)
      //           << last_localization_time;
      // std::cout << "cur_localization_time: " << std::setprecision(20)
      //           << cur_localization_time;
      HLOG_ERROR << "receieve localization time error";
    } else {
      if (input_data_time_error_flag) {
        phm_fault->Report(MAKE_FM_TUPLE(
            hozon::perception::base::FmModuleId::MAPPING,
            hozon::perception::base::FaultType::
                LOCALMAPPING_MUL_FRAM_LOCALIZATION_INPUT_TIME_ERROR,
            hozon::perception::base::FaultStatus::RESET,
            hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
        input_data_time_error_flag = false;
      }
    }
  }
  last_localization_time = cur_localization_time;

  std::shared_ptr<Localization> latest_localization =
      std::make_shared<Localization>();
  DataConvert::SetLocalization(*msg, latest_localization.get());

  lmap_->OnLocalization(latest_localization);
  HLOG_DEBUG << "processed localization data";
  return 0;
}

int32_t LocalMappingOnboard::OnIns(adf_lite_Bundle* input) {
  auto ins_msg = input->GetOne("ins");
  auto msg = std::static_pointer_cast<hozon::localization::HafNodeInfo>(
      ins_msg->proto_msg);
  if (!lmap_) {
    return -1;
  }

  std::shared_ptr<InsData> latest_ins_data = std::make_shared<InsData>();
  DataConvert::SetIns(*msg, latest_ins_data.get());
  lmap_->OnIns(latest_ins_data);
  return 0;
}

std::shared_ptr<adsfi_proto::viz::CompressedImage> YUVNV12ImageToVizImage(
    const std::shared_ptr<hozon::soc::Image>& yuv_image, int quality,
    double resize_factor) {
  if (yuv_image == nullptr) {
    HLOG_ERROR << "nullptr input yuv";
    return nullptr;
  }
  auto width = static_cast<int>(yuv_image->width());
  auto height = static_cast<int>(yuv_image->height());
  size_t nv12_bytes_num = height * width * 3 / 2;
  if (yuv_image->data().size() != nv12_bytes_num) {
    HLOG_ERROR << "invalid yuv data size";
    return nullptr;
  }
  if (quality <= 0 || quality > 100) {
    HLOG_ERROR << "invalid quality, should in range (0, 100]";
    return nullptr;
  }
  if (resize_factor <= 0. || resize_factor > 1.) {
    HLOG_ERROR << "invalid resize factor, should in range (0, 1]";
    return nullptr;
  }
  cv::Mat yuv_nv12;
  yuv_nv12.create(height * 3 / 2, width, CV_8UC1);
  memcpy(yuv_nv12.data, yuv_image->data().data(), nv12_bytes_num);
  cv::Mat bgr;
  cv::cvtColor(yuv_nv12, bgr, cv::COLOR_YUV2BGR_NV12);
  if (resize_factor < 1.0) {
    auto resized_cols = static_cast<int>(bgr.cols * resize_factor);
    auto resized_rows = static_cast<int>(bgr.rows * resize_factor);
    cv::resize(bgr, bgr, cv::Size(resized_cols, resized_rows));
  }
  std::vector<int> param(2);
  param[0] = cv::IMWRITE_JPEG_QUALITY;
  param[1] = quality;
  std::vector<uchar> buf;
  cv::imencode(".jpg", bgr, buf, param);
  auto viz_image = std::make_shared<adsfi_proto::viz::CompressedImage>();
  viz_image->mutable_header()->set_seq(yuv_image->header().seq());
  viz_image->mutable_header()->set_frameid(yuv_image->header().frame_id());
  auto raw_secs = yuv_image->header().sensor_stamp().camera_stamp();
  auto sec = static_cast<uint32_t>(raw_secs);
  auto nsec = static_cast<uint32_t>((raw_secs - sec) * 1e9);
  viz_image->mutable_header()->mutable_timestamp()->set_sec(sec);
  viz_image->mutable_header()->mutable_timestamp()->set_nsec(nsec);
  viz_image->set_format("jpeg");
  viz_image->mutable_data()->resize(buf.size());
  memcpy(viz_image->mutable_data()->data(), buf.data(), buf.size());
  return viz_image;
}

int32_t LocalMappingOnboard::OnImage(adf_lite_Bundle* input) {
  auto image_msg = input->GetOne("lm_image");
  HLOG_DEBUG << "image_msg->proto_msg " << image_msg.get();
  if (image_msg.get() == nullptr) {
    return -1;
  }
  auto msg = std::static_pointer_cast<hozon::soc::Image>(image_msg->proto_msg);
  auto viz_image = YUVNV12ImageToVizImage(msg, 50, 0.25);
  if (RVIZ_AGENT.Ok()) {
    RVIZ_AGENT.Publish("/localmap/image", viz_image);
  }
  return 0;
}

int32_t LocalMappingOnboard::PublishPostLaneLine(
    std::shared_ptr<const hozon::perception::measurement::MeasurementPb>
        measure_pbdata,
    std::shared_ptr<const perception_base::FusionFrame> fusion_frame) {
  auto transport_element =
      std::make_shared<common_onboard::NetaTransportElement>();
  if (!common_onboard::DataMapping::CvtMultiLanesToPb(
          fusion_frame->scene_->lane_lines->lanelines, transport_element)) {
    HLOG_ERROR << "multi laneline convert to proto struct failed.";
  }

  if (!common_onboard::DataMapping::CvtMultiRoadEdgesToPb(
          fusion_frame->scene_->road_edges->road_edges, transport_element)) {
    HLOG_ERROR << "multi laneline convert to proto struct failed.";
  }

  // 箭头、斑马线、停止线等其他路面元素透传
  *(transport_element->mutable_arrow()) =
      measure_pbdata->transport_element().arrow();
  *(transport_element->mutable_zebra_crossing()) =
      measure_pbdata->transport_element().zebra_crossing();
  *(transport_element->mutable_stopline()) =
      measure_pbdata->transport_element().stopline();
  *(transport_element->mutable_slow_downs()) =
      measure_pbdata->transport_element().slow_downs();
  *(transport_element->mutable_cross_points()) =
      measure_pbdata->transport_element().cross_points();
  *(transport_element->mutable_turn_waiting_zone()) =
      measure_pbdata->transport_element().turn_waiting_zone();
  *(transport_element->mutable_no_parking_zone()) =
      measure_pbdata->transport_element().no_parking_zone();
  transport_element->mutable_header()->set_data_stamp(
      measure_pbdata->header().data_stamp());
  transport_element->mutable_header()->set_seq(measure_pbdata->header().seq());

  auto EnvDataPtr = std::make_shared<hozon::netaos::adf_lite::BaseData>();
  EnvDataPtr->proto_msg = transport_element;

  adf_lite_Bundle lane_node_bundle;
  lane_node_bundle.Add("percep_transport", EnvDataPtr);
  SendOutput(&lane_node_bundle);

  return 0;
}

int32_t LocalMappingOnboard::PublishLocalMap() {
  HLOG_DEBUG << "start publish localmap...";
  auto* phm_fault = hozon::perception::lib::FaultManager::Instance();
  static double last_localmap_publish_time = -1.0;

  std::shared_ptr<hozon::mapping::LocalMap> result =
      std::make_shared<hozon::mapping::LocalMap>();
  if (lmap_->FetchLocalMap(result)) {
    double cur_localmap_publish_time = result->header().data_stamp();
    if (last_localmap_publish_time > 0) {
      if (cur_localmap_publish_time - last_localmap_publish_time <= 0) {
        phm_fault->Report(MAKE_FM_TUPLE(
            hozon::perception::base::FmModuleId::MAPPING,
            hozon::perception::base::FaultType::
                LOCALMAPPING_MUL_FRAM_OUTPUT_LOCAL_MAP_TIME_ERROR,
            hozon::perception::base::FaultStatus::OCCUR,
            hozon::perception::base::SensorOrientation::UNKNOWN, 3, 500));
        HLOG_ERROR << "publish localmap time error";
      } else {
        phm_fault->Report(MAKE_FM_TUPLE(
            hozon::perception::base::FmModuleId::MAPPING,
            hozon::perception::base::FaultType::
                LOCALMAPPING_MUL_FRAM_OUTPUT_LOCAL_MAP_TIME_ERROR,
            hozon::perception::base::FaultStatus::RESET,
            hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
      }
    }
    last_localmap_publish_time = cur_localmap_publish_time;
    auto workflow1 = std::make_shared<hozon::netaos::adf_lite::BaseData>();
    workflow1->proto_msg = result;
    adf_lite_Bundle bundle;
    bundle.Add("local_map", workflow1);
    SendOutput(&bundle);
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
            LOCALMAPPING_CANNOT_OUTPUT_LOCAL_MAP,
        hozon::perception::base::FaultStatus::RESET,
        hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
    HLOG_DEBUG << "publish localmap suceessed...";
  } else {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
            LOCALMAPPING_CANNOT_OUTPUT_LOCAL_MAP,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 1, 1000));
  }
  HLOG_DEBUG << "processed publish localmap";
  return 0;
}

REGISTER_ADF_CLASS(LocalMappingOnboard, LocalMappingOnboard);

}  // namespace lm
}  // namespace mp
}  // namespace hozon
