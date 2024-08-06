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
#include "depend/perception-base/base/state_machine/state_machine_info.h"
#include "depend/perception-lib/lib/fault_manager/fault_manager.h"
#include "depend/proto/perception/perception_measurement.pb.h"
#include "modules/local_mapping/data_mapping/data_mapping.h"
#include "modules/local_mapping/lib/datalogger/freespace_manager.h"
#include "modules/local_mapping/lib/datalogger/map_manager.h"
// #include "modules/local_mapping/lib/interface/base_lane_process.h"
// #include "modules/local_mapping/lib/interface/base_roadedge_process.h"
#include "depend/proto/perception/perception_freespace.pb.h"
#include "onboard/onboard_lite/phm_comment_lite/proto/running_mode.pb.h"
#include "perception-base/base/state_machine/state_machine_info.h"
#include "perception-lib/lib/environment/environment.h"
#include "perception-lib/lib/fault_manager/fault_manager.h"
#include "perception-lib/lib/health_manager/health_manager.h"
#include "perception-lib/lib/state_manager/state_manager.h"
#include "proto/perception/transport_element.pb.h"
namespace hozon {
namespace mp {
namespace lm {

int32_t LocalMappingOnboard::AlgInit() {
  REGISTER_PROTO_MESSAGE_TYPE("percep_detection",
                              hozon::perception::measurement::MeasurementPb);
  REGISTER_PROTO_MESSAGE_TYPE("localization",
                              hozon::localization::Localization);
  REGISTER_PROTO_MESSAGE_TYPE("percep_freespace",
                              hozon::perception::FreeSpaceOutArray);
  REGISTER_PROTO_MESSAGE_TYPE("camera_0", hozon::soc::Image);
  REGISTER_PROTO_MESSAGE_TYPE("running_mode",
                              hozon::perception::common_onboard::running_mode);

  app_ptr_ = std::make_shared<LocalMapApp>();
  CHECK(app_ptr_->Init());
  HLOG_DEBUG << "Init LocalMap App successfully ...";

  RegistAlgProcessFunc("recv_perception",
                       std::bind(&LocalMappingOnboard::OnPerception, this,
                                 std::placeholders::_1));
  RegistAlgProcessFunc("recv_localization",
                       std::bind(&LocalMappingOnboard::Onlocalization, this,
                                 std::placeholders::_1));
  RegistAlgProcessFunc("recv_freespace",
                       std::bind(&LocalMappingOnboard::OnFreeSpace, this,
                                 std::placeholders::_1));
  RegistAlgProcessFunc("recv_running_mode",
                       std::bind(&LocalMappingOnboard::OnRunningMode, this,
                                 std::placeholders::_1));
  RegistAlgProcessFunc("recv_ins", std::bind(&LocalMappingOnboard::OnIns, this,
                                             std::placeholders::_1));
  RegistAlgProcessFunc("recv_image", std::bind(&LocalMappingOnboard::OnImage,
                                               this, std::placeholders::_1));

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
  static int last_runmode =
      static_cast<int>(hozon::perception::base::RunningMode::DRIVING);
  HLOG_DEBUG << " *** get run mode : *** " << runmode;
  if (runmode ==
      static_cast<int>(hozon::perception::base::RunningMode::PARKING)) {
    if (last_runmode != runmode) {
      PauseTrigger("recv_perception");
      PauseTrigger("recv_localization");
      if (RVIZ_AGENT.Ok()) {
        PauseTrigger("recv_ins");
        PauseTrigger("recv_image");
      }
      last_runmode = runmode;
      HLOG_INFO << "!!!!!!!!!!get run mode PARKING";
    }
    std::shared_ptr<hozon::perception::TransportElement> transport_element_pb =
        std::make_shared<hozon::perception::TransportElement>();
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
        tp = std::chrono::time_point_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now());
    transport_element_pb->mutable_header()->set_publish_stamp(
        static_cast<double>(tp.time_since_epoch().count()) * 1.0e-9);
    transport_element_pb->mutable_header()->set_data_stamp(
        static_cast<double>(tp.time_since_epoch().count()) * 1.0e-9);

    auto workflow1 = std::make_shared<hozon::netaos::adf_lite::BaseData>();
    workflow1->proto_msg = transport_element_pb;
    adf_lite_Bundle bundle;
    bundle.Add("percep_transport_lm", workflow1);
    SendOutput(&bundle);
    HLOG_DEBUG << "!!!!!!!!!!get run mode PARKING";
  } else if (runmode == static_cast<int>(
                            hozon::perception::base::RunningMode::DRIVING) ||
             runmode ==
                 static_cast<int>(hozon::perception::base::RunningMode::ALL)) {
    if (last_runmode != runmode) {
      ResumeTrigger("recv_perception");
      ResumeTrigger("recv_localization");
      if (RVIZ_AGENT.Ok()) {
        ResumeTrigger("recv_ins");
        ResumeTrigger("recv_image");
      }
      last_runmode = runmode;
      HLOG_INFO << "!!!!!!!!!!get run mode DRIVER & UNKNOWN";
    }
    // HLOG_ERROR << "!!!!!!!!!!get run mode DRIVER & UNKNOWN";
  }
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
    HLOG_ERROR << "nullptr detect laneline plugin";
    return -1;
  }

  phm_fault->Report(
      MAKE_FM_TUPLE(hozon::perception::base::FmModuleId::MAPPING,
                    hozon::perception::base::FaultType::
                        LOCALMAPPING_MUL_FRAM_PERCEPTION_INPUT_DATA_LOSS,
                    hozon::perception::base::FaultStatus::RESET,
                    hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));

  if (!app_ptr_) {
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
  HLOG_DEBUG << "start translate measurement pb to internal data ...";
  MeasurementFramePtr measurement_frame = std::make_shared<MeasurementFrame>();
  if (!data_mapping::DataMapping::CvtPb2Measurement(measure_pbdata,
                                                    measurement_frame)) {
    HLOG_ERROR << "DataMapping::CvtPb2Measurement failed";
    return -1;
  }
  HLOG_DEBUG << "finish translate measurement pb to internal data ...";

  // freespace数据映射到measurement_frame统一处理
  if (!FillFreespaceData(measurement_frame)) {
    HLOG_ERROR << "Fill freespace data into measurement frame filled";
  }

  // 建图模块功能实现
  HLOG_DEBUG << "start  do localmap work job...";
  if (app_ptr_->OnPerception(measurement_frame)) {
    HLOG_DEBUG << "finish do localmap work job...";

    HLOG_DEBUG << "start send localmap data to External pb ...";
    PublishLaneLine();
    PublishLocalMap();
    HLOG_DEBUG << "finish send localmap data to External pb ...";

    HLOG_INFO << "*** LocalMappingOnboard Run End ***";
    return 0;
  } else {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
            LOCALMAPPING_CANNOT_OUTPUT_LOCAL_MAP,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 3, 500));

    HLOG_ERROR << "localmapping can not output loaclmap";
    return -1;
  }
}

// freespace数据映射到measurement_frame统一处理
bool LocalMappingOnboard::FillFreespaceData(
    const std::shared_ptr<MeasurementFrame>& measure_frame) {
  measure_frame->occ_edges_ptr = std::make_shared<OccEdges>();
  auto& freespace_buffer = OCC_MANAGER->GetFreeSpaceBuffer();
  if (freespace_buffer.buffer_size() == 0) {
    HLOG_ERROR << "freespace_buffer is null";
    return false;
  }
  DrDataConstPtr perception_pose =
      POSE_MANAGER->GetDrPoseByTimeStamp(measure_frame->header.timestamp);
  if (perception_pose == nullptr) {
    HLOG_ERROR << "perception_pose is nullptr";
    return false;
  }
  const auto& freespace_pose = freespace_buffer.back()->freespaces_pose;
  const Eigen::Affine3d mapping_pose =
      perception_pose->pose.inverse() * freespace_pose->pose;
  for (const auto& edge : freespace_buffer.back()->edges.occ_edges) {
    auto occ_edge_ptr = std::make_shared<OccEdge>();
    for (const auto& pt : edge->vehicle_points) {
      occ_edge_ptr->vehicle_points.emplace_back(mapping_pose * pt);
    }
    occ_edge_ptr->detect_id = edge->detect_id;
    occ_edge_ptr->id = edge->detect_id;
    occ_edge_ptr->type = edge->type;
    measure_frame->occ_edges_ptr->occ_edges.emplace_back(occ_edge_ptr);
  }
  return true;
}

int32_t LocalMappingOnboard::Onlocalization(adf_lite_Bundle* input) {
  auto* phm_fault = hozon::perception::lib::FaultManager::Instance();
  static double last_localization_time = -1.0;
  static bool input_data_loss_error_flag = false;
  static bool input_data_value_error_flag = false;
  static bool input_data_time_error_flag = false;
  static int running_mode_count = 0;
  ++running_mode_count;
  if (running_mode_count >= 100) {
    running_mode_count = 0;
    HLOG_INFO << "receive localization data...";
  }

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
    return -1;
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

  std::shared_ptr<Location> latest_location = std::make_shared<Location>();
  data_mapping::DataMapping::CvtPbLocation2Location(msg, latest_location);

  app_ptr_->OnLocalization(latest_location);
  HLOG_DEBUG << "processed localization data";
  return 0;
}

int32_t LocalMappingOnboard::OnFreeSpace(adf_lite_Bundle* input) {
  auto percept_freespace = input->GetOne("percep_freespace");
  if (!percept_freespace) {
    HLOG_ERROR << "get null freespace";
    return -1;
  }
  auto pb_freespace =
      std::static_pointer_cast<hozon::perception::FreeSpaceOutArray>(
          percept_freespace->proto_msg);
  std::shared_ptr<FreeSpaces> latest_freespace = std::make_shared<FreeSpaces>();
  if (!data_mapping::DataMapping::CvtPbFreeSpaces2FreeSpaces(
          pb_freespace, latest_freespace)) {
    HLOG_ERROR << "DataMapping::CvtPbFreeSpaces2FreeSpaces failed";
    return -1;
  }
  app_ptr_->OnFreeSpace(latest_freespace);
  HLOG_DEBUG << "processed freespace data";
  return 0;
}

int32_t LocalMappingOnboard::OnIns(adf_lite_Bundle* input) {
  auto ins_msg = input->GetOne("ins");
  auto msg = std::static_pointer_cast<hozon::localization::HafNodeInfo>(
      ins_msg->proto_msg);
  if (!app_ptr_) {
    return -1;
  }

  std::shared_ptr<InsData> latest_ins = std::make_shared<InsData>();
  data_mapping::DataMapping::CvtPbIns2Ins(msg, latest_ins);
  app_ptr_->OnIns(latest_ins);
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
  if (RVIZ_AGENT.Ok()) {
    auto image_msg = input->GetOne("camera_0");
    HLOG_DEBUG << "image_msg->proto_msg " << image_msg.get();
    if (image_msg.get() == nullptr) {
      return -1;
    }
    auto msg =
        std::static_pointer_cast<hozon::soc::Image>(image_msg->proto_msg);
    auto viz_image = YUVNV12ImageToVizImage(msg, 50, 0.25);
    static bool register_flag = true;

    if (register_flag) {
      register_flag = false;
      RVIZ_AGENT.Register<adsfi_proto::viz::CompressedImage>("/camera_0");
    }
    RVIZ_AGENT.Publish("/camera_0", viz_image);
  }
  return 0;
}

int32_t LocalMappingOnboard::PublishLaneLine() {
  auto* phm_fault = hozon::perception::lib::FaultManager::Instance();
  static double last_localmap_publish_time = -1.0;
  std::shared_ptr<LocalMapFrame> te_localmap_frame = MAP_MANAGER->GetLocalMap();
  std::shared_ptr<hozon::perception::TransportElement> transport_frame_pb =
      std::make_shared<hozon::perception::TransportElement>();
  data_mapping::DataMapping::CvtLocalMap2TePb(te_localmap_frame,
                                              transport_frame_pb);

  auto workflow1 = std::make_shared<hozon::netaos::adf_lite::BaseData>();
  workflow1->proto_msg = transport_frame_pb;
  adf_lite_Bundle bundle;
  bundle.Add("percep_transport_lm", workflow1);
  SendOutput(&bundle);
  return 0;
}

int32_t LocalMappingOnboard::PublishLocalMap() {
  auto* phm_fault = hozon::perception::lib::FaultManager::Instance();
  static double last_localmap_publish_time = -1.0;
  std::shared_ptr<LocalMapFrame> localmap_frame = MAP_MANAGER->GetLocalMap();

  std::shared_ptr<hozon::mapping::LocalMap> localmap_pb =
      std::make_shared<hozon::mapping::LocalMap>();
  data_mapping::DataMapping::CvtLocalMap2Pb(localmap_frame, localmap_pb);

  double cur_localmap_publish_time = localmap_pb->header().data_stamp();
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
  workflow1->proto_msg = localmap_pb;
  adf_lite_Bundle bundle;
  bundle.Add("local_map", workflow1);
  SendOutput(&bundle);
  phm_fault->Report(MAKE_FM_TUPLE(
      hozon::perception::base::FmModuleId::MAPPING,
      hozon::perception::base::FaultType::LOCALMAPPING_CANNOT_OUTPUT_LOCAL_MAP,
      hozon::perception::base::FaultStatus::RESET,
      hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));

  return 0;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
