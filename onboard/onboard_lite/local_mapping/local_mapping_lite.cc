/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： prior_provider_lite.cc
 *   author     ： taoshaoyuan
 *   date       ： 2023.09
 ******************************************************************************/
#include "onboard/onboard_lite/local_mapping/local_mapping_lite.h"
// #include "perception-lib/lib/location_manager/location_manager.h"
#include <adf-lite/include/base.h>
#include <base/utils/log.h>
#include <common_onboard/adapter/adapter.h>
#include <common_onboard/adapter/onboard_lite/onboard_lite.h>
#include <gflags/gflags.h>
#include <proto/localization/node_info.pb.h>

#include <filesystem>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "perception-lib/lib/environment/environment.h"

namespace hozon {
namespace perception {
namespace common_onboard {

int32_t LocalMappingOnboard::AlgInit() {
  hozon::netaos::log::InitLogging("lm_executor", "lm_executor test",
                                  hozon::netaos::log::LogLevel::kInfo,
                                  HZ_LOG2CONSOLE, "./", 10, (20));

  hozon::netaos::adf::NodeLogger::GetInstance().CreateLogger(
      "lm_executor", "lm_executor test", hozon::netaos::log::LogLevel::kInfo);

  REGISTER_MESSAGE_TYPE("percep_transport",
                        hozon::perception::TransportElement);
  REGISTER_MESSAGE_TYPE("dr", hozon::dead_reckoning::DeadReckoning);
  REGISTER_MESSAGE_TYPE("lm_image", hozon::soc::Image);

  std::string default_work_root = "/app/";
  std::string work_root = lib::GetEnv("ADFLITE_ROOT_PATH", default_work_root);
  if (work_root.empty()) {
    HLOG_ERROR << "ENV: ADFLITE_ROOT_PATH is not set.";
    return 0;
  }
  std::string config_file = work_root +
                            "/runtime_service/mapping/conf/mapping/"
                            "local_mapping/local_mapping_conf.yaml";
  std::string mapping_path = work_root + "/runtime_service/mapping/";
  YAML::Node config = YAML::LoadFile(config_file);
  if (config["use_rviz"].as<bool>()) {
    HLOG_INFO << "Start RvizAgent!!!";
    int ret = hozon::mp::util::RvizAgent::Instance().Init(
        config["rviz_addr"].as<std::string>());
    if (ret < 0) {
      HLOG_ERROR << "RvizAgent start failed";
    }
  }

  if (RVIZ_AGENT.Ok()) {
    RVIZ_AGENT.Register<adsfi_proto::viz::CompressedImage>("/localmap/image");
  }

  lmap_ = std::make_shared<LMapApp>(mapping_path, config_file);
  // NOLINTBEGIN
  RegistAlgProcessFunc(
      "recv_laneline",
      std::bind(&LocalMappingOnboard::OnLaneLine, this, std::placeholders::_1));

  RegistAlgProcessFunc("recv_dr", std::bind(&LocalMappingOnboard::OnDr, this,
                                            std::placeholders::_1));

  RegistAlgProcessFunc("recv_ins", std::bind(&LocalMappingOnboard::OnIns, this,
                                             std::placeholders::_1));

  RegistAlgProcessFunc("recv_image", std::bind(&LocalMappingOnboard::OnImage,
                                               this, std::placeholders::_1));

  //  RegistAlgProcessFunc(
  //      "recv_roadedge",
  //      std::bind(&LocalMappingOnboard::OnRoadEdge, this,
  //      std::placeholders::_1));

  RegistAlgProcessFunc("send_lm",
                       std::bind(&LocalMappingOnboard::LocalMapPublish, this,
                                 std::placeholders::_1));
  // NOLINTEND
  return 0;
}

void LocalMappingOnboard::AlgRelease() {
  hozon::mp::util::RvizAgent::Instance().Term();
}

int32_t LocalMappingOnboard::OnLaneLine(Bundle* input) {
  HLOG_ERROR << "receive laneline data...";
  BaseDataTypePtr laneline_msg = input->GetOne("percep_transport");
  if (!laneline_msg) {
    HLOG_ERROR << "nullptr track lane plugin";
    return -1;
  }

  if (!lmap_) {
    HLOG_ERROR << "LMapApp init failed!!!";
    return 0;
  }

  auto msg = std::static_pointer_cast<hozon::perception::TransportElement>(
      laneline_msg->proto_msg);

  lmap_->OnLaneLine(msg);
  HLOG_ERROR << "processed laneline data";
  return 0;
}

int32_t LocalMappingOnboard::OnDr(Bundle* input) {
  HLOG_ERROR << "receive dr data...";
  BaseDataTypePtr dr_msg = input->GetOne("dr");
  if (!dr_msg) {
    HLOG_ERROR << "nullptr dr_msg plugin";
    return -1;
  }

  if (!lmap_) {
    return 0;
  }

  auto msg = std::static_pointer_cast<hozon::dead_reckoning::DeadReckoning>(
      dr_msg->proto_msg);

  if (msg->pose().pose_local().quaternion().w() == 0 &&
      msg->pose().pose_local().quaternion().x() == 0 &&
      msg->pose().pose_local().quaternion().y() == 0 &&
      msg->pose().pose_local().quaternion().z() == 0) {
    // HLOG_ERROR << "processed dr false";
    return false;
  }
  lmap_->OnDr(msg);
  HLOG_ERROR << "processed dr data";
  return 0;
}

int32_t LocalMappingOnboard::OnIns(Bundle* input) {
  BaseDataTypePtr ins_msg = input->GetOne("ins");
  auto msg = std::static_pointer_cast<hozon::localization::HafNodeInfo>(
      ins_msg->proto_msg);
  if (!lmap_) {
    return 0;
  }

  lmap_->OnIns(msg);
  return 1;
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

int32_t LocalMappingOnboard::OnImage(Bundle* input) {
  BaseDataTypePtr image_msg = input->GetOne("lm_image");
  // HLOG_ERROR << "image_msg->proto_msg " << image_msg.get();
  if (image_msg.get() == nullptr) {
    return 0;
  }
  auto msg = std::static_pointer_cast<hozon::soc::Image>(image_msg->proto_msg);
  auto viz_image = YUVNV12ImageToVizImage(msg, 50, 0.25);
  if (RVIZ_AGENT.Ok()) {
    RVIZ_AGENT.Publish("/localmap/image", viz_image);
  }
  return 1;
}

// int32_t LocalMappingOnboard::OnRoadEdge(Bundle* input) {
//   BaseDataTypePtr road_edge_msg = input->GetOne("percep_transport");
//   auto msg = std::static_pointer_cast<common_onboard::NetaTransportElement>(
//       road_edge_msg->proto_msg);
//   if (!lmap_) {
//     return false;
//   }
//   lmap_->OnRoadEdge(msg);
// }

int32_t LocalMappingOnboard::LocalMapPublish(Bundle* /*output*/) {
  HLOG_ERROR << "start publish localmap...";
  std::shared_ptr<hozon::mapping::LocalMap> result =
      std::make_shared<hozon::mapping::LocalMap>();
  if (lmap_->FetchLocalMap(result)) {
    BaseDataTypePtr workflow1 =
        std::make_shared<hozon::netaos::adf_lite::BaseData>();

    workflow1->proto_msg = result;
    Bundle bundle;
    bundle.Add("local_map", workflow1);
    SendOutput(&bundle);
    HLOG_DEBUG << "publish localmap suceessed...";
  }
  HLOG_DEBUG << "processed publish localmap";
  usleep(100 * 1e3);
  return 0;
}

REGISTER_EXECUTOR_CLASS(LocalMappingOnboard, LocalMappingOnboard);

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
