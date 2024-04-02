/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： global_lite.cc
 *   author     ： taoshaoyuan
 *   date       ： 2024.01
 ******************************************************************************/

#include "onboard/onboard_lite/global_lite/global_lite.h"
#include <perception-lib/lib/environment/environment.h>
#include <proto/soc/sensor_image.pb.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "modules/util/include/util/global_data.h"
#include "modules/util/include/util/rviz_agent/rviz_agent.h"

namespace hozon {
namespace mp {
namespace global_lite {

namespace fs = std::filesystem;

std::shared_ptr<adsfi_proto::viz::CompressedImage> YUVNV12ImageToVizImage(
    const std::shared_ptr<hozon::soc::Image>& yuv_image, int quality,
    double resize_factor);

int32_t GlobalLite::AlgInit() {
  const std::string kWorkRootEnv = "ADFLITE_ROOT_PATH";
  const std::string kGlobalDataYaml =
      "runtime_service/mapping/conf/mapping/global_data/global_data.yaml";
  const std::string kDefaultWorkRoot = "/app";

  std::string work_root =
      hozon::perception::lib::GetEnv(kWorkRootEnv, kDefaultWorkRoot);
  fs::path data_yaml_path = work_root;
  data_yaml_path /= kGlobalDataYaml;

  YAML::Node root;
  try {
    root = YAML::LoadFile(data_yaml_path.string());
  } catch (YAML::ParserException& ex) {
    HLOG_ERROR << "parse yaml " << data_yaml_path.string()
               << " failed: " << ex.what();
    return -1;
  } catch (YAML::BadFile& ex) {
    HLOG_ERROR << "load yaml " << data_yaml_path.string()
               << " failed: " << ex.what();
    return -1;
  }

  if (!GLOBAL_DATA.Init(root, work_root)) {
    HLOG_ERROR << "GlobalData init failed";
    return -1;
  }

  std::string viz_addr = GLOBAL_DATA.VizAddr();
  if (GLOBAL_DATA.EnableViz() && RVIZ_AGENT.Init(viz_addr) < 0) {
    HLOG_ERROR << "Init RvizAgent on " << viz_addr << " failed";
    return -1;
  }

  if (RVIZ_AGENT.Ok() && GLOBAL_DATA.VizImg()) {
    RVIZ_AGENT.Register<adsfi_proto::viz::CompressedImage>(kImgRosTopic);
    REGISTER_PROTO_MESSAGE_TYPE(kImgLiteTopic, hozon::soc::Image);
    RegistAlgProcessFunc(kImgTrigger, std::bind(&GlobalLite::OnImg, this,
                                                std::placeholders::_1));
  }

  return 0;
}

void GlobalLite::AlgRelease() {
  if (RVIZ_AGENT.Ok()) {
    RVIZ_AGENT.Term();
  }
}

int32_t GlobalLite::OnImg(hozon::netaos::adf_lite::Bundle* input) {
  auto image_msg = input->GetOne(kImgLiteTopic);
  if (image_msg.get() == nullptr) {
    return -1;
  }
  auto msg = std::static_pointer_cast<hozon::soc::Image>(image_msg->proto_msg);
  auto viz_image = YUVNV12ImageToVizImage(msg, 50, 0.25);
  if (RVIZ_AGENT.Ok()) {
    RVIZ_AGENT.Publish(kImgRosTopic, viz_image);
  }
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

}  // namespace global_lite
}  // namespace mp
}  // namespace hozon
