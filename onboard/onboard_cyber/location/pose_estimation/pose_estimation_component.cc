/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： pose_estimation_component.cc
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#include "onboard/onboard_cyber/location/pose_estimation/pose_estimation_component.h"

#include <gflags/gflags.h>
#include <yaml-cpp/yaml.h>

#include "modules/util/include/util/rviz_agent/rviz_agent.h"
DEFINE_string(config_yaml,
              "conf/mapping/location/pose_estimation/pose_estimation.yaml",
              "path to pose estimation config yaml");
DEFINE_string(config_cam_yaml, "conf/mapping/location/pose_estimation",
              "path to map matching camera config yaml");
DEFINE_string(viz_addr, "ipc:///tmp/rviz_agent_loc",
              "RvizAgent's working address, this should corresponds to the "
              "address used in RvizBridge. Leaving empty represents not using "
              "RvizAgent for visualization");
// DEFINE_string(mm_fault_output_topic, "/localization/modules_fault",
//               "mm fault output topic");

namespace hozon {
namespace mp {
namespace loc {

PoseEstimationComponent::~PoseEstimationComponent() {
  if (!FLAGS_viz_addr.empty()) {
    util::RvizAgent::Instance().Term();
  }
}

bool PoseEstimationComponent::Init() {
  if (!FLAGS_viz_addr.empty()) {
    int ret = hozon::mp::util::RvizAgent::Instance().Init(FLAGS_viz_addr);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent init failed";
    }
  }

  mm_ = std::make_shared<MapMatching>();
  mm_->Init(FLAGS_config_yaml, FLAGS_config_cam_yaml);

  YAML::Node node;
  try {
    node = YAML::LoadFile(FLAGS_config_yaml);
  } catch (const YAML::Exception &e) {
    HLOG_ERROR << "load config yaml " << FLAGS_config_yaml
               << " failed, throw: " << e.what();
    return false;
  }

  std::string key = "hdmap_topic";
  if (!node[key]) {
    HLOG_ERROR << key << " not found in config yaml";
    return false;
  }
  hdmap_topic_ = node[key].as<std::string>();

  key = "ins_topic";
  if (!node[key]) {
    HLOG_ERROR << key << " not found in config yaml";
    return false;
  }
  ins_topic_ = node[key].as<std::string>();

  key = "lane_line_topic";
  if (!node[key]) {
    HLOG_ERROR << key << " not found in config yaml";
    return false;
  }
  lane_line_topic_ = node[key].as<std::string>();

  key = "node_info_topic";
  if (!node[key]) {
    HLOG_ERROR << key << " not found in config yaml";
    return false;
  }
  node_info_topic_ = node[key].as<std::string>();

  // key = "location_topic";
  // if (!node[key]) {
  //   HLOG_ERROR << key << " not found in config yaml";
  //   return false;
  // }
  // location_topic_ = node[key].as<std::string>();

  hdmap_reader_ = node_->CreateReader<adsfi_proto::internal::SubMap>(
      hdmap_topic_,
      [this](const std::shared_ptr<adsfi_proto::internal::SubMap> &msg) {
        OnHdMap(msg);
      });

  ins_reader_ = node_->CreateReader<::adsfi_proto::internal::HafNodeInfo>(
      ins_topic_,
      [this](const std::shared_ptr<::adsfi_proto::internal::HafNodeInfo> &msg) {
        OnIns(msg);
      });

  road_marking_reader_ =
      node_->CreateReader<::adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray>(
          lane_line_topic_,
          [this](const std::shared_ptr<
                 ::adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray> &msg) {
            OnPerception(msg);
            // OnMarkPole(msg);
          });

  //   location_reader_ = node_->CreateReader<location::HafLocation>(
  //       location_topic_,
  //       [this](const std::shared_ptr<location::HafLocation> &msg) {
  //         OnLocation(msg);
  //       });

  mm_pub_thread_ = std::thread(&PoseEstimationComponent::pubMmMsg, this);

  node_info_writer_ = node_->CreateWriter<::adsfi_proto::internal::HafNodeInfo>(
      node_info_topic_);
  //   fault_writer_ = node_->CreateWriter<minieye::LocalizationFault>(
  //       FLAGS_mm_fault_output_topic);
  return true;
}

void PoseEstimationComponent::pubMmMsg() {
  while (apollo::cyber::OK()) {
    std::shared_ptr<::adsfi_proto::internal::HafNodeInfo> node_info =
        mm_->getMmNodeInfo();
    if (node_info != nullptr && node_info_writer_ != nullptr) {
      std::cout << "hit writer" << std::endl;
      node_info_writer_->Write(node_info);
    } else {
      usleep(100 * 1e3);
      continue;
    }
    usleep(10 * 1e3);
  }
}

// void PoseEstimationComponent::ConverMiniEyeSubMapToAdfSubMap(
//     const std::shared_ptr<minieye::SubMap> &minieye_msg,
//     std::shared_ptr<adsfi_proto::internal::SubMap> &adf_msg) {
//   if (adf_msg == nullptr) {
//     return;
//   }
//   adf_msg->mutable_header()->set_seq(minieye_msg->header().seq());
//   adf_msg->mutable_header()->set_frameid(minieye_msg->header().frame_id());
//   adf_msg->mutable_header()->mutable_timestamp()->set_sec(
//       minieye_msg->header().timestamp().sec());
//   adf_msg->mutable_header()->mutable_timestamp()->set_nsec(
//       minieye_msg->header().timestamp().nsec());
//   adf_msg->mutable_header()->mutable_gnssstamp()->set_sec(
//       minieye_msg->header().timestamp().sec());
//   adf_msg->mutable_header()->mutable_gnssstamp()->set_nsec(
//       minieye_msg->header().timestamp().nsec());
//   for (int i = 0; i < minieye_msg->lines_size(); ++i) {
//     auto line = adf_msg->add_lines();
//     line->set_global_id(minieye_msg->lines(i).global_id());
//     line->set_linetype(static_cast<adsfi_proto::internal::SubMap_LineType>(
//         minieye_msg->lines(i).linetype()));
//     for (int j = 0; j < minieye_msg->lines(i).points_size(); ++j) {
//       auto point = line->add_points();
//       point->mutable_wgs84_point()->set_longitude(
//           minieye_msg->lines(i).points(j).wgs84_point().longitude());
//       point->mutable_wgs84_point()->set_latitude(
//           minieye_msg->lines(i).points(j).wgs84_point().latitude());
//       point->mutable_wgs84_point()->set_altitude(
//           minieye_msg->lines(i).points(j).wgs84_point().altitude());
//     }
//   }
// }

void PoseEstimationComponent::OnHdMap(
    const std::shared_ptr<adsfi_proto::internal::SubMap> &msg) {
  if (!mm_) {
    return;
  }
  // std::shared_ptr<adsfi_proto::internal::SubMap> adf_submap_msg =
  //     std::make_shared<adsfi_proto::internal::SubMap>();
  // ConverMiniEyeSubMapToAdfSubMap(msg, adf_submap_msg);
  mm_->OnHdMap(msg);
}

// void PoseEstimationComponent::OnLocation(
//     const std::shared_ptr<location::HafLocation> &msg) {
//   if (!mm_) {
//     return;
//   }
//   mm_->OnLocation(msg);
// }

void PoseEstimationComponent::OnIns(
    const std::shared_ptr<const ::adsfi_proto::internal::HafNodeInfo> &msg) {
  if (!mm_) {
    return;
  }
  mm_->OnIns(msg);
  //   auto fault = std::make_shared<minieye::LocalizationFault>();
  //   if (mm_->GetFault(fault.get())) {
  //     fault_writer_->Write(fault);
  //   }
}

void PoseEstimationComponent::OnPerception(
    const std::shared_ptr<
        const ::adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray> &msg) {
  if (!mm_) {
    return;
  }
  mm_->OnPerception(msg);
}

// void PoseEstimationComponent::OnMarkPole(
//     const std::shared_ptr<::adsfi_proto::hz_Adsfi::AlgLaneDetectionOut> &msg)
//     {
//   if (!mm_) {
//     return;
//   }
//   mm_->OnMarkPole(msg);
// }

}  // namespace loc
}  // namespace mp
}  // namespace hozon
