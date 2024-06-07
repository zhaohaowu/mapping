/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： viz_converter_lite.cc
 *   author     ： taoshaoyuan
 *   date       ： 2024.01
 ******************************************************************************/

#include "onboard/onboard_lite/viz_converter/viz_converter_lite.h"
#include <perception-lib/lib/environment/environment.h>
#include <proto/local_mapping/local_map.pb.h>
#include <proto/localization/localization.pb.h>
#include <proto/map/navigation.pb.h>
#include <proto/perception/perception_measurement.pb.h>
#include <proto/planning/planning.pb.h>
#include <proto/soc/sensor_image.pb.h>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <vector>

#include "base/utils/log.h"
#include "depend/common/utm_projection/coordinate_convertor.h"
#include "modules/util/include/util/geo.h"
#include "modules/util/include/util/rviz_agent/rviz_agent.h"
#include "modules/util/include/util/viz_helper.h"

namespace hozon {
namespace perception {
namespace common_onboard {

using namespace hozon::mp::util;  // NOLINT

int32_t VizConverterLite::AlgInit() {
  std::string default_work_root = "/app/";
  std::string work_root = lib::GetEnv("ADFLITE_ROOT_PATH", default_work_root);
  if (work_root == "") {
    HLOG_ERROR << "ENV: ADFLITE_ROOT_PATH is not set.";
    return -1;
  }
  std::string config_file =
      work_root +
      "/runtime_service/mapping/conf/lite/viz_cvt/viz_cvt_config.yaml";

  YAML::Node config = YAML::LoadFile(config_file);

  if (!config["viz_addr"].IsDefined()) {
    HLOG_ERROR << "viz_addr not found in " << config_file;
    return -1;
  }

  const auto viz_addr = config["viz_addr"].as<std::string>();

  int ret = RVIZ_AGENT.Init(viz_addr);
  if (ret < 0) {
    HLOG_ERROR << "RvizAgent init failed on " << viz_addr;
    return -1;
  }

  ret = RVIZ_AGENT.Register<hozon::localization::Localization>(
      kVizTopicLocalization);
  if (ret < 0) {
    HLOG_ERROR << "RvizAgent register " << kVizTopicLocalization << " failed";
    return -1;
  }

  ret = RVIZ_AGENT.Register<hozon::soc::ImuIns>(kVizTopicImuIns);
  if (ret < 0) {
    HLOG_ERROR << "RvizAgent register " << kVizTopicImuIns << " failed";
    return -1;
  }

  ret =
      RVIZ_AGENT.Register<hozon::localization::HafNodeInfo>(kVizTopicInsPlugin);
  if (ret < 0) {
    HLOG_ERROR << "RvizAgent register " << kVizTopicInsPlugin << " failed";
    return -1;
  }

  ret = RVIZ_AGENT.Register<hozon::dead_reckoning::DeadReckoning>(
      kVizTopicDeadReckoning);
  if (ret < 0) {
    HLOG_ERROR << "RvizAgent register " << kVizTopicDeadReckoning << " failed";
    return -1;
  }

  ret = RVIZ_AGENT.Register<hozon::soc::Chassis>(kVizTopicChassis);
  if (ret < 0) {
    HLOG_ERROR << "RvizAgent register " << kVizTopicChassis << " failed";
    return -1;
  }

  std::vector<std::string> odom_topics = {
      kVizTopicLocalizationLocalPose,
      kVizTopicLocalizationLocalEnuPose,
  };

  ret = RVIZ_AGENT.Register<adsfi_proto::viz::Odometry>(odom_topics);
  if (ret < 0) {
    HLOG_ERROR << "RvizAgent register odometry failed";
    return -1;
  }

  std::vector<std::string> marker_topics = {
      kVizTopicLocalizationStatus,
      kVizTopicInsPluginStatus,
      kVizTopicDrivingStatus,
      kVizTopicPlanningPath,
  };
  ret = RVIZ_AGENT.Register<adsfi_proto::viz::Marker>(marker_topics);
  if (ret < 0) {
    HLOG_ERROR << "RvizAgent register marker failed";
    return -1;
  }

  std::vector<std::string> marker_array_topics = {
      kVizTopicMapMsgMap,
      kVizTopicMapMsgRouting,
      kVizTopicMapMsgStatus,
      kVizTopicPercepDetectionTransportElement,
      kVizTopicPercepTransportTransportElement,
      kVizTopicLocalMap,
  };
  ret = RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(marker_array_topics);
  if (ret < 0) {
    HLOG_ERROR << "RvizAgent register marker array failed";
    return -1;
  }

  ret =
      RVIZ_AGENT.Register<adsfi_proto::viz::CompressedImage>(kVizTopicCamera0);
  if (ret < 0) {
    HLOG_ERROR << "RvizAgent register " << kVizTopicCamera0 << " failed";
    return -1;
  }

  std::vector<std::string> tf_topics = {
      kVizTopicTfVehicleToLocal,
      kVizTopicTfLocalEnuToLocal,
  };
  ret = RVIZ_AGENT.Register<adsfi_proto::viz::TransformStamped>(tf_topics);
  if (ret < 0) {
    HLOG_ERROR << "RvizAgent register tf failed";
    return -1;
  }

  REGISTER_PROTO_MESSAGE_TYPE(kTopicLocalization,
                              hozon::localization::Localization);
  REGISTER_PROTO_MESSAGE_TYPE(kTopicPoseEstimation,
                              hozon::localization::HafNodeInfo);
  REGISTER_PROTO_MESSAGE_TYPE(kTopicImuIns, hozon::soc::ImuIns);
  REGISTER_PROTO_MESSAGE_TYPE(kTopicInsPlugin,
                              hozon::localization::HafNodeInfo);
  REGISTER_PROTO_MESSAGE_TYPE(kTopicDeadReckoning,
                              hozon::dead_reckoning::DeadReckoning);
  REGISTER_PROTO_MESSAGE_TYPE(kTopicChassis, hozon::soc::Chassis);
  REGISTER_PROTO_MESSAGE_TYPE(kTopicMapMsg, hozon::navigation_hdmap::MapMsg);
  REGISTER_PROTO_MESSAGE_TYPE(kTopicCamera0, hozon::soc::Image);
  REGISTER_PROTO_MESSAGE_TYPE(kTopicPercepDetection,
                              hozon::perception::measurement::MeasurementPb);
  REGISTER_PROTO_MESSAGE_TYPE(kTopicPercepTransport,
                              hozon::perception::TransportElement);
  REGISTER_PROTO_MESSAGE_TYPE(kTopicLocalMap, hozon::mapping::LocalMap);
  REGISTER_PROTO_MESSAGE_TYPE(kTopicDrivingStatus,
                              hozon::planning::ADCTrajectory);

  RegistAlgProcessFunc("recv_localization",
                       std::bind(&VizConverterLite::OnLocalization, this,
                                 std::placeholders::_1));
  RegistAlgProcessFunc("recv_pe", std::bind(&VizConverterLite::OnPoseEstimation,
                                            this, std::placeholders::_1));
  RegistAlgProcessFunc("recv_imu_ins", std::bind(&VizConverterLite::OnImuIns,
                                                 this, std::placeholders::_1));
  RegistAlgProcessFunc("recv_inspva", std::bind(&VizConverterLite::OnInsPlugin,
                                                this, std::placeholders::_1));
  RegistAlgProcessFunc("recv_dr", std::bind(&VizConverterLite::OnDeadReckoning,
                                            this, std::placeholders::_1));
  RegistAlgProcessFunc("recv_chassis", std::bind(&VizConverterLite::OnChassis,
                                                 this, std::placeholders::_1));
  RegistAlgProcessFunc(
      "recv_map_fusion",
      std::bind(&VizConverterLite::OnMapMsg, this, std::placeholders::_1));
  RegistAlgProcessFunc("recv_camera_0", std::bind(&VizConverterLite::OnCamera0,
                                                  this, std::placeholders::_1));
  RegistAlgProcessFunc("recv_percep_detection",
                       std::bind(&VizConverterLite::OnPercepDetection, this,
                                 std::placeholders::_1));
  RegistAlgProcessFunc("recv_percep_transport",
                       std::bind(&VizConverterLite::OnPercepTransport, this,
                                 std::placeholders::_1));
  RegistAlgProcessFunc(
      "recv_local_map",
      std::bind(&VizConverterLite::OnLocalMap, this, std::placeholders::_1));
  RegistAlgProcessFunc("recv_driving_status",
                       std::bind(&VizConverterLite::OnDrivingStatus, this,
                                 std::placeholders::_1));
  return 0;
}

void VizConverterLite::AlgRelease() { RVIZ_AGENT.Term(); }

int32_t VizConverterLite::OnLocalization(
    hozon::netaos::adf_lite::Bundle* input) {
  if (input == nullptr) {
    HLOG_ERROR << "nullptr input Bundle";
    return -1;
  }

  auto input_msg = input->GetOne(kTopicLocalization);
  if (input_msg == nullptr) {
    HLOG_ERROR << "GetOne " << kTopicLocalization << " nullptr";
    return -1;
  }

  const auto proto_msg =
      std::static_pointer_cast<hozon::localization::Localization>(
          input_msg->proto_msg);
  if (proto_msg == nullptr) {
    HLOG_ERROR << "proto_msg is nullptr in Bundle input";
    return -1;
  }

  auto loc = std::make_shared<hozon::localization::Localization>();
  loc->CopyFrom(*proto_msg);

  uint32_t sec = 0;
  uint32_t nsec = 0;
  SplitStamp(loc->header().data_stamp(), &sec, &nsec);

  const auto& pose = loc->pose();
  const auto& pose_local = loc->pose_local();

  Eigen::Vector3d pos_veh_in_local(pose_local.position().x(),
                                   pose_local.position().y(),
                                   pose_local.position().z());
  Eigen::Quaterniond quat_veh_in_local(
      pose_local.quaternion().w(), pose_local.quaternion().x(),
      pose_local.quaternion().y(), pose_local.quaternion().z());

  adsfi_proto::viz::TransformStamped tf_veh_to_local;
  tf_veh_to_local.mutable_header()->mutable_timestamp()->set_sec(sec);
  tf_veh_to_local.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  tf_veh_to_local.mutable_header()->set_frameid(mp::util::kFrameLocal);
  tf_veh_to_local.set_child_frame_id(mp::util::kFrameVehicle);
  tf_veh_to_local.mutable_transform()->mutable_translation()->set_x(
      pos_veh_in_local.x());
  tf_veh_to_local.mutable_transform()->mutable_translation()->set_y(
      pos_veh_in_local.y());
  tf_veh_to_local.mutable_transform()->mutable_translation()->set_z(
      pos_veh_in_local.z());
  tf_veh_to_local.mutable_transform()->mutable_rotation()->set_w(
      quat_veh_in_local.w());
  tf_veh_to_local.mutable_transform()->mutable_rotation()->set_x(
      quat_veh_in_local.x());
  tf_veh_to_local.mutable_transform()->mutable_rotation()->set_y(
      quat_veh_in_local.y());
  tf_veh_to_local.mutable_transform()->mutable_rotation()->set_z(
      quat_veh_in_local.z());
  RVIZ_AGENT.Publish(kVizTopicTfVehicleToLocal, tf_veh_to_local);

  adsfi_proto::viz::Odometry odom_veh_in_local;
  odom_veh_in_local.mutable_header()->CopyFrom(tf_veh_to_local.header());
  odom_veh_in_local.set_child_frame_id(mp::util::kFrameVehicle);
  odom_veh_in_local.mutable_pose()->mutable_pose()->mutable_position()->set_x(
      pose_local.position().x());
  odom_veh_in_local.mutable_pose()->mutable_pose()->mutable_position()->set_y(
      pose_local.position().y());
  odom_veh_in_local.mutable_pose()->mutable_pose()->mutable_position()->set_z(
      pose_local.position().z());
  odom_veh_in_local.mutable_pose()
      ->mutable_pose()
      ->mutable_orientation()
      ->CopyFrom(tf_veh_to_local.transform().rotation());
  RVIZ_AGENT.Publish(kVizTopicLocalizationLocalPose, odom_veh_in_local);

  Eigen::Vector3d gcj(pose.gcj02().x(), pose.gcj02().y(), pose.gcj02().z());

  if (!enu_station_inited_ &&
      !hozon::common::coordinate_convertor::OutOfChina(gcj.x(), gcj.y())) {
    enu_station_ = gcj;
    enu_station_inited_ = true;

    Eigen::Vector3d pos_veh_in_enu =
        mp::util::Geo::Gcj02ToEnu(gcj, enu_station_);
    Eigen::Quaterniond quat_veh_in_enu(
        pose.quaternion().w(), pose.quaternion().x(), pose.quaternion().y(),
        pose.quaternion().z());
    Eigen::Quaterniond quat_enu_in_veh = quat_veh_in_enu.inverse();

    quat_enu_in_local_ = quat_veh_in_local * quat_enu_in_veh;
    pos_enu_in_local_ =
        quat_enu_in_local_ * pos_veh_in_enu * (-1) + pos_veh_in_local;

    HLOG_INFO << "enu station init over: " << SET_PRECISION(15)
              << enu_station_.x() << ", " << enu_station_.y() << ", "
              << enu_station_.z();
  }

  if (enu_station_inited_) {
    adsfi_proto::viz::TransformStamped tf_enu_to_local;
    tf_enu_to_local.mutable_header()->CopyFrom(tf_veh_to_local.header());
    tf_enu_to_local.mutable_header()->set_frameid(mp::util::kFrameLocal);
    tf_enu_to_local.set_child_frame_id(mp::util::kFrameLocalEnu);
    tf_enu_to_local.mutable_transform()->mutable_translation()->set_x(
        pos_enu_in_local_.x());
    tf_enu_to_local.mutable_transform()->mutable_translation()->set_y(
        pos_enu_in_local_.y());
    tf_enu_to_local.mutable_transform()->mutable_translation()->set_z(
        pos_enu_in_local_.z());
    tf_enu_to_local.mutable_transform()->mutable_rotation()->set_w(
        quat_enu_in_local_.w());
    tf_enu_to_local.mutable_transform()->mutable_rotation()->set_x(
        quat_enu_in_local_.x());
    tf_enu_to_local.mutable_transform()->mutable_rotation()->set_y(
        quat_enu_in_local_.y());
    tf_enu_to_local.mutable_transform()->mutable_rotation()->set_z(
        quat_enu_in_local_.z());
    RVIZ_AGENT.Publish(kVizTopicTfLocalEnuToLocal, tf_enu_to_local);

    Eigen::Vector3d pos_veh_in_enu =
        mp::util::Geo::Gcj02ToEnu(gcj, enu_station_);
    Eigen::Quaterniond quat_veh_in_enu(
        pose.quaternion().w(), pose.quaternion().x(), pose.quaternion().y(),
        pose.quaternion().z());
    adsfi_proto::viz::Odometry odom_veh_in_local_enu;
    odom_veh_in_local_enu.mutable_header()->mutable_timestamp()->CopyFrom(
        tf_veh_to_local.header().timestamp());
    odom_veh_in_local_enu.mutable_header()->set_frameid(
        mp::util::kFrameLocalEnu);
    odom_veh_in_local_enu.set_child_frame_id(mp::util::kFrameVehicle);
    odom_veh_in_local_enu.mutable_pose()
        ->mutable_pose()
        ->mutable_position()
        ->set_x(pos_veh_in_enu.x());
    odom_veh_in_local_enu.mutable_pose()
        ->mutable_pose()
        ->mutable_position()
        ->set_y(pos_veh_in_enu.y());
    odom_veh_in_local_enu.mutable_pose()
        ->mutable_pose()
        ->mutable_position()
        ->set_z(pos_veh_in_enu.z());
    odom_veh_in_local_enu.mutable_pose()
        ->mutable_pose()
        ->mutable_orientation()
        ->set_w(quat_veh_in_enu.w());
    odom_veh_in_local_enu.mutable_pose()
        ->mutable_pose()
        ->mutable_orientation()
        ->set_x(quat_veh_in_enu.x());
    odom_veh_in_local_enu.mutable_pose()
        ->mutable_pose()
        ->mutable_orientation()
        ->set_y(quat_veh_in_enu.y());
    odom_veh_in_local_enu.mutable_pose()
        ->mutable_pose()
        ->mutable_orientation()
        ->set_z(quat_veh_in_enu.z());
    RVIZ_AGENT.Publish(kVizTopicLocalizationLocalEnuPose,
                       odom_veh_in_local_enu);
  } else {
    HLOG_ERROR << "enu station not inited, not pub TF of local_enu to local, "
                  "and odom of vehicle in local_enu, curr gcj: ["
               << SET_PRECISION(15) << gcj.x() << ", " << gcj.y() << ", "
               << gcj.z() << "]";
  }

  adsfi_proto::viz::Marker marker_status;
  marker_status.mutable_header()->mutable_timestamp()->CopyFrom(
      tf_veh_to_local.header().timestamp());
  marker_status.mutable_header()->set_frameid(mp::util::kFrameVehicle);
  marker_status.set_ns("localization/status");
  marker_status.set_id(0);
  marker_status.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
  marker_status.set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  const Eigen::Vector3d status_pos(0, 10, 10);
  marker_status.mutable_pose()->mutable_position()->set_x(status_pos.x());
  marker_status.mutable_pose()->mutable_position()->set_y(status_pos.y());
  marker_status.mutable_pose()->mutable_position()->set_z(status_pos.z());
  marker_status.mutable_pose()->mutable_orientation()->set_x(0.);
  marker_status.mutable_pose()->mutable_orientation()->set_y(0.);
  marker_status.mutable_pose()->mutable_orientation()->set_z(0.);
  marker_status.mutable_pose()->mutable_orientation()->set_w(1.);
  const double text_size = 1;
  marker_status.mutable_scale()->set_z(text_size);
  const double lifetime = 0;
  uint32_t life_sec = 0;
  uint32_t life_nsec = 0;
  SplitStamp(lifetime, &life_sec, &life_nsec);
  marker_status.mutable_lifetime()->set_sec(life_sec);
  marker_status.mutable_lifetime()->set_nsec(life_nsec);

  if (loc->rtk_status() == 4 && loc->location_state() == 2) {
    marker_status.mutable_color()->set_a(1.0);
    marker_status.mutable_color()->set_r(0);
    marker_status.mutable_color()->set_g(1);
    marker_status.mutable_color()->set_b(0);
  } else {
    marker_status.mutable_color()->set_a(1.0);
    marker_status.mutable_color()->set_r(1);
    marker_status.mutable_color()->set_g(0);
    marker_status.mutable_color()->set_b(0);
  }
  auto* text = marker_status.mutable_text();
  *text = "rtk_status: " + std::to_string(loc->rtk_status()) +
          "\nlocation_state: " + std::to_string(loc->location_state());
  RVIZ_AGENT.Publish(kVizTopicLocalizationStatus, marker_status);

  RVIZ_AGENT.Publish(kVizTopicLocalization, loc);

  return 0;
}

int32_t VizConverterLite::OnPoseEstimation(
    hozon::netaos::adf_lite::Bundle* input) {
  if (input == nullptr) {
    HLOG_ERROR << "nullptr input Bundle";
    return -1;
  }

  auto input_msg = input->GetOne(kTopicPoseEstimation);
  if (input_msg == nullptr) {
    HLOG_ERROR << "GetOne " << kTopicPoseEstimation << " nullptr";
    return -1;
  }

  const auto proto_msg =
      std::static_pointer_cast<hozon::localization::HafNodeInfo>(
          input_msg->proto_msg);
  if (proto_msg == nullptr) {
    HLOG_ERROR << "proto_msg is nullptr in Bundle input";
    return -1;
  }

  auto loc = std::make_shared<hozon::localization::HafNodeInfo>();
  loc->CopyFrom(*proto_msg);

  static bool register_flag = true;
  if (register_flag) {
    register_flag = false;
    RVIZ_AGENT.Register<adsfi_proto::viz::Odometry>(kTopicPoseEstimation +
                                                    "/local_enu_pose");
  }
  uint32_t sec = 0;
  uint32_t nsec = 0;
  SplitStamp(loc->header().data_stamp(), &sec, &nsec);
  Eigen::Vector3d gcj(loc->pos_gcj02().x(), loc->pos_gcj02().y(),
                      loc->pos_gcj02().z());
  Eigen::Vector3d pos_veh_in_enu = mp::util::Geo::Gcj02ToEnu(gcj, enu_station_);
  Eigen::Quaterniond quat_veh_in_enu(
      loc->quaternion().w(), loc->quaternion().x(), loc->quaternion().y(),
      loc->quaternion().z());
  adsfi_proto::viz::Odometry odom_veh_in_local_enu;
  odom_veh_in_local_enu.mutable_header()->mutable_timestamp()->set_sec(sec);
  odom_veh_in_local_enu.mutable_header()->mutable_timestamp()->set_nsec(nsec);

  odom_veh_in_local_enu.mutable_header()->set_frameid(mp::util::kFrameLocalEnu);
  odom_veh_in_local_enu.set_child_frame_id(mp::util::kFrameVehicle);
  odom_veh_in_local_enu.mutable_pose()
      ->mutable_pose()
      ->mutable_position()
      ->set_x(pos_veh_in_enu.x());
  odom_veh_in_local_enu.mutable_pose()
      ->mutable_pose()
      ->mutable_position()
      ->set_y(pos_veh_in_enu.y());
  odom_veh_in_local_enu.mutable_pose()
      ->mutable_pose()
      ->mutable_position()
      ->set_z(pos_veh_in_enu.z());
  odom_veh_in_local_enu.mutable_pose()
      ->mutable_pose()
      ->mutable_orientation()
      ->set_w(quat_veh_in_enu.w());
  odom_veh_in_local_enu.mutable_pose()
      ->mutable_pose()
      ->mutable_orientation()
      ->set_x(quat_veh_in_enu.x());
  odom_veh_in_local_enu.mutable_pose()
      ->mutable_pose()
      ->mutable_orientation()
      ->set_y(quat_veh_in_enu.y());
  odom_veh_in_local_enu.mutable_pose()
      ->mutable_pose()
      ->mutable_orientation()
      ->set_z(quat_veh_in_enu.z());
  RVIZ_AGENT.Publish(kTopicPoseEstimation + "/local_enu_pose",
                     odom_veh_in_local_enu);
  return 0;
}

int32_t VizConverterLite::OnCamera0(hozon::netaos::adf_lite::Bundle* input) {
  auto image_msg = input->GetOne(kTopicCamera0);
  if (image_msg == nullptr) {
    return -1;
  }
  auto msg = std::static_pointer_cast<hozon::soc::Image>(image_msg->proto_msg);
  auto viz_image = YUVNV12ImageToVizImage(msg, 50, 0.25);
  RVIZ_AGENT.Publish(kVizTopicCamera0, viz_image);
  return 0;
}

int32_t VizConverterLite::OnImuIns(hozon::netaos::adf_lite::Bundle* input) {
  if (input == nullptr) {
    HLOG_ERROR << "nullptr input Bundle";
    return -1;
  }

  auto input_msg = input->GetOne(kTopicImuIns);
  if (input_msg == nullptr) {
    HLOG_ERROR << "GetOne " << kTopicImuIns << " nullptr";
    return -1;
  }

  const auto proto_msg =
      std::static_pointer_cast<hozon::soc::ImuIns>(input_msg->proto_msg);
  if (proto_msg == nullptr) {
    HLOG_ERROR << "proto_msg is nullptr in Bundle input";
    return -1;
  }

  auto msg = std::make_shared<hozon::soc::ImuIns>();
  msg->CopyFrom(*proto_msg);

  RVIZ_AGENT.Publish(kVizTopicImuIns, msg);

  return 0;
}

int32_t VizConverterLite::OnInsPlugin(hozon::netaos::adf_lite::Bundle* input) {
  if (input == nullptr) {
    HLOG_ERROR << "nullptr input Bundle";
    return -1;
  }

  auto input_msg = input->GetOne(kTopicInsPlugin);
  if (input_msg == nullptr) {
    HLOG_ERROR << "GetOne " << kTopicInsPlugin << " nullptr";
    return -1;
  }

  const auto proto_msg =
      std::static_pointer_cast<hozon::localization::HafNodeInfo>(
          input_msg->proto_msg);
  if (proto_msg == nullptr) {
    HLOG_ERROR << "proto_msg is nullptr in Bundle input";
    return -1;
  }

  auto msg = std::make_shared<hozon::localization::HafNodeInfo>();
  msg->CopyFrom(*proto_msg);

  RVIZ_AGENT.Publish(kVizTopicInsPlugin, msg);

  uint32_t sec = 0;
  uint32_t nsec = 0;
  SplitStamp(msg->header().data_stamp(), &sec, &nsec);

  adsfi_proto::viz::Marker marker_status;
  marker_status.mutable_header()->mutable_timestamp()->set_sec(sec);
  marker_status.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  marker_status.mutable_header()->set_frameid(mp::util::kFrameVehicle);
  marker_status.set_ns("inspva/status");
  marker_status.set_id(0);
  marker_status.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
  marker_status.set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  const Eigen::Vector3d status_pos(-5, 10, 10);
  marker_status.mutable_pose()->mutable_position()->set_x(status_pos.x());
  marker_status.mutable_pose()->mutable_position()->set_y(status_pos.y());
  marker_status.mutable_pose()->mutable_position()->set_z(status_pos.z());
  marker_status.mutable_pose()->mutable_orientation()->set_x(0.);
  marker_status.mutable_pose()->mutable_orientation()->set_y(0.);
  marker_status.mutable_pose()->mutable_orientation()->set_z(0.);
  marker_status.mutable_pose()->mutable_orientation()->set_w(1.);
  const double text_size = 1;
  marker_status.mutable_scale()->set_z(text_size);
  const double lifetime = 0;
  uint32_t life_sec = 0;
  uint32_t life_nsec = 0;
  SplitStamp(lifetime, &life_sec, &life_nsec);
  marker_status.mutable_lifetime()->set_sec(life_sec);
  marker_status.mutable_lifetime()->set_nsec(life_nsec);

  if (msg->gps_status() == 4 && msg->sys_status() == 2) {
    marker_status.mutable_color()->set_a(1.0);
    marker_status.mutable_color()->set_r(0);
    marker_status.mutable_color()->set_g(1);
    marker_status.mutable_color()->set_b(0);
  } else {
    marker_status.mutable_color()->set_a(1.0);
    marker_status.mutable_color()->set_r(1);
    marker_status.mutable_color()->set_g(0);
    marker_status.mutable_color()->set_b(0);
  }
  auto* text = marker_status.mutable_text();
  *text = "gps_status: " + std::to_string(msg->gps_status()) +
          "\nsys_status: " + std::to_string(msg->sys_status());
  RVIZ_AGENT.Publish(kVizTopicInsPluginStatus, marker_status);

  static bool register_flag = true;
  if (register_flag) {
    register_flag = false;
    RVIZ_AGENT.Register<adsfi_proto::viz::Odometry>(kVizTopicInsPlugin +
                                                    "/local_enu_pose");
  }

  Eigen::Vector3d gcj(msg->pos_gcj02().x(), msg->pos_gcj02().y(),
                      msg->pos_gcj02().z());
  Eigen::Vector3d pos_veh_in_enu = mp::util::Geo::Gcj02ToEnu(gcj, enu_station_);
  Eigen::Quaterniond quat_veh_in_enu(
      msg->quaternion().w(), msg->quaternion().x(), msg->quaternion().y(),
      msg->quaternion().z());
  adsfi_proto::viz::Odometry odom_veh_in_local_enu;
  odom_veh_in_local_enu.mutable_header()->mutable_timestamp()->set_sec(sec);
  odom_veh_in_local_enu.mutable_header()->mutable_timestamp()->set_nsec(nsec);

  odom_veh_in_local_enu.mutable_header()->set_frameid(mp::util::kFrameLocalEnu);
  odom_veh_in_local_enu.set_child_frame_id(mp::util::kFrameVehicle);
  odom_veh_in_local_enu.mutable_pose()
      ->mutable_pose()
      ->mutable_position()
      ->set_x(pos_veh_in_enu.x());
  odom_veh_in_local_enu.mutable_pose()
      ->mutable_pose()
      ->mutable_position()
      ->set_y(pos_veh_in_enu.y());
  odom_veh_in_local_enu.mutable_pose()
      ->mutable_pose()
      ->mutable_position()
      ->set_z(pos_veh_in_enu.z());
  odom_veh_in_local_enu.mutable_pose()
      ->mutable_pose()
      ->mutable_orientation()
      ->set_w(quat_veh_in_enu.w());
  odom_veh_in_local_enu.mutable_pose()
      ->mutable_pose()
      ->mutable_orientation()
      ->set_x(quat_veh_in_enu.x());
  odom_veh_in_local_enu.mutable_pose()
      ->mutable_pose()
      ->mutable_orientation()
      ->set_y(quat_veh_in_enu.y());
  odom_veh_in_local_enu.mutable_pose()
      ->mutable_pose()
      ->mutable_orientation()
      ->set_z(quat_veh_in_enu.z());
  RVIZ_AGENT.Publish(kVizTopicInsPlugin + "/local_enu_pose",
                     odom_veh_in_local_enu);

  return 0;
}

int32_t VizConverterLite::OnDeadReckoning(
    hozon::netaos::adf_lite::Bundle* input) {
  if (input == nullptr) {
    HLOG_ERROR << "nullptr input Bundle";
    return -1;
  }

  auto input_msg = input->GetOne(kTopicDeadReckoning);
  if (input_msg == nullptr) {
    HLOG_ERROR << "GetOne " << kTopicDeadReckoning << " nullptr";
    return -1;
  }

  const auto proto_msg =
      std::static_pointer_cast<hozon::dead_reckoning::DeadReckoning>(
          input_msg->proto_msg);
  if (proto_msg == nullptr) {
    HLOG_ERROR << "proto_msg is nullptr in Bundle input";
    return -1;
  }

  auto msg = std::make_shared<hozon::dead_reckoning::DeadReckoning>();
  msg->CopyFrom(*proto_msg);

  RVIZ_AGENT.Publish(kVizTopicDeadReckoning, msg);

  return 0;
}

int32_t VizConverterLite::OnChassis(hozon::netaos::adf_lite::Bundle* input) {
  if (input == nullptr) {
    HLOG_ERROR << "nullptr input Bundle";
    return -1;
  }

  auto input_msg = input->GetOne(kTopicChassis);
  if (input_msg == nullptr) {
    HLOG_ERROR << "GetOne " << kTopicChassis << " nullptr";
    return -1;
  }

  const auto proto_msg =
      std::static_pointer_cast<hozon::soc::Chassis>(input_msg->proto_msg);
  if (proto_msg == nullptr) {
    HLOG_ERROR << "proto_msg is nullptr in Bundle input";
    return -1;
  }

  auto msg = std::make_shared<hozon::soc::Chassis>();
  msg->CopyFrom(*proto_msg);

  RVIZ_AGENT.Publish(kVizTopicChassis, msg);

  return 0;
}

int32_t VizConverterLite::OnMapMsg(hozon::netaos::adf_lite::Bundle* input) {
  //! 考虑当没有消息到达时（比如暂停播包时），一直发上一帧marker数据，这样rviz上就一直能看到了
  static std::shared_ptr<adsfi_proto::viz::MarkerArray> last_map_ma = nullptr;
  static std::shared_ptr<adsfi_proto::viz::MarkerArray> last_routing_ma =
      nullptr;
  static std::shared_ptr<adsfi_proto::viz::MarkerArray> last_status_ma =
      nullptr;

  if (input == nullptr) {
    HLOG_ERROR << "nullptr input Bundle";
    return -1;
  }

  auto input_msg = input->GetOne(kTopicMapMsg);
  if (input_msg == nullptr) {
    HLOG_ERROR << "GetOne " << kTopicMapMsg << " nullptr";

    if (last_map_ma != nullptr) {
      HLOG_WARN << "no new " << kVizTopicMapMsgMap << ", send last data";
      RVIZ_AGENT.Publish(kVizTopicMapMsgMap, last_map_ma);
    }
    if (last_routing_ma != nullptr) {
      HLOG_WARN << "no new " << kVizTopicMapMsgRouting << ", send last data";
      RVIZ_AGENT.Publish(kVizTopicMapMsgRouting, last_routing_ma);
    }
    if (last_status_ma != nullptr) {
      HLOG_WARN << "no new " << kVizTopicMapMsgStatus << ", send last data";
      RVIZ_AGENT.Publish(kVizTopicMapMsgStatus, last_status_ma);
    }
    return -1;
  }

  const auto proto_msg =
      std::static_pointer_cast<hozon::navigation_hdmap::MapMsg>(
          input_msg->proto_msg);
  if (proto_msg == nullptr) {
    HLOG_ERROR << "proto_msg is nullptr in Bundle input";
    last_map_ma = nullptr;
    last_routing_ma = nullptr;
    last_status_ma = nullptr;
    return -1;
  }

  uint32_t sec = 0;
  uint32_t nsec = 0;
  SplitStamp(proto_msg->header().data_stamp(), &sec, &nsec);
  adsfi_proto::hz_Adsfi::AlgHeader viz_header;
  viz_header.mutable_timestamp()->set_sec(sec);
  viz_header.mutable_timestamp()->set_nsec(nsec);
  viz_header.set_frameid(kFrameLocal);

  const double lifetime = 0.1;
  uint32_t life_sec = 0;
  uint32_t life_nsec = 0;
  SplitStamp(lifetime, &life_sec, &life_nsec);
  adsfi_proto::hz_Adsfi::HafTime viz_lifetime;
  viz_lifetime.set_sec(life_sec);
  viz_lifetime.set_nsec(life_nsec);

  last_map_ma = std::make_shared<adsfi_proto::viz::MarkerArray>();
  last_routing_ma = std::make_shared<adsfi_proto::viz::MarkerArray>();
  last_status_ma = std::make_shared<adsfi_proto::viz::MarkerArray>();

  MapMsgToMarkers(viz_header, viz_lifetime, *proto_msg, last_map_ma.get(),
                  last_routing_ma.get(), last_status_ma.get());
  RVIZ_AGENT.Publish(kVizTopicMapMsgMap, last_map_ma);
  RVIZ_AGENT.Publish(kVizTopicMapMsgRouting, last_routing_ma);
  RVIZ_AGENT.Publish(kVizTopicMapMsgStatus, last_status_ma);

  return 0;
}

int32_t VizConverterLite::OnPercepDetection(
    hozon::netaos::adf_lite::Bundle* input) {
  static std::shared_ptr<adsfi_proto::viz::MarkerArray> last_trans_ele_ma =
      nullptr;

  if (input == nullptr) {
    HLOG_ERROR << "nullptr input Bundle";
    return -1;
  }

  auto input_msg = input->GetOne(kTopicPercepDetection);
  if (input_msg == nullptr) {
    HLOG_ERROR << "GetOne " << kTopicPercepDetection << " nullptr";

    if (last_trans_ele_ma != nullptr) {
      HLOG_WARN << "no new " << kVizTopicPercepDetectionTransportElement
                << ", send last data";
      RVIZ_AGENT.Publish(kVizTopicPercepDetectionTransportElement,
                         last_trans_ele_ma);
    }
    return -1;
  }

  const auto proto_msg =
      std::static_pointer_cast<hozon::perception::measurement::MeasurementPb>(
          input_msg->proto_msg);
  if (proto_msg == nullptr) {
    HLOG_ERROR << "proto_msg is nullptr in Bundle input";
    last_trans_ele_ma = nullptr;
    return -1;
  }

  uint32_t sec = 0;
  uint32_t nsec = 0;
  SplitStamp(proto_msg->header().data_stamp(), &sec, &nsec);
  adsfi_proto::hz_Adsfi::AlgHeader viz_header;
  viz_header.mutable_timestamp()->set_sec(sec);
  viz_header.mutable_timestamp()->set_nsec(nsec);
  viz_header.set_frameid(kFrameVehicle);

  const double lifetime = 0.1;
  uint32_t life_sec = 0;
  uint32_t life_nsec = 0;
  SplitStamp(lifetime, &life_sec, &life_nsec);
  adsfi_proto::hz_Adsfi::HafTime viz_lifetime;
  viz_lifetime.set_sec(life_sec);
  viz_lifetime.set_nsec(life_nsec);

  last_trans_ele_ma = std::make_shared<adsfi_proto::viz::MarkerArray>();
  TransportElementToMarkers(viz_header, viz_lifetime,
                            proto_msg->transport_element(),
                            last_trans_ele_ma.get());
  RVIZ_AGENT.Publish(kVizTopicPercepDetectionTransportElement,
                     last_trans_ele_ma);

  return 0;
}

int32_t VizConverterLite::OnPercepTransport(
    hozon::netaos::adf_lite::Bundle* input) {
  static std::shared_ptr<adsfi_proto::viz::MarkerArray> last_trans_ele_ma =
      nullptr;
  if (input == nullptr) {
    HLOG_ERROR << "nullptr input Bundle";
    return -1;
  }

  auto input_msg = input->GetOne(kTopicPercepTransport);
  if (input_msg == nullptr) {
    HLOG_ERROR << "GetOne " << kTopicPercepTransport << " nullptr";

    if (last_trans_ele_ma != nullptr) {
      HLOG_WARN << "no new " << kVizTopicPercepTransportTransportElement
                << ", send last data";
      RVIZ_AGENT.Publish(kVizTopicPercepTransportTransportElement,
                         last_trans_ele_ma);
    }
    return -1;
  }

  const auto proto_msg =
      std::static_pointer_cast<hozon::perception::TransportElement>(
          input_msg->proto_msg);
  if (proto_msg == nullptr) {
    HLOG_ERROR << "proto_msg is nullptr in Bundle input";
    last_trans_ele_ma = nullptr;
    return -1;
  }

  uint32_t sec = 0;
  uint32_t nsec = 0;
  SplitStamp(proto_msg->header().data_stamp(), &sec, &nsec);
  adsfi_proto::hz_Adsfi::AlgHeader viz_header;
  viz_header.mutable_timestamp()->set_sec(sec);
  viz_header.mutable_timestamp()->set_nsec(nsec);
  viz_header.set_frameid(kFrameVehicle);

  const double lifetime = 0.1;
  uint32_t life_sec = 0;
  uint32_t life_nsec = 0;
  SplitStamp(lifetime, &life_sec, &life_nsec);
  adsfi_proto::hz_Adsfi::HafTime viz_lifetime;
  viz_lifetime.set_sec(life_sec);
  viz_lifetime.set_nsec(life_nsec);

  last_trans_ele_ma = std::make_shared<adsfi_proto::viz::MarkerArray>();
  TransportElementToMarkers(viz_header, viz_lifetime, *proto_msg,
                            last_trans_ele_ma.get());
  RVIZ_AGENT.Publish(kVizTopicPercepTransportTransportElement,
                     last_trans_ele_ma);

  return 0;
}

int32_t VizConverterLite::OnLocalMap(hozon::netaos::adf_lite::Bundle* input) {
  static std::shared_ptr<adsfi_proto::viz::MarkerArray> last_local_map_ma =
      nullptr;

  if (input == nullptr) {
    HLOG_ERROR << "nullptr input Bundle";
    return -1;
  }

  auto input_msg = input->GetOne(kTopicLocalMap);
  if (input_msg == nullptr) {
    HLOG_ERROR << "GetOne " << kTopicLocalMap << " nullptr";

    if (last_local_map_ma != nullptr) {
      HLOG_WARN << "no new " << kVizTopicLocalMap << ", send last data";
      RVIZ_AGENT.Publish(kVizTopicLocalMap, last_local_map_ma);
    }
    return -1;
  }

  const auto proto_msg =
      std::static_pointer_cast<hozon::mapping::LocalMap>(input_msg->proto_msg);
  if (proto_msg == nullptr) {
    HLOG_ERROR << "proto_msg is nullptr in Bundle input";
    last_local_map_ma = nullptr;
    return -1;
  }

  uint32_t sec = 0;
  uint32_t nsec = 0;
  SplitStamp(proto_msg->header().data_stamp(), &sec, &nsec);
  adsfi_proto::hz_Adsfi::AlgHeader viz_header;
  viz_header.mutable_timestamp()->set_sec(sec);
  viz_header.mutable_timestamp()->set_nsec(nsec);
  viz_header.set_frameid(kFrameVehicle);

  const double lifetime = 0.1;
  uint32_t life_sec = 0;
  uint32_t life_nsec = 0;
  SplitStamp(lifetime, &life_sec, &life_nsec);
  adsfi_proto::hz_Adsfi::HafTime viz_lifetime;
  viz_lifetime.set_sec(life_sec);
  viz_lifetime.set_nsec(life_nsec);

  last_local_map_ma = std::make_shared<adsfi_proto::viz::MarkerArray>();
  LocalMapToMarkers(viz_header, viz_lifetime, *proto_msg,
                    last_local_map_ma.get());
  RVIZ_AGENT.Publish(kVizTopicLocalMap, last_local_map_ma);

  return 0;
}

int32_t VizConverterLite::OnDrivingStatus(
    hozon::netaos::adf_lite::Bundle* input) {
  if (input == nullptr) {
    HLOG_ERROR << "nullptr input Bundle";
    return -1;
  }

  auto input_msg = input->GetOne(kTopicDrivingStatus);
  if (input_msg == nullptr) {
    HLOG_ERROR << "GetOne " << kTopicDrivingStatus << " nullptr";
    return -1;
  }

  const auto proto_msg =
      std::static_pointer_cast<hozon::planning::ADCTrajectory>(
          input_msg->proto_msg);
  if (proto_msg == nullptr) {
    HLOG_ERROR << "proto_msg is nullptr in Bundle input";
    return -1;
  }

  auto msg = std::make_shared<hozon::planning::ADCTrajectory>();
  msg->CopyFrom(*proto_msg);

  uint32_t sec = 0;
  uint32_t nsec = 0;
  SplitStamp(msg->header().data_stamp(), &sec, &nsec);

  adsfi_proto::viz::Marker marker_status;
  marker_status.mutable_header()->mutable_timestamp()->set_sec(sec);
  marker_status.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  marker_status.mutable_header()->set_frameid(mp::util::kFrameVehicle);
  marker_status.set_ns("driving_status");
  marker_status.set_id(0);
  marker_status.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
  marker_status.set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  const Eigen::Vector3d status_pos(0, -30, 10);
  marker_status.mutable_pose()->mutable_position()->set_x(status_pos.x());
  marker_status.mutable_pose()->mutable_position()->set_y(status_pos.y());
  marker_status.mutable_pose()->mutable_position()->set_z(status_pos.z());
  marker_status.mutable_pose()->mutable_orientation()->set_x(0.);
  marker_status.mutable_pose()->mutable_orientation()->set_y(0.);
  marker_status.mutable_pose()->mutable_orientation()->set_z(0.);
  marker_status.mutable_pose()->mutable_orientation()->set_w(1.);
  const double text_size = 1;
  marker_status.mutable_scale()->set_z(text_size);
  const double lifetime = 0;
  uint32_t life_sec = 0;
  uint32_t life_nsec = 0;
  SplitStamp(lifetime, &life_sec, &life_nsec);
  marker_status.mutable_lifetime()->set_sec(life_sec);
  marker_status.mutable_lifetime()->set_nsec(life_nsec);

  mp::util::Color color = mp::util::WHITE;

  auto nnp_state = msg->function_manager_in().fct_nnp_in().nnp_sysstate();
  auto pilot_state = msg->function_manager_in().fct_nnp_in().npilot_state();
  auto acc_state = msg->function_manager_in().fct_nnp_in().acc_state();
  if (nnp_state == hozon::functionmanager::NNPS_ACTIVE ||
      pilot_state == hozon::functionmanager::FctToNnpInput::PILOT_ACTIVE ||
      acc_state == hozon::functionmanager::FctToNnpInput::ACC_ACTIVE) {
    color = mp::util::GREEN;
  }

  std::string try_activate_str = "";
  if (msg->function_manager_in()
          .nnp_switch_conditions()
          .da_in_is_nnpswstsonswa_bl()) {
    color = mp::util::YELLOW;
    try_activate_str = "TRY ACTIVATE";
  }

  auto rgb = mp::util::ColorRgb(color);
  marker_status.mutable_color()->set_a(1.0);
  marker_status.mutable_color()->set_r(rgb.r);
  marker_status.mutable_color()->set_g(rgb.g);
  marker_status.mutable_color()->set_b(rgb.b);

  auto nnp_state_str = hozon::functionmanager::NNPSysState_Name(nnp_state);
  auto pilot_state_str =
      hozon::functionmanager::FctToNnpInput::NPILOT_State_Name(pilot_state);
  auto acc_state_str =
      hozon::functionmanager::FctToNnpInput::ADCS8_ACCState_Name(acc_state);
  auto fsm_state = msg->function_manager_out().fsm_state();
  auto fsm_state_str = hozon::functionmanager::MachineStateType_Name(fsm_state);
  auto hdmap_sub_state = msg->function_manager_out().hdmap_sub_state();
  auto hdmap_sub_state_str =
      hozon::functionmanager::HdmapSubState_Name(hdmap_sub_state);

  auto* text = marker_status.mutable_text();
  *text = "nnp: " + nnp_state_str + "\npilot: " + pilot_state_str +
          "\nacc: " + acc_state_str + "\npnc_used_map: " + fsm_state_str +
          ", " + hdmap_sub_state_str + "\n" + try_activate_str;
  RVIZ_AGENT.Publish(kVizTopicDrivingStatus, marker_status);

  if (msg->debug().planning_data().path_size() >= 5 &&
      msg->debug().planning_data().path(4).path_point_size() >= 2) {
    const auto& path = msg->debug().planning_data().path(4);
    adsfi_proto::viz::Marker marker_path;
    marker_path.mutable_header()->mutable_timestamp()->set_sec(sec);
    marker_path.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    marker_path.mutable_header()->set_frameid(mp::util::kFrameLocal);
    marker_path.set_ns("planning_path");
    marker_path.set_id(0);
    marker_path.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
    marker_path.set_action(adsfi_proto::viz::MarkerAction::MODIFY);
    marker_status.mutable_lifetime()->set_sec(life_sec);
    marker_status.mutable_lifetime()->set_nsec(life_nsec);
    rgb = mp::util::ColorRgb(mp::util::BLUE);
    marker_path.mutable_color()->set_a(0.3);
    marker_path.mutable_color()->set_r(rgb.r);
    marker_path.mutable_color()->set_g(rgb.g);
    marker_path.mutable_color()->set_b(rgb.b);
    double width = 1.5;
    marker_path.mutable_scale()->set_x(width);
    marker_path.mutable_pose()->mutable_orientation()->set_x(0.);
    marker_path.mutable_pose()->mutable_orientation()->set_y(0.);
    marker_path.mutable_pose()->mutable_orientation()->set_z(0.);
    marker_path.mutable_pose()->mutable_orientation()->set_w(1.);
    for (const auto& pt : path.path_point()) {
      auto* mpt = marker_path.add_points();
      mpt->set_x(pt.x());
      mpt->set_y(pt.y());
      mpt->set_z(0.);
    }
    RVIZ_AGENT.Publish(kVizTopicPlanningPath, marker_path);
  }

  return 0;
}

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
