/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： viz_converter_lite.h
 *   author     ： taoshaoyuan
 *   date       ： 2024.01
 ******************************************************************************/
#pragma once

#include <adf-lite/include/base.h>
#include <depend/nos/x86_2004/include/adf-lite/include/executor.h>
#include <depend/nos/x86_2004/include/adf/include/node_proto_register.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <mutex>
#include <string>

namespace hozon {
namespace perception {
namespace common_onboard {

class VizConverterLite : public hozon::netaos::adf_lite::Executor {
 public:
  VizConverterLite() = default;
  ~VizConverterLite() = default;

  int32_t AlgInit() override;
  void AlgRelease() override;

 private:
  int32_t OnLocalization(hozon::netaos::adf_lite::Bundle* input);
  int32_t OnPoseEstimation(hozon::netaos::adf_lite::Bundle* input);
  int32_t OnCamera0(hozon::netaos::adf_lite::Bundle* input);
  int32_t OnImuIns(hozon::netaos::adf_lite::Bundle* input);
  int32_t OnInsPlugin(hozon::netaos::adf_lite::Bundle* input);
  int32_t OnDeadReckoning(hozon::netaos::adf_lite::Bundle* input);
  int32_t OnChassis(hozon::netaos::adf_lite::Bundle* input);
  int32_t OnMapMsg(hozon::netaos::adf_lite::Bundle* input);
  int32_t OnPercepDetection(hozon::netaos::adf_lite::Bundle* input);
  int32_t OnPercepTransport(hozon::netaos::adf_lite::Bundle* input);
  int32_t OnLocalMap(hozon::netaos::adf_lite::Bundle* input);
  int32_t OnDrivingStatus(hozon::netaos::adf_lite::Bundle* input);

  const std::string kTopicLocalization = "localization";
  const std::string kTopicPoseEstimation = "pe";
  const std::string kVizTopicLocalization = "localization";
  const std::string kVizTopicLocalizationLocalPose =
      kVizTopicLocalization + "/local_pose";
  const std::string kVizTopicLocalizationLocalEnuPose =
      kVizTopicLocalization + "/local_enu_pose";
  const std::string kVizTopicLocalizationStatus =
      kVizTopicLocalization + "/status";

  const std::string kTopicCamera0 = "camera_0";
  const std::string kVizTopicCamera0 = kTopicCamera0;

  const std::string kTopicDeadReckoning = "dr";
  const std::string kVizTopicDeadReckoning = kTopicDeadReckoning;

  const std::string kTopicImuIns = "imu_ins";
  const std::string kVizTopicImuIns = kTopicImuIns;

  const std::string kTopicInsPlugin = "inspva";
  const std::string kVizTopicInsPlugin = kTopicInsPlugin;
  const std::string kVizTopicInsPluginStatus = kVizTopicInsPlugin + "/status";

  const std::string kTopicChassis = "chassis";
  const std::string kVizTopicChassis = kTopicChassis;

  const std::string kTopicMapMsg = "map_fusion";
  const std::string kVizTopicMapMsgMap = kTopicMapMsg + "/map";
  const std::string kVizTopicMapMsgRouting = kTopicMapMsg + "/routing";
  const std::string kVizTopicMapMsgStatus = kTopicMapMsg + "/status";

  const std::string kTopicPercepDetection = "percep_detection";
  const std::string kVizTopicPercepDetectionTransportElement =
      kTopicPercepDetection + "/transport_element";

  const std::string kTopicPercepTransport = "percep_transport";
  const std::string kVizTopicPercepTransportTransportElement =
      kTopicPercepTransport + "/transport_element";

  const std::string kTopicLocalMap = "local_map";
  const std::string kVizTopicLocalMap = kTopicLocalMap;

  const std::string kTopicDrivingStatus = "driving_status";
  const std::string kVizTopicDrivingStatus = kTopicDrivingStatus;
  const std::string kVizTopicPlanningPath = "planning_path";

  const std::string kVizTopicTfVehicleToLocal = "veh_to_local";
  const std::string kVizTopicTfLocalEnuToLocal = "local_enu_to_local";

  bool enu_station_inited_ = false;
  Eigen::Vector3d enu_station_;
  Eigen::Quaterniond quat_enu_in_local_;
  Eigen::Vector3d pos_enu_in_local_;
};

REGISTER_ADF_CLASS(VizConverterLite, VizConverterLite);

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
