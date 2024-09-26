/***************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： location_rviz.h
 *   author     ： zhaohaowu
 *   date       ： 2024.01
 ******************************************************************************/

#pragma once
#include <string>
#include <unordered_map>
#include <vector>

#include "Eigen/Dense"
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Geometry/Transform.h"
#include "base/utils/macros.h"
#include "modules/location/pose_estimation/lib/pose_estimate/pose_estimate.h"
#include "modules/location/pose_estimation/lib/reloc/reloc.hpp"
#include "util/rviz_agent/rviz_agent.h"
namespace hozon {
namespace mp {
namespace loc {

class LocationRviz {
 public:
  ~LocationRviz();
  std::string Name() const;
  bool Init();

  void PubFcOdom(const Eigen::Affine3d& T_W_V, uint64_t sec, uint64_t nsec,
                 const std::string& topic);
  void PubMmOdom(const Eigen::Affine3d& T_W_V, uint64_t sec, uint64_t nsec,
                 const std::string& topic);
  void PubDrOdom(const Eigen::Affine3d& T_W_V, uint64_t sec, uint64_t nsec,
                 const std::string& topic);
  void PubInsOdom(const Eigen::Affine3d& T_W_V, uint64_t sec, uint64_t nsec,
                  const std::string& topic);
  void PubInputOdom(const Eigen::Affine3d& T_W_V, uint64_t sec, uint64_t nsec,
                    const std::string& topic);
  void PubRelocOdom(const Eigen::Affine3d& T_W_V, uint64_t sec, uint64_t nsec,
                    const std::string& topic);

  void PubFcTf(const Eigen::Affine3d& T_W_V, const std::string& topic);
  void PubPerceptionByInput(const Eigen::Affine3d& T_input,
                            const pe::TrackingManager& perception,
                            const std::string& topic);
  void PubPerceptionMarkerByFc(const Eigen::Affine3d& T_fc_10hz,
                               const pe::TrackingManager& perception,
                               const std::string& topic);
  void PubHdmapMarker(const Eigen::Affine3d& T_fc_10hz,
                      const pe::MappingManager& hdmap,
                      const std::string& topic);
  void PubInsLocationState(int ins_state, double ins_sd_position,
                           double ins_height, double ins_heading,
                           double gps_week, double gps_sec, int fc_state,
                           double velocity_vrf, double fc_heading,
                           const std::string& fc_conv, bool mm_valid,
                           int warn_info, double per_time,
                           const std::string& topic);
  void PubOriginConnectMapPoints(const hozon::mp::loc::Connect& origin_connect,
                                 uint64_t sec, uint64_t nsec,
                                 const std::string& topic);
  void PubOriginConnectPercepPoints(
      const hozon::mp::loc::Connect& origin_connect,
      const Eigen::Affine3d& T_W_V, uint64_t sec, uint64_t nsec,
      const std::string& topic);
  void PubConnectMapPoints(const hozon::mp::loc::Connect& origin_connect,
                           uint64_t sec, uint64_t nsec,
                           const std::string& topic);
  void PubConnectPercepPoints(const hozon::mp::loc::Connect& origin_connect,
                              const Eigen::Affine3d& T_W_V, uint64_t sec,
                              uint64_t nsec, const std::string& topic);
  void PubMergeMapLines(
      const std::unordered_map<std::string, std::vector<ControlPoint>>&
          merged_map_lines,
      const Eigen::Affine3d& T_W_V, uint64_t sec, uint64_t nsec,
      const std::string& topic);

  void SetHdmap(const pe::MappingManager& hd_map);
  void SetPerception(const pe::TrackingManager& perception);
  void SetFcTf(const Eigen::Affine3d& T_fc_100hz);
  void SetFc(const Eigen::Affine3d& T_fc_10hz);
  void SetInputPose(const Eigen::Affine3d& T_input);
  // ins
  void SetInsState(int ins_state);
  void SetInsSdPosition(double ins_sd_position);
  void SetInsHeight(double ins_height);
  void SetInsHeading(double ins_heading);
  void SetGpsWeek(double gps_week);
  void SetGpsSec(double gps_sec);
  // fc
  void SetFcState(int fc_state);
  void SetVelocityVrf(double velocity_vrf);
  void SetFcHeading(double fc_heading);
  void SetFcConv(const std::string& fc_conv);
  // mm
  void SetWarnInfo(int warn_info);
  void SetMmValid(bool mm_valid);
  void SetPerTime(double per_time);

  void LocRvizRun();

 private:
  bool inited_ = false;
  std::mutex loc_rviz_mutex_;
  std::thread loc_rviz_thread_;
  unsigned int sec_;
  unsigned int nsec_;
  bool loc_rviz_thread_run_ = false;
  // 定位信息
  Eigen::Affine3d T_fc_10hz_;
  Eigen::Affine3d T_fc_100hz_;
  Eigen::Affine3d T_input_;
  // 地图信息
  pe::MappingManager hd_map_;
  // 感知信息
  pe::TrackingManager perception_;
  // ins信息
  int ins_state_ = -1;
  double ins_sd_position_ = -1;
  double ins_height_ = -1;
  double ins_heading_ = -1;
  double gps_week_ = -1;
  double gps_sec_ = -1;
  // fc信息
  int fc_state_ = -1;
  double velocity_vrf_ = -1;
  double fc_heading_ = -1;
  std::string fc_conv_;
  // mm信息
  bool mm_valid_ = false;
  int warn_info_ = -1;
  // 感知信息
  double per_time_ = -1;

  DECLARE_SINGLETON_PERCEPTION(LocationRviz)
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon

#define LOC_RVIZ hozon::mp::loc::LocationRviz::Instance()
