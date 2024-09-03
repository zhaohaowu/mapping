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
namespace hozon {
namespace mp {
namespace loc {

class LocationRviz {
 public:
  ~LocationRviz() = default;
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

  void PubFcTf(const Eigen::Affine3d& T_W_V, uint64_t sec, uint64_t nsec,
               const std::string& topic);
  void PubMapLocalmapTf(const Eigen::Affine3d& T_W_V, uint64_t sec,
                        uint64_t nsec, const std::string& topic);
  void PubFcPath(const Eigen::Affine3d& T_W_V, uint64_t sec, uint64_t nsec,
                 const std::string& topic);
  void PubPerceptionByFc(const Eigen::Affine3d& T_W_V,
                         const pe::TrackingManager& perception, uint64_t sec,
                         uint64_t nsec, const std::string& topic);
  void PubPerceptionByInput(const Eigen::Affine3d& T_W_V,
                            const pe::TrackingManager& perception, uint64_t sec,
                            uint64_t nsec, const std::string& topic);
  void PubPerceptionMarkerByFc(const Eigen::Affine3d& T_W_V,
                               const pe::TrackingManager& perception,
                               uint64_t sec, uint64_t nsec,
                               const std::string& topic);
  void PubPerceptionMarkerReloc(const Eigen::Affine3d& T_W_V,
                                const pe::TrackingManager& perception,
                                uint64_t sec, uint64_t nsec,
                                const std::string& topic);
  void PubHdmap(const Eigen::Affine3d& T_W_V, const pe::MappingManager& hdmap,
                uint64_t sec, uint64_t nsec, const std::string& topic);
  void PubHdmapMarker(const Eigen::Affine3d& T_W_V,
                      const pe::MappingManager& hdmap, uint64_t sec,
                      uint64_t nsec, const std::string& topic);
  void PubInsLocationState(int ins_state, double sd_position,
                           int location_state, double timestamp,
                           double velocity, double fc_heading,
                           double ins_heading, const std::string& conv,
                           double gps_week, double gps_sec,
                           const Eigen::Vector3d& FC_KF_kydiff,
                           const Eigen::Vector3d& FC_KF_cov, uint64_t sec,
                           uint64_t nsec, const std::string& topic);
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

 private:
  bool inited_ = false;

  DECLARE_SINGLETON_PERCEPTION(LocationRviz)
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon

#define LOC_RVIZ hozon::mp::loc::LocationRviz::Instance()
