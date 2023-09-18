/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： local_map_provider.h
 *   author     ： xuliang
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <adsfi_proto/internal/node_info.pb.h>
#include <adsfi_proto/location/location.pb.h>
#include <adsfi_proto/map/local_map.pb.h>
#include <adsfi_proto/perception/lanes.pb.h>

#include <memory>
#include <string>
#include <vector>

#include "util/geo.h"
#include "util/rviz_agent/rviz_agent.h"

namespace hozon {
namespace mp {
namespace mf {

class LocalMapProvider {
 public:
  LocalMapProvider() = default;
  ~LocalMapProvider();

  int Init();

  void OnInsNodeInfo(
      const std::shared_ptr<adsfi_proto::internal::HafNodeInfo>& msg);
  void OnLocation(
      const std::shared_ptr<adsfi_proto::hz_Adsfi::AlgLocation>& msg);
  void OnLaneLine(
      const std::shared_ptr<adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray>&
          msg);
  void OnRoadEdge(
      const std::shared_ptr<adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray>&
          msg);

  std::shared_ptr<hozon::mapping::LocalMap> GetLocalMap();

 private:
  std::shared_ptr<hozon::mapping::LocalMap> local_map_ = nullptr;

  Eigen::Quaterniond q_W_V_;
  Eigen::Vector3d ref_point_;
  Eigen::Vector3d enu_;
  Eigen::Vector3d pos_enu_add_lane_;

  bool init_ = false;
  bool flag_ = false;

  const std::string kTopicLocalMapProviderLaneLine = "/localmap/lane";
  const std::string kTopicLocalMapProviderTf = "/localmap/tf";
  const std::string kTopicLocalMapLocation = "/localmap/location";
  const std::string KTopicLocalMapProviderMap = "/localmap/local_map";

  adsfi_proto::viz::Path location_path_;

  void VizLocalMap(const std::shared_ptr<hozon::mapping::LocalMap>& local_map);
  void SetLaneLine(
      std::vector<Eigen::Vector3d>* points,
      const std::shared_ptr<adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray>&
          msg);
  void VizLaneLine(const std::vector<Eigen::Vector3d>& points,
                   const adsfi_proto::hz_Adsfi::HafTime& stamp);
  void VizLocation(const Eigen::Vector3d& pose, const Eigen::Quaterniond& q_W_V,
                   const adsfi_proto::hz_Adsfi::HafTime& stamp);
  void LaneLineToMarker(double stamp, const hozon::mapping::LaneInfo& lane_line,
                        adsfi_proto::viz::Marker* marker);
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
