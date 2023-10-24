/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： local_map_provider.h
 *   author     ： xuliang
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <depend/proto/common/header.pb.h>
#include <depend/proto/local_mapping/local_map.pb.h>
#include <depend/proto/localization/localization.pb.h>
#include <depend/proto/localization/node_info.pb.h>
#include <depend/proto/perception/transport_element.pb.h>

#include <memory>
#include <mutex>
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
      const std::shared_ptr<hozon::localization::HafNodeInfo>& msg);
  void OnLocation(
      const std::shared_ptr<hozon::localization::Localization>& msg);
  void OnLaneLine(
      const std::shared_ptr<hozon::perception::TransportElement>& msg);
  void OnRoadEdge(
      const std::shared_ptr<hozon::perception::TransportElement>& msg);

  std::shared_ptr<hozon::mapping::LocalMap> GetLocalMap();

 private:
  std::shared_ptr<hozon::mapping::LocalMap> local_map_ = nullptr;
  std::shared_ptr<hozon::mapping::LocalMap> local_map_write_ = nullptr;

  Eigen::Quaterniond q_W_V_;
  Eigen::Vector3d ref_point_;
  Eigen::Vector3d enu_;
  Eigen::Vector3d pos_enu_add_lane_;

  bool init_ = false;
  bool flag_ = false;

  std::mutex map_mtx_;

  const std::string kTopicLocalMapProviderLaneLine = "/localmap/lane";
  const std::string kTopicLocalMapProviderTf = "/localmap/tf";
  const std::string kTopicLocalMapLocation = "/localmap/location";
  const std::string KTopicLocalMapProviderMap = "/localmap/local_map";

  adsfi_proto::viz::Path location_path_;

  void VizLocalMap(const std::shared_ptr<hozon::mapping::LocalMap>& local_map);
  void SetLaneLine(
      std::vector<Eigen::Vector3d>* points,
      const std::shared_ptr<hozon::perception::TransportElement>& msg);
  void VizLaneLine(const std::vector<Eigen::Vector3d>& points,
                   const double stamp);
  void VizLocation(const Eigen::Vector3d& pose, const Eigen::Quaterniond& q_W_V,
                   const double stamp);
  void LaneLineToMarker(const double stamp,
                        const hozon::mapping::LaneLine& lane_line,
                        adsfi_proto::viz::Marker* marker);
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
