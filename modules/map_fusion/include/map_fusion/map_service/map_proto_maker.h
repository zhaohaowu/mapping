/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： local_map_provider.h
 *   author     ： zuodongsheng
 *   date       ： 2023.09
 ******************************************************************************/
#pragma once

#include <depend/proto/localization/node_info.pb.h>
#include <depend/proto/map/map.pb.h>

#include <memory>
#include <string>
#include <vector>

#include "common/utm_projection/coordinate_convertor.h"
#include "util/geo.h"
#include "util/rviz_agent/rviz_agent.h"

namespace hozon {
namespace mp {
namespace mf {

class MapProtoMarker {
 public:
  MapProtoMarker() = default;
  ~MapProtoMarker() = default;

  adsfi_proto::viz::TransformStamped CarTrackTF(
      const Eigen::Vector3d& pose, const Eigen::Quaterniond& q_W_V,
      const hozon::common::Header& stamp, bool utm = false);
  //   用时将location_path设成私有变量
  void CarTrack(const Eigen::Vector3d& pose, const Eigen::Quaterniond& q_W_V,
                adsfi_proto::viz::Path* location_path,
                const hozon::common::Header& stamp, bool utm = false);
  adsfi_proto::viz::MarkerArray LaneID(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos, bool utm = false);
  // right_neighbor_forward message
  adsfi_proto::viz::MarkerArray LaneRightNeighborForward(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos, bool utm = false);
  // left_neighbor_forward messag
  adsfi_proto::viz::MarkerArray LaneLeftNeighborForward(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos, bool utm = false);
  // predecessor message
  adsfi_proto::viz::MarkerArray LanePredecessor(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos, bool utm = false);
  // successor message
  adsfi_proto::viz::MarkerArray LaneSuccessor(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos, bool utm = false);
  // 地图坐标系UTM  VECTOR <CENTRAL CURVE ; LEFT ;RIGHT>
  std::vector<adsfi_proto::viz::MarkerArray> LaneToMarker(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos, bool utm = false);
  adsfi_proto::viz::MarkerArray JunctionToMarker(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos, bool utm = false);
  adsfi_proto::viz::MarkerArray RoadToMarker(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos, bool utm = false);
  adsfi_proto::viz::MarkerArray SignalToMarker(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos, bool utm = false);
  adsfi_proto::viz::MarkerArray CrossWalkToMarker(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos, bool utm = false);
  adsfi_proto::viz::MarkerArray StopSignToMarker(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos, bool utm = false);
  adsfi_proto::viz::MarkerArray ClearAreaToMarker(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos, bool utm = false);
  adsfi_proto::viz::MarkerArray SpeedBumpToMarker(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos, bool utm = false);

 private:
  std::shared_ptr<hozon::hdmap::Map> prior_map_ = nullptr;
  adsfi_proto::viz::MarkerArray LaneLeftBoundaryMarker(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos, bool utm = false);
  adsfi_proto::viz::MarkerArray LaneRightBoundaryMarker(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos, bool utm = false);
  Eigen::Vector3d ConvertPoint(const hozon::common::PointENU& end_point,
                               const Eigen::Vector3d& enupos) {
    Eigen::Vector3d point_utm_end(end_point.x(), end_point.y(), end_point.z());
    int zone = 51;
    double x_end = point_utm_end.x();
    double y_end = point_utm_end.y();
    hozon::common::coordinate_convertor::UTM2GCS(zone, &x_end, &y_end);
    Eigen::Vector3d point_gcj_end(y_end, x_end, 0);
    Eigen::Vector3d point_enu_end =
        util::Geo::Gcj02ToEnu(point_gcj_end, enupos);

    return point_enu_end;
  }
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
