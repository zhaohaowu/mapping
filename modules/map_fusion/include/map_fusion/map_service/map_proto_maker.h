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

  static adsfi_proto::viz::TransformStamped CarTrackTF(
      const Eigen::Vector3d& pose, const Eigen::Quaterniond& q_W_V,
      const hozon::common::Header& stamp);
  //   用时将location_path设成私有变量
  static void CarTrack(const Eigen::Vector3d& pose,
                       const Eigen::Quaterniond& q_W_V,
                       adsfi_proto::viz::Path* location_path,
                       const hozon::common::Header& stamp);
  static adsfi_proto::viz::MarkerArray LaneID(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos, bool utm = false);
  // right_neighbor_forward message
  static adsfi_proto::viz::MarkerArray LaneRightNeighborForward(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos, bool utm = false);
  // left_neighbor_forward messag
  static adsfi_proto::viz::MarkerArray LaneLeftNeighborForward(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos, bool utm = false);
  // predecessor message
  static adsfi_proto::viz::MarkerArray LanePredecessor(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos, bool utm = false);
  // successor message
  static adsfi_proto::viz::MarkerArray LaneSuccessor(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos, bool utm = false);
  // 地图坐标系UTM  VECTOR <CENTRAL CURVE ; LEFT ;RIGHT>
  static std::vector<adsfi_proto::viz::MarkerArray> LaneToMarker(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos, bool utm = false);
  static adsfi_proto::viz::MarkerArray JunctionToMarker(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map);
  static adsfi_proto::viz::MarkerArray RoadToMarker(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos, bool utm = false);
  static adsfi_proto::viz::MarkerArray SignalToMarker(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos, bool utm = false);
  static adsfi_proto::viz::MarkerArray CrossWalkToMarker(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos);
  static adsfi_proto::viz::MarkerArray StopSignToMarker(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos);
  static adsfi_proto::viz::MarkerArray ClearAreaToMarker(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos);
  static adsfi_proto::viz::MarkerArray SpeedBumpToMarker(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos);

 private:
  std::shared_ptr<hozon::hdmap::Map> prior_map_ = nullptr;
  static adsfi_proto::viz::MarkerArray LaneLeftBoundaryMarker(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos, bool utm = false);
  static adsfi_proto::viz::MarkerArray LaneRightBoundaryMarker(
      const std::shared_ptr<hozon::hdmap::Map>& prior_map,
      const Eigen::Vector3d& enupos, bool utm = false);
  static Eigen::Vector3d ConvertPoint(const hozon::common::PointENU& end_point,
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
  static void AddPointToMarker(const hozon::common::PointENU& point,
                               adsfi_proto::viz::Marker* marker,
                               const Eigen::Vector3d& enupos, bool utm) {
    if (utm) {
      Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
      int zone = 51;
      double x = point_utm.x();
      double y = point_utm.y();
      hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
      Eigen::Vector3d point_gcj(y, x, 0);
      Eigen::Vector3d point_enu = util::Geo::Gcj02ToEnu(point_gcj, enupos);
      auto* pt = marker->add_points();
      pt->set_x(point_enu.x());
      pt->set_y(point_enu.y());
      pt->set_z(point_enu.z());
    } else {
      Eigen::Vector3d point_enu(point.x(), point.y(), point.z());
      auto* pt = marker->add_points();
      pt->set_x(point_enu.x());
      pt->set_y(point_enu.y());
      pt->set_z(point_enu.z());
    }
  }
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
