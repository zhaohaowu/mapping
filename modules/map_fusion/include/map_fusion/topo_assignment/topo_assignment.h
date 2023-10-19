/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： topo_assignment.h
 *   author     ： xuliang
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <depend/map/hdmap/hdmap.h>
#include <depend/proto/local_mapping/local_map.pb.h>
#include <depend/proto/localization/localization.pb.h>
#include <depend/proto/localization/node_info.pb.h>
#include <depend/proto/map/map.pb.h>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "map_fusion/map_service/map_proto_maker.h"
#include "util/geo.h"
#include "util/rviz_agent/rviz_agent.h"

namespace hozon {
namespace mp {
namespace mf {

struct LaneLine {
  int32_t id;
  std::vector<std::string> left_lanes;   // 属于哪些车道的左边线
  std::vector<std::string> right_lanes;  // 属于哪些车道的右边线
  std::vector<hozon::common::Point3D> points;
};

struct Lane {
  std::string id;

  std::vector<int32_t> left_lines;  // left LaneLine id左边车道线的track id
  std::vector<int32_t> right_lines;  // right LaneLine id右边车道线的track id

  std::vector<std::string> left_lanes;
  std::vector<std::string> right_lanes;

  std::vector<std::string> prev_lanes;
  std::vector<std::string> next_lanes;
};

class TopoAssignment {
 public:
  TopoAssignment() = default;
  ~TopoAssignment() = default;

  int Init();

  void OnInsNodeInfo(
      const std::shared_ptr<hozon::localization::HafNodeInfo>& msg);
  void OnHQMap(const std::shared_ptr<hozon::hdmap::Map>& msg);
  void OnLocalMap(const std::shared_ptr<hozon::mapping::LocalMap>& msg);
  void OnLocalMapLocation(
      const std::shared_ptr<hozon::localization::Localization>& msg);

  std::shared_ptr<hozon::hdmap::Map> GetTopoMap();

 private:
  std::shared_ptr<hozon::hdmap::Map> topo_map_ = nullptr;
  std::shared_ptr<hozon::hdmap::Map> crop_map_ = nullptr;
  std::shared_ptr<hozon::hdmap::HDMap> hq_map_server_ = nullptr;
  Eigen::Vector3d vehicle_pose_;
  Eigen::Vector3d ref_point_;

  std::vector<hozon::hdmap::Id> location_left_id_;
  std::vector<hozon::hdmap::Id> location_right_id_;

  std::vector<hozon::hdmap::Id> location_left_id_next_;
  std::vector<hozon::hdmap::Id> location_right_id_next_;

  std::map<int32_t, LaneLine> all_lanelines_;

  std::mutex map_mtx_;
  std::mutex pose_mtx_;
  std::mutex location_mtx_;
  std::mutex crop_map_mtx_;

  Eigen::Vector3d local_map_pose_;
  Eigen::Quaterniond local_map_q_w_v_;

  Eigen::Vector3d ins_pose_;
  Eigen::Quaterniond ins_q_w_v_;

  bool init_ = false;
  double cur_timestamp_ = 0.;

  const std::string kTopicTopoAsignTf = "/topo/tf";
  const std::string kTopicTopoAsignLocation = "/topo/location";
  const std::string KTopicTopoAsignLocalMap = "/topo/local_map";
  const std::string KTopicTopoAsignHQMapRoad = "/topo/hq_map_road";
  const std::string KTopicTopoAsignHQMapLane = "/topo/hq_map_lane";
  const std::string KTopicTopoAsignTopoMapRoad = "/topo/topo_map_road";
  const std::string KTopicTopoAsignTopoMapLane = "/topo/topo_map_lane";

  adsfi_proto::viz::Path location_path_;

  void AppendLaneLine(const hozon::hdmap::Id& lane_id,
                      hozon::mp::mf::LaneLine* lane_line,
                      const Eigen::Vector2d& p1, const bool left);
  void AppendLane(const std::map<int32_t, LaneLine>& all_lanelines,
                  std::map<std::string, Lane>* all_lanes);
  void AppendTopoMap(const std::map<std::string, Lane>& all_lanes,
                     const std::shared_ptr<hozon::hdmap::Map>& topo_map);
  void VizLocalMap(const std::shared_ptr<hozon::mapping::LocalMap>& local_map,
                   const Eigen::Isometry3d& T_U_W);
  void VizLocation(const Eigen::Vector3d& pose, const Eigen::Quaterniond& q_W_V,
                   const double stamp);
  void VizHQMap(const std::shared_ptr<hozon::hdmap::Map>& msg);
  void VizTopoMap(const std::shared_ptr<hozon::hdmap::Map>& msg);
  void PointsToMarker(const double stamp,
                      const std::vector<Eigen::Vector3d>& points,
                      adsfi_proto::viz::Marker* marker, double color_type);
  void LineIdToMarker(const double stamp, const Eigen::Vector3d& point,
                      const std::string& id, adsfi_proto::viz::Marker* marker);
  int FindNearestPointIndex(const Eigen::Vector2d& point,
                            const std::vector<hozon::common::Point3D>& points);
  bool PerpendicularFootInSegment(const Eigen::Vector2d& p0,
                                  const Eigen::Vector2d& p1,
                                  const Eigen::Vector2d& pt);
  std::vector<Eigen::Vector2d> GetLaneStartAndEndPoint(
      const hozon::hdmap::Lane& lane, const bool left);
  // 可视化的class
  MapProtoMarker marker_rviz_;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
