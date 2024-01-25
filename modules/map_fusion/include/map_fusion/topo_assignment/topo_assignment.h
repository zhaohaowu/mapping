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
#include <unordered_map>
#include <utility>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>

#include "map_fusion/map_service/map_proto_maker.h"
#include "map_fusion/map_service/map_table.h"
#include "util/geo.h"
#include "util/rviz_agent/rviz_agent.h"

namespace hozon {
namespace mp {
namespace mf {

struct AllLaneInfo {
  std::string id;
  uint32_t extra_boundary = 0;
  std::vector<Eigen::Vector2d> left_start_end;
  std::vector<Eigen::Vector2d> right_start_end;
  std::vector<std::string> left_lane_ids;
  std::vector<std::string> right_lane_ids;
  std::vector<std::string> prev_lane_ids;
  std::vector<std::string> next_lane_ids;
};

struct LaneLine {
  int32_t id;
  std::vector<std::string> left_lanes;   // 属于哪些车道的左边线
  std::vector<std::string> right_lanes;  // 属于哪些车道的右边线
  std::vector<hozon::common::Point3D> points;
  std::shared_ptr<cv::flann::Index> lane_line_kdtree;
};

struct Lane {
  std::string id;
  uint32_t extra_boundary = 0;
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
  void OnLocalMap(
      const std::shared_ptr<hozon::mapping::LocalMap>& msg,
      const std::unordered_map<std::string, hozon::mp::mf::LaneInfo>&
          all_lane_info);
  void OnLocalization(
      const std::shared_ptr<hozon::localization::Localization>& msg);
  void TopoAssign();
  std::shared_ptr<hozon::hdmap::Map> GetTopoMap();

 private:
  std::shared_ptr<hozon::hdmap::Map> topo_map_ = nullptr;
  std::shared_ptr<hozon::mapping::LocalMap> local_map_ = nullptr;
  // gcj02 global position
  Eigen::Vector3d vehicle_pose_;
  Eigen::Vector3d ref_point_;

  int utm_zone_ = 0;

  std::vector<std::string> location_left_id_;
  std::vector<std::string> location_right_id_;

  std::vector<std::string> location_left_id_next_;
  std::vector<std::string> location_right_id_next_;

  //! TSY：每来一帧local map都重新构造出来
  std::map<int32_t, LaneLine> all_lanelines_;
  std::map<std::string, AllLaneInfo> all_lane_info_;

  std::mutex pose_mtx_;
  std::mutex vehicle_pose_mtx_;
  std::mutex local_map_mtx_;
  // position in local enu frame
  Eigen::Vector3d ins_pose_;
  // orientation in local enu frame
  Eigen::Quaterniond ins_q_w_v_;

  bool init_ref_point_ = false;
  double cur_timestamp_ = 0.;

  const std::string kTopicTopoAssignTf = "/topo/tf";
  const std::string kTopicTopoAssignLocation = "/topo/location";
  const std::string KTopicTopoAssignLocalMap = "/topo/local_map";
  const std::string KTopicTopoAssignHQMapRoad = "/topo/hq_map_road";
  const std::string KTopicTopoAssignHQMapLane = "/topo/hq_map_lane";
  const std::string KTopicTopoAssignTopoMapRoad = "/topo/topo_map_road";
  const std::string KTopicTopoAssignTopoMapLane = "/topo/topo_map_lane";
  const std::string KTopicTopoAssignTopoMapElements = "/topo/topo_map_elements";

  adsfi_proto::viz::Path location_path_;

  void AppendAllLaneInfo(
      const std::unordered_map<std::string, hozon::mp::mf::LaneInfo>&
          all_lane_info);
  void AppendLocationLaneId(const hozon::hdmap::LaneInfoConstPtr& lane_ptr);
  void AppendLaneLineCaseLeft(hozon::mp::mf::LaneLine* lane_line,
                              const Eigen::Vector2d& p0,
                              const Eigen::Vector2d& p1, const size_t size);
  void AppendLaneLineCaseRight(hozon::mp::mf::LaneLine* lane_line,
                               const Eigen::Vector2d& p0,
                               const Eigen::Vector2d& p1, const size_t size);
  void AppendLaneLineCaseLeftRight(hozon::mp::mf::LaneLine* lane_line,
                                   const Eigen::Vector2d& p0,
                                   const Eigen::Vector2d& p1,
                                   const size_t size);
  void AppendLaneLineLeft(const std::string& lane_id,
                          hozon::mp::mf::LaneLine* lane_line,
                          const Eigen::Vector2d& p0, const Eigen::Vector2d& p1);
  void AppendLaneLineLeftOutside(const std::string& lane_id,
                                 hozon::mp::mf::LaneLine* lane_line,
                                 const Eigen::Vector2d& p0,
                                 const Eigen::Vector2d& p1,
                                 const double distance_p0,
                                 const double distance_p1);
  void AppendLaneLineRight(const std::string& lane_id,
                           hozon::mp::mf::LaneLine* lane_line,
                           const Eigen::Vector2d& p0,
                           const Eigen::Vector2d& p1);
  void AppendLaneLineRightOutside(const std::string& lane_id,
                                  hozon::mp::mf::LaneLine* lane_line,
                                  const Eigen::Vector2d& p0,
                                  const Eigen::Vector2d& p1,
                                  const double distance_p0,
                                  const double distance_p1);
  std::vector<std::string> FindLaneLineHead(const std::string& lane_id,
                                            const Eigen::Vector2d& pt,
                                            const bool left);
  std::vector<std::string> FindLaneLineTail(const std::string& lane_id,
                                            const Eigen::Vector2d& pt,
                                            const bool left);
  void AppendLane(const std::map<int32_t, LaneLine>& all_lanelines,
                  std::pair<bool, std::map<std::string, Lane>>* all_lanes);
  void AppendLaneLanes(std::pair<bool, std::map<std::string, Lane>>* all_lanes);
  static void AppendLaneLanesSplit(
      std::pair<bool, std::map<std::string, Lane>>* all_lanes);
  void AppendTopoMapGeometry(
      std::pair<bool, std::map<std::string, Lane>>* all_lanes,
      const std::shared_ptr<hozon::hdmap::Map>& topo_map_geo);
  static void AppendTopoMapTopology(
      const std::map<std::string, Lane>& all_lanes,
      const std::shared_ptr<hozon::hdmap::Map>& topo_map_geo,
      const std::shared_ptr<hozon::hdmap::Map>& topo_map);
  void AppendTopoMapRoadEdge(const std::shared_ptr<hozon::hdmap::Map>& topo_map,
                             const Eigen::Isometry3d& T_U_V);
  void AppendTopoMapElements(const std::shared_ptr<hozon::hdmap::Map>& topo_map,
                             const Eigen::Isometry3d& T_U_V);
  void AppendTopoMapLeftLanes(
      const hozon::mp::mf::Lane& lane_it, hozon::hdmap::Lane* lane,
      const std::vector<Eigen::Vector2d>& hq_lane_left_points,
      const size_t size, const bool split);
  void AppendTopoMapRightLanes(
      const hozon::mp::mf::Lane& lane_it, hozon::hdmap::Lane* lane,
      const std::vector<Eigen::Vector2d>& hq_lane_right_points,
      const size_t size, const bool split);
  void AppendTopoMapLeftLanePoints(
      const std::vector<Eigen::Vector2d>& lane_points, hozon::hdmap::Lane* lane,
      const int start_index, const int end_index, const int track_id,
      const bool split);
  void AppendTopoMapRightLanePoints(
      const std::vector<Eigen::Vector2d>& lane_points, hozon::hdmap::Lane* lane,
      const int start_index, const int end_index, const int track_id,
      const bool split);
  void VizLocalMap(const std::shared_ptr<hozon::mapping::LocalMap>& local_map,
                   const Eigen::Isometry3d& T_U_V);
  void VizLocation(const Eigen::Vector3d& pose, const Eigen::Quaterniond& q_W_V,
                   const double stamp);
  void VizHQMap(const std::shared_ptr<hozon::hdmap::Map>& msg);
  void VizHQMapRoad(const std::shared_ptr<hozon::hdmap::Map>& msg,
                    adsfi_proto::viz::MarkerArray* markers_road);
  void VizHQMapLane(const std::shared_ptr<hozon::hdmap::Map>& msg,
                    adsfi_proto::viz::MarkerArray* markers_lane);
  Eigen::Vector3d GcjPtToLocalEnu(const hozon::common::PointENU& point_gcj);
  void VizHQMap();
  void VizTopoMap(const std::shared_ptr<hozon::hdmap::Map>& msg);
  void VizTopoMapRoad(const std::shared_ptr<hozon::hdmap::Map>& msg,
                      adsfi_proto::viz::MarkerArray* markers_road) const;
  void VizTopoMapLane(const std::shared_ptr<hozon::hdmap::Map>& msg,
                      adsfi_proto::viz::MarkerArray* markers_lane) const;
  void VizTopoMapElements(
      const std::shared_ptr<hozon::hdmap::Map>& msg,
      adsfi_proto::viz::MarkerArray* markers_elements) const;
  static void PointsToMarker(const double stamp,
                             const std::vector<Eigen::Vector3d>& points,
                             adsfi_proto::viz::Marker* marker,
                             double color_type);
  static void LineIdToMarker(const double stamp, const Eigen::Vector3d& point,
                             const std::string& id,
                             adsfi_proto::viz::Marker* marker);
  static size_t FindNearestPointIndex(
      const Eigen::Vector2d& point,
      const std::vector<hozon::common::Point3D>& points);
  static int KnnSearchNearestPointIndex(
      const int dim, const std::vector<float>& query_points,
      const std::shared_ptr<cv::flann::Index>& kd_tree);
  static bool PerpendicularFootInSegment(const Eigen::Vector2d& p0,
                                         const Eigen::Vector2d& p1,
                                         const Eigen::Vector2d& pt);
  std::vector<Eigen::Vector2d> GetLaneLeftStartAndEndPoint(
      const hozon::hdmap::Lane& lane);
  std::vector<Eigen::Vector2d> GetLaneRightStartAndEndPoint(
      const hozon::hdmap::Lane& lane);
  static double PointDistanceToSegment(
      const std::vector<Eigen::Vector2d>& points, const Eigen::Vector2d& pt);
  bool LaneLineBelongToLane(const std::vector<std::string>& lanes,
                            const bool left, const Eigen::Vector2d& p0,
                            const Eigen::Vector2d& p1);
  void LaneLineBelongToLane(std::vector<std::string>* lanes, const bool left,
                            const Eigen::Vector2d& p0,
                            const Eigen::Vector2d& p1);
  static bool LaneBelongToLaneLine(
      const std::vector<Eigen::Vector2d>& lane_points,
      const Eigen::Vector2d& p0, const Eigen::Vector2d& p1);
  static Eigen::Vector3d GetIntersection(const hozon::common::Polygon& points);
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
