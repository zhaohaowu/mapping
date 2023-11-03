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

#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>

#include "map_fusion/map_service/map_proto_maker.h"
#include "util/geo.h"
#include "util/rviz_agent/rviz_agent.h"

namespace hozon {
namespace mp {
namespace mf {

struct AllLaneInfo {
  std::string id;
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
  //! 临时使用，用原始ins作为全局系定位，以后统一用OnLocalization
  void OnInsNodeInfo(
      const std::shared_ptr<hozon::localization::HafNodeInfo>& msg);
  void OnLocalMap(const std::shared_ptr<hozon::mapping::LocalMap>& msg);
  void OnLocalization(
      const std::shared_ptr<hozon::localization::Localization>& msg);
  void TopoAssign();
  std::shared_ptr<hozon::hdmap::Map> GetTopoMap();

 private:
  void OnLocationInGlobal(const Eigen::Vector3d& pos,
                          const Eigen::Quaterniond& quat, double stamp);

  std::shared_ptr<hozon::hdmap::Map> topo_map_ = nullptr;
  std::shared_ptr<hozon::mapping::LocalMap> local_map_ = nullptr;
  // gcj02 global position
  Eigen::Vector3d vehicle_pose_;
  Eigen::Vector3d ref_point_;

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

  void AppendLaneLine(const std::string& lane_id,
                      hozon::mp::mf::LaneLine* lane_line,
                      const Eigen::Vector2d& p0, const Eigen::Vector2d& p1,
                      const bool left);
  std::vector<std::string> FindLaneLineHead(const std::string& lane_id,
                                            const Eigen::Vector2d& pt,
                                            const bool left);
  std::vector<std::string> FindLaneLineTail(const std::string& lane_id,
                                            const Eigen::Vector2d& pt,
                                            const bool left);
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
  int KnnSearchNearestPointIndex(
      const int dim, const std::vector<float>& query_points,
      const std::shared_ptr<cv::flann::Index>& kd_tree);
  bool PerpendicularFootInSegment(const Eigen::Vector2d& p0,
                                  const Eigen::Vector2d& p1,
                                  const Eigen::Vector2d& pt);
  std::vector<Eigen::Vector2d> GetLaneStartAndEndPoint(
      const hozon::hdmap::Lane& lane, const bool left);
  double PointDistanceToSegment(const std::vector<Eigen::Vector2d>& points,
                                const Eigen::Vector2d& pt);
  bool LaneLineBelongToLane(const std::vector<std::string>& lanes,
                            const bool left, const Eigen::Vector2d& p0,
                            const Eigen::Vector2d& p1);
  // 可视化的class
  MapProtoMarker marker_rviz_;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
