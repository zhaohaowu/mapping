/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： viz_map.h
 *   author     ： zhangshuo
 *   date       ： 2023.11
 ******************************************************************************/

#pragma once

#include <adsfi_proto/internal/node_info.pb.h>
#include <adsfi_proto/location/location.pb.h>
#include <adsfi_proto/perception/lanes.pb.h>
#include <adsfi_proto/viz/visualization_msgs.pb.h>

#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "map/hdmap/hdmap.h"
#include "map/hdmap/hdmap_common.h"
#include "map_fusion/map_prediction/viz_map.h"
#include "util/geo.h"
#include "util/rviz_agent/rviz_agent.h"

namespace hozon {
namespace mp {
namespace mf {

struct Section {
  std::string section_id;
  std::vector<std::string> lane_id;
  // std::vector<Eigen::Vector3d> left_boundary;
  // std::vector<Eigen::Vector3d> right_boundary;
  std::vector<Eigen::Vector3d> road_boundary;
};

struct LaneInfo {
  std::string lane_id;
  std::string road_id;
  std::string section_id;
  std::vector<Eigen::Vector3d> left_line;
  std::vector<Eigen::Vector3d> right_line;
  // hozon::hdmap::LaneBoundaryType_Type left_line_type;
  // hozon::hdmap::LaneBoundaryType_Type right_line_type;
  std::vector<Eigen::Vector3d> pred_left_line;   // predict left line
  std::vector<Eigen::Vector3d> pred_right_line;  // predict right line
  std::vector<std::string> left_lane_ids;
  std::vector<std::string> right_lane_ids;
  std::vector<std::string> prev_lane_ids;
  std::vector<std::string> next_lane_ids;
  std::vector<double> lane_width;
  Eigen::Vector3d last_normal;  // last two point normal
  uint32_t extra_boundary = 0;  // 1-->left,2-->right
  std::pair<Eigen::Vector3d, Eigen::Vector3d> se_point;
  double tan_theta = 0.;
  double length;  // lane_length
  Eigen::Vector3d ref_point;
};

struct RoadInfo {
  std::string road_id;
  // <section_id, Boundary>
  std::unordered_map<std::string, Section> section_ids;
};

class MapTable {
 public:
  MapTable() = default;
  ~MapTable() = default;

 public:
  void OnLocalization(
      const std::shared_ptr<hozon::localization::Localization>& msg,
      hozon::routing::RoutingResponse* routing);
  std::tuple<std::unordered_map<std::string, LaneInfo>,
             std::unordered_map<std::string, RoadInfo>>
  GetMapTable();

 private:
  void Clear();
  void OnLocationInGlobal(double utm_x, double utm_y);
  void BuildLaneTable(hozon::routing::RoutingResponse* routing);
  static void ObtainLaneAndRoad(
      const hozon::common::PointENU& utm_pos, const double& range,
      std::vector<hozon::hdmap::LaneInfoConstPtr>* lanes_in_range,
      std::vector<hozon::hdmap::RoadInfoConstPtr>* roads_in_range);
  void CreatLaneTable(
      const std::vector<hozon::hdmap::LaneInfoConstPtr>& lanes_in_range);
  static void CalculateVirtual(uint32_t* extra_boundary,
                               const hdmap::Lane& lane);
  void CreatRoadTable(
      const std::vector<hozon::hdmap::RoadInfoConstPtr>& roads_in_range);
  // void StoreLeftAndRightBoundary(const hozon::hdmap::RoadSection& it,
  // Section* section);
  void CalculateTanTheta(const std::string& idd);
  void ObtainEquation(const std::string& idd, const int& next_size);
  void ConstructLaneLine(const std::vector<Eigen::Vector3d>& road_boundary,
                         const std::vector<std::string>& sec_lane_id);
  void FitPredLaneLine(
      const std::vector<Eigen::Vector3d>& road_boundary,
      const std::vector<std::string>& sec_lane_id,
      std::vector<std::vector<Eigen::Vector3d>>* predict_lanelines);
  void FitMiddlePoint(const std::vector<Eigen::Vector3d>& road_boundary,
                      const std::vector<std::string>& sec_lane_id,
                      std::vector<std::vector<Eigen::Vector3d>>* predict_line,
                      std::vector<Eigen::Vector3d>* virtual_line,
                      std::string* virtual_id);
  void FitLastPoint(const std::vector<Eigen::Vector3d>& road_boundary,
                    const std::vector<std::string>& sec_lane_id,
                    std::vector<std::vector<Eigen::Vector3d>>* predict_line,
                    std::vector<Eigen::Vector3d>* virtual_line,
                    std::string* virtual_id);
  void LaneLastNormal(Eigen::Vector3d* AB_N, const std::string& last_id);
  void FitVirtualPoint(const std::vector<std::string>& sec_lane_id,
                       std::vector<Eigen::Vector3d>* virtual_line,
                       std::string* virtual_id, const double& s,
                       const Eigen::Vector3d& A, const Eigen::Vector3d& AB_C,
                       const int& index,
                       std::vector<std::vector<Eigen::Vector3d>>* predict_line);
  void ComputeStartWidth(const std::vector<std::string>& sec_lane_ids,
                         const std::string& idd, double* width,
                         const int& next_size);
  void ComputeEndWidth(const std::vector<std::string>& lane_idds,
                       const std::string& lane_idd_nxt, double* width_last,
                       const std::vector<std::string>& lane_idd_nxts);
  void FindLastExtraLane(std::string* lane_idd,
                         std::vector<std::string>* lane_idd_nexts);
  static void ComputeVirtualPoint(const Eigen::Vector3d& A,
                                  const Eigen::Vector3d& B,
                                  const Eigen::Vector3d& C,
                                  const Eigen::Vector3d& D,
                                  Eigen::Vector3d* virtual_point);
  static std::vector<Eigen::Vector3d> SmoothedPoint(
      const std::vector<Eigen::Vector3d>& pred_line);
  Eigen::Vector3d GcjPtToLocalEnu(const hozon::common::PointENU& point_gcj);

  std::unordered_map<std::string, LaneInfo> lane_table_;
  std::unordered_map<std::string, RoadInfo> road_table_;
  std::vector<std::string> rout_lane_id_;
  std::vector<std::string> all_section_ids_;
  std::unordered_map<std::string, std::vector<Eigen::Vector3d>>
      left_virtual_line_;
  std::vector<std::string> had_equ_;

  std::mutex mtx_;
  // gcj02
  Eigen::Vector3d location_;
  Eigen::Vector2d location_utm_;
  hozon::common::Pose init_pose_;
  Eigen::Vector3d local_enu_center_;
  bool local_enu_center_flag_ = false;
  bool init_ = false;

  VizMap viz_map_;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
