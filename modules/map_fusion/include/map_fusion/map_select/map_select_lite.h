/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_select_lite.h
 *   author     ： zhangrui
 *   date       ： 2024.02
 ******************************************************************************/
#pragma once
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
// #include "Eigen/Core"
// #include "Eigen/Dense"
// #include "Eigen/Geometry"
// #include "modules/map_fusion/include/map_fusion/map_fusion.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>

#include "depend/map/hdmap/hdmap.h"
#include "depend/proto/dead_reckoning/dr.pb.h"
#include "depend/proto/localization/localization.pb.h"
#include "depend/proto/map/map.pb.h"
#include "depend/proto/perception/transport_element.pb.h"
#include "modules/map_fusion/include/map_fusion/map_select/data_decider_lite.h"
#include "modules/map_fusion/include/map_fusion/map_select/double_rise_decider.h"
#include "modules/map_fusion/include/map_fusion/map_select/lane_change_observer.h"
#include "modules/map_fusion/include/map_fusion/map_select/lane_line_delay.h"
#include "modules/map_fusion/include/map_fusion/map_service/global_hd_map.h"
#include "proto/common/types.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/fsm/nnp_fct.pb.h"

namespace hozon {
namespace mp {
namespace mf {
namespace select {
struct Point2d {
  double x{0.0};
  double y{0.0};
};
struct SelectLaneLine {
  std::string id;
  std::shared_ptr<cv::flann::Index> lane_line_kdtree;  // 中心线的Kdtree
  std::vector<Eigen::Vector2d> lane_line_points;
  float min_dist_value{FLT_MAX};
  int nearest_index{0};
};
struct LaneInMapInfo {
  bool is_lane_in_map;
  std::string lane_id;
  std::string predecessor_id;
  std::string successor_id;
  hozon::hdmap::Lane lane;
};
struct DistanceInfo{
    float car_cental_line_dis;
    float half_lane_width;
};
constexpr double kDfaultLaneWidth = 1.875;
constexpr int kLaneMarkerDelaySize = 5;

struct MapSelectResult {
  hozon::navigation_hdmap::MapMsg_MapType map_type;
  bool valid;
  uint32_t fault_level;
};

class MapSelectLite {
 public:
  MapSelectLite() = default;
  ~MapSelectLite() = default;
  bool Init();
  MapSelectResult Process(
      const std::shared_ptr<hozon::localization::Localization>& localization,
      const std::shared_ptr<hozon::hdmap::Map>& fusion_map,
      const std::shared_ptr<hozon::hdmap::Map>& perc_map,
      const std::shared_ptr<const ::hozon::perception::TransportElement>&
          perception_msg,
      const std::shared_ptr<const ::hozon::dead_reckoning::DeadReckoning>&
          dr_msg,
      const std::shared_ptr<hozon::functionmanager::FunctionManagerIn>& fct_in);
  MapSelectResult Process(
      const std::shared_ptr<hozon::localization::Localization>& localization,
      const std::shared_ptr<hozon::hdmap::Map>& fusion_map,
      const std::shared_ptr<hozon::hdmap::Map>& perc_map,
      const std::shared_ptr<hozon::functionmanager::FunctionManagerIn>& fct_in);
  int32_t ArbitrateMap(
      const std::shared_ptr<hozon::localization::Localization>& localization,
      const std::shared_ptr<hozon::hdmap::Map>& fusion_map,
      const std::shared_ptr<hozon::hdmap::Map>& perc_map,
      const std::shared_ptr<const ::hozon::perception::TransportElement>&
          perception_msg,
      const std::shared_ptr<const ::hozon::dead_reckoning::DeadReckoning>&
          dr_msg,
      const std::shared_ptr<hozon::functionmanager::FunctionManagerIn>& fct_in);
  void Reset();

  enum LocErrState { Loc_Init, Shift, Err };
  hozon::navigation_hdmap::MapMsg_MapType GetMapTypeByRoadType();
  hozon::routing::RoutingResponse GetDebug() { return debug_; }

 private:
  int LocationErrDecider(
      const std::shared_ptr<hozon::localization::Localization>& localization);
  bool DeciderPercpMsg(
      const std::shared_ptr<const ::hozon::perception::TransportElement>&
          perception_msg,
      const std::shared_ptr<const ::hozon::dead_reckoning::DeadReckoning>&
          dr_msg);
  bool SelectEgoLaneInfo(
      const std::shared_ptr<const ::hozon::perception::TransportElement>&
          perception_msg);
  static bool GetSuccessorLanes(
      const std::vector<hozon::hdmap::LaneInfoConstPtr>& lanes,
      const std::string& lane_id, hozon::hdmap::LaneInfoConstPtr lane_ptr);

  bool CheckPerceMsg(
      const std::shared_ptr<const ::hozon::perception::TransportElement>&
          perception_msg);
  bool CheckLocalizationMsg(
      const std::shared_ptr<hozon::localization::Localization>& localization);
  double CalculateLanemarkerY(
      const double distance,
      const hozon::perception::LaneCubicCurve& lane_attribute);
  bool FusionMapLaneChecker(const hozon::common::PointENU& local_pos);
  bool MatchPerceLaneAndFusionLane(const hozon::common::Pose& pose,
                                   const hozon::hdmap::LaneInfoConstPtr& lane,
                                   bool match_prerequisite);
  void ResetMatchState();
  static hozon::common::math::Vec2d InterPolateVec2dPoint(
      const hozon::common::math::Vec2d& point_start,
      const hozon::common::math::Vec2d& point_end, double dis);
  static std::vector<hozon::common::math::Vec2d> InterPolateVec2dPoints(
      const std::vector<hozon::common::math::Vec2d>& map_points, double start_x,
      double end_x, double delta);
  std::tuple<double, double, double> CheckerDistance(
      const std::vector<hozon::common::math::Vec2d>& map_bound_points,
      const ::hozon::perception::LaneInfo& lane_info);
  bool GetEgoLanes(
      hozon::common::PointENU local_pos,
      std::vector<hozon::hdmap::LaneInfoConstPtr>& lanes_ptr,  // NOLINT
      hozon::hdmap::LaneInfoConstPtr& lane_ptr);               // NOLINT
  bool CalculateValidLocErrHasMap(std::pair<double, double>* width_diff);
  void LanechangeRiseDecider(const std::pair<bool, bool> perception,
                             const std::pair<bool, bool> map);
  void DealPerceptionLocErr(const std::pair<double, double>& width_diff,
                            bool is_bad_lanemarkers, bool has_no_width_diff,
                            bool not_in_main_road, bool is_match);
  void DealLocInitState(const std::pair<double, double> width_diff,
                        const bool is_bad_lanemarkers,
                        const bool has_no_width_diff,
                        const bool not_in_main_road, const bool is_match);
  void DealLocShiftState(const std::pair<double, double> width_diff,
                         const bool is_bad_lanemarkers,
                         const bool has_no_width_diff,
                         const bool not_in_main_road, const bool is_match);
  static bool IsLaneInLanes(
      const std::string& lane_id,
      const std::vector<hozon::hdmap::LaneInfoConstPtr>& lanes,
      hozon::hdmap::LaneInfoConstPtr& lane_ptr);  // NOLINT
  void RemoveDuplicates(std::vector<hozon::common::math::Vec2d>* points);
  void AddMapPoints(
      std::vector<hozon::common::math::Vec2d>* const left_map_points,
      std::vector<hozon::common::math::Vec2d>* const right_map_points,
      const hozon::hdmap::LaneInfoConstPtr& lane, const common::Pose& pose);
  hozon::common::math::Vec2d PointEarth2Bus(
      const hozon::common::math::Vec2d& point, const hozon::common::Pose& pose);
  static bool GetU32BitByIndex(const uint32_t input, const uint32_t index);
  bool CheckDrMsg(
      const std::shared_ptr<const ::hozon::dead_reckoning::DeadReckoning>&
          dr_msg);
  bool CheckFCTIN(
      const std::shared_ptr<hozon::functionmanager::FunctionManagerIn>& fct_in);
  bool CheckFusionMapMsg(const std::shared_ptr<hozon::hdmap::Map>& fusion_map);
  bool CheckPercepMapMsg(const std::shared_ptr<hozon::hdmap::Map>& percep_map);

  bool NnpSwitchOn(
      const std::shared_ptr<hozon::functionmanager::FunctionManagerIn>& fct_in);
  bool CheckFctIn(
      const std::shared_ptr<hozon::functionmanager::FunctionManagerIn>& fct_in);
  bool CheckMapMsg(
      const std::shared_ptr<hozon::hdmap::Map>& map,
      const std::shared_ptr<hozon::localization::Localization>& localization,
      bool loc_state, bool is_fusion_map);
  bool CheckGlobalLoc(
      const std::shared_ptr<hozon::localization::Localization>& loc);
  bool CheckLocalLoc(
      const std::shared_ptr<hozon::localization::Localization>& loc);
  bool PercepMapAvailable(
      const std::shared_ptr<hozon::hdmap::Map>& map,
      const std::shared_ptr<hozon::localization::Localization>& local_loc);
  bool FusionMapAvailable(
      const std::shared_ptr<hozon::hdmap::Map>& map,
      const std::shared_ptr<hozon::localization::Localization>& global_loc,
      const std::shared_ptr<hozon::localization::Localization>& local_loc);
  bool IsCarInLanes(
      const std::shared_ptr<hozon::hdmap::Map>& map,
      const std::shared_ptr<hozon::localization::Localization>& localization);
  bool CheckLane(const hozon::hdmap::Lane& lane);
  bool UpdateSelfBoundaryPoints(const std::shared_ptr<hozon::hdmap::Map>& map,
                                std::string lane_id,
                                std::vector<Eigen::Vector2d>* left_points,
                                std::vector<Eigen::Vector2d>* right_points);
  bool UpdateThresholdValue(const std::shared_ptr<hozon::hdmap::Map>& map,const std::string lane_id,
                            float* dis_value);
  LaneInMapInfo IsLaneInMap(const std::shared_ptr<hozon::hdmap::Map>& map,
                            std::string lane_id);
  bool ForkLaneCheck(const std::shared_ptr<hozon::hdmap::Map>& map);

  double left_map_c0_ = kDfaultLaneWidth;
  double right_map_c0_ = -kDfaultLaneWidth;
  int width_diff_count_ = 0;
  int width_same_count_ = 0;
  int err_state_count_ = 0;
  bool is_left_lane_change_ = false;
  bool is_right_lane_change_ = false;
  bool is_lane_change_ = false;

  bool is_location_init_{false};
  bool is_loc_pose_jump_{false};
  bool not_change_lane_{false};
  double history_pose_time_ = 0.0;
  uint localization_zone_id_ = 0;
  Point2d history_location_point_;

  DoubleRiseDecider double_left_lane_change_rise_decider_;
  DoubleRiseDecider double_right_lane_change_rise_decider_;
  DebounceModule original_location_err_debounce_{0.0, 2.0, 0.1};
  DebounceModule is_in_main_road_debounce_;
  DebounceModule left_lanemarker_quality_debounce_;
  DebounceModule right_lanemarker_quality_debounce_;
  Delay<double> left_lanemarker_a0_delay_{kLaneMarkerDelaySize};
  Delay<double> left_lanemarker_a1_delay_{kLaneMarkerDelaySize};
  Delay<double> right_lanemarker_a0_delay_{kLaneMarkerDelaySize};
  Delay<double> right_lanemarker_a1_delay_{kLaneMarkerDelaySize};
  DebounceModule is_good_lanemarker_debounce_;
  DebounceModule is_good_init_lanemarker_debounce_;

  DebounceModule left_lanemarker_viewrange_debounce_;
  DebounceModule right_lanemarker_viewrange_debounce_;
  std::pair<double, double> width_diff_history_{0.0, 0.0};
  double left_lanemarker_c1_delta_{0.0};
  double right_lanemarker_c1_delta_{0.0};

  // err state
  LocErrState per_loc_err_state_ = LocErrState::Loc_Init;
  int loc_err_state_{0};
  ::hozon::perception::LaneInfo ego_left_lane_;
  ::hozon::perception::LaneInfo ego_right_lane_;
  LaneChangeObserver per_lanechange_observer_;
  LaneChangeObserver map_lanechange_observer_;
  bool only_using_locationself_err = false;
  // 0: init_state 1: start_no_match_state 2:no_match_state
  int match_state_{0};
  int left_match_state_{0};
  int right_match_state_{0};
  int left_match_state_count_{0};
  int right_match_state_count_{0};
  int left_check_dis_count_{0};
  int right_check_dis_count_{0};
  double vehicle_spd_{0.0};
  hozon::hdmap::Road::Type current_road_type_{
      hozon::hdmap::Road::Type::Road_Type_UNKNOWN};
  double last_loc_time_{0.0};
  double last_perce_time_{0.0};
  double last_dr_time_{0.0};
  double last_fct_in_time_{0.0};
  hozon::routing::RoutingResponse debug_;
  std::map<std::string, SelectLaneLine> lanes_lines_map_;
  std::vector<hozon::common::math::Vec2d> left_boundary_points_;
  std::vector<hozon::common::math::Vec2d> right_boundary_points_;
  std::map<std::string,DistanceInfo>dis_infos_;
  float min_dist_value_;
  std::string min_dist_id_;
  int nearest_index_;
  float max_half_lane_dis_ = 3.5;
  float min_half_lane_dis_ = 1.35;
  float lane_range_value_ = 0.5;
  bool is_routing_null_ = false;
  float dis_scope_ = 5;
};
}  // namespace select
}  // namespace mf
}  // namespace mp
}  // namespace hozon
