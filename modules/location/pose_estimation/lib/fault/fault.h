/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： fault.h
 *   author     ： luoning
 *   date       ： 2024.06
 ******************************************************************************/

#pragma once

#include <memory>
#include <vector>
#include <list>
#include <tuple>
#include <unordered_map>
#include <string>

#include "modules/location/pose_estimation/lib/hd_map/hd_map_base.h"
#include "modules/location/pose_estimation/lib/pose_estimate/pose_estimate_base.h"
#include "modules/location/pose_estimation/lib/pose_estimate/pose_estimate_lane_line.h"
#include "modules/location/pose_estimation/lib/util/globals.h"

namespace hozon {
namespace mp {
namespace loc {

enum class ERROR_TYPE : int {
  NO_ERROR = 0,
  NO_VALID_PECEP_LANE = 1,   // 无有效感知车道线
  ERROR_EDGE = 2,            // 路沿错误
  NO_VALID_MAP_LANE = 3,     // 无有效地图车道线-1
  NO_MAP_BOUNDARY_LINE = 4,  // 无有效地图车道线-2
  NO_MERGE_MAP_LANE = 5,     // 无有效地图车道线-3
  MAP_LANE_MATCH_FAIL = 6,   // 感知和地图车道线偏差大
  OFFSET_ONELANE = 7         // FC偏移一个车道
};

struct FaultParam{
    double left_dist_near_v;
    double left_dist_far_v;
    double right_dist_near_v;
    double right_dist_far_v;

    double left_near_check_dist_v;
    double left_far_check_dist_v;
    double right_near_check_dist_v;
    double right_far_check_dist_v;

    double left_error;
    double right_error;
    double left_check_error;
    double right_check_error;
};
struct CalWidthParam {
  double percep_near_point_y;
  double percep_far_point_y;
  double map_near_point_y;
  double map_far_point_y;
};
class MmFault {
 public:
  MmFault() : err_type_(ERROR_TYPE::NO_ERROR) {}
  void Fault(
      const std::unordered_map<std::string, std::vector<ControlPoint>>&
          merged_fcmap_lines,
      const std::unordered_map<std::string, std::vector<ControlPoint>>&
          merged_map_edges,
      const std::list<std::list<LaneLinePerceptionPtr>>& percep_lanelines,
      const ValidPose& T_fc, bool is_big_curvature_frame, bool is_ramp_road);
  void CheckIsGoodMatchFCbyLine(
      const std::unordered_map<std::string, std::vector<ControlPoint>>&
          merged_fcmap_lines,
      const std::unordered_map<std::string, std::vector<ControlPoint>>&
          merged_map_edges,
      const std::list<std::list<LaneLinePerceptionPtr>>& percep_lanelines,
      const SE3& FC_pose, const Eigen::Vector3d& FC_vel, bool is_ramp_road);
  inline ERROR_TYPE GetErrorType() { return err_type_; }
  void set_ins_ts(const double& ins_ts);
  void CalLinesMinDist(const LaneLinePerceptionPtr& percep,
                       const std::unordered_map<std::string, std::vector<V3>>&
                           filtered_fcmap_lines,
                       double* const near, double* const far,
                       const double& far_dis, const double& near_dis,
                       std::string* cur_line_id, V3* pt, CalWidthParam* param);
  double CalCulatePointToLineDistance(const V3& selected_point,
                                      const V3& line_point1,
                                      const V3& line_point2);
  bool FaultGetFitPoints(const VP& control_poins, const double x, V3* pt);
  bool GetFcFitMapPoints(const VP& control_poins, const double x, V3* pt);
  bool GetFcFitPoints(const VP& control_poins, const double x, V3* pt);
  bool FaultGetFitMapPoints(const std::vector<ControlPoint>& control_poins,
                            const double x, V3* pt);
  std::tuple<bool, bool, bool> FaultDetected(
      const FaultParam& faultParam, const std::string& map_right_check_near_id,
      const V3& percep_right_target_point,
      const std::unordered_map<std::string, std::vector<V3>>&
          right_filtered_fcmap_lines,
      const std::string& map_left_check_near_id,
      const V3& percep_left_target_point,
      const std::unordered_map<std::string, std::vector<V3>>&
          left_filtered_fcmap_lines,
      const Eigen::Vector3d& FC_vel, const double& width_diff,
      bool is_ramp_road, const double& map_width);
  double GetDistanceBySecondFaultDetected(
      const std::unordered_map<std::string, std::vector<V3>>&
          filtered_fcmap_lines,
      const std::string& map_check_near_id, const V3& percep_target_point,
      const Eigen::Vector3d& FC_vel);

 private:
  double last_ins_timestamp_ = 0.f;
  double ins_timestamp_ = 0.f;
  ERROR_TYPE err_type_;
  bool is_big_curvature_ = false;
  bool check_error_last_ = false;
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
