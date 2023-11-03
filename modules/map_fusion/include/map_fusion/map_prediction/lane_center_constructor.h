/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： lane_center_constructor.h
 *   author     ： zhangshuo
 *   date       ： 2023.10
 ******************************************************************************/

#pragma once
// #include "Eigen/Core"
// #include "Eigen/Dense"
// #include "Eigen/Geometry"
// #include "Eigen/src/Core/Matrix.h"

// #include "common/status/status.h"
// namespace hozon {
// namespace mp {
// namespace mf {

// using hozon::common::Status;

// class LaneCenterConstructor {
//  public:
//   LaneCenterConstructor() = default;
//   ~LaneCenterConstructor() = default;

//   int Init();

//  public:
//   Status BuildReferenceLine(  // NOLINT
//       const double central_length, const double lane_width,
//       std::vector<Eigen::Vector3d>* left_points,
//       std::vector<Eigen::Vector3d>* right_points,
//       std::vector<Eigen::Vector3d>* const central_line_pionts,
//       std::vector<Eigen::Vector3d>* left_boundary,
//       std::vector<Eigen::Vector3d>* right_boundary);

//  private:
//   Status EvaulateBoundaries(
//       const double& length,
//       std::vector<Eigen::Vector3d>* const left_input_points,
//       std::vector<Eigen::Vector3d>* const right_input_points,
//       std::vector<Eigen::Vector3d>* const left_vecs,
//       std::vector<Eigen::Vector3d>* const right_vecs,
//       const double lane_width) const;
//   static bool AddStartAndEndPoints(
//       std::vector<Eigen::Vector3d>* boundary_points, double start_x,
//       double end_x, double step_length);
//   static double CalculateVec2dLength(
//       const std::vector<Eigen::Vector3d>& input_points);
//   std::vector<Eigen::Vector3d> InterpolateVec3dPoints(
//       const std::vector<Eigen::Vector3d>& raw_points, double delta_s) const;
//   Eigen::Vector3d InterpolatePoint(const Eigen::Vector3d& start,
//                                    const Eigen::Vector3d& end, double length,
//                                    double s);
//   static void RemoveRedundantPoint(
//       std::vector<Eigen::Vector3d>* ori_left_points,
//       std::vector<Eigen::Vector3d>* ori_right_points);
//   static Status LaneWidthCheck(std::vector<Eigen::Vector3d>* left_points,
//                                std::vector<Eigen::Vector3d>* right_points,
//                                double min_lane_width,
//                                std::vector<Eigen::Vector3d>* ori_left_points,
//                                std::vector<Eigen::Vector3d>*
//                                ori_right_points, double min_length);
//   Status BuildCentralLine(const std::vector<Eigen::Vector3d>& left_line,
//                           const std::vector<Eigen::Vector3d>& right_line,
//                           const double& weight_left_boundary,
//                           const double& width,
//                           std::vector<Eigen::Vector3d>* central_pts) const;
//   static Status DoProjection(const std::vector<Eigen::Vector3d>& refV,
//                              const double& width,
//                              std::vector<Eigen::Vector3d>* results);
//   static bool FindPerpendicular(const Eigen::Vector3d& p0, const
//   Eigen::Vector3d& p1,
//                                 const double& distance, Eigen::Vector3d*
//                                 res);
// };

// }  // namespace mf
// }  // namespace mp
// }  // namespace hozon
