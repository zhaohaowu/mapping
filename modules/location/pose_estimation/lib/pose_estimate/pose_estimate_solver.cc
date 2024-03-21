/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： pose_estimate_solver.cc
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/location/pose_estimation/lib/pose_estimate/pose_estimate_solver.h"

#include "modules/location/pose_estimation/lib/factor/point_factor_2d/pose2d_error.h"

namespace hozon {
namespace mp {
namespace loc {

MapMatchSolver::MapMatchSolver() {}

Sophus::SE3d MapMatchSolver::solve2D(const Connect &connect,
                                     const Sophus::SE3d &pose,
                                     const Sophus::SE3d &pre_pose,
                                     bool *is_ok,
                                     const double &ins_timestamp) {
  if (is_ok == nullptr) {
    return pose;
  }
  auto n_lane_line_match = connect.lane_line_match_pairs.size();
  if (n_lane_line_match < 3) {
    *is_ok = false;
    HLOG_ERROR << "n_lane_line n:" << n_lane_line_match;
    return pose;
  }
  double x = pose.translation().x();
  double y = pose.translation().y();
  double yaw = pose.log().tail<3>().z();
  Eigen::Vector3d pose_plane(x, y, yaw);
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  options.minimizer_progress_to_stdout = false;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 100;
  options.logging_type = ceres::SILENT;
  ceres::Problem problem;
  for (int i = 0; i < n_lane_line_match; i++) {
    const auto &mp_i = connect.lane_line_match_pairs[i];
    Eigen::Vector2d p_v(mp_i.pecep_pv.x(), mp_i.pecep_pv.y());
    Eigen::Vector2d p_w(mp_i.map_pw.x(), mp_i.map_pw.y());
    double weight = mp_i.weight;
    problem.AddResidualBlock(Pose2DError::CreateNumericDiff(p_v, p_w, weight),
                             nullptr, &x, &y, &yaw);
  }
  auto start = std::chrono::steady_clock::now();
  ceres::Solve(options, &problem, &summary);
  auto end = std::chrono::steady_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start)
          .count();
  HLOG_DEBUG << "test mm | solve_time = " << duration * 0.000001 << "s";
  HLOG_DEBUG << "summary.num_successful_steps : "
             << summary.num_successful_steps;
  HLOG_DEBUG << "summary.final_cost : " << summary.final_cost;
  if (summary.num_successful_steps > 0) {
    *is_ok = true;
  } else {
    *is_ok = false;
  }
  double ori_yaw = pose.log().tail<3>().z(), ori_x = pose.translation().x(),
         ori_y = pose.translation().y();
  Sophus::SE2d ori_pose_2d(Sophus::SO2d::exp(ori_yaw),
                           Eigen::Vector2d(ori_x, ori_y));
  Sophus::SE2d new_pose_2d(Sophus::SO2d::exp(yaw), Eigen::Vector2d(x, y));
  Sophus::SE2d delta_2d = ori_pose_2d.inverse() * new_pose_2d;
  Sophus::SE3d delta_3d(
      Sophus::SO3d::exp(Eigen::Vector3d(0, 0, delta_2d.so2().log())),
      Eigen::Vector3d(delta_2d.translation()(0), delta_2d.translation()(1), 0));
  return pose * delta_3d;
}

}  // namespace loc
}  // namespace mp
}  // namespace hozon
