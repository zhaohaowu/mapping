/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： pose_estimate_solver.cc
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/location/pose_estimation/lib/pose_estimate/pose_estimate_solver.h"

#include "Eigen/src/Geometry/Quaternion.h"
#include "base/utils/log.h"
#include "modules/location/pose_estimation/lib/factor/point_factor_2d/pose2d_error.h"

namespace hozon {
namespace mp {
namespace loc {

bool MapMatchSolver::solve2D(const Connect& connect,
                             const Sophus::SE3d& input_pose,
                             Sophus::SE3d* result_pose) {
  if (result_pose == nullptr) {
    HLOG_ERROR << "why empty pointer";
    return false;
  }
  int match_pairs_num = static_cast<int>(connect.lane_line_match_pairs.size());
  if (match_pairs_num < 3) {
    HLOG_ERROR << "pairs num:" << match_pairs_num;
    return false;
  }
  double x = input_pose.translation().x();
  double y = input_pose.translation().y();
  double yaw = input_pose.log().tail<3>().z();
  Eigen::Vector3d pose_plane(x, y, yaw);
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  options.minimizer_progress_to_stdout = false;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 100;
  options.logging_type = ceres::SILENT;
  ceres::Problem problem;
  for (int i = 0; i < match_pairs_num; i++) {
    const auto& mp_i = connect.lane_line_match_pairs[i];
    Eigen::Vector2d p_v(mp_i.pecep_pv.x(), mp_i.pecep_pv.y());
    Eigen::Vector2d p_w(mp_i.map_pw.x(), mp_i.map_pw.y());
    double weight = mp_i.weight;
    problem.AddResidualBlock(Pose2DError::CreateNumericDiff(p_v, p_w, weight),
                             nullptr, &x, &y, &yaw);
  }
  ceres::Solve(options, &problem, &summary);
  if (summary.num_successful_steps <= 0) {
    HLOG_ERROR << "num_successful_steps < 0";
    return false;
  }
  Eigen::Matrix3d R;
  R << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
  Eigen::Quaterniond q(R);
  *result_pose = Sophus::SE3d(q, Eigen::Vector3d(x, y, 0));
  return true;
}

}  // namespace loc
}  // namespace mp
}  // namespace hozon
