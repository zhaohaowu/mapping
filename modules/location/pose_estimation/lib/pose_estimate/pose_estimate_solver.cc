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

// Sophus::SE3d MapMatchSolver::solve(const Connect &connect, Sophus::SE3d pose,
//                                    Sophus::SE3d pre_pose,
//                                    const Sophus::SE3d &T_V_C, bool &is_ok) {
//   Sophus::SE3d delta_pose = pre_pose.inverse() * pose;
//   double n_pole = connect._pole.size();
//   double n_traffic_sign = connect._traffic_sign.size();
//   double n_lane_line_match = connect._lane_line_match_pairs_.size();
//   double n_road_edge = connect._road_edge.size() / 2;
//   double n = n_lane_line_match;
//   if (n < 3) {
//     is_ok = false;
//     HLOG_INFO << "n_lane_line n:" << n;
//     return pose;
//   }
//   ceres::Solver::Options options;
//   ceres::Solver::Summary summary;
//   options.minimizer_progress_to_stdout = false;
//   options.linear_solver_type = ceres::DENSE_QR;
//   options.max_num_iterations = 100;

//   ceres::Problem problem;
//   problem.AddParameterBlock(pose.data(), Sophus::SE3d::num_parameters,
//                             Sophus::LocalParamsSE3::Create());
//   double sqr_pole = 1.0f;
//   for (auto &pole : connect._pole) {
//     Eigen::Vector3d pp = pose.inverse() * pole.pp;
//     Eigen::Vector3d pm = pole.pm;
//     // auto id = problem.AddResidualBlock(
//     //     Point2PonintVerticalFactor::Create(pp, pm, sqr_pole),
//     //     new ceres::HuberLoss(1.1), T_W_V.data());
//   }
//   double sqr_traffic_sign = 1.0f;
//   for (auto &ts : connect._traffic_sign) {
//     Eigen::Vector3d pp = pose.inverse() * ts.pp;
//     Eigen::Vector3d pm = ts.pm;
//     // auto id = problem.AddResidualBlock(
//     //     Point2PonintVerticalFactor::Create(pp, pm, sqr_traffic_sign),
//     //     new ceres::HuberLoss(1.1), T_W_V.data());
//   }

// if (!pre_pose.translation().isZero()) {
//   problem.AddResidualBlock(
//       DeltaFromPreFrameFactor::Create(pre_pose, delta_pose), nullptr,
//       pose.data());
// }

// int l_cnt = 0, r_cnt = 0;
// for (int i = 0; i < n_lane_line_match; i++) {
//   const auto &mp_i = connect._lane_line_match_pairs_[i];
// if (mp_i.type == LINE_TYPE::L_LINE) {
//   l_cnt++;
//   if (l_cnt < 100) {
//     HLOG_ERROR << "l mp_i.pecep_pv.transpose() :"
//               << mp_i.pecep_pv.transpose();
//     HLOG_ERROR << "l pose.inverse() * mp_i.map_pw.transpose() :"
//               << (pose.inverse() * mp_i.map_pw).transpose();
//   }
// }
// if (mp_i.type == LINE_TYPE::R_LINE) {
//   r_cnt++;
//   if (r_cnt < 100) {
//     HLOG_ERROR << "r mp_i.pecep_pv.transpose() :"
//               << mp_i.pecep_pv.transpose();
//     HLOG_ERROR << "r pose.inverse() * mp_i.map_pw.transpose() :"
//               << (pose.inverse() * mp_i.map_pw).transpose();
//   }
// }
//   problem.AddResidualBlock(Point2PointFactor::Create(mp_i), nullptr,
//                            pose.data());
// }

// double sqr_lpf = 10 * n_lane_line_match;
// auto id = problem.AddResidualBlock(SE3LPF::Create(pose, sqr_lpf), nullptr,
//                                    pose.data());
// auto start = std::chrono::steady_clock::now();
// ceres::Solve(options, &problem, &summary);
// auto end = std::chrono::steady_clock::now();
// auto duration =
//     std::chrono::duration_cast<std::chrono::microseconds>(end - start)
//         .count();
// HLOG_INFO << SETPRECISION(15)
//           << "test mm | solve_time = " << duration * 0.000001 << "s";
// HLOG_ERROR << "lat summary.num_successful_steps : "
//           << summary.num_successful_steps;
// HLOG_ERROR << "summary.final_cost : " << summary.final_cost;
// HLOG_ERROR << "summary.BriefReport : " << summary.BriefReport();
//   if (summary.num_successful_steps > 0) {
//     is_ok = true;
//   } else {
//     is_ok = false;
//   }
//   return pose;
// }

Sophus::SE3d MapMatchSolver::solve2D(const Connect &connect,
                                     const Sophus::SE3d &pose,
                                     const Sophus::SE3d &pre_pose,
                                     bool *is_ok) {
  if (is_ok == nullptr) {
    return pose;
  }
  double n_lane_line_match = connect.lane_line_match_pairs.size();
  double n = n_lane_line_match;
  if (n < 3) {
    *is_ok = false;
    HLOG_ERROR << "n_lane_line n:" << n;
    return pose;
  }

  // extract x, y, yaw
  double x = pose.translation().x();
  double y = pose.translation().y();
  double yaw = pose.log().tail<3>().z();
  // HLOG_DEBUG << "MapMatchSolver::solve2D x_y_yaw:" << SETPRECISION(15)
  //   << x << "," << y << "," << yaw;

  Eigen::Vector3d pose_plane(x, y, yaw);

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  options.minimizer_progress_to_stdout = false;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 100;

  ceres::Problem problem;

  // used only im autodiff
  // problem.AddParameterBlock(&x, 1);
  // problem.AddParameterBlock(&y, 1);
  // problem.AddParameterBlock(&yaw, 1, LocalParamsAngle::Create());
  // problem.AddParameterBlock(pose_plane.data(), 3, LocalParamsSE2::Create());

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
  HLOG_INFO << SETPRECISION(15)
            << "test mm | solve_time = " << duration * 0.000001 << "s";
  HLOG_ERROR << "summary.num_successful_steps : "
             << summary.num_successful_steps;
  HLOG_ERROR << "summary.final_cost : " << summary.final_cost;
  // HLOG_ERROR << "summary.BriefReport : " << summary.BriefReport();

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

  // HLOG_DEBUG << "inner solve2D diff yaw|y|x:" << SETPRECISION(15)
  //   << delta_3d.so3().log().z() * 180 / M_PI << ","
  //   << delta_3d.translation().y() << "," << delta_3d.translation().x()
  //   << " roll|pitch|z:" <<  delta_3d.so3().log().x() * 180 / M_PI
  //   << "," <<  delta_3d.so3().log().y() * 180 / M_PI << ","
  //   << delta_3d.translation().z();

  return pose * delta_3d;
}

}  // namespace loc
}  // namespace mp
}  // namespace hozon
