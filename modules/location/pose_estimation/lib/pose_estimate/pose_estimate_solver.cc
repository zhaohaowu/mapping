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

class CustomCallback : public ceres::IterationCallback {
 public:
  CustomCallback(double* x, double* y, double* yaw) : x_(x), y_(y), yaw_(yaw) {}
  virtual ceres::CallbackReturnType operator()(
      const ceres::IterationSummary& summary) override {
    // 打印当前状态参数
    HLOG_DEBUG << "x: " << *x_ << ", y: " << *y_ << ", yaw: " << *yaw_;
    return ceres::SOLVER_CONTINUE;
  }

 private:
  double* x_;
  double* y_;
  double* yaw_;
};

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

  auto rotation_matrix = input_pose.rotationMatrix();
  Sophus::SO3d rot(rotation_matrix);
  Eigen::Vector3d euler = RotToEuler312(rot.matrix());
  euler = euler - ((euler.array() > M_PI).cast<double>() * 2.0 * M_PI).matrix();
  double roll = euler[0];   // Roll
  double pitch = euler[1];  // Pitch
  double yaw = euler[2];    // Yaw
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  options.minimizer_progress_to_stdout = false;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 100;
  options.logging_type = ceres::SILENT;
  // options.callbacks.push_back(new CustomCallback(&x, &y, &yaw));
  ceres::Problem problem;
  for (int i = 0; i < match_pairs_num; i++) {
    const auto& mp_i = connect.lane_line_match_pairs[i];
    Eigen::Vector2d p_v(mp_i.pecep_pv.x(), mp_i.pecep_pv.y());
    Eigen::Vector2d p_w(mp_i.map_pw.x(), mp_i.map_pw.y());
    double weight = mp_i.weight;
    problem.AddResidualBlock(Pose2DError::CreateNumericDiff(p_v, p_w, x, weight),
                             nullptr, &y, &yaw);
  }
  ceres::Solve(options, &problem, &summary);
  if (summary.num_successful_steps <= 0) {
    HLOG_ERROR << "num_successful_steps < 0";
    return false;
  }
  // Eigen::Matrix3d R;
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = yawAngle * rollAngle * pitchAngle;
  *result_pose = Sophus::SE3d(q, Eigen::Vector3d(x, y, 0));
  return true;
}

}  // namespace loc
}  // namespace mp
}  // namespace hozon
