/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： path_manager.cc
 *   author     ： taoshaoyuan
 *   date       ： 2024.01
 ******************************************************************************/

#include "map_fusion/road_recognition/path_manager.h"

#include "map_fusion/fusion_common/calc_util.h"

namespace hozon {
namespace mp {
namespace mf {

void PathManager::AddPose(const KinePose& pose) {
  //! TBD 1:
  //! 上游LocalMap的频率是10Hz，车速100Km/h时两帧LocalMap间移动的间距大概2.7m，
  //! 车速30Km/h时的间距大概0.8m；即间距都不大，因此这里对于新加入的Pose先简单处理：
  //! 直接判断与前一帧间距是否太小，如果太小就不加入队列，否则就直接加入队列，但每帧pose
  //! 都还是要记录到latest_pose_里，表示车辆当前位姿。
  //! 后续考虑更精确的方法，比如间距较大时，插值出等间距的位置点及其时间戳，并且根据时间戳
  //! 球面插值出姿态。
  //! TBD 2:
  //! 对于调头、倒车场景下，车辆移动方向变化较大，当前处理方式可能不足。

  latest_pose_ = pose;

  auto p = std::make_shared<KinePose>(pose);
  if (poses_.empty()) {
    poses_.emplace_back(p);
    return;
  }

  auto last_pos = poses_.back()->pos;
  auto curr_pos = pose.pos;
  auto dist = Dist(last_pos, curr_pos);
  if (dist < conf_.interval) {
    return;
  }

  poses_.emplace_back(p);
  back_length_ += dist;

  while (back_length_ > conf_.back_range && poses_.size() >= 2) {
    auto front = poses_.at(0);
    auto next = poses_.at(1);
    auto front_dist = Dist(next->pos, front->pos);
    poses_.pop_front();
    back_length_ -= front_dist;
  }
}

void PathManager::GetPath(std::vector<KinePose::Ptr>* path,
                          float predict_range) {
  if (path == nullptr || poses_.empty()) {
    return;
  }

  for (const auto& pose : poses_) {
    auto p = std::make_shared<KinePose>(*pose);
    path->emplace_back(p);
  }

  if (predict_range <= 0) {
    return;
  }

  auto curr_vel = latest_pose_.vel;
  auto curr_ang_vel = latest_pose_.ang_vel;
  float norm_vel = curr_vel.head<2>().norm();
  Eigen::Quaternionf last_quat(1.0, 0.0, 0.0, 0.0);
  // 车辆静止时，为了能继续往前预测，设定一个常量速度，并且角速度设为0
  const float kStationaryVelThreshold = 0.5;  // 判断为静止的速度阈值
  const float kPseudoVelKmph = 30.0;          // 静止时假设的前进速度
  if (std::abs(norm_vel) <= kStationaryVelThreshold) {
    curr_vel << kPseudoVelKmph / 3.6F, 0, 0;
    norm_vel = curr_vel.head<2>().norm();
    curr_ang_vel.setZero();
  } else {
    last_quat = poses_.back()->quat;
  }

  int avg_n = std::min(static_cast<int>(path->size()), 5);
  if (avg_n < 2) {
    return;
  }
  std::vector<KinePose::Ptr> last_n_poses(path->end() - avg_n, path->end());
  float length = 0;
  for (int i = 0; i < avg_n - 1; ++i) {
    const auto& p0 = last_n_poses[i]->pos;
    const auto& p1 = last_n_poses[i + 1]->pos;
    length += Dist(p0, p1);
  }
  float avg_interval = length / static_cast<float>(avg_n - 1);

  float dt = avg_interval / norm_vel;
  int predict_counts = static_cast<int>(predict_range / avg_interval);
  auto last_stamp = poses_.back()->stamp;
  // 使用匀速匀旋转来递推位姿
  //! TBD:
  //! 当前为了简单实现，n时刻的位置都是直接按起始时刻位姿一直向前递推得到的，
  //! 这就导致递推的位置是一根直线，后期考虑n时刻位姿在n-1时刻位姿基础上进行积分递推

  for (int i = 0; i < predict_counts; ++i) {
    float ndt = static_cast<float>(i + 1) * dt;
    Eigen::Quaternionf dq_ndt = ExpQ<float>(curr_ang_vel * ndt);
    // HLOG_ERROR << "curr_ang_vel =" << curr_ang_vel.x() << "  "
    //            << curr_ang_vel.y() << " " << curr_ang_vel.z();
    Eigen::Quaternionf q_ndt = last_quat * dq_ndt;
    Eigen::Vector3f dp_ndt = curr_vel * ndt;
    // 由于这里速度是在车体系下，因此dp_ndt是车体系下的移动后的位置点，这里转换到local系下
    Eigen::Vector3f p_ndt = poses_.back()->TransformPoint(dp_ndt);
    auto pose_ndt = std::make_shared<KinePose>();
    pose_ndt->stamp = last_stamp + ndt;
    pose_ndt->pos = p_ndt;
    pose_ndt->quat = q_ndt;
    path->emplace_back(pose_ndt);
  }
}

void PathManager::GetPath(std::vector<KinePose::Ptr>* path, float predict_range,
                          const std::vector<double>& line_param) {
  if (path == nullptr || poses_.empty()) {
    return;
  }

  for (const auto& pose : poses_) {
    auto p = std::make_shared<KinePose>(*pose);
    path->emplace_back(p);
  }

  if (predict_range <= 0) {
    return;
  }

  auto curr_vel = latest_pose_.vel;
  auto curr_ang_vel = latest_pose_.ang_vel;
  float norm_vel = curr_vel.head<2>().norm();
  // 车辆静止时，为了能继续往前预测，设定一个常量速度，并且角速度设为0
  const float kStationaryVelThreshold = 0.1;  // 判断为静止的速度阈值
  const float kPseudoVelKmph = 30.0;          // 静止时假设的前进速度
  if (std::abs(norm_vel) <= kStationaryVelThreshold) {
    curr_vel << kPseudoVelKmph / 3.6F, 0, 0;
    norm_vel = curr_vel.head<2>().norm();
    curr_ang_vel.setZero();
  }

  int avg_n = std::min(static_cast<int>(path->size()), 5);
  if (avg_n < 2) {
    return;
  }
  std::vector<KinePose::Ptr> last_n_poses(path->end() - avg_n, path->end());
  float length = 0;
  for (int i = 0; i < avg_n - 1; ++i) {
    const auto& p0 = last_n_poses[i]->pos;
    const auto& p1 = last_n_poses[i + 1]->pos;
    length += Dist(p0, p1);
  }
  float avg_interval = length / static_cast<float>(avg_n - 1);
  float dt = avg_interval / norm_vel;
  int predict_counts = static_cast<int>(predict_range / avg_interval);
  auto last_quat = poses_.back()->quat;
  auto last_stamp = poses_.back()->stamp;
  auto p = poses_.back();
  // 使用匀速匀旋转来递推位姿
  //! TBD:
  //! 当前为了简单实现，n时刻的位置都是直接按起始时刻位姿一直向前递推得到的，
  //! 这就导致递推的位置是一根直线，后期考虑n时刻位姿在n-1时刻位姿基础上进行积分递推
  for (int i = 0; i < predict_counts; ++i) {
    float ndt = static_cast<float>(i + 1) * dt;
    float length_tmp = avg_interval * (i + 1);
    Eigen::Vector3f point(
        length_tmp,
        line_param[1] * length_tmp + line_param[2] * length_tmp * length_tmp +
            line_param[3] * length_tmp * length_tmp * length_tmp,
        0);
    point = point / point.norm() * length_tmp;
    Eigen::Vector3f point_p = p->TransformPoint(point);
    HLOG_ERROR << "length_tmp = " << length_tmp
               << "  point.x() = " << point.x();
    // Eigen::Quaternionf dq_ndt = ExpQ<float>(curr_ang_vel * ndt);
    // Eigen::Quaternionf q_ndt = last_quat * dq_ndt;
    Eigen::Vector3f deta_pos = point_p - poses_.back()->pos;
    Eigen::Vector3f vector_angel(0, 0, atan2(deta_pos.y(), deta_pos.x()));
    Eigen::Quaternionf dq_ndt2 = ExpQ<float>(vector_angel);
    // HLOG_ERROR<<"dq_ndt2  = "<<dq_ndt2.w()<<"  "<<dq_ndt2.x()<<"
    // "<<dq_ndt2.y()<<"   "<<dq_ndt2.z();
    auto pose_ndt = std::make_shared<KinePose>();
    pose_ndt->stamp = last_stamp + ndt;
    pose_ndt->pos = point_p;
    pose_ndt->quat = dq_ndt2;
    path->emplace_back(pose_ndt);
  }
}

KinePose::Ptr PathManager::LatestPose() {
  if (latest_pose_.stamp < 0) {
    return nullptr;
  }

  auto latest = std::make_shared<KinePose>(latest_pose_);
  return latest;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
