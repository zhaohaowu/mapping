/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： pose_estimation.cc
 *   author     ： nihongjie
 *   date       ： 2024.04
 ******************************************************************************/
#include "modules/location/pose_estimation/lib/pose_estimation.h"

#include <algorithm>
#include <unordered_set>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Geometry/Transform.h"
#include "Sophus/se3.hpp"
#include "base/utils/log.h"
#include "modules/location/pose_estimation/lib/reloc/reloc_rviz.hpp"
#include "modules/util/include/util/rviz_agent/rviz_agent.h"
#include "modules/util/include/util/tic_toc.h"

namespace hozon {
namespace mp {
namespace loc {
namespace pe {

PoseEstimation::PoseEstimation() : proc_thread_run_(true) {
  proc_thread_ = std::thread(&PoseEstimation::ProcData, this);
  map_matching_ = std::make_unique<MapMatching>();
  reloc_ = std::make_unique<Reloc>();
}

PoseEstimation::~PoseEstimation() {
  proc_thread_run_ = false;
  {
    std::unique_lock<std::mutex> lock(cv_mutex_);
    cv_.notify_one();
  }
  if (proc_thread_.joinable()) {
    proc_thread_.join();
  }
  if (use_rviz_ && RVIZ_AGENT.Ok() && rviz_thread_.joinable()) {
    stop_rviz_thread_ = true;
    rviz_thread_.join();
  }
}

bool PoseEstimation::LoadParams(const std::string& configfile) {
  boost::filesystem::path path(configfile);
  if (!boost::filesystem::exists(path)) {
    HLOG_ERROR << "location pose_estimation conf:" << configfile
               << " not exist";
    return false;
  }
  YAML::Node node = YAML::LoadFile(configfile);
  ins_deque_max_size_ = node["ins_deque_max_size"].as<int>();
  perception_deque_max_size_ = node["perception_deque_max_size"].as<int>();
  fc_deque_max_size_ = node["fc_deque_max_size"].as<int>();
  use_rviz_ = node["use_rviz"].as<bool>();
  rviz_addr_ = node["rviz_addr"].as<std::string>();
  reloc_cnt_ = node["reloc_cnt"].as<int>();
  reloc_test_ = node["reloc_test"].as<bool>();
  return true;
}

bool PoseEstimation::Init(const std::string& pose_estimation_yaml,
                          const std::string& map_matching_yaml) {
  if (!LoadParams(pose_estimation_yaml)) {
    return false;
  }
  if (use_rviz_) {
    HLOG_DEBUG << "Start RvizAgent!!!";
    int ret = RVIZ_AGENT.Init(rviz_addr_);
    if (ret < 0) {
      HLOG_ERROR << "RvizAgent start failed";
    }
  }
  if (use_rviz_ && RVIZ_AGENT.Ok()) {
    stop_rviz_thread_ = false;
    rviz_thread_ = std::thread(&PoseEstimation::RvizFunc, this);
  }
  if (!map_matching_) {
    return false;
  }
  return map_matching_->Init(map_matching_yaml);
}

void PoseEstimation::OnLocation(
    const std::shared_ptr<const Localization>& msg) {
  if (!msg) {
    HLOG_WARN << "fc input is nullptr";
    return;
  }

  // 判断fc数据是否有nan值
  if (std::isnan(msg->pose().gcj02().x()) ||
      std::isnan(msg->pose().gcj02().y()) ||
      std::isnan(msg->pose().gcj02().z()) ||
      std::isnan(msg->pose().quaternion().w()) ||
      std::isnan(msg->pose().quaternion().x()) ||
      std::isnan(msg->pose().quaternion().y()) ||
      std::isnan(msg->pose().quaternion().z())) {
    HLOG_ERROR << "gcj02 or quaternion is nan";
    return;
  }
  if (msg->location_state() == 0) {
    HLOG_ERROR << "Global localization is error";
    return;
  }

  // 判断enu系下的车辆fc姿态是否异常
  Eigen::Quaternionf enu_quaternion(
      msg->pose().quaternion().w(), msg->pose().quaternion().x(),
      msg->pose().quaternion().y(), msg->pose().quaternion().z());
  if (enu_quaternion.norm() < 1e-6) {
    HLOG_ERROR << "enu quaternion from fc is abnormal";
    return;
  }
  enu_quaternion.normalize();

  // fc数据构建并存放至队列
  fc_deque_mutex_.lock();
  fc_deque_.emplace_back(*msg);
  ShrinkQueue(&fc_deque_, fc_deque_max_size_);
  fc_deque_mutex_.unlock();

  // rviz可视化
  if (use_rviz_ && RVIZ_AGENT.Ok()) {
    Eigen::Vector3d gcj_position{msg->pose().gcj02().x(),
                                 msg->pose().gcj02().y(),
                                 msg->pose().gcj02().z()};
    ref_point_mutex_.lock();
    Eigen::Vector3d enu_position =
        hozon::mp::util::Geo::Gcj02ToEnu(gcj_position, ref_point_);
    ref_point_mutex_.unlock();
    Eigen::Vector3d t_fc(enu_position.x(), enu_position.y(), enu_position.z());
    Eigen::Quaterniond q_fc(enu_quaternion.w(), enu_quaternion.x(),
                            enu_quaternion.y(), enu_quaternion.z());
    Eigen::Affine3d T_fc = Eigen::Translation3d(t_fc) * Eigen::Affine3d(q_fc);
    rviz_mutex_.lock();
    T_fc_ = T_fc;
    rviz_mutex_.unlock();
  }
}

void PoseEstimation::OnIns(const std::shared_ptr<const HafNodeInfo>& msg) {
  if (!msg) {
    HLOG_WARN << "ins input is nullptr";
    return;
  }

  // 判断ins数据是否有nan值
  if (std::isnan(msg->pos_gcj02().x()) || std::isnan(msg->pos_gcj02().y()) ||
      std::isnan(msg->pos_gcj02().z()) || std::isnan(msg->quaternion().w()) ||
      std::isnan(msg->quaternion().x()) || std::isnan(msg->quaternion().y()) ||
      std::isnan(msg->quaternion().z())) {
    HLOG_ERROR << "ins gcj02 or quaternion is nan";
    return;
  }

  // 判断enu系下的车辆ins姿态是否异常
  Eigen::Quaternionf enu_quaternion(
      msg->quaternion().w(), msg->quaternion().x(), msg->quaternion().y(),
      msg->quaternion().z());
  if (enu_quaternion.norm() < 1e-6) {
    HLOG_ERROR << "enu quaternion from ins is abnormal";
    return;
  }
  enu_quaternion.normalize();

  // refpoint初始赋值
  Eigen::Vector3d gcj_position{msg->pos_gcj02().x(), msg->pos_gcj02().y(),
                               msg->pos_gcj02().z()};
  static bool ins_init = false;
  ref_point_mutex_.lock();
  if (!ins_init) {
    ins_init = true;
    ref_point_ = gcj_position;
  }
  // enu系下的车辆运动超过20000米，重置refpoint
  Eigen::Vector3d enu_position =
      hozon::mp::util::Geo::Gcj02ToEnu(gcj_position, ref_point_);
  if (enu_position.head<2>().norm() > 20000) {
    ref_point_ = gcj_position;
  }
  ref_point_mutex_.unlock();

  // ins数据构建并存放至队列
  Localization ins_msg;
  ins_msg.set_rtk_status(msg->gps_status());
  ins_msg.mutable_header()->set_data_stamp(msg->header().data_stamp());
  ins_msg.mutable_pose()->set_heading(msg->heading());
  SetXYZ(msg->pos_gcj02(), ins_msg.mutable_pose()->mutable_gcj02());
  SetXYZ(msg->linear_velocity(),
         ins_msg.mutable_pose()->mutable_linear_velocity());
  SetXYZ(msg->sd_position(),
         ins_msg.mutable_pose()->mutable_linear_acceleration());
  SetXYZW(enu_quaternion, ins_msg.mutable_pose()->mutable_quaternion());
  ins_deque_mutex_.lock();
  ins_deque_.emplace_back(ins_msg);
  ShrinkQueue(&ins_deque_, ins_deque_max_size_);
  ins_deque_mutex_.unlock();

  // rviz可视化
  if (RVIZ_AGENT.Ok() && use_rviz_) {
    Eigen::Vector3d t_ins(enu_position.x(), enu_position.y(), enu_position.z());
    Eigen::Quaterniond q_ins(enu_quaternion.w(), enu_quaternion.x(),
                             enu_quaternion.y(), enu_quaternion.z());
    Eigen::Affine3d T_ins =
        Eigen::Translation3d(t_ins) * Eigen::Affine3d(q_ins);
    rviz_mutex_.lock();
    T_ins_ = T_ins;
    rviz_mutex_.unlock();
  }
}

void PoseEstimation::OnPerception(
    const std::shared_ptr<const TransportElement>& msg) {
  if (!msg) {
    return;
  }
  perception_mutex_.lock();
  perception_ = *msg;
  perception_mutex_.unlock();
  {
    std::unique_lock<std::mutex> lock(cv_mutex_);
    cv_.notify_one();
  }
}

void PoseEstimation::ProcData() {
  pthread_setname_np(pthread_self(), "mp_loc_proc");
  while (proc_thread_run_) {
    {
      std::unique_lock<std::mutex> lock(cv_mutex_);
      cv_.wait(lock);
    }
    HLOG_INFO << "pose_estimation proc_data thread heartbeat";
    util::TicToc tic;
    // 获取感知数据
    perception_mutex_.lock();
    TransportElement perception = perception_;
    perception_mutex_.unlock();
    static double last_per_timestamp = perception.header().data_stamp();
    if (perception.header().data_stamp() - last_per_timestamp < 1e-3) {
      HLOG_ERROR << "perception data repeat";
      continue;
    }
    last_per_timestamp = perception.header().data_stamp();
    // 获取refpoint
    ref_point_mutex_.lock();
    auto ref_point = ref_point_;
    ref_point_mutex_.unlock();
    // 获取同步后的fc定位数据
    LocalizationPtr fc_pose_ptr =
        GetFcPoseForTimestamp(ref_point, perception.header().data_stamp());
    if (!fc_pose_ptr) {
      HLOG_ERROR << "GetFcPoseForTimestamp Failed";
      continue;
    }
    // 获取同步后的ins定位数据
    LocalizationPtr ins_pose_ptr = GetInsPoseForTimestamp(
        ref_point, perception.header().data_stamp());  // ins
    if (!ins_pose_ptr) {
      HLOG_ERROR << "GetInsPoseForTimestamp Failed";
      continue;
    }
    // ins标准差赋值、定位状态、ins状态、线速度赋值
    sd_position_ = std::max(ins_pose_ptr->pose().linear_acceleration().x(),
                            ins_pose_ptr->pose().linear_acceleration().y());
    location_state_ = static_cast<int>(fc_pose_ptr->location_state());
    ins_state_ = static_cast<int>(fc_pose_ptr->rtk_status());
    velocity_ = fc_pose_ptr->pose().linear_velocity_vrf().x();
    ins_height_ = ins_pose_ptr->pose().gcj02().z();
    // 获取高精地图
    std::vector<LaneInfoPtr> hdmap_lanes;
    if (!GetHdMapLane(fc_pose_ptr, &hdmap_lanes)) {
      HLOG_ERROR << "GetHdMapLane Failed";
      continue;
    }
    // 感知转换
    TrackingManager tracking_manager;
    if (!PercepConvert(perception, &tracking_manager)) {
      HLOG_ERROR << "percep transform failed";
      continue;
    }
    // 地图转换,enu系下的点
    MappingManager map_manager;
    if (!MapConvert(*fc_pose_ptr, ref_point, hdmap_lanes, &map_manager)) {
      HLOG_ERROR << "map transform failed";
      continue;
    }
    if (use_rviz_ && RVIZ_AGENT.Ok()) {
      rviz_mutex_.lock();
      tracking_manager_ = tracking_manager;
      map_manager_ = map_manager;
      rviz_mutex_.unlock();
    }
    bool ins_good = ins_state_ == 4 && sd_position_ <= 0.1;
    bool location_good = location_state_ == 2;
    if (((!location_good && !ins_good) || reloc_flag_) && reloc_test_) {
      reloc_->ResetStep(std::max(12.0, static_cast<double>(sd_position_)));
      if (!reloc_->ProcData(use_rviz_, fc_pose_ptr,
                            std::make_shared<TrackingManager>(tracking_manager),
                            std::make_shared<MappingManager>(map_manager))) {
        HLOG_ERROR << "Reloc Failed";
        continue;
      }
    }
    // 待求解输入位姿
    Sophus::SE3d T_input;
    SetInputPose(fc_pose_ptr, ins_pose_ptr, &T_input);
    // mm模块
    map_matching_->ProcData(use_rviz_, T_input, fc_pose_ptr, perception,
                            hdmap_lanes, ref_point, ins_height_);
    HLOG_DEBUG << "sum " << tic.Toc() << " ms";
  }
}

void PoseEstimation::SetInputPose(const LocalizationPtr& fc_pose_ptr,
                                  const LocalizationPtr& ins_pose_ptr,
                                  Sophus::SE3d* T_input) {
  Eigen::Affine3d T_w_v = Eigen::Affine3d(Eigen::Matrix4d::Identity());
  // fc位姿
  Eigen::Quaterniond q_fc(fc_pose_ptr->pose().quaternion().w(),
                          fc_pose_ptr->pose().quaternion().x(),
                          fc_pose_ptr->pose().quaternion().y(),
                          fc_pose_ptr->pose().quaternion().z());
  Eigen::Vector3d t_fc(fc_pose_ptr->pose().position().x(),
                       fc_pose_ptr->pose().position().y(),
                       fc_pose_ptr->pose().position().z());
  Eigen::Affine3d T_fc = Eigen::Translation3d(t_fc) * Eigen::Affine3d(q_fc);
  // ins位姿
  Eigen::Quaterniond q_ins(ins_pose_ptr->pose().quaternion().w(),
                           ins_pose_ptr->pose().quaternion().x(),
                           ins_pose_ptr->pose().quaternion().y(),
                           ins_pose_ptr->pose().quaternion().z());
  Eigen::Vector3d t_ins(ins_pose_ptr->pose().position().x(),
                        ins_pose_ptr->pose().position().y(),
                        ins_pose_ptr->pose().position().z());
  Eigen::Affine3d T_ins = Eigen::Translation3d(t_ins) * Eigen::Affine3d(q_ins);
  if (ins_state_ == 4 && sd_position_ < 0.1 && !reloc_flag_) {
    T_w_v = T_ins;
  } else if ((location_state_ == 2 && !reloc_flag_) || (!reloc_test_)) {
    Eigen::Affine3d T_diff = T_ins.inverse() * T_fc;
    Eigen::Matrix4d T_tmp = Eigen::Matrix4d::Identity();
    Eigen::Affine3d T_new_diff(T_tmp);
    T_new_diff.translation().y() = T_diff.translation().y();
    T_w_v = T_ins * T_new_diff;
  } else {
    static int reloc_cnt = 0;
    if (reloc_cnt < reloc_cnt_) {
      Eigen::Vector3d linear_velocity{
          ins_pose_ptr->pose().linear_velocity().x(),
          ins_pose_ptr->pose().linear_velocity().y(),
          ins_pose_ptr->pose().linear_velocity().z()};
      if (linear_velocity.norm() > 1) {
        ++reloc_cnt;
      }
      reloc_flag_ = true;
    } else {
      reloc_cnt = 0;
      reloc_flag_ = false;
    }
    reloc_->GetRelocPose(&T_w_v);
  }
  if (use_rviz_ && RVIZ_AGENT.Ok()) {
    rviz_mutex_.lock();
    T_input_ = T_w_v;
    rviz_mutex_.unlock();
  }
  Eigen::Vector3d t_input = T_w_v.translation();
  Eigen::Quaterniond q_input(T_w_v.rotation());
  *T_input = Sophus::SE3d(q_input, t_input);
}

template <typename T>
void PoseEstimation::ShrinkQueue(T* deque, int maxsize) {
  if (!deque) {
    return;
  }
  while (static_cast<int>(deque->size()) > maxsize) {
    deque->pop_front();
  }
}

void PoseEstimation::Gcj2Enu(const Eigen::Vector3d& ref_point,
                             Localization* msg) {
  Eigen::Vector3d gcj_position{msg->pose().gcj02().x(), msg->pose().gcj02().y(),
                               msg->pose().gcj02().z()};
  Eigen::Vector3d enu_position =
      hozon::mp::util::Geo::Gcj02ToEnu(gcj_position, ref_point);
  msg->mutable_pose()->mutable_position()->set_x(enu_position.x());
  msg->mutable_pose()->mutable_position()->set_y(enu_position.y());
  msg->mutable_pose()->mutable_position()->set_z(enu_position.z());
}

LocalizationPtr PoseEstimation::GetFcPoseForTimestamp(
    const Eigen::Vector3d& ref_point, double timestamp) {
  LocalizationPtr fc_pose_ptr = std::make_shared<Localization>();
  fc_deque_mutex_.lock();
  auto fc_deque = fc_deque_;
  fc_deque_mutex_.unlock();
  if (fc_deque.empty()) {
    HLOG_ERROR << "fc_deque is empty!";
    return nullptr;
  }
  // | __ fc队列的时间在感知时间前面很远
  if (fc_deque.front().header().data_stamp() > timestamp + 0.01) {
    HLOG_ERROR << "fc is too new!";
    HLOG_ERROR << "fc_deque.back().header().data_stamp() "
               << fc_deque.back().header().data_stamp() << "percep timestamp "
               << timestamp;
    return nullptr;
  }
  // __ | fc队列的时间在感知时间后面很远
  if (fc_deque.back().header().data_stamp() < timestamp + 0.01) {
    HLOG_ERROR << "fc is too old!";
    HLOG_ERROR << "fc_deque.back().header().data_stamp() "
               << fc_deque.back().header().data_stamp() << ",percep timestamp "
               << timestamp;
    return nullptr;
  }
  std::deque<Localization> tmp_deque;
  while (fc_deque.front().header().data_stamp() < timestamp) {
    tmp_deque.push_back(fc_deque.front());
    fc_deque.pop_front();
    if (fc_deque.empty()) {
      break;
    }
  }
  if (tmp_deque.empty() && fc_deque.empty()) {
    return nullptr;
  }
  // |__ fc队列的时间在感知时间前面不远处，取第一帧fc，TODO速度插值
  if (tmp_deque.empty()) {
    *fc_pose_ptr = fc_deque.front();
    Gcj2Enu(ref_point, fc_pose_ptr.get());
    return fc_pose_ptr;
  }
  // __| fc队列的时间在感知时间后面不远处，取最后一帧fc，TODO速度插值
  if (fc_deque.empty()) {
    *fc_pose_ptr = tmp_deque.back();
    Gcj2Enu(ref_point, fc_pose_ptr.get());
    return fc_pose_ptr;
  }
  // _|_ fc队列的时间包含感知时间，取相邻的两帧fc插值
  *fc_pose_ptr = tmp_deque.back();
  auto front = tmp_deque.back();
  Gcj2Enu(ref_point, &front);
  auto back = fc_deque.front();
  Gcj2Enu(ref_point, &back);
  double front_scale =
      (back.header().data_stamp() - timestamp) /
      (back.header().data_stamp() - front.header().data_stamp());
  double back_scale =
      (timestamp - front.header().data_stamp()) /
      (back.header().data_stamp() - front.header().data_stamp());
  // 全局定位插值
  Vector3dToEnu3d(front.pose().position(), back.pose().position(),
                  fc_pose_ptr->mutable_pose()->mutable_position(), front_scale,
                  back_scale);
  Vector4dToEnu4d(front.pose().quaternion(), back.pose().quaternion(),
                  fc_pose_ptr->mutable_pose()->mutable_quaternion(),
                  front_scale, back_scale);
  // 速度插值
  Vector3dToEnu3d(front.pose().linear_velocity_vrf(),
                  back.pose().linear_velocity_vrf(),
                  fc_pose_ptr->mutable_pose()->mutable_linear_velocity_vrf(),
                  front_scale, back_scale);
  // 局部定位插值
  Vector3dToEnu3d(front.pose_local().position(), back.pose_local().position(),
                  fc_pose_ptr->mutable_pose_local()->mutable_position(),
                  front_scale, back_scale);
  Vector4dToEnu4d(front.pose_local().quaternion(),
                  back.pose_local().quaternion(),
                  fc_pose_ptr->mutable_pose_local()->mutable_quaternion(),
                  front_scale, back_scale);
  return fc_pose_ptr;
}

LocalizationPtr PoseEstimation::GetInsPoseForTimestamp(
    const Eigen::Vector3d& ref_point, double timestamp) {
  LocalizationPtr ins_pose_ptr = std::make_shared<Localization>();
  ins_deque_mutex_.lock();
  auto ins_deque = ins_deque_;
  ins_deque_mutex_.unlock();
  if (ins_deque.empty()) {
    HLOG_ERROR << "ins_deque is empty!";
    return nullptr;
  }
  // | __ ins队列的时间在感知时间前面很远
  if (ins_deque.front().header().data_stamp() > timestamp + 0.01) {
    HLOG_ERROR << "ins is too new!";
    return nullptr;
  }
  // __ | ins队列的时间在感知时间后面很远
  if (ins_deque.back().header().data_stamp() < timestamp + 0.01) {
    HLOG_ERROR << "ins is too old!";
    return nullptr;
  }
  std::deque<Localization> tmp_deque;
  while (ins_deque.front().header().data_stamp() < timestamp) {
    tmp_deque.push_back(ins_deque.front());
    ins_deque.pop_front();
    if (ins_deque.empty()) {
      break;
    }
  }
  if (tmp_deque.empty() && ins_deque.empty()) {
    return nullptr;
  }
  // |__ ins队列的时间在感知时间前面不远处，取第一帧ins，TODO速度插值
  if (tmp_deque.empty()) {
    *ins_pose_ptr = ins_deque.front();
    Gcj2Enu(ref_point, ins_pose_ptr.get());
    return ins_pose_ptr;
  }
  // __| ins队列的时间在感知时间后面不远处，取最后一帧ins，TODO速度插值
  if (ins_deque.empty()) {
    *ins_pose_ptr = tmp_deque.back();
    Gcj2Enu(ref_point, ins_pose_ptr.get());
    return ins_pose_ptr;
  }
  // _|_ ins队列的时间包含感知时间，取相邻的两帧ins插值
  *ins_pose_ptr = tmp_deque.back();
  auto front = tmp_deque.back();
  Gcj2Enu(ref_point, &front);
  auto back = ins_deque.front();
  Gcj2Enu(ref_point, &back);
  double front_scale =
      (back.header().data_stamp() - timestamp) /
      (back.header().data_stamp() - front.header().data_stamp());
  double back_scale =
      (timestamp - front.header().data_stamp()) /
      (back.header().data_stamp() - front.header().data_stamp());
  // 全局定位插值
  Vector3dToEnu3d(front.pose().position(), back.pose().position(),
                  ins_pose_ptr->mutable_pose()->mutable_position(), front_scale,
                  back_scale);
  Vector4dToEnu4d(front.pose().quaternion(), back.pose().quaternion(),
                  ins_pose_ptr->mutable_pose()->mutable_quaternion(),
                  front_scale, back_scale);
  return ins_pose_ptr;
}

bool PoseEstimation::GetHdMapLane(const LocalizationPtr& fc_pose_ptr,
                                  std::vector<LaneInfoPtr>* hdmap_lanes) {
  // 计算车辆的utm坐标
  double x = fc_pose_ptr->pose().gcj02().x();
  double y = fc_pose_ptr->pose().gcj02().y();
  hozon::common::coordinate_convertor::GCS2UTM(51, &y, &x);
  hozon::common::PointENU utm_position;
  utm_position.set_x(y);
  utm_position.set_y(x);
  utm_position.set_z(0);
  // fc的heading和ins_fusion的坐标系都是北东地方向（heading范围为0到360度），utm的坐标系为东北天方向（heading范围为-180到180度）
  // GetLanesWithHeading函数需要传入的参数为utm坐标系下的heading，因此下面6行表示将ins_fusion坐标系的heading转成utm坐标系的heading
  double heading = -fc_pose_ptr->pose().heading() * M_PI / 180.0 + M_PI / 2;
  if (heading > M_PI) {
    heading -= 2 * M_PI;
  } else if (heading < -M_PI) {
    heading += 2 * M_PI;
  }
  GLOBAL_HD_MAP->GetLanesWithHeading(utm_position, 150, heading, M_PI_4,
                                     hdmap_lanes);
  if (hdmap_lanes->empty()) {
    HLOG_ERROR << "Get HdMap Failed";
    HLOG_ERROR << "x " << fc_pose_ptr->pose().gcj02().x() << " y "
               << fc_pose_ptr->pose().gcj02().y() << " heading " << heading;
    return false;
  }
  return true;
}

std::shared_ptr<HafNodeInfo> PoseEstimation::GetMmNodeInfo() {
  return map_matching_->getMmNodeInfo();
}

template <typename T0, typename T1, typename T2>
void PoseEstimation::Vector3dToEnu3d(const T0& front_vector,
                                     const T1& back_vector, T2* const enu3d,
                                     const double& front_scale,
                                     const double& back_scale) {
  enu3d->set_x(front_vector.x() * static_cast<float>(front_scale) +
               back_vector.x() * static_cast<float>(back_scale));
  enu3d->set_y(front_vector.y() * static_cast<float>(front_scale) +
               back_vector.y() * static_cast<float>(back_scale));
  enu3d->set_z(front_vector.z() * static_cast<float>(front_scale) +
               back_vector.z() * static_cast<float>(back_scale));
}

template <typename T0, typename T1, typename T2>
void PoseEstimation::Vector4dToEnu4d(const T0& front_vector,
                                     const T1& back_vector, T2* const enu3d,
                                     const double& front_scale,
                                     const double& back_scale) {
  enu3d->set_z(front_vector.z() * static_cast<float>(front_scale) +
               back_vector.z() * static_cast<float>(back_scale));
  enu3d->set_x(front_vector.x() * static_cast<float>(front_scale) +
               back_vector.x() * static_cast<float>(back_scale));
  enu3d->set_y(front_vector.y() * static_cast<float>(front_scale) +
               back_vector.y() * static_cast<float>(back_scale));
  enu3d->set_z(front_vector.z() * static_cast<float>(front_scale) +
               back_vector.z() * static_cast<float>(back_scale));
}

template <typename T0, typename T1>
void PoseEstimation::SetXYZ(const T0& t0, T1* const t1) {
  t1->set_x(t0.x());
  t1->set_y(t0.y());
  t1->set_z(t0.z());
}

template <typename T0, typename T1>
void PoseEstimation::SetXYZW(const T0& t0, T1* const t1) {
  t1->set_w(t0.w());
  t1->set_x(t0.x());
  t1->set_y(t0.y());
  t1->set_z(t0.z());
}

bool PoseEstimation::CheckLocalizationState(const Localization& localization) {
  return !(std::isnan(localization.pose().quaternion().w()) ||
           std::isnan(localization.pose().quaternion().x()) ||
           std::isnan(localization.pose().quaternion().y()) ||
           std::isnan(localization.pose().quaternion().z()));
}

void PoseEstimation::AddMapLine(const Eigen::Affine3d& T_V_W,
                                const Eigen::Vector3d& ref_point,
                                const hozon::hdmap::LaneBoundary& lane_boundary,
                                MappingManager* map_manager) {
  if (map_manager == nullptr) {
    return;
  }
  if (lane_boundary.id().empty()) {
    HLOG_WARN << "lane_boundary id is empty";
    return;
  }
  if (lane_boundary.virtual_()) {
    HLOG_DEBUG << "lane_boundary is virtual";
    return;
  }
  // TODO(zhw): 后续改为long long
  int lane_boundary_id = std::stoi(lane_boundary.id(0).id());
  if (map_manager->lane_lines.count(lane_boundary_id) != 0) {
    HLOG_DEBUG << lane_boundary_id << " this lane_boundary_id had added";
    return;
  }

  auto& every_lane_line = map_manager->lane_lines[lane_boundary_id];
  for (const auto& curve_segment : lane_boundary.curve().segment()) {
    for (const auto& p : curve_segment.line_segment().original_point()) {
      Eigen::Vector3d p_gcj(p.y(), p.x(), 0);
      Eigen::Vector3d p_enu = util::Geo::Gcj02ToEnu(p_gcj, ref_point);
      Eigen::Vector3d p_vehicle = T_V_W * p_enu;
      every_lane_line.points.emplace_back(p_vehicle.x(), p_vehicle.y(),
                                          p_vehicle.z());
      if (lane_boundary.boundary_type().empty() ||
          (!lane_boundary.boundary_type().empty() &&
           lane_boundary.boundary_type()[0].types().empty())) {
        HLOG_WARN << "lane boundary type is empty";
        continue;
      }
      if (lane_boundary.boundary_type()[0].types()[0] ==
              hozon::hdmap::LaneBoundaryType::Type::
                  LaneBoundaryType_Type_SOLID_WHITE ||
          lane_boundary.boundary_type()[0].types()[0] ==
              hozon::hdmap::LaneBoundaryType::Type::
                  LaneBoundaryType_Type_SOLID_YELLOW ||
          lane_boundary.boundary_type()[0].types()[0] ==
              hozon::hdmap::LaneBoundaryType::Type::
                  LaneBoundaryType_Type_DOUBLE_YELLOW) {
        every_lane_line.lane_type = LaneType::SOLID_LINE;
      } else if (lane_boundary.boundary_type()[0].types()[0] ==
                     hozon::hdmap::LaneBoundaryType::Type::
                         LaneBoundaryType_Type_DOTTED_WHITE ||
                 lane_boundary.boundary_type()[0].types()[0] ==
                     hozon::hdmap::LaneBoundaryType::Type::
                         LaneBoundaryType_Type_DOTTED_YELLOW) {
        every_lane_line.lane_type = LaneType::DASHED_LINE;
      } else {
        every_lane_line.lane_type = LaneType::Road_Edge;
      }
    }
  }
}

bool PoseEstimation::PercepConvert(const TransportElement& perception,
                                   TrackingManager* tracking_manager) {
  if (perception.lane().empty() && perception.road_edges().empty()) {
    HLOG_ERROR << "perception: both laneline and roadedge is empty";
    return false;
  }
  tracking_manager->timestamp = perception.header().data_stamp();
  auto& lane_lines = tracking_manager->lane_lines;
  int per_id = 0;
  for (const auto& lane_line : perception.lane()) {
    auto& every_lane_line = lane_lines[per_id++];
    for (const auto& point : lane_line.points()) {
      if (point.x() > 40) {
        continue;
      }
      every_lane_line.points.emplace_back(point.x(), point.y(), point.z());
      if (lane_line.lanetype() == perception::SolidLine ||
          lane_line.lanetype() == perception::DoubleSolidLine ||
          lane_line.lanetype() == perception::FishBoneLine) {
        every_lane_line.lane_type = LaneType::SOLID_LINE;
      } else if (lane_line.lanetype() == perception::DashedLine ||
                 lane_line.lanetype() == perception::DoubleDashedLine ||
                 lane_line.lanetype() == perception::FishBoneDashedLine ||
                 lane_line.lanetype() == perception::LeftSolidRightDashed ||
                 lane_line.lanetype() == perception::RightSolidLeftDashed) {
        every_lane_line.lane_type = LaneType::DASHED_LINE;
      } else {
        every_lane_line.lane_type = LaneType::UNKNOWN;
      }
    }
  }
  for (const auto& road_edge : perception.road_edges()) {
    auto& every_lane_line = lane_lines[per_id++];
    for (const auto& point : road_edge.points()) {
      if (point.x() > 40) {
        continue;
      }
      every_lane_line.lane_type = LaneType::Road_Edge;
      every_lane_line.points.emplace_back(point.x(), point.y(), point.z());
    }
  }
  return true;
}

bool PoseEstimation::MapConvert(
    const Localization& localization, const Eigen::Vector3d& ref_point,
    const std::vector<hozon::hdmap::LaneInfoConstPtr>& hdmap_lanes,
    MappingManager* map_manager) {
  if (hdmap_lanes.empty() || (map_manager == nullptr)) {
    HLOG_ERROR << "hdmap_lanes is empty";
    return false;
  }
  Eigen::Affine3d T_V_W;
  if (!Pose2Eigen(localization.pose(), &T_V_W)) {
    HLOG_ERROR << "ins quaternion is 0 0 0 0";
    return false;
  }
  T_V_W = T_V_W.inverse();
  std::unordered_set<std::string> road_ids;
  std::unordered_set<std::string> sec_ids;
  for (const auto& lane_ptr : hdmap_lanes) {
    if (!lane_ptr) {
      continue;
    }
    auto lane = lane_ptr->lane();

    // 过滤车系x小于-20m的高精地图点
    if (!lane.has_left_boundary() || !lane.has_right_boundary()) {
      continue;
    }
    const auto& segment = lane.left_boundary().curve().segment();
    if (segment.empty()) {
      continue;
    }
    const auto gcj_points =
        segment[segment.size() - 1].line_segment().original_point();
    if (gcj_points.empty()) {
      continue;
    }
    const auto& p = gcj_points[gcj_points.size() - 1];
    Eigen::Vector3d p_gcj(p.y(), p.x(), 0);
    Eigen::Vector3d p_enu = util::Geo::Gcj02ToEnu(p_gcj, ref_point);
    Eigen::Vector3d p_vehicle = T_V_W * p_enu;
    if (p_vehicle.x() < -20) {
      continue;
    }

    AddMapLine(T_V_W, ref_point, lane.left_boundary(), map_manager);
    AddMapLine(T_V_W, ref_point, lane.right_boundary(), map_manager);
    if (lane.has_extra_left_boundary()) {
      AddMapLine(T_V_W, ref_point, lane.extra_left_boundary(), map_manager);
    }
    if (lane.has_extra_right_boundary()) {
      AddMapLine(T_V_W, ref_point, lane.extra_right_boundary(), map_manager);
    }
    road_ids.insert(lane_ptr->road_id().id());
    sec_ids.insert(lane_ptr->section_id().id());
  }
  for (const auto& road_id : road_ids) {
    hozon::hdmap::Id road_ptr;
    road_ptr.set_id(road_id);
    auto road = GLOBAL_HD_MAP->GetRoadById(road_ptr)->road();
    for (const auto& section : road.section()) {
      if (sec_ids.find(section.id().id()) == sec_ids.end()) {
        continue;
      }
      for (const auto& edge : section.boundary().outer_polygon().edge()) {
        int road_boundary_id = std::stoi(edge.id().id());
        if (map_manager->lane_lines.count(road_boundary_id) != 0) {
          continue;
        }
        auto& every_road_edge = map_manager->lane_lines[road_boundary_id];
        for (const auto& segment : edge.curve().segment()) {
          for (const auto& p : segment.line_segment().original_point()) {
            Eigen::Vector3d p_gcj(p.y(), p.x(), 0);
            Eigen::Vector3d p_enu = util::Geo::Gcj02ToEnu(p_gcj, ref_point);
            Eigen::Vector3d p_vehicle = T_V_W * p_enu;
            every_road_edge.points.emplace_back(p_vehicle.x(), p_vehicle.y(),
                                                p_vehicle.z());
            every_road_edge.lane_type = LaneType::Road_Edge;
          }
        }
      }
    }
  }
  return true;
}

bool PoseEstimation::Pose2Eigen(const hozon::common::Pose& pose,
                                Eigen::Affine3d* const affine3d) {
  Eigen::Quaterniond q(pose.quaternion().w(), pose.quaternion().x(),
                       pose.quaternion().y(), pose.quaternion().z());
  if (q.norm() < 1e-10) {
    HLOG_WARN << "quaternion is 0 0 0 0";
    return false;
  }
  Eigen::Vector3d t(pose.position().x(), pose.position().y(),
                    pose.position().z());
  *affine3d = Eigen::Translation3d(t) * Eigen::Affine3d(q);
  return true;
}

void PoseEstimation::RvizFunc() {
  while (!stop_rviz_thread_) {
    rviz_mutex_.lock();
    TrackingManager perception = tracking_manager_;
    MappingManager hdmap = map_manager_;
    Eigen::Affine3d T_ins = T_ins_;
    Eigen::Affine3d T_fc = T_fc_;
    Eigen::Affine3d T_input = T_input_;
    rviz_mutex_.unlock();

    auto sec = static_cast<uint64_t>(perception.timestamp);
    auto nsec = static_cast<uint64_t>(
        (perception.timestamp - static_cast<double>(sec)) * 1e9);
    RelocRviz::PubFcOdom(T_fc, sec, nsec, "/pe/fc_odom");
    RelocRviz::PubFcTf(T_fc, sec, nsec, "/pe/fc_tf");
    RelocRviz::PubFcPath(T_fc, sec, nsec, "/pe/fc_path");
    RelocRviz::PubPerception(T_input, perception, sec, nsec, "/pe/perception");
    RelocRviz::PubPerceptionMarker(T_fc, perception, sec, nsec,
                                   "/pe/perception_marker");
    RelocRviz::PubHdmap(T_fc, hdmap, sec, nsec, "/pe/hdmap");
    RelocRviz::PubHdmapMarker(T_fc, hdmap, sec, nsec, "/pe/hdmap_marker");
    RelocRviz::PubInsLocationState(T_fc, ins_state_, sd_position_,
                                   location_state_, velocity_, sec, nsec,
                                   "/pe/ins_location_state");
    RelocRviz::PubInsOdom(T_ins, sec, nsec, "/pe/ins_odom");
    RelocRviz::PubInputOdom(T_input, sec, nsec, "/pe/input_odom");
    usleep(10 * 1e3);
  }
}

}  // namespace pe
}  // namespace loc
}  // namespace mp
}  // namespace hozon
