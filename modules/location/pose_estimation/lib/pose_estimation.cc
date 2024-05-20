/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： pose_estimation.cc
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/location/pose_estimation/lib/pose_estimation.h"

#include <algorithm>
#include <chrono>
#include <list>
#include <memory>
#include <string>
#include <tuple>

#include "modules/location/pose_estimation/lib/perception/perception.h"
#include "modules/util/include/util/rviz_agent/rviz_agent.h"

namespace hozon {
namespace mp {
namespace loc {

bool MapMatching::Init(const std::string& config_file,
                       const std::string& cfg_cam_path) {
  YAML::Node config = YAML::LoadFile(config_file);

  bool use_pole = config["use_pole"].as<bool>();
  bool use_traffic_sign = config["use_traffic_sign"].as<bool>();

  map_crop_front_ = config["map_crop_front"].as<double>();
  map_crop_width_ = config["map_crop_width"].as<double>();
  delay_frame_ = config["delay_frame"].as<int>();
  max_frame_buf_ = config["max_frame_buf"].as<int>();
  ins_deque_max_size_ = config["ins_deque_max_size"].as<int>();
  fc_deque_max_size_ = config["fc_deque_max_size"].as<int>();

  // global fault
  mm_params.use_map_lane_match_fault =
      config["use_map_lane_match_fault"].as<bool>();
  mm_params.min_vel = config["min_vel"].as<double>();
  mm_params.map_lane_match_diver = config["map_lane_match_diver"].as<double>();
  mm_params.map_lane_match_max = config["map_lane_match_max"].as<double>();
  mm_params.map_lane_match_buff = config["map_lane_match_buff"].as<int>();
  mm_params.map_lane_match_ser_max =
      config["map_lane_match_ser_max"].as<double>();
  mm_params.map_lane_match_ser_buff =
      config["map_lane_match_ser_buff"].as<int>();
  mm_params.near_dis = config["near_dis"].as<double>();
  mm_params.straight_far_dis = config["straight_far_dis"].as<double>();
  mm_params.curve_far_dis = config["curve_far_dis"].as<double>();
  // mm_params.curvature_thr = config["curvature_thr"].as<double>();
  mm_params.use_fc_offset_onelane_fault =
      config["use_fc_offset_onelane_fault"].as<bool>();
  mm_params.offset_onelane_max_err =
      config["offset_onelane_max_err"].as<double>();
  mm_params.double_line_point_weight =
      config["double_line_point_weight"].as<float>();
  mm_params.set_point_weight_switch =
      config["set_point_weight_switch"].as<bool>();
  mm_params.offset_onelane_cnt = config["offset_onelane_cnt"].as<double>();
  mm_params.offset_maplane_min_dis =
      config["offset_maplane_min_dis"].as<double>();
  mm_params.offset_maplane_max_dis =
      config["offset_maplane_max_dis"].as<double>();
  mm_params.offset_maxlane = config["offset_maxlane"].as<double>();
  mm_params.use_valid_pecep_lane_fault =
      config["use_valid_pecep_lane_fault"].as<bool>();
  mm_params.use_valid_map_lane_fault =
      config["use_valid_map_lane_fault"].as<bool>();
  mm_params.invalid_pecep_thr = config["invalid_pecep_thr"].as<double>();
  mm_params.invalid_map_thr = config["invalid_map_thr"].as<double>();

  // global params
  mm_params.map_distance = config["map_distance"].as<double>();
  mm_params.lane_confidence_thre = config["lane_confidence_thre"].as<double>();
  mm_params.solve_weight = config["solve_weight"].as<double>();
  mm_params.solve_point2line_base =
      config["solve_point2line_base"].as<double>();
  mm_params.thre_continue_badmatch = config["thre_continue_badmatch"].as<int>();
  mm_params.thre_delta_y_diff = config["thre_delta_y_diff"].as<double>();
  mm_params.perceplane_len_lowerbound =
      config["perceplane_len_lowerbound"].as<double>();
  mm_params.lane_width_diff_thre_offset =
      config["lane_width_diff_thre_offset"].as<double>();
  mm_params.avg_diff_offset = config["avg_diff_offset"].as<double>();
  mm_params.window_size = config["window_size"].as<int>();
  mm_params.max_pair_count_thre = config["max_pair_count_thre"].as<int>();
  mm_params.min_pair_count_thre = config["min_pair_count_thre"].as<int>();
  mm_params.lane_width_diff_thre = config["lane_width_diff_thre"].as<double>();
  mm_params.lane_width_check_switch =
      config["lane_width_check_switch"].as<bool>();
  mm_params.adjust_weight_by_lane_width_check =
      config["adjust_weight_by_lane_width_check"].as<bool>();
  mm_params.min_lane_width_check_thre =
      config["min_lane_width_check_thre"].as<double>();
  mm_params.max_lane_width_check_thre =
      config["max_lane_width_check_thre"].as<double>();
  mm_params.debug_plot_info = config["debug_plot_info"].as<bool>();
  mm_params.rviz_show_map_centerlane =
      config["rviz_show_map_centerlane"].as<bool>();
  mm_params.rviz_show_pceplane_oriT =
      config["rviz_show_pceplane_oriT"].as<bool>();
  mm_params.can_ref_point_changed = config["can_ref_point_changed"].as<bool>();
  mm_params.thre_ref_point_change =
      config["thre_ref_point_change"].as<double>();
  mm_params.use_ll_perceplane = config["use_ll_perceplane"].as<bool>();
  mm_params.line_error_normal_thr =
      config["line_error_normal_thr"].as<double>();
  mm_params.use_rviz_bridge = config["use_rviz_bridge"].as<bool>();
  mm_params.common_max_line_length =
      config["common_max_line_length"].as<double>();
  mm_params.common_min_line_length =
      config["common_min_line_length"].as<double>();
  mm_params.max_length_offset = config["max_length_offset"].as<double>();
  mm_params.curvature_thresold = config["curvature_thresold"].as<double>();

  if (mm_params.use_rviz_bridge) {
    int ret = hozon::mp::util::RvizAgent::Instance()
                  .Register<adsfi_proto::viz::TransformStamped>(kTopicMmTf);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register " << kTopicMmTf << " failed";
    }
    ret =
        hozon::mp::util::RvizAgent::Instance().Register<adsfi_proto::viz::Path>(
            kTopicMmCarPath);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register " << kTopicMmCarPath << " failed";
    }
    ret = hozon::mp::util::RvizAgent::Instance()
              .Register<adsfi_proto::viz::PointCloud>(kTopicMmFrontPoints);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register " << kTopicMmFrontPoints << "failed";
    }
    ret = hozon::mp::util::RvizAgent::Instance()
              .Register<adsfi_proto::viz::PointCloud>(
                  kTopicMmMergedMapLaneLinePoints);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register " << kTopicMmMergedMapLaneLinePoints
                << "failed";
    }
    ret = hozon::mp::util::RvizAgent::Instance()
              .Register<adsfi_proto::viz::MarkerArray>(KTopicMmHdMap);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register " << KTopicMmHdMap << " failed";
    }
    ret = hozon::mp::util::RvizAgent::Instance()
              .Register<adsfi_proto::viz::Marker>(kTopicMmTimeStamp);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register " << kTopicMmTimeStamp << "failed";
    }
    ret = hozon::mp::util::RvizAgent::Instance()
              .Register<adsfi_proto::viz::Odometry>(kTopicInsOdom);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register " << kTopicInsOdom << " failed";
    }
    ret = hozon::mp::util::RvizAgent::Instance()
              .Register<adsfi_proto::viz::Odometry>(kTopicMmOdom);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register " << kTopicMmOdom << " failed";
    }
    ret = hozon::mp::util::RvizAgent::Instance()
              .Register<adsfi_proto::viz::Odometry>(kTopicDrOdom);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register " << kTopicMmOdom << " failed";
    }
    ret = hozon::mp::util::RvizAgent::Instance()
              .Register<adsfi_proto::viz::Odometry>(kTopicFcOdom);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register " << kTopicFcOdom << " failed";
    }
    ret = hozon::mp::util::RvizAgent::Instance()
              .Register<adsfi_proto::viz::Marker>(kTopicLocstate);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register " << kTopicLocstate << " failed";
    }
    ret = hozon::mp::util::RvizAgent::Instance()
              .Register<adsfi_proto::viz::PointCloud>(
                  kTopicMmConnectPercepPoints);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register " << kTopicMmConnectPercepPoints
                << " failed";
    }
    ret = hozon::mp::util::RvizAgent::Instance()
              .Register<adsfi_proto::viz::PointCloud>(kTopicMmConnectMapPoints);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register " << kTopicMmConnectMapPoints
                << " failed";
    }
    ret = hozon::mp::util::RvizAgent::Instance()
              .Register<adsfi_proto::viz::PointCloud>(
                  kTopicMmOriginConnectPercepPoints);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register " << kTopicMmOriginConnectPercepPoints
                << " failed";
    }
    ret = hozon::mp::util::RvizAgent::Instance()
              .Register<adsfi_proto::viz::PointCloud>(
                  kTopicMmOriginConnectMapPoints);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register " << kTopicMmOriginConnectMapPoints
                << " failed";
    }
    ret = hozon::mp::util::RvizAgent::Instance()
              .Register<adsfi_proto::viz::Odometry>(kTopicInputOdom);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent register " << kTopicInputOdom << " failed";
    }
  }

  proc_thread_run_ = true;
  proc_thread_ = std::thread(&MapMatching::mmProcCallBack, this);

  HLOG_DEBUG << "MM Init finish ";
  sleep(1);
  return true;
}

MapMatching::~MapMatching() {
  proc_thread_run_ = false;
  if (proc_thread_.joinable()) {
    proc_thread_.join();
  }

  interp_thread_run_ = false;
  if (interp_thread_.joinable()) {
    interp_thread_.join();
  }
  if (mm_params.use_rviz_bridge &&
      hozon::mp::util::RvizAgent::Instance().Ok()) {
    hozon::mp::util::RvizAgent::Instance().Term();
  }
}

void MapMatching::OnLocation(
    const std::shared_ptr<const ::hozon::localization::Localization>& msg) {
  setLocation(*msg);
}

void MapMatching::OnPerception(
    const std::shared_ptr<const ::hozon::perception::TransportElement>& msg) {
  setFrontRoadMark(*msg, true);
}

void MapMatching::OnIns(
    const std::shared_ptr<const ::hozon::localization::HafNodeInfo>& msg) {
  setIns(*msg);
}

template <typename T>
void MapMatching::ShrinkQueue(T* const deque, uint32_t maxsize) {
  if (!deque) {
    return;
  }
  while (deque->size() > maxsize) {
    deque->pop_front();
  }
}

void MapMatching::setIns(const ::hozon::localization::HafNodeInfo& ins) {
  if (ins.header().seq() == latest_ins_.header().seq()) {
    HLOG_WARN << "INS seq repeats,seq:" << ins.header().seq();
    return;
  }
  auto ins_time_diff =
      ins.header().data_stamp() - latest_ins_.header().data_stamp();
  if (ins_time_diff > 1.0 || ins_time_diff <= 0) {
    HLOG_WARN << "ins data delay :" << ins_time_diff;
  }

  Eigen::Quaterniond q_W_V(ins.quaternion().w(), ins.quaternion().x(),
                           ins.quaternion().y(), ins.quaternion().z());
  if (std::isnan(q_W_V.w()) || std::isnan(q_W_V.x()) || std::isnan(q_W_V.y()) ||
      std::isnan(q_W_V.z())) {
    HLOG_WARN << "Inspva_quaternion is NAN";
    return;
  }
  if (q_W_V.norm() < 1e-6) {
    HLOG_WARN << "setIns q_W_V.norm() < 1e-6 ";
    return;
  }
  q_W_V.normalize();

  std::unique_lock<std::mutex> lock(latest_ins_mutex_);
  latest_ins_ = ins;
  latest_ins_mutex_.unlock();
  // 插入INS队列
  {
    std::unique_lock<std::mutex> lock(ins_deque_mutex_);
    ins_deque_.emplace_back(ins);
    ShrinkQueue(&ins_deque_, ins_deque_max_size_);
  }
  Eigen::Vector3d pose(ins.pos_gcj02().x(), ins.pos_gcj02().y(),
                       ins.pos_gcj02().z());
  // ref_point
  if (!init_) {
    HLOG_INFO << "ref_point_ = pose";
    ref_point_mutex_.lock();
    ref_point_ = pose;
    ref_point_mutex_.unlock();
    init_ = true;
    return;
  }

  auto ins_status = ins.gps_status();
  Eigen::Vector3d enu = hozon::mp::util::Geo::Gcj02ToEnu(pose, ref_point_);
  if (mm_params.can_ref_point_changed &&
      ins_status == static_cast<int>(InsStatus::RTK_STABLE) &&
      enu.head<2>().norm() > mm_params.thre_ref_point_change) {
    ref_point_mutex_.lock();
    ref_point_ = pose;
    enu = hozon::mp::util::Geo::Gcj02ToEnu(pose, ref_point_);
    HLOG_INFO << "ref point changed: " << ins_timestamp_
              << " newref: " << ref_point_.x() << " " << ref_point_.y() << " "
              << ref_point_.z();
    ref_point_mutex_.unlock();
  }
  auto T02_W_V = SE3(q_W_V, enu);
  if (mm_params.use_rviz_bridge) {
    auto time_sec = static_cast<uint64_t>(ins.header().data_stamp());
    auto time_nsec =
        static_cast<uint64_t>((ins.header().data_stamp() - time_sec) * 1e9);
    pubVehicle(T02_W_V, time_sec, time_nsec);
    pubOdomPoints(kTopicInsOdom, enu, q_W_V, time_sec, time_nsec);
  }
}
bool MapMatching::FindPecepINS(HafNodeInfo* cur_ins) {
  // 时间戳同步:使用将mm时间戳对齐至ins时间戳
  percep_stamp_mutex_.lock();
  double percep_stamp = percep_stamp_;
  percep_stamp_mutex_.unlock();

  std::unique_lock<std::mutex> lock(ins_deque_mutex_);
  auto it = ins_deque_.rbegin();
  for (; it != ins_deque_.rend(); ++it) {
    if ((*it).header().data_stamp() <= percep_stamp) {
      break;
    }
  }
  if (it == ins_deque_.rend()) {
    HLOG_ERROR << "Not find ins node in INS deque,error!";
    return false;
  } else if (it == ins_deque_.rbegin()) {
    *cur_ins = *it;
    return true;
  } else {
    auto it_next = std::prev(it);
    if (abs(percep_stamp - (*it).header().data_stamp()) <=
        abs(percep_stamp - (*it_next).header().data_stamp())) {
      *cur_ins = *it;
    } else {
      *cur_ins = *it_next;
    }
    return true;
  }
}

bool MapMatching::ExtractInsMsg(const HafNodeInfo& cur_ins, SE3* T02_W_V_ins,
                                const Eigen::Vector3d& ref_point) {
  static SE3 T02_W_V_pre(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  // 1.RTK state
  ins_status_type_ = cur_ins.gps_status();
  if (ins_status_type_ !=
          static_cast<int>(InsStatus::SINGLE_POINT_LOCATION_ORIEN) &&
      ins_status_type_ != static_cast<int>(InsStatus::PSEUD_DIFF) &&
      ins_status_type_ != static_cast<int>(InsStatus::COMB_EXTRAPOLATE) &&
      ins_status_type_ != static_cast<int>(InsStatus::RTK_STABLE) &&
      ins_status_type_ != static_cast<int>(InsStatus::RTK_FLOAT)) {
    HLOG_ERROR << "INS RTK state error,RTK state:" << ins_status_type_;
    return false;
  }
  // 2.旋转与平移提取,enu转换
  Eigen::Vector3d pose(cur_ins.pos_gcj02().x(), cur_ins.pos_gcj02().y(),
                       cur_ins.pos_gcj02().z());
  ins_altitude_ = pose.z();
  Eigen::Vector3d enu = hozon::mp::util::Geo::Gcj02ToEnu(pose, ref_point);
  Eigen::Quaterniond q_W_V(cur_ins.quaternion().w(), cur_ins.quaternion().x(),
                           cur_ins.quaternion().y(), cur_ins.quaternion().z());
  q_W_V.normalize();
  if (q_W_V.norm() < 1e-7) {
    HLOG_ERROR << "setIns q_W_V.norm() < 1e-7 ";
    return false;
  }
  if (std::fabs(q_W_V.norm() - 1) > 1e-3) {
    HLOG_ERROR << "HafNodeInfo quaternion(w,x,y,z) " << cur_ins.quaternion().w()
               << "," << cur_ins.quaternion().x() << ","
               << cur_ins.quaternion().y() << "," << cur_ins.quaternion().z()
               << ",norm:" << q_W_V.norm();
    return false;
  }

  // 3.结果赋值
  {
    std::lock_guard<std::mutex> lock_set_map(map_lck_);
    map_pos_ = pose;
    map_rot_ = q_W_V;
    map_rot_.normalize();
  }
  if (!T02_W_V_pre.translation().isZero()) {
    T02_W_V_pre_ = T02_W_V_pre;
  }
  *T02_W_V_ins = SE3(q_W_V, enu);
  T02_W_V_pre = *T02_W_V_ins;

  time_sec_ = static_cast<uint64_t>(cur_ins.header().data_stamp());
  time_nsec_ =
      static_cast<uint64_t>((cur_ins.header().data_stamp() - time_sec_) * 1e9);

  return true;
}

void MapMatching::pubOdomPoints(const std::string& topic,
                                const Eigen::Vector3d& trans,
                                const Eigen::Quaterniond& q, uint64_t sec,
                                uint64_t nsec) {
  if (!hozon::mp::util::RvizAgent::Instance().Ok()) {
    return;
  }
  adsfi_proto::viz::Odometry odom;
  static uint32_t seq = 0;
  int curr_seq = seq++;
  odom.mutable_header()->set_seq(curr_seq);
  odom.mutable_header()->set_frameid("map");
  odom.mutable_header()->mutable_timestamp()->set_sec(sec);
  odom.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  odom.mutable_pose()->mutable_pose()->mutable_position()->set_x(trans.x());
  odom.mutable_pose()->mutable_pose()->mutable_position()->set_y(trans.y());
  odom.mutable_pose()->mutable_pose()->mutable_position()->set_z(trans.z());
  odom.mutable_pose()->mutable_pose()->mutable_orientation()->set_x(q.x());
  odom.mutable_pose()->mutable_pose()->mutable_orientation()->set_y(q.y());
  odom.mutable_pose()->mutable_pose()->mutable_orientation()->set_z(q.z());
  odom.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(q.w());
  for (int i = 0; i < 36; ++i) {
    odom.mutable_pose()->add_covariance(0.);
  }
  hozon::mp::util::RvizAgent::Instance().Publish(topic, odom);
  return;
}

bool MapMatching::GetHdCurrLaneType(const Eigen::Vector3d& utm) {
  if (GLOBAL_HD_MAP->Empty()) {
    HLOG_ERROR << "hqmap server load map failed";
    return false;
  }
  double nearest_s = 0.0;
  double nearest_l = 0.0;
  hozon::hdmap::LaneInfoConstPtr lane_ptr = nullptr;
  hozon::common::PointENU point;
  point.set_x(utm(0));
  point.set_y(utm(1));
  point.set_z(utm(2));

  int ret =
      GLOBAL_HD_MAP->GetNearestLane(point, &lane_ptr, &nearest_s, &nearest_l);

  if (ret != 0 || lane_ptr == nullptr) {
    HLOG_ERROR << "get nearest lane failed";
    return false;
  }
  auto curr_lane = GLOBAL_HD_MAP->GetLaneById(lane_ptr->id());
  if (curr_lane == nullptr) {
    HLOG_ERROR << "curr_lane nullptr";
  } else {
    if (curr_lane->lane().map_lane_type().toll_lane()) {
      is_toll_lane_ = true;
    } else {
      is_toll_lane_ = false;
    }
  }

  is_ramp_road_ = lane_ptr->IsRampRoad();
  is_main_road_ = lane_ptr->IsMainRoad();
  return true;
}

void MapMatching::setLocation(const ::hozon::localization::Localization& info) {
  ref_point_mutex_.lock();
  Eigen::Vector3d cur_ref_point = ref_point_;
  ref_point_mutex_.unlock();

  // 1. FC deque
  if (info.location_state() == 0 || info.location_state() == 12 ||
      info.location_state() >= 100) {
    HLOG_DEBUG << "info.location_state() : " << info.location_state();
  }

  {
    std::unique_lock<std::mutex> lock(fc_deque_mutex_);
    fc_deque_.emplace_back(info);
    ShrinkQueue(&fc_deque_, fc_deque_max_size_);
  }

  // 2. FC rviz
  if (!hozon::mp::util::RvizAgent::Instance().Ok() ||
      !mm_params.use_rviz_bridge) {
    return;
  }
  static double time = -1;
  uint64_t fc_sec = uint64_t(info.header().data_stamp());
  uint64_t fc_nsec = uint64_t((info.header().data_stamp() - fc_sec) * 1e9);
  double stampe = static_cast<double>(info.header().data_stamp());
  if (time > 0) {
    auto dt = stampe - time;
    if (dt < 0 || dt > 1) {
      // 0.时间出现超前
      // 1.延时超过1s
      HLOG_ERROR << "Time error:" << dt;
      time = stampe;
      return;
    }
  }
  Eigen::Vector3d pose(info.pose().gcj02().x(), info.pose().gcj02().y(),
                       info.pose().gcj02().z());

  Eigen::Quaterniond q_W_V(
      info.pose().quaternion().w(), info.pose().quaternion().x(),
      info.pose().quaternion().y(), info.pose().quaternion().z());
  q_W_V.normalize();
  if (q_W_V.norm() < 1e-7) {
    HLOG_ERROR << "setLocation q_W_V.norm() < 1e-7";
    return;
  }

  if (!init_) {
    return;
  }

  Eigen::Vector3d dr_pose(info.pose_dr().position().x(),
                          info.pose_dr().position().y(),
                          info.pose_dr().position().z());

  ref_point_mutex_.lock();
  Eigen::Vector3d enu = util::Geo::Gcj02ToEnu(pose, cur_ref_point);
  Eigen::Vector3d dr_enu = util::Geo::Gcj02ToEnu(dr_pose, cur_ref_point);
  ref_point_mutex_.unlock();
  pubOdomPoints(kTopicFcOdom, enu, q_W_V, fc_sec, fc_nsec);
  pubOdomPoints(kTopicDrOdom, dr_enu, q_W_V, fc_sec, fc_nsec);

  adsfi_proto::viz::Marker text_marker;
  text_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
  text_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
  text_marker.set_id(0);
  text_marker.mutable_lifetime()->set_sec(0);
  text_marker.mutable_header()->mutable_timestamp()->set_sec(fc_sec);
  text_marker.mutable_header()->mutable_timestamp()->set_nsec(fc_nsec);
  text_marker.mutable_header()->set_frameid("map");
  text_marker.mutable_pose()->mutable_position()->set_x(enu(0));
  text_marker.mutable_pose()->mutable_position()->set_y(enu(1));
  text_marker.mutable_pose()->mutable_position()->set_z(12);

  text_marker.mutable_pose()->mutable_orientation()->set_x(0);
  text_marker.mutable_pose()->mutable_orientation()->set_y(0);
  text_marker.mutable_pose()->mutable_orientation()->set_z(0);
  text_marker.mutable_pose()->mutable_orientation()->set_w(1);

  text_marker.mutable_color()->set_r(1);
  text_marker.mutable_color()->set_g(1);
  text_marker.mutable_color()->set_b(1);
  text_marker.mutable_color()->set_a(1);

  text_marker.set_text("loc-state:" + std::to_string(info.location_state()));
  text_marker.mutable_scale()->set_x(0.1);
  text_marker.mutable_scale()->set_y(0);
  text_marker.mutable_scale()->set_z(0.8);
  hozon::mp::util::RvizAgent::Instance().Publish(kTopicLocstate, text_marker);
}

bool MapMatching::FindPecepFC(hozon::localization::Localization* cur_fc) {
  // 时间戳同步:使用将mm时间戳对齐至fc时间戳
  percep_stamp_mutex_.lock();
  double percep_stamp = percep_stamp_;
  percep_stamp_mutex_.unlock();

  std::unique_lock<std::mutex> lock(fc_deque_mutex_);
  auto it = fc_deque_.rbegin();
  for (; it != fc_deque_.rend(); ++it) {
    if ((*it).header().data_stamp() <= percep_stamp) {
      break;
    }
  }
  if (it == fc_deque_.rend()) {
    HLOG_ERROR << "Not find fc node in FC deque,error!";
    return false;
  } else if (it == fc_deque_.rbegin()) {
    *cur_fc = *it;
    return true;
  } else {
    auto it_next = std::prev(it);
    if (abs(percep_stamp - (*it).header().data_stamp()) <=
        abs(percep_stamp - (*it_next).header().data_stamp())) {
      *cur_fc = *it;
    } else {
      *cur_fc = *it_next;
    }
    return true;
  }
}

bool MapMatching::CompensateInsYError(SE3* T02_W_V_INPUT,
                                      double ins_timeStamp) {
  if (abs(ins_timeStamp - T_fc_.timeStamp) > 0.1) {
    return false;
  }
  // 构建补偿y后的当前INS测量,将INS转到车体系下进行y的偏差，以INS节点为原点
  SE3 T_ins = *T02_W_V_INPUT;
  SE3 T_fc = T_fc_.pose;
  SE3 T_diff = T_ins.inverse() * T_fc;
  Eigen::Vector3d ori_diff(0, 0, 0);
  Sophus::SO3d R_diff = Sophus::SO3d::exp(ori_diff);
  Eigen::Vector3d pose_diff(0, T_diff.translation().y(), 0);
  SE3 T_com_diff(R_diff, pose_diff);
  *T02_W_V_INPUT = T_ins * T_com_diff;
  return true;
}

void MapMatching::procData() {
  Eigen::Vector3d esti_ref_point;
  ref_point_mutex_.lock();
  esti_ref_point = ref_point_;
  ref_point_mutex_.unlock();
  auto t1 = std::chrono::steady_clock::now();
  auto t2 = std::chrono::steady_clock::now();
  double t = 0;
  // 地图数据处理
  {
    std::lock_guard<std::mutex> lock_set_map(map_lck_);
    setSubMap(map_pos_, map_rot_.toRotationMatrix(), esti_ref_point);
  }

  // 感知车道线时间戳获取
  SensorSync curr_roadmark_sensor;
  road_mark_mutex_.lock();
  if (!roadmark_sensor_.empty()) {
    curr_roadmark_sensor = *(roadmark_sensor_.back());
  }
  road_mark_mutex_.unlock();
  if (curr_roadmark_sensor.frame_id != 0) {
    percep_stamp_mutex_.lock();
    percep_stamp_ =
        curr_roadmark_sensor.transport_element.header().data_stamp();
    percep_stamp_mutex_.unlock();
  }
  //  寻找FC
  hozon::localization::Localization cur_fc;
  Eigen::Vector3d cur_vel;
  if (!FindPecepFC(&cur_fc)) {
    HLOG_ERROR << "Dont find fc when use perception line to find fc deque";
    T_fc_.valid = false;
  } else {
    T_fc_.valid = true;
    Eigen::Quaterniond q_W_V(
        cur_fc.pose().quaternion().w(), cur_fc.pose().quaternion().x(),
        cur_fc.pose().quaternion().y(), cur_fc.pose().quaternion().z());
    if (fabs(q_W_V.norm() - 1) < 1e-3) {
      q_W_V.normalize();
      Eigen::Vector3d pose(cur_fc.pose().gcj02().x(), cur_fc.pose().gcj02().y(),
                           cur_fc.pose().gcj02().z());
      ref_point_mutex_.lock();
      Eigen::Vector3d enu = util::Geo::Gcj02ToEnu(pose, esti_ref_point);
      ref_point_mutex_.unlock();
      T_fc_.pose = SE3(q_W_V, enu);
      Eigen::Vector3d velocity_vrf(cur_fc.pose().linear_velocity_vrf().x(),
                                   cur_fc.pose().linear_velocity_vrf().y(),
                                   cur_fc.pose().linear_velocity_vrf().z());
      cur_vel = velocity_vrf;
      T_fc_.velocity_vrf = velocity_vrf;
      T_fc_.timeStamp = cur_fc.header().data_stamp();
    }
  }

  // ins数据处理
  SE3 T02_W_V, T02_W_V_pre, T02_W_V_INPUT;
  double input_stamp = percep_stamp_;
  proc_stamp_ = percep_stamp_;
  HafNodeInfo cur_ins;
  ins_deque_mutex_.lock();
  if (ins_deque_.empty()) {
    HLOG_ERROR << "ProcessData: INS deque is empty!";
    return;
  }
  ins_deque_mutex_.unlock();

  if (!FindPecepINS(&cur_ins)) {
    HLOG_ERROR << "ProcessData: Dont find ins when use perception line to find "
                  "ins deque";
    std::unique_lock<std::mutex> lock(latest_ins_mutex_);
    cur_ins = latest_ins_;
  }

  if (!ExtractInsMsg(cur_ins, &T02_W_V, esti_ref_point)) {
    HLOG_ERROR << "ExtractInsMsg Fail!";
    return;
  }

  T02_W_V_pre = T02_W_V_pre_;
  T02_W_V_INPUT = T02_W_V;

  bool use_extrapolate = static_cast<bool>(
      cur_ins.gps_status() != static_cast<int>(InsStatus::RTK_STABLE));
  if (use_extrapolate && proc_stamp_last_ > 0.0 && T_fc_.valid) {
    if (!CompensateInsYError(&T02_W_V_INPUT, cur_ins.header().data_stamp())) {
      HLOG_ERROR << "CompensateInsYError Fail!";
    }
    pubOdomPoints(kTopicInputOdom, T02_W_V_INPUT.translation(),
                  T02_W_V_INPUT.unit_quaternion(), time_sec_, time_nsec_);
  }

  // 感知数据处理
  std::shared_ptr<Perception> all_perception = std::make_shared<Perception>();
  time_.evaluate(
      [&, this] {
        if (curr_roadmark_sensor.frame_id != 0) {
          all_perception->SetLaneLineList(
              curr_roadmark_sensor.transport_element);
        }
      },
      "pb convert:");
  pubTimeAndInsStatus(T02_W_V, input_stamp);
  Sophus::SE3d T02_W_VF = T02_W_V, _T_W_V_fine = T02_W_V;
  hozon::mp::loc::Connect connect;
  hozon::mp::loc::Connect origin_connect;

  time_.evaluate(
      [&, this] {
        map_match_->SetInsTs(input_stamp);
        map_match_->SetVel(cur_vel);
        map_match_->Match(mhd_map_, all_perception, T02_W_V_INPUT, T_fc_);
      },
      "match lane :");
  if (hozon::mp::util::RvizAgent::Instance().Ok()) {
    VP merged_map_lane_lines = map_match_->GetRvizMergeMapLines();
    pubPoints(merged_map_lane_lines, time_sec_, time_nsec_,
              kTopicMmMergedMapLaneLinePoints);
  }

  matched_lane_pair_size_ = map_match_->GetMatchPairSize();
  if (matched_lane_pair_size_ < 2) {
    HLOG_WARN << "ProcessData: matched_lane_pair_size stamp:" << ins_timestamp_
              << " size:" << matched_lane_pair_size_;
  }
  ERROR_TYPE mm_err_type{static_cast<ERROR_TYPE>(map_match_->GetErrorType())};
  switch (mm_err_type) {
    case ERROR_TYPE::NO_ERROR:
      mmfault_.pecep_lane_error = false;
      mmfault_.map_lane_error = false;
      mmfault_.map_lane_match_error = false;
      mmfault_.fc_offset_onelane_error = false;
      break;
    case ERROR_TYPE::NO_VALID_PECEP_LANE:
      mmfault_.pecep_lane_error = true;
      return;
    case ERROR_TYPE::NO_VALID_MAP_LANE:
      mmfault_.map_lane_error = true;
      return;
    case ERROR_TYPE::NO_MAP_BOUNDARY_LINE:
      mmfault_.map_lane_error = true;
      return;
    case ERROR_TYPE::NO_MERGE_MAP_LANE:
      mmfault_.map_lane_error = true;
      return;
    case ERROR_TYPE::MAP_LANE_MATCH_FAIL:
      mmfault_.map_lane_match_error = true;
      break;
    case ERROR_TYPE::OFFSET_ONELANE:
      mmfault_.fc_offset_onelane_error = true;
      break;
    default:
      break;
  }
  connect = map_match_->Result();
  origin_connect = map_match_->OriginResult();
  bool solve_is_ok = true;
  auto t1_solver = std::chrono::steady_clock::now();
  time_.evaluate(
      [&, this] {
        T02_W_VF = hozon::mp::loc::MapMatchSolver::solve2D(
            connect, T02_W_V_INPUT, T02_W_V_pre, &solve_is_ok, input_stamp);
      },
      "sovler:");
  output_valid_ = true;
  auto t2_solver = std::chrono::steady_clock::now();
  auto solver_time =
      (t2_solver.time_since_epoch() - t1_solver.time_since_epoch()).count() /
      1e9;
  if (!solve_is_ok) {
    HLOG_ERROR << "ProcessData: solver is not ok stamp:" << ins_timestamp_;
    output_valid_ = false;
  }
  {
    auto T_V_VSF = T02_W_V_INPUT.inverse() * T02_W_VF;
    double yaw_diff = T_V_VSF.log().tail<3>().z();
    V3 rot(0, 0, yaw_diff);
    V3 trans = V3(0, T_V_VSF.translation().y(), 0);
    if (!CheckLaneMatch(T_V_VSF)) {
      bad_lane_match_count_++;
    }
    if (output_valid_ && bad_lane_match_count_ > 5) {
      HLOG_ERROR << "ProcessData: bad_lane_match";
      output_valid_ = false;
    }
    bad_lane_match_count_ = 0;
    T_delta_last_ = T_V_VSF;
    T_V_VSF = Sophus::SE3d(Sophus::SO3d::exp(rot), trans);
    _T_W_V_fine = T02_W_V_INPUT * T_V_VSF;
  }

  bool lane_width_check = false;
  if (mm_params.lane_width_check_switch) {
    lane_width_check = map_match_->CheckLaneWidth(T02_W_VF);
  }
  if (output_valid_ && lane_width_check) {
    HLOG_ERROR << "ProcessData: lane width check failed!";
    output_valid_ = false;
  }
  {
    {
      std::lock_guard<std::mutex> lg(mm_output_lck_);
      T_output_ = _T_W_V_fine;
      if (!output_valid_) {
        mm_node_info_ = generateNodeInfo(T_output_, 0, 0, true, esti_ref_point);
      } else {
        uint64_t curr_sec = 0;
        uint64_t curr_nsec = 0;
        percep_stamp_mutex_.lock();
        curr_sec = uint64_t(percep_stamp_);
        curr_nsec =
            uint64_t((percep_stamp_ - static_cast<double>(curr_sec)) * 1e9);
        percep_stamp_mutex_.unlock();
        output_valid_ = false;
        if (!match_inited) {
          match_inited = true;
        }
        if (hozon::mp::util::RvizAgent::Instance().Ok() &&
            mm_params.use_rviz_bridge) {
          pubOdomPoints(kTopicMmOdom, T_output_.translation(),
                        T_output_.unit_quaternion(), time_sec_, time_nsec_);
        }
        mm_node_info_ = generateNodeInfo(T_output_, curr_sec, curr_nsec, false,
                                         esti_ref_point);
      }
    }
    proc_stamp_ = input_stamp;
    if (curr_roadmark_sensor.frame_id != 0) {
      auto lane_lines =
          all_perception->GetElement(PERCEPTYION_LANE_BOUNDARY_LINE);
      auto lane =
          std::static_pointer_cast<PerceptionLaneLineList>(lane_lines[0]);
      if (hozon::mp::util::RvizAgent::Instance().Ok() &&
          mm_params.use_rviz_bridge) {
        setPoints(*lane, T02_W_V, &front_points_);
        pubPoints(front_points_, time_sec_, time_nsec_, kTopicMmFrontPoints);
        front_points_.clear();
        setConnectPercepPoints(connect, T_output_, front_points_);
        pubConnectPercepPoints(front_points_, time_sec_, time_nsec_);
        front_points_.clear();
        setConnectMapPoints(connect, T02_W_V, front_points_);
        pubConnectMapPoints(front_points_, time_sec_, time_nsec_);
        front_points_.clear();
        setOriginConnectMapPoints(origin_connect, T02_W_V, front_points_);
        pubOriginConnectMapPoints(front_points_, time_sec_, time_nsec_);
        front_points_.clear();
        setOriginConnectPercepPoints(origin_connect, T_output_, front_points_);
        pubOriginConnectPercepPoints(front_points_, time_sec_, time_nsec_);
      }
    }
  }

  T02_W_V_last_ = T02_W_V;
  T02_W_VF_last_ = T02_W_VF;
  proc_stamp_last_ = input_stamp;
  time_.print();

  // 帧率打印
  t2 = std::chrono::steady_clock::now();
  t = t2.time_since_epoch().count() / 1e9;
  auto proc_time =
      (t2.time_since_epoch() - t1.time_since_epoch()).count() / 1e9;
  HLOG_DEBUG << "test mm | proc_time = " << proc_time;
  static MapMatchingFrameRateRecord mm_effective_fr;
  mm_effective_fr.CalFrameRate(t, "mm effective frame rate");
}

PtrNodeInfo MapMatching::getMmNodeInfo() {
  {
    std::lock_guard<std::mutex> lg(mm_output_lck_);
    auto output_node_info =
        std::make_shared<::hozon::localization::HafNodeInfo>();
    if (mm_node_info_ == nullptr) {
      return nullptr;
    }
    output_node_info = mm_node_info_;
    return output_node_info;
  }
}

void MapMatching::setSubMap(const Eigen::Vector3d& vehicle_position,
                            const Eigen::Matrix3d& vehicle_rotation,
                            const Eigen::Vector3d& ref_point) {
  if (!init_) {
    HLOG_ERROR << "setSubMap init failed";
    return;
  }

  double x = vehicle_position.x();
  double y = vehicle_position.y();

  hozon::common::coordinate_convertor::GCS2UTM(51, &y, &x);
  hozon::common::PointENU enupt;
  enupt.set_x(y);
  enupt.set_y(x);
  enupt.set_z(0);

  if (GLOBAL_HD_MAP->Empty()) {
    HLOG_ERROR << "hdmap server load map failed";
    return;
  }
  mhd_map_.Clear();
  mhd_map_.set_ref_point(ref_point);
  mhd_map_.SetMap(enupt, vehicle_rotation, mm_params.map_distance, ref_point);
  if (!mm_params.use_rviz_bridge ||
      !hozon::mp::util::RvizAgent::Instance().Ok()) {
    return;
  }

  if (hozon::mp::util::RvizAgent::Instance().Ok()) {
    adsfi_proto::viz::MarkerArray markers;
    for (auto elment : mhd_map_.elment_) {
      switch (elment->type_) {
        case hozon::mp::loc::HD_MAP_LANE_BOUNDARY_LINE: {
          auto p =
              std::static_pointer_cast<hozon::mp::loc::MapBoundaryLine>(elment);
          for (auto line : p->boundary_line_) {
            auto& new_line = line.second;
            VP points;
            for (auto& point : new_line.control_point) {
              points.emplace_back(point.point);
            }
            if (!points.empty()) {
              auto line_first_point = points[0];
              auto line_id_mark =
                  lineIdToMarker(line_first_point, new_line.id_boundary);
              auto* line_marker = markers.add_markers();
              line_marker->CopyFrom(line_id_mark);

              auto lane_mark =
                  laneToMarker(points, new_line.id_boundary, false, true, 1);
              auto* marker = markers.add_markers();
              marker->CopyFrom(lane_mark);
            }
          }
          break;
        }

        case hozon::mp::loc::HD_MAP_ROAD_EDGE: {
          auto p =
              std::static_pointer_cast<hozon::mp::loc::MapRoadEdge>(elment);
          for (auto line : p->edge_line_) {
            auto& new_line = line.second;
            VP points;
            for (auto& point : new_line.control_point) {
              points.emplace_back(point.point);
            }
            if (!points.empty()) {
              auto edge_mark = RoadEdgeToMarker(points, new_line.id_edge, false,
                                                false, 2, true);
              auto* marker = markers.add_markers();
              marker->CopyFrom(edge_mark);
            }
          }
          break;
        }
        default:
          break;
      }
    }
    hozon::mp::util::RvizAgent::Instance().Publish(KTopicMmHdMap, markers);
  }
}

void MapMatching::setFrontRoadMark(
    const ::hozon::perception::TransportElement& roadmark,
    bool is_roadmark = true) {
  sensorPush(roadmark, is_roadmark);
}

void MapMatching::sensorPush(
    const ::hozon::perception::TransportElement& transport_element,
    bool is_roadmark = true) {
  if (is_roadmark) {
    std::unique_lock<std::mutex> ul(road_mark_mutex_);
    if (!roadmark_sensor_.empty()) {
      auto& back = roadmark_sensor_.back();
      auto back_id = back->frame_id;
      auto frame_id = transport_element.header().seq();
      if (frame_id != back_id) {
        back->setFinsh();
        std::shared_ptr<SensorSync> new_sensor = std::make_shared<SensorSync>();
        new_sensor->frame_id = frame_id;
        new_sensor->transport_element.CopyFrom(transport_element);
        roadmark_sensor_.push_back(new_sensor);
      } else {
        back->transport_element.CopyFrom(transport_element);
        if (back->ok()) {
          HLOG_ERROR << "sensor sync error at road marking push";
        }
      }
    } else {
      auto frame_id = transport_element.header().seq();
      std::shared_ptr<SensorSync> new_sensor = std::make_shared<SensorSync>();
      new_sensor->frame_id = frame_id;
      new_sensor->transport_element.CopyFrom(transport_element);
      roadmark_sensor_.push_back(new_sensor);
    }
  }
}
std::tuple<bool, SensorSync> MapMatching::sensorFront(void) {
  std::unique_lock<std::mutex> ul(road_mark_mutex_);
  if (roadmark_sensor_.empty()) {
    return std::tuple<bool, SensorSync>(false, hozon::mp::loc::SensorSync());
  } else {
    auto& ret = (*(roadmark_sensor_.front()));
    if (!ret.ok()) {
      return std::tuple<bool, SensorSync>(false, hozon::mp::loc::SensorSync());
    }
    return std::tuple<bool, SensorSync>(true, ret);
  }
}
unsigned int MapMatching::sensorSize() {
  std::unique_lock<std::mutex> ul(road_mark_mutex_);
  return roadmark_sensor_.size();
}
void MapMatching::eraseFront(bool is_erase_buf = true) {
  if (is_erase_buf) {
    std::unique_lock<std::mutex> ul(road_mark_mutex_);
    while (static_cast<int>(roadmark_sensor_.size()) > max_frame_buf_) {
      roadmark_sensor_.erase(roadmark_sensor_.begin());
    }
  } else {
    std::unique_lock<std::mutex> ul(road_mark_mutex_);
    if (!roadmark_sensor_.empty()) {
      roadmark_sensor_.erase(roadmark_sensor_.begin());
    }
  }
}

void MapMatching::mmProcCallBack(void) {
  pthread_setname_np(pthread_self(), "loc_mm_proc");
  while (proc_thread_run_) {
    static int loc_mm_proc_count = 0;
    eraseFront(true);
    if (static_cast<int>(this->sensorSize()) < delay_frame_) {
      usleep(50 * 1e3);
      continue;
    }
    auto front = sensorFront();
    if (!std::get<0>(front)) {
      continue;
    }
    eraseFront(false);
    procData();
    ++loc_mm_proc_count;
    if (loc_mm_proc_count >= 100) {
      loc_mm_proc_count = 0;
      HLOG_INFO << "loc_mm_proc heartbeat";
    }
    usleep(33 * 1e3);
  }
}

PtrNodeInfo MapMatching::generateNodeInfo(const Sophus::SE3d& T_W_V,
                                          uint64_t sec, uint64_t nsec,
                                          const bool& has_err,
                                          Eigen::Vector3d ref_point) {
  PtrNodeInfo node_info =
      std::make_shared<::hozon::localization::HafNodeInfo>();
  auto blh = hozon::mp::util::Geo::EnuToGcj02(
      T_W_V.translation().cast<double>(), ref_point);
  blh.z() = ins_altitude_;
  node_info->set_type(::hozon::localization::HafNodeInfo_NodeType::
                          HafNodeInfo_NodeType_MapMatcher);
  double node_stamp = nsec * 1e-9 + static_cast<double>(sec);
  node_info->mutable_header()->set_data_stamp(node_stamp);
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
      tp = std::chrono::time_point_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now());
  node_info->mutable_header()->set_publish_stamp(tp.time_since_epoch().count() *
                                                 1.0e-9);
  node_info->mutable_header()->set_frame_id("node_info_mm");
  node_info->set_is_valid(true);
  node_info->mutable_pos_gcj02()->set_x(blh.x());
  node_info->mutable_pos_gcj02()->set_y(blh.y());
  node_info->mutable_pos_gcj02()->set_z(blh.z());
  node_info->mutable_quaternion()->set_x(T_W_V.unit_quaternion().x());
  node_info->mutable_quaternion()->set_y(T_W_V.unit_quaternion().y());
  node_info->mutable_quaternion()->set_z(T_W_V.unit_quaternion().z());
  node_info->mutable_quaternion()->set_w(T_W_V.unit_quaternion().w());
  if (!has_err) {
    HLOG_DEBUG << "blh.x()," << blh.x() << ",blh.y()," << blh.y() << ",blh.z(),"
               << blh.z() << ",proc_stamp_," << proc_stamp_ << ",";
    node_info->set_valid_estimate(true);
  } else {
    node_info->set_valid_estimate(false);
    HLOG_ERROR << "MM is not valid due to rare connect!!!";
  }
  // error code
  node_info->set_warn_info(0);
  if (mmfault_.pecep_lane_error == true) {
    node_info->set_warn_info(123);
    HLOG_ERROR << "mmfault:123";
  }
  if (mmfault_.map_lane_error == true) {
    node_info->set_warn_info(124);
    HLOG_ERROR << "mmfault:124";
  }
  if (mmfault_.map_lane_match_error == true) {
    node_info->set_warn_info(130);
    HLOG_ERROR << "mmfault:130";
  }
  return node_info;
}

void MapMatching::setPoints(const PerceptionLaneLineList& line_list,
                            const SE3& T_W_V, VP* points) {
  if (points == nullptr) {
    return;
  }
  (*points).clear();
  for (const auto& line : line_list.lane_line_list_) {
    if (std::fabs(line->lane_position_type() > 2) || line->points().empty()) {
      continue;
    }
    auto line_points = line->points();
    auto point_size = std::min(500, static_cast<int>(line_points.size()));
    for (auto i = 0; i < point_size; ++i) {
      auto point = T_W_V * line_points[i];
      (*points).emplace_back(point);
    }
  }
}

void MapMatching::pubPoints(const VP& points, const uint64_t& sec,
                            const uint64_t& nsec,
                            const std::string& krviz_topic) {
  if (hozon::mp::util::RvizAgent::Instance().Ok()) {
    adsfi_proto::viz::PointCloud lane_points;
    static uint32_t seq = 0;
    int curr_seq = seq++;

    lane_points.mutable_header()->set_seq(curr_seq);
    lane_points.mutable_header()->mutable_timestamp()->set_sec(time_sec_);
    lane_points.mutable_header()->mutable_timestamp()->set_nsec(time_nsec_);
    lane_points.mutable_header()->set_frameid("map");

    auto* channels = lane_points.add_channels();
    channels->set_name("rgb");

    for (auto p : points) {
      auto* points_ = lane_points.add_points();
      points_->set_x(p.x());
      points_->set_y(p.y());
      points_->set_z(p.z());
    }

    hozon::mp::util::RvizAgent::Instance().Publish(krviz_topic, lane_points);
  }
}

void MapMatching::pubVehicle(const SE3& T, const double& sec,
                             const double& nsec) {
  if (hozon::mp::util::RvizAgent::Instance().Ok()) {
    adsfi_proto::viz::Odometry mm_odom;
    static uint32_t seq = 0;
    int curr_seq = seq++;
    mm_odom.mutable_header()->set_seq(curr_seq);
    mm_odom.mutable_header()->set_frameid("map");
    mm_odom.mutable_header()->mutable_timestamp()->set_sec(sec);
    mm_odom.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    mm_odom.mutable_pose()->mutable_pose()->mutable_position()->set_x(
        T.translation().x());
    mm_odom.mutable_pose()->mutable_pose()->mutable_position()->set_y(
        T.translation().y());
    mm_odom.mutable_pose()->mutable_pose()->mutable_position()->set_z(
        T.translation().z());

    mm_odom.mutable_pose()->mutable_pose()->mutable_orientation()->set_x(
        T.unit_quaternion().x());
    mm_odom.mutable_pose()->mutable_pose()->mutable_orientation()->set_y(
        T.unit_quaternion().y());
    mm_odom.mutable_pose()->mutable_pose()->mutable_orientation()->set_z(
        T.unit_quaternion().z());
    mm_odom.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(
        T.unit_quaternion().w());
    for (int i = 0; i < 36; ++i) {
      mm_odom.mutable_pose()->add_covariance(0.);
    }
    geo_tf_.mutable_header()->set_seq(curr_seq);
    geo_tf_.mutable_header()->mutable_timestamp()->set_sec(sec);
    geo_tf_.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    geo_tf_.mutable_header()->set_frameid("map");
    geo_tf_.set_child_frame_id("vehicle");
    geo_tf_.mutable_transform()->mutable_translation()->set_x(
        T.translation().x());
    geo_tf_.mutable_transform()->mutable_translation()->set_y(
        T.translation().y());
    geo_tf_.mutable_transform()->mutable_translation()->set_z(
        T.translation().z());
    geo_tf_.mutable_transform()->mutable_rotation()->set_x(
        T.unit_quaternion().x());
    geo_tf_.mutable_transform()->mutable_rotation()->set_y(
        T.unit_quaternion().y());
    geo_tf_.mutable_transform()->mutable_rotation()->set_z(
        T.unit_quaternion().z());
    geo_tf_.mutable_transform()->mutable_rotation()->set_w(
        T.unit_quaternion().w());
    hozon::mp::util::RvizAgent::Instance().Publish(kTopicMmTf, geo_tf_);
  }

  if (hozon::mp::util::RvizAgent::Instance().Ok()) {
    auto* pose = gnss_gcj02_path_.add_poses();
    static uint32_t seq = 0;
    int curr_seq = seq++;

    gnss_gcj02_path_.mutable_header()->set_seq(curr_seq);
    gnss_gcj02_path_.mutable_header()->mutable_timestamp()->set_sec(sec);
    gnss_gcj02_path_.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    gnss_gcj02_path_.mutable_header()->set_frameid("map");

    pose->mutable_header()->set_seq(curr_seq);
    pose->mutable_header()->mutable_timestamp()->set_sec(sec);
    pose->mutable_header()->mutable_timestamp()->set_nsec(nsec);
    pose->mutable_header()->set_frameid("map");
    pose->mutable_pose()->mutable_position()->set_x(T.translation().x());
    pose->mutable_pose()->mutable_position()->set_y(T.translation().y());
    pose->mutable_pose()->mutable_position()->set_z(T.translation().z());

    pose->mutable_pose()->mutable_orientation()->set_w(T.unit_quaternion().w());
    pose->mutable_pose()->mutable_orientation()->set_x(T.unit_quaternion().x());
    pose->mutable_pose()->mutable_orientation()->set_y(T.unit_quaternion().y());
    pose->mutable_pose()->mutable_orientation()->set_z(T.unit_quaternion().z());

    if (gnss_gcj02_path_.poses().size() > 50) {
      gnss_gcj02_path_.mutable_poses()->DeleteSubrange(0, 1);
    }
    hozon::mp::util::RvizAgent::Instance().Publish(kTopicMmCarPath,
                                                   gnss_gcj02_path_);
  }
}

void MapMatching::pubTimeAndInsStatus(const SE3& T, double stamp) {
  if (hozon::mp::util::RvizAgent::Instance().Ok()) {
    while (se3_buffer_.size() >= 10) {
      se3_buffer_.pop_front();
    }
    se3_buffer_.emplace_back(T);

    double trans_x = 0.;
    double trans_y = 0.;
    if (se3_buffer_.size() == 10) {
      for (auto it = se3_buffer_.rbegin(); it != se3_buffer_.rbegin() + 10;
           it++) {
        trans_x += it->translation().x();
        trans_y += it->translation().y();
      }
      trans_x /= 10;
      trans_y /= 10;
    }

    adsfi_proto::viz::Marker text_marker;
    text_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
    text_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
    text_marker.set_id(0);
    text_marker.mutable_lifetime()->set_sec(0);
    text_marker.mutable_header()->mutable_timestamp()->set_sec(time_sec_);
    text_marker.mutable_header()->mutable_timestamp()->set_nsec(time_nsec_);
    text_marker.mutable_header()->set_frameid("map");
    text_marker.mutable_pose()->mutable_position()->set_x(trans_x + 1);
    text_marker.mutable_pose()->mutable_position()->set_y(trans_y + 1);
    text_marker.mutable_pose()->mutable_position()->set_z(12);

    text_marker.mutable_pose()->mutable_orientation()->set_x(0);
    text_marker.mutable_pose()->mutable_orientation()->set_y(0);
    text_marker.mutable_pose()->mutable_orientation()->set_z(0);
    text_marker.mutable_pose()->mutable_orientation()->set_w(1);

    text_marker.mutable_color()->set_r(1);
    text_marker.mutable_color()->set_g(1);
    text_marker.mutable_color()->set_b(1);
    text_marker.mutable_color()->set_a(1);

    text_marker.set_text("ins-state:" + std::to_string(ins_status_type_) +
                         " time:" + std::to_string(stamp));
    text_marker.mutable_scale()->set_x(0.1);
    text_marker.mutable_scale()->set_y(0);
    text_marker.mutable_scale()->set_z(0.8);

    hozon::mp::util::RvizAgent::Instance().Publish(kTopicMmTimeStamp,
                                                   text_marker);
  }
}

adsfi_proto::viz::Marker MapMatching::RoadEdgeToMarker(
    const VP& points, std::string id, bool is_points, bool is_center,
    float point_size, bool is_edge) {
  adsfi_proto::viz::Marker block;
  if (is_points) {
    block.set_type(adsfi_proto::viz::MarkerType::POINTS);
  } else {
    block.set_type(adsfi_proto::viz::MarkerType::LINE_LIST);
  }
  block.set_action(adsfi_proto::viz::MarkerAction::ADD);
  const char* c_id = id.c_str();
  block.set_id(std::atoi(c_id));
  block.mutable_lifetime()->set_sec(0);
  block.mutable_lifetime()->set_nsec(0);
  block.mutable_header()->mutable_timestamp()->set_sec(time_sec_);
  block.mutable_header()->mutable_timestamp()->set_nsec(time_nsec_);
  block.mutable_header()->set_frameid("map");
  std::string control = is_points ? "_control" : "";
  std::string center = is_center ? "_center" : "";
  std::string edge = is_edge ? "_edge" : "";
  block.set_ns("road" + control + center + edge);
  block.mutable_pose()->mutable_position()->set_x(0);
  block.mutable_pose()->mutable_position()->set_y(0);
  block.mutable_pose()->mutable_position()->set_z(0);
  block.mutable_pose()->mutable_orientation()->set_x(0.0);
  block.mutable_pose()->mutable_orientation()->set_y(0.0);
  block.mutable_pose()->mutable_orientation()->set_z(0.0);
  block.mutable_pose()->mutable_orientation()->set_w(1.0);
  float size = 0.05;
  if (is_points) size = 0.25;
  size = size * point_size;
  block.mutable_scale()->set_x(size);
  block.mutable_scale()->set_y(size);
  block.mutable_scale()->set_z(size);
  if (is_edge) {
    block.mutable_color()->set_r(0 / 255.0);
    block.mutable_color()->set_g(0 / 255.0);
    block.mutable_color()->set_b(255 / 255.0);
  }
  if (is_center) {
    block.mutable_color()->set_r(255.0 / 255.0);
    block.mutable_color()->set_g(255.0 / 255.0);
    block.mutable_color()->set_b(0.0 / 255.0);
  }
  if (!is_edge && !is_center) {
    block.mutable_color()->set_r(1.f);
    block.mutable_color()->set_g(0);
    block.mutable_color()->set_b(0);
  }
  block.mutable_color()->set_a(1.0f);
  for (size_t i = 1; i < points.size(); i++) {
    auto* enu = block.add_points();
    enu->set_x(points[i - 1].x());
    enu->set_y(points[i - 1].y());
    enu->set_z(points[i - 1].z());
    auto* enu_ = block.add_points();
    enu_->set_x(points[i].x());
    enu_->set_y(points[i].y());
    enu_->set_z(points[i].z());
  }
  return block;
}

adsfi_proto::viz::Marker MapMatching::laneToMarker(
    const VP& points, std::string id, bool is_points, bool is_center,
    float point_size, bool is_boundary) {
  adsfi_proto::viz::Marker block;
  if (is_points)
    block.set_type(adsfi_proto::viz::MarkerType::POINTS);
  else
    block.set_type(adsfi_proto::viz::MarkerType::LINE_LIST);
  block.set_action(adsfi_proto::viz::MarkerAction::ADD);
  const char* c_id = id.c_str();
  block.set_id(std::atoi(c_id));
  block.mutable_lifetime()->set_sec(0);
  block.mutable_lifetime()->set_nsec(0);
  block.mutable_header()->mutable_timestamp()->set_sec(time_sec_);
  block.mutable_header()->mutable_timestamp()->set_nsec(time_nsec_);
  block.mutable_header()->set_frameid("map");
  std::string control = is_points ? "_control" : "";
  std::string center = is_center ? "_center" : "";
  std::string boundary = is_boundary ? "_boundary" : "";
  block.set_ns("lane" + control + center + boundary);
  block.mutable_pose()->mutable_position()->set_x(0);
  block.mutable_pose()->mutable_position()->set_y(0);
  block.mutable_pose()->mutable_position()->set_z(0);
  block.mutable_pose()->mutable_orientation()->set_x(0.0);
  block.mutable_pose()->mutable_orientation()->set_y(0.0);
  block.mutable_pose()->mutable_orientation()->set_z(0.0);
  block.mutable_pose()->mutable_orientation()->set_w(1.0);
  float size = 0.05;
  if (is_points) size = 0.25;
  size = size * point_size;
  block.mutable_scale()->set_x(size);
  block.mutable_scale()->set_y(size);
  block.mutable_scale()->set_z(size);
  if (is_boundary) {
    block.mutable_color()->set_r(229.0 / 255.0);
    block.mutable_color()->set_g(10 / 255.0);
    block.mutable_color()->set_b(245 / 255.0);
  }
  if (is_center) {
    block.mutable_color()->set_r(255.0 / 255.0);
    block.mutable_color()->set_g(255.0 / 255.0);
    block.mutable_color()->set_b(0.0 / 255.0);
  }
  if (!is_boundary && !is_center) {
    block.mutable_color()->set_r(1.f);
    block.mutable_color()->set_g(0);
    block.mutable_color()->set_b(0);
  }
  block.mutable_color()->set_a(1.0f);
  for (size_t i = 1; i < points.size(); i++) {
    auto* enu = block.add_points();
    enu->set_x(points[i - 1].x());
    enu->set_y(points[i - 1].y());
    enu->set_z(points[i - 1].z());
    auto* enu_ = block.add_points();
    enu_->set_x(points[i].x());
    enu_->set_y(points[i].y());
    enu_->set_z(points[i].z());
  }
  return block;
}

adsfi_proto::viz::Marker MapMatching::lineIdToMarker(const V3 point,
                                                     std::string id) {
  adsfi_proto::viz::Marker marker;
  marker.mutable_header()->set_frameid("map");
  marker.mutable_header()->mutable_timestamp()->set_sec(time_sec_);
  marker.mutable_header()->mutable_timestamp()->set_nsec(time_nsec_);
  const char* c_id = id.c_str();
  marker.set_id(std::atoi(c_id));
  marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
  marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
  marker.mutable_pose()->mutable_orientation()->set_x(0.);
  marker.mutable_pose()->mutable_orientation()->set_y(0.);
  marker.mutable_pose()->mutable_orientation()->set_z(0.);
  marker.mutable_pose()->mutable_orientation()->set_w(1.);
  const double text_size = 0.6;
  marker.mutable_scale()->set_z(text_size);
  marker.mutable_lifetime()->set_sec(0);
  marker.mutable_lifetime()->set_nsec(0);
  marker.mutable_color()->set_a(1.0);
  marker.mutable_color()->set_r(0);
  marker.mutable_color()->set_g(1);
  marker.mutable_color()->set_b(0);
  marker.mutable_pose()->mutable_position()->set_x(point.x());
  marker.mutable_pose()->mutable_position()->set_y(point.y());
  marker.mutable_pose()->mutable_position()->set_z(point.z());
  auto text = marker.mutable_text();
  *text = "ID: " + id;
  return marker;
}
void MapMatching::pubConnectPercepPoints(const VP& points, uint64_t sec,
                                         uint64_t nsec) {
  if (!hozon::mp::util::RvizAgent::Instance().Ok()) {
    return;
  }
  adsfi_proto::viz::PointCloud lane_points;
  static uint32_t seq = 0;
  int curr_seq = seq++;
  lane_points.mutable_header()->set_seq(curr_seq);
  lane_points.mutable_header()->mutable_timestamp()->set_sec(time_sec_);
  lane_points.mutable_header()->mutable_timestamp()->set_nsec(time_nsec_);
  lane_points.mutable_header()->set_frameid("map");
  auto* channels = lane_points.add_channels();
  channels->set_name("rgb");
  for (const auto& p : points) {
    auto* points_ = lane_points.add_points();
    points_->set_x(p.x());
    points_->set_y(p.y());
    points_->set_z(p.z());
  }
  hozon::mp::util::RvizAgent::Instance().Publish(kTopicMmConnectPercepPoints,
                                                 lane_points);
}

void MapMatching::pubConnectMapPoints(const VP& points, uint64_t sec,
                                      uint64_t nsec) {
  if (hozon::mp::util::RvizAgent::Instance().Ok()) {
    adsfi_proto::viz::PointCloud lane_points;
    static uint32_t seq = 0;
    int curr_seq = seq++;
    lane_points.mutable_header()->set_seq(curr_seq);
    lane_points.mutable_header()->mutable_timestamp()->set_sec(time_sec_);
    lane_points.mutable_header()->mutable_timestamp()->set_nsec(time_nsec_);
    lane_points.mutable_header()->set_frameid("map");
    auto* channels = lane_points.add_channels();
    channels->set_name("rgb");
    for (const auto& p : points) {
      auto* points_ = lane_points.add_points();
      points_->set_x(p.x());
      points_->set_y(p.y());
      points_->set_z(p.z());
    }
    hozon::mp::util::RvizAgent::Instance().Publish(kTopicMmConnectMapPoints,
                                                   lane_points);
  }
}

void MapMatching::pubOriginConnectPercepPoints(const VP& points, uint64_t sec,
                                               uint64_t nsec) {
  if (hozon::mp::util::RvizAgent::Instance().Ok()) {
    adsfi_proto::viz::PointCloud lane_points;
    static uint32_t seq = 0;
    int curr_seq = seq++;
    lane_points.mutable_header()->set_seq(curr_seq);
    lane_points.mutable_header()->mutable_timestamp()->set_sec(time_sec_);
    lane_points.mutable_header()->mutable_timestamp()->set_nsec(time_nsec_);
    lane_points.mutable_header()->set_frameid("map");
    auto* channels = lane_points.add_channels();
    channels->set_name("rgb");
    for (const auto& p : points) {
      auto* points_ = lane_points.add_points();
      points_->set_x(p.x());
      points_->set_y(p.y());
      points_->set_z(p.z());
    }
    hozon::mp::util::RvizAgent::Instance().Publish(
        kTopicMmOriginConnectPercepPoints, lane_points);
  }
}

void MapMatching::pubOriginConnectMapPoints(const VP& points, uint64_t sec,
                                            uint64_t nsec) {
  if (hozon::mp::util::RvizAgent::Instance().Ok()) {
    adsfi_proto::viz::PointCloud lane_points;
    static uint32_t seq = 0;
    int curr_seq = seq++;
    lane_points.mutable_header()->set_seq(curr_seq);
    lane_points.mutable_header()->mutable_timestamp()->set_sec(time_sec_);
    lane_points.mutable_header()->mutable_timestamp()->set_nsec(time_nsec_);
    lane_points.mutable_header()->set_frameid("map");
    auto* channels = lane_points.add_channels();
    channels->set_name("rgb");
    for (const auto& p : points) {
      auto* points_ = lane_points.add_points();
      points_->set_x(p.x());
      points_->set_y(p.y());
      points_->set_z(p.z());
    }
    hozon::mp::util::RvizAgent::Instance().Publish(
        kTopicMmOriginConnectMapPoints, lane_points);
  }
}

void MapMatching::setConnectPercepPoints(const Connect& connect,
                                         const SE3& T_W_V,
                                         VP& points) {  // NOLINT
  for (int i = 0; i < connect.lane_line_match_pairs.size(); ++i) {
    auto perception_point = connect.lane_line_match_pairs[i].pecep_pv;
    auto point = T_W_V * perception_point;
    points.emplace_back(point);
  }
}

void MapMatching::setConnectMapPoints(const Connect& connect, const SE3& T_W_V,
                                      VP& points) {  // NOLINT
  for (int i = 0; i < connect.lane_line_match_pairs.size(); ++i) {
    auto point = connect.lane_line_match_pairs[i].map_pw;
    points.emplace_back(point);
  }
}

void MapMatching::setOriginConnectPercepPoints(const Connect& connect,
                                               const SE3& T_W_V,
                                               VP& points) {  // NOLINT
  for (int i = 0; i < connect.lane_line_match_pairs.size(); ++i) {
    auto perception_point = connect.lane_line_match_pairs[i].pecep_pv;
    auto point = T_W_V * perception_point;
    points.emplace_back(point);
  }
}

void MapMatching::setOriginConnectMapPoints(const Connect& connect,
                                            const SE3& T_W_V,
                                            VP& points) {  // NOLINT
  for (int i = 0; i < connect.lane_line_match_pairs.size(); ++i) {
    auto point = connect.lane_line_match_pairs[i].map_pw;
    points.emplace_back(point);
  }
}

bool MapMatching::CheckLaneMatch(const SE3& T_delta_cur) {
  if (match_inited && (matched_lane_pair_size_ < 2) &&
      (bad_lane_match_count_ < mm_params.thre_continue_badmatch) &&
      (fabs(T_delta_cur.translation().y() - T_delta_last_.translation().y()) >
       mm_params.thre_delta_y_diff)) {
    HLOG_DEBUG << "CheckLaneMatch " << ins_timestamp_ << " "
               << matched_lane_pair_size_ << " " << bad_lane_match_count_ << " "
               << T_delta_cur.translation().y() << " "
               << T_delta_last_.translation().y();
    return false;
  }
  return true;
}
}  // namespace loc
}  // namespace mp
}  // namespace hozon
