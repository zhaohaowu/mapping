/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： pose_estimation.h
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <adsfi_proto/viz/geometry_msgs.pb.h>
#include <adsfi_proto/viz/nav_msgs.pb.h>
#include <adsfi_proto/viz/sensor_msgs.pb.h>
#include <adsfi_proto/viz/visualization_msgs.pb.h>
#include <google/protobuf/util/json_util.h>
#include <yaml-cpp/yaml.h>

#include <condition_variable>
#include <deque>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "depend/proto/localization/node_info.pb.h"
#include "depend/proto/map/map.pb.h"
#include "modules/location/pose_estimation/lib/hd_map/hd_map.h"
#include "modules/location/pose_estimation/lib/perception/perception.h"
#include "modules/location/pose_estimation/lib/pose_estimate/pose_estimate.h"
#include "modules/location/pose_estimation/lib/pose_estimate/pose_estimate_solver.h"
#include "modules/location/pose_estimation/lib/util/globals.h"
#include "modules/location/pose_estimation/lib/util/timer.h"
#include "modules/util/include/util/geo.h"

namespace hmu = hozon::mp::util;
namespace hozon {
namespace mp {
namespace loc {

using hozon::localization::HafNodeInfo;

struct SensorSync {
  int status = 0;
  int32_t frame_id = 0;
  hozon::perception::TransportElement transport_element;
  //   ::perception::ObjectList object;
  void setFinsh() { status = 1; }
  bool ok() { return status != 0; }
};

struct InsMsg {
  InsMsg(const Sophus::SE3d &pose, double stamp) : pose(pose), stamp(stamp) {}
  InsMsg() {
    pose = Sophus::SE3d(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
    stamp = 0;
  }
  Sophus::SE3d pose;
  double stamp;
};

struct MmOptMsg {
  MmOptMsg(const Sophus::SE3d &pose, double stamp) : pose(pose), stamp(stamp) {}
  MmOptMsg() {
    pose = Sophus::SE3d(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
    stamp = 0;
  }
  Sophus::SE3d pose;
  double stamp;
};

struct INTEGRAL_INFO {
  INTEGRAL_INFO() {
    stamp = 0.f;
    x = 0.0;
    y = 0.0;
    vx = 0.0;
    vy = 0.0;
    yaw = 0.0;
    speed = 0.0;
    yawrate = 0.0;
  }

  double stamp;
  double x;
  double y;
  double vx;
  double vy;
  double yaw;
  double speed;
  double yawrate;
};

class MapMatchingFrameRateRecord {
 public:
  void CalFrameRate(const double &ts, const std::string &prefix) {
    static double last_ts = ts;
    static int cnt = 0;
    static int fps = 0;
    cnt++;
    if (ts - last_ts > 6) {
      fps = cnt / (ts - last_ts);
      cnt = 0;
      last_ts = ts;
      HLOG_ERROR << prefix << " : " << fps;
    }
  }
};

class MapMatching {
 public:
  MapMatching()
      : ins_status_type_(static_cast<int>(InsStatus::INVALID)),
        delay_frame_(0),
        max_frame_buf_(0),
        optimize_success_(false),
        output_valid_(false),
        proc_stamp_(0.f),
        ins_input_ready_(false),
        output_stamp_(0.f) {
    map_match_ = std::make_shared<hozon::mp::loc::MapMatch>();
    T_output_ = SE3();
  }
  ~MapMatching();

  bool Init(const std::string &config_file, const std::string &cfg_cam_path);
  void OnIns(
      const std::shared_ptr<const ::hozon::localization::HafNodeInfo> &msg);
  void OnPerception(
      const std::shared_ptr<const ::hozon::perception::TransportElement> &msg);
  // void OnLocation(const std::shared_ptr<const ::location::HafLocation> &msg);
  // void OnMarkPole(
  //     const std::shared_ptr<const
  //     ::adsfi_proto::hz_Adsfi::AlgLaneDetectionOut>
  //         &msg);
  // void OnObjectList(const std::shared_ptr<const ::perception::ObjectList>
  // &msg);

  std::tuple<bool, SensorSync> sensorFront(void);
  void eraseFront(bool is_erase_buf);
  void sensorPush(
      const ::hozon::perception::TransportElement &transport_element,
      bool is_roadmark);
  // void sensorPush(const ::perception::ObjectList &object);
  unsigned int sensorSize();

  void setSubMap(const Eigen::Vector3d &vehicle_position,
                 const Eigen::Matrix3d &vehicle_rotation);
  void setFrontRoadMark(const ::hozon::perception::TransportElement &roadmark,
                        bool is_roadmark);
  void setLocation(const ::hozon::localization::Localization& info);
  void OnLocation(const std::shared_ptr<const ::hozon::localization::Localization>& msg);

  // void setObject(const ::perception::ObjectList &object);
  void setIns(const ::hozon::localization::HafNodeInfo &ins);
  void interpolateOptimizeResult();
  void mmProcCallBack(void);
  // void mmInterpCallBack(void);
  void procData();
  bool smoothResult(const Sophus::SE3d &pose);
  void start(void);
  void reset(void);
  std::shared_ptr<::hozon::localization::HafNodeInfo> getMmNodeInfo();
  std::shared_ptr<::hozon::localization::HafNodeInfo> generateNodeInfo(
      const Sophus::SE3d &T_W_V, uint64_t sec, uint64_t nsec,
      const bool &has_err);
  inline double normalizeAngle(double z) { return atan2(sin(z), cos(z)); }
  inline double transAngleValue2ZeroToTwoPi(double z) {
    return (z < 0) ? (M_PI * 2 + z) : z;
  }
  inline double transAngleValue2MinusPiToPi(double z) {
    return (z > M_PI) ? (z - M_PI * 2) : z;
  }

 public:
  std::list<std::shared_ptr<SensorSync>> roadmark_sensor_;
  std::list<std::shared_ptr<SensorSync>> pole_sensor_;
  std::list<std::shared_ptr<SensorSync>> object_sensor_;
  hozon::mp::loc::Map<hozon::hdmap::Map> mhd_map_;

  std::thread proc_thread_;
  std::thread interp_thread_;
  bool proc_thread_run_;
  bool interp_thread_run_;

  INTEGRAL_INFO integral_info_;
  // hozon::mp::loc::Tracking tracking;
  std::shared_ptr<hozon::mp::loc::MapMatch> map_match_;
  // std::shared_ptr<Project> project;

  Eigen::Vector3d ref_point_;
  Eigen::Vector3d esti_ref_point_;
  Eigen::Vector3f _att;
  Eigen::Vector3d map_pos_;
  Eigen::Quaterniond map_rot_;

  SE3 T02_W_V_;
  SE3 T02_W_V_pre_;
  SE3 T02_W_V_last_;
  SE3 T84_W_V_;
  SE3 T_fine_;
  SE3 T_output_;
  SE3 T02_W_VF_last_;

  std::mutex road_mark_mutex_;
  std::mutex map_mutex_;
  std::mutex object_mutex_;
  std::mutex pole_mutex_;
  std::mutex mm_proc_lck_;
  std::mutex mm_output_lck_;
  std::mutex ins_msg_lck_;
  std::mutex map_lck_;
  std::condition_variable ins_input_cv_;

  double proc_stamp_last_ = -1.0;
  double ins_altitude_ = 0.f;

  InsMsg newest_ins_msg_;
  InsMsg mm_nearest_ins_msg_;

  std::string hdmap_topic_;
  std::string location_topic_;
  std::string road_marking_topic_;
  std::string node_info_topic_;

  Timer time_;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  void setPoints(const PerceptionLaneLineList &line_list, const SE3 &T_W_V,
                 VP *points);
  void pubPoints(const VP &points, const uint64_t &sec, const uint64_t &nsec);
  void pubOdomPoints(const std::string &topic, const Eigen::Vector3d &trans,
                     const Eigen::Quaterniond &q, uint64_t sec, uint64_t nsec);
  void pubVehicle(const SE3 &T, const double &sec, const double &nsec);
  void pubTimeAndInsStatus(const SE3 &T, double stamp);
  void pubMatchPoints(const VP &points);
  void PubMatchPoints(const VP &points, const uint64_t &sec,
                      const uint64_t &nsec, const std::string &topic_name);

  adsfi_proto::viz::Marker laneToMarker(const VP &points, std::string id,
                                        bool is_points, bool is_center,
                                        float point_size = 1,
                                        bool is_boundary = false);
  adsfi_proto::viz::Marker lineIdToMarker(const V3 point, std::string id);
  adsfi_proto::viz::Marker poleToMarker(const VP &points, int id);
  adsfi_proto::viz::Marker trafficsignToMarker(const std::vector<VP> &vpoints,
                                               int id);
  adsfi_proto::viz::Marker roadmarkingToMarker(const std::vector<VP> &vpoints,
                                               int id);
  bool CheckLaneMatch(const SE3 &T_delta_cur);
  bool GetHdCurrLaneType(const Eigen::Vector3d& utm);


 private:
  int ins_status_type_;
  int delay_frame_;
  int max_frame_buf_;

  bool init_ = false;
  bool optimize_success_;
  bool output_valid_;
  bool ins_input_ready_;
  bool use_inter_;
  bool use_smooth_ = false;
  bool use_extrapolate_ = false;

  double map_crop_front_ = 1550.0;
  double map_crop_width_ = 300.0;
  double proc_stamp_;
  double output_stamp_;
  double ins_timestamp_ = -0.1;
  double stampe_ = -1;
  u_int64_t sec;
  u_int64_t nsec;
  uint32_t time_sec_ = 0;
  uint32_t time_nsec_ = 0;
  int64_t last_submap_seq_ = -1;
  Eigen::Vector3d fc_enu_pose_;

  std::mutex ins_mutex_;

  adsfi_proto::viz::TransformStamped geo_tf_;
  adsfi_proto::viz::Path gnss_gcj02_path_;
  adsfi_proto::viz::Path gnss_gcj02_inter_path_;

  VP front_points_;
  std::deque<SE3> se3_buffer_;

  // 车道线匹配效果校验相关
  int matched_lane_pair_size_;
  SE3 T_delta_last_;
  int bad_lane_match_count_;
  bool match_inited;
  bool is_chging_ins_ref_ = false;
  bool is_chging_map_ref_ = false;
  bool is_toll_lane_ = false;
  bool is_ramp_road_ = false;
  bool is_main_road_ = false;
  const std::string kTopicMmTf = "/mm/tf";
  const std::string kTopicMmInterTf = "/mm/tf_inter";
  const std::string kTopicMmCarPath = "/mm/car_path";
  const std::string kTopicMmInterCarPath = "/mm/car_path_inter";
  const std::string kTopicMmFrontPoints = "/mm/front_point";
  const std::string kTopicInsOdom = "/mm/ins_odom";
  const std::string kTopicMmOdom = "/mm/mm_odom";
  const std::string kTopicFcOdom = "/mm/fc_odom";
  const std::string kTopicInputOdom = "/mm/input_odom";
  const std::string kTopicMmMatchPoints = "/mm/link";
  const std::string KTopicMmHdMap = "/mm/hd_map";
  const std::string kTopicMmTimeStamp = "/mm/time_stamp";
  const std::string kTopicMmPerceptionPointsEdge = "/mm/perception_point_edge";
  const std::string kTopicMmMapPointsEdge = "/mm/map_point_edge";
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
