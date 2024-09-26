/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： pose_estimation.h
 *   author     ： nihongjie
 *   date       ： 2024.04
 ******************************************************************************/

#pragma once
#include <Eigen/Dense>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <boost/filesystem.hpp>

#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Geometry/Transform.h"
#include "depend/proto/local_mapping/local_map.pb.h"
#include "modules/location/pose_estimation/lib/map_matching.h"
#include "modules/location/pose_estimation/lib/reloc/reloc.hpp"
#include "proto/local_mapping/local_map.pb.h"
#include "proto/localization/localization.pb.h"
#include "proto/localization/node_info.pb.h"
namespace hozon {
namespace mp {
namespace loc {
namespace pe {

using HafNodeInfo = hozon::localization::HafNodeInfo;
using Localization = hozon::localization::Localization;
using LocalMap = hozon::mapping::LocalMap;
using LocalizationPtr = std::shared_ptr<Localization>;
using LaneInfoPtr = hozon::hdmap::LaneInfoConstPtr;

class PoseEstimation {
 public:
  PoseEstimation();
  ~PoseEstimation();
  PoseEstimation(const PoseEstimation& other);
  PoseEstimation& operator=(const PoseEstimation& other);
  PoseEstimation(PoseEstimation&& other) noexcept;
  PoseEstimation& operator=(PoseEstimation&& other) noexcept;

  bool LoadParams(const std::string& configfile);
  bool Init(const std::string& pose_estimation_yaml,
            const std::string& map_matching_yaml);
  void OnIns(const std::shared_ptr<const HafNodeInfo>& msg);
  void OnPerception(const std::shared_ptr<const LocalMap>& msg);
  void OnLocation(const std::shared_ptr<const Localization>& msg);
  std::shared_ptr<HafNodeInfo> GetMmNodeInfo();
  bool GetHdMapLane(const LocalizationPtr& fc_pose_ptr,
                    std::vector<hozon::hdmap::LaneInfoConstPtr>* hdmap_lanes);

 private:
  void Gcj2Enu(const Eigen::Vector3d& ref_point, Localization* msg);
  LocalizationPtr GetFcPoseForTimestamp(const Eigen::Vector3d& ref_point,
                                        double timestamp);
  LocalizationPtr GetInsPoseForTimestamp(const Eigen::Vector3d& ref_point,
                                         double timestamp);
  bool CheckMmTrackingState();

  void ProcData();
  void RvizFunc(const LocalizationPtr& fc_pose_ptr,
                const TrackingManager& tracking_manager,
                const MappingManager& map_manager,
                const Eigen::Affine3d& T_input,
                const Eigen::Affine3d& T_fc_10hz, double per_timestamp);
  Eigen::Affine3d Localization2Eigen(const LocalizationPtr& pose_ptr);
  template <typename T>
  void ShrinkQueue(T* deque, int maxsize);
  template <typename T0, typename T1, typename T2>
  void Vector3dToEnu3d(const T0& front_vector, const T1& back_vector,
                       T2* const enu3d, const double& front_scale = 1,
                       const double& back_scale = 1);
  template <typename T0, typename T1, typename T2>
  void Vector4dToEnu4d(const T0& front_vector, const T1& back_vector,
                       T2* const enu3d, const double& front_scale = 1,
                       const double& back_scale = 1);
  template <typename T0, typename T1>
  void SetXYZ(const T0& t0, T1* const t1);
  template <typename T0, typename T1>
  void SetXYZW(const T0& t0, T1* const t1);
  bool CheckLocalizationState(const Localization& localization);
  void AddMapLine(const Eigen::Affine3d& T_V_W,
                  const Eigen::Vector3d& fc_ref_point,
                  const hozon::hdmap::LaneBoundary& lane_boundary,
                  MappingManager* map_manager);
  bool PercepConvert(const LocalMap& perception,
                     TrackingManager* tracking_manager);
  bool MapConvert(
      const Localization& localization, const Eigen::Vector3d& fc_ref_point,
      const std::vector<hozon::hdmap::LaneInfoConstPtr>& hdmap_lanes,
      MappingManager* map_manager);
  bool Pose2Eigen(const hozon::common::Pose& pose,
                  Eigen::Affine3d* const affine3d);
  bool ExtractInsMsg(const LocalizationPtr& cur_ins, Sophus::SE3d* T02_W_V_ins,
                     const Eigen::Vector3d& ref_point);
  Eigen::Vector3d RotionMatrix2EulerAngle321(
      const Eigen::Matrix3d& rotation_matrix);

 private:
  std::deque<Localization> ins_deque_;
  std::deque<Localization> fc_deque_;
  LocalMap perception_;

  std::mutex ins_deque_mutex_;
  std::mutex fc_deque_mutex_;
  std::mutex perception_mutex_;
  std::mutex ref_point_mutex_;
  std::mutex rviz_mutex_;
  std::mutex pe_cv_mutex_;

  std::condition_variable pe_cv_;

  int ins_deque_max_size_ = 100;
  int fc_deque_max_size_ = 100;
  int perception_deque_max_size_ = 2;

  std::atomic<bool> reloc_test_ = false;

  TrackingManager tracking_manager_;
  MappingManager map_manager_;

  std::thread proc_thread_;
  bool proc_thread_run_{false};
  Eigen::Vector3d ref_point_;

  std::unique_ptr<MapMatching> map_matching_ = nullptr;
  std::unique_ptr<Reloc> reloc_ = nullptr;

  class Ref {
   private:
    bool refpoint_inited_ = false;
    Eigen::Vector3d ref_point_ = Eigen::Vector3d::Zero();
    std::mutex ref_point_mutex_;

   public:
    bool init() {
      ref_point_mutex_.lock();
      bool init_flag = refpoint_inited_;
      ref_point_mutex_.unlock();
      return init_flag;
    }
    void UpdateRefpoint(const Eigen::Vector3d& gcj_position) {
      ref_point_mutex_.lock();
      if (!refpoint_inited_) {
        refpoint_inited_ = true;
        ref_point_ = gcj_position;
      }
      // enu系下的车辆运动超过10000米，重置refpoint
      Eigen::Vector3d enu_position =
          util::Geo::Gcj02ToEnu(gcj_position, ref_point_);
      if (enu_position.head<2>().norm() > 10000) {
        ref_point_ = gcj_position;
      }
      ref_point_mutex_.unlock();
    }
    Eigen::Vector3d GetRefPoint() {
      ref_point_mutex_.lock();
      Eigen::Vector3d ref_point = ref_point_;
      ref_point_mutex_.unlock();
      return ref_point;
    }
  };

  Ref ref_point;
};

}  // namespace pe
}  // namespace loc
}  // namespace mp
}  // namespace hozon
