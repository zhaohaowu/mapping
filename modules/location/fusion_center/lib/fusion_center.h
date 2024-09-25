/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： fusion_center.h
 *   author     ： zhaohaowu
 *   date       ： 2024.09
 ******************************************************************************/

#pragma once

#include <atomic>
#include <condition_variable>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <Sophus/se3.hpp>

#include "Eigen/src/Core/Matrix.h"
#include "depend/proto/dead_reckoning/dr.pb.h"
#include "depend/proto/localization/localization.pb.h"
#include "depend/proto/localization/node_info.pb.h"
#include "depend/proto/soc/chassis.pb.h"
#include "depend/proto/soc/sensor_imu_ins.pb.h"
#include "modules/location/fusion_center/lib/data_buffer.h"
#include "modules/location/fusion_center/lib/type.h"
#include "modules/util/include/util/geo.h"
namespace hozon {
namespace mp {
namespace loc {
namespace fc {

using hozon::dead_reckoning::DeadReckoning;
using hozon::localization::HafNodeInfo;
using hozon::localization::Localization;
using hozon::soc::Chassis;
using hozon::soc::ImuIns;

class FusionCenter {
 public:
  FusionCenter() = default;
  ~FusionCenter();
  bool Init(const std::string& config_file);
  void OnIns(const HafNodeInfo& ins);
  void OnDr(const DeadReckoning& dr);
  void OnChassis(const Chassis& chassis);
  void OnImu(const ImuIns& imu);
  void OnMm(const HafNodeInfo& mm);
  std::shared_ptr<Localization> GetFcOutput();

 private:
  void Run();
  void Clear();
  bool PreConditionCheck();
  bool StateInit();
  bool GetFirstFc();
  void BackEskf();
  void UpdateLatestFc();
  void StatePredict(double dt, const Predict::ConstPtr& cur_imu,
                    FcState* fc_state);
  Measure::ConstPtr GetNewIns(const Measure::ConstPtr& cur_ins,
                              const Measure& latest_mm,
                              const Eigen::Vector3d& refpoint);
  Eigen::Vector3d RotionMatrix2EulerAngle321(
      const Eigen::Matrix3d& rotation_matrix);
  void StateMeasure(const Measure::ConstPtr& measure, FcState* fc_state);
  Measure::ConstPtr GetChassisByTimestamp(double timestamp);
  Measure::ConstPtr GetInsByTimestamp(double timestamp);
  Predict::ConstPtr GetImuByTimestamp(
      const std::vector<Predict::ConstPtr>& imu_vector, double timestamp);
  Dr::ConstPtr GetDrByTimestamp(double timestamp);

 private:
  std::shared_ptr<MessageBuffer<Predict::ConstPtr>> imu_buffer_ = nullptr;
  std::shared_ptr<MessageBuffer<Measure::ConstPtr>> mm_buffer_ = nullptr;
  std::shared_ptr<MessageBuffer<Measure::ConstPtr>> ins_buffer_ = nullptr;
  std::shared_ptr<MessageBuffer<Measure::ConstPtr>> chassis_buffer_ = nullptr;
  std::shared_ptr<MessageBuffer<Dr::ConstPtr>> dr_buffer_ = nullptr;
  Option option_;
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

  Eigen::Vector3d fc_ref_point_ = Eigen::Vector3d::Zero();
  std::shared_ptr<std::thread> fc_thread_ = nullptr;
  std::atomic<bool> fc_thread_run_{false};
  std::condition_variable imu_cv_;
  std::mutex imu_cv_mutex_;
  std::vector<Predict::ConstPtr> imu_vector_;
  std::vector<Measure::ConstPtr> measure_vector_;
  std::map<double, FcState> fc_map_;
  std::mutex fc_map_mutex_;
  Predict latest_imu_;
  Measure latest_ins_;
  Measure latest_chassis_;
  Measure latest_mm_;
  FcState first_fc_;
};

}  // namespace fc
}  // namespace loc
}  // namespace mp
}  // namespace hozon
