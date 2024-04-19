/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <list>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

#include <boost/circular_buffer.hpp>

#include "common/thread_base.hpp"
#include "localization/data_type/base.hpp"
#include "localization/data_type/imu.hpp"
#include "localization/data_type/odomstate_info.hpp"
#include "localization/data_type/smart_ptr.hpp"
#include "localization/data_type/vehicle_info.hpp"
#include "localization/localization_param.hpp"

namespace senseAD {
namespace localization {

namespace dr {
class DRLocator;
}
class EvaluatorLocalization;
class EvaluatorMotion;
class LocalizationVisualizer;

// struct holds the dead reckoning localization, only fuse imu and wheel encoder
class LocalizationDeadReckoning : public ThreadBase {
 public:
  DEFINE_PTR(LocalizationDeadReckoning)

  LocalizationDeadReckoning() = default;
  LocalizationDeadReckoning(const LocalizationDeadReckoning&) = delete;
  LocalizationDeadReckoning& operator=(const LocalizationDeadReckoning&) =
      delete;
  ~LocalizationDeadReckoning();

  adLocStatus_t Init(const LocalizationParam& param);

  adLocStatus_t Restart();

  bool CheckNeedRestart() const;

  // @brief: set/get instance pointer
  void SetLocalizationVisualizer(
      std::shared_ptr<LocalizationVisualizer> visualizer);

  void SetCanData(uint64_t timestamp, const VehicleInfo& can_data);

  void SetImuData(uint64_t timestamp, const Imu& imu_data);

  // @brief: causion! this interface just for publish call
  adLocStatus_t GetOdomStateInfo(OdomStateInfo* info) const;

  adLocStatus_t QueryOdomPoseByTime(uint64_t timestamp,
                                    OdomState* odom_state) const;

  adLocStatus_t GetLatestState(OdomState* odom_state) const;

 protected:
  // @brief: thread main
  void Run();

  void DRProcess();

  // @brief: init module
  adLocStatus_t InitLocator();

  adLocStatus_t InitLocEvaluator();

  adLocStatus_t OnInitDataBuffer();

  void SetThreadName() final;

  // @brief: dr locator process
  adLocStatus_t DRLocatorProcess(OdomState* odom_state,
                                 bool* continue_to_process);

  // @brief: store odom state into window states
  OdomState StoreOdomState(const OdomState& newest_odom_state);

  // @brief: get current odom status
  OdomStatus GetCurrentOdomStatus(const OdomState& odom_state);

  // @brief: interpolate odom state
  void OdomStateInterp(const OdomState& s_ns, const OdomState& e_ns,
                       double factor, OdomState* ns) const;

  // @brief: convert odom state to odom state info
  adLocStatus_t ConvertOdomStateInfo(const OdomState& odom_state,
                                     OdomStateInfo* info) const;

 protected:
  static constexpr size_t kDataSizeUpperLimit = 20;

  // Localization config parameters
  LocalizationParam param_;

  // unique id for dr origin id, start from 0, plus by 1 if dr restart occurs
  static uint64_t dr_origin_id_;
  // flag for need restart
  std::atomic<bool> need_restart_{false};

  // last imu and can timestamp
  std::atomic<uint64_t> last_imu_timestamp_{0};
  std::atomic<uint64_t> last_can_timestamp_{0};

  // input datas
  std::mutex imu_mutex_;
  boost::circular_buffer<std::pair<uint64_t, Imu>> imu_data_list_;

  // output odom states
  mutable std::mutex window_odom_mutex_;
  boost::circular_buffer<OdomState> window_odom_states_;
  mutable std::atomic<uint64_t> last_get_timestamp_{0};  // for dr publish

  // DR locator and evaluator instances
  std::shared_ptr<dr::DRLocator> dr_locator_;
  std::shared_ptr<EvaluatorLocalization> dr_evaluator_;
  std::shared_ptr<EvaluatorMotion> dr_motion_evaluator_;

  // localization visualizer instance, shallow copy
  std::shared_ptr<LocalizationVisualizer> loc_visualizer_;
};

}  // namespace localization
}  // namespace senseAD
