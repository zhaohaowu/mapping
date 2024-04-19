/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <atomic>
#include <deque>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <boost/circular_buffer.hpp>

#include "common/thread_base.hpp"
#include "localization/data_type/base.hpp"
#include "localization/data_type/navstate_info.hpp"
#include "localization/data_type/percept_data.hpp"
#include "localization/data_type/smart_ptr.hpp"
#include "localization/localization_param.hpp"

namespace senseAD {
namespace localization {

namespace msf {
class MSFFusion;
}
class EvaluatorLocalization;
class LocalizationFrontend;
class LocalizationDeadReckoning;
class LocalizationVisualizer;

// struct holds the system localization backend running in internal thread,
// contains backend localize, add factor into msf etc.
class LocalizationBackend : public ThreadBase {
 public:
  DEFINE_PTR(LocalizationBackend)

  LocalizationBackend() = default;
  LocalizationBackend(const LocalizationBackend&) = delete;
  LocalizationBackend& operator=(const LocalizationBackend&) = delete;
  virtual ~LocalizationBackend();

  adLocStatus_t Init(const LocalizationParam& param);

  adLocStatus_t Restart();

  adLocStatus_t SwitchOriginProc();

  virtual NavStatus GetCurrentLocatorStatus() const = 0;

  // @brief: set external instance pointer
  void SetMSFFusion(std::shared_ptr<msf::MSFFusion> msf_fusion);
  void SetLocalizationFrontend(std::shared_ptr<LocalizationFrontend> fe);
  void SetLocalizationDeadReckoning(
      std::shared_ptr<LocalizationDeadReckoning> dr);
  void SetLocalizationVisualizer(
      std::shared_ptr<LocalizationVisualizer> visualizer);

 protected:
  // @brief: thread main
  void Run();

  virtual adLocStatus_t InitLocator() = 0;

  virtual adLocStatus_t InitLocEvaluator() = 0;

  virtual adLocStatus_t InitDataBuffer() = 0;

  virtual adLocStatus_t ResetSubData() = 0;

  virtual adLocStatus_t SwitchOriginSubProc() = 0;

  virtual adLocStatus_t LocatorProcess() = 0;

 protected:
  // Localization config parameters
  LocalizationParam param_;

  // external modules
  std::shared_ptr<EvaluatorLocalization> evaluator_;  // evaluator instance
  std::shared_ptr<msf::MSFFusion> msf_fusion_;  // msf instance, shallow copy
  // localization frontend instance, shallow copy
  std::weak_ptr<LocalizationFrontend> loc_frontend_;
  // localization DR instance, shallow copy
  std::shared_ptr<LocalizationDeadReckoning> loc_dr_;
  // localization visualizer instance, shallow copy
  std::shared_ptr<LocalizationVisualizer> loc_visualizer_;
};

template <typename Locator, typename InputData>
class BackendAdapter {
 public:
  DEFINE_PTR(BackendAdapter)

  BackendAdapter() = default;
  virtual ~BackendAdapter() = default;

  void InitCapacity() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    data_list_.set_capacity(kDataSizeUpperLimit);
  }

  void SetInputData(uint64_t timestamp, const InputData& in_data) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    data_list_.push_back(std::make_pair(timestamp, in_data));
  }

  void ClearData() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    data_list_.clear();
  }

 protected:
  static constexpr size_t kDataSizeUpperLimit = 10;

  std::mutex data_mutex_;
  boost::circular_buffer<std::pair<uint64_t, InputData>> data_list_;

  std::shared_ptr<Locator> locator_;  // locator instance
};

// template specification, as SMM should support multi camera perception
template <typename Locator>
class BackendAdapter<Locator, std::shared_ptr<PerceptData>> {
 public:
  DEFINE_PTR(BackendAdapter)

  BackendAdapter() = default;
  virtual ~BackendAdapter() = default;

  void InitCapacity(const std::vector<std::string>& cameras) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (auto camera : cameras)
      data_deques_[camera].set_capacity(kDataSizeUpperLimit);
  }

  void SetInputData(uint64_t timestamp,
                    const std::shared_ptr<PerceptData>& in_data) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto& data_dq = data_deques_[in_data->camera_name];
    data_dq.push_back(std::make_pair(timestamp, in_data));
  }

  void ClearData() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    data_deques_.clear();
  }

 protected:
  static constexpr size_t kDataSizeUpperLimit = 10;

  std::mutex data_mutex_;
  std::map<std::string, boost::circular_buffer<
                            std::pair<uint64_t, std::shared_ptr<PerceptData>>>>
      data_deques_;

  std::shared_ptr<Locator> locator_;  // locator instance
};

}  // namespace localization
}  // namespace senseAD
