/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <deque>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include "localization/data_type/base.hpp"
#include "localization/data_type/percept_data.hpp"
#include "localization/data_type/road_structure.hpp"
#include "localization/data_type/smart_ptr.hpp"
#include "localization/localization_param.hpp"
#include "semantic_mm/semantic_mm_locator.hpp"
#include "system/localization_backend.hpp"

namespace senseAD {
namespace localization {

class LocalizationBackendSMM
    : public LocalizationBackend,
      public BackendAdapter<smm::SemanticMMLocator,
                            std::shared_ptr<PerceptData>> {
 public:
  DEFINE_PTR(LocalizationBackendSMM)

  LocalizationBackendSMM() = default;
  LocalizationBackendSMM(const LocalizationBackendSMM&) = delete;
  LocalizationBackendSMM& operator=(const LocalizationBackendSMM&) = delete;
  virtual ~LocalizationBackendSMM() = default;

  void SetLocalMapData(uint64_t timestamp,
                       const std::shared_ptr<RoadStructure>& local_map_data);

  NavStatus GetCurrentLocatorStatus() const final;

 protected:
  adLocStatus_t InitLocator() final;

  adLocStatus_t InitLocEvaluator() final;

  adLocStatus_t InitDataBuffer() final;

  void SetThreadName() final;

  adLocStatus_t ResetSubData() final;

  adLocStatus_t SwitchOriginSubProc() final;

  adLocStatus_t LocatorProcess() final;

  bool SyncPerceptDatas(
      std::vector<std::shared_ptr<PerceptData>>* percept_datas);

  void UpdateNavStatus(uint64_t timestamp, double confidence);

 private:
  static constexpr size_t kMapDataWindowSize_ = 2;
  static constexpr size_t kConfDataWindowSize = 20;

  // local map datas
  std::mutex local_map_data_mutex_;
  boost::circular_buffer<std::pair<uint64_t, std::shared_ptr<RoadStructure>>>
      local_map_data_list_;

  // SMM localization confidences
  mutable std::mutex conf_mutex_;
  boost::circular_buffer<std::pair<uint64_t, double>> conf_list_;

  // SMM localization status
  std::atomic<NavStatus> smm_status_{NavStatus::FATAL_ACCURACY};
};

}  // namespace localization
}  // namespace senseAD
