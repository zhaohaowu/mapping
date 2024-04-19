/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#include "system/localization_backend.hpp"

#include <memory>

#include "localization/common/log.hpp"
#include "system/localization_dead_reckoning.hpp"
#include "system/localization_frontend.hpp"
#include "system/localization_visualizer.hpp"

namespace senseAD {
namespace localization {

LocalizationBackend::~LocalizationBackend() {
  if (thread_ && thread_->joinable()) thread_->join();
}

adLocStatus_t LocalizationBackend::Init(const LocalizationParam& param) {
  param_ = param;

  // init locator
  if (InitLocator() != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to init backend locator.";
    return LOC_LOCALIZATION_ERROR;
  }

  // init evaluator
  if (param_.ci_param.enable_evaluation) {
    if (InitLocEvaluator() != LOC_SUCCESS) {
      LC_LERROR(SYSTEM) << "Failed to init backend evaluator.";
      return LOC_LOCALIZATION_ERROR;
    }
  }

  // init data buffer
  if (InitDataBuffer() != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to init backend data buffer.";
    return LOC_LOCALIZATION_ERROR;
  }

  // init thread
  thread_.reset(new std::thread(&LocalizationBackend::Run, this));
  if (nullptr == thread_) {
    LC_LERROR(SYSTEM) << "failed to create backend thread";
    return LOC_LOCALIZATION_ERROR;
  }
  SetThreadName();

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationBackend::Restart() {
  // restart sub data
  if (ResetSubData() != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to reset backend sub data.";
    return LOC_LOCALIZATION_ERROR;
  }

  // restart locator
  if (InitLocator() != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to restart backend locator.";
    return LOC_LOCALIZATION_ERROR;
  }

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationBackend::SwitchOriginProc() {
  // switch origin sub process
  if (SwitchOriginSubProc() != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to switch origin sub process.";
    return LOC_LOCALIZATION_ERROR;
  }

  return LOC_SUCCESS;
}

void LocalizationBackend::SetMSFFusion(
    std::shared_ptr<msf::MSFFusion> msf_fusion) {
  msf_fusion_ = msf_fusion;
}

void LocalizationBackend::SetLocalizationFrontend(
    std::shared_ptr<LocalizationFrontend> fe) {
  loc_frontend_ = fe;
}

void LocalizationBackend::SetLocalizationDeadReckoning(
    std::shared_ptr<LocalizationDeadReckoning> dr) {
  loc_dr_ = dr;
}

void LocalizationBackend::SetLocalizationVisualizer(
    std::shared_ptr<LocalizationVisualizer> visualizer) {
  loc_visualizer_ = visualizer;
}

void LocalizationBackend::Run() {
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // check finish request
    if (CheckFinishRequest()) break;

    // check pause request
    if (CheckPauseRequest()) {
      SetPause();
      while (IsPaused()) WaitReleasePaused(1000);
    }

    // backend process core
    if (LOC_SUCCESS != LocatorProcess()) continue;
  }
  SetFinish();
}

}  // namespace localization
}  // namespace senseAD
