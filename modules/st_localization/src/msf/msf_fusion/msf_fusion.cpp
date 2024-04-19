/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#include "msf/msf_fusion/msf_fusion.hpp"

#include "common/coordinate_converter.hpp"
#include "localization/common/log.hpp"
#include "msf/msf_factor/msf_factor.hpp"

namespace senseAD {
namespace localization {
namespace msf {

adLocStatus_t MSFFusion::SwitchOriginProc() {
  // convert enu position to new origin
  {
    std::lock_guard<std::mutex> lock(factor_mutex_);
    for (auto iter = obs_factors_.begin(); iter != obs_factors_.end(); ++iter) {
      auto obs_factor = iter->second;
      auto source = obs_factor->GetSource();
      if (source == SMM || source == MOCK) {
        Eigen::VectorXd state = obs_factor->GetObservationState();
        Eigen::Vector3d new_enu;
        CoordinateConverter::GetInstance()->ConvertENUToNewOrigin(
            state.segment<3>(0), &new_enu);
        state.segment<3>(0) = new_enu;
        obs_factor->SetObservationState(state);
      }
    }
  }
  if (LOC_SUCCESS != SwitchOriginSubProc()) {
    LC_LERROR(MSF) << "switch origin sub process failed.";
    return LOC_LOCALIZATION_ERROR;
  }

  return LOC_SUCCESS;
}

adLocStatus_t MSFFusion::AddFactor(
    uint64_t timestamp, const std::shared_ptr<MSFFactor>& obs_factor) {
  std::lock_guard<std::mutex> lock(factor_mutex_);
  if (obs_factors_.empty()) {
    obs_factors_.emplace_back(timestamp, obs_factor);
    return LOC_SUCCESS;
  }

  if (CompareTime(timestamp, obs_factors_.back().first) > 0) {
    obs_factors_.emplace_back(timestamp, obs_factor);
    return LOC_SUCCESS;
  }

  for (auto iter = obs_factors_.begin(); iter != obs_factors_.end(); ++iter) {
    if (CompareTime(timestamp, iter->first) < 0) {
      obs_factors_.insert(iter, std::make_pair(timestamp, obs_factor));
      break;
    } else if (CompareTime(timestamp, iter->first) == 0) {
      if ((*iter).second->GetSource() < obs_factor->GetSource()) {
        (*iter).second = obs_factor;
        LC_LDEBUG(MSF) << "factor has same timestamp, replace...";
        break;
      }
    }
  }
  return LOC_SUCCESS;
}

adLocStatus_t MSFFusion::StateUpdate() {
  if (latest_frontend_ts_ == 0) return LOC_SUCCESS;
  // do filter update
  std::lock_guard<std::mutex> lock(factor_mutex_);
  while (!obs_factors_.empty()) {
    // avoid factor timestamp is ahead of frontend timestamp
    if (obs_factors_.front().first > latest_frontend_ts_) break;
    StateUpdate(obs_factors_.front().first, obs_factors_.front().second);
    obs_factors_.pop_front();
  }
  return LOC_SUCCESS;
}

int MSFFusion::CompareTime(uint64_t base_time, uint64_t timestamp) {
  constexpr double time_tolerance = 0.005;
  if (base_time >= timestamp) {
    double timegap = (base_time - timestamp) * kNanoSecToSec;
    if (timegap > time_tolerance) return 1;
  } else {
    double timegap = (timestamp - base_time) * kNanoSecToSec;
    if (timegap > time_tolerance) return -1;
  }
  return 0;
}

}  // namespace msf
}  // namespace localization
}  // namespace senseAD
