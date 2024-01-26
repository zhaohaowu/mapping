// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: LiGuo (liguo03@hozon.com)
// @file: lane_target.h
// @brief: lane target head file

#pragma once
#include <algorithm>
#include <atomic>
#include <boost/circular_buffer.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/laneline_postprocess/lib/laneline/proto/lane_postprocess.pb.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/datalogger/load_data_singleton.h"
#include "modules/laneline_postprocess/lib/laneline/utils/lane_utils.h"
#include "perception-base/base/measurement/laneline_measurement.h"
#include "perception-base/base/scene/laneline.h"
#include "perception-base/base/utils/log.h"

namespace hozon {
namespace mp {
namespace environment {

using namespace hozon::perception;

struct LaneTargetInitOption {
  LaneTargetParam lane_target_param;
};

// @brief TrackStatus, descript the track life cycle
enum class TrackStatus {
  INIT = 0,
  TRACKED = 1,
  LOST = 2,
  DIE = 3,
  MAX_TRACK_STATUS_NUM = 4
};

class LaneTarget {
 public:
  LaneTarget();
  ~LaneTarget();

  bool Init(const LaneTargetInitOption& options,
            const base::LaneLineMeasurementPtr& detected_lane_line);

  void UpdateWithDetectedLaneLine(
      const base::LaneLineMeasurementPtr& detected_lane_line);

  void UpdateWithoutDetectedLaneLine();

  void Reset();

  inline const base::LaneLinePtr GetConstTrackedLaneLine() const {
    return tracked_laneline_;
  }

  inline base::LaneLinePtr& GetTrackedLaneLine() { return tracked_laneline_; }

  inline bool IsInit() const { return track_status_ == TrackStatus::INIT; }

  inline bool IsLost() const { return track_status_ == TrackStatus::LOST; }

  inline bool IsDie() const { return track_status_ == TrackStatus::DIE; }

  inline bool IsTracked() const {
    return track_status_ == TrackStatus::TRACKED;
  }

  inline void SetLastestTrackedTimestamp(const double& timestamp) {
    lastest_tracked_timestamp_ = timestamp;
  }

  inline double GetLastestTrackedTimestamp() const {
    return lastest_tracked_timestamp_;
  }

  inline void UpdateTrackStatus(bool cur_lost) {
    if (cur_lost) {
      // for init and lost status, set init status
      if (lost_age_ > lane_target_param_.reserve_age()) {
        track_status_ = TrackStatus::DIE;
      } else {
        track_status_ = tracked_count_ < lane_target_param_.tracked_init_life()
                            ? TrackStatus::INIT
                            : TrackStatus::LOST;
      }
    } else {
      track_status_ = tracked_count_ < lane_target_param_.tracked_init_life()
                          ? TrackStatus::INIT
                          : TrackStatus::TRACKED;
    }
  }

  inline void UpdateTrackStatus(bool cur_lost,
                                const base::LaneLinePtr& tracked_lane_line_) {
    int reserve_age = lane_target_param_.reserve_age();
    InputDataSingleton* local_data = InputDataSingleton::Instance();
    if (local_data->GetTurnState()) {
      reserve_age = 1;
    }
    int init_life = lane_target_param_.tracked_init_life();
    if (cur_lost) {
      // for init and lost status, set init status
      if (lost_age_ > reserve_age) {
        track_status_ = TrackStatus::DIE;
      } else {
        track_status_ =
            tracked_count_ < init_life ? TrackStatus::INIT : TrackStatus::LOST;
      }
    } else {
      track_status_ =
          tracked_count_ < init_life ? TrackStatus::INIT : TrackStatus::TRACKED;
    }
  }

  inline void SetLostAge(int lost_age) { lost_age_ = lost_age; }

  inline const int Id() const { return id_; }

  inline const int Count() const { return tracked_count_; }

  inline std::string ToStr() const {
    std::string des;
    des += "id_:" + std::to_string(id_);
    des += ",lost_age_:" + std::to_string(lost_age_);
    des += ",tracked_count_:" + std::to_string(tracked_count_);
    des += ",track_status:" + std::to_string(static_cast<int>(track_status_));
    // des += ", laneline_info:" + GetLaneLineInfo(tracked_lane_line_);

    return des;
  }

 private:
  // tracked_lane_object
  base::LaneLinePtr tracked_laneline_ = nullptr;

  int id_ = 0;
  int lost_age_ = 0;
  int tracked_count_ = 0;
  TrackStatus track_status_ = TrackStatus::MAX_TRACK_STATUS_NUM;
  double start_tracked_timestamp_ = 0.0;
  double lastest_tracked_timestamp_ = 0.0;
  double tracking_time_ = 0.0;

 private:
  static std::atomic<int> s_global_track_id_;

  LaneTargetParam lane_target_param_;
};

typedef std::shared_ptr<LaneTarget> LaneTargetPtr;
typedef std::shared_ptr<const LaneTarget> LaneTargetConstPtr;

}  // namespace environment
}  // namespace mp
}  // namespace hozon
