// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: chenlongxi
// @file: lane_target.h
// @brief: base lane element target head file

#pragma once
#include <algorithm>
#include <atomic>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <boost/circular_buffer.hpp>

#include "modules/local_mapping/base/scene/arrow.h"
#include "modules/local_mapping/base/scene/laneline.h"
#include "modules/local_mapping/base/scene/roadedge.h"
#include "modules/local_mapping/base/scene/stopline.h"
#include "modules/local_mapping/base/scene/zebracrossing.h"
#include "modules/local_mapping/lib/interface/base_init_options.h"
#include "modules/util/include/util/mapping_log.h"
namespace hozon {
namespace mp {
namespace lm {

// @brief TrackStatus, descript the track life cycle
enum class TrackStatus {
  INIT = 0,
  TRACKED = 1,
  LOST = 2,
  DIE = 3,
  MAX_TRACK_STATUS_NUM = 4
};

template <typename Element>
class BaseTarget {
 public:
  using ElementPtr = std::shared_ptr<Element>;

 public:
  virtual ~BaseTarget() = default;
  bool InitBase(const ProcessOption& options,
                const ElementPtr& detected_element_ptr);
  void UpdateWithDetectedObject(const ProcessOption& options,
                                const ElementPtr& detected_element_ptr);
  void UpdateWithoutDetectedObject(const ProcessOption& options);
  void Reset();
  void UpdateTrackStatus(bool cur_lost);
  std::string ToStr() const;

  inline ElementPtr GetConstTrackedObject() const { return tracked_element_; }

  inline ElementPtr& GetTrackedObject() { return tracked_element_; }

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

  inline int GetLostAge() const { return lost_age_; }

  inline void SetLostAge(int lost_age) { lost_age_ = lost_age; }

  inline void SetDeleteFlag(bool delete_flag) { delete_flag_ = delete_flag; }
  inline bool GetDeleteFlag() const { return delete_flag_; }

  inline void SetDeletedTrackIds(const BaseTarget& delete_target) {
    if (this == &delete_target) {
      return;
    }
    // 先插入delete_target
    deleted_track_ids_.emplace_back(delete_target.id_,
                                    delete_target.lastest_tracked_timestamp_);
    // 去重插入因delete_target删除的ids
    for (const auto& id : delete_target.deleted_track_ids_) {
      bool find_flag = false;
      for (const auto& id_ : deleted_track_ids_) {
        if (id_.first == id.first) {
          find_flag = true;
          break;
        }
      }
      if (!find_flag) {
        deleted_track_ids_.push_back(id);
      }
    }
    std::sort(deleted_track_ids_.begin(), deleted_track_ids_.end(),
              [](const auto& left, const auto& right) {
                return left.second > right.second;
              });
    if (deleted_track_ids_.size() > 10) {
      deleted_track_ids_.resize(10);
    }
  }

  inline std::vector<int> GetDeletedTrackIds() const {
    std::vector<int> deleted_ids;
    deleted_ids.reserve(deleted_track_ids_.size());
    for (const auto& pair : deleted_track_ids_) {
      deleted_ids.push_back(pair.first);
    }
    return deleted_ids;
  }
  inline int Id() const { return id_; }

  inline int Count() const { return tracked_count_; }
  boost::circular_buffer<bool> lastest_n_tracked_state_;

 protected:
  ElementPtr tracked_element_ = nullptr;
  std::vector<std::pair<int, double>> deleted_track_ids_;
  int id_ = 0;
  int lost_age_ = 0;
  int tracked_count_ = 0;
  bool send_postlane_ = true;  // 是否后处理的车道线发送给定位用
  bool delete_flag_ = false;  // merge 或者删除逻辑标记是否被删除掉了
  TrackStatus track_status_ = TrackStatus::MAX_TRACK_STATUS_NUM;
  double start_tracked_timestamp_ = 0.0;
  double lastest_tracked_timestamp_ = 0.0;
  double tracking_time_ = 0.0;

 protected:
  // 后续需要放到配置文件中 TODO(张文海)。
  int reserve_age_ = 5;
  int tracked_init_life_ = 3;

  static std::atomic<int> s_global_track_id_;
};

template <typename Element>
std::atomic<int> BaseTarget<Element>::s_global_track_id_ = {0};

template <typename Element>
bool BaseTarget<Element>::InitBase(const ProcessOption& options,
                                   const ElementPtr& detected_element_ptr) {
  using baseType = BaseTarget<Element>;
  id_ = baseType::s_global_track_id_++;
  if (baseType::s_global_track_id_ >= std::numeric_limits<int>::max()) {
    baseType::s_global_track_id_ = 0;
    id_ = baseType::s_global_track_id_;
  }
  send_postlane_ = true;
  lost_age_ = 0;
  tracked_count_++;
  start_tracked_timestamp_ = options.timestamp;
  lastest_tracked_timestamp_ = options.timestamp;
  UpdateTrackStatus(false);
  return true;
}

template <typename Element>
void BaseTarget<Element>::UpdateTrackStatus(bool cur_lost) {
  send_postlane_ = false;
  if (cur_lost) {
    // for init and lost status, set init status
    // 跟踪状态车道线不删,
    // 它不会进入DIE状态，RemoveLostTracker函数需要做好Tracker删除
    if (track_status_ == TrackStatus::TRACKED) {
      send_postlane_ = lost_age_ <= reserve_age_;
      return;
    }
    if (lost_age_ > reserve_age_) {
      track_status_ = TrackStatus::DIE;
    } else {
      track_status_ = tracked_count_ < tracked_init_life_ ? TrackStatus::INIT
                                                          : TrackStatus::LOST;
    }
  } else {
    track_status_ = tracked_count_ < tracked_init_life_ ? TrackStatus::INIT
                                                        : TrackStatus::TRACKED;
  }
  send_postlane_ = track_status_ == TrackStatus::TRACKED;
}

template <typename Element>
std::string BaseTarget<Element>::ToStr() const {
  std::stringstream des;
  des << "id_:" << id_ << ",lost_age_:" << lost_age_
      << ",tracked_count_:" << tracked_count_
      << ",track_status:" << static_cast<int>(track_status_);
  return des.str();
}

template <typename Element>
void BaseTarget<Element>::Reset() {
  tracked_element_ = nullptr;
  id_ = 0;
  lost_age_ = 0;
  start_tracked_timestamp_ = 0.0;
  tracked_count_ = 0;
}

template <typename Element>
void BaseTarget<Element>::UpdateWithoutDetectedObject(
    const ProcessOption& options) {
  ++lost_age_;
  tracked_element_->lost_age = lost_age_;
  UpdateTrackStatus(true);
  if (track_status_ == TrackStatus::TRACKED) {
    tracked_element_->state = TrackState::MATURED;
  } else {
    tracked_element_->state = TrackState::NOTMATURED;
  }
  tracked_element_->lost_age = lost_age_;
  lastest_n_tracked_state_.push_back(false);
  HLOG_DEBUG << "lane_id:" << id_ << ",lastest_tracked_timestamp_"
             << lastest_tracked_timestamp_ << ", options.timestamp"
             << options.timestamp << ", lost_age_" << lost_age_;
}

template <typename Element>
void BaseTarget<Element>::UpdateWithDetectedObject(
    const ProcessOption& options, const ElementPtr& detected_element_ptr) {
  lost_age_ = 0;
  tracked_count_++;
  tracked_element_->tracked_count = tracked_count_;
  tracked_element_->lost_age = lost_age_;

  UpdateTrackStatus(false);
  if (track_status_ == TrackStatus::TRACKED) {
    tracked_element_->state = TrackState::MATURED;
  } else {
    tracked_element_->state = TrackState::NOTMATURED;
  }
  SetLastestTrackedTimestamp(options.timestamp);
  tracking_time_ = lastest_tracked_timestamp_ - start_tracked_timestamp_;
  lastest_n_tracked_state_.push_back(true);
  tracked_element_->tracking_time = tracking_time_;
  tracked_element_->latest_tracked_time = lastest_tracked_timestamp_;
}

// LaneTarget 定义
// 特殊函数可以在cc文件复写基类的函数
class LaneTarget : public BaseTarget<LaneLine> {
 public:
  bool Init(const ProcessOption& options,
            const LaneLinePtr& detected_lane_line);
  inline bool SendPostLane() const { return send_postlane_; }
};

using LaneTargetPtr = std::shared_ptr<LaneTarget>;
using LaneTargetConstPtr = std::shared_ptr<const LaneTarget>;

// RoadEdgeTarget 定义
// 特殊函数可以在cc文件可以复写基类的函数
class RoadEdgeTarget : public BaseTarget<RoadEdge> {
 public:
  bool Init(const ProcessOption& options,
            const RoadEdgePtr& detected_road_edge);
  inline bool SendPostLane() const { return send_postlane_; }
};

using RoadEdgeTargetPtr = std::shared_ptr<RoadEdgeTarget>;
using RoadEdgeTargetConstPtr = std::shared_ptr<const RoadEdgeTarget>;

// StopLineTarget 定义
// 特殊函数可以在cc文件可以复写基类的函数
class StopLineTarget : public BaseTarget<StopLine> {
 public:
  bool Init(const ProcessOption& options,
            const StopLinePtr& detected_stopline_ptr);
};

using StopLineTargetPtr = std::shared_ptr<StopLineTarget>;
using StopLineTargetConstPtr = std::shared_ptr<const StopLineTarget>;

// ArrowTarget 定义
// 特殊函数可以在cc文件可以复写基类的函数
class ArrowTarget : public BaseTarget<Arrow> {
 public:
  bool Init(const ProcessOption& options, const ArrowPtr& detected_Arrow_ptr);
};

using ArrowTargetPtr = std::shared_ptr<ArrowTarget>;
using ArrowTargetConstPtr = std::shared_ptr<const ArrowTarget>;

// ZebraCrossingTarget 定义
// 特殊函数可以在cc文件可以复写基类的函数
class ZebraCrossingTarget : public BaseTarget<ZebraCrossing> {
 public:
  bool Init(const ProcessOption& options,
            const ZebraCrossingPtr& detected_ZebraCrossing_ptr);
};

using ZebraCrossingTargetPtr = std::shared_ptr<ZebraCrossingTarget>;
using ZebraCrossingTargetConstPtr = std::shared_ptr<const ZebraCrossingTarget>;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
