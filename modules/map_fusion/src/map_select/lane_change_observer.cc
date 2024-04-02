/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： lane_change_observer.cc
 *   author     ： zhangrui
 *   date       ： 2024.02
 ******************************************************************************/
#include "map_fusion/map_select/lane_change_observer.h"

namespace hozon {
namespace mp {
namespace mf {

void LaneChangeObserver::Reset() {
  change_lane_state_ = ChangeLaneState::None;
  change_lane_delay_.Clear();
  is_left_change_ = false;
  is_right_change_ = false;
  change_count_ = 0;
}

// apollo::common::Status LaneChangeObserver::Init() {
//   navigation_hdmap::NavigationHdmapConfig config;
//   if (!common::GetProtoFromFile(FLAGS_navigation_hdmap_config_filename,
//                                 &config)) {
//     return apollo::common::Status(
//         apollo::common::ErrorCode::LOCALVIEW_MAP_LANE_ERROR,
//         "Unable to load relative map conf file: " +
//             FLAGS_navigation_hdmap_config_filename);
//   }
//   change_laneline_jump_width_ =
//       config.lanemarker_decider_config().change_laneline_jump_width();
//   better_quality_threshold_ =
//       config.lanemarker_quality_reliableforwarning_value();
//   good_quality_threshold_ = config.lanemarker_quality_not_reliable_value();
//   return apollo::common::Status::OK();
// }

std::pair<bool, bool> LaneChangeObserver::Observer(
    const ::hozon::perception::LaneInfo& left_lane,
    const ::hozon::perception::LaneInfo& right_lane) {
  const double max_lane_c0_diff = 0.5;
  bool is_left_change = false;
  bool is_right_change = false;
  auto left_c0 = left_lane.lane_param().cubic_curve_set(0).c0();
  auto left_quality = left_lane.confidence();
  auto right_c0 = right_lane.lane_param().cubic_curve_set(0).c0();
  auto right_quality = right_lane.confidence();
  change_lane_delay_.Deal(
      std::make_tuple(left_c0, left_quality, right_c0, right_quality));
  bool good_quality =
      left_quality > good_quality_threshold_ &&
      std::get<1>(change_lane_delay_.GetDelay(2)) > good_quality_threshold_ &&
      right_quality > good_quality_threshold_ &&
      std::get<3>(change_lane_delay_.GetDelay(2)) > good_quality_threshold_;
  bool start_change_lane_flag =
      (std::fabs(left_c0) < better_quality_threshold_ ||
       std::fabs(right_c0) < better_quality_threshold_) &&
      good_quality;
  const double left_c0_delta =
      std::fabs(left_c0 - std::get<0>(change_lane_delay_.GetDelay(1)));
  const double right_c0_delta =
      std::fabs(right_c0 - std::get<2>(change_lane_delay_.GetDelay(1)));
  bool jump_change = left_c0_delta > change_laneline_jump_width_ &&
                     right_c0_delta > change_laneline_jump_width_;
  HLOG_ERROR << "good_quality: " << good_quality
             << ", start_change_lane_flag: " << start_change_lane_flag
             << ", left_c0_delta: " << left_c0_delta
             << ", right_c0_delta: " << right_c0_delta
             << ", jump_change: " << jump_change << ",left_c0: " << left_c0
             << ",std::get<0>(change_lane_delay_.GetDelay(1))="
             << std::get<0>(change_lane_delay_.GetDelay(1));
  switch (change_lane_state_) {
    case ChangeLaneState::None:
      change_count_ = 0;
      if (start_change_lane_flag) {
        change_lane_state_ = ChangeLaneState::Start;
      }
      break;
    case ChangeLaneState::Start:
      if (!good_quality ||
          (std::fabs(left_c0) > 1.0 && std::fabs(right_c0) > 1.0)) {
        change_lane_state_ = ChangeLaneState::None;
      } else if (good_quality && jump_change) {
        change_lane_state_ = ChangeLaneState::In;
        is_left_change = is_left_change_;
        is_right_change = is_right_change_;
      }
      break;
    case ChangeLaneState::In:
      if (!good_quality) {
        change_lane_state_ = ChangeLaneState::None;
      } else {
        if (left_c0_delta < max_lane_c0_diff &&
            right_c0_delta < max_lane_c0_diff) {
          change_count_++;
        } else {
          change_count_ = 0;
        }

        if (change_count_ > 2) {
          change_lane_state_ = ChangeLaneState::End;
        } else {
          is_left_change = is_left_change_;
          is_right_change = is_right_change_;
        }
      }
      break;
    case ChangeLaneState::End:
      change_count_ = 0;
      change_lane_state_ = ChangeLaneState::None;
      break;
    default:
      break;
  }

  if (change_lane_state_ == ChangeLaneState::Start) {
    if (std::fabs(left_c0) > std::fabs(right_c0)) {
      is_right_change_ = true;
    } else {
      is_left_change_ = true;
    }
  } else if (change_lane_state_ == ChangeLaneState::End ||
             change_lane_state_ == ChangeLaneState::None) {
    is_right_change_ = false;
    is_left_change_ = false;
  }

  HLOG_ERROR << "is_lane_change:change_state: " << change_lane_state_
             << ", is_left_change: " << is_left_change
             << ", is_right_change: " << is_right_change
             << ", count: " << change_count_;
  return std::make_pair(is_left_change, is_right_change);
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
