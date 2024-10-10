/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： base_lane_loc.h
 *   author     ： zhaohaowu
 *   date       ： 2024.07
 ******************************************************************************/
#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cfloat>
#include <cstdlib>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "base/utils/log.h"
#include "depend/proto/localization/node_info.pb.h"
#include "modules/map_fusion/base/group.h"

namespace hozon {
namespace mp {
namespace mf {
namespace lane_loc {

using Groups = std::vector<Group::Ptr>;
using GroupsPtr = std::shared_ptr<Groups>;
using HafNodeInfo = hozon::localization::HafNodeInfo;
using HafNodeInfoPtr = std::shared_ptr<HafNodeInfo>;
using HafNodeInfoConstPtr = std::shared_ptr<const HafNodeInfo>;

struct Section {
  struct Lane {
    enum LineType {
      UNKNOWN = 0,
      DOTTED_YELLOW = 1,
      DOTTED_WHITE = 2,
      SOLID_YELLOW = 3,
      SOLID_WHITE = 4,
      DOUBLE_YELLOW = 5
    };
    std::string lane_id;
    struct LaneLine {
      std::string line_id;
      bool is_near_road_edge;
      LineType line_type;
      std::vector<Eigen::Vector3d> points;
      double y_min_abs_x;
    };
    LaneLine left_line;
    LaneLine right_line;
    bool is_ego_lane;
    std::vector<std::string> left_lane_ids;
    std::vector<std::string> right_lane_ids;
  };
  Lane ego_lane;
  Lane leftest_lane;
  Lane rightest_lane;
  std::map<double, Lane, std::greater<>> sorted_lanes;
};

using SectionPtr = std::shared_ptr<Section>;

struct LaneLocInfo {
  struct NextLaneInfo {
    Section::Lane next_lane;
    int lane_num = 0;
  };
  std::vector<NextLaneInfo> next_lanes_info;
  int cur_lane_index;
};

enum TurnState { STARIGHT = 0, TURN_LEFT = 1, TURN_RIGHT = 2 };

class BaseLaneLoc {
 public:
  static Group::Ptr GetVehicleGroup(const Groups& groups) {
    Group::Ptr vehicle_group_ptr = nullptr;
    for (const auto& group_ptr : groups) {
      if (group_ptr->start_slice.po.x() <= 0 &&
          group_ptr->end_slice.po.x() >= 0) {
        vehicle_group_ptr = group_ptr;
        break;
      }
    }
    return vehicle_group_ptr;
  }

  static Lane::Ptr GetVehicleLane(const Groups& groups) {
    // 找到车辆所在group
    Group::Ptr vehicle_group_ptr = GetVehicleGroup(groups);
    if (vehicle_group_ptr == nullptr) {
      HLOG_WARN << "can not find vehicle group";
      return nullptr;
    }

    // 找到车辆所在车道
    Lane::Ptr vehicle_lane_ptr = nullptr;
    for (const auto& lane : vehicle_group_ptr->lanes) {
      if (lane == nullptr) {
        HLOG_INFO << "why lane from groups is nullptr";
        continue;
      }
      auto left_y_ptr = GetYForMinAbsX(lane->left_boundary);
      if (left_y_ptr == nullptr) {
        HLOG_WARN << "left_boundary points size is 0";
        continue;
      }
      auto right_y_ptr = GetYForMinAbsX(lane->right_boundary);
      if (right_y_ptr == nullptr) {
        HLOG_WARN << "right_boundary points size is 0";
        continue;
      }
      if (*left_y_ptr > 0 && *right_y_ptr < 0) {
        vehicle_lane_ptr = lane;
        break;
      }
    }
    return vehicle_lane_ptr;
  }

  static Lane::Ptr GetLeftestLane(const Groups& groups) {
    // 找到车辆所在group
    Group::Ptr vehicle_group_ptr = GetVehicleGroup(groups);
    if (vehicle_group_ptr == nullptr) {
      HLOG_WARN << "can not find vehicle group";
      return nullptr;
    }

    // 找到车辆最左侧车道
    Lane::Ptr leftest_lane_ptr = nullptr;
    for (const auto& lane : vehicle_group_ptr->lanes) {
      if (lane->left_lane_str_id_with_group.empty()) {
        leftest_lane_ptr = lane;
      }
    }
    return leftest_lane_ptr;
  }

  static Lane::Ptr GetRightestLane(const Groups& groups) {
    // 找到车辆所在group
    Group::Ptr vehicle_group_ptr = GetVehicleGroup(groups);
    if (vehicle_group_ptr == nullptr) {
      HLOG_WARN << "can not find vehicle group";
      return nullptr;
    }

    // 找到车辆最右侧车道
    Lane::Ptr rightest_lane_ptr = nullptr;
    for (const auto& lane : vehicle_group_ptr->lanes) {
      if (lane->right_lane_str_id_with_group.empty()) {
        rightest_lane_ptr = lane;
      }
    }
    return rightest_lane_ptr;
  }

  static std::shared_ptr<double> GetYForMinAbsX(
      const LineSegment::Ptr& boundary) {
    if (boundary == nullptr) {
      HLOG_INFO << "why boundary is nullpty";
      return nullptr;
    }
    std::shared_ptr<double> y_ptr = nullptr;
    double min_abs_x = FLT_MAX;
    for (const auto& point : boundary->pts) {
      if (std::abs(point.pt.x()) < min_abs_x) {
        min_abs_x = std::abs(point.pt.x());
        y_ptr = std::make_shared<double>(point.pt.y());
      }
    }
    return y_ptr;
  }
};

}  // namespace lane_loc
}  // namespace mf
}  // namespace mp
}  // namespace hozon
