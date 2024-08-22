/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： lane_loc.cc
 *   author     ： zhaohaowu
 *   date       ： 2024.07
 ******************************************************************************/
#include "modules/map_fusion_02/modules/lane_loc/lane_loc.h"

#include <algorithm>
#include <cfloat>
#include <functional>
#include <map>
#include <string>
#include <unordered_set>
#include <vector>

#include "Eigen/src/Geometry/Transform.h"
#include "base/utils/log.h"
#include "data_manager/ins_data_manager.h"
#include "depend/common/utm_projection/coordinate_convertor.h"
#include "modules/map_fusion_02/common/calc_util.h"
#include "modules/map_fusion_02/modules/lane_loc/base_lane_loc.h"
#include "modules/map_fusion_02/modules/lane_loc/detect_turn_state.h"
#include "modules/map_fusion_02/modules/lane_loc/fusion_lane_loc.h"
#include "modules/map_fusion_02/modules/lane_loc/measure_lane_loc.h"
#include "modules/map_fusion_02/modules/map_ld/include/global_ld_map.h"
#include "modules/rviz/map_fusion_rviz.h"
#include "modules/util/include/util/geo.h"
#include "util/rviz_agent/rviz_agent.h"
namespace hozon {
namespace mp {
namespace mf {
namespace lane_loc {
LaneLoc::LaneLoc() {
  measure_lane_loc_ptr_ = std::make_shared<MeasureLaneLoc>();
  detect_turn_state_ptr_ = std::make_shared<DetectTurnState>();
  fusion_lane_loc_ptr_ = std::make_shared<FusionLaneLoc>();
}

bool LaneLoc::UpdateGroups(Groups* groups_ptr) {
  TicToc tic;
  // 1. 校验输入是否有效, 并获取ref_point
  if (groups_ptr == nullptr) {
    HLOG_WARN << "1. groups_ptr is nullptr";
    return false;
  }
  auto ins_ptr = INS_MANAGER->GetIns();
  if (ins_ptr == nullptr) {
    HLOG_WARN << "1. ins_ptr is nullptr";
    return false;
  }

  // 2. 获取ref_point
  auto ref_point_ptr = GetRefPoint(ins_ptr);
  if (ref_point_ptr == nullptr) {
    HLOG_WARN << "2. get ref_point failed";
    return false;
  }

  Eigen::Vector3d t_ins = hozon::mp::util::Geo::Gcj02ToEnu(
      {ins_ptr->pos_gcj02().x(), ins_ptr->pos_gcj02().y(),
       ins_ptr->pos_gcj02().z()},
      *ref_point_ptr);
  Eigen::Quaterniond q_ins(ins_ptr->quaternion().w(), ins_ptr->quaternion().x(),
                           ins_ptr->quaternion().y(),
                           ins_ptr->quaternion().z());
  Eigen::Affine3d T_ins = Eigen::Translation3d(t_ins) * Eigen::Affine3d(q_ins);
  Eigen::Affine3d T_V_W = T_ins.inverse();
  timespec cur_time{};
  clock_gettime(CLOCK_REALTIME, &cur_time);
  auto sec = cur_time.tv_sec;
  auto nsec = cur_time.tv_nsec;
  MF_RVIZ->PubInsOdom(T_ins, sec, nsec, "/lanc_loc/ins_odom");
  MF_RVIZ->PubInsPath(T_ins, sec, nsec, "/lanc_loc/ins_path");
  MF_RVIZ->PubInsTf(T_ins, sec, nsec, "/lanc_loc/ins_tf");

  // 3. 获取感知section (将groups转换成当前车所在section)
  SectionPtr per_section_ptr = GetPerSection(*groups_ptr);
  if (per_section_ptr == nullptr) {
    HLOG_WARN << "3. get per_section_ptr failed";
    return false;
  }
  MF_RVIZ->PubPerSection(*per_section_ptr, sec, nsec, "/lane_loc/per_section");

  // 4. 获取地图所有车道
  std::vector<hdmap::LaneInfoConstPtr> ldmap_lanes = GetLdmapLanes(ins_ptr);
  if (ldmap_lanes.empty()) {
    HLOG_WARN << "4. get ldmap_lanes failed";
    return false;
  }

  // 5.获取地图section
  SectionPtr map_section_ptr =
      GetMapSection(ldmap_lanes, *ref_point_ptr, T_V_W);
  if (map_section_ptr == nullptr) {
    HLOG_WARN << "5. get map_section_ptr failed";
    return false;
  }
  MF_RVIZ->PubMapSection(*map_section_ptr, sec, nsec, "/lane_loc/map_section");

  // 6. 获取车道级定位的观测结果，每条车道的观测概率，观测权重，观测车道索引
  double measure_weight = 0;
  int measure_lane_index = 0;
  std::string road_edge_state;
  std::shared_ptr<std::vector<double>> p_measure_ptr =
      measure_lane_loc_ptr_->Run(
          &measure_weight, &measure_lane_index, &road_edge_state,
          static_cast<int>(map_section_ptr->sorted_lanes.size()),
          *map_section_ptr, *per_section_ptr);
  if (p_measure_ptr == nullptr) {
    HLOG_WARN << "6. get measure failed";
    return false;
  }

  // 7. 换道检测
  std::shared_ptr<TurnState> turn_state_ptr =
      detect_turn_state_ptr_->Run(*groups_ptr);
  if (turn_state_ptr == nullptr) {
    HLOG_WARN << "7. get turn state failed";
    return false;
  }

  // 8. 获取车道级定位的融合结果，每条车道的融合概率，预测概率，融合车道索引
  int fusion_lane_index = 0;
  std::vector<double> p_predict;
  std::shared_ptr<std::vector<double>> p_fusion_ptr =
      fusion_lane_loc_ptr_->Run(&fusion_lane_index, &p_predict, measure_weight,
                                p_measure_ptr, *turn_state_ptr);
  if (p_fusion_ptr == nullptr) {
    HLOG_WARN << "8. get fusion lane_index_ptr failed";
    return false;
  }

  // 9. 根据车道级定位结果获取当前车道的后继
  LaneLocInfo lane_loc_info =
      GetLaneLocInfo(fusion_lane_index, map_section_ptr, *ref_point_ptr, T_V_W);

  // rviz可视化
  MF_RVIZ->PubState(std::to_string(map_section_ptr->sorted_lanes.size()),
                    road_edge_state, std::to_string(measure_lane_index),
                    *p_measure_ptr, *turn_state_ptr, p_predict,
                    std::to_string(fusion_lane_index), *p_fusion_ptr, sec, nsec,
                    "/lanc_loc/state");
  MF_RVIZ->PubLaneLocInfo(lane_loc_info, sec, nsec, "/lane_loc/lane_loc_info");

  // HLOG_FATAL << "lane_loc " << tic.Toc() << " ms";
  return true;
}

std::shared_ptr<Eigen::Vector3d> LaneLoc::GetRefPoint(
    const HafNodeInfoConstPtr& ins_ptr) {
  if (ins_ptr == nullptr) {
    HLOG_WARN << "ins_ptr is nullptr";
    return nullptr;
  }

  if (std::isnan(ins_ptr->pos_gcj02().x()) ||
      std::isnan(ins_ptr->pos_gcj02().y()) ||
      std::isnan(ins_ptr->pos_gcj02().z()) ||
      std::isnan(ins_ptr->quaternion().w()) ||
      std::isnan(ins_ptr->quaternion().x()) ||
      std::isnan(ins_ptr->quaternion().y()) ||
      std::isnan(ins_ptr->quaternion().z())) {
    HLOG_ERROR << "ins gcj02 or quaternion is nan";
    return nullptr;
  }
  // 判断enu系下的车辆ins姿态是否异常
  Eigen::Quaternionf enu_quaternion(
      ins_ptr->quaternion().w(), ins_ptr->quaternion().x(),
      ins_ptr->quaternion().y(), ins_ptr->quaternion().z());
  if (enu_quaternion.norm() < 1e-6) {
    HLOG_ERROR << "enu quaternion from ins is abnormal";
    return nullptr;
  }

  static std::shared_ptr<Eigen::Vector3d> ref_point_ptr = nullptr;
  // refpoint初始赋值
  Eigen::Vector3d gcj_position{ins_ptr->pos_gcj02().x(),
                               ins_ptr->pos_gcj02().y(),
                               ins_ptr->pos_gcj02().z()};
  static bool ins_init = false;
  if (!ins_init) {
    ins_init = true;
    ref_point_ptr = std::make_shared<Eigen::Vector3d>(gcj_position);
  }
  // enu系下的车辆运动超过20000米，重置refpoint
  Eigen::Vector3d enu_position =
      hozon::mp::util::Geo::Gcj02ToEnu(gcj_position, *ref_point_ptr);
  if (enu_position.head<2>().norm() > 20000) {
    ref_point_ptr->x() = gcj_position.x();
    ref_point_ptr->y() = gcj_position.y();
    ref_point_ptr->z() = gcj_position.z();
  }
  return ref_point_ptr;
}

SectionPtr LaneLoc::GetPerSection(const Groups& groups) {
  Section per_section;
  // 找到当前group
  Group::Ptr cur_group = BaseLaneLoc::GetVehicleGroup(groups);
  if (cur_group == nullptr) {
    HLOG_WARN << "GetVehicleGroup failed";
    return nullptr;
  }
  for (const auto& lane : cur_group->lanes) {
    Section::Lane per_lane;
    // 1. 车道左右id数组转换
    per_lane.left_lane_ids = lane->left_lane_str_id_with_group;
    per_lane.right_lane_ids = lane->right_lane_str_id_with_group;
    // 2. 左右车道线类型转换，5种，双黄线，黄实线，黄虚线，白实线，白虚线
    if (lane->left_boundary->color == YELLOW &&
        (lane->left_boundary->type == LaneType_DASHED ||
         lane->left_boundary->type == LaneType_SHORT_DASHED ||
         lane->left_boundary->type == LaneType_DOUBLE_DASHED ||
         lane->left_boundary->type == LaneType_LEFT_SOLID_RIGHT_DASHED)) {
      per_lane.left_line.line_type = Section::Lane::DOTTED_YELLOW;
    } else if (lane->left_boundary->color == YELLOW &&
               (lane->left_boundary->type == LaneType_SOLID ||
                lane->left_boundary->type ==
                    LaneType_RIGHT_SOLID_LEFT_DASHED)) {
      per_lane.left_line.line_type = Section::Lane::SOLID_YELLOW;
    } else if (lane->left_boundary->color == YELLOW &&
               lane->left_boundary->type == LaneType_DOUBLE_SOLID) {
      per_lane.left_line.line_type = Section::Lane::DOUBLE_YELLOW;
    } else if (lane->left_boundary->color == WHITE &&
               (lane->left_boundary->type == LaneType_DASHED ||
                lane->left_boundary->type == LaneType_SHORT_DASHED ||
                lane->left_boundary->type == LaneType_DOUBLE_DASHED ||
                lane->left_boundary->type ==
                    LaneType_LEFT_SOLID_RIGHT_DASHED)) {
      per_lane.left_line.line_type = Section::Lane::DOTTED_WHITE;
    } else if (lane->left_boundary->color == WHITE &&
               (lane->left_boundary->type == LaneType_SOLID ||
                lane->left_boundary->type == LaneType_DOUBLE_SOLID ||
                lane->left_boundary->type ==
                    LaneType_RIGHT_SOLID_LEFT_DASHED)) {
      per_lane.left_line.line_type = Section::Lane::SOLID_WHITE;
    } else {
      per_lane.left_line.line_type = Section::Lane::UNKNOWN;
    }
    // 赋予右侧车道线类型: 5种，双黄线，黄实线，黄虚线，白实线，白虚线
    if (lane->right_boundary->color == YELLOW &&
        (lane->right_boundary->type == LaneType_DASHED ||
         lane->right_boundary->type == LaneType_SHORT_DASHED ||
         lane->right_boundary->type == LaneType_DOUBLE_DASHED ||
         lane->right_boundary->type == LaneType_RIGHT_SOLID_LEFT_DASHED)) {
      per_lane.right_line.line_type = Section::Lane::DOTTED_YELLOW;
    } else if (lane->right_boundary->color == YELLOW &&
               (lane->right_boundary->type == LaneType_SOLID ||
                lane->right_boundary->type ==
                    LaneType_LEFT_SOLID_RIGHT_DASHED)) {
      per_lane.right_line.line_type = Section::Lane::SOLID_YELLOW;
    } else if (lane->right_boundary->color == YELLOW &&
               lane->right_boundary->type == LaneType_DOUBLE_SOLID) {
      per_lane.right_line.line_type = Section::Lane::DOUBLE_YELLOW;
    } else if (lane->right_boundary->color == WHITE &&
               (lane->right_boundary->type == LaneType_DASHED ||
                lane->right_boundary->type == LaneType_SHORT_DASHED ||
                lane->right_boundary->type == LaneType_DOUBLE_DASHED ||
                lane->right_boundary->type ==
                    LaneType_RIGHT_SOLID_LEFT_DASHED)) {
      per_lane.right_line.line_type = Section::Lane::DOTTED_WHITE;
    } else if (lane->right_boundary->color == WHITE &&
               (lane->right_boundary->type == LaneType_SOLID ||
                lane->right_boundary->type == LaneType_DOUBLE_SOLID ||
                lane->right_boundary->type ==
                    LaneType_LEFT_SOLID_RIGHT_DASHED)) {
      per_lane.right_line.line_type = Section::Lane::SOLID_WHITE;
    } else {
      per_lane.right_line.line_type = Section::Lane::UNKNOWN;
    }
    // 3. 左右车道线点转换，并记录左右车道线的y
    double left_y = NAN;
    double left_y_min_abs_x = FLT_MAX;
    for (const auto& point : lane->left_boundary->pts) {
      per_lane.left_line.points.emplace_back(point.pt.x(), point.pt.y(),
                                             point.pt.z());
      if (std::abs(point.pt.x()) < left_y_min_abs_x) {
        left_y_min_abs_x = std::abs(point.pt.x());
        left_y = point.pt.y();
      }
    }
    double right_y = NAN;
    double right_y_min_abs_x = FLT_MAX;
    for (const auto& point : lane->right_boundary->pts) {
      per_lane.right_line.points.emplace_back(point.pt.x(), point.pt.y(),
                                              point.pt.z());
      if (std::abs(point.pt.x()) < right_y_min_abs_x) {
        right_y_min_abs_x = std::abs(point.pt.x());
        right_y = point.pt.y();
      }
    }
    if (left_y == NAN || right_y == NAN) {
      continue;
    }
    per_lane.left_line.y_min_abs_x = left_y;
    per_lane.right_line.y_min_abs_x = right_y;
    // 4. 自车道属性赋予
    if (left_y >= 0 && right_y <= 0) {
      per_lane.is_ego_lane = true;
      per_section.ego_lane = per_lane;
    }
    // 5. line_id赋予
    per_lane.left_line.line_id = std::to_string(lane->left_boundary->id);
    per_lane.right_line.line_id = std::to_string(lane->right_boundary->id);
    // 6. near_road_edge赋予
    per_lane.left_line.is_near_road_edge =
        lane->left_boundary->is_near_road_edge ||
        (lane->left_boundary->id > 1000 && lane->left_boundary->id < 2000);
    per_lane.right_line.is_near_road_edge =
        lane->right_boundary->is_near_road_edge ||
        (lane->right_boundary->id > 1000 && lane->right_boundary->id < 2000);
    // if (per_lane.left_line.line_id == std::to_string(12) ||
    //     per_lane.left_line.line_id == std::to_string(14)) {
    //   per_lane.left_line.is_near_road_edge = true;
    // }
    // if (per_lane.right_line.line_id == std::to_string(12) ||
    //     per_lane.right_line.line_id == std::to_string(14)) {
    //   per_lane.right_line.is_near_road_edge = true;
    // }
    // 7. 车道id赋予
    per_lane.lane_id = lane->str_id;

    per_section.sorted_lanes[(left_y + right_y) / 2] = per_lane;
  }
  FilterPerSection(&per_section);
  if (per_section.sorted_lanes.empty()) {
    HLOG_WARN << "per_section.sorted_lanes.empty()";
    return nullptr;
  }
  per_section.leftest_lane = per_section.sorted_lanes.begin()->second;
  per_section.rightest_lane = per_section.sorted_lanes.rbegin()->second;
  return std::make_shared<Section>(per_section);
}

void LaneLoc::FilterPerSection(Section* per_section_ptr) {
  std::map<double, Section::Lane::LaneLine> left_road_edges;
  std::map<double, Section::Lane::LaneLine, std::greater<>> right_road_edges;

  for (const auto& lane : per_section_ptr->sorted_lanes) {
    if ((lane.second.left_line.is_near_road_edge &&
         lane.second.left_line.y_min_abs_x > 0) ||
        (lane.second.right_line.is_near_road_edge &&
         lane.second.right_line.y_min_abs_x > 0)) {
      left_road_edges.insert(
          {lane.second.left_line.y_min_abs_x, lane.second.left_line});
    } else if ((lane.second.left_line.is_near_road_edge &&
                lane.second.left_line.y_min_abs_x < 0) ||
               (lane.second.right_line.is_near_road_edge &&
                lane.second.right_line.y_min_abs_x < 0)) {
      right_road_edges.insert(
          {lane.second.right_line.y_min_abs_x, lane.second.right_line});
    }
  }

  std::vector<double> remove_keys;
  if (!left_road_edges.empty()) {
    for (const auto& lane : per_section_ptr->sorted_lanes) {
      if (lane.second.left_line.y_min_abs_x > left_road_edges.begin()->first) {
        remove_keys.emplace_back(lane.first);
      }
    }
  }
  if (!right_road_edges.empty()) {
    for (const auto& lane : per_section_ptr->sorted_lanes) {
      if (lane.second.right_line.y_min_abs_x <
          right_road_edges.begin()->first) {
        remove_keys.emplace_back(lane.first);
      }
    }
  }
  for (auto key : remove_keys) {
    per_section_ptr->sorted_lanes.erase(key);
  }
}

std::vector<hdmap::LaneInfoConstPtr> LaneLoc::GetLdmapLanes(
    const HafNodeInfoConstPtr& ins_pose_ptr) {
  // 计算车辆的utm坐标
  double x = ins_pose_ptr->pos_gcj02().x();
  double y = ins_pose_ptr->pos_gcj02().y();
  hozon::common::coordinate_convertor::GCS2UTM(51, &y, &x);
  hozon::common::PointENU utm_position;
  utm_position.set_x(y);
  utm_position.set_y(x);
  utm_position.set_z(0);
  // fc的heading和ins_fusion的坐标系都是北东地方向（heading范围为0到360度），utm的坐标系为东北天方向（heading范围为
  // -180到180度）
  // GetLanesWithHeading函数需要传入的参数为utm坐标系下的heading，因此下面6行表示将ins_fusion坐标系的heading转成utm坐标系的heading
  Eigen::Vector3f euler_angle =
      hozon::mp::mf::math::Quat2EulerAngle(Eigen::Quaternionf(
          ins_pose_ptr->quaternion().w(), ins_pose_ptr->quaternion().x(),
          ins_pose_ptr->quaternion().y(), ins_pose_ptr->quaternion().z()));
  std::vector<hdmap::LaneInfoConstPtr> ldmap_lanes;
  GLOBAL_LD_MAP->GetLanesWithHeading(utm_position, 80, euler_angle[2], M_PI_4,
                                     &ldmap_lanes);
  return ldmap_lanes;
}

SectionPtr LaneLoc::GetMapSection(
    const std::vector<hdmap::LaneInfoConstPtr>& ldmap_lanes,
    const Eigen::Vector3d& ref_point, const Eigen::Affine3d& T_V_W) {
  std::unordered_set<std::string> road_ids;
  std::unordered_set<std::string> sec_ids;
  for (const auto& lane_ptr : ldmap_lanes) {
    if (!lane_ptr) {
      continue;
    }
    road_ids.insert(lane_ptr->road_id().id());
    sec_ids.insert(lane_ptr->section_id().id());
  }
  std::unordered_set<std::string> cur_section_lane_ids;
  for (const auto& road_id : road_ids) {
    hdmap::Id road_ptr;
    road_ptr.set_id(road_id);
    auto road = GLOBAL_LD_MAP->GetRoadById(road_ptr)->road();
    for (const auto& section : road.section()) {
      if (sec_ids.find(section.id().id()) == sec_ids.end()) {
        continue;
      }
      double max_x = -FLT_MAX;
      double min_x = FLT_MAX;
      for (const auto& edge : section.boundary().outer_polygon().edge()) {
        for (const auto& segment : edge.curve().segment()) {
          for (const auto& p : segment.line_segment().original_point()) {
            Eigen::Vector3d p_gcj(p.y(), p.x(), 0);
            Eigen::Vector3d p_enu = util::Geo::Gcj02ToEnu(p_gcj, ref_point);
            Eigen::Vector3d p_vehicle = T_V_W * p_enu;
            max_x = std::max(max_x, p_vehicle.x());
            min_x = std::min(min_x, p_vehicle.x());
          }
        }
      }
      // 找到车辆所在section的所有车道id
      if (max_x >= 0 && min_x <= 0) {
        for (const auto& id : section.lane_id()) {
          cur_section_lane_ids.insert(id.id());
        }
      }
    }
  }
  Section map_section;
  for (const auto& lane_ptr : ldmap_lanes) {
    if (!lane_ptr) {
      continue;
    }
    if (!lane_ptr->lane().has_left_boundary() ||
        !lane_ptr->lane().has_right_boundary()) {
      continue;
    }
    if (cur_section_lane_ids.find(lane_ptr->lane().id().id()) ==
        cur_section_lane_ids.end()) {
      continue;
    }
    auto left_types = lane_ptr->lane().left_boundary().boundary_type();
    auto right_types = lane_ptr->lane().right_boundary().boundary_type();
    if (left_types.empty() ||
        (!left_types.empty() && left_types[0].types().empty())) {
      HLOG_WARN << "left_boundary type is empty";
      continue;
    }
    if (right_types.empty() ||
        (!right_types.empty() && right_types[0].types().empty())) {
      HLOG_WARN << "right_boundary type is empty";
      continue;
    }
    Section::Lane map_lane;
    // 1. 车道左右id数组转换
    for (const auto& lane_id :
         lane_ptr->lane().left_neighbor_forward_lane_id()) {
      map_lane.left_lane_ids.emplace_back(lane_id.id());
    }
    for (const auto& lane_id :
         lane_ptr->lane().right_neighbor_forward_lane_id()) {
      map_lane.right_lane_ids.emplace_back(lane_id.id());
    }
    // 2. 左右车道线类型转换
    auto SetLineType = [](Section::Lane::LineType* line_type,
                          int boundary_type) {
      if (boundary_type == 0 || boundary_type == 6 || boundary_type == 7) {
        *line_type = Section::Lane::UNKNOWN;
      } else {
        *line_type = static_cast<Section::Lane::LineType>(boundary_type);
      }
    };
    SetLineType(&map_lane.left_line.line_type,
                lane_ptr->lane().left_boundary().boundary_type()[0].types()[0]);
    SetLineType(
        &map_lane.right_line.line_type,
        lane_ptr->lane().right_boundary().boundary_type()[0].types()[0]);

    // 3. 左右车道线点转换，并记录左右车道线的y
    double left_y = NAN;
    double left_y_min_abs_x = FLT_MAX;
    for (const auto& curve_segment :
         lane_ptr->lane().left_boundary().curve().segment()) {
      for (const auto& p : curve_segment.line_segment().original_point()) {
        Eigen::Vector3d p_gcj(p.y(), p.x(), 0);
        Eigen::Vector3d p_enu = util::Geo::Gcj02ToEnu(p_gcj, ref_point);
        Eigen::Vector3d p_vehicle = T_V_W * p_enu;
        map_lane.left_line.points.emplace_back(p_vehicle);
        if (std::abs(p_vehicle.x()) < left_y_min_abs_x) {
          left_y_min_abs_x = std::abs(p_vehicle.x());
          left_y = p_vehicle.y();
        }
      }
    }
    double right_y = NAN;
    double right_y_min_abs_x = FLT_MAX;
    for (const auto& curve_segment :
         lane_ptr->lane().right_boundary().curve().segment()) {
      for (const auto& p : curve_segment.line_segment().original_point()) {
        Eigen::Vector3d p_gcj(p.y(), p.x(), 0);
        Eigen::Vector3d p_enu = util::Geo::Gcj02ToEnu(p_gcj, ref_point);
        Eigen::Vector3d p_vehicle = T_V_W * p_enu;
        map_lane.right_line.points.emplace_back(p_vehicle);
        if (std::abs(p_vehicle.x()) < right_y_min_abs_x) {
          right_y_min_abs_x = std::abs(p_vehicle.x());
          right_y = p_vehicle.y();
        }
      }
    }
    if (left_y == NAN || right_y == NAN) {
      continue;
    }
    map_lane.left_line.y_min_abs_x = left_y;
    map_lane.right_line.y_min_abs_x = right_y;
    // 4. 自车道属性赋予
    if (left_y >= 0 && right_y <= 0) {
      map_lane.is_ego_lane = true;
      map_section.ego_lane = map_lane;
    }
    // 5. line_id赋予
    if (lane_ptr->lane().left_boundary().id().empty() ||
        lane_ptr->lane().right_boundary().id().empty()) {
      continue;
    }
    map_lane.left_line.line_id = lane_ptr->lane().left_boundary().id()[0].id();
    map_lane.right_line.line_id =
        lane_ptr->lane().right_boundary().id()[0].id();
    // 6. near_road_edge赋予
    map_lane.left_line.is_near_road_edge = false;
    map_lane.right_line.is_near_road_edge = false;
    // 7. 车道id赋予
    map_lane.lane_id = lane_ptr->lane().id().id();

    map_section.sorted_lanes[(left_y + right_y) / 2] = map_lane;
  }
  if (map_section.sorted_lanes.empty()) {
    HLOG_WARN << "map_section.sorted_lanes.empty()";
    return nullptr;
  }
  map_section.leftest_lane = map_section.sorted_lanes.begin()->second;
  map_section.rightest_lane = map_section.sorted_lanes.rbegin()->second;
  return std::make_shared<Section>(map_section);
}

LaneLocInfo LaneLoc::GetLaneLocInfo(int fusion_lane_index,
                                    const SectionPtr& map_section_ptr,
                                    const Eigen::Vector3d& ref_point,
                                    const Eigen::Affine3d& T_V_W) {
  LaneLocInfo lane_loc_info;
  if (fusion_lane_index < 1) {
    return lane_loc_info;
  }
  hdmap::Id curr_lane_id;
  auto it = map_section_ptr->sorted_lanes.begin();
  std::advance(it, fusion_lane_index - 1);
  curr_lane_id.set_id(it->second.lane_id);
  hdmap::LaneInfoConstPtr cur_lane = GLOBAL_LD_MAP->GetLaneById(curr_lane_id);
  if (cur_lane == nullptr) {
    HLOG_WARN << "get cur_lane failed";
    return lane_loc_info;
  }
  // 获取next_lane和next_lane所在的section的车道数量
  for (const auto& id : cur_lane->lane().successor_id()) {
    hdmap::LaneInfoConstPtr next_lane_ptr = GLOBAL_LD_MAP->GetLaneById(id);
    Section::Lane next_lane;
    if (next_lane_ptr == nullptr) {
      continue;
    }
    if (!next_lane_ptr->lane().has_left_boundary() ||
        !next_lane_ptr->lane().has_right_boundary()) {
      continue;
    }
    auto left_types = next_lane_ptr->lane().left_boundary().boundary_type();
    auto right_types = next_lane_ptr->lane().right_boundary().boundary_type();
    if (left_types.empty() ||
        (!left_types.empty() && left_types[0].types().empty())) {
      HLOG_WARN << "left_boundary type is empty";
      continue;
    }
    if (right_types.empty() ||
        (!right_types.empty() && right_types[0].types().empty())) {
      HLOG_WARN << "right_boundary type is empty";
      continue;
    }
    // 1. 车道左右id数组转换
    for (const auto& lane_id :
         next_lane_ptr->lane().left_neighbor_forward_lane_id()) {
      next_lane.left_lane_ids.emplace_back(lane_id.id());
    }
    for (const auto& lane_id :
         next_lane_ptr->lane().right_neighbor_forward_lane_id()) {
      next_lane.right_lane_ids.emplace_back(lane_id.id());
    }
    // 2. 左右车道线类型转换
    auto SetLineType = [](Section::Lane::LineType* line_type,
                          int boundary_type) {
      if (boundary_type == 0 || boundary_type == 6 || boundary_type == 7) {
        *line_type = Section::Lane::UNKNOWN;
      } else {
        *line_type = static_cast<Section::Lane::LineType>(boundary_type);
      }
    };
    SetLineType(
        &next_lane.left_line.line_type,
        next_lane_ptr->lane().left_boundary().boundary_type()[0].types()[0]);
    SetLineType(
        &next_lane.right_line.line_type,
        next_lane_ptr->lane().right_boundary().boundary_type()[0].types()[0]);

    // 3. 左右车道线点转换，并记录左右车道线的y
    double left_y = NAN;
    double left_y_min_abs_x = FLT_MAX;
    for (const auto& curve_segment :
         next_lane_ptr->lane().left_boundary().curve().segment()) {
      for (const auto& p : curve_segment.line_segment().original_point()) {
        Eigen::Vector3d p_gcj(p.y(), p.x(), 0);
        Eigen::Vector3d p_enu = util::Geo::Gcj02ToEnu(p_gcj, ref_point);
        Eigen::Vector3d p_vehicle = T_V_W * p_enu;
        next_lane.left_line.points.emplace_back(p_vehicle);
        if (std::abs(p_vehicle.x()) < left_y_min_abs_x) {
          left_y_min_abs_x = std::abs(p_vehicle.x());
          left_y = p_vehicle.y();
        }
      }
    }
    double right_y = NAN;
    double right_y_min_abs_x = FLT_MAX;
    for (const auto& curve_segment :
         next_lane_ptr->lane().right_boundary().curve().segment()) {
      for (const auto& p : curve_segment.line_segment().original_point()) {
        Eigen::Vector3d p_gcj(p.y(), p.x(), 0);
        Eigen::Vector3d p_enu = util::Geo::Gcj02ToEnu(p_gcj, ref_point);
        Eigen::Vector3d p_vehicle = T_V_W * p_enu;
        next_lane.right_line.points.emplace_back(p_vehicle);
        if (std::abs(p_vehicle.x()) < right_y_min_abs_x) {
          right_y_min_abs_x = std::abs(p_vehicle.x());
          right_y = p_vehicle.y();
        }
      }
    }
    if (left_y == NAN || right_y == NAN) {
      continue;
    }
    next_lane.left_line.y_min_abs_x = left_y;
    next_lane.right_line.y_min_abs_x = right_y;
    // 4. 自车道属性赋予
    if (left_y >= 0 && right_y <= 0) {
      next_lane.is_ego_lane = true;
    }
    // 5. line_id赋予
    if (next_lane_ptr->lane().left_boundary().id().empty() ||
        next_lane_ptr->lane().right_boundary().id().empty()) {
      continue;
    }
    next_lane.left_line.line_id =
        next_lane_ptr->lane().left_boundary().id()[0].id();
    next_lane.right_line.line_id =
        next_lane_ptr->lane().right_boundary().id()[0].id();
    // 6. near_road_edge赋予
    next_lane.left_line.is_near_road_edge = false;
    next_lane.right_line.is_near_road_edge = false;
    // 7. 车道id赋予
    next_lane.lane_id = next_lane_ptr->lane().id().id();

    // 获取next_lane所在section的车道数量
    int lane_num = 0;
    hdmap::Id next_lane_road_id;
    next_lane_road_id.set_id(next_lane_ptr->road_id().id());
    hdmap::RoadInfoConstPtr road =
        GLOBAL_LD_MAP->GetRoadById(next_lane_road_id);
    if (road == nullptr) {
      HLOG_WARN << "next lane road can't find";
      continue;
    }
    for (const auto& section : road->sections()) {
      if (section.id().id() == next_lane_ptr->section_id().id()) {
        lane_num += section.lane_id().size();
        break;
      }
    }

    // 输出结果
    LaneLocInfo::NextLaneInfo next_lane_info;
    next_lane_info.lane_num = lane_num;
    next_lane_info.next_lane = next_lane;
    lane_loc_info.next_lanes_info.emplace_back(next_lane_info);
  }
  lane_loc_info.cur_lane_index = fusion_lane_index;
  return lane_loc_info;
}

}  // namespace lane_loc
}  // namespace mf
}  // namespace mp
}  // namespace hozon
