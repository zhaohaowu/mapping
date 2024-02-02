/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： viz_map.cc
 *   author     ： zhangshuo
 *   date       ： 2023.11
 ******************************************************************************/

#include "map_fusion/map_service/map_table.h"
#include <gflags/gflags.h>

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "Eigen/src/Core/Matrix.h"
#include "adsfi_proto/viz/sensor_msgs.pb.h"
#include "adsfi_proto/viz/visualization_msgs.pb.h"
#include "base/utils/log.h"
#include "map_fusion/map_service/global_hd_map.h"
#include "opencv2/ml.hpp"
#include "util/mapping_log.h"

namespace hozon {
namespace mp {
namespace mf {

void MapTable::OnLocalization(
    const std::shared_ptr<hozon::localization::Localization>& msg) {
  // gcj02
  Eigen::Vector3d pos_global(msg->pose().gcj02().x(), msg->pose().gcj02().y(),
                             0);
  OnLocationInGlobal(msg->pose().pos_utm_01().x(),
                     msg->pose().pos_utm_01().y());

  if (!local_enu_center_flag_) {
    local_enu_center_ << pos_global.x(), pos_global.y(), pos_global.z();
    local_enu_center_flag_ = true;
  }

  {
    std::lock_guard<std::mutex> lock(mtx_);
    Clear();
    BuildLaneTable();
  }
}

std::tuple<std::unordered_map<std::string, LaneInfo>,
           std::unordered_map<std::string, RoadInfo>>
MapTable::GetMapTable() {
  // 返回哈希表
  std::tuple<std::unordered_map<std::string, LaneInfo>,
             std::unordered_map<std::string, RoadInfo>>
      map_table_;
  map_table_ = std::make_tuple(lane_table_, road_table_);
  return map_table_;
}

void MapTable::OnLocationInGlobal(double utm_x, double utm_y) {
  location_utm_.x() = utm_x;
  location_utm_.y() = utm_y;
}

void MapTable::BuildLaneTable() {
  // 创建lane_table
  hozon::common::PointENU utm_pos;
  utm_pos.set_x(location_utm_.x());
  utm_pos.set_y(location_utm_.y());
  utm_pos.set_z(0);

  const double range = 300.;
  std::vector<hozon::hdmap::LaneInfoConstPtr> lanes_in_range;
  std::vector<hozon::hdmap::RoadInfoConstPtr> roads_in_range;
  ObtainLaneAndRoad(utm_pos, range, &lanes_in_range, &roads_in_range);
  CreatLaneTable(lanes_in_range);
  CreatRoadTable(roads_in_range);

  for (const auto& it : lanes_in_range) {
    if (lane_table_.find(it->lane().id().id()) == lane_table_.end()) {
      continue;
    }
    if (lane_table_.at(it->lane().id().id()).extra_boundary == 2) {
      auto right_lanes = lane_table_.at(it->lane().id().id()).right_lane_ids;
      if (right_lanes.empty()) {
        continue;
      }
      auto right_lane = right_lanes[0];
      if (lane_table_.find(right_lane) == lane_table_.end()) {
        continue;
      }
      lane_table_.at(right_lane).extra_boundary = 1;
    }
  }

  for (const auto& it : lanes_in_range) {
    // 计算变道引导线的tan theta
    CalculateTanTheta(it->lane().id().id());
  }

  for (const auto& road : roads_in_range) {
    const auto& road_id = road->id().id();
    if (road_table_.find(road_id) == road_table_.end()) {
      continue;
    }
    const auto& sections = road_table_.at(road_id).section_ids;
    for (const auto& it : road->sections()) {
      if (std::find(all_section_ids_.begin(), all_section_ids_.end(),
                    it.id().id()) == all_section_ids_.end()) {
        continue;
      }
      auto road_boundary = sections.at(it.id().id()).road_boundary;
      auto sec_lane_id = sections.at(it.id().id()).lane_id;
      // 由单边几何预测所有车道线，并填充车道线
      ConstructLaneLine(road_boundary, sec_lane_id);
    }
  }
}

void MapTable::ObtainLaneAndRoad(
    const hozon::common::PointENU& utm_pos, const double& range,
    std::vector<hozon::hdmap::LaneInfoConstPtr>* lanes_in_range,
    std::vector<hozon::hdmap::RoadInfoConstPtr>* roads_in_range) {
  if (!GLOBAL_HD_MAP) {
    HLOG_ERROR << "nullptr hq_map_server_";
    return;
  }
  int ret = GLOBAL_HD_MAP->GetLanes(utm_pos, range, lanes_in_range);
  if (ret != 0) {
    HLOG_ERROR << "get local map lanes failed";
    return;
  }
  ret = GLOBAL_HD_MAP->GetRoads(utm_pos, range, roads_in_range);
  if (ret != 0) {
    HLOG_ERROR << "get local map roads failed";
    return;
  }
}

void MapTable::CreatLaneTable(
    const std::vector<hozon::hdmap::LaneInfoConstPtr>& lanes_in_range) {
  // 创建lane
  // 创建lane_table_
  for (const auto& lane : lanes_in_range) {
    const auto& lane_id = lane->id().id();
    LaneInfo local_lane;
    local_lane.lane_id = lane_id;
    local_lane.road_id = lane->road_id().id();
    local_lane.section_id = lane->section_id().id();
    all_section_ids_.emplace_back(lane->section_id().id());

    // 判断是否是变道虚拟线，如果是的话计算出theta角
    uint32_t extra_boundary = 0;
    CalculateVirtual(&extra_boundary, lane->lane());
    local_lane.extra_boundary = extra_boundary;

    const auto& lane_length = lane->lane().left_boundary().length();
    local_lane.length = lane_length;

    // 存lane左右边线
    for (const auto& it : lane->lane().left_boundary().curve().segment()) {
      for (const auto& itt : it.line_segment().original_point()) {
        Eigen::Vector3d pt = GcjPtToLocalEnu(itt);
        local_lane.left_line.emplace_back(pt);
      }
    }
    for (const auto& it : lane->lane().right_boundary().curve().segment()) {
      for (const auto& itt : it.line_segment().original_point()) {
        Eigen::Vector3d pt = GcjPtToLocalEnu(itt);
        local_lane.right_line.emplace_back(pt);
      }
    }

    // 存储车道的类型（实线或者是虚线）
    // auto left_line_type =
    //     lane->lane().left_boundary().boundary_type(0).types(0);
    // auto right_line_type =
    //     lane->lane().right_boundary().boundary_type(0).types(0);
    // local_lane.left_line_type = left_line_type;
    // local_lane.right_line_type = right_line_type;

    // 存储每条lane右边线最后两个点的单位向量
    const auto& right_size = local_lane.right_line.size();
    const auto& last_normal = (local_lane.right_line[right_size - 2] -
                               local_lane.right_line[right_size - 1])
                                  .normalized();
    local_lane.last_normal = last_normal;

    // 存储拓扑
    for (const auto& it : lane->lane().left_neighbor_forward_lane_id()) {
      local_lane.left_lane_ids.emplace_back(it.id());
    }
    for (const auto& it : lane->lane().right_neighbor_forward_lane_id()) {
      local_lane.right_lane_ids.emplace_back(it.id());
    }
    for (const auto& it : lane->lane().predecessor_id()) {
      local_lane.prev_lane_ids.emplace_back(it.id());
    }
    for (const auto& it : lane->lane().successor_id()) {
      local_lane.next_lane_ids.emplace_back(it.id());
    }
    // 存储lane的宽度
    double length = lane->lane().length();
    double s = 0.;
    std::vector<double> lane_width;
    while (s < length) {
      double width = lane->GetWidth(s);
      lane_width.push_back(width);
      s += 0.5;
    }
    double end_width = lane->GetWidth(length);
    lane_width.push_back(end_width);
    local_lane.lane_width = lane_width;
    // 存储站心
    local_lane.ref_point = local_enu_center_;
    lane_table_.insert_or_assign(lane_id, local_lane);
  }
}

void MapTable::CalculateVirtual(uint32_t* extra_boundary,
                                const hdmap::Lane& lane) {
  const auto& extra_left_size =
      lane.extra_left_boundary().curve().segment_size();
  const auto& extra_right_size =
      lane.extra_right_boundary().curve().segment_size();
  if (extra_left_size != 0) {
    *extra_boundary = 1;
  }
  if (extra_right_size != 0) {
    *extra_boundary = 2;
  }
}

Eigen::Vector3d MapTable::GcjPtToLocalEnu(
    const hozon::common::PointENU& point_gcj) {
  Eigen::Vector3d pt_gcj(point_gcj.y(), point_gcj.x(), 0);
  Eigen::Vector3d point_enu = util::Geo::Gcj02ToEnu(pt_gcj, local_enu_center_);
  return point_enu;
}

void MapTable::CreatRoadTable(
    const std::vector<hozon::hdmap::RoadInfoConstPtr>& roads_in_range) {
  // 创建road_table_
  for (const auto& road : roads_in_range) {
    const auto& road_id = road->id().id();
    RoadInfo local_road;
    local_road.road_id = road_id;
    for (const auto& it : road->sections()) {
      if (std::find(all_section_ids_.begin(), all_section_ids_.end(),
                    it.id().id()) == all_section_ids_.end()) {
        continue;
      }
      Section section;
      section.section_id = it.id().id();
      for (const auto& itt : it.lane_id()) {
        section.lane_id.emplace_back(itt.id());
      }

      // 存储原始hd_map中的左右road边界
      // StoreLeftAndRightBoundary(it, &section);

      // 将左二车道的左边界作为road_boundary
      const auto& left_second_left_id = section.lane_id.back();
      if (lane_table_.find(left_second_left_id) != lane_table_.end()) {
        section.road_boundary = lane_table_.at(left_second_left_id).right_line;
      }

      local_road.section_ids.insert_or_assign(section.section_id, section);
    }
    road_table_.insert_or_assign(road_id, local_road);
  }
}

#if 0
void MapTable::StoreLeftAndRightBoundary(const hozon::hdmap::RoadSection& it,
                                         Section* section) {
  // 存储road edge
  for (const auto& edge : it.boundary().outer_polygon().edge()) {
    if (edge.type() == 2) {
      for (const auto& seg : edge.curve().segment()) {
        for (const auto& pt : seg.line_segment().point()) {
          Eigen::Vector3d ptt = GcjPtToLocalEnu(pt);
          section->left_boundary.emplace_back(ptt);
        }
      }
    }

    if (edge.type() == 3) {
      for (const auto& seg : edge.curve().segment()) {
        for (const auto& pt : seg.line_segment().point()) {
          Eigen::Vector3d ptt = GcjPtToLocalEnu(pt);
          section->right_boundary.emplace_back(ptt);
        }
      }
    }
  }
}
#endif

void MapTable::CalculateTanTheta(const std::string& idd) {
  if (lane_table_.find(idd) == lane_table_.end()) {
    HLOG_ERROR << "idd have not in lane_table_";
    return;
  }
  /*
      1.根据前继车道判断其是否是第一个车道引导线
      2.如果是，求该车道的所有后继车道，直道后继车道跨车道或者不是引导线
    */
  auto prev_ids = lane_table_.at(idd).prev_lane_ids;
  if (prev_ids.empty()) {
    return;
  }
  auto prev_id = prev_ids[0];
  if (lane_table_.find(prev_id) == lane_table_.end()) {
    return;
  }
  const auto& next_size = lane_table_.at(prev_id).next_lane_ids.size();
  if (next_size != 2 && lane_table_.at(prev_id).extra_boundary != 0 &&
      prev_ids.size() != 2) {
    return;
  }
  // 左引导线
  if (lane_table_.at(idd).extra_boundary == 1 &&
      std::find(had_equ_.begin(), had_equ_.end(), idd) == had_equ_.end()) {
    ObtainEquation(idd, static_cast<int>(next_size));
  } else {
    lane_table_.at(idd).se_point.first << 0.0, 0.0, 0.0;
    lane_table_.at(idd).se_point.second << 0.0, 0.0, 0.0;
  }
}

void MapTable::ObtainEquation(const std::string& idd, const int& next_size) {
  // idd是第一个车道引导线的，其是同一个车道内
  // 求左边线第一个点的坐标（这里由车道宽度来进行求解)
  had_equ_.emplace_back(idd);
  const auto& sec_id = lane_table_.at(idd).section_id;
  const auto& road_id = lane_table_.at(idd).road_id;
  if (road_table_.find(road_id) == road_table_.end()) {
    return;
  }
  const auto& section = road_table_.at(road_id).section_ids;
  const auto& sec_lane_ids = section.at(sec_id).lane_id;
  double width = 0.;

  ComputeStartWidth(sec_lane_ids, idd, &width, next_size);
  const auto& bound_id = sec_lane_ids.back();
  if (lane_table_.find(bound_id) == lane_table_.end()) {
    return;
  }
  const auto& A = lane_table_.at(bound_id).right_line.front();

  Eigen::Vector3d AB_N;
  const auto& last_id = sec_lane_ids.back();
  LaneLastNormal(&AB_N, last_id);
  Eigen::Vector3d AB_C(-AB_N.y(), AB_N.x(), 0);
  // 计算起点坐标
  Eigen::Vector3d start_point = A - AB_C * width;
  if (idd == sec_lane_ids.back()) {
    start_point = A + AB_C * width;
  }

  auto point = lane_table_.at(idd).left_line.front();
  // 计算终点坐标
  std::vector<std::string> lane_idd_nexts;
  auto lane_idd = idd;
  FindLastExtraLane(&lane_idd, &lane_idd_nexts);
  const auto& lane_idd_nxts = lane_table_.at(lane_idd).next_lane_ids;
  if (lane_idd_nxts.empty()) {
    return;
  }
  const auto& lane_idd_nxt = lane_idd_nxts[0];
  // lane_idd所对应车道的终点
  if (lane_table_.find(lane_idd_nxt) == lane_table_.end()) {
    return;
  }
  const auto& sec_idd = lane_table_.at(lane_idd_nxt).section_id;
  const auto& road_idd = lane_table_.at(lane_idd_nxt).road_id;
  if (road_table_.find(road_idd) == road_table_.end()) {
    return;
  }
  const auto& section_idd = road_table_.at(road_idd).section_ids;
  if (section_idd.find(sec_idd) == section_idd.end()) {
    return;
  }
  const auto& lane_idds = section_idd.at(sec_idd).lane_id;

  const auto& last_bound = lane_idds.back();
  if (lane_table_.find(last_bound) == lane_table_.end()) {
    return;
  }

  const auto& A_last = lane_table_.at(last_bound).right_line.front();

  Eigen::Vector3d AB_N_last;
  const auto& last_idd = lane_idds.back();
  LaneLastNormal(&AB_N_last, last_idd);
  Eigen::Vector3d AB_C_last(-AB_N_last.y(), AB_N_last.x(), 0);

  double width_last = 0.;
  ComputeEndWidth(lane_idds, lane_idd_nxt, &width_last, lane_idd_nxts);
  if (lane_idds.size() >= 2 && lane_idds[lane_idds.size() - 2] == lane_idd_nxt) {
    width_last = 0;
  }
  if (lane_idds.size() == 1) {
    width_last = lane_table_.at(lane_idds.back()).lane_width.front();
  }
  Eigen::Vector3d end_point;
  if (lane_idd_nxt == lane_idds.back()) {
    end_point = A_last + AB_C_last * width_last;
  } else {
    end_point = A_last - AB_C_last * width_last;
  }

  lane_table_.at(idd).se_point.first = start_point;
  lane_table_.at(idd).se_point.second = end_point;
  for (const auto& it : lane_idd_nexts) {
    lane_table_.at(it).se_point.first = start_point;
    lane_table_.at(it).se_point.second = end_point;
  }
}

void MapTable::ComputeStartWidth(const std::vector<std::string>& sec_lane_ids,
                                 const std::string& idd, double* width,
                                 const int& next_size) {
  for (int i = static_cast<int>(sec_lane_ids.size()) - 2; i >= 0; i--) {
    if (sec_lane_ids[i] == idd) {
      if (next_size == 2 && sec_lane_ids.size() > 2) {
        *width -= lane_table_.at(sec_lane_ids[i]).lane_width.front();
      }
      break;
    }
    if (lane_table_.find(sec_lane_ids[i]) == lane_table_.end()) {
      break;
    }
    *width += lane_table_.at(sec_lane_ids[i]).lane_width.front();
  }
}

void MapTable::ComputeEndWidth(const std::vector<std::string>& lane_idds,
                               const std::string& lane_idd_nxt,
                               double* width_last,
                               const std::vector<std::string>& lane_idd_nxts) {
  for (int i = static_cast<int>(lane_idds.size()) - 2; i >= 0; i--) {
    if (lane_idds[i] == lane_idd_nxt) {
      if (lane_table_.at(lane_idd_nxt).extra_boundary == 1) {
        if (lane_idd_nxts.size() == 2) {
          *width_last -= lane_table_.at(lane_idds[i]).lane_width.front();
        }
      }
      break;
    }
    if (lane_table_.find(lane_idds[i]) == lane_table_.end()) {
      break;
    }
    *width_last += lane_table_.at(lane_idds[i]).lane_width.front();
  }
}

void MapTable::FindLastExtraLane(std::string* lane_idd,
                                 std::vector<std::string>* lane_idd_nexts) {
  while (lane_table_.at(*lane_idd).extra_boundary == 1) {
    auto idd_nexts = lane_table_.at(*lane_idd).next_lane_ids;
    if (idd_nexts.empty()) {
      break;
    }
    auto idd_next = idd_nexts[0];
    if (lane_table_.find(idd_next) == lane_table_.end()) {
      break;
    }
    auto idd_next_prev = lane_table_.at(idd_next).prev_lane_ids;
    auto extra_boundary = lane_table_.at(idd_next).extra_boundary;

    if (extra_boundary == 1 && idd_nexts.size() != 2 &&
        idd_next_prev.size() != 2) {
      *lane_idd = idd_next;
      had_equ_.emplace_back(*lane_idd);
      lane_idd_nexts->emplace_back(*lane_idd);
    } else {
      break;
    }
  }
}

void MapTable::ConstructLaneLine(
    const std::vector<Eigen::Vector3d>& road_boundary,
    const std::vector<std::string>& sec_lane_id) {
  // 预测车道线
  std::vector<std::vector<Eigen::Vector3d>> predict_lanelines;
  // 可视化
  FitPredLaneLine(road_boundary, sec_lane_id, &predict_lanelines);
  viz_map_.VizCompanLane(predict_lanelines);
  const auto& pred_line_size = static_cast<int>(predict_lanelines.size());
  if (pred_line_size == 0) {
    return;
  }
  for (int i = 0; i < static_cast<int>(sec_lane_id.size()); i++) {
    if (lane_table_.find(sec_lane_id[i]) == lane_table_.end()) {
      break;
    }
    if (left_virtual_line_.find(sec_lane_id[i]) != left_virtual_line_.end() &&
        lane_table_.at(sec_lane_id[i]).extra_boundary == 1) {
      lane_table_.at(sec_lane_id[i]).pred_left_line =
          left_virtual_line_.at(sec_lane_id[i]);
    } else {
      lane_table_.at(sec_lane_id[i]).pred_left_line =
          predict_lanelines[pred_line_size - i - 2];
    }
    lane_table_.at(sec_lane_id[i]).pred_right_line =
        predict_lanelines[pred_line_size - i - 1];
    if (predict_lanelines[pred_line_size - i - 1].empty()) {
      lane_table_.at(sec_lane_id[i]).pred_left_line.clear();
    }
  }
}

void MapTable::FitPredLaneLine(
    const std::vector<Eigen::Vector3d>& road_boundary,
    const std::vector<std::string>& sec_lane_id,
    std::vector<std::vector<Eigen::Vector3d>>* predict_lanelines) {
  // 新的拟合车道线的方法
  if (road_boundary.empty() || sec_lane_id.empty()) {
    return;
  }
  int lane_num = static_cast<int>(sec_lane_id.size()) + 1;
  // JudgeExtra(&lane_num, sec_lane_id);
  std::vector<std::vector<Eigen::Vector3d>> predict_line(lane_num);
  // STD!!这里有个假设，即一个section中只有一个变道虚拟线
  std::vector<Eigen::Vector3d> virtual_line;
  std::string virtual_id;
  // 拟合中间点
  FitMiddlePoint(road_boundary, sec_lane_id, &predict_line, &virtual_line,
                 &virtual_id);
  // 求最后一个点的左右点
  FitLastPoint(road_boundary, sec_lane_id, &predict_line, &virtual_line,
               &virtual_id);
  left_virtual_line_.insert_or_assign(virtual_id, virtual_line);

  if (!virtual_id.empty()) {
    for (int i = static_cast<int>(predict_line.size()) - 1; i >= 0; i--) {
      // 对opt_line进行平滑处理(滑动窗口法)
      auto opt_line = SmoothedPoint(predict_line[i]);
      predict_line[i].clear();
      predict_line[i] = opt_line;
    }
  }

  if (!predict_line.empty()) {
    *predict_lanelines = predict_line;
  }
}

std::vector<Eigen::Vector3d> MapTable::SmoothedPoint(
    const std::vector<Eigen::Vector3d>& pred_line) {
  // 对车道线点进行平滑处理
  if (pred_line.size() < 3) {
    return pred_line;
  }
  std::vector<Eigen::Vector3d> smooth_line;
  smooth_line.emplace_back(pred_line.front());
  for (size_t i = 0; i <= pred_line.size() - 3; i++) {
    auto new_point = (pred_line[i] + pred_line[i + 1] + pred_line[i + 2]) / 3.0;
    smooth_line.emplace_back(new_point);
  }
  smooth_line.emplace_back(pred_line.back());
  return smooth_line;
}

void MapTable::FitMiddlePoint(
    const std::vector<Eigen::Vector3d>& road_boundary,
    const std::vector<std::string>& sec_lane_id,
    std::vector<std::vector<Eigen::Vector3d>>* predict_line,
    std::vector<Eigen::Vector3d>* virtual_line, std::string* virtual_id) {
  const auto& road_size = road_boundary.size();
  int index = 0;  // 表示第几个点的索引
  double s = 0.;
  const auto& start_point = road_boundary.front();
  bool flag = true;
  for (size_t i = 1; i < road_size; ++i) {
    const auto& A = road_boundary[i - 1];
    const auto& B = road_boundary[i];
    // if ((A - B).norm() < 1.0 && i + 1 < road_size) {
    //   B = road_boundary[i + 1];
    // }
    Eigen::Vector3d AB = B - A;
    Eigen::Vector3d AB_N = AB.normalized();
    if (i == 1) {
      const auto& last_id = sec_lane_id.back();
      LaneLastNormal(&AB_N, last_id);
    } else {
      AB_N = AB.normalized();
    }

    Eigen::Vector3d AB_C(-AB_N.y(), AB_N.x(), 0);
    // 根据距离求其他的点
    // 先求左侧的车道线点
    if (lane_table_.find(sec_lane_id.back()) == lane_table_.end()) {
      HLOG_ERROR << "lane id not in lane table!";
      continue;
    }
    if (i > 1) {
      s += (B - A).norm();
      index = static_cast<int>(s / 0.5);
    }
    const auto& first_width = lane_table_.at(sec_lane_id.back()).lane_width;
    // if (index > first_width.size()) {
    //   continue;
    // }
    Eigen::Vector3d first_point = A + AB_C * first_width[index];
    predict_line->at(0).push_back(first_point);
    predict_line->at(1).push_back(A);
    FitVirtualPoint(sec_lane_id, virtual_line, virtual_id, s, A, AB_C, index,
                    predict_line);
  }
}

void MapTable::LaneLastNormal(Eigen::Vector3d* AB_N,
                              const std::string& last_id) {
  if (lane_table_.find(last_id) == lane_table_.end()) {
    HLOG_ERROR << "lane id not in lane table!";
    return;
  }
  const auto& last_id_pre = lane_table_.at(last_id).prev_lane_ids;
  if (last_id_pre.empty()) {
    HLOG_ERROR << "the last id have no prev lane id!";
    return;
  }
  const auto& prev_id = last_id_pre.front();
  if (lane_table_.find(prev_id) == lane_table_.end()) {
    HLOG_ERROR << "prev lane id not in lane table!";
    return;
  }
  const auto& last_normal = lane_table_.at(prev_id).last_normal;
  *AB_N << -last_normal.x(), -last_normal.y(), 0;
}

void MapTable::FitVirtualPoint(
    const std::vector<std::string>& sec_lane_id,
    std::vector<Eigen::Vector3d>* virtual_line, std::string* virtual_id,
    const double& s, const Eigen::Vector3d& A, const Eigen::Vector3d& AB_C,
    const int& index, std::vector<std::vector<Eigen::Vector3d>>* predict_line) {
  // right point
  double width = 0.;
  if (sec_lane_id.size() < 2) {
    return;
  }
  for (int j = static_cast<int>(sec_lane_id.size()) - 2; j >= 0; --j) {
    if (lane_table_.find(sec_lane_id[j]) == lane_table_.end()) {
      continue;
    }
    const auto& wid = lane_table_.at(sec_lane_id[j]).lane_width;
    // width += wid[index];
    if (lane_table_.at(sec_lane_id[j]).extra_boundary == 1) {
      // 如果是变道引导线，求两个法向量之间的交点
      HLOG_ERROR << "---------------------------sec_lane_id.size(): " << sec_lane_id.size();
      HLOG_ERROR << "---------------------------sec_lane_id.size() - j - 1: " << sec_lane_id.size() - j - 1;
      HLOG_ERROR << "---------------------------predict_line: " << predict_line->size();
      if (predict_line->at(sec_lane_id.size() - j - 1).empty()) {
        continue;
      }
      Eigen::Vector3d B = predict_line->at(sec_lane_id.size() - j - 1).back();
      const auto& C = lane_table_.at(sec_lane_id[j]).se_point.first;
      const auto& D = lane_table_.at(sec_lane_id[j]).se_point.second;
      if (C.isZero() || D.isZero()) {
        HLOG_ERROR << "AB_D vector is zero!";
        continue;
      }

      // 计算向量AB和向量CD
      Eigen::Vector3d AB = B - A;
      Eigen::Vector3d CD = D - C;

      double t = ((C - A).cross(CD)).norm() / AB.cross(CD).norm();
      Eigen::Vector3d virtual_point;
      if (AB.cross(CD).norm() == 0) {
        virtual_point = A;
      } else {
        virtual_point = A + t * AB;
      }

      double virtual_width = (A - virtual_point).norm();
      virtual_line->emplace_back(virtual_point);
      *virtual_id = sec_lane_id[j];
      if (sec_lane_id.size() == 2 &&
          lane_table_.at(sec_lane_id.back()).extra_boundary == 2) {
        break;
        width = wid[index] - virtual_width;
      } else {
        width = virtual_width + wid[index];
      }
    } else {
      width += wid[index];
    }
    Eigen::Vector3d point = A - AB_C * width;
    predict_line->at(sec_lane_id.size() - j).push_back(point);
  }
}

void MapTable::FitLastPoint(
    const std::vector<Eigen::Vector3d>& road_boundary,
    const std::vector<std::string>& sec_lane_id,
    std::vector<std::vector<Eigen::Vector3d>>* predict_line,
    std::vector<Eigen::Vector3d>* virtual_line, std::string* virtual_id) {
  // const auto& road_size = road_boundary.size();
  // const auto& A = road_boundary[road_size - 1];
  // const auto& B = road_boundary[road_size - 2];
  // Eigen::Vector3d AB = B - A;
  // Eigen::Vector3d AB_N = AB.normalized();
  // Eigen::Vector3d AB_C(-AB_N.y(), AB_N.x(), 0);
  // 先求右侧的车道线点
  if (lane_table_.find(sec_lane_id.back()) == lane_table_.end()) {
    HLOG_ERROR << "lane id not in lane table!";
    return;
  }
  const auto& A = road_boundary.back();
  Eigen::Vector3d AB_N = lane_table_.at(sec_lane_id.back()).last_normal;
  Eigen::Vector3d AB_C(-AB_N.y(), AB_N.x(), 0);

  const auto& end_width = lane_table_.at(sec_lane_id.back()).lane_width;
  Eigen::Vector3d first_point = A - AB_C * end_width.back();
  predict_line->at(0).push_back(first_point);
  predict_line->at(1).push_back(A);
  double width = 0.;
  if (sec_lane_id.size() < 2) {
    HLOG_ERROR << "sec_lane_id.size() < 2";
    return;
  }
  for (int j = static_cast<int>(sec_lane_id.size()) - 2; j >= 0; --j) {
    if (lane_table_.find(sec_lane_id[j]) == lane_table_.end()) {
      HLOG_ERROR << "lane id not in lane table!";
      continue;
    }
    const auto& wid = lane_table_.at(sec_lane_id[j]).lane_width;
    if (lane_table_.at(sec_lane_id[j]).extra_boundary == 1) {
      if (predict_line->at(sec_lane_id.size() - j - 1).empty()) {
        continue;
      }
      Eigen::Vector3d B = predict_line->at(sec_lane_id.size() - j - 1).back();
      const auto& C = lane_table_.at(sec_lane_id[j]).se_point.first;
      const auto& D = lane_table_.at(sec_lane_id[j]).se_point.second;
      if (C.isZero() || D.isZero()) {
        HLOG_ERROR << "AB_D vector is zero!";
        continue;
      }

      // 计算向量AB和向量CD
      Eigen::Vector3d virtual_point;
      ComputeVirtualPoint(A, B, C, D, &virtual_point);
      Eigen::Vector3d prev_point =
          predict_line->at(sec_lane_id.size() - j - 2).back();
      double virtual_width = (A - virtual_point).norm();

      virtual_line->emplace_back(virtual_point);
      *virtual_id = sec_lane_id[j];
      if (sec_lane_id.size() == 2 &&
          lane_table_.at(sec_lane_id.back()).extra_boundary == 2) {
        break;
        width = wid.back() - virtual_width;
      } else {
        width = virtual_width + wid.back();
      }
      // width = virtual_width + wid.back();
    } else {
      width += wid.back();
    }
    //  width += wid.back();
    Eigen::Vector3d point = A + AB_C * width;
    predict_line->at(sec_lane_id.size() - j).push_back(point);
  }
}

void MapTable::ComputeVirtualPoint(const Eigen::Vector3d& A,
                                   const Eigen::Vector3d& B,
                                   const Eigen::Vector3d& C,
                                   const Eigen::Vector3d& D,
                                   Eigen::Vector3d* virtual_point) {
  Eigen::Vector3d AB = B - A;
  Eigen::Vector3d CD = D - C;

  double t = ((C - A).cross(CD)).norm() / AB.cross(CD).norm();
  if (AB.cross(CD).norm() == 0) {
    *virtual_point = A;
  } else {
    *virtual_point = A + t * AB;
  }
}

void MapTable::Clear() {
  lane_table_.clear();
  road_table_.clear();
  all_section_ids_.clear();
  left_virtual_line_.clear();
  had_equ_.clear();
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
