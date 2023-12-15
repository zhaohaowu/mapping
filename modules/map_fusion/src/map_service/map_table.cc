/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： viz_map.cc
 *   author     ： zhangshuo
 *   date       ： 2023.11
 ******************************************************************************/

#include "map_fusion/map_service/map_table.h"
#include <gflags/gflags.h>

#include "Eigen/src/Core/Matrix.h"
#include "adsfi_proto/viz/sensor_msgs.pb.h"
#include "adsfi_proto/viz/visualization_msgs.pb.h"
#include "common/utm_projection/coordinate_convertor.h"
#include "map_fusion/map_service/global_hd_map.h"
#include "util/mapping_log.h"

namespace hozon {
namespace mp {
namespace mf {

void MapTable::OnLocalization(
    const std::shared_ptr<hozon::localization::Localization>& msg) {
  // utm转gcj02
  double utm_x = msg->pose().pos_utm_01().x();
  double utm_y = msg->pose().pos_utm_01().y();
  uint32_t zone_t = msg->pose().utm_zone_01();
  int zone = static_cast<int>(zone_t);
  bool ret = hozon::common::coordinate_convertor::UTM2GCS(zone, &utm_x, &utm_y);
  if (!ret) {
    HLOG_ERROR << "UTM2GCS failed";
    return;
  }

  // gcj02
  Eigen::Vector3d pos_global(utm_y, utm_x, 0);
  OnLocationInGlobal(pos_global, zone, msg->pose().pos_utm_01().x(),
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

void MapTable::OnLocationInGlobal(const Eigen::Vector3d& pos_gcj02,
                                  uint32_t utm_zone_id, double utm_x,
                                  double utm_y) {
  location_ = pos_gcj02;
  location_utm_.x() = utm_x;
  location_utm_.y() = utm_y;
  utm_zone_ = utm_zone_id;
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
      for (const auto& itt : it.line_segment().point()) {
        Eigen::Vector3d pt = UtmPtToLocalEnu(itt);
        local_lane.left_line.emplace_back(pt);
      }
    }
    for (const auto& it : lane->lane().right_boundary().curve().segment()) {
      for (const auto& itt : it.line_segment().point()) {
        Eigen::Vector3d pt = UtmPtToLocalEnu(itt);
        local_lane.right_line.emplace_back(pt);
      }
    }

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

Eigen::Vector3d MapTable::UtmPtToLocalEnu(
    const hozon::common::PointENU& point_utm) {
  double x = point_utm.x();
  double y = point_utm.y();
  hozon::common::coordinate_convertor::UTM2GCS(static_cast<int>(utm_zone_), &x,
                                               &y);
  Eigen::Vector3d point_gcj(y, x, 0);
  Eigen::Vector3d point_enu =
      util::Geo::Gcj02ToEnu(point_gcj, local_enu_center_);
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

void MapTable::CalculateTanTheta(const std::string& idd) {
  /*
        1、针对每个lane_id，判断其是否是变道虚线
        2、如果是变道虚拟线，循环判断其前继车道是否是变道虚拟线，直到找不到
        3、从第一个变道虚拟线的theta角,并填充到lane_table中
        4、循环查找其后继车道，如果是变道引导线，同样塞到lane_table中
        */
  double tan_theta = 0;
  if (lane_table_.find(idd) == lane_table_.end()) {
    HLOG_ERROR << "idd have not in lane_table_";
    return;
  }
  const auto& sec_id = lane_table_.at(idd).section_id;
  // 这里还不能使用sec_id,因为road还没存储完整
  const auto& road_id = lane_table_.at(idd).road_id;
  if (road_table_.find(road_id) == road_table_.end()) {
    return;
  }
  const auto& section = road_table_.at(road_id).section_ids;
  //   if (section.find(sec_id) == section.end()) {
  //     return;
  //   }
  auto curr_left_id =
      static_cast<uint32_t>(std::stoll(section.at(sec_id).lane_id.back()) % 10);

  if (lane_table_.at(idd).extra_boundary == 1) {
    // 该lane的左边线是变道引导线
    // 该变道引导线的左邻id
    const auto& lane_idd_left = std::to_string(std::stoll(idd) + 1);
    if (lane_table_.find(lane_idd_left) == lane_table_.end()) {
      return;
    }
    double l1 = lane_table_.at(idd).length;
    double l2 = lane_table_.at(lane_idd_left).length;
    // 判断前继
    uint32_t prev_num = 0;
    ObtainPrevLength(&l1, &l2, &prev_num, idd);
    // 判断后继
    uint32_t next_num = 0;
    ObtainNextLength(&l1, &l2, &next_num, idd);

    if (l1 == 0) {
      return;
    }
    double cos_theta = l2 / l1;
    double tan_theta = std::sqrt(1.0 - cos_theta * cos_theta) / cos_theta;
    // tan_theta = 0.05;
    if (cos_theta >= 1.0) {
      tan_theta = 0.05;
    }
    if ((next_num < prev_num && prev_num == curr_left_id) ||
        ((next_num - prev_num) > 2 && next_num > prev_num)) {
      tan_theta = -tan_theta;
    }
    lane_table_.at(idd).tan_theta = tan_theta;

    // ((next_num - prev_num) > 2 && next_num > prev_num)
    // 将lane_idd的后继车道是变道引导线的车道也赋上tan_theta
    // while (lane_table_.at(lane_idd).extra_boundary == 1) {
    //   const auto& lane_idd_next = lane_table_.at(lane_idd).next_lane_ids;
    //   if (lane_idd_next.empty()) {
    //     break;
    //   }
    //   const auto& lane_idd_next_id = lane_idd_next[0];
    //   if (lane_table_.find(lane_idd_next_id) == lane_table_.end()) {
    //     break;
    //   }
    //   if (lane_table_.at(lane_idd_next_id).extra_boundary == 1) {
    //     lane_table_.at(lane_idd_next_id).tan_theta = tan_theta;
    //     lane_idd = lane_idd_next_id;
    //   } else {
    //     break;
    //   }
    // }
  }
}

void MapTable::ObtainPrevLength(double* l1, double* l2, uint32_t* prev_num,
                                const std::string& idd) {
  // prev
  // 求出此时idd_prev[0]车道所在section的车道数
  const auto& sec_id = lane_table_.at(idd).section_id;
  // 这里还不能使用sec_id,因为road还没存储完整
  const auto& road_id = lane_table_.at(idd).road_id;
  if (road_table_.find(road_id) == road_table_.end()) {
    return;
  }
  const auto& section = road_table_.at(road_id).section_ids;
  // if (section.find(sec_id) == section.end()) {
  //   return;
  // }
  auto curr_left_id =
      static_cast<uint32_t>(std::stoll(section.at(sec_id).lane_id.back()) % 10);
  auto lane_idd = idd;
  while (lane_table_.at(lane_idd).extra_boundary == 1) {
    auto idd_prev = lane_table_.at(lane_idd).prev_lane_ids;
    if (idd_prev.empty()) {
      break;
    }
    if (lane_table_.find(idd_prev[0]) == lane_table_.end()) {
      break;
    }
    // 求出此时idd_prev[0]车道所在section的车道数
    const auto& sec_id = lane_table_.at(idd_prev[0]).section_id;
    // 这里还不能使用sec_id,因为road还没存储完整
    const auto& road_id = lane_table_.at(idd_prev[0]).road_id;
    if (road_table_.find(road_id) == road_table_.end()) {
      break;
    }
    const auto& section = road_table_.at(road_id).section_ids;
    // if (section.find(sec_id) == section.end()) {
    //   break;
    // }
    auto prev_left_id = static_cast<uint32_t>(
        std::stoll(section.at(sec_id).lane_id.back()) % 10);
    // auto prev_left_id = static_cast<uint32_t>(
    //     static_cast<unsigned
    //     char>(section.at(sec_id).lane_id.back().back()));
    auto extra_boundary = lane_table_.at(idd_prev[0]).extra_boundary;
    if (extra_boundary == 1 && prev_left_id == curr_left_id) {
      lane_idd = idd_prev[0];
      const auto& lane_idd_left = std::to_string(std::stoll(lane_idd) + 1);
      if (lane_table_.find(lane_idd_left) == lane_table_.end()) {
        break;
      }
      *l1 += lane_table_.at(lane_idd).length;
      *l2 += lane_table_.at(lane_idd_left).length;
    } else {
      *prev_num = prev_left_id;
      break;
    }
  }
}

void MapTable::ObtainNextLength(double* l1, double* l2, uint32_t* next_num,
                                const std::string& idd) {
  // next
  // 求出此时idd_prev[0]车道所在section的车道数
  const auto& sec_id = lane_table_.at(idd).section_id;
  // 这里还不能使用sec_id,因为road还没存储完整
  const auto& road_id = lane_table_.at(idd).road_id;
  if (road_table_.find(road_id) == road_table_.end()) {
    return;
  }
  const auto& section = road_table_.at(road_id).section_ids;
  // if (section.find(sec_id) == section.end()) {
  //   return;
  // }
  // auto curr_lane_num = section.at(sec_id).lane_id.size();
  auto curr_left_id =
      static_cast<uint32_t>(std::stoll(section.at(sec_id).lane_id.back()) % 10);
  auto lane_iddd = idd;
  while (lane_table_.at(lane_iddd).extra_boundary == 1) {
    auto idd_next = lane_table_.at(lane_iddd).next_lane_ids;
    if (idd_next.empty()) {
      break;
    }
    if (lane_table_.find(idd_next[0]) == lane_table_.end()) {
      break;
    }
    // 求出此时lane_iddd车道所在section的车道数
    const auto& sec_id = lane_table_.at(idd_next[0]).section_id;
    const auto& road_id = lane_table_.at(idd_next[0]).road_id;
    if (road_table_.find(road_id) == road_table_.end()) {
      break;
    }
    const auto& section = road_table_.at(road_id).section_ids;
    // auto lane_num = section.at(sec_id).lane_id.size();
    auto extra_boundary = lane_table_.at(idd_next[0]).extra_boundary;
    auto next_left_id = static_cast<uint32_t>(
        std::stoll(section.at(sec_id).lane_id.back()) % 10);
    if (extra_boundary == 1 && next_left_id == curr_left_id) {
      lane_iddd = idd_next[0];
      const auto& lane_idd_left = std::to_string(std::stoll(lane_iddd) + 1);
      if (lane_table_.find(lane_idd_left) == lane_table_.end()) {
        break;
      }
      *l1 += lane_table_.at(lane_iddd).length;
      *l2 += lane_table_.at(lane_idd_left).length;
    } else {
      // *next_num = section.at(sec_id).lane_id.size();
      *next_num = next_left_id;
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
    if (left_virtual_line_.find(sec_lane_id[i]) != left_virtual_line_.end()) {
      lane_table_.at(sec_lane_id[i]).pred_left_line =
          left_virtual_line_.at(sec_lane_id[i]);
    } else {
      lane_table_.at(sec_lane_id[i]).pred_left_line =
          predict_lanelines[pred_line_size - i - 2];
    }
    lane_table_.at(sec_lane_id[i]).pred_right_line =
        predict_lanelines[pred_line_size - i - 1];
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
  if (!predict_line.empty()) {
    *predict_lanelines = predict_line;
  }
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
                    predict_line, &flag);
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
    const int& index, std::vector<std::vector<Eigen::Vector3d>>* predict_line,
    bool* flag) {
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
      // 创建函数检测sec_lane_id[j]的前继是否也是变道引导线
      double l = 0;
      JudgePrevLength(&l, sec_lane_id[j]);
      const auto& tan_theta = lane_table_.at(sec_lane_id[j]).tan_theta;
      const auto& cos_theta = 1 / sqrt(1 + pow(tan_theta, 2));
      const auto& sin_theta = tan_theta / sqrt(1 + pow(tan_theta, 2));
      // index = static_cast<int>((s / cos_theta) / 0.5);
      // 左虚拟车道线的点
      double virtual_width = 0.;
      if (tan_theta < 0) {
        virtual_width = width + (s + l) * tan_theta;
      } else {
        virtual_width = width + (s + l) * tan_theta -
                        lane_table_.at(sec_lane_id[j + 1]).lane_width[index];
      }
      Eigen::Vector3d virtual_point = A - AB_C * virtual_width;
      if (*flag) {
        virtual_line->emplace_back(virtual_point);
        *virtual_id = sec_lane_id[j];
        *flag = false;
      }
      width = virtual_width + wid[index];
    } else {
      width += wid[index];
    }
    Eigen::Vector3d point = A - AB_C * width;
    predict_line->at(sec_lane_id.size() - j).push_back(point);
  }
}

void MapTable::JudgePrevLength(double* l, const std::string& lane_id) {
  // 判断其前继车道是否是左变道引导线
  // 求出此时lane_id车道所在section的车道数
  const auto& sec_id = lane_table_.at(lane_id).section_id;
  // 这里还不能使用sec_id,因为road还没存储完整
  const auto& road_id = lane_table_.at(lane_id).road_id;
  if (road_table_.find(road_id) == road_table_.end()) {
    return;
  }
  const auto& section = road_table_.at(road_id).section_ids;
  //   if (section.find(sec_id) == section.end()) {
  //     return;
  //   }
  auto curr_left_id =
      static_cast<uint32_t>(std::stoll(section.at(sec_id).lane_id.back()) % 10);

  // auto lane_num = section.at(sec_id).lane_id.size();

  auto lane_idd = lane_id;
  while (lane_table_.at(lane_idd).extra_boundary == 1) {
    const auto& lane_prev_idds = lane_table_.at(lane_idd).prev_lane_ids;
    if (lane_prev_idds.empty()) {
      break;
    }
    const auto& lane_prev_id = lane_prev_idds[0];
    if (lane_table_.find(lane_prev_id) == lane_table_.end()) {
      break;
    }
    const auto& sec_id = lane_table_.at(lane_prev_id).section_id;
    const auto& road_id = lane_table_.at(lane_prev_id).road_id;
    if (road_table_.find(road_id) == road_table_.end()) {
      return;
    }
    const auto& section = road_table_.at(road_id).section_ids;
    // if (section.find(sec_id) == section.end()) {
    //   return;
    // }
    // auto pre_lane_num = section.at(sec_id).lane_id.size();
    auto pre_left_id = static_cast<uint32_t>(
        std::stoll(section.at(sec_id).lane_id.back()) % 10);
    if (lane_table_.at(lane_prev_id).extra_boundary == 1) {
      if (std::abs(static_cast<int>(pre_left_id - curr_left_id)) == 1) {
        break;
      }
      const auto& left_prev = std::to_string(std::stoll(lane_prev_id) + 1);
      if (lane_table_.find(left_prev) == lane_table_.end()) {
        break;
      }
      const auto& length = lane_table_.at(left_prev).length;
      *l += length;
      lane_idd = lane_prev_id;
    } else {
      break;
    }
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
      double l = 0;
      JudgePrevLength(&l, sec_lane_id[j]);
      double s = lane_table_.at(sec_lane_id[j]).length;
      const auto& tan_theta = lane_table_.at(sec_lane_id[j]).tan_theta;
      const auto& cos_theta = 1 / sqrt(1 + pow(tan_theta, 2));
      const auto& sin_theta = tan_theta / sqrt(1 + pow(tan_theta, 2));
      double virtual_width = 0.;
      // 判断该引导线的后继是否是引导线
      auto is_next_vir = JudgeNextIsVir(sec_lane_id[j]);
      if (tan_theta < 0 && is_next_vir) {
        virtual_width = width + (s + l) * tan_theta;
      } else if (tan_theta < 0 && !is_next_vir) {
        virtual_width =
            width - lane_table_.at(sec_lane_id[j + 1]).lane_width.back();
      } else if (tan_theta > 0 && is_next_vir) {
        virtual_width = width + (s + l) * tan_theta -
                        lane_table_.at(sec_lane_id[j + 1]).lane_width.back();
      } else {
        virtual_width = width;
      }
      Eigen::Vector3d virtual_point = A + AB_C * virtual_width;
      virtual_line->emplace_back(virtual_point);
      *virtual_id = sec_lane_id[j];
      width = virtual_width + wid.back();
    } else {
      width += wid.back();
    }
    //  width += wid.back();
    Eigen::Vector3d point = A + AB_C * width;
    predict_line->at(sec_lane_id.size() - j).push_back(point);
  }
}

bool MapTable::JudgeNextIsVir(const std::string& lane_id) {
  if (lane_table_.find(lane_id) == lane_table_.end()) {
    return true;
  }
  const auto& next_ids = lane_table_.at(lane_id).next_lane_ids;
  if (next_ids.empty()) {
    return true;
  }
  for (const auto& it : next_ids) {
    if (lane_table_.find(it) == lane_table_.end()) {
      continue;
    }
    if (lane_table_.at(it).extra_boundary != 1) {
      return false;
      break;
    }
  }
  return true;
}

void MapTable::Clear() {
  lane_table_.clear();
  road_table_.clear();
  all_section_ids_.clear();
  left_virtual_line_.clear();
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
