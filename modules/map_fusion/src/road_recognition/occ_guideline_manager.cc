/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： group_map.cc
 *   author     ： taoshaoyuan
 *   date       ： 2024.01
 ******************************************************************************/

#include "map_fusion/road_recognition/occ_guideline_manager.h"
#include <math.h>
#include <pcl/segmentation/extract_clusters.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iterator>
#include <map>
#include <memory>
#include <numeric>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "base/utils/log.h"
#include "common/math/vec2d.h"
#include "map_fusion/fusion_common/calc_util.h"
#include "map_fusion/fusion_common/common_data.h"
#include "map_fusion/fusion_common/elemap2proto.h"
#include "map_fusion/fusion_common/element_map.h"
#include "map_fusion/fusion_common/proto2elemap.h"
#include "map_fusion/road_recognition/base_data.h"
#include "proto/local_mapping/local_map.pb.h"

namespace hozon {
namespace mp {
namespace mf {

void OccGuideLineManager::Process(
    const em::ElementMap::Ptr& ele_map,
    std::map<int, std::vector<Line_kd>>* bev_linelines,
    const KinePose& curr_pose, const RoadScene& road_scene) {
  HLOG_DEBUG << "current ts:" << std::to_string(curr_pose.stamp);
  HLOG_DEBUG << "current road scene is:" << static_cast<int>(road_scene);
  std::vector<em::Boundary> virtual_guide_lines;
  input_ele_map_ = ele_map;
  curr_pose_ = curr_pose;
  bevlanelines_ = bev_linelines;

  if (road_scene == RoadScene::NON_JUNCTION) {
    // 非路口时.进行数据重置操作
    Reset();
    return;
  }

  // 将all_lines中的车道线转换为em::Boundary格式，方便后续数据处理。
  bev_laneline_boundarys_.clear();
  // std::vector<std::pair<int, em::Boundary::Ptr>> bev_laneline_boundarys;
  for (auto& line : *bevlanelines_) {
    if (line.second.empty()) {
      continue;
    }
    for (const auto& line_vector : line.second) {
      const auto& line = line_vector.line;
      em::Boundary::Ptr bev_laneline_boundary = TransformPbLine2Boundary(line);
      bev_laneline_boundarys_.emplace_back(line->track_id(),
                                           bev_laneline_boundary);
    }
  }

  RecordExitLaneInfos();

  if (!is_occ_stable_state_) {
    // occ状态不稳定，则添加当前帧的occ观测数据。
    AddCurrentMeasurementOcc();
  }

  if (!CheckOccWhetherStable()) {
    HLOG_DEBUG << "[occ module] Occ stable state is false";
    return;
  }

  HLOG_DEBUG << "[occ module] Occ stable state is true";
  SetStableOcc();
  // 如果模型路沿成对出现, 则使用模型路沿位置代替occ豁口对.
  // 注释该函数，两者几何特性相差比较大，不能使用模型路沿优化occ路沿，暂时以occ路沿为准。
  // ReplaceOccLocByBevRoadEdge();

  // 计算occ方向向量。
  auto best_occ_pair = GetStableOcc();
  const auto& left_occ = best_occ_pair.back();
  const auto& right_occ = best_occ_pair.front();
  occ_dir_vec_ = CalcuDirectionVecV2(right_occ, left_occ);

  FineTuneOccPair(best_occ_pair);
  std::vector<em::Boundary::Ptr> bev_lanelines = GetBevLaneLineInOcc();
  if (!bev_lanelines.empty()) {
    HLOG_DEBUG << "[occ module] Occ region has some bev laneline..."
               << bev_lanelines.size();
    virtual_guide_lines =
        FineTuneGuideLineV2(bev_lanelines);  // (感知出现的线保留, 其他的线虚拟)
  } else {
    // 只根据occ区域来生成引导线(全是虚拟线)
    HLOG_DEBUG << "[occ module] Occ region has no bev laneline...";
    virtual_guide_lines = InferGuideLineOnlyByOcc();
  }

#if 1
  HLOG_DEBUG << "[occ module] Occ virtual_guide_lines size..."
             << virtual_guide_lines.size();
  for (int line_idx = 0; line_idx < virtual_guide_lines.size(); ++line_idx) {
    auto& virtual_line = virtual_guide_lines.at(line_idx);
    std::shared_ptr<hozon::mapping::LaneLine> laneline =
        TransformBoundary2PbLine(std::make_shared<em::Boundary>(virtual_line));
    laneline->set_track_id(3000 + line_idx);
    std::vector<cv::Point2f> kdtree_points;
    Line_kd line_kd;
    for (const auto& point : laneline->points()) {
      if (std::isnan(point.x()) || std::isnan(point.y()) ||
          std::isnan(point.z())) {
        HLOG_ERROR << "found nan point in local map lane line";
        continue;
      }
      Eigen::Vector3d point_vehicle(point.x(), point.y(), point.z());
      kdtree_points.emplace_back(static_cast<float>(point_vehicle.x()),
                                 static_cast<float>(point_vehicle.y()));
    }
    if (kdtree_points.size() < 2) {
      continue;
    }
    cv::flann::KDTreeIndexParams index_param(1);
    std::shared_ptr<cv::flann::Index> kdtree_ptr =
        std::make_shared<cv::flann::Index>(cv::Mat(kdtree_points).reshape(1),
                                           index_param);
    line_kd.line_kdtree = kdtree_ptr;
    line_kd.line = laneline;
    (*bevlanelines_)[static_cast<int>(laneline->lanepos())].emplace_back(
        line_kd);
  }
#endif
}

void OccGuideLineManager::RecordExitLaneInfos() {
  if (exit_lane_info_.exist) {
    return;
  }

  std::vector<em::Boundary::Ptr> exit_lane_boundarys;
  for (const auto& boundary : bev_laneline_boundarys_) {
    auto& nodes = boundary.second->nodes;
    if (nodes.front()->point.x() <= 0.0) {
      exit_lane_boundarys.emplace_back(boundary.second);
    }
  }

  if (exit_lane_boundarys.size() < 2) {
    exit_lane_info_.exist = false;
    return;
  }

  // 从右到左进行排序
  std::sort(exit_lane_boundarys.begin(), exit_lane_boundarys.end(),
            [](const em::Boundary::Ptr& a, const em::Boundary::Ptr& b) {
              return a->nodes.front()->point.y() < b->nodes.front()->point.y();
            });

  const auto& right_lane = exit_lane_boundarys.front()->nodes;
  const auto& left_lane = exit_lane_boundarys.back()->nodes;
  if (!exit_lane_info_.right_boundary_points.empty() ||
      !exit_lane_info_.left_boundary_points.empty()) {
    exit_lane_info_.right_boundary_points.clear();
    exit_lane_info_.left_boundary_points.clear();
  }

  for (const auto& node : right_lane) {
    Eigen::Vector3f f_point{node->point.x(), node->point.y(), node->point.z()};
    auto local_pt = curr_pose_.TransformPoint(f_point);
    exit_lane_info_.right_boundary_points.emplace_back(local_pt);
  }

  for (const auto& node : left_lane) {
    Eigen::Vector3f f_point{node->point.x(), node->point.y(), node->point.z()};
    auto local_pt = curr_pose_.TransformPoint(f_point);
    exit_lane_info_.left_boundary_points.emplace_back(local_pt);
  }

  exit_lane_info_.exist = true;
}

void OccGuideLineManager::SetStableOcc() {
  is_occ_stable_state_ = true;
  stable_occ_datas_.clear();

  auto& right_occ = history_n_best_occs_.back().front();
  auto& left_occ = history_n_best_occs_.back().back();

  em::Boundary::Ptr right_vehicle_occ = std::make_shared<em::Boundary>();
  std::vector<hozon::common::math::Vec2d> src_right_pts;
  for (const auto& node : right_occ->nodes) {
    Eigen::Vector3f f_point{node->point.x(), node->point.y(), node->point.z()};
    auto vehicle_point = curr_pose_.TransLocalToVehicle(f_point);
    em::BoundaryNode vehicle_node;
    vehicle_node.point = vehicle_point;
    right_vehicle_occ->nodes.emplace_back(
        std::make_shared<em::BoundaryNode>(vehicle_node));
    src_right_pts.emplace_back(
        hozon::common::math::Vec2d{vehicle_point.x(), vehicle_point.y()});
  }

  std::vector<double> fit_result;
  math::FitLaneLinePoint(src_right_pts, &fit_result);
  right_vehicle_occ->curve_params = std::vector<float>{
      static_cast<float>(fit_result[0]), static_cast<float>(fit_result[1]),
      static_cast<float>(fit_result[2]), static_cast<float>(fit_result[3])};
  stable_occ_datas_.emplace_back(right_vehicle_occ);
  HLOG_DEBUG << "occ id:" << right_vehicle_occ->id << ", "
             << "right_occ_curve_param: " << right_vehicle_occ->curve_params[0]
             << "," << right_vehicle_occ->curve_params[1] << ","
             << right_vehicle_occ->curve_params[2] << ","
             << right_vehicle_occ->curve_params[3];

  em::Boundary::Ptr left_vehicle_occ = std::make_shared<em::Boundary>();
  std::vector<hozon::common::math::Vec2d> src_left_pts;
  for (const auto& node : left_occ->nodes) {
    Eigen::Vector3f f_point{node->point.x(), node->point.y(), node->point.z()};
    auto vehicle_point = curr_pose_.TransLocalToVehicle(f_point);
    em::BoundaryNode vehicle_node;
    vehicle_node.point = vehicle_point;
    left_vehicle_occ->nodes.emplace_back(
        std::make_shared<em::BoundaryNode>(vehicle_node));
    src_left_pts.emplace_back(
        hozon::common::math::Vec2d{vehicle_point.x(), vehicle_point.y()});
  }

  fit_result.clear();
  math::FitLaneLinePoint(src_left_pts, &fit_result);
  left_vehicle_occ->curve_params = std::vector<float>{
      static_cast<float>(fit_result[0]), static_cast<float>(fit_result[1]),
      static_cast<float>(fit_result[2]), static_cast<float>(fit_result[3])};
  stable_occ_datas_.emplace_back(left_vehicle_occ);

  HLOG_DEBUG << "occ id:" << left_vehicle_occ->id << ", "
             << "left_occ_curve_param: " << left_vehicle_occ->curve_params[0]
             << "," << left_vehicle_occ->curve_params[1] << ","
             << left_vehicle_occ->curve_params[2] << ","
             << left_vehicle_occ->curve_params[3];
}

std::vector<em::Boundary::Ptr> OccGuideLineManager::GetStableOcc() {
  return stable_occ_datas_;
}

em::Boundary::Ptr OccGuideLineManager::TransformPbLine2Boundary(
    const std::shared_ptr<hozon::mapping::LaneLine>& pb_line) {
  if (pb_line == nullptr) {
    return nullptr;
  }

  em::Boundary::Ptr bev_laneline_boundary = std::make_shared<em::Boundary>();
  for (const auto& point : pb_line->points()) {
    em::BoundaryNode::Ptr b_point = std::make_shared<em::BoundaryNode>();
    b_point->point = Eigen::Vector3f{static_cast<float>(point.x()),
                                     static_cast<float>(point.y()),
                                     static_cast<float>(point.z())};
    bev_laneline_boundary->nodes.emplace_back(b_point);
  }

  // hozon::mp::mf::em::Boundary* lane_line,
  // hozon::mapping::LaneType lanetype
  em::FillLaneType(bev_laneline_boundary.get(), pb_line->lanetype());
  em::FillLanePos(bev_laneline_boundary.get(), pb_line->lanepos());
  em::FillLaneColor(bev_laneline_boundary.get(), pb_line->color());
  bev_laneline_boundary->id = pb_line->track_id();
  return bev_laneline_boundary;
}

std::shared_ptr<hozon::mapping::LaneLine>
OccGuideLineManager::TransformBoundary2PbLine(const em::Boundary::Ptr& line) {
  if (line == nullptr) {
    return nullptr;
  }

  std::shared_ptr<hozon::mapping::LaneLine> bev_laneline =
      std::make_shared<hozon::mapping::LaneLine>();
  for (const auto& node : line->nodes) {
    auto* pb_point = bev_laneline->add_points();
    pb_point->set_x(static_cast<double>(node->point.x()));
    pb_point->set_y(static_cast<double>(node->point.y()));
    pb_point->set_z(static_cast<double>(node->point.z()));
  }

  hozon::mapping::Color lanecolor = hozon::mapping::Color::UNKNOWN;
  em::SetLaneColor(*line, &lanecolor);
  bev_laneline->set_color(lanecolor);

  hozon::mapping::LanePositionType lanepos =
      hozon::mapping::LanePositionType::LanePositionType_OTHER;
  em::SetLanePos(*line, &lanepos);
  bev_laneline->set_lanepos(lanepos);

  hozon::mapping::LaneType lanetype = hozon::mapping::LaneType::LaneType_OTHER;
  em::SetLaneType(*line, &lanetype);
  bev_laneline->set_lanetype(lanetype);
  return bev_laneline;
}

bool OccGuideLineManager::ComputerPointIsInLine(const Eigen::Vector3f& P,
                                                const Eigen::Vector3f& A,
                                                const Eigen::Vector3f& B) {
  Eigen::Vector3f AB = B - A;
  Eigen::Vector3f AP = P - A;
  double ABLength = AB.norm();
  if (ABLength < 0.01) {
    return false;
  }
  double t = AB.dot(AP) / (ABLength * ABLength);
  HLOG_DEBUG << "[occ module] t value:" << t;
  return t >= -0.01 && t <= 1.01;
}

bool OccGuideLineManager::IsPointOnOccRightSide(
    const em::BoundaryNode::Ptr& node, const em::Boundary::Ptr& occ_data) {
  bool flag = false;

  if (occ_data->nodes.size() == 1) {
    flag = node->point.y() < occ_data->nodes.at(0)->point.y();
    return flag;
  }

  Eigen::Vector2f query_node{node->point.x(), node->point.y()};
  Eigen::Vector3f second_point_it;
  Eigen::Vector3f first_point_it;
  auto best_point_it = std::min_element(
      occ_data->nodes.begin(), occ_data->nodes.end(),
      [&point = node->point](const em::BoundaryNode::Ptr& a,
                             const em::BoundaryNode::Ptr& b) {
        return (point - a->point).norm() < (point - b->point).norm();
      });

  if (best_point_it == std::prev(occ_data->nodes.end())) {
    first_point_it = (*std::prev(best_point_it))->point;
    second_point_it = (*best_point_it)->point;
  } else {
    first_point_it = (*best_point_it)->point;
    second_point_it = (*std::next(best_point_it))->point;
  }

  HLOG_DEBUG << "[occ module] occ first node point:" << first_point_it.x()
             << "," << first_point_it.y();
  HLOG_DEBUG << "[occ module] occ second node point:" << second_point_it.x()
             << "," << second_point_it.y();

  flag = PointInVectorSide(
             Eigen::Vector2f{first_point_it.x(), first_point_it.y()},
             Eigen::Vector2f{second_point_it.x(), second_point_it.y()},
             query_node) >= 0;

  bool add_flag = PointInVectorSide(
                      Eigen::Vector2f{first_point_it.x(), first_point_it.y()},
                      Eigen::Vector2f{second_point_it.x(), second_point_it.y()},
                      query_node) <= 0 &&
                  PointToVectorDist(
                      Eigen::Vector2f{first_point_it.x(), first_point_it.y()},
                      Eigen::Vector2f{second_point_it.x(), second_point_it.y()},
                      query_node) < 1.0;

  return flag || add_flag;
}

bool OccGuideLineManager::IsPointOnOccLeftSide(
    const em::BoundaryNode::Ptr& node, const em::Boundary::Ptr& occ_data) {
  bool flag = false;

  if (occ_data->nodes.size() == 1) {
    flag = node->point.y() > occ_data->nodes.at(0)->point.y();
    return flag;
  }

  Eigen::Vector2f query_node{node->point.x(), node->point.y()};
  Eigen::Vector3f second_point_it;
  Eigen::Vector3f first_point_it;
  auto best_point_it = std::min_element(
      occ_data->nodes.begin(), occ_data->nodes.end(),
      [&point = node->point](const em::BoundaryNode::Ptr& a,
                             const em::BoundaryNode::Ptr& b) {
        return (point - a->point).norm() < (point - b->point).norm();
      });

  if (best_point_it == std::prev(occ_data->nodes.end())) {
    first_point_it = (*std::prev(best_point_it))->point;
    second_point_it = (*best_point_it)->point;
  } else {
    first_point_it = (*best_point_it)->point;
    second_point_it = (*std::next(best_point_it))->point;
  }

  HLOG_DEBUG << "[occ module] occ first node point:" << first_point_it.x()
             << "," << first_point_it.y();
  HLOG_DEBUG << "[occ module] occ second node point:" << second_point_it.x()
             << "," << second_point_it.y();

  flag = PointInVectorSide(
             Eigen::Vector2f{first_point_it.x(), first_point_it.y()},
             Eigen::Vector2f{second_point_it.x(), second_point_it.y()},
             query_node) <= 0;

  bool add_flag = PointInVectorSide(
                      Eigen::Vector2f{first_point_it.x(), first_point_it.y()},
                      Eigen::Vector2f{second_point_it.x(), second_point_it.y()},
                      query_node) >= 0 &&
                  PointToVectorDist(
                      Eigen::Vector2f{first_point_it.x(), first_point_it.y()},
                      Eigen::Vector2f{second_point_it.x(), second_point_it.y()},
                      query_node) < 1.0;

  return flag || add_flag;
}

bool OccGuideLineManager::IsLineBetweenOcc(const em::Boundary::Ptr& line) {
  if (line == nullptr || line->nodes.empty()) {
    return false;
  }

  auto stable_occs = GetStableOcc();
  auto& right_occ = stable_occs.front();
  auto& left_occ = stable_occs.back();
  HLOG_DEBUG << "right occ node size:" << right_occ->nodes.size();
  HLOG_DEBUG << "left occ node size:" << left_occ->nodes.size();

  if (right_occ->nodes.empty() || left_occ->nodes.empty()) {
    return false;
  }

  for (auto& node : right_occ->nodes) {
    HLOG_DEBUG << "right occ point:" << node->point.x() << ","
               << node->point.y();
  }

  for (auto& node : left_occ->nodes) {
    HLOG_DEBUG << "left occ point:" << node->point.x() << ","
               << node->point.y();
  }

  Eigen::Vector2f right_occ_start_point =
      right_occ->nodes.front()->point.head<2>();
  Eigen::Vector2f right_occ_end_point =
      right_occ->nodes.back()->point.head<2>();
  Eigen::Vector2f left_occ_start_point =
      left_occ->nodes.front()->point.head<2>();
  Eigen::Vector2f left_occ_end_point = left_occ->nodes.back()->point.head<2>();
  float occ_min_max =
      std::max(left_occ_start_point.x(), right_occ_start_point.x());
  float occ_max_min = std::min(left_occ_end_point.x(), right_occ_end_point.x());

  int point_in_occ_num = 0;
  int valid_node_num = 0;
  for (const auto& node : line->nodes) {
    HLOG_DEBUG << "[occ module] node point:" << node->point.x() << ","
               << node->point.y();

    if (node->point.x() < occ_min_max || node->point.x() > occ_max_min) {
      continue;
    }

    valid_node_num++;

    bool in_occ_left = IsPointOnOccLeftSide(node, right_occ);
    bool in_occ_right = IsPointOnOccRightSide(node, left_occ);

    if (in_occ_left && in_occ_right) {
      ++point_in_occ_num;
      HLOG_DEBUG << "[occ module] node point in occ region:";
    } else {
      HLOG_DEBUG << "[occ module] node point not in occ region:";
    }
  }

  HLOG_DEBUG << "[occ module] points in occ ratio:"
             << 1.0 * point_in_occ_num / (valid_node_num + 0.001);
  bool line_in_occ = (1.0 * point_in_occ_num / (valid_node_num + 0.001)) > 0.8;

  return line_in_occ;
}

std::vector<em::Boundary::Ptr> OccGuideLineManager::GetBevLaneLineInOcc() {
  std::vector<em::Boundary::Ptr> bev_lanelines_in_occ;
  const auto stable_occs = GetStableOcc();
  const auto& right_occ = stable_occs.front();
  const auto& left_occ = stable_occs.back();
  Eigen::Vector2f right_occ_start_point =
      right_occ->nodes.front()->point.head<2>();
  Eigen::Vector2f right_occ_end_point =
      right_occ->nodes.back()->point.head<2>();
  Eigen::Vector2f left_occ_start_point =
      left_occ->nodes.front()->point.head<2>();
  Eigen::Vector2f left_occ_end_point = left_occ->nodes.back()->point.head<2>();

  for (const auto& laneline : bev_laneline_boundarys_) {
    HLOG_DEBUG << "[occ module] input laneline id:" << laneline.first;
    // 判断bev车道线是否位于occ之间。
    if (IsLineBetweenOcc(laneline.second)) {
      HLOG_DEBUG << "[occ module] laneline id:" << laneline.first
                 << " in occ pairs region";
      Eigen::Vector2f laneline_start_point =
          laneline.second->nodes.front()->point.head<2>();
      Eigen::Vector2f laneline_end_point =
          laneline.second->nodes.back()->point.head<2>();
      float occ_min_x =
          std::min(left_occ_start_point.x(), right_occ_start_point.x());
      float occ_max_x =
          std::max(left_occ_end_point.x(), right_occ_end_point.x());
      float x_start = std::max(occ_min_x, laneline_start_point.x());
      float x_end = std::min(occ_max_x, laneline_end_point.x());
      if (x_end - x_start < 5.0) {
        HLOG_DEBUG << "but intersect length is to short..."
                   << " line length:" << x_end - x_start << "line start point,"
                   << laneline_start_point.x() << "line end point,"
                   << laneline_end_point.x();
        continue;
      }

      bev_lanelines_in_occ.emplace_back(laneline.second);
    } else {
      HLOG_DEBUG << "[occ module] laneline id:" << laneline.first
                 << " not in occ pairs region";
    }
  }

  // 从右到左进行排序
  std::sort(bev_lanelines_in_occ.begin(), bev_lanelines_in_occ.end(),
            [](const em::Boundary::Ptr& a, const em::Boundary::Ptr& b) {
              return a->nodes.front()->point.y() < b->nodes.front()->point.y();
            });

  return bev_lanelines_in_occ;
}

void OccGuideLineManager::ReplaceOccLocByBevRoadEdge() {
  auto stable_occs = GetStableOcc();
  auto& right_occ = stable_occs.front();
  auto& left_occ = stable_occs.back();
  const double right_occ_x_min = right_occ->nodes.front()->point.x();
  const double right_occ_x_max = right_occ->nodes.back()->point.x();

  const double left_occ_x_min = left_occ->nodes.front()->point.x();
  const double left_occ_x_max = left_occ->nodes.back()->point.x();
  const auto& bev_roadedges = input_ele_map_->road_edges;
  HLOG_DEBUG << "[occ module] bev roadedges size:" << bev_roadedges.size();

  em::Boundary::Ptr right_bev_roadedge = nullptr;
  em::Boundary::Ptr left_bev_roadedge = nullptr;

  // HLOG_DEBUG<< "right occ curve params:" << right_occ->curve_params[3] <<
  // ","
  //           << right_occ->curve_params[2] << "," <<
  //           right_occ->curve_params[1]
  //           << "," << right_occ->curve_params[0];
  // for (auto& node : right_occ->nodes) {
  //   HLOG_DEBUG<< "right occ: point loc:" << node->point.x() << ","
  //             << node->point.y();
  // }

  // HLOG_DEBUG<< "left occ curve params:" << left_occ->curve_params[3] << ","
  //           << left_occ->curve_params[2] << "," << left_occ->curve_params[1]
  //           << "," << left_occ->curve_params[0];
  // for (auto& node : left_occ->nodes) {
  //   HLOG_DEBUG<< "left occ: point loc:" << node->point.x() << ","
  //             << node->point.y();
  // }

  // 从所有的bev路沿中找到和occ右侧路沿空间上属于同一个的路沿.
  for (const auto& roadedge : bev_roadedges) {
    const double roadedge_start_x = roadedge.second->nodes.front()->point.x();
    const double roadedge_end_x = roadedge.second->nodes.back()->point.x();
    double x_start = std::max(roadedge_start_x, right_occ_x_min);
    double x_end = std::min(roadedge_end_x, right_occ_x_max);

    if (x_end - x_start < 30) {
      continue;
    }

    HLOG_DEBUG << "[occ module] right Occ with Bev roadedge intersect length:"
               << x_end - x_start;

    double delta_error = 0.0;
    double point_num = 0;
    for (const auto& node : roadedge.second->nodes) {
      double x = node->point.x();
      if (x < x_start || x > x_end) {
        continue;
      }
      std::vector<float> occ_curve = right_occ->curve_params;
      double occ_y = occ_curve[3] * std::pow(x, 3) +
                     occ_curve[2] * std::pow(x, 2) + occ_curve[1] * x +
                     occ_curve[0];
      // HLOG_DEBUG<< "bev roadedge point:" << x << "," << node->point.y() <<
      // "\n"
      //           << "occ roadedge point:" << x << "," << occ_y;
      delta_error += std::abs(occ_y - node->point.y());
      point_num += 1;
    }

    double ave_error = delta_error / point_num;
    HLOG_DEBUG << "[occ module] right Occ and Pair bev roadedge:"
               << "ave_y_error is:" << ave_error;
    if (ave_error <= y_distance_error_thresh_) {
      right_bev_roadedge = roadedge.second;
      break;
    }
  }

  // 从所有的bev路沿中找到和occ左侧路沿空间上属于同一个的路沿.
  for (const auto& roadedge : bev_roadedges) {
    const double roadedge_start_x = roadedge.second->nodes.front()->point.x();
    const double roadedge_end_x = roadedge.second->nodes.back()->point.x();
    double x_start = std::max(roadedge_start_x, left_occ_x_min);
    double x_end = std::min(roadedge_end_x, left_occ_x_max);
    if (x_end - x_start < 30) {
      continue;
    }

    HLOG_DEBUG << "[occ module] left Occ with Bev roadedge intersect length:"
               << x_end - x_start;

    double delta_error = 0.0;
    double point_num = 0;
    for (const auto& node : roadedge.second->nodes) {
      double x = node->point.x();
      if (x < x_start || x > x_end) {
        continue;
      }
      std::vector<float> occ_curve = left_occ->curve_params;
      double occ_y = occ_curve[3] * std::pow(x, 3) +
                     occ_curve[2] * std::pow(x, 2) + occ_curve[1] * x +
                     occ_curve[0];
      delta_error += std::abs(occ_y - node->point.y());
      point_num += 1;
    }

    double ave_error = delta_error / point_num;
    HLOG_DEBUG << "[occ module] left Occ and Pair bev roadedge:"
               << "ave_y_error is:" << ave_error;
    if (ave_error <= y_distance_error_thresh_) {
      left_bev_roadedge = roadedge.second;
      break;
    }
  }

  if (right_bev_roadedge != nullptr && left_bev_roadedge != nullptr) {
    *right_occ = *right_bev_roadedge;
    *left_occ = *left_bev_roadedge;
    float right_occ_start = right_occ->nodes.front()->point.x();
    float left_occ_start = left_occ->nodes.front()->point.x();

    /* 只是简单的用模型路沿点代替occ几何点会出现潜在问题
    （模型路沿在路口时存在弯曲的部分，occ则对此进行了截断优化）。*/
    right_occ->nodes.clear();
    left_occ->nodes.clear();
    for (const auto& bev_node : right_bev_roadedge->nodes) {
      if (bev_node->point.x() > right_occ_start) {
        right_occ->nodes.emplace_back(bev_node);
      }
    }

    for (const auto& bev_node : left_bev_roadedge->nodes) {
      if (bev_node->point.x() > left_occ_start) {
        left_occ->nodes.emplace_back(bev_node);
      }
    }

    HLOG_DEBUG
        << "[occ module] Find Pair bev left roadedge and right roadedge...";
  } else {
    HLOG_DEBUG << "[occ module] Can't Find Pair bev left roadedge and right "
                  "roadedge...";
  }
}

float OccGuideLineManager::GetEntranceLaneWidth(
    const std::vector<em::Boundary::Ptr>& bev_lanelines) {
  float averge_entrance_lane_width = averge_exit_lane_width_;
  if (bev_lanelines.size() < 2) {
    return averge_exit_lane_width_;
  }

  float front_lane_width_totally = 0.0;
  int front_lane_num_calu = 0;
  for (int i = 0; i < static_cast<int>(bev_lanelines.size()) - 1; ++i) {
    if (bev_lanelines.at(i)->nodes.empty() ||
        bev_lanelines.at(i + 1)->nodes.empty()) {
      continue;
    }
    float lane_space = std::fabs(
        GetTwoBoundayDis(bev_lanelines.at(i), bev_lanelines.at(i + 1)));

    if (lane_space < 5.0 && lane_space > 2.0) {
      front_lane_width_totally += lane_space;
      ++front_lane_num_calu;
    }
  }

  if (front_lane_num_calu != 0) {
    averge_entrance_lane_width =
        front_lane_width_totally / static_cast<float>(front_lane_num_calu);
  }
  if (averge_entrance_lane_width < 5.0 && averge_entrance_lane_width > 2.0) {
    return averge_entrance_lane_width;
  }

  return averge_exit_lane_width_;
}

float OccGuideLineManager::GetTwoBoundayDis(
    const em::Boundary::Ptr& left_boundary,
    const em::Boundary::Ptr& right_boundary) {
  float lane_space = 3.5;
  em::Boundary::Ptr query_line = nullptr;
  em::Boundary::Ptr value_line = nullptr;
  bool query_left_occ = true;
  if (left_boundary->nodes.front()->point.x() <
      right_boundary->nodes.front()->point.x()) {
    query_line = right_boundary;
    value_line = left_boundary;
    query_left_occ = false;
  } else {
    query_line = left_boundary;
    value_line = right_boundary;
    query_left_occ = true;
  }

  const auto& query_point = query_line->nodes.front()->point;
  auto it = std::min_element(
      value_line->nodes.begin(), value_line->nodes.end(),
      [&point = query_point](const em::BoundaryNode::Ptr& a,
                             const em::BoundaryNode::Ptr& b) {
        return (point - a->point).norm() < (point - b->point).norm();
      });

  if (value_line->nodes.size() == 1) {
    lane_space = ((*it)->point - query_point).norm();
  } else if (it == std::prev(value_line->nodes.end())) {
    lane_space = math::perpendicular_distance((*std::prev(it))->point,
                                              (*it)->point, query_point);
  } else {
    lane_space = math::perpendicular_distance((*std::next(it))->point,
                                              (*it)->point, query_point);
  }

  float output_distance = NAN;
  if (query_left_occ) {
    output_distance =
        (*it)->point.y() < query_point.y() ? -1 * lane_space : lane_space;
  } else {
    output_distance =
        (*it)->point.y() < query_point.y() ? lane_space : -1 * lane_space;
  }

  return output_distance;
}

float OccGuideLineManager::GetTwoBoundaryMinDis(
    const em::Boundary::Ptr& left_boundary,
    const em::Boundary::Ptr& right_boundary) {
  float lane_space = 3.5;
  float output_distance = NAN;
  em::Boundary::Ptr query_line = nullptr;
  em::Boundary::Ptr value_line = nullptr;
  bool query_left_occ = true;
  float min_dist_two_boundary = MAXFLOAT;
  if (left_boundary->nodes.front()->point.x() <
      right_boundary->nodes.front()->point.x()) {
    query_line = right_boundary;
    value_line = left_boundary;
    query_left_occ = false;
  } else {
    query_line = left_boundary;
    value_line = right_boundary;
    query_left_occ = true;
  }

  float point_max_x = std::min(left_boundary->nodes.back()->point.x(),
                               right_boundary->nodes.back()->point.x());

  auto it = std::min_element(
      value_line->nodes.begin(), value_line->nodes.end(),
      [&point = query_line->nodes.front()->point](
          const em::BoundaryNode::Ptr& a, const em::BoundaryNode::Ptr& b) {
        return (point - a->point).norm() < (point - b->point).norm();
      });
  min_dist_two_boundary =
      ((*it)->point - query_line->nodes.front()->point).norm();

  bool flag = false;
  for (auto& query_node : query_line->nodes) {
    const auto& query_point = query_node->point;

    if (query_point.x() > point_max_x) {
      continue;
    }
    auto it = std::min_element(
        value_line->nodes.begin(), value_line->nodes.end(),
        [&point = query_point](const em::BoundaryNode::Ptr& a,
                               const em::BoundaryNode::Ptr& b) {
          return (point - a->point).norm() < (point - b->point).norm();
        });

    if (value_line->nodes.size() == 1) {
      lane_space = ((*it)->point - query_point).norm();
      HLOG_DEBUG << "[debug lane space], -1111, " << lane_space;
    } else if (it == std::prev(value_line->nodes.end())) {
      lane_space = math::perpendicular_distance((*std::prev(it))->point,
                                                (*it)->point, query_point);
      HLOG_DEBUG << "[debug lane space], 000, " << lane_space;
    } else {
      lane_space = math::perpendicular_distance((*std::next(it))->point,
                                                (*it)->point, query_point);
      // HLOG_DEBUG<< "[debug lane space], 111, pointA" <<
      // (*std::next(it))->point.x() << "," << (*std::next(it))->point.y();
      // HLOG_DEBUG<< "[debug lane space], 111, pointB" << (*it)->point.x() <<
      // "," << (*it)->point.y(); HLOG_DEBUG<< "[debug lane space], 111, pointP"
      // << query_point.x() << "," << query_point.y(); HLOG_DEBUG<< "[debug lane
      // space], 111, " << lane_space;
    }

    if (lane_space < min_dist_two_boundary) {
      min_dist_two_boundary = lane_space;
      HLOG_DEBUG << "[debug min_dist_two_boundary], 111, "
                 << min_dist_two_boundary;
      if (query_left_occ) {
        flag = (*it)->point.y() < query_point.y();
      } else {
        flag = (*it)->point.y() > query_point.y();
      }
    }
  }

  if (flag) {
    min_dist_two_boundary = min_dist_two_boundary * -1;
  }
  HLOG_DEBUG << "[debug min_dist_two_boundary], 222, " << min_dist_two_boundary;

  return min_dist_two_boundary;
}

float OccGuideLineManager::GetTwoBoundayDis(const em::OccRoad::Ptr& left_occ,
                                            const em::OccRoad::Ptr& right_occ) {
  float lane_space = 3.5;
  em::OccRoad::Ptr query_line = nullptr;
  em::OccRoad::Ptr value_line = nullptr;
  bool query_left_occ = true;
  if (left_occ->road_points.front().x() < right_occ->road_points.front().x()) {
    query_line = right_occ;
    value_line = left_occ;
    query_left_occ = false;
  } else {
    query_line = left_occ;
    value_line = right_occ;
    query_left_occ = true;
  }

  const auto& query_point = query_line->road_points.front();
  auto it = std::min_element(value_line->road_points.begin(),
                             value_line->road_points.end(),
                             [&point = query_point](const Eigen::Vector3d& a,
                                                    const Eigen::Vector3d& b) {
                               return (point - a).norm() < (point - b).norm();
                             });

  if (value_line->road_points.size() == 1) {
    lane_space = static_cast<float>(((*it) - query_point).norm());
  } else if (it == std::prev(value_line->road_points.end())) {
    lane_space =
        math::perpendicular_distance((*std::prev(it)), (*it), query_point);
  } else {
    lane_space =
        math::perpendicular_distance((*std::next(it)), (*it), query_point);
  }

  float output_distance = NAN;
  if (query_left_occ) {
    output_distance =
        (*it).y() < query_point.y() ? -1 * lane_space : lane_space;
  } else {
    output_distance =
        (*it).y() < query_point.y() ? lane_space : -1 * lane_space;
  }

  return output_distance;
}

std::vector<em::Boundary> OccGuideLineManager::FineTuneGuideLine(
    const std::vector<em::Boundary::Ptr>& bev_lanelines) {
  std::vector<em::Boundary> virutal_lines;
  int vitual_lane_num = -1;

  auto stable_occs = GetStableOcc();
  auto& right_occ = stable_occs.front();
  auto& left_occ = stable_occs.back();

  averge_entrance_lane_width_ = GetEntranceLaneWidth(bev_lanelines);
  // 如果退出车道数计算过，则保持历史车道数。反之，需计算退出车道数。
  if (assume_entrancelane_nums_by_entrancelane_ == -1) {
    assume_entrancelane_nums_by_entrancelane_ = static_cast<int>(std::max(
        static_cast<double>(std::floor((occ_width_ - safe_distance_ * 2) /
                                           averge_entrance_lane_width_ +
                                       0.3)),
        1.0));
  }

  HLOG_DEBUG << "[occ module] fine_lane_width:" << averge_entrance_lane_width_;
  const auto& far_right_line = bev_lanelines.front();
  const auto& far_left_line = bev_lanelines.back();
  if (right_occ->nodes.empty() || left_occ->nodes.empty()) {
    return {};
  }

  if (1) {
    float right_blank_space =
        std::fabs(GetTwoBoundayDis(right_occ, far_right_line));
    HLOG_DEBUG << "[occ module] right_blank_space:" << right_blank_space;
    if (right_blank_space < -0.3) {
      return {};
    }

    vitual_lane_num = static_cast<int>(
        std::floor(right_blank_space / averge_entrance_lane_width_ + 0.3));

    // !TBD 这里对于生成的车道线需要做个check，需确保没有超出occ边界。
    if (vitual_lane_num > 0) {
      for (int i = 1; i <= vitual_lane_num; ++i) {
        em::Boundary virtual_line;
        for (const auto& node : far_right_line->nodes) {
          em::BoundaryNode::Ptr virtual_node =
              std::make_shared<em::BoundaryNode>();
          virtual_node->point =
              node->point +
              Eigen::Vector3f{0.0, -1 * averge_entrance_lane_width_ * i, 0.0};
          virtual_node->dash_seg = em::DashSegType::UNKNOWN_DASH_SEG;
          virtual_line.nodes.emplace_back(virtual_node);
        }
        virtual_line.linetype = em::LaneType_INTERSECTION_VIRTUAL_MARKING;
        virtual_line.color = em::WHITE;
        virtual_line.lanepos = em::LanePositionType_OTHER;
        virutal_lines.emplace_back(virtual_line);
      }
    }
  }

  if (1) {
    float left_blank_space =
        std::fabs(GetTwoBoundayDis(left_occ, far_left_line));
    HLOG_DEBUG << "[occ module] left_blank_space:" << left_blank_space;
    if (left_blank_space < -0.3) {
      return {};
    }

    vitual_lane_num = static_cast<int>(
        std::floor(left_blank_space / averge_entrance_lane_width_ + 0.3));

    if (vitual_lane_num > 0) {
      for (int i = 1; i <= vitual_lane_num; ++i) {
        em::Boundary virtual_line;
        for (const auto& node : far_left_line->nodes) {
          em::BoundaryNode::Ptr virtual_node =
              std::make_shared<em::BoundaryNode>();
          virtual_node->point =
              node->point +
              Eigen::Vector3f{0.0, averge_entrance_lane_width_ * i, 0.0};
          virtual_node->dash_seg = em::DashSegType::UNKNOWN_DASH_SEG;
          virtual_line.nodes.emplace_back(virtual_node);
        }
        virtual_line.linetype = em::LaneType_INTERSECTION_VIRTUAL_MARKING;
        virtual_line.color = em::WHITE;
        virtual_line.lanepos = em::LanePositionType_OTHER;
        virutal_lines.emplace_back(virtual_line);
      }
    }
  }

  // 车道线之间是否需要虚拟车道线。
  if (bev_lanelines.size() > 1) {
    for (int line_idx = 0;
         line_idx < static_cast<int>(bev_lanelines.size()) - 1; ++line_idx) {
      if (bev_lanelines.at(line_idx)->nodes.empty() ||
          bev_lanelines.at(line_idx + 1)->nodes.empty()) {
        continue;
      }
      float lane_width = std::fabs(GetTwoBoundayDis(
          bev_lanelines.at(line_idx), bev_lanelines.at(line_idx + 1)));
      vitual_lane_num = static_cast<int>(
          std::floor(lane_width / averge_entrance_lane_width_ + 0.3));
      HLOG_DEBUG << "[occ module] line_blank_space:" << lane_width
                 << ", and virtual lane num:" << vitual_lane_num;
      if (vitual_lane_num > 1) {
        for (int i = 1; i <= vitual_lane_num - 1; ++i) {
          em::Boundary virtual_line;
          for (const auto& node : bev_lanelines.at(line_idx)->nodes) {
            em::BoundaryNode::Ptr virtual_node =
                std::make_shared<em::BoundaryNode>();
            virtual_node->point =
                node->point +
                Eigen::Vector3f{0.0, averge_entrance_lane_width_ * i, 0.0};
            virtual_node->dash_seg = em::DashSegType::UNKNOWN_DASH_SEG;
            virtual_line.nodes.emplace_back(virtual_node);
          }
          virtual_line.linetype = em::LaneType_INTERSECTION_VIRTUAL_MARKING;
          virtual_line.color = em::WHITE;
          virtual_line.lanepos = em::LanePositionType_OTHER;
          virutal_lines.emplace_back(virtual_line);
        }
      }
    }
  }

  return virutal_lines;
}
std::vector<em::Boundary> OccGuideLineManager::FineTuneGuideLineV2(
    const std::vector<em::Boundary::Ptr>& bev_lanelines) {
  std::vector<em::Boundary> virutal_lines;
  int vitual_lane_num = -1;

  auto stable_occs = GetStableOcc();
  auto& right_occ = stable_occs.front();
  auto& left_occ = stable_occs.back();

  averge_entrance_lane_width_ = GetEntranceLaneWidth(bev_lanelines);
  // 如果退出车道数计算过，则保持历史车道数。反之，需计算退出车道数。
  if (assume_entrancelane_nums_by_entrancelane_ == -1) {
    assume_entrancelane_nums_by_entrancelane_ = static_cast<int>(std::max(
        static_cast<double>(std::floor((occ_width_ - safe_distance_ * 2) /
                                           averge_entrance_lane_width_ +
                                       0.3)),
        1.0));
  }

  HLOG_DEBUG << "[occ module] fine_lane_width:" << averge_entrance_lane_width_
             << " ,bev_lanelines size():" << bev_lanelines.size();
  const auto& far_right_line = bev_lanelines.front();
  const auto& far_left_line = bev_lanelines.back();
  if (right_occ->nodes.empty() || left_occ->nodes.empty()) {
    return {};
  }
  // 如果大于3.5m，n=width/3.5 -1,则先大胆的虚拟n条3.5m的车道线；
  // 然后，对width-((n)*3.5)=余数,进行判断，如果大于5.5m，则平均为两条，否则直接到occ边缘虚拟1条。
  float default_lane_width = 3.5f;
  if (averge_entrance_lane_width_ <= 0) {
    HLOG_ERROR << "[occ module] averge_entrance_lane_width_ <=0!";
    return {};
  }
  HLOG_DEBUG << "right_occ id:" << right_occ->id
             << " ,left_occ id:" << left_occ->id;
  HLOG_DEBUG << "far_right_line start x:"
             << far_right_line->nodes.front()->point.x()
             << " ,y:" << far_right_line->nodes.front()->point.y();
  HLOG_DEBUG << "far_left_line start x:"
             << far_left_line->nodes.front()->point.x()
             << " ,y:" << far_left_line->nodes.front()->point.y();
  HLOG_DEBUG << "right_occ start x:" << right_occ->nodes.front()->point.x()
             << " ,y:" << right_occ->nodes.front()->point.y();
  HLOG_DEBUG << "left_occ start x:" << left_occ->nodes.front()->point.x()
             << " ,y:" << left_occ->nodes.front()->point.y();
  if (1) {
    float right_blank_space = GetTwoBoundayDis(right_occ, far_right_line);
    if (right_blank_space < -1.0) {
      // 无需补虚拟线。
    } else {
      float right_blank_min_dist =
          GetTwoBoundaryMinDis(right_occ, far_right_line);
      HLOG_DEBUG << "[occ module] right_blank_space:" << right_blank_space
                 << " ,right_blank_min_dist:" << right_blank_min_dist;

      // 最小需要补的3.5m车道数目
      int min_virtual_lane_num =
          static_cast<int>(std::floor(std::fabs(right_blank_min_dist) /
                                      default_lane_width)) > 0
              ? static_cast<int>(std::floor(std::fabs(right_blank_min_dist) /
                                            default_lane_width)) -
                    1
              : 0;

      // 补充完最小车道数后，距离occ边界剩余的宽度
      float remaining_width_right = std::fabs(right_blank_min_dist) -
                                    min_virtual_lane_num * default_lane_width;
      HLOG_DEBUG << " right min_virtual_lane_num:" << min_virtual_lane_num
                 << " ,right remaining_width:" << remaining_width_right;
      // 如果剩余宽度大于5.5m，则在平均补两条线，否则补一条。

      if (remaining_width_right < 5.5) {
        if (std::fabs(right_blank_min_dist) > 2.2) {
          em::Boundary virtual_line;
          for (const auto& node : far_right_line->nodes) {
            Eigen::Vector2d point(node->point.x(), node->point.y());
            em::BoundaryNode::Ptr virtual_node =
                std::make_shared<em::BoundaryNode>();
            virtual_node->point =
                node->point +
                Eigen::Vector3f{0.0, -1 * right_blank_min_dist, 0.0};
            virtual_node->dash_seg = em::DashSegType::UNKNOWN_DASH_SEG;
            virtual_line.nodes.emplace_back(virtual_node);
          }
          virtual_line.linetype = em::LaneType_INTERSECTION_VIRTUAL_MARKING;
          virtual_line.color = em::WHITE;
          virtual_line.lanepos = em::LanePositionType_OTHER;
          virutal_lines.emplace_back(virtual_line);
          HLOG_DEBUG << "virtual line by right occ and right line 000";
        }

      } else {
        for (int i = 1; i <= 2; ++i) {
          em::Boundary virtual_line;
          for (const auto& node : far_right_line->nodes) {
            Eigen::Vector2d point(node->point.x(), node->point.y());
            em::BoundaryNode::Ptr virtual_node =
                std::make_shared<em::BoundaryNode>();
            virtual_node->point =
                node->point +
                Eigen::Vector3f{0.0,
                                -1 * default_lane_width * min_virtual_lane_num -
                                    remaining_width_right * i / 2.0,
                                0.0};
            virtual_node->dash_seg = em::DashSegType::UNKNOWN_DASH_SEG;
            virtual_line.nodes.emplace_back(virtual_node);
          }
          virtual_line.linetype = em::LaneType_INTERSECTION_VIRTUAL_MARKING;
          virtual_line.color = em::WHITE;
          virtual_line.lanepos = em::LanePositionType_OTHER;
          virutal_lines.emplace_back(virtual_line);
          HLOG_DEBUG << "virtual line by right occ and right line 111";
        }
      }

      for (int i = 1; i <= min_virtual_lane_num; ++i) {
        em::Boundary virtual_line;
        for (const auto& node : far_right_line->nodes) {
          Eigen::Vector2d point(node->point.x(), node->point.y());
          em::BoundaryNode::Ptr virtual_node =
              std::make_shared<em::BoundaryNode>();
          virtual_node->point =
              node->point +
              Eigen::Vector3f{0.0, -1 * default_lane_width * i, 0.0};
          virtual_node->dash_seg = em::DashSegType::UNKNOWN_DASH_SEG;
          virtual_line.nodes.emplace_back(virtual_node);
        }
        virtual_line.linetype = em::LaneType_INTERSECTION_VIRTUAL_MARKING;
        virtual_line.color = em::WHITE;
        virtual_line.lanepos = em::LanePositionType_OTHER;
        virutal_lines.emplace_back(virtual_line);
        HLOG_DEBUG << "virtual line by right occ and right line 222";
      }
    }
  }

  if (1) {
    float left_blank_space = GetTwoBoundayDis(left_occ, far_left_line);
    if (left_blank_space > 1.0) {
      // 无需补虚拟线。
    } else {
      float left_blank_min_dist = GetTwoBoundaryMinDis(left_occ, far_left_line);
      HLOG_DEBUG << "[occ module] left_blank_space:" << left_blank_space
                 << " ,left_blank_min_dist:" << left_blank_min_dist;
      // 最小需要补的3.5m车道数目
      int min_virtual_lane_num =
          static_cast<int>(std::floor(std::fabs(left_blank_min_dist) /
                                      default_lane_width)) > 0
              ? static_cast<int>(std::floor(std::fabs(left_blank_min_dist) /
                                            default_lane_width)) -
                    1
              : 0;
      // 如果大于3.5m，n=width/3.5 -1,则先大胆的虚拟n条3.5m的车道线；
      // 然后，对width-((n)*3.5)=余数,进行判断，如果大于5.5m，则平均为两条，否则直接到occ边缘虚拟1条
      float remaining_width = std::fabs(left_blank_min_dist) -
                              min_virtual_lane_num * default_lane_width;
      HLOG_DEBUG << " left min_virtual_lane_num:" << min_virtual_lane_num
                 << " ,left remaining_width:" << remaining_width;
      if (remaining_width < 5.5) {
        if (std::fabs(left_blank_min_dist) > 2.2) {
          em::Boundary virtual_line;
          for (const auto& node : far_left_line->nodes) {
            Eigen::Vector2d point(node->point.x(), node->point.y());
            em::BoundaryNode::Ptr virtual_node =
                std::make_shared<em::BoundaryNode>();
            virtual_node->point =
                node->point +
                Eigen::Vector3f{0.0, -1 * left_blank_min_dist, 0.0};
            virtual_node->dash_seg = em::DashSegType::UNKNOWN_DASH_SEG;
            virtual_line.nodes.emplace_back(virtual_node);
          }
          virtual_line.linetype = em::LaneType_INTERSECTION_VIRTUAL_MARKING;
          virtual_line.color = em::WHITE;
          virtual_line.lanepos = em::LanePositionType_OTHER;
          virutal_lines.emplace_back(virtual_line);
          HLOG_DEBUG << "virtual line by left occ and left line 444 add width:"
                     << virtual_line.nodes.front()->point.x() << ","
                     << virtual_line.nodes.front()->point.y();
        }

      } else {
        for (int i = 1; i <= 2; ++i) {
          em::Boundary virtual_line;
          for (const auto& node : far_left_line->nodes) {
            Eigen::Vector2d point(node->point.x(), node->point.y());
            em::BoundaryNode::Ptr virtual_node =
                std::make_shared<em::BoundaryNode>();
            virtual_node->point =
                node->point +
                Eigen::Vector3f{0.0,
                                default_lane_width * min_virtual_lane_num +
                                    remaining_width * i / 2.0,
                                0.0};
            virtual_node->dash_seg = em::DashSegType::UNKNOWN_DASH_SEG;
            virtual_line.nodes.emplace_back(virtual_node);
          }
          virtual_line.linetype = em::LaneType_INTERSECTION_VIRTUAL_MARKING;
          virtual_line.color = em::WHITE;
          virtual_line.lanepos = em::LanePositionType_OTHER;
          virutal_lines.emplace_back(virtual_line);
          HLOG_DEBUG << "virtual line by left occ and left line 555 add width:"
                     << virtual_line.nodes.front()->point.x() << ","
                     << virtual_line.nodes.front()->point.y();
        }
      }

      for (int i = 1; i <= min_virtual_lane_num; ++i) {
        em::Boundary virtual_line;
        for (const auto& node : far_left_line->nodes) {
          Eigen::Vector2d point(node->point.x(), node->point.y());
          em::BoundaryNode::Ptr virtual_node =
              std::make_shared<em::BoundaryNode>();
          virtual_node->point =
              node->point + Eigen::Vector3f{0.0, default_lane_width * i, 0.0};

          virtual_node->dash_seg = em::DashSegType::UNKNOWN_DASH_SEG;
          virtual_line.nodes.emplace_back(virtual_node);
        }
        virtual_line.linetype = em::LaneType_INTERSECTION_VIRTUAL_MARKING;
        virtual_line.color = em::WHITE;
        virtual_line.lanepos = em::LanePositionType_OTHER;
        virutal_lines.emplace_back(virtual_line);
        HLOG_DEBUG << "virtual line by left occ and left line 666"
                   << virtual_line.nodes.front()->point.x() << ","
                   << virtual_line.nodes.front()->point.y();
      }
    }
  }

  // 车道线之间是否需要虚拟车道线。
  if (bev_lanelines.size() > 1) {
    for (int line_idx = 0;
         line_idx < static_cast<int>(bev_lanelines.size()) - 1; ++line_idx) {
      if (bev_lanelines.at(line_idx)->nodes.empty() ||
          bev_lanelines.at(line_idx + 1)->nodes.empty()) {
        continue;
      }
      float lane_width = GetTwoBoundayDis(bev_lanelines.at(line_idx),
                                          bev_lanelines.at(line_idx + 1));
      vitual_lane_num = static_cast<int>(std::floor(
          std::fabs(lane_width) / averge_entrance_lane_width_ + 0.3));
      HLOG_DEBUG << "[occ module] line_blank_space:" << lane_width
                 << ", and virtual lane num:" << vitual_lane_num;
      if (vitual_lane_num > 1) {
        for (int i = 1; i <= vitual_lane_num - 1; ++i) {
          em::Boundary virtual_line;
          for (const auto& node : bev_lanelines.at(line_idx)->nodes) {
            em::BoundaryNode::Ptr virtual_node =
                std::make_shared<em::BoundaryNode>();
            virtual_node->point =
                node->point +
                Eigen::Vector3f{0.0, averge_entrance_lane_width_ * i, 0.0};
            virtual_node->dash_seg = em::DashSegType::UNKNOWN_DASH_SEG;
            virtual_line.nodes.emplace_back(virtual_node);
          }
          virtual_line.linetype = em::LaneType_INTERSECTION_VIRTUAL_MARKING;
          virtual_line.color = em::WHITE;
          virtual_line.lanepos = em::LanePositionType_OTHER;
          virutal_lines.emplace_back(virtual_line);
          HLOG_DEBUG << "virtual line by left line and right line 333";
        }
      }
    }
  }

  return virutal_lines;
}  // namespace mf
std::vector<std::pair<em::Id, em::OccRoad::Ptr>>
OccGuideLineManager::GetFrontOccRoadPair() {
  // 获取路口前的occ路沿线
  // 若当前帧没有occ目标数据，直接返回空。
  if (input_ele_map_->occ_roads.empty()) {
    HLOG_WARN << "occ roadedge source data num is NULL...";
    return {};
  }

  std::vector<std::pair<em::Id, em::OccRoad::Ptr>> front_occ_set;
  for (const auto& roadedge : input_ele_map_->occ_roads) {
    if (!roadedge.second->is_forward ||
        roadedge.second->group_id != static_cast<em::Id>(1)) {
      continue;
    }

    const auto& track_id = roadedge.first;
    const auto& occ_road = roadedge.second;
    if (occ_road->road_points.empty()) {
      continue;
    }
    HLOG_DEBUG << "[occ module] track_id" << occ_road->track_id
               << ", grop_id:" << 1 << ","
               << "start_x:" << occ_road->road_points.front().x() << ","
               << "start_y:" << occ_road->road_points.front().y() << ","
               << "end_x:" << occ_road->road_points.back().x() << ","
               << "end_y:" << occ_road->road_points.back().y() << ","
               << "left_occ_id:" << occ_road->left_occ_id
               << "right_occ_id:" << occ_road->right_occ_id;
    front_occ_set.emplace_back(track_id, occ_road);
  }

  // 从右到左进行排序
  std::sort(front_occ_set.begin(), front_occ_set.end(),
            [](const auto& a, const auto& b) {
              return a.second->road_points.front().y() <
                     b.second->road_points.front().y();
            });

  // 筛选可以构成pair的occ集合

  std::vector<em::Id> keep_occs;
  std::vector<std::pair<em::Id, em::OccRoad::Ptr>> front_occ_pair;
  for (auto iter = front_occ_set.begin(); iter != front_occ_set.end(); ++iter) {
    const auto& track_id = iter->first;
    const auto& occ_road = iter->second;
    if (occ_road->left_occ_id != static_cast<em::Id>(-1)) {
      if (std::find(keep_occs.begin(), keep_occs.end(), track_id) ==
          keep_occs.end()) {
        keep_occs.emplace_back(track_id);
        front_occ_pair.emplace_back(track_id, occ_road);
        HLOG_DEBUG << "[occ module] emplace back track_id"
                   << occ_road->track_id;
      }
    }
    if (iter == std::prev(front_occ_set.end()) &&
        occ_road->right_occ_id != static_cast<em::Id>(-1)) {
      if (std::find(keep_occs.begin(), keep_occs.end(), track_id) ==
          keep_occs.end()) {
        keep_occs.emplace_back(track_id);
        front_occ_pair.emplace_back(track_id, occ_road);
        HLOG_DEBUG << "[occ module] emplace left est track_id"
                   << occ_road->track_id;
      }
    }
  }

  return front_occ_pair;
}

std::vector<std::pair<em::Id, em::OccRoad::Ptr>>
OccGuideLineManager::GetBestOccPair(
    const std::vector<std::pair<em::Id, em::OccRoad::Ptr>>& front_occ_pair) {
  // 寻找与自车角度最小的豁口作为目标豁口
  if (front_occ_pair.empty()) {
    return {};
  }

  float min_occ_angle = FLT_MAX;
  int min_occ_idx = -1;
  for (int i = 0; i < static_cast<int>(front_occ_pair.size()) - 1; ++i) {
    // 如果两个豁口宽度比较小，则直接过滤掉
    if (GetOccWidth(front_occ_pair.at(i).second,
                    front_occ_pair.at(i + 1).second) < 6.5) {
      continue;
    }
    const auto occ_center_point =
        (front_occ_pair.at(i).second->road_points.front() +
         front_occ_pair.at(i + 1).second->road_points.front()) /
        2.0;

    float cur_angle =
        std::fabs(std::atan2(occ_center_point.y(), occ_center_point.x()));
    if (cur_angle < min_occ_angle) {
      min_occ_angle = cur_angle;
      min_occ_idx = i;
    }
  }

  if (min_occ_idx < 0 ||
      min_occ_idx >= static_cast<int>(front_occ_pair.size()) - 1) {
    return {};
  }

  HLOG_DEBUG << "[occ module] best passible region:" << "idx:" << min_occ_idx
             << "," << "angle:" << min_occ_angle;

  std::vector<std::pair<em::Id, em::OccRoad::Ptr>> best_occ_pair;
  const auto& right_occ = front_occ_pair.at(min_occ_idx);
  const auto& left_occ = front_occ_pair.at(min_occ_idx + 1);
  best_occ_pair.emplace_back(right_occ);
  best_occ_pair.emplace_back(left_occ);

  return best_occ_pair;
}

double OccGuideLineManager::GetOccWidth(const em::OccRoad::Ptr& right_occ,
                                        const em::OccRoad::Ptr& left_occ) {
  em::OccRoad::Ptr query_line = nullptr;
  em::OccRoad::Ptr value_line = nullptr;
  if (right_occ->road_points.front().x() < left_occ->road_points.front().x()) {
    query_line = left_occ;
    value_line = right_occ;
  } else {
    query_line = right_occ;
    value_line = left_occ;
  }

  Eigen::Vector3d dir_vector;
  int query_line_index = -1;
  int value_line_index = -1;
  const int Window_Point_NUMS = 3;
  for (int i = 0; i < static_cast<int>(query_line->road_points.size()) - 2;
       ++i) {
    std::vector<Eigen::Vector3d> windows_dir_vector;
    std::vector<int> window_value_indexs;
    for (int window_start = i; window_start < i + Window_Point_NUMS;
         ++window_start) {
      const auto query_point = query_line->road_points.at(i);
      auto it = std::min_element(
          value_line->road_points.begin(), value_line->road_points.end(),
          [&point = query_point](const Eigen::Vector3d& a,
                                 const Eigen::Vector3d& b) {
            return (point - a).norm() < (point - b).norm();
          });

      window_value_indexs.emplace_back(
          std::distance(value_line->road_points.begin(), it));
      HLOG_DEBUG << "[occ module] dir vector start point:" << query_point.x()
                 << "," << query_point.y();
      HLOG_DEBUG << "[occ module] dir vector end point:" << (*it).x() << ","
                 << (*it).y();

      if (value_line->road_points.size() == 1) {
        dir_vector = (*it) - query_point;
      } else if (it == std::prev(value_line->road_points.end())) {
        Eigen::Vector3d AB = (*it) - (*std::prev(it));
        Eigen::Vector3d AP = query_point - (*std::prev(it));
        AB.normalize();
        Eigen::Vector3d crossProduct = AB.cross(AP);
        dir_vector = crossProduct;
      } else {
        Eigen::Vector3d AB = (*it) - (*std::next(it));
        Eigen::Vector3d AP = query_point - (*std::next(it));
        AB.normalize();
        Eigen::Vector3d crossProduct = AB.cross(AP);
        dir_vector = crossProduct;
      }
      windows_dir_vector.emplace_back(dir_vector);
    }

    if (windows_dir_vector[0].norm() > windows_dir_vector[1].norm() &&
        windows_dir_vector[1].norm() > windows_dir_vector[2].norm()) {
      continue;
    } else {
      dir_vector = windows_dir_vector[2];
      query_line_index = i;
      query_line->valid_index = query_line_index;
      value_line_index = window_value_indexs[2];
      value_line->valid_index = value_line_index;
      return dir_vector.norm();
    }
  }

  const auto query_point = query_line->road_points.front();
  auto it = std::min_element(value_line->road_points.begin(),
                             value_line->road_points.end(),
                             [&point = query_point](const Eigen::Vector3d& a,
                                                    const Eigen::Vector3d& b) {
                               return (point - a).norm() < (point - b).norm();
                             });
  value_line_index = std::distance(value_line->road_points.begin(), it);
  if (value_line->road_points.size() == 1) {
    dir_vector = (*it) - query_point;
  } else if (it == std::prev(value_line->road_points.end())) {
    Eigen::Vector3d AB = (*it) - (*std::prev(it));
    Eigen::Vector3d AP = query_point - (*std::prev(it));
    AB.normalize();
    Eigen::Vector3d crossProduct = AB.cross(AP);
    dir_vector = crossProduct;
  } else {
    Eigen::Vector3d AB = (*it) - (*std::next(it));
    Eigen::Vector3d AP = query_point - (*std::next(it));
    AB.normalize();
    Eigen::Vector3d crossProduct = AB.cross(AP);
    dir_vector = crossProduct;
  }
  query_line->valid_index = 0;
  value_line->valid_index = value_line_index;

  return dir_vector.norm();
}

Eigen::Vector3f OccGuideLineManager::CalcuDirectionVecV2(
    em::Boundary::Ptr right_occ, em::Boundary::Ptr left_occ) {
  if (right_occ->nodes.empty() || left_occ->nodes.empty()) {
    return {};
  }

  Eigen::Vector3f dir_vector;

  em::Boundary::Ptr query_line = nullptr;
  em::Boundary::Ptr value_line = nullptr;
  if (right_occ->nodes.front()->point.x() <
      left_occ->nodes.front()->point.x()) {
    query_line = left_occ;
    value_line = right_occ;
  } else {
    query_line = right_occ;
    value_line = left_occ;
  }

  const int Window_Point_NUMS = 3;
  if (query_line->nodes.size() < Window_Point_NUMS) {
    const auto query_point = query_line->nodes.front()->point;
    auto it = std::min_element(
        value_line->nodes.begin(), value_line->nodes.end(),
        [&point = query_point](const em::BoundaryNode::Ptr& a,
                               const em::BoundaryNode::Ptr& b) {
          return (point - a->point).norm() < (point - b->point).norm();
        });
    HLOG_DEBUG << "[occ module] dir vector start point:" << query_point.x()
               << "," << query_point.y();
    HLOG_DEBUG << "[occ module] dir vector end point:" << (*it)->point.x()
               << "," << (*it)->point.y();

    if (value_line->nodes.size() == 1) {
      dir_vector = (*it)->point - query_point;
    } else if (it == std::prev(value_line->nodes.end())) {
      Eigen::Vector3f AB = (*it)->point - (*std::prev(it))->point;
      Eigen::Vector3f AP = query_point - (*std::prev(it))->point;
      AB.normalize();
      Eigen::Vector3f crossProduct = AB.cross(AP);
      dir_vector = crossProduct;
    } else {
      Eigen::Vector3f AB = (*it)->point - (*std::next(it))->point;
      Eigen::Vector3f AP = query_point - (*std::next(it))->point;
      AB.normalize();
      Eigen::Vector3f crossProduct = AB.cross(AP);
      dir_vector = crossProduct;
    }
    return dir_vector;
  }

  int query_line_index = -1;
  int value_line_index = -1;

  for (int i = 0; i < static_cast<int>(query_line->nodes.size()) - 2; ++i) {
    std::vector<Eigen::Vector3f> windows_dir_vector;
    std::vector<int> window_value_indexs;
    for (int window_start = i; window_start < i + Window_Point_NUMS;
         ++window_start) {
      const auto query_point = query_line->nodes.at(i)->point;
      auto it = std::min_element(
          value_line->nodes.begin(), value_line->nodes.end(),
          [&point = query_point](const em::BoundaryNode::Ptr& a,
                                 const em::BoundaryNode::Ptr& b) {
            return (point - a->point).norm() < (point - b->point).norm();
          });

      window_value_indexs.emplace_back(
          std::distance(value_line->nodes.begin(), it));
      HLOG_DEBUG << "[occ module] dir vector start point:" << query_point.x()
                 << "," << query_point.y();
      HLOG_DEBUG << "[occ module] dir vector end point:" << (*it)->point.x()
                 << "," << (*it)->point.y();

      if (value_line->nodes.size() == 1) {
        dir_vector = (*it)->point - query_point;
      } else if (it == std::prev(value_line->nodes.end())) {
        Eigen::Vector3f AB = (*it)->point - (*std::prev(it))->point;
        Eigen::Vector3f AP = query_point - (*std::prev(it))->point;
        AB.normalize();
        Eigen::Vector3f crossProduct = AB.cross(AP);
        dir_vector = crossProduct;
      } else {
        Eigen::Vector3f AB = (*it)->point - (*std::next(it))->point;
        Eigen::Vector3f AP = query_point - (*std::next(it))->point;
        AB.normalize();
        Eigen::Vector3f crossProduct = AB.cross(AP);
        dir_vector = crossProduct;
      }
      windows_dir_vector.emplace_back(dir_vector);
    }

    if (windows_dir_vector[0].norm() > windows_dir_vector[1].norm() &&
        windows_dir_vector[1].norm() > windows_dir_vector[2].norm()) {
      continue;
    } else {
      query_line_index = i;
      dir_vector = windows_dir_vector[2];
      value_line_index = window_value_indexs[2];
      break;
    }
  }

  if (query_line_index == -1 || value_line_index == -1) {
    const auto query_point = query_line->nodes.front()->point;
    auto it = std::min_element(
        value_line->nodes.begin(), value_line->nodes.end(),
        [&point = query_point](const em::BoundaryNode::Ptr& a,
                               const em::BoundaryNode::Ptr& b) {
          return (point - a->point).norm() < (point - b->point).norm();
        });
    HLOG_DEBUG << "[occ module] dir vector start point:" << query_point.x()
               << "," << query_point.y();
    HLOG_DEBUG << "[occ module] dir vector end point:" << (*it)->point.x()
               << "," << (*it)->point.y();

    if (value_line->nodes.size() == 1) {
      dir_vector = (*it)->point - query_point;
    } else if (it == std::prev(value_line->nodes.end())) {
      Eigen::Vector3f AB = (*it)->point - (*std::prev(it))->point;
      Eigen::Vector3f AP = query_point - (*std::prev(it))->point;
      AB.normalize();
      Eigen::Vector3f crossProduct = AB.cross(AP);
      dir_vector = crossProduct;
    } else {
      Eigen::Vector3f AB = (*it)->point - (*std::next(it))->point;
      Eigen::Vector3f AP = query_point - (*std::next(it))->point;
      AB.normalize();
      Eigen::Vector3f crossProduct = AB.cross(AP);
      dir_vector = crossProduct;
    }
    return dir_vector;
  }

  // 使用迭代器遍历, 删除前端非有效豁口区域
  auto it = query_line->nodes.begin();
  for (size_t index = 0; it != query_line->nodes.end(); ++index) {
    if (index < query_line_index) {
      it = query_line->nodes.erase(it);
    } else {
      ++it;
    }
  }

  // 使用迭代器遍历, 删除前端非有效豁口区域
  it = value_line->nodes.begin();
  for (size_t index = 0; it != value_line->nodes.end(); ++index) {
    if (index < value_line_index) {
      it = value_line->nodes.erase(it);
    } else {
      ++it;
    }
  }

  return dir_vector;
}

Eigen::Vector3f OccGuideLineManager::CalcuDirectionVec(
    const em::Boundary::Ptr& right_occ, const em::Boundary::Ptr& left_occ) {
  if (right_occ->nodes.empty() || left_occ->nodes.empty()) {
    return {};
  }

  em::Boundary::Ptr query_line = nullptr;
  em::Boundary::Ptr value_line = nullptr;
  if (right_occ->nodes.front()->point.x() <
      left_occ->nodes.front()->point.x()) {
    query_line = left_occ;
    value_line = right_occ;
  } else {
    query_line = right_occ;
    value_line = left_occ;
  }

  const auto query_point = query_line->nodes.front()->point;
  auto it = std::min_element(
      value_line->nodes.begin(), value_line->nodes.end(),
      [&point = query_point](const em::BoundaryNode::Ptr& a,
                             const em::BoundaryNode::Ptr& b) {
        return (point - a->point).norm() < (point - b->point).norm();
      });
  HLOG_DEBUG << "[occ module] dir vector start point:" << query_point.x() << ","
             << query_point.y();
  HLOG_DEBUG << "[occ module] dir vector end point:" << (*it)->point.x() << ","
             << (*it)->point.y();
  Eigen::Vector3f dir_vector;
  if (value_line->nodes.size() == 1) {
    dir_vector = (*it)->point - query_point;
  } else if (it == std::prev(value_line->nodes.end())) {
    Eigen::Vector3f AB = (*it)->point - (*std::prev(it))->point;
    Eigen::Vector3f AP = query_point - (*std::prev(it))->point;
    AB.normalize();
    Eigen::Vector3f crossProduct = AB.cross(AP);
    dir_vector = crossProduct;
  } else {
    Eigen::Vector3f AB = (*it)->point - (*std::next(it))->point;
    Eigen::Vector3f AP = query_point - (*std::next(it))->point;
    AB.normalize();
    Eigen::Vector3f crossProduct = AB.cross(AP);
    dir_vector = crossProduct;
  }

  return dir_vector;
}

bool OccGuideLineManager::CanOccPairOtherOcc(const em::OccRoad::Ptr& occ_i,
                                             const em::OccRoad::Ptr& occ_j) {
  if (occ_i->road_points.empty() || occ_j->road_points.empty()) {
    return false;
  }
  const auto& occ_i_last_point = occ_i->road_points.back();
  const auto& occ_j_last_point = occ_j->road_points.back();
  const auto& occ_i_first_point = occ_i->road_points.front();
  const auto& occ_j_first_point = occ_j->road_points.front();
  const auto& occ_i_last_point_x = occ_i_last_point.x();
  const auto& occ_j_last_point_x = occ_j_last_point.x();
  const auto& occ_i_first_point_x = occ_i_first_point.x();
  const auto& occ_j_first_point_x = occ_j_first_point.x();
  if (occ_i_last_point_x < 0.0 && occ_j_last_point_x < 0.0) {
    // 对于出现在ego_car后方的两条线，不进行配对。
    HLOG_DEBUG << "occ_i_last_point_x < 0.0 and occ_j_last_point_x < 0.0";
    return false;
  }
  if (occ_i_last_point_x < 0.0 || occ_j_last_point_x < 0.0) {
    // 对于其中一条occ线位于车子后方的情况， 不进行配对(暂时也不支持处理)。
    HLOG_DEBUG << "occ_i_last_point_x < 0.0 or occ_j_last_point_x < 0.0";
    return false;
  }

  // 两条线贯穿自车位置的情况，考虑专门设置一个模块在自车位置附近去补线。
  if (occ_i_first_point_x < 0.0 && occ_j_first_point_x < 0.0) {
    HLOG_DEBUG << "occ_i_last_point_x < 0.0 and occ_j_first_point_x < 0.0";
    return false;
  }

  float overlay_min_max = std::max(occ_i_first_point_x, occ_j_first_point_x);
  float overlay_max_min = std::min(occ_i_last_point_x, occ_j_last_point_x);

  if (overlay_max_min - overlay_min_max < 10.0) {
    HLOG_DEBUG << "occ overlay < 10.0";
    return false;
  }

  if (overlay_min_max < ego_line_x_dis_) {
    return false;
  }

  // 从两条线上分别最接近x=overlay_min_max的点。
  Eigen::Vector3d occ_i_nearest_point;
  Eigen::Vector3d occ_j_nearest_point;
  for (const auto& point : occ_i->road_points) {
    if (point.x() < overlay_min_max) {
      continue;
    } else {
      occ_i_nearest_point = point;
      break;
    }
  }

  for (const auto& point : occ_j->road_points) {
    if (point.x() < overlay_min_max) {
      continue;
    } else {
      occ_j_nearest_point = point;
      break;
    }
  }

  Eigen::Vector3d nearest_point =
      0.5 * (occ_i_nearest_point + occ_j_nearest_point);

  auto theta = std::atan2(nearest_point.y(), nearest_point.x());
  // 小于45度的豁口pair，不进行配对。
  // TODO(张文海) 后续优化为以路口判定初始时刻下的自车位置点进行角度判断。
  if (std::abs(theta) < 1.0) {
    // return false;
  }

  // 如果豁口宽度比较小，不进行配对。
  if (GetOccWidth(occ_i, occ_j) < 6.5) {
    HLOG_DEBUG << "occ width < 6.5";
    return false;
  }

  return true;
}

std::vector<std::vector<bool>> OccGuideLineManager::ConstructPairTable() {
  const int& occ_num = static_cast<int>(input_occ_set_.size());
  if (occ_num == 0) {
    return {};
  }
  std::vector<std::vector<bool>> pair_table(occ_num,
                                            std::vector<bool>(occ_num, false));
  for (int i = 0; i < occ_num - 1; ++i) {
    for (int j = i + 1; j < occ_num; ++j) {
      const auto& occ_i = input_occ_set_.at(i);
      const auto& occ_j = input_occ_set_.at(j);
      if (CanOccPairOtherOcc(occ_i, occ_j)) {
        pair_table[i][j] = true;
      } else {
        pair_table[i][j] = false;
      }
    }
  }

  return pair_table;
}

void OccGuideLineManager::CloseOccPairStatus(
    std::vector<std::vector<bool>>* pair_table, int i) {
  // 将i行置为false
  std::fill((*pair_table)[i].begin(), (*pair_table)[i].end(), false);

  // 将i列置为false
  for (auto& row : (*pair_table)) {
    row[i] = false;
  }
}

void OccGuideLineManager::FilterBadOccByVertical(
    std::vector<std::vector<bool>>* pair_table) {
  std::vector<std::vector<double>> pair_dis_table(
      (*pair_table).size(), std::vector<double>((*pair_table)[0].size(), NAN));

  // 构建occ pair之间的距离表
  for (int i = 0; i < static_cast<int>((*pair_table).size()) - 1; ++i) {
    for (int j = i + 1; j < static_cast<int>((*pair_table)[i].size()); ++j) {
      if ((*pair_table)[i][j]) {
        const auto& occ_i = input_occ_set_.at(i);
        const auto& occ_j = input_occ_set_.at(j);

        double occ_dis = GetTwoBoundayDis(occ_i, occ_j);
        HLOG_DEBUG << "[occ debug] " << "track id:" << occ_i->track_id
                   << " and track id:" << occ_j->track_id
                   << " distance:" << occ_dis;
        pair_dis_table[i][j] = occ_dis;
        pair_dis_table[j][i] = -occ_dis;
      }
    }
  }

  // 横向上occ位置靠近的occ线集， 只保留离车最近的occ线。
  for (int i = 0; i < static_cast<int>(pair_table->size()) - 1; ++i) {
    std::vector<int> pair_indexs;
    for (int j = 0; j < static_cast<int>((*pair_table)[i].size()); ++j) {
      if (j == i) {
        continue;
      }

      if ((i > j) && (*pair_table)[j][i]) {
        pair_indexs.emplace_back(j);
      }

      if ((i < j) && (*pair_table)[i][j]) {
        pair_indexs.emplace_back(j);
      }
    }

    if (pair_indexs.size() < 2) {
      continue;
    }
    for (int index = 0; index < static_cast<int>(pair_indexs.size()) - 1;
         ++index) {
      const auto& occ_i = input_occ_set_.at(pair_indexs[index]);
      const auto& occ_j = input_occ_set_.at(pair_indexs[index + 1]);
      // HLOG_DEBUG< "[occ debug] " << "track_id:" << occ_i->track_id << "," <<
      // "wi"
      if (std::fabs(pair_dis_table[i][pair_indexs[index + 1]] -
                    pair_dis_table[i][pair_indexs[index]]) < 2.0) {
        const auto& occ_i_last_point = occ_i->road_points.back();
        const auto& occ_j_last_point = occ_j->road_points.back();
        const auto& occ_i_first_point = occ_i->road_points.front();
        const auto& occ_j_first_point = occ_j->road_points.front();
        const auto& occ_i_last_point_x = occ_i_last_point.x();
        const auto& occ_j_last_point_x = occ_j_last_point.x();
        const auto& occ_i_first_point_x = occ_i_first_point.x();
        const auto& occ_j_first_point_x = occ_j_first_point.x();
        if (occ_j_last_point_x < 0.0) {
          // 此时需要过滤掉j线。
          CloseOccPairStatus(pair_table, pair_indexs[index + 1]);
        } else if (occ_i_last_point_x < 0.0) {
          // 此时需要过滤掉i线。
          CloseOccPairStatus(pair_table, pair_indexs[index]);
        } else if (occ_j_first_point_x > 0.0 &&
                   occ_i_first_point_x > occ_j_first_point_x) {
          // 此时需要过滤掉i线。
          CloseOccPairStatus(pair_table, pair_indexs[index]);
        } else if (occ_i_first_point_x > 0.0 &&
                   occ_j_first_point_x > occ_i_first_point_x) {
          // 此时需要过滤掉j线。
          CloseOccPairStatus(pair_table, pair_indexs[index + 1]);
        } else if (occ_i_first_point_x < 0.0 && occ_j_first_point_x > 0.0) {
          // 此时需要过滤掉i线。
          CloseOccPairStatus(pair_table, pair_indexs[index]);
        } else if (occ_i_first_point_x > 0.0 && occ_j_first_point_x < 0.0) {
          // 此时需要过滤掉j线。
          CloseOccPairStatus(pair_table, pair_indexs[index + 1]);
        } else if (occ_i_first_point_x < 0.0 && occ_j_first_point_x < 0.0 &&
                   occ_i_last_point_x < occ_j_last_point_x) {
          // 此时需要过滤掉i线。
          CloseOccPairStatus(pair_table, pair_indexs[index]);
        } else if (occ_i_first_point_x < 0.0 && occ_j_first_point_x < 0.0 &&
                   occ_i_last_point_x > occ_j_last_point_x) {
          // 此时需要过滤掉i线。
          CloseOccPairStatus(pair_table, pair_indexs[index + 1]);
        } else {
          HLOG_WARN << "It's unnormal scene...";
        }
      }
    }
  }
}

void OccGuideLineManager::FilterBadOccByHorizon(
    std::vector<std::vector<bool>>* pair_table) {
  // 对于循环配对的情况，进行解循环处理。
  /*

  |    |     |
  |    |     |
  |    |     |
  |    |     |
  |    |     |

  1    3     4

  1 <--> 3, 3 <--> 4, 1 <--> 4
  解循环配对，1 <--> 3, 3 <--> 4
  */

  // 横向上occ位置靠近的occ线集， 只保留离车最近的occ线。
  for (int i = 0; i < static_cast<int>((*pair_table).size()); ++i) {
    std::vector<int> pair_indexs;
    for (int j = i + 1; j < static_cast<int>((*pair_table)[0].size()); ++j) {
      if ((*pair_table)[i][j]) {
        pair_indexs.emplace_back(j);
      }
    }

    if (pair_indexs.size() < 2) {
      continue;
    }
    for (int index = 0; index < static_cast<int>(pair_indexs.size()) - 1;
         ++index) {
      if ((*pair_table)[pair_indexs[index]][pair_indexs[index + 1]]) {
        (*pair_table)[i][pair_indexs[index + 1]] = false;
      }
    }
  }
}

std::vector<std::pair<em::Id, em::OccRoad::Ptr>>
OccGuideLineManager::SelectBestOccPair(
    const std::vector<std::vector<bool>>* pair_table) {
  // 如果退出车道能够找到，则使用退出车道中心点来计算与occ pair之间的角度。
  float average_exitlane_heading = 0.0;
  Eigen::Vector3d exitlane_cp{0.0, 0.0, 0.0};
  int calcu_times = 0;
  if (exit_lane_info_.exist &&
      exit_lane_info_.right_boundary_points.size() >= 5 &&
      exit_lane_info_.left_boundary_points.size() >= 5) {
    const auto& right_ep = exit_lane_info_.right_boundary_points.back();
    const auto& right_sp = exit_lane_info_.right_boundary_points.at(
        exit_lane_info_.right_boundary_points.size() - 5);
    auto right_vehicle_ep = curr_pose_.TransLocalToVehicle(right_ep);
    auto right_vehicle_sp = curr_pose_.TransLocalToVehicle(right_sp);
    auto right_delta_vec = right_vehicle_ep - right_vehicle_sp;
    const auto& right_heading =
        std::atan2(right_delta_vec.y(), right_delta_vec.x());
    average_exitlane_heading += right_heading;
    calcu_times++;

    const auto& left_ep = exit_lane_info_.left_boundary_points.back();
    const auto& left_sp = exit_lane_info_.left_boundary_points.at(
        exit_lane_info_.left_boundary_points.size() - 5);
    auto left_vehicle_ep = curr_pose_.TransLocalToVehicle(left_ep);
    auto left_vehicle_sp = curr_pose_.TransLocalToVehicle(left_sp);
    auto left_delta_vec = left_vehicle_ep - left_vehicle_sp;
    const auto& left_heading =
        std::atan2(left_delta_vec.y(), left_delta_vec.x());
    average_exitlane_heading += left_heading;
    calcu_times++;
  }

  if (calcu_times != 0 && exit_lane_info_.right_boundary_points.size() >= 5 &&
      exit_lane_info_.left_boundary_points.size() >= 5) {
    average_exitlane_heading =
        static_cast<float>(average_exitlane_heading / 1.0 * calcu_times);
    const auto& right_ep = exit_lane_info_.right_boundary_points.back();
    const auto& left_ep = exit_lane_info_.left_boundary_points.back();
    const auto& center_cp = 0.5 * (right_ep + left_ep);
    exitlane_cp = {center_cp.x(), center_cp.y(), center_cp.z()};
  }

  float min_occ_angle = FLT_MAX;
  float first_passbile_dis = FLT_MAX;
  em::OccRoad::Ptr right_occ = nullptr;
  em::OccRoad::Ptr left_occ = nullptr;

  em::OccRoad::Ptr passble_right_occ = nullptr;
  em::OccRoad::Ptr passble_left_occ = nullptr;

  for (int i = 0; i < static_cast<int>((*pair_table).size()) - 1; ++i) {
    for (int j = i + 1; j < static_cast<int>((*pair_table)[i].size()); ++j) {
      if ((*pair_table)[i][j]) {
        const auto& occ_i = input_occ_set_.at(i);
        const auto& occ_j = input_occ_set_.at(j);
        const auto occ_center_point =
            (occ_i->road_points.front() + occ_j->road_points.front()) / 2.0;
        // float cur_angle = std::fabs(std::atan2(occ_center_point.y(),
        // occ_center_point.x()));
        const auto connect_vector = occ_center_point - exitlane_cp;
        float cur_angle = std::atan2(connect_vector.y(), connect_vector.x());
        cur_angle = std::fabs(cur_angle - average_exitlane_heading);
        HLOG_DEBUG << "[occ debug] "
                   << ", occ pair: right_occ track id:" << occ_i->track_id
                   << "," << "left_occ track id:" << occ_j->track_id
                   << ", angle:" << cur_angle;

        HLOG_DEBUG << "[occ debug] "
                   << ", occ pair: right_occ track id:" << occ_i->track_id
                   << "," << " y:" << occ_i->road_points.front().y() << ","
                   << "left_occ track id:" << occ_j->track_id
                   << ", angle:" << " y:" << occ_j->road_points.front().y();
        if (cur_angle < min_occ_angle) {
          min_occ_angle = cur_angle;
          right_occ = occ_i;
          left_occ = occ_j;
        }

        if ((occ_i->road_points.front().y() < 0 &&
             occ_j->road_points.front().y() > 0)) {
          if (occ_center_point.x() < first_passbile_dis) {
            first_passbile_dis = static_cast<float>(occ_center_point.x());
            passble_right_occ = occ_i;
            passble_left_occ = occ_j;
          }
        }
      }
    }
  }

  if (right_occ == nullptr || left_occ == nullptr) {
    return {};
  }

  HLOG_DEBUG << "[occ debug] " << ", min angle occ pair: right_occ track id:"
             << right_occ->track_id << ","
             << "left_occ track id:" << left_occ->track_id;

  if (passble_right_occ == nullptr || passble_left_occ == nullptr) {
    HLOG_DEBUG << "[occ debug] " << ", first passbile occ pair: right_occ null:"
               << "left_occ null:";
  } else {
    HLOG_DEBUG << "[occ debug] "
               << ", first passbile occ pair: right_occ track id:"
               << passble_right_occ->track_id << ","
               << "left_occ track id:" << passble_left_occ->track_id;
  }

  std::vector<std::pair<em::Id, em::OccRoad::Ptr>> best_occ_pair;

  if ((passble_right_occ != nullptr && passble_right_occ != nullptr) &&
      (passble_right_occ->track_id != right_occ->track_id ||
       passble_left_occ->track_id != left_occ->track_id)) {
    const auto pasble_occ_center_point =
        (passble_right_occ->road_points.front() +
         passble_left_occ->road_points.front()) /
        2.0;
    const auto min_angle_occ_center_point =
        (right_occ->road_points.front() + left_occ->road_points.front()) / 2.0;
    if (pasble_occ_center_point.x() < min_angle_occ_center_point.x() - 8.0) {
      best_occ_pair.emplace_back(passble_right_occ->track_id,
                                 passble_right_occ);
      best_occ_pair.emplace_back(passble_left_occ->track_id, passble_left_occ);
    } else {
      best_occ_pair.emplace_back(right_occ->track_id, right_occ);
      best_occ_pair.emplace_back(left_occ->track_id, left_occ);
    }
  } else {
    best_occ_pair.emplace_back(right_occ->track_id, right_occ);
    best_occ_pair.emplace_back(left_occ->track_id, left_occ);
  }

  return best_occ_pair;
}

void OccGuideLineManager::DebugPrint(
    const std::string tag, const std::vector<std::vector<bool>>& pair_table,
    bool show = false) {
  for (int i = 0; i < static_cast<int>((pair_table).size()); ++i) {
    for (int j = i + 1; j < static_cast<int>((pair_table)[i].size()); ++j) {
      if ((pair_table)[i][j]) {
        if (show) {
          input_occ_set_[i]->is_forward = true;
          input_occ_set_[j]->is_forward = true;
        }
        HLOG_DEBUG << "[occ_debug] " << tag
                   << ", occ track id:" << input_occ_set_[i]->track_id
                   << ", and "
                   << ", occ track id:" << input_occ_set_[j]->track_id
                   << " can pair...";
      }
    }
  }
}

std::vector<std::pair<em::Id, em::OccRoad::Ptr>>
OccGuideLineManager::GetBestOccPair(
    std::vector<std::vector<bool>>* pair_table) {
  /* 选择最佳豁口的逻辑实现*/
  if (pair_table->empty()) {
    return {};
  }

  // 根据横向距离一致的，在纵向位置上策略进行配对过滤;
  DebugPrint("before vertical filter", *pair_table, false);
  FilterBadOccByVertical(pair_table);
  DebugPrint("before horizon filter", *pair_table, false);

  // 根据横向位置解除循环配对的occ pair;
  FilterBadOccByHorizon(pair_table);
  DebugPrint("before select best filter", *pair_table, false);

  // 逆向障碍物标记TODO(朱晓林)

  // 选泽最佳occ pair;
  return SelectBestOccPair(pair_table);
}

float OccGuideLineManager::GetEgoLineNearestDistance() {
  std::vector<float> x_dis;
  for (const auto& boundary : bev_laneline_boundarys_) {
    auto& nodes = boundary.second->nodes;
    if (nodes.empty()) {
      continue;
    }
    if (boundary.second->lanepos == em::LanePos::LanePositionType_EGO_LEFT &&
        nodes.front()->point.x() <= 0.0 && nodes.back()->point.x() >= 0.0) {
      x_dis.emplace_back(nodes.back()->point.x());
    }
    if (boundary.second->lanepos == em::LanePos::LanePositionType_EGO_RIGHT &&
        nodes.front()->point.x() <= 0.0 && nodes.back()->point.x() >= 0.0) {
      x_dis.emplace_back(nodes.back()->point.x());
    }
  }

  if (x_dis.size() == 2) {
    return std::min(x_dis[0], x_dis[1]);
  }

  if (x_dis.size() == 1 && x_dis[0] < 35.0) {
    return x_dis[0];
  }

  return 15.0;
}

void OccGuideLineManager::AddCurrentMeasurementOcc() {
  input_occ_set_.clear();
  for (const auto& roadedge : input_ele_map_->occ_roads) {
    const auto& track_id = roadedge.first;
    const auto& occ_road = roadedge.second;
    if (occ_road->road_points.empty()) {
      continue;
    }
    input_occ_set_.emplace_back(occ_road);
  }

  HLOG_DEBUG << "[occ debug], input occ set:" << input_occ_set_.size();
  // 根据线条第一个点的y坐标从右到左进行排序
  std::sort(input_occ_set_.begin(), input_occ_set_.end(),
            [](const auto& a, const auto& b) {
              return a->road_points.front().y() < b->road_points.front().y();
            });

  // 获取主车道最近的车道线纵向长度。
  ego_line_x_dis_ = GetEgoLineNearestDistance();

  // 使用邻接表表示所有occ集合的配对关系。
  std::vector<std::vector<bool>> pair_table = ConstructPairTable();
  DebugPrint("construct pair table", pair_table, false);

  // 从全部配对的occ中挑选一个最佳的occ pair。
  HLOG_DEBUG << "[occ debug] " << " start get best occ pair...";
  std::vector<std::pair<em::Id, em::OccRoad::Ptr>> best_occ_pair =
      GetBestOccPair(&pair_table);

  std::vector<em::Boundary::Ptr> best_occ_pair_local;
  if (best_occ_pair.empty()) {
    history_n_best_occs_.push_back(best_occ_pair_local);
    return;
  }
  HLOG_DEBUG << "[occ debug] " << " best occ pair:" << "occ track id, "
             << best_occ_pair.front().first << " and occ track id, "
             << best_occ_pair.back().first << ", can pair...";

  // 将occ中的点转换到local系下。
  for (int i = 0; i < best_occ_pair.size(); ++i) {
    em::Boundary::Ptr boundary_data = std::make_shared<em::Boundary>();
    auto& occ_points = best_occ_pair.at(i).second->road_points;
    for (auto& point : occ_points) {
      Eigen::Vector3f f_point{point.x(), point.y(), point.z()};
      auto local_pt = curr_pose_.TransformPoint(f_point);
      em::BoundaryNode node;
      node.point = local_pt;
      boundary_data->nodes.emplace_back(
          std::make_shared<em::BoundaryNode>(node));
    }

    boundary_data->occ_valid_start_index =
        best_occ_pair.at(i).second->valid_index;
    boundary_data->id = best_occ_pair.at(i).second->track_id;
    boundary_data->curve_params = best_occ_pair.at(i).second->curve_params;
    best_occ_pair_local.emplace_back(boundary_data);
  }

  history_n_best_occs_.push_back(best_occ_pair_local);
}

bool OccGuideLineManager::CheckOccWhetherStable() {
  HLOG_DEBUG << "[occ debug] " << "history_n_best_occs_size, "
             << history_n_best_occs_.size();
  if (history_n_best_occs_.size() < history_measure_size_) {
    return false;
  }

  double left_x_min = DBL_MAX;
  double left_y_min = DBL_MAX;
  double left_x_max = -DBL_MAX;
  double left_y_max = -DBL_MAX;
  double right_x_min = DBL_MAX;
  double right_y_min = DBL_MAX;
  double right_x_max = -DBL_MAX;
  double right_y_max = -DBL_MAX;

  bool right_occ_sp_before_ego = false;
  bool left_occ_sp_before_ego = false;
  for (int i = 0; i < history_n_best_occs_.size(); ++i) {
    const auto& occ_pair = history_n_best_occs_.at(i);
    if (occ_pair.empty() || occ_pair.front() == nullptr ||
        occ_pair.back() == nullptr || occ_pair.front()->nodes.empty() ||
        occ_pair.back()->nodes.empty()) {
      return false;
    }

    // 如果有效长度比较小， 则认为是不稳定的。
    const auto& right_occ_vaild_start_point =
        occ_pair.front()
            ->nodes.at(occ_pair.front()->occ_valid_start_index)
            ->point;
    const auto& right_occ_vaild_end_point =
        occ_pair.front()->nodes.back()->point;
    const auto& right_occ_valid_length =
        (right_occ_vaild_end_point - right_occ_vaild_start_point).norm();

    // 如果有效长度比较小， 则认为是不稳定的。
    const auto& left_occ_vaild_start_point =
        occ_pair.back()
            ->nodes.at(occ_pair.back()->occ_valid_start_index)
            ->point;
    const auto& left_occ_vaild_end_point = occ_pair.back()->nodes.back()->point;
    const auto& left_occ_valid_length =
        (left_occ_vaild_end_point - left_occ_vaild_start_point).norm();

    if (left_occ_valid_length < virtual_occ_line_length_thresh_ ||
        right_occ_valid_length < virtual_occ_line_length_thresh_) {
      return false;
    }

    // 对于出现在ego_car前方的occ，
    auto& right_occ_local_point = occ_pair.front()->nodes.front()->point;
    auto& left_occ_local_point = occ_pair.back()->nodes.front()->point;
    Eigen::Vector3f right_occ_sp{right_occ_local_point.x(),
                                 right_occ_local_point.y(),
                                 right_occ_local_point.z()};
    auto right_occ_point = curr_pose_.TransLocalToVehicle(right_occ_sp);
    Eigen::Vector3f left_occ_sp{left_occ_local_point.x(),
                                left_occ_local_point.y(),
                                left_occ_local_point.z()};
    auto left_occ_point = curr_pose_.TransLocalToVehicle(left_occ_sp);

    right_occ_sp_before_ego = right_occ_point.x() < 0.0;
    left_occ_sp_before_ego = left_occ_point.x() < 0.0;

    if (right_occ_point.x() < right_x_min) {
      right_x_min = right_occ_point.x();
    }
    if (right_occ_point.x() > right_x_max) {
      right_x_max = right_occ_point.x();
    }

    if (right_occ_point.y() < right_y_min) {
      right_y_min = right_occ_point.y();
    }
    if (right_occ_point.y() > right_y_max) {
      right_y_max = right_occ_point.y();
    }

    if (left_occ_point.x() < left_x_min) {
      left_x_min = left_occ_point.x();
    }
    if (left_occ_point.x() > left_x_max) {
      left_x_max = left_occ_point.x();
    }

    if (left_occ_point.y() < left_y_min) {
      left_y_min = left_occ_point.y();
    }
    if (left_occ_point.y() > left_y_max) {
      left_y_max = left_occ_point.y();
    }
  }

  HLOG_DEBUG << "[occ module]:" << "right_x_max:" << right_x_max << ","
             << "right_x_min:" << right_x_min << ","
             << "delta_x:" << right_x_max - right_x_min << ","
             << "right_y_max:" << right_y_max << ","
             << "right_y_min:" << right_y_min << ","
             << "delta_y:" << right_y_max - right_y_min;

  HLOG_DEBUG << "[occ module]:" << "left_x_max:" << left_x_max << ","
             << "left_x_min:" << left_x_min << ","
             << "delta_x:" << left_x_max - left_x_min << ","
             << "left_y_max:" << left_y_max << ","
             << "left_y_min:" << left_y_min << ","
             << "delta_y:" << left_y_max - left_y_min;
  HLOG_DEBUG << "[occ module]:" << "right_occ_sp_before_ego"
             << right_occ_sp_before_ego;
  HLOG_DEBUG << "[occ module]:" << "left_occ_sp_before_ego"
             << left_occ_sp_before_ego;
  float right_x_distance_thresh = x_distance_error_thresh_;
  float left_x_distance_thresh = x_distance_error_thresh_;
  if (right_occ_sp_before_ego) {
    right_x_distance_thresh = right_x_distance_thresh * 2;
  }
  if (left_occ_sp_before_ego) {
    left_x_distance_thresh = left_x_distance_thresh * 2;
  }

  return std::abs(right_x_max - right_x_min) <= right_x_distance_thresh &&
         std::abs(right_y_max - right_y_min) <= y_distance_error_thresh_ &&
         std::abs(left_x_max - left_x_min) <= left_x_distance_thresh &&
         std::abs(left_y_max - left_y_min) <= y_distance_error_thresh_;
}

float OccGuideLineManager::GetExitLaneWidth() {
  float averge_exit_lane_width = 3.5;
  const auto& lineline_boundries = bev_laneline_boundarys_;
  HLOG_DEBUG << "[occ module]:" << "bev laneline nums..."
             << lineline_boundries.size();
  if (lineline_boundries.empty()) {
    return averge_exit_lane_width;
  }

  std::vector<em::Boundary::Ptr> back_lines;
  for (const auto& boundary : lineline_boundries) {
    if (boundary.second->nodes.empty()) {
      continue;
    }
    HLOG_DEBUG << "[occ module]:" << "bevlaneline boundary lanepos: "
               << static_cast<int>(boundary.second->lanepos);
    HLOG_DEBUG << "start x:" << boundary.second->nodes.front()->point.x();
    if (boundary.second->nodes.front()->point.x() <= 5.0) {
      HLOG_DEBUG << "back_lines add one line";
      back_lines.emplace_back(boundary.second);
    }
  }

  // 确保是从右到左的车道线排序方式
  std::sort(back_lines.begin(), back_lines.end(),
            [](em::Boundary::Ptr& a, em::Boundary::Ptr& b) {
              return a->nodes.back()->point.y() < b->nodes.back()->point.y();
            });

  if (back_lines.size() < 2) {
    return averge_exit_lane_width;
  }

  int back_lane_num_calu = 0;
  float back_lane_width_totally = 0.0;
  HLOG_DEBUG << "[occ module]:" << "back has more than 2 lines";
  for (int i = 0; i < static_cast<int>(back_lines.size()) - 1; ++i) {
    em::Boundary::Ptr query_line = nullptr;
    em::Boundary::Ptr value_line = nullptr;
    if (back_lines.at(i)->nodes.back()->point.x() <
        back_lines.at(i + 1)->nodes.back()->point.x()) {
      query_line = back_lines.at(i);
      value_line = back_lines.at(i + 1);
    } else {
      query_line = back_lines.at(i + 1);
      value_line = back_lines.at(i);
    }

    const auto query_point = query_line->nodes.back()->point;
    auto it = std::min_element(
        value_line->nodes.begin(), value_line->nodes.end(),
        [&point = query_point](const em::BoundaryNode::Ptr& a,
                               const em::BoundaryNode::Ptr& b) {
          return (point - a->point).norm() < (point - b->point).norm();
        });

    float lane_width = 3.5;
    if (value_line->nodes.size() == 1) {
      lane_width = ((*it)->point - query_point).norm();
    } else if (it == std::prev(value_line->nodes.end())) {
      lane_width = math::perpendicular_distance((*std::prev(it))->point,
                                                (*it)->point, query_point);
    } else {
      lane_width = math::perpendicular_distance((*std::next(it))->point,
                                                (*it)->point, query_point);
    }
    HLOG_DEBUG << "[occ module] calcu back lane width:" << lane_width;
    if (lane_width < 5.0 && lane_width > 2.0) {
      back_lane_width_totally += lane_width;
      ++back_lane_num_calu;
    }
  }

  if (back_lane_num_calu != 0) {
    averge_exit_lane_width =
        back_lane_width_totally / static_cast<float>(back_lane_num_calu);
  }
  if (averge_exit_lane_width < 5.0 && averge_exit_lane_width > 2.0) {
    return averge_exit_lane_width;
  }

  return averge_exit_lane_width;
}

void OccGuideLineManager::FineTuneOccPair(
    std::vector<em::Boundary::Ptr> best_occ_pair) {
  auto& left_occ = best_occ_pair.back();
  auto& right_occ = best_occ_pair.front();

  if (left_occ->nodes.size() < 2 || right_occ->nodes.size() < 2) {
    return;
  }

  const auto left_occ_start_point = left_occ->nodes.front()->point;
  const auto right_occ_start_point = right_occ->nodes.front()->point;
  auto left_occ_start_x = left_occ->nodes.front()->point.x();
  auto left_occ_start_y = left_occ->nodes.front()->point.y();
  auto left_occ_end_x = left_occ->nodes.back()->point.x();
  auto right_occ_start_x = right_occ->nodes.front()->point.x();
  auto right_occ_start_y = right_occ->nodes.front()->point.y();
  auto right_occ_end_x = right_occ->nodes.back()->point.x();

  // 计算左边路沿的前5米的平均heading;
  Eigen::Vector3f left_occ_heading{0.0, 0.0, 0.0};
  int left_heading_nums = 0;
  for (int i = 0; i < static_cast<int>(left_occ->nodes.size()) - 1; ++i) {
    const auto& prev_point = left_occ->nodes.at(i)->point;
    const auto& next_point = left_occ->nodes.at(i + 1)->point;
    if (next_point.x() - left_occ_start_x > 5.0) {
      break;
    }
    left_heading_nums += 1;
    // [-pi, pi]
    // auto heading =
    //     atan2(next_point.y() - prev_point.y(), next_point.x() -
    //     prev_point.x());
    left_occ_heading += (prev_point - next_point);
  }

  if (left_heading_nums == 0) {
    return;
  }
  left_occ_heading /= static_cast<float>(left_heading_nums);
  left_occ_heading.normalize();
  HLOG_DEBUG << "left heading:" << left_occ_heading.x() << ","
             << left_occ_heading.y();

  // 计算右边路沿的前5米的平均heading;
  Eigen::Vector3f right_occ_heading{0.0, 0.0, 0.0};
  int right_heading_nums = 0;

  for (int i = 0; i < static_cast<int>(right_occ->nodes.size()) - 1; ++i) {
    const auto& prev_point = right_occ->nodes.at(i)->point;
    const auto& next_point = right_occ->nodes.at(i + 1)->point;
    if (next_point.x() - right_occ_start_x > 5.0) {
      break;
    }
    right_heading_nums += 1;
    // [-pi, pi]
    // auto heading =
    //     atan2(next_point.y() - prev_point.y(), next_point.x() -
    //     prev_point.x());
    right_occ_heading += (prev_point - next_point);
  }

  if (right_heading_nums == 0) {
    return;
  }
  right_occ_heading /= static_cast<float>(right_heading_nums);
  right_occ_heading.normalize();

  // 两侧occ路沿线近端补齐
  if (left_occ_start_x < right_occ_start_x - 0.8) {
    for (auto x = static_cast<float>(right_occ_start_x - 0.8);
         x > left_occ_start_x; x = static_cast<float>(x - 0.8)) {
      // auto& coeff_params = right_occ->curve_params;
      // auto y = std::pow(x, 3) * coeff_params[3] +
      //          std::pow(x, 2) * coeff_params[2] + x * coeff_params[1] +
      //          coeff_params[0];
      // Eigen::Vector3f virtual_point{x, static_cast<float>(y), 0.0};

      // 补点方式优化，使用一次方程来补点
      Eigen::Vector3f virtual_point =
          (right_occ_start_x - x) * right_occ_heading + right_occ_start_point;
      // HLOG_DEBUG<< "[occ module]add right points:" << x << "," << y;
      if (!ComputerPointIsInLine(virtual_point, left_occ->nodes.front()->point,
                                 left_occ->nodes.back()->point)) {
        break;
      }
      em::BoundaryNode::Ptr node = std::make_shared<em::BoundaryNode>();
      node->point = virtual_point;
      node->dash_seg = em::DashSegType::DASH_START_POINT;
      right_occ->nodes.push_front(node);
    }
  } else if (right_occ_start_x < left_occ_start_x - 0.8) {
    for (auto x = static_cast<float>(left_occ_start_x - 0.8);
         x > right_occ_start_x; x = static_cast<float>(x - 0.8)) {
      // auto& coeff_params = left_occ->curve_params;
      // auto y = std::pow(x, 3) * coeff_params[3] +
      //          std::pow(x, 2) * coeff_params[2] + x * coeff_params[1] +
      //          coeff_params[0];
      // Eigen::Vector3f virtual_point{x, static_cast<float>(y), 0.0};
      // 补点方式优化，使用一次方程来补点
      Eigen::Vector3f virtual_point =
          (left_occ_start_x - x) * left_occ_heading + left_occ_start_point;

      // HLOG_DEBUG<< "[occ module] add left points:" << virtual_point.x()
      // <<
      // ","
      //           << virtual_point.y();
      // HLOG_DEBUG<< "[occ module] right occ start points:"
      //           << right_occ->nodes.front()->point.x() << ","
      //           << right_occ->nodes.front()->point.y();
      // HLOG_DEBUG<< "[occ module] right occ end points:"
      //           << right_occ->nodes.back()->point.x() << ","
      //           << right_occ->nodes.back()->point.y();
      if (!ComputerPointIsInLine(virtual_point, right_occ->nodes.front()->point,
                                 right_occ->nodes.back()->point)) {
        HLOG_DEBUG << "[occ module] add point not in right_occ region !!!";
        break;
      }
      em::BoundaryNode::Ptr node = std::make_shared<em::BoundaryNode>();
      node->point = virtual_point;
      node->dash_seg = em::DashSegType::DASH_START_POINT;
      left_occ->nodes.push_front(node);
    }
  }
// 两侧occ路沿线远端通过裁减的方式进行补齐
#if 1
  if (left_occ_end_x < right_occ_end_x && left_occ_end_x >= right_occ_start_x) {
    right_occ->nodes.erase(
        std::remove_if(right_occ->nodes.begin(), right_occ->nodes.end(),
                       [&](const em::BoundaryNode::Ptr& node) {
                         return node->point.x() > left_occ_end_x;
                       }),
        right_occ->nodes.end());
  } else if (right_occ_end_x < left_occ_end_x &&
             right_occ_end_x >= left_occ_start_x) {
    left_occ->nodes.erase(
        std::remove_if(left_occ->nodes.begin(), left_occ->nodes.end(),
                       [&](const em::BoundaryNode::Ptr& node) {
                         return node->point.x() > right_occ_end_x;
                       }),
        left_occ->nodes.end());
  }

#endif
}

std::vector<em::Boundary> OccGuideLineManager::OccRegionSplit(
    const std::vector<em::Boundary::Ptr>& best_occ_pair) {
  const auto& left_occ = best_occ_pair.back();
  const auto& right_occ = best_occ_pair.front();
  if (left_occ->nodes.empty() || right_occ->nodes.empty()) {
    return {};
  }

  // 更新豁口宽度
  occ_width_ = std::min(occ_dir_vec_.norm(), 40.0F);

  // 如果退出车道数计算过，则保持历史车道数。反之，需计算退出车道数。
  if (assume_entrancelane_nums_by_exitlane_ == -1) {
    assume_entrancelane_nums_by_exitlane_ = static_cast<int>(std::max(
        static_cast<double>(std::floor(
            (occ_width_ - safe_distance_ * 2) / averge_exit_lane_width_ + 0.3)),
        1.0));
  }

  HLOG_DEBUG << "[occ module] entrance lanes width:"
             << "averge_exit_lane_width:" << averge_exit_lane_width_
             << ", occ width: " << occ_width_ << ", entrance lanes nums:"
             << assume_entrancelane_nums_by_exitlane_;

  std::vector<em::Boundary> virtual_lines(
      assume_entrancelane_nums_by_exitlane_ + 1);

  for (const auto& node : right_occ->nodes) {
    HLOG_DEBUG << "444 right occ node:" << node->point.x() << ","
               << node->point.y();
    auto it = std::min_element(
        left_occ->nodes.begin(), left_occ->nodes.end(),
        [&point = node->point](const em::BoundaryNode::Ptr& a,
                               const em::BoundaryNode::Ptr& b) {
          return (point - a->point).norm() < (point - b->point).norm();
        });
    Eigen::Vector3f closestPointB = (*it)->point;
    Eigen::Vector3f delta_norm_vector =
        (closestPointB - node->point).normalized();
    float norm_value =
        (closestPointB - node->point).norm() - 2 * safe_distance_;
    for (int i = 0; i <= assume_entrancelane_nums_by_exitlane_; ++i) {
      Eigen::Vector3f point_i =
          node->point +
          delta_norm_vector *
              (norm_value * i / assume_entrancelane_nums_by_exitlane_ +
               safe_distance_);
      em::BoundaryNode::Ptr node = std::make_shared<em::BoundaryNode>();
      node->point = point_i;
      node->dash_seg = em::DashSegType::UNKNOWN_DASH_SEG;
      virtual_lines[i].nodes.emplace_back(node);
      virtual_lines[i].linetype = em::LaneType_INTERSECTION_VIRTUAL_MARKING;
      virtual_lines[i].color = em::WHITE;
      virtual_lines[i].lanepos = em::LanePositionType_OTHER;
    }
  }

  // 对于occ的线进行插值， 确保间隔小于1m/点
  for (int i = 0; i < virtual_lines.size(); ++i) {
    auto& line = virtual_lines[i];
    for (int point_idx = 1; point_idx < line.nodes.size(); ++point_idx) {
      Eigen::Vector3f p1 = line.nodes[point_idx - 1]->point;
      Eigen::Vector3f p2 = line.nodes[point_idx]->point;
      float distance = (p2 - p1).norm();
      // 如果距离大于0.8米，则插入足够的点以减少距离。
      if (distance > 0.8F) {
        int num_segments = static_cast<int>(distance / (0.8F + 0.001F));
        Eigen::Vector3f delta = (p2 - p1).normalized();
        // 插入中间点。
        for (int j = 1; j <= num_segments; ++j) {
          Eigen::Vector3f new_point = p1 + delta * j * 0.8;
          em::BoundaryNode::Ptr inter_node =
              std::make_shared<em::BoundaryNode>();
          inter_node->point = new_point;
          inter_node->dash_seg = em::DashSegType::UNKNOWN_DASH_SEG;
          line.nodes.insert(line.nodes.begin() + point_idx, inter_node);
          ++point_idx;  // 跳过新插入的点
        }
      }
    }
  }

  HLOG_DEBUG << "OccRegionSplit FineTuneOccPair right occ node size:"
             << best_occ_pair.front()->nodes.size();
  HLOG_DEBUG << "OccRegionSplit FineTuneOccPair left occ node size:"
             << best_occ_pair.back()->nodes.size();
  // IsLineBetweenOcc
  virtual_lines.erase(
      std::remove_if(virtual_lines.begin(), virtual_lines.end(),
                     [&](const em::Boundary& line) {
                       return !IsLineBetweenOcc(
                           std::make_shared<em::Boundary>(line));
                     }),
      virtual_lines.end());

  return virtual_lines;
}

std::vector<em::Boundary> OccGuideLineManager::InferGuideLineOnlyByOcc() {
  // 猜想进入车道的平均宽度。
  averge_exit_lane_width_ = GetExitLaneWidth();
  HLOG_DEBUG << "[occ module]:" << "exit_lane_width:"
             << averge_exit_lane_width_;
  auto best_occ_pair = GetStableOcc();

  // !TBD, 基于最短距离点做截断，根据一次防方程做前端补齐。
  // FineTuneOccPair(best_occ_pair);
  HLOG_DEBUG << "after FineTuneOccPair right occ node size:"
             << best_occ_pair.front()->nodes.size();
  HLOG_DEBUG << "after FineTuneOccPair left occ node size:"
             << best_occ_pair.back()->nodes.size();

  HLOG_DEBUG << "right occ end node x loc:"
             << best_occ_pair.front()->nodes.back()->point.x();
  HLOG_DEBUG << "left occ end node x loc:"
             << best_occ_pair.back()->nodes.back()->point.x();

  HLOG_DEBUG << "right occ front node x loc:"
             << best_occ_pair.front()->nodes.front()->point.x();
  HLOG_DEBUG << "left occ front node x loc:"
             << best_occ_pair.back()->nodes.front()->point.x();
  std::vector<em::Boundary> virtual_lines = OccRegionSplit(best_occ_pair);
  return virtual_lines;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
