/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： group_map.cc
 *   author     ： taoshaoyuan
 *   date       ： 2024.01
 ******************************************************************************/

#include "map_fusion/road_recognition/occ_guideline_manager.h"
#include <pcl/segmentation/extract_clusters.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstddef>
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
        FineTuneGuideLine(bev_lanelines);  // (感知出现的线保留, 其他的线虚拟)
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

  if (node->point.x() > occ_data->nodes.back()->point.x()) {
    flag = node->point.y() < occ_data->nodes.back()->point.y();
    return flag;
  }

  if (node->point.x() < occ_data->nodes.front()->point.x()) {
    flag = node->point.y() < occ_data->nodes.front()->point.y();
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
                  PointInVectorSide(
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

  if (node->point.x() > occ_data->nodes.back()->point.x()) {
    flag = node->point.y() > occ_data->nodes.back()->point.y();
    return flag;
  }

  if (node->point.x() < occ_data->nodes.front()->point.x()) {
    flag = node->point.y() > occ_data->nodes.front()->point.y();
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
                  PointInVectorSide(
                      Eigen::Vector2f{first_point_it.x(), first_point_it.y()},
                      Eigen::Vector2f{second_point_it.x(), second_point_it.y()},
                      query_node) < 1;

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
  // Eigen::Vector2f right_occ_start_point =
  //     right_occ->nodes.front()->point.head<2>();
  // Eigen::Vector2f right_occ_end_point =
  //     right_occ->nodes.back()->point.head<2>();
  // Eigen::Vector2f left_occ_start_point =
  //     left_occ->nodes.front()->point.head<2>();
  // Eigen::Vector2f left_occ_end_point =
  // left_occ->nodes.back()->point.head<2>();

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

  int point_in_occ_num = 0;
  for (const auto& node : line->nodes) {
    HLOG_DEBUG << "[occ module] node point:" << node->point.x() << ","
               << node->point.y();
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
             << 1.0 * point_in_occ_num / static_cast<int>(line->nodes.size());
  bool line_in_occ =
      1.0 * point_in_occ_num / static_cast<int>(line->nodes.size()) > 0.8;

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

  HLOG_DEBUG << "000 right occ end node x loc:"
             << right_occ->nodes.back()->point.x();
  HLOG_DEBUG << "000 left occ end node x loc:"
             << left_occ->nodes.back()->point.x();
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

  // HLOG_DEBUG << "right occ curve params:" << right_occ->curve_params[3] <<
  // ","
  //           << right_occ->curve_params[2] << "," <<
  //           right_occ->curve_params[1]
  //           << "," << right_occ->curve_params[0];
  // for (auto& node : right_occ->nodes) {
  //   HLOG_DEBUG << "right occ: point loc:" << node->point.x() << ","
  //             << node->point.y();
  // }

  // HLOG_DEBUG << "left occ curve params:" << left_occ->curve_params[3] << ","
  //           << left_occ->curve_params[2] << "," << left_occ->curve_params[1]
  //           << "," << left_occ->curve_params[0];
  // for (auto& node : left_occ->nodes) {
  //   HLOG_DEBUG << "left occ: point loc:" << node->point.x() << ","
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
      // HLOG_DEBUG << "bev roadedge point:" << x << "," << node->point.y() <<
      // "\n"
      //           << "occ roadedge point:" << x << "," << occ_y;
      delta_error += std::abs(occ_y - node->point.y());
      point_num += 1;
    }

    double ave_error = delta_error / point_num;
    HLOG_DEBUG << "[occ module] right Occ and Pair bev roadedge:"
               << "ave_y_error is:" << ave_error;
    if (ave_error <= distance_error_thresh_) {
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
    if (ave_error <= distance_error_thresh_) {
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

float OccGuideLineManager::CorrectLaneWidth(
    const std::vector<em::Boundary::Ptr>& bev_lanelines) {
  if (bev_lanelines.size() < 2) {
    return assume_lane_width_;
  }

  for (int i = 0; i < static_cast<int>(bev_lanelines.size()) - 1; ++i) {
    if (bev_lanelines.at(i)->nodes.empty() ||
        bev_lanelines.at(i + 1)->nodes.empty()) {
      continue;
    }
    float lane_space =
        GetTwoBoundayDis(bev_lanelines.at(i), bev_lanelines.at(i + 1));
    if (lane_space > 4.5 || lane_space < 2.0) {
      continue;
    }

    return lane_space;
  }

  return assume_lane_width_;
}

float OccGuideLineManager::GetTwoBoundayDis(
    const em::Boundary::Ptr& left_boundary,
    const em::Boundary::Ptr& right_boundary) {
  float lane_space = 3.5;
  em::Boundary::Ptr query_line = nullptr;
  em::Boundary::Ptr value_line = nullptr;
  if (left_boundary->nodes.front()->point.x() <
      right_boundary->nodes.front()->point.x()) {
    query_line = right_boundary;
    value_line = left_boundary;
  } else {
    query_line = left_boundary;
    value_line = right_boundary;
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
    lane_space = perpendicular_distance((*std::prev(it))->point, (*it)->point,
                                        query_point);
  } else {
    lane_space = perpendicular_distance((*std::next(it))->point, (*it)->point,
                                        query_point);
  }
  return lane_space;
}

std::vector<em::Boundary> OccGuideLineManager::FineTuneGuideLine(
    const std::vector<em::Boundary::Ptr>& bev_lanelines) {
  std::vector<em::Boundary> virutal_lines;
  int vitual_lane_num = -1;

  auto stable_occs = GetStableOcc();
  auto& right_occ = stable_occs.front();
  auto& left_occ = stable_occs.back();

  float fine_lane_width = CorrectLaneWidth(bev_lanelines);
  HLOG_DEBUG << "[occ module] fine_lane_width:" << fine_lane_width;
  const auto& far_right_line = bev_lanelines.front();
  const auto& far_left_line = bev_lanelines.back();
  if (right_occ->nodes.empty() || left_occ->nodes.empty()) {
    return {};
  }

  if (1) {
    float right_blank_space = GetTwoBoundayDis(right_occ, far_right_line);
    HLOG_DEBUG << "[occ module] right_blank_space:" << right_blank_space;
    if (right_blank_space < -0.3) {
      return {};
    }

    vitual_lane_num =
        static_cast<int>(std::floor(right_blank_space / fine_lane_width + 0.3));

    // !TBD 这里对于生成的车道线需要做个check，需确保没有超出occ边界。
    if (vitual_lane_num > 0) {
      for (int i = 1; i <= vitual_lane_num; ++i) {
        em::Boundary virtual_line;
        for (const auto& node : far_right_line->nodes) {
          em::BoundaryNode::Ptr virtual_node =
              std::make_shared<em::BoundaryNode>();
          virtual_node->point =
              node->point + Eigen::Vector3f{0.0, -1 * fine_lane_width * i, 0.0};
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
    float left_blank_space = GetTwoBoundayDis(left_occ, far_left_line);
    HLOG_DEBUG << "[occ module] left_blank_space:" << left_blank_space;
    if (left_blank_space < -0.3) {
      return {};
    }

    vitual_lane_num =
        static_cast<int>(std::floor(left_blank_space / fine_lane_width + 0.3));

    if (vitual_lane_num > 0) {
      for (int i = 1; i <= vitual_lane_num; ++i) {
        em::Boundary virtual_line;
        for (const auto& node : far_left_line->nodes) {
          em::BoundaryNode::Ptr virtual_node =
              std::make_shared<em::BoundaryNode>();
          virtual_node->point =
              node->point + Eigen::Vector3f{0.0, fine_lane_width * i, 0.0};
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
      float lane_width = GetTwoBoundayDis(bev_lanelines.at(line_idx),
                                          bev_lanelines.at(line_idx + 1));
      vitual_lane_num =
          static_cast<int>(std::floor(lane_width / fine_lane_width + 0.3));
      HLOG_DEBUG << "[occ module] line_blank_space:" << lane_width
                 << ", and virtual lane num:" << vitual_lane_num;
      if (vitual_lane_num > 1) {
        for (int i = 1; i <= vitual_lane_num - 1; ++i) {
          em::Boundary virtual_line;
          for (const auto& node : bev_lanelines.at(line_idx)->nodes) {
            em::BoundaryNode::Ptr virtual_node =
                std::make_shared<em::BoundaryNode>();
            virtual_node->point =
                node->point + Eigen::Vector3f{0.0, fine_lane_width * i, 0.0};
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
    const auto center_point =
        (front_occ_pair.at(i).second->road_points.front() +
         front_occ_pair.at(i + 1).second->road_points.front()) /
        2.0;

    float cur_angle = std::fabs(std::atan2(center_point.y(), center_point.x()));
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

// Eigen::Vector3f OccGuideLineManager::CalcuDirectionVec(
//     em::Boundary::Ptr occ_l1_pts, em::Boundary::Ptr occ_l2_pts) {
//   /*
//     情况1:
//     ------------------ occ_l2_pts
//     →
//       --------------- occ_l1_pts
//   */
//   /*
//     情况2:
//         --------------- occ_l2_pts
//     →
//     ------------------- occ_l1_pts
//   */
//   // 更新方向向量
//   if (occ_l1_pts->nodes.empty() || occ_l2_pts->nodes.empty()) {
//     return {};
//   }

//   Eigen::Vector3f dir_vec =
//       occ_l2_pts->nodes.front()->point - occ_l1_pts->nodes.front()->point;
//   if (occ_l1_pts->nodes.front()->point.x() >
//       occ_l2_pts->nodes.front()->point.x()) {
//     auto p = occ_l1_pts->nodes.front()->point;
//     for (int i = 0; i < occ_l2_pts->nodes.size() - 1; i++) {
//       auto a = occ_l2_pts->nodes.at(i)->point;
//       auto b = occ_l2_pts->nodes.at(i + 1)->point;
//       const auto& ab = b - a;
//       const auto& ap = p - a;
//       double ABLengthSquared = ab.squaredNorm();
//       if (std::fabs(ABLengthSquared) < 1e-6) {
//         continue;
//       }
//       double t = ab.dot(ap) / ABLengthSquared;
//       if (t < 0 || t > 1) {
//         continue;
//       }
//       t = std::max(0.0, std::min(t, 1.0));
//       auto c = a + t * ab;  // 点到线段的最近点
//       dir_vec = c - p;
//       break;
//     }
//   } else {
//     auto p = occ_l2_pts->nodes.front()->point;
//     for (int i = 0; i < occ_l1_pts->nodes.size() - 1; i++) {
//       auto a = occ_l1_pts->nodes.at(i)->point;
//       auto b = occ_l1_pts->nodes.at(i + 1)->point;
//       const auto& ab = b - a;
//       const auto& ap = p - a;
//       double ABLengthSquared = ab.squaredNorm();
//       if (std::fabs(ABLengthSquared) < 1e-6) {
//         continue;
//       }
//       double t = ab.dot(ap) / ABLengthSquared;
//       if (t < 0 || t > 1) {
//         continue;
//       }
//       t = std::max(0.0, std::min(t, 1.0));
//       auto c = a + t * ab;  // 点到线段的最近点
//       dir_vec = p - c;
//       break;
//     }
//   }
//   return dir_vec;
// }

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

void OccGuideLineManager::AddCurrentMeasurementOcc() {
  std::vector<std::pair<em::Id, em::OccRoad::Ptr>> front_occ_pair =
      GetFrontOccRoadPair();

  std::vector<std::pair<em::Id, em::OccRoad::Ptr>> best_occ_pair =
      GetBestOccPair(front_occ_pair);

  std::vector<em::Boundary::Ptr> best_occ_pair_local;
  if (best_occ_pair.empty()) {
    history_n_best_occs_.push_back(best_occ_pair_local);
    return;
  }
  // HLOG_DEBUG << "right occ curve params:"
  //           << best_occ_pair.front().second->curve_params[3] << ","
  //           << best_occ_pair.front().second->curve_params[2] << ","
  //           << best_occ_pair.front().second->curve_params[1] << ","
  //           << best_occ_pair.front().second->curve_params[0];
  // for (auto& node : best_occ_pair.front().second->road_points) {
  //   HLOG_DEBUG << "right occ: before trans point loc: " << node.x() << ","
  //             << node.y();
  // }

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
    boundary_data->id = best_occ_pair.at(i).second->track_id;
    boundary_data->curve_params = best_occ_pair.at(i).second->curve_params;
    best_occ_pair_local.emplace_back(boundary_data);
  }

  history_n_best_occs_.push_back(best_occ_pair_local);
}

bool OccGuideLineManager::CheckOccWhetherStable() {
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
  for (int i = 0; i < history_n_best_occs_.size(); ++i) {
    if (history_n_best_occs_.at(i).empty() ||
        history_n_best_occs_.at(i).front() == nullptr ||
        history_n_best_occs_.at(i).back() == nullptr ||
        history_n_best_occs_.at(i).front()->nodes.empty() ||
        history_n_best_occs_.at(i).back()->nodes.empty()) {
      return false;
    }
    auto& right_occ_point =
        history_n_best_occs_.at(i).front()->nodes.front()->point;
    auto& left_occ_point =
        history_n_best_occs_.at(i).back()->nodes.front()->point;
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

  return std::abs(right_x_max - right_x_min) <= 2 * distance_error_thresh_ &&
         std::abs(right_y_max - right_y_min) <= distance_error_thresh_ &&
         std::abs(left_x_max - left_x_min) <= 2 * distance_error_thresh_ &&
         std::abs(left_y_max - left_y_min) <= distance_error_thresh_;
}

float OccGuideLineManager::perpendicular_distance(const Eigen::Vector3f& A,
                                                  const Eigen::Vector3f& B,
                                                  const Eigen::Vector3f& P) {
  // 向量 AB
  Eigen::Vector3f AB = B - A;
  // 向量 AP
  Eigen::Vector3f AP = P - A;
  // 叉乘 AB x AP
  Eigen::Vector3f crossProduct = AB.cross(AP);
  // AB 的模长
  float lengthAB = AB.norm();
  // 垂线距离
  float distance = crossProduct.norm() / (lengthAB + 0.001);

  return distance;
}

float OccGuideLineManager::AssumeOccVirtualLaneWidth() {
  float averge_lane_width = 3.5;
  const auto& lineline_boundries = bev_laneline_boundarys_;
  HLOG_DEBUG << "[occ module]:" << "bev laneline nums..."
             << lineline_boundries.size();
  if (lineline_boundries.empty()) {
    return averge_lane_width;
  }

  std::vector<em::Boundary::Ptr> back_lines;
  std::vector<em::Boundary::Ptr> front_lines;

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
    if (boundary.second->nodes.front()->point.x() > 5.0) {
      HLOG_DEBUG << "front_lines add one line";
      front_lines.emplace_back(boundary.second);
    }
  }

  // 确保是从右到左的车道线排序方式
  std::sort(back_lines.begin(), back_lines.end(),
            [](em::Boundary::Ptr& a, em::Boundary::Ptr& b) {
              return static_cast<int>(a->lanepos) >
                     static_cast<int>(b->lanepos);
            });

  std::sort(front_lines.begin(), front_lines.end(),
            [](em::Boundary::Ptr& a, em::Boundary::Ptr& b) {
              return a->nodes.front()->point.y() < b->nodes.front()->point.y();
            });

  if (front_lines.size() >= 2) {
    int front_lane_num_calu = 0;
    float front_lane_width_totally = 0.0;
    HLOG_DEBUG << "[occ module]:" << "front has more than 2 lines";
    for (int i = 0; i < static_cast<int>(front_lines.size()) - 1; ++i) {
      em::Boundary::Ptr query_line = nullptr;
      em::Boundary::Ptr value_line = nullptr;
      if (front_lines.at(i)->nodes.front()->point.x() <
          front_lines.at(i + 1)->nodes.front()->point.x()) {
        query_line = front_lines.at(i + 1);
        value_line = front_lines.at(i);
      } else {
        query_line = front_lines.at(i);
        value_line = front_lines.at(i + 1);
      }

      const auto query_point = query_line->nodes.front()->point;
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
        lane_width = perpendicular_distance((*std::prev(it))->point,
                                            (*it)->point, query_point);
      } else {
        lane_width = perpendicular_distance((*std::next(it))->point,
                                            (*it)->point, query_point);
      }

      HLOG_DEBUG << "[occ module] calcu front lane width:" << averge_lane_width;
      if (lane_width < 5.0 && lane_width > 2.0) {
        front_lane_width_totally += lane_width;
        ++front_lane_num_calu;
      }
    }

    if (front_lane_num_calu != 0) {
      averge_lane_width =
          front_lane_width_totally / static_cast<float>(front_lane_num_calu);
    }
    if (averge_lane_width < 5.0 && averge_lane_width > 2.0) {
      return averge_lane_width;
    }
  }

  if (back_lines.size() >= 2) {
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
        lane_width = perpendicular_distance((*std::prev(it))->point,
                                            (*it)->point, query_point);
      } else {
        lane_width = perpendicular_distance((*std::next(it))->point,
                                            (*it)->point, query_point);
      }
      HLOG_DEBUG << "[occ module] calcu back lane width:" << lane_width;
      if (lane_width < 5.0 && lane_width > 2.0) {
        back_lane_width_totally += lane_width;
        ++back_lane_num_calu;
      }
    }

    if (back_lane_num_calu != 0) {
      averge_lane_width =
          back_lane_width_totally / static_cast<float>(back_lane_num_calu);
    }
    if (averge_lane_width < 5.0 && averge_lane_width > 2.0) {
      return averge_lane_width;
    }
  }

  return averge_lane_width;
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
      // HLOG_DEBUG << "[occ module]add right points:" << x << "," << y;
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

      // HLOG_DEBUG << "[occ module] add left points:" << virtual_point.x() <<
      // ","
      //           << virtual_point.y();
      // HLOG_DEBUG << "[occ module] right occ start points:"
      //           << right_occ->nodes.front()->point.x() << ","
      //           << right_occ->nodes.front()->point.y();
      // HLOG_DEBUG << "[occ module] right occ end points:"
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
  const auto occ_width = std::min(occ_dir_vec_.norm(), 40.0F);
  int assume_lane_nums = static_cast<int>(std::max(
      static_cast<double>(std::floor(
          (occ_width - safe_distance_ * 2) / assume_lane_width_ + 0.3)),
      1.0));
  HLOG_DEBUG << "[occ module] entrance lanes width:" << "averge_lane_width:"
             << assume_lane_width_ << ", occ width: " << occ_width
             << ", entrance lanes nums:" << assume_lane_nums;

  std::vector<em::Boundary> virtual_lines(assume_lane_nums + 1);

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
    HLOG_DEBUG << "444 left occ node:" << closestPointB.x() << ","
               << closestPointB.y();
    Eigen::Vector3f delta_norm_vector =
        (closestPointB - node->point).normalized();
    HLOG_DEBUG << "444 delta_norm_vector:" << delta_norm_vector.x() << ","
               << delta_norm_vector.y();
    float norm_value =
        (closestPointB - node->point).norm() - 2 * safe_distance_;
    for (int i = 0; i <= assume_lane_nums; ++i) {
      Eigen::Vector3f point_i =
          node->point + delta_norm_vector * (norm_value * i / assume_lane_nums +
                                             safe_distance_);
      HLOG_DEBUG << "444 create occ node:" << point_i.x() << "," << point_i.y();
      em::BoundaryNode::Ptr node = std::make_shared<em::BoundaryNode>();
      node->point = point_i;
      node->dash_seg = em::DashSegType::UNKNOWN_DASH_SEG;
      virtual_lines[i].nodes.emplace_back(node);
      virtual_lines[i].linetype = em::LaneType_INTERSECTION_VIRTUAL_MARKING;
      virtual_lines[i].color = em::WHITE;
      virtual_lines[i].lanepos = em::LanePositionType_OTHER;
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
  float assume_lane_width = AssumeOccVirtualLaneWidth();
  HLOG_DEBUG << "[occ module]:" << "assume_lane_width:" << assume_lane_width;
  assume_lane_width_ = assume_lane_width;
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
