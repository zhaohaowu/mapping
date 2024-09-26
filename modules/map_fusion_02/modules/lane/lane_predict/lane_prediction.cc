/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： lane_prediction.cc
 *   author     ： xuliang
 *   date       ： 2024.08
 ******************************************************************************/
#include "modules/map_fusion_02/modules/lane/lane_predict/lane_prediction.h"
#include <pcl/segmentation/extract_clusters.h>

#include <algorithm>
#include <limits>
#include <tuple>

#include "base/group.h"
#include "base/utils/log.h"
#include "modules/map_fusion_02/base/element_base.h"
#include "modules/map_fusion_02/common/calc_util.h"
#include "modules/map_fusion_02/data_manager/location_data_manager.h"

using hozon::common::math::Vec2d;
using hozon::common::math::double_type::Compare;

namespace hozon {
namespace mp {
namespace mf {

bool LanePrediction::Init(const LaneFusionProcessOption& conf) {
  conf_ = conf;
  HLOG_INFO << "Lane prediction init";
  return true;
}

void LanePrediction::Clear() {
  ego_lane_id_.left_id = -200;
  ego_lane_id_.right_id = -200;
  delta_pose_heading_ = 0.;
}

void LanePrediction::SetEgoLaneId() {
  ego_lane_id_ = LOCATION_MANAGER->GetEgoLane();
}

bool LanePrediction::LaneForwardPredict(std::vector<Group::Ptr>* groups,
                                        const double& stamp) {
  // 对远处车道线进行预测，仅对无后继的lane尝试预测
  HLOG_DEBUG << "predict success lane";
  if (conf_.predict_farthest_dist > conf_.robust_percep_dist) {
    Group::Ptr last_grp = nullptr;
    // 找到最后一个非空的group，只预测最后一个group里的lane
    for (auto it = groups->rbegin(); it != groups->rend(); ++it) {
      if ((*it) == nullptr || (*it)->lanes.empty()) {
        continue;
      }
      last_grp = *it;
      break;
    }
    bool need_pred_kappa = true;
    if (last_grp != nullptr) {
      // 判断车辆距离group的距离，如果太近，就认为是路口，就将kappa、dkappa置为0
      auto last_grp_end_slice = last_grp->end_slice;
      Eigen::Vector2f end_pl(last_grp_end_slice.pl.x(),
                             last_grp_end_slice.pl.y());
      Eigen::Vector2f end_pr(last_grp_end_slice.pr.x(),
                             last_grp_end_slice.pr.y());
      Eigen::Vector2f curr_pos(0, 0);
      auto dist_to_slice = PointToVectorDist(end_pl, end_pr, curr_pos);
      if (dist_to_slice <= 30) {
        need_pred_kappa = false;
      }

      bool check_back = true;
      if (last_grp->is_last_after_erased) {
        check_back = false;
      }
      std::vector<Lane::Ptr> lanes_wo_next;  // 末端lane，即无后继的lane

      for (const auto& lane : last_grp->lanes) {
        if (lane == nullptr) {
          HLOG_ERROR << "found nullptr lane";
          continue;
        }
        if (lane->next_lane_str_id_with_group.empty()) {
          lanes_wo_next.emplace_back(lane);
        }
      }
      std::vector<LineSegment::Ptr> lines_need_pred;
      std::vector<Lane::Ptr> lanes_need_pred;
      for (auto& lane : lanes_wo_next) {
        int lane_center_need_pre = 0;
        if (lane->left_boundary != nullptr &&
            LaneLineNeedToPredict(*lane->left_boundary, check_back)) {
          lane_center_need_pre++;
        }
        if (lane->right_boundary != nullptr &&
            LaneLineNeedToPredict(*lane->right_boundary, check_back)) {
          lane_center_need_pre++;
        }
        // 左右边线都满足预测要求才会往前预测
        if (lane_center_need_pre == 2) {
          lines_need_pred.emplace_back(lane->left_boundary);
          lines_need_pred.emplace_back(lane->right_boundary);
          lanes_need_pred.emplace_back(lane);
        }
      }
      // 计算切割并构建group之后的车道线heading避免local map尾端不准带来误差
      ComputeLineHeadingPredict(groups, &lines_need_pred);
      // 使用平均heading，这样可以使得预测的线都是平行的，不交叉
      // 由于部分弯道heading偏差较大，导致整体平均heading发生偏差，
      // 现增加pred_end_heading字段用于预测，使用PCL欧式聚类对heading进行聚类
      if (!lines_need_pred.empty()) {
        // PCL欧式聚类 阈值为10度
        size_t index = last_grp->str_id.find("-");
        index++;
        std::string last_grp_lines_id =
            last_grp->str_id.substr(index, last_grp->str_id.size() - index);
        const double heading_threshold = 10. / 180. * M_PI;
        HeadingCluster(lanes_need_pred, &lines_need_pred, last_grp_lines_id,
                       heading_threshold, need_pred_kappa);

        // 重新构建group 线的类型为虚线
        std::vector<Lane::Ptr> center_line_pred;
        for (auto& lane : lanes_need_pred) {
          PredictLaneLine(&center_line_pred, lane);
        }
        Group grp;
        grp.stamp = stamp;
        grp.group_state = Group::GroupState::VIRTUAL;
        grp.str_id = last_grp->str_id + "P";
        grp.lanes = center_line_pred;
        (*groups).emplace_back(std::make_shared<Group>(grp));
      }
    }
  }

  return true;
}

void LanePrediction::ComputeHeadingCompensation(const KinePosePtr& curr_pose) {
  // 计算上一帧和这一帧pose的heading偏差
  if (last_pose_ != nullptr) {
    delta_pose_heading_ =
        math::CalculateHeading(last_pose_->quat, curr_pose->quat);
  }
  last_pose_ = curr_pose;
}

// 判断车道线是否需要预测，当前预测的条件是：
// 1.线的末端点必须在车前方；
// 2.线的末端点距离自车距离在可信赖的感知范围之外，即可信赖的感知范围的线是不会预测的；
// 3.线末端平均heading足够小，即heading与自车行驶方向差异较大的线也不会预测；
// 4.线末端heading的标准差足够小（太大说明heading变换较大，可能是曲线）；
// 5.线末端平均点间距足够大；
//! TBD：当前仅对比较平直的线进行直线预测，后续考虑对弯道进行预测
bool LanePrediction::LaneLineNeedToPredict(const LineSegment& line,
                                           bool check_back) {
  if (line.pts.empty()) {
    return false;
  }
  Eigen::Vector2f back_pt = line.pts.back().pt.head<2>();
  auto mean_heading = line.mean_end_heading;
  auto mean_heading_std = line.mean_end_heading_std_dev;
  auto mean_interval = line.mean_end_interval;
  float dist_to_veh = back_pt.norm();
  bool valid_back = true;
  if (check_back && back_pt.x() < -conf_.junction_predict_distance) {
    valid_back = false;
  }
  if (valid_back) {
    return true;
  }
  return false;
}

void LanePrediction::ComputeLineHeadingPredict(
    std::vector<Group::Ptr>* groups,
    std::vector<LineSegment::Ptr>* lines_need_pred) {
  for (auto& line : *lines_need_pred) {
    std::vector<Vec2d> line_points;
    // 从group中往前递推寻找同一个track id的车道线并把点填充到line points当中
    for (auto it = groups->rbegin(); it != groups->rend(); ++it) {
      if ((*it) == nullptr || (*it)->lanes.empty()) {
        continue;
      }
      std::vector<Vec2d> line_points_temp;
      for (const auto& lane : (*it)->lanes) {
        if (lane->left_boundary->id == line->id ||
            lane->right_boundary->id == line->id) {
          std::vector<Vec2d> lane_boundary_points;
          if (lane->left_boundary->id == line->id) {
            for (const auto& point : lane->left_boundary->pts) {
              lane_boundary_points.emplace_back(point.pt.x(), point.pt.y());
            }
          } else {
            for (const auto& point : lane->right_boundary->pts) {
              lane_boundary_points.emplace_back(point.pt.x(), point.pt.y());
            }
          }
          Vec2d dist_point_temp;
          for (const auto& point : lane_boundary_points) {
            Vec2d temp_point;
            temp_point.set_x(dist_point_temp.x() - point.x());
            temp_point.set_y(dist_point_temp.y() - point.y());
            if (temp_point.Length() >= 0.4) {
              line_points_temp.emplace_back(point.x(), point.y());
            }
            dist_point_temp.set_x(point.x());
            dist_point_temp.set_y(point.y());
          }
          break;
        }
      }

      line_points_temp.insert(line_points_temp.end(), line_points.begin(),
                              line_points.end());
      line_points.clear();
      line_points = line_points_temp;
      if (line_points.size() >= 10) {
        break;
      }
    }

    double heading = 0;
    std::vector<double> thetas;
    for (int i = static_cast<int>(line_points.size()) - 1; i > 0; i--) {
      int j = i - 1;
      for (; j >= 0; j--) {
        const Eigen::Vector2f pb(line_points[i].x(), line_points[i].y());
        const Eigen::Vector2f pa(line_points[j].x(), line_points[j].y());
        Eigen::Vector2f v = pb - pa;
        if (v.norm() > 5.0) {
          double theta = atan2(v.y(), v.x());
          thetas.emplace_back(theta);
          i = j;
          break;
        }
      }
      if (thetas.size() >= 1) {
        break;
      }
    }
    heading =
        thetas.empty()
            ? 0.0
            : std::accumulate(thetas.begin(), thetas.end(), 0.) / thetas.size();

    std::vector<double> fit_result;
    std::vector<Vec2d> fit_points;
    std::vector<double> kappas;
    std::vector<double> dkappas;
    double kappa = 0.;
    double dkappa = 0.;
    if (line_points.size() >= 10) {
      int fit_num = std::min(50, static_cast<int>(line_points.size()));
      std::copy(line_points.rbegin(), line_points.rbegin() + fit_num,
                std::back_inserter(fit_points));
      math::FitLaneLinePoint(fit_points, &fit_result);
      std::vector<Vec2d> cmp_points;
      std::copy(line_points.rbegin(), line_points.rbegin() + 10,
                std::back_inserter(cmp_points));
      math::ComputeDiscretePoints(cmp_points, fit_result, &kappas, &dkappas);
      // 整体平均
      kappa = kappas.empty() ? 0.0
                             : std::accumulate(kappas.begin() + 5,
                                               kappas.begin() + 10, 0.) /
                                   5.;
      dkappa = dkappas.empty() ? 0.0
                               : std::accumulate(dkappas.begin() + 5,
                                                 dkappas.begin() + 10, 0.) /
                                     5.;
      kappa = kappa / 8.;
      dkappa = dkappa / 8.;
    }

    line->pred_end_heading = std::make_tuple(heading, kappa, dkappa);
  }
}

void LanePrediction::HeadingCluster(
    const std::vector<Lane::Ptr>& lanes_need_pred,
    std::vector<LineSegment::Ptr>* lines_need_pred,
    const std::string& last_grp_lines_id, double threshold,
    bool need_pred_kappa) {
  static double last_predict_angle = 0.0;
  static double last_predict_kappa = 0.0;
  static double last_predict_dkappa = 0.0;
  last_predict_angle -= delta_pose_heading_;

  std::tuple<double, double, double> ego_left_pred_data{0, 0, 0};
  std::tuple<double, double, double> ego_right_pred_data{0, 0, 0};
  pcl::PointCloud<pcl::PointXYZ>::Ptr heading_data(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& line : *lines_need_pred) {
    if (last_grp_lines_id.find(std::to_string(line->id)) == std::string::npos) {
      continue;
    }
    pcl::PointXYZ point;
    point.x = static_cast<float>(
        std::get<0>(line->pred_end_heading));  // 将一维heading添加到x轴上
    point.y = 0.0;
    point.z = 0.0;
    heading_data->emplace_back(point);
    HLOG_DEBUG << "----mean_heading:" << line->id << ","
               << std::get<0>(line->pred_end_heading) * 180 / M_PI;
    HLOG_DEBUG << "----mean_heading:" << line->id << ","
               << std::get<0>(line->pred_end_heading) * 180 / M_PI;

    if (line->id == ego_lane_id_.left_id) {
      ego_left_pred_data = line->pred_end_heading;
    }
    if (line->id == ego_lane_id_.right_id) {
      ego_right_pred_data = line->pred_end_heading;
    }
  }

  double heading = 0.0;
  double kappa = 0.;
  double dkappa = 0.;
  bool ego_line_flag = false;

  if (std::get<1>(ego_left_pred_data) != 0. &&
      std::get<1>(ego_right_pred_data) != 0.) {
    heading =
        (std::get<0>(ego_left_pred_data) + std::get<0>(ego_right_pred_data)) /
        2;
    kappa =
        (std::get<1>(ego_left_pred_data) + std::get<1>(ego_right_pred_data)) /
        2;
    dkappa =
        (std::get<2>(ego_left_pred_data) + std::get<2>(ego_right_pred_data)) /
        2;
    ego_line_flag = true;
  } else if (std::get<1>(ego_left_pred_data) != 0.) {
    heading = std::get<0>(ego_left_pred_data);
    kappa = std::get<1>(ego_left_pred_data);
    dkappa = std::get<2>(ego_left_pred_data);
    ego_line_flag = true;
  } else if (std::get<1>(ego_right_pred_data) != 0.) {
    heading = std::get<0>(ego_right_pred_data);
    kappa = std::get<1>(ego_right_pred_data);
    dkappa = std::get<2>(ego_right_pred_data);
    ego_line_flag = true;
  }

  if (math::DoubleHasSameSign(kappa, dkappa) ||
      Compare(std::fabs(dkappa), std::fabs(kappa)) > 0) {
    dkappa = 0.0;
  }

  if (!need_pred_kappa) {
    kappa = 0.;
    dkappa = 0.;
  }

  pcl::search::KdTree<pcl::PointXYZ>::Ptr heading_data_tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  heading_data_tree->setInputCloud(heading_data);

  // 对heading进行欧式聚类
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> heading_cluster;
  heading_cluster.setClusterTolerance(threshold);
  heading_cluster.setMinClusterSize(1);
  heading_cluster.setMaxClusterSize(heading_data->size());
  heading_cluster.setSearchMethod(heading_data_tree);
  heading_cluster.setInputCloud(heading_data);

  std::vector<pcl::PointIndices> heading_cluster_indices;
  heading_cluster.extract(heading_cluster_indices);
  HLOG_DEBUG << "cluser size:" << heading_cluster_indices.size();
  double predict_heading = 0.0;
  if (ego_line_flag) {
    predict_heading = heading;
    predict_heading = 0.8 * last_predict_angle + 0.2 * predict_heading;
    HLOG_DEBUG << "heading mode 1:" << predict_heading;
  } else {
    if (heading_cluster_indices.size() == 1) {
      double heading_sum = 0.;
      int heading_data_size = 0;
      for (const auto& idx : heading_cluster_indices[0].indices) {
        auto heading_data_element = (*heading_data)[idx].x;
        heading_sum = heading_sum + heading_data_element;
        heading_data_size = heading_data_size + 1;
      }
      predict_heading =
          heading_data_size != 0 ? heading_sum / heading_data_size : 0;
      HLOG_DEBUG << "heading mode 2:" << predict_heading << ","
                 << heading_cluster_indices.size();
      predict_heading = 0.8 * last_predict_angle + 0.2 * predict_heading;
    } else {
      int nearest_lane_index = -1;
      double nearest_lane_dist = std::numeric_limits<float>::max();
      // 找到自车所在的lane
      Eigen::Vector3f cur_pose = {0, 0, 0};
      for (int i = 0; i < lanes_need_pred.size(); i++) {
        auto lane_ptr = lanes_need_pred[i];
        float cur_dist = PointToLaneDis(lane_ptr, cur_pose);
        if (cur_dist < nearest_lane_dist) {
          nearest_lane_dist = cur_dist;
          nearest_lane_index = i;
        }
      }
      if (nearest_lane_index != -1) {
        auto lane_ptr = lanes_need_pred[nearest_lane_index];
        predict_heading =
            (std::get<0>(lane_ptr->left_boundary->pred_end_heading) +
             std::get<0>(lane_ptr->right_boundary->pred_end_heading)) /
            2.0;
        HLOG_DEBUG << "heading mode 3:" << predict_heading << ","
                   << nearest_lane_dist;
        predict_heading = 0.8 * last_predict_angle + 0.2 * predict_heading;
      } else {
        // 求平均值
        double sum = 0;
        for (const auto& line : *lines_need_pred) {
          sum += std::get<0>(line->pred_end_heading);
        }
        predict_heading =
            lines_need_pred->empty() ? 0.0 : (sum / lines_need_pred->size());
        HLOG_DEBUG << "heading mode 4:" << predict_heading;
        predict_heading = 0.8 * last_predict_angle + 0.2 * predict_heading;
      }
    }
  }

  kappa = 0.4 * last_predict_kappa + 0.6 * kappa;
  dkappa = 0.4 * last_predict_dkappa + 0.6 * dkappa;
  // 对曲率kappa做一个阈值限制 曲率半径200m
  kappa = std::min(kappa, static_cast<double>(1.0 / 200.0));

  last_predict_angle = predict_heading;
  last_predict_kappa = kappa;
  last_predict_dkappa = dkappa;
  for (auto& line : *lines_need_pred) {
    line->pred_end_heading = std::make_tuple(predict_heading, kappa, dkappa);
  }
}

void LanePrediction::PredictLaneLine(std::vector<Lane::Ptr>* pred_lane,
                                     const Lane::Ptr curr_lane) {
  // 预测左边线
  Lane lane_pre;
  LineSegment left_bound;
  left_bound.id = curr_lane->left_boundary->id;
  left_bound.lanepos = curr_lane->left_boundary->lanepos;
  left_bound.type = LineType::LaneType_DASHED;
  left_bound.color = Color::WHITE;
  left_bound.isego = curr_lane->left_boundary->isego;
  left_bound.is_near_road_edge = curr_lane->left_boundary->is_near_road_edge;

  left_bound.mean_end_heading = curr_lane->left_boundary->mean_end_heading;
  left_bound.mean_end_heading_std_dev =
      curr_lane->left_boundary->mean_end_heading_std_dev;
  left_bound.mean_end_interval = curr_lane->left_boundary->mean_end_interval;
  for (const auto& delete_id : curr_lane->left_boundary->deteled_ids) {
    left_bound.deteled_ids.emplace_back(delete_id);
  }
  if (curr_lane->left_boundary != nullptr &&
      !curr_lane->left_boundary->pts.empty()) {
    Eigen::Vector2f back_pt = curr_lane->left_boundary->pts.back().pt.head<2>();
    auto mean_interval = curr_lane->left_boundary->mean_end_interval;
    float dist_to_veh = back_pt.norm();
    auto pred_length = conf_.predict_farthest_dist - dist_to_veh;
    auto pred_counts = static_cast<int>(pred_length / mean_interval);

    double left_kappa = std::get<1>(curr_lane->left_boundary->pred_end_heading);
    double left_heading =
        std::get<0>(curr_lane->left_boundary->pred_end_heading);
    double new_kappa = 0.0;

    for (int i = 0; i <= pred_counts; ++i) {
      new_kappa =
          left_kappa + std::get<2>(curr_lane->left_boundary->pred_end_heading);
      new_kappa = std::min(new_kappa, static_cast<double>(1.0 / 150.0));
      // new_kappa = left_kappa;
      if (math::DoubleHasSameSign(
              new_kappa,
              std::get<1>(curr_lane->left_boundary->pred_end_heading))) {
        left_kappa = new_kappa;
      }
      if (left_heading >= 0.) {
        left_heading += left_kappa;
      } else {
        left_heading -= left_kappa;
      }

      Eigen::Vector2f n(std::cos(left_heading), std::sin(left_heading));
      Eigen::Vector2f pt = back_pt + i * mean_interval * n;
      Point pred_pt;
      pred_pt.pt << pt.x(), pt.y(), 0;
      pred_pt.type = PointType::VIRTUAL;
      left_bound.pts.emplace_back(pred_pt);
    }
  }

  // 预测右边线
  LineSegment right_bound;
  right_bound.id = curr_lane->right_boundary->id;
  right_bound.lanepos = curr_lane->right_boundary->lanepos;
  right_bound.type = LineType::LaneType_DASHED;
  right_bound.color = Color::WHITE;
  right_bound.isego = curr_lane->right_boundary->isego;
  right_bound.is_near_road_edge = curr_lane->right_boundary->is_near_road_edge;
  right_bound.mean_end_heading = curr_lane->right_boundary->mean_end_heading;
  right_bound.mean_end_heading_std_dev =
      curr_lane->right_boundary->mean_end_heading_std_dev;
  right_bound.mean_end_interval = curr_lane->right_boundary->mean_end_interval;
  for (const auto& delete_id : curr_lane->right_boundary->deteled_ids) {
    right_bound.deteled_ids.emplace_back(delete_id);
  }
  if (curr_lane->right_boundary != nullptr &&
      !curr_lane->right_boundary->pts.empty()) {
    Eigen::Vector2f back_pt =
        curr_lane->right_boundary->pts.back().pt.head<2>();
    auto mean_interval = curr_lane->right_boundary->mean_end_interval;
    float dist_to_veh = back_pt.norm();
    auto pred_length = conf_.predict_farthest_dist - dist_to_veh;
    auto pred_counts = static_cast<int>(pred_length / mean_interval);

    double right_kappa =
        std::get<1>(curr_lane->right_boundary->pred_end_heading);
    double right_heading =
        std::get<0>(curr_lane->right_boundary->pred_end_heading);
    double new_kappa = 0.0;

    for (int i = 0; i <= pred_counts; ++i) {
      new_kappa = right_kappa +
                  std::get<2>(curr_lane->right_boundary->pred_end_heading);
      new_kappa = std::min(new_kappa, static_cast<double>(1.0 / 150.0));
      // new_kappa = right_kappa;
      if (math::DoubleHasSameSign(
              right_kappa,
              std::get<1>(curr_lane->right_boundary->pred_end_heading))) {
        right_kappa = new_kappa;
      }
      if (right_heading >= 0.) {
        right_heading += right_kappa;
      } else {
        right_heading -= right_kappa;
      }

      Eigen::Vector2f n(std::cos(right_heading), std::sin(right_heading));
      Eigen::Vector2f pt = back_pt + i * mean_interval * n;
      Point pred_pt;
      pred_pt.pt << pt.x(), pt.y(), 0;
      pred_pt.type = PointType::VIRTUAL;
      right_bound.pts.emplace_back(pred_pt);
    }
  }

  // 预测中心线
  std::vector<Point> ctr_pts;
  if (!curr_lane->center_line_pts.empty()) {
    Eigen::Vector2f back_pt = curr_lane->center_line_pts.back().pt.head<2>();
    auto mean_interval = (curr_lane->left_boundary->mean_end_interval +
                          curr_lane->right_boundary->mean_end_interval) /
                         2;
    float dist_to_veh = back_pt.norm();
    auto pred_length = conf_.predict_farthest_dist - dist_to_veh;
    auto pred_counts = static_cast<int>(pred_length / mean_interval);

    double center_kappa =
        (std::get<1>(curr_lane->left_boundary->pred_end_heading) +
         std::get<1>(curr_lane->right_boundary->pred_end_heading)) /
        2;
    double center_heading =
        (std::get<0>(curr_lane->left_boundary->pred_end_heading) +
         std::get<0>(curr_lane->right_boundary->pred_end_heading)) /
        2;
    double new_kappa = 0.0;

    for (int i = 0; i <= pred_counts; ++i) {
      new_kappa = center_kappa +
                  (std::get<2>(curr_lane->left_boundary->pred_end_heading) +
                   std::get<2>(curr_lane->right_boundary->pred_end_heading)) /
                      2;
      new_kappa = std::min(new_kappa, static_cast<double>(1.0 / 150.0));
      // new_kappa = center_kappa;
      if (math::DoubleHasSameSign(
              center_kappa,
              (std::get<1>(curr_lane->left_boundary->pred_end_heading) +
               std::get<1>(curr_lane->right_boundary->pred_end_heading)) /
                  2)) {
        center_kappa = new_kappa;
      }
      if (center_heading >= 0.) {
        center_heading += center_kappa;
      } else {
        center_heading -= center_kappa;
      }

      Eigen::Vector2f n(std::cos(center_heading), std::sin(center_heading));
      Eigen::Vector2f pt = back_pt + i * mean_interval * n;
      Point pred_pt;
      pred_pt.pt << pt.x(), pt.y(), 0;
      pred_pt.type = PointType::VIRTUAL;
      ctr_pts.emplace_back(pred_pt);
    }
  }

  lane_pre.str_id = curr_lane->str_id;
  lane_pre.lanepos_id = curr_lane->lanepos_id;
  size_t index = curr_lane->str_id_with_group.find(":");
  lane_pre.str_id_with_group =
      curr_lane->str_id_with_group.substr(0, index) + "P:" + lane_pre.str_id;
  lane_pre.left_boundary = std::make_shared<LineSegment>(left_bound);
  lane_pre.right_boundary = std::make_shared<LineSegment>(right_bound);
  lane_pre.center_line_param = curr_lane->center_line_param;
  lane_pre.center_line_param_front = curr_lane->center_line_param;
  lane_pre.center_line_pts = ctr_pts;
  // 添加预测lane的前后继
  lane_pre.prev_lane_str_id_with_group.emplace_back(
      curr_lane->str_id_with_group);
  curr_lane->next_lane_str_id_with_group.emplace_back(
      lane_pre.str_id_with_group);
  // 添加预测lane的左右邻
  std::vector<std::string> pre_str_id_with_group;
  for (const auto& str_id : curr_lane->left_lane_str_id_with_group) {
    if (!str_id.empty()) {
      size_t index = str_id.find(":");
      auto str_id_with_group =
          str_id.substr(0, index) + "P" + str_id.substr(index);
      pre_str_id_with_group.emplace_back(str_id_with_group);
    }
  }
  lane_pre.left_lane_str_id_with_group = pre_str_id_with_group;

  pre_str_id_with_group.clear();
  for (const auto& str_id : curr_lane->right_lane_str_id_with_group) {
    if (!str_id.empty()) {
      size_t index = str_id.find(":");
      auto str_id_with_group =
          str_id.substr(0, index) + "P" + str_id.substr(index);
      pre_str_id_with_group.emplace_back(str_id_with_group);
    }
  }
  lane_pre.right_lane_str_id_with_group = pre_str_id_with_group;
  if (lane_pre.left_boundary->pts.size() > 1 &&
      lane_pre.right_boundary->pts.size() > 1 &&
      lane_pre.center_line_pts.size() > 1) {
    pred_lane->emplace_back(std::make_shared<Lane>(lane_pre));
  }
}

float LanePrediction::PointToLaneDis(const Lane::Ptr& lane_ptr,
                                     Eigen::Vector3f point) {
  if (lane_ptr == nullptr || lane_ptr->left_boundary == nullptr ||
      lane_ptr->right_boundary == nullptr) {
    return std::numeric_limits<float>::max();
  }
  if (lane_ptr->left_boundary->pts.size() < 2 ||
      lane_ptr->right_boundary->pts.size() < 2) {
    return std::numeric_limits<float>::max();
  }
  float left_dist = 0;
  if (lane_ptr->left_boundary->pts.back().pt.x() < point.x()) {
    int size = lane_ptr->left_boundary->pts.size();
    left_dist =
        PointToVectorDist(lane_ptr->left_boundary->pts[size - 2].pt,
                          lane_ptr->left_boundary->pts[size - 1].pt, point);
  } else {
    left_dist = PointToVectorDist(lane_ptr->left_boundary->pts[0].pt,
                                  lane_ptr->left_boundary->pts[1].pt, point);
  }

  float right_dist = 0;
  if (lane_ptr->right_boundary->pts.back().pt.x() < point.x()) {
    int size = lane_ptr->right_boundary->pts.size();
    right_dist =
        PointToVectorDist(lane_ptr->right_boundary->pts[size - 2].pt,
                          lane_ptr->right_boundary->pts[size - 1].pt, point);
  } else {
    right_dist = PointToVectorDist(lane_ptr->right_boundary->pts[0].pt,
                                   lane_ptr->right_boundary->pts[1].pt, point);
  }
  HLOG_DEBUG << "left_dist:" << left_dist << ",right_dist:" << right_dist << ","
             << left_dist * right_dist;
  return left_dist * right_dist;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
