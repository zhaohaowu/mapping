/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： reloc.cc
 *   author     ： zhaohaowu/nihongjie
 *   date       ： 2024.04
 ******************************************************************************/

#include "modules/location/pose_estimation/lib/reloc/reloc.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Geometry/Transform.h"
#include "base/utils/log.h"
#include "modules/rviz/location_rviz.h"
#include "reloc/base.hpp"

namespace hozon {
namespace mp {
namespace loc {
namespace pe {

Reloc::Reloc()
    : step_size_(MatchParam::grid_size / MatchParam::search_scale),
      lat_steps_(static_cast<int>(
          std::round(MatchParam::max_search_lateral / step_size_))) {
  grid_idx_converter_ = std::make_shared<VoxelIndexConverter>(
      MatchParam::grid_size, MatchParam::grid_size);

  search_ys_.clear();

  for (int i = -lat_steps_; i <= lat_steps_; ++i) {
    double y = i * step_size_;
    search_ys_.emplace_back(y);
  }

  // lane level smm reloc heading cov
  lane_level_heading_std_ = 2.0 * M_PI / 180.0;
  odom_state_last_converge_ =
      std::make_shared<::hozon::localization::Localization>();
}

void Reloc::ResetStep(const double& max_search_lateral) {
  if (max_search_lateral <= 0 || step_size_ <= 0) {
    return;
  }
  lat_steps_ = static_cast<int>(std::round(max_search_lateral / step_size_));
  search_ys_.clear();

  for (int i = -lat_steps_; i <= lat_steps_; ++i) {
    double y = i * step_size_;
    search_ys_.emplace_back(y);
  }
}

void Reloc::InitMatch() {
  // reset map grids
  map_grids_.clear();
  map_type_nums_.clear();

  // reset percept data
  percept_line_types_.clear();
  vehicle_to_percept_line_dists_.clear();
  percept_type_nums_.clear();
  matched_type_nums_.clear();
  percept_frame_sources_.clear();
  percept_left_rs_id_ = -1;
  percept_right_rs_id_ = -1;
  percept_line_ave_heading_ = 0;
  percept_left_line_id_ = -1;
  percept_right_line_id_ = -1;

  // reset distance db
  map_dist_db_ = std::make_shared<ConsistencyDistDB>();
  percept_dist_db_ = std::make_shared<ConsistencyDistDB>();

  // search space related
  search_poses_.clear();

  // search related
  possible_match_pairs_vec_.clear();
  last_possible_match_pairs_vec_.clear();
  map_line_type_centroids_.clear();

  // search result related
  scores_predict_.clear();
  scores_measurement_.clear();
  thresholded_lateral_scores_.clear();
  lateral_peaks_.clear();

  // final result
  inlier_points_.clear();
  match_samples_.clear();
  d_heading_ = 0;
  d_lateral_ = 0;
  lateral_std_ = 0;
  heading_std_ = 0;
  multi_frame_match_.clear();
}

bool Reloc::ProcData(
    const std::shared_ptr<::hozon::localization::Localization>& localization,
    const std::shared_ptr<TrackingManager>& tracking_manager,
    const std::shared_ptr<MappingManager>& map_manager) {
  // 0. 初始化参数，清空数组
  InitMatch();

  // 1.过滤类型只有一种且感知和地图路沿宽度差异大于3m的数据，过滤感知或地图只有一根的数据
  if (!PreConditionCheck(tracking_manager, map_manager)) {
    HLOG_ERROR << "stamp " << tracking_manager->timestamp
               << " step1: width_diff between percep and map is more than 3m ";
    return false;
  }

  // 2.计算感知和地图的匹配得分 ----> scores_measurement_
  if (!LateralSpaceScoring(tracking_manager, map_manager)) {
    HLOG_ERROR << "stamp " << tracking_manager->timestamp
               << " step2: match_score between percep and map failed ";
    return false;
  }

  // 3. histogram filtering ----> d_heading_ d_lateral_
  if (!HistogramFiltering(localization)) {
    HLOG_ERROR << "stamp " << tracking_manager->timestamp
               << " step3: histogram filtering failed ";
    return false;
  }

  // 4. 类型匹配检查
  if (!PostConditionCheck(tracking_manager, map_manager)) {
    HLOG_ERROR << "stamp " << tracking_manager->timestamp
               << " step4: type_match between percep and map failed ";
    return false;
  }

  // 5. 计算重定位位姿
  ComputeRelocPose(localization);

  // 6. rviz可视化
  timespec cur_time{};
  clock_gettime(CLOCK_REALTIME, &cur_time);
  auto sec = cur_time.tv_sec;
  auto nsec = cur_time.tv_nsec;
  LOC_RVIZ->PubPerceptionMarkerReloc(T_w_v_, *tracking_manager, sec, nsec,
                                     "/pe/perception_marker_reloc");
  LOC_RVIZ->PubRelocOdom(T_w_v_, sec, nsec, "/pe/reloc_odom");

  return true;
}

bool Reloc::LateralSpaceScoring(
    const std::shared_ptr<TrackingManager>& tracking_manager,
    const std::shared_ptr<MappingManager>& map_manager) {
  // 计算所有感知车道线平均heading ----> percept_line_ave_heading_
  // 1.对较短车道线直线拟合，否则三次曲线拟合，如果三次曲线拟合残差较大，则直线拟合
  // 2.利用点数、长度和拟合残差计算拟合置信度，置信度大于0.3有效
  // 3.计算感知车道线的平均heading，主车道线权重为其他车道线的5倍
  if (!EstimatePerceptLanelineHeading(tracking_manager)) {
    HLOG_ERROR << "estimate laneline param failed!";
    return false;
  }
  // 构建地图栅格 ----> map_grids_
  // 1.将地图车道线点栅格化，如果相邻地图车道线点距离大于栅格，则对其升采样
  // 2.地图栅格哈希表的键为栅格索引，值为栅格内容，栅格内容哈希表的键为车道线类型，值为MapPoint数组，
  //   每个MapPoint存储了所属车道线id、heading和地图点坐标
  if (!ConstructMapGrids(map_manager)) {
    HLOG_ERROR << "construct map grids failed!";
    return false;
  }
  // 生成搜索位姿 ----> search_poses_
  // 以步长为0.6m，从-7.2m到7.2m遍历y，构建搜索点(0,y)，使用向量二分搜索，找到搜索点最近的两个MapPoint，
  // 取两个MapPoint的平均heading为地图点在伪车系下的heading，计算地图heading－感知平均heading的偏差h
  // 所以车系到伪车系的旋转矩阵为(cosh, -sinh, sinh, cosh)，平移向量为(0,y)
  if (!GenerateSearchPose()) {
    HLOG_ERROR << "generate search poses failed!";
    return false;
  }
  // 给所有搜索位姿打分 ----> scores_measurement_
  if (!ScoreAllPoses(tracking_manager)) {
    HLOG_ERROR << "score all poses failed!";
    return false;
  }
  return true;
}

bool Reloc::HistogramFiltering(
    const std::shared_ptr<::hozon::localization::Localization>& localization) {
  // 根据相对和绝对定位预测当前帧位姿得分的先验 ----> scores_predict_
  // last_localization_node_  has_prediction_  last_scores_posterior_
  // 1.预测步，利用全局位姿增量与局部位姿增量的偏差将上一帧车系到伪车系的变换矩阵转到当前帧下
  // 2.基于局部位姿增量调整位姿分数，局部位姿增量越大，每个车系到伪车系的位姿对应的得分越高
  if (!FilterPrediction(localization)) {
    HLOG_ERROR << "filter prediction failed!";
    return false;
  }
  // 将预测和观测融合获得当前帧位姿得分的后验 ----> last_scores_posterior_
  // 1.更新步，如果预测失败，融合结果等于观测结果；
  // 2.否则，计算上一帧经过预测的先验得分与当前帧的观测得分的KL散度，如果KL散度低于阈值，融合结果使用当前帧观测得分；
  // 3.否则，融合结果等于观测得分*预测得分^(1/KL散度)
  if (!FilterUpdate()) {
    HLOG_ERROR << "filter update failed!";
    return false;
  }
  // 检测滤波是否收敛 ---->d_lateral_ d_heading_
  // converge_travel_dist_ odom_state_last_converge_ lateral_peaks_
  // 1.单峰检测：按得分降序排序,使用变化最大的得分作为阈值，阈值得分上限为最高值的一半。挑选符合要求的得分，按照y从左到右排序，
  //   将相邻y距离小于0.9m的两个y按照得分加权平均，取最高得分输出一个波峰，如果有多个波峰，则没有收敛，重定位失败
  // 2.预测观察误差校验：当前帧先验最高分的y与融合最高分的y相差大于0.6m，则没有收敛，重定位失败；
  // 3.两帧重定位成功的时间间隔大于1s，则没有收敛，重定位失败
  // 4.如果前面某一步失败导致未收敛，需要连续重定位的距离大于5m满足重定位条件
  if (!FilterConvergenceCheck()) {
    HLOG_ERROR << "filter not converged yet!";
    return false;
  }
  return true;
}

bool Reloc::PreConditionCheck(
    const std::shared_ptr<TrackingManager>& tracking_manager,
    const std::shared_ptr<MappingManager>& map_manager) {
  std::unordered_set<LaneType> lane_types;
  for (const auto& lane_line : tracking_manager->lane_lines) {
    if (lane_line.second.lane_type != LaneType::UNKNOWN) {
      lane_types.emplace(lane_line.second.lane_type);
    }
  }
  int perception_type_num = static_cast<int>(lane_types.size());

  bool map_rs_valid = false;
  bool percept_rs_valid = false;
  double inconsist_dist = NAN;
  // 感知和地图至少要两根车道线，否则无效，并计算感知和地图道路宽度差异
  GetRoadsideDistanceCoarse(tracking_manager, map_manager, &percept_rs_valid,
                            &map_rs_valid, &inconsist_dist);

  // 条件1：有两种感知车道线类型，预检查成功
  if (perception_type_num >= 2) {
    return true;
  }

  // 条件2：有一种感知车道线类型，并且感知和地图均至少两根车道线，并且感知和地图宽度差异小于阈值3m
  if (perception_type_num >= 1) {
    // must have both roadside and pass consistency check
    if (map_rs_valid && percept_rs_valid &&
        inconsist_dist < MatchParam::max_prematch_rs_inconsist) {
      return true;
    }
    HLOG_ERROR << "percept/map roadside width inconsistent!";
  }

  HLOG_ERROR << "no precondition satisfied!";
  return false;
}

void Reloc::GetRoadsideDistanceCoarse(
    const std::shared_ptr<TrackingManager>& tracking_manager,
    const std::shared_ptr<MappingManager>& map_manager,
    bool* percept_roadside_valid, bool* map_roadside_valid,
    double* inconsist_distance) {
  // for perception
  auto percept_found = false;
  percept_left_rs_id_ = percept_right_rs_id_ = -1;
  // 计算感知道路宽度
  auto p_rs_dist = FindGoodLane(tracking_manager, &percept_found,
                                &percept_left_rs_id_, &percept_right_rs_id_);

  // for map
  auto map_found = false;
  map_left_rs_id_ = map_right_rs_id_ = -1;
  // 计算地图道路宽度
  auto m_rs_dist = FindGoodLane(map_manager, &map_found, &map_left_rs_id_,
                                &map_right_rs_id_);
  double inconsist_dist = std::fabs(p_rs_dist - m_rs_dist);
  // 计算感知和地图道路宽度差异
  *percept_roadside_valid = percept_found;
  *map_roadside_valid = map_found;
  *inconsist_distance = inconsist_dist;
}

bool Reloc::GenerateSearchPose() {
  // get map line points along lateral search line(x = 0)
  // 获取栅格索引x值为0的栅格内容，包括车道线类型和MapPoint数组，每个MapPoint存储了所属车道线id、heading和地图点坐标
  std::vector<MapPoint> map_grid_points;
  for (const auto& grid : map_grids_) {
    if (grid.first.x_idx != 0) {
      continue;
    }
    for (const auto& map_elemment : grid.second.map_elements) {
      if (!IsLaneLine(map_elemment.first)) {
        continue;
      }
      for (const auto& pt : map_elemment.second) {
        map_grid_points.emplace_back(pt);
      }
    }
  }
  if (map_grid_points.size() < 2) {
    return false;
  }

  // sort map grids in lateral direction
  // 从右到左排序
  std::sort(map_grid_points.begin(), map_grid_points.end(),
            [](const MapPoint& a, const MapPoint& b) {
              return a.point.y() < b.point.y();
            });

  for (const auto& y : search_ys_) {
    int left = static_cast<int>(map_grid_points.size()) - 1;
    int right = 0;
    if (y < map_grid_points.front().point.y()) {
      left = 0;
      right = -1;
    }
    if (y > map_grid_points.back().point.y()) {
      left = -1;
      right = static_cast<int>(map_grid_points.size()) - 1;
    }
    Eigen::Vector2d search_pt(0, y);
    Eigen::Vector2d direct =
        map_grid_points.back().point - map_grid_points.front().point;
    direct = direct / std::max(1e-9, direct.norm());
    double pt_proj = (search_pt - map_grid_points.front().point).dot(direct);
    while (left - right > 1) {
      int mid = (left + right) / 2;
      double proj = (map_grid_points[mid].point - map_grid_points.front().point)
                        .dot(direct);
      if (proj < pt_proj) {
        right = mid;
      } else {
        left = mid;
      }
    }
    double map_heading = 0;
    double cnt = 0;
    if (left != -1) {
      map_heading += map_grid_points[left].local_heading;
      ++cnt;
    }
    if (right != -1) {
      map_heading += map_grid_points[right].local_heading;
      ++cnt;
    }
    map_heading /= cnt;
    double heading_diff =
        NormalizeAngleDiff(map_heading - percept_line_ave_heading_);

    // limit aligned heading
    if (std::fabs(heading_diff) > MatchParam::max_align_heading) {
      double sign = heading_diff / std::fabs(heading_diff);
      heading_diff = sign * MatchParam::max_align_heading;
    }

    search_poses_.emplace_back(y, heading_diff);
  }
  return true;
}

bool Reloc::ConstructMapGrids(
    const std::shared_ptr<MappingManager>& map_manager) {
  auto UpdateGrid = [this](const VoxelIndex& index, const LaneType& grid_type,
                           const MapPoint& map_pt) {
    auto& grid = map_grids_[index];
    auto& element_ids = grid.map_elements[grid_type];
    element_ids.emplace_back(map_pt);
  };

  map_grids_.clear();
  // lane lines
  for (const auto& item : map_manager->lane_lines) {
    if (item.second.points.size() < 2) {
      continue;
    }
    int id = item.first;
    std::vector<Eigen::Vector3d> ll_pts = item.second.points;
    // estimate local heading
    std::vector<double> local_headings;
    double heading_x0 = 0;
    double dist_x0 = std::numeric_limits<double>::max();
    for (int i = 0; i < static_cast<int>(ll_pts.size()) - 1; ++i) {
      Eigen::Vector3d pi = ll_pts[i];
      Eigen::Vector3d pj = ll_pts[i + 1];
      if (pi.x() > pj.x()) {
        pi = ll_pts[i + 1];
        pj = ll_pts[i];
      }
      double heading_line =
          std::atan2(pj.y() - pi.y(), std::max(pj.x() - pi.x(), 1e-8));
      local_headings.emplace_back(heading_line);
      // 估计x=0附近的heading
      if (std::fabs(pi.x()) < dist_x0) {
        dist_x0 = std::fabs(pi.x());
        heading_x0 = heading_line;
      }
    }
    local_headings.emplace_back(local_headings.back());
    // reject map lines with large heading w.r.t. vehicle
    // 限制x=0附近的heading最大为30度
    if (std::fabs(heading_x0) > MatchParam::max_map_heading) {
      continue;
    }
    LaneType grid_type = item.second.lane_type;

    // upsample map points into continuous grid index
    std::unordered_set<VoxelIndex, VoxelIndexHash> index_added;
    for (size_t i = 0; i < ll_pts.size(); ++i) {
      Eigen::Vector2d pt(ll_pts[i].x(), ll_pts[i].y());
      // 将每个地图点栅格化，栅格大小为1.2m
      auto index = grid_idx_converter_->PointInnerToVoxelIndex(pt);
      if (index_added.count(index) != 0) {
        continue;
      }
      // 线的id、线上每个点、线上每个点对应的heading
      MapPoint map_pt(id, pt, local_headings[i]);
      // 栅格点、线类型、点坐标、线id、点对应的heading
      // 每个索引对应多个由类型和点数组的map对
      UpdateGrid(index, grid_type, map_pt);
      index_added.insert(index);
      if (i == 0) {
        continue;
      }

      // upsample points if map points distance is larger than grid size
      Eigen::Vector2d pt_last(ll_pts[i - 1].x(), ll_pts[i - 1].y());
      Eigen::Vector2d vec = pt - pt_last;
      double scale_x = 1.0;
      double scale_y = 1.0;
      int sample_num = 0;
      if (std::fabs(vec.x()) > std::fabs(vec.y())) {
        sample_num = static_cast<int>(std::round(vec.x()));
        scale_y = vec.y() / std::max(1e-9, vec.x());
      } else {
        sample_num = static_cast<int>(std::round(vec.y()));
        scale_x = vec.x() / std::max(1e-9, vec.y());
      }
      double heading_last = local_headings[i - 1];
      for (int j = 1; j <= sample_num; ++j) {
        double delta = j * MatchParam::grid_size;
        double sample_x = pt_last.x() + scale_x * delta;
        double sample_y = pt_last.y() + scale_y * delta;
        Eigen::Vector2d pt_sample(sample_x, sample_y);
        auto sample_index =
            grid_idx_converter_->PointInnerToVoxelIndex(pt_sample);
        if (index_added.count(sample_index) == 0) {
          MapPoint sample_map_pt(id, pt_sample, heading_last);
          UpdateGrid(sample_index, grid_type, sample_map_pt);
          index_added.insert(sample_index);
        }
      }
    }
  }
  return !map_grids_.empty();
}

bool Reloc::EstimatePerceptLanelineHeading(
    const std::shared_ptr<TrackingManager>& tracking_manager) {
  struct PolyLineParam {
    bool valid{};
    Eigen::VectorXd line_param;
    double heading{};
    double confidence{};
  };
  std::unordered_map<int, PolyLineParam> pecept_poly_line_params;

  // laneline process
  for (const auto& item : tracking_manager->lane_lines) {
    std::vector<Eigen::Vector3d> points = item.second.points;
    if (points.empty()) {
      HLOG_ERROR << "why percept line " << item.first << " is empty?";
      continue;
    }

    Eigen::VectorXd line_param(4, 1);
    double mean_residual = 0;
    double max_residual = 0;
    double line_length = (points.front() - points.back()).norm();
    bool has_fit = false;
    int order = 0;
    std::vector<double> weights(static_cast<int>(points.size()), 1);
    if (points.back().x() < 0 || points.front().x() > 2 || line_length < 5.0 ||
        points.size() < 10) {
      // line too short or far away from vehicle, straght line fitting
      order = 1;
      Eigen::VectorXd straight_line_param;
      // c3 c2 c1 c0顺序对应line_param[0] [1] [2] [3]
      has_fit = PolyLineLeastSquareFitting(points, weights, order,
                                           &straight_line_param, &mean_residual,
                                           &max_residual);
      if (has_fit) {
        line_param.setZero();
        line_param[2] = straight_line_param[0];
        line_param[3] = straight_line_param[1];
      }
    } else {
      // try cubic poly line fitting
      order = 3;
      has_fit = PolyLineLeastSquareFitting(points, weights, order, &line_param,
                                           &mean_residual, &max_residual);
      // downgrade to straght line fitting
      if (has_fit && (mean_residual > 0.2 && max_residual > 0.5)) {
        order = 1;
        Eigen::VectorXd straight_line_param;
        has_fit = PolyLineLeastSquareFitting(points, weights, order,
                                             &straight_line_param,
                                             &mean_residual, &max_residual);
        if (has_fit) {
          line_param.setZero();
          line_param[2] = straight_line_param[0];
          line_param[3] = straight_line_param[1];
        }
      }
    }
    PolyLineParam polyline_param;
    polyline_param.valid = has_fit;
    if (!has_fit) {
      polyline_param.confidence = 0;
      HLOG_ERROR << "percept line " << item.first
                 << " has invalid line fitting param";
    } else {
      double fit_confidence = std::exp(-mean_residual / 0.1);
      double len_confidence = std::min(10.0, line_length) / 10.0;
      double num_confidence =
          std::min(10.0, static_cast<double>(points.size())) / 10.0;
      polyline_param.line_param = line_param;
      polyline_param.confidence =
          fit_confidence * len_confidence * num_confidence;
      // line heading at x = 0
      double tanh = line_param[2];
      double heading_line = std::atan2(tanh, 1.0);
      polyline_param.heading = heading_line;
    }
    // 存放车道线id、是否有效、置信度、拟合系数
    pecept_poly_line_params.insert({item.first, polyline_param});
  }

  // check fitting quality, corse check
  int good_line_cnt = 0;
  for (const auto& param : pecept_poly_line_params) {
    if (param.second.confidence > 0.3) {
      ++good_line_cnt;
    }
  }
  if (good_line_cnt == 0) {
    HLOG_ERROR << "percept line quality bad!";
    return false;
  }

  // get average percept line headings
  double left_line_distance = 100;
  double right_line_distance = -100;
  int invalid = std::numeric_limits<int>::min();
  int left_id = invalid;
  int right_id = invalid;
  for (const auto& pecept_line_param : pecept_poly_line_params) {
    if (!pecept_line_param.second.valid) {
      continue;
    }
    int id = pecept_line_param.first;
    double dist = pecept_line_param.second.line_param[3];
    if (dist > 0 && dist < left_line_distance) {
      left_line_distance = dist;
      left_id = id;
    }
    if (dist < 0 && dist > right_line_distance) {
      right_line_distance = dist;
      right_id = id;
    }
  }

  percept_line_ave_heading_ = 0;
  double sum_confidence = 0;
  int sum_cnt = 0;
  for (const auto& pecept_line_param : pecept_poly_line_params) {
    const auto& param = pecept_line_param.second;
    if (!param.valid) {
      continue;
    }
    if (std::fabs(param.heading) > MatchParam::max_percept_heading) {
      continue;
    }
    int id = pecept_line_param.first;
    // increase weight for lines close to vehcle
    double weight = 1.0;
    if (id == left_id || id == right_id) {
      weight = 5.0;
    }
    percept_line_ave_heading_ += weight * param.confidence * param.heading;
    sum_confidence += weight * param.confidence;
    ++sum_cnt;
  }
  if (sum_cnt == 0) {
    return false;
  }
  // 计算所有车道线的平均heading
  percept_line_ave_heading_ /= sum_confidence;
  return true;
}

bool Reloc::ScoreAllPoses(
    const std::shared_ptr<TrackingManager>& tracking_manager) {
  scores_measurement_.clear();
  double min_score = std::numeric_limits<double>::max();
  double max_score = std::numeric_limits<double>::min();
  for (const auto pose : search_poses_) {
    double y = pose.first;
    double heading = pose.second;
    // 计算每个感知点平移y和旋转heading后，与地图点的差异得分
    double score = ScorePose(y, heading, tracking_manager);
    PoseScore pose_score{};
    pose_score.y = y;
    pose_score.heading = heading;
    pose_score.score = score;
    scores_measurement_.emplace_back(pose_score);
    min_score = std::min(min_score, score);
    if (max_score < score) {
      max_score = score;
      // 测试用
      // d_heading_ = heading;
      // d_lateral_ = y;
    }
  }
  // normalization
  double total_score = 0;
  for (auto& item : scores_measurement_) {
    item.score -= min_score;  // remove negative
    total_score += item.score;
  }
  if (total_score < 1e-8) {
    HLOG_ERROR << "no valid score";
    return false;
  }
  for (auto& item : scores_measurement_) {
    item.score /= total_score;
  }
  return true;
}

double Reloc::ScorePose(
    double y, double heading,
    const std::shared_ptr<TrackingManager>& tracking_manager,
    std::unordered_map<int, std::vector<int>>* hit_info) {
  static auto DTScore = [](double dist, double sigma) {
    return std::exp(-0.5 * dist * dist / (sigma * sigma));
  };
  double cosh = std::cos(heading);
  double sinh = std::sin(heading);
  Eigen::Matrix3d R_v1_v;
  R_v1_v << cosh, -sinh, 0, sinh, cosh, 0, 0, 0, 1;
  Eigen::Quaterniond q_v1_v(R_v1_v);
  Eigen::Vector3d t_v1_v(0, y, 0);
  Eigen::Affine3d T_v1_v =
      Eigen::Translation3d(t_v1_v) * Eigen::Affine3d(q_v1_v);
  double score = 0.0;
  std::unordered_map<int, std::vector<int>> hit_info_record;
  std::unordered_map<int, LaneLine> percept_points =
      tracking_manager->lane_lines;
  for (const auto& percept_elem : percept_points) {
    hit_info_record[percept_elem.first].reserve(
        percept_elem.second.points.size());
    double score_for_element = 0;
    // iterate through all points of a single peception element
    // 计算每个感知点平移y和旋转yaw后，与地图点的差异
    LaneType percept_type = percept_elem.second.lane_type;
    for (const auto& pt : percept_elem.second.points) {
      Eigen::Vector3d p_v{pt.x(), pt.y(), 0};
      Eigen::Vector3d p_v1 = T_v1_v * p_v;
      Eigen::Vector2d percept_pt(p_v1.x(), p_v1.y());
      int hit_map_id = -1;  // -1 is not map id
      double score_for_pt = 0;

      // point level scoring
      if (!IsLaneLine(percept_type)) {
        HLOG_ERROR << "percep type is not solid_line, dashed_line or road_edge";
        continue;
      }
      auto hit_index = grid_idx_converter_->PointInnerToVoxelIndex(percept_pt);
      auto local_idx_pattern = LocalIndexPattern(hit_index, percept_type);
      for (const auto& grid_idx : local_idx_pattern) {
        auto grid_iter = map_grids_.find(grid_idx);
        if (grid_iter == map_grids_.end()) {
          continue;  // no grid hit
        }
        const auto& map_elements = grid_iter->second.map_elements;
        auto grid_type_iter = map_elements.find(percept_type);
        if (grid_type_iter == map_elements.end()) {
          continue;  // no same type find
        }
        if (grid_type_iter->second.empty()) {
          continue;
        }

        const auto& map_pt = grid_type_iter->second.front().point;
        double dt_score =
            DTScore(percept_pt.y() - map_pt.y(), MatchParam::grid_size);
        // TODO(fy): weights are not used yet, as proning to overfit
        // to single line! need better cov/weight estimation
        score_for_pt = dt_score * 1.0;
        hit_map_id = grid_type_iter->second.front().id;
        break;
      }
      score_for_element += score_for_pt;
      if (hit_info != nullptr) {
        hit_info_record[percept_elem.first].emplace_back(hit_map_id);
      }
    }
    // TODO(fy): how to normalize for each element/semantic?
    score += score_for_element;
  }

  if (hit_info != nullptr) {
    *hit_info = std::move(hit_info_record);
  }
  return score;
}

std::vector<VoxelIndex> Reloc::LocalIndexPattern(const VoxelIndex& hit_index,
                                                 LaneType type) {
  std::vector<VoxelIndex> pattern;
  if (IsLaneLine(type)) {
    int lat_extend = MatchParam::lat_dt_extend;
    pattern.reserve(2 * lat_extend + 1);
    pattern.emplace_back(hit_index);
    for (int j = -lat_extend; j <= lat_extend; ++j) {
      if (j == 0) {
        continue;
      }
      VoxelIndex index(hit_index.x_idx, hit_index.y_idx + j);
      pattern.emplace_back(index);
    }
  } else {
    int long_extend = 2;
    int lat_extend = 1;
    int pattern_size = (2 * lat_extend + 1) * (2 * long_extend + 1);
    pattern.reserve(pattern_size);
    for (int i = -long_extend; i <= long_extend; ++i) {
      for (int j = -lat_extend; j <= lat_extend; ++j) {
        VoxelIndex index(hit_index.x_idx + i, hit_index.y_idx + j);
        pattern.emplace_back(index);
      }
    }
  }
  return pattern;
}

bool Reloc::FilterPrediction(
    const std::shared_ptr<::hozon::localization::Localization>& localization) {
  // check if need reset filter
  if (!last_localization_node_) {
    HLOG_ERROR << "first frame filter initial";
    last_localization_node_ = localization;
    return false;
  }
  if (!localization) {
    HLOG_ERROR << "fc is nullptr";
    return false;
  }
  // check if need reset filter
  hozon::mp::loc::fc::Node curr_global_node;
  hozon::mp::loc::fc::Node curr_dr_node;
  LocalizationToNode(*localization, &curr_global_node, &curr_dr_node);
  bool timestamp_large_gap =
      localization->header().data_stamp() -
          last_localization_node_->header().data_stamp() >
      0.5;

  if (last_scores_posterior_.empty() ||
      localization->header().data_stamp() <= 0 || timestamp_large_gap) {
    std::string reason;
    if (last_scores_posterior_.empty()) {
      reason = "last_scores_posterior_ is empty | ";
    }
    if (localization->header().data_stamp() <= 0) {
      reason += " fc data_stamp < 0 | ";
    }
    if (timestamp_large_gap) {
      reason += " time_diff between last frame and curr frame > 0.5s | ";
    }
    HLOG_ERROR << "reset histogram filter! reason: " << reason;
    last_localization_node_ = localization;
    has_prediction_ = false;
    scores_predict_.clear();
    return true;
  }

  hozon::mp::loc::fc::Node last_global_node;
  hozon::mp::loc::fc::Node last_dr_node;
  LocalizationToNode(*last_localization_node_, &last_global_node,
                     &last_dr_node);
  // grid center prediction
  auto delta_nav_state =
      Node2SE3(last_global_node).inverse() * Node2SE3(curr_global_node);
  auto delta_odom_state =
      Node2SE3(last_dr_node).inverse() * Node2SE3(curr_dr_node);
  auto delta_pose = delta_nav_state.inverse() * delta_odom_state;
  // grid center prediction

  double y_delta = delta_pose.translation()(1);
  double heading_delta = delta_pose.rotationMatrix().eulerAngles(0, 1, 2)[2];
  // 利用全局位姿增量与局部位姿增量的偏差将上一帧车系到伪车系的变换矩阵转到当前帧下
  scores_predict_ = std::move(last_scores_posterior_);
  for (auto& item : scores_predict_) {
    double heading_prior = item.heading + heading_delta;
    double y_prior = std::cos(item.heading) * y_delta + item.y;
    item.heading = heading_prior;
    item.y = y_prior;
  }

  // align to current search grids
  int max_step_num = static_cast<int>(search_ys_.size());
  std::vector<int> search_y_idx(max_step_num, -1);
  for (auto i = 0; i < static_cast<int>(scores_predict_.size()); ++i) {
    // quantize lateral position
    auto& item = scores_predict_[i];
    int step = static_cast<int>(std::round(item.y / step_size_));
    item.y = step * step_size_;
    // buffer index
    int y_idx = lat_steps_ + step;
    if (y_idx < 0 || y_idx >= max_step_num) {
      continue;
    }
    search_y_idx[y_idx] = i;
  }
  // 对齐操作
  std::vector<PoseScore> temp_lat_scores;
  temp_lat_scores.resize(max_step_num);
  for (int i = 0; i < max_step_num; ++i) {
    PoseScore predict_pose_score{};
    if (search_y_idx[i] == -1) {
      predict_pose_score.score = 0;
      predict_pose_score.y = (i - lat_steps_) * step_size_;
      predict_pose_score.heading = 0;  // not used
    } else {
      predict_pose_score = scores_predict_[search_y_idx[i]];
    }
    temp_lat_scores[i] = predict_pose_score;
  }
  scores_predict_ = temp_lat_scores;

  // grid uncertainty prediction, consider only lateral dimension
  // assume that uncertainty correlates with odom distance
  double sigma = 0.01 * delta_odom_state.translation().norm();
  double lower_bound = MatchParam::min_hf_predict_sigma;
  sigma = std::max(sigma, lower_bound);  // limit lower bound
  double sigma2 = sigma * sigma;
  double total_score = 0;
  for (int i = 0; i < max_step_num; ++i) {
    double pre_score = 0;
    const auto& item = temp_lat_scores[i];
    for (int j = 0; j < max_step_num; ++j) {
      const auto& item_other = temp_lat_scores[j];
      double dist = item.y - item_other.y;
      double exp = std::exp(-0.5 * dist * dist / sigma2);
      pre_score += exp * item_other.score;
    }
    scores_predict_[i].score = pre_score;
    total_score += pre_score;
  }

  // normalization
  if (total_score < 1e-8) {
    HLOG_ERROR << "reset histogram filter! reason: "
               << "no valid predict score";
    last_localization_node_ = localization;
    has_prediction_ = false;
    scores_predict_.clear();
    return true;
  }
  for (auto& item : scores_predict_) {
    item.score /= total_score;
  }

  last_localization_node_ = localization;
  has_prediction_ = true;
  return true;
}

bool Reloc::FilterUpdate() {
  static auto KLDivergence = [](const std::vector<PoseScore>& p,
                                const std::vector<PoseScore>& q,
                                double* kl_divergence) {
    if (p.size() != q.size()) {
      return false;
    }
    double result = 0;
    for (size_t i = 0; i < p.size(); ++i) {
      double pi = p[i].score;
      double qi = q[i].score;
      if (pi != 0) {
        result += pi * std::log(pi / qi);
      }
    }
    *kl_divergence = std::max(0.0, result);
    return true;
  };

  if (!has_prediction_) {
    HLOG_ERROR << "no prediction, using measurement as posterior directly!";
    last_scores_posterior_ = scores_measurement_;
    return true;
  }

  // evaluate kl divergence btw meansure and predict distribution
  double kl_divergence = 0.0;
  if (!KLDivergence(scores_measurement_, scores_predict_, &kl_divergence)) {
    HLOG_ERROR << "get kl divergence btw meansurement and prediction failed !";
    return false;
  }
  HLOG_DEBUG << "kl_divergence " << kl_divergence;
  if (kl_divergence < 1e-8) {
    HLOG_DEBUG << "kl divergence close to 0, using measurement as "
                  "posterior directly!";
    last_scores_posterior_ = scores_measurement_;
    return false;
  }

  last_scores_posterior_ = scores_measurement_;
  // add bias to reduce prediction weight
  double kl_inv = 1.0 / (kl_divergence + MatchParam::kl_divergence_bias);
  double total_score = 0.0;
  for (size_t i = 0; i < scores_measurement_.size(); ++i) {
    double p_measure = scores_measurement_[i].score;
    double p_predict = scores_predict_[i].score;
    double p_update = p_measure * std::pow(p_predict, kl_inv);
    last_scores_posterior_[i].score = p_update;
    total_score += p_update;

    if (MatchParam::enable_hf_heading_fusion) {
      double yaw_meas = scores_measurement_[i].heading;
      double yaw_pred = scores_predict_[i].heading;
      last_scores_posterior_[i].heading =
          (p_measure * yaw_meas + p_predict * yaw_pred) /
          (p_measure + p_predict);
    }
  }
  // normalization
  if (total_score < 1e-8) {
    HLOG_ERROR << "no valid score";
    return false;
  }
  for (auto& item : last_scores_posterior_) {
    item.score /= total_score;
  }
  return true;
}

bool Reloc::FilterConvergenceCheck() {
  if (!last_localization_node_) {
    return false;
  }
  if (!odom_state_last_converge_) {
    return false;
  }
  // 按得分降序排序；使用变化最大的得分作为阈值，阈值得分上限为最高值的一半；挑选符合要求的得分，按照y从左到右排序，
  // 将相邻y距离小于0.9m的两个y按照得分加权平均，取最高得分输出一个波峰，如果有多个波峰，则没有收敛，重定位失败；
  // lateral_peaks_ 波峰数组
  if (!HistogramPeakDetection()) {
    HLOG_ERROR << "1、single peak detection failed: 0 peak";
    converge_travel_dist_ = 0.0;
    if (odom_state_last_converge_) {
      odom_state_last_converge_->mutable_header()->set_data_stamp(0);
    }
    return false;
  }
  // check if filter converged to single peak
  if (lateral_peaks_.size() != 1) {
    HLOG_ERROR << "1、single peak detection failed: too many peaks"
               << lateral_peaks_.size();
    converge_travel_dist_ = 0.0;
    if (odom_state_last_converge_) {
      odom_state_last_converge_->mutable_header()->set_data_stamp(0);
    }
    return false;
  }

  // std::sort(
  //     lateral_peaks_.begin(), lateral_peaks_.end(),
  //     [](const PoseScore& a, const PoseScore& b) { return a.score > b.score;
  //     });
  d_lateral_ = lateral_peaks_.front().y;
  d_heading_ = lateral_peaks_.front().heading;

  // check converge timegap
  double timegap = last_localization_node_->header().data_stamp() -
                   odom_state_last_converge_->header().data_stamp();
  // 前后帧时间差超过1s
  if (timegap >= MatchParam::max_converge_timegap) {
    converge_travel_dist_ = 0.0;
    odom_state_last_converge_ = last_localization_node_;
    HLOG_ERROR << "time_diff between last frame and curr frame > 1s, is "
               << timegap << " s";
    return false;
  }

  // check error state
  // 预测观察误差校验：当前帧先验最高分的y与融合最高分的y相差大于0.9m，则没有收敛，重定位失败
  // if (!scores_predict_.empty()) {
  //   PoseScore max_prior{};
  //   for (auto& i : scores_predict_) {
  //     if (i.score > max_prior.score) {
  //       max_prior = i;
  //     }
  //   }
  //   double err_state_lateral = d_lateral_ - max_prior.y;
  //   if (std::fabs(err_state_lateral) > 0.9) {
  //     HLOG_ERROR << "2、diff between prediction and measure is to large, is "
  //            << std::fabs(err_state_lateral) << " m ";
  //     converge_travel_dist_ = 0.0;
  //     if (odom_state_last_converge_) {
  //       odom_state_last_converge_->mutable_header()->set_data_stamp(0);
  //     }
  //     return false;
  //   }
  // }

  // 如果前面某一步失败导致未收敛，需要连续重定位的距离大于5m满足重定位条件
  hozon::mp::loc::fc::Node last_odom_state_global_node;
  hozon::mp::loc::fc::Node last_odom_state_dr_node;
  LocalizationToNode(*odom_state_last_converge_, &last_odom_state_global_node,
                     &last_odom_state_dr_node);
  hozon::mp::loc::fc::Node last_global_node;
  hozon::mp::loc::fc::Node last_dr_node;
  LocalizationToNode(*last_localization_node_, &last_global_node,
                     &last_dr_node);

  // check converge distance
  auto delta_pose =
      Node2SE3(last_odom_state_dr_node).inverse() * Node2SE3(last_dr_node);
  converge_travel_dist_ += delta_pose.translation().norm();
  if (converge_travel_dist_ < MatchParam::min_converge_dist) {
    odom_state_last_converge_ = last_localization_node_;
    HLOG_ERROR << "reloc distance" << converge_travel_dist_ << " m";
    return false;
  }
  odom_state_last_converge_ = last_localization_node_;
  return true;
}

bool Reloc::HistogramPeakDetection() {
  // calculate sample score threshold
  if (last_scores_posterior_.empty()) {
    return false;
  }
  std::vector<PoseScore> lateral_scores_tmp = last_scores_posterior_;
  std::sort(
      lateral_scores_tmp.begin(), lateral_scores_tmp.end(),
      [](const PoseScore& a, const PoseScore& b) { return a.score > b.score; });
  double score_thres = 0;
  double max_gap = 0;
  size_t n = lateral_scores_tmp.size();
  for (size_t i = 1; i < n; ++i) {
    double score_prev = lateral_scores_tmp[i - 1].score;
    double score_curr = lateral_scores_tmp[i].score;
    double gap = score_prev - score_curr;
    if (gap > max_gap) {
      max_gap = gap;
      score_thres = score_curr;
    }
  }
  double score_median = n % 2 == 0 ? 0.5 * (lateral_scores_tmp[n / 2].score +
                                            lateral_scores_tmp[n / 2 - 1].score)
                                   : lateral_scores_tmp[n / 2].score;
  double score_max = lateral_scores_tmp.front().score;
  double upper_bound = score_max * MatchParam::max_peak_scale;
  score_thres = std::max(score_thres, score_median);  // limit lower bound
  score_thres = std::min(score_thres, upper_bound);   // limit upper bound

  // thresholding good samples
  thresholded_lateral_scores_.clear();
  for (const auto& item : lateral_scores_tmp) {
    if (item.score > score_thres) {
      thresholded_lateral_scores_.emplace_back(item);
    }
  }
  if (thresholded_lateral_scores_.empty()) {
    return false;
  }

  // clustering according to lateral position
  lateral_peaks_.clear();
  std::sort(thresholded_lateral_scores_.begin(),
            thresholded_lateral_scores_.end(),
            [](const PoseScore& a, const PoseScore& b) { return a.y > b.y; });
  std::vector<std::pair<int, int>> cluster_ids;
  int split_i = 0;
  for (auto j = 1; j < static_cast<int>(thresholded_lateral_scores_.size());
       ++j) {
    double delta_y =
        thresholded_lateral_scores_[j - 1].y - thresholded_lateral_scores_[j].y;
    if (delta_y < MatchParam::grid_size / MatchParam::search_scale * 1.5) {
      continue;
    }
    cluster_ids.emplace_back(split_i, j - 1);
    split_i = j;
  }
  cluster_ids.emplace_back(
      split_i, static_cast<int>(thresholded_lateral_scores_.size()) - 1);
  // get center of each cluster
  for (const auto& pair : cluster_ids) {
    if (pair.first > pair.second) {
      continue;
    }
    double cluster_width = thresholded_lateral_scores_[pair.first].y -
                           thresholded_lateral_scores_[pair.second].y;
    // cluster too wide
    if (cluster_width > MatchParam::max_cluster_width) {
      continue;
    }

    double y_mean = 0;
    double score_sum = 0;
    double best_score = 0;
    double best_heading = 0;
    for (int i = pair.first; i <= pair.second; ++i) {
      double score = thresholded_lateral_scores_[i].score;
      if (score > best_score) {
        best_score = score;
        best_heading = thresholded_lateral_scores_[i].heading;
      }
      y_mean += score * thresholded_lateral_scores_[i].y;
      score_sum += score;
    }
    y_mean /= score_sum;

    PoseScore pose_score{};
    pose_score.y = y_mean;
    pose_score.heading = best_heading;
    pose_score.score = best_score;
    lateral_peaks_.emplace_back(pose_score);
  }
  return !lateral_peaks_.empty();
}

bool Reloc::GetMatchSamples(
    const std::shared_ptr<TrackingManager>& tracking_manager) {
  // transform histogram peaks into match results
  match_samples_.clear();
  for (const auto& sample : lateral_peaks_) {
    MatchScore match_sample;
    match_sample.match = std::make_shared<MatchIndex>();
    match_sample.pose_sample = sample;
    // get match info
    int hit_pt_cnt = 0;
    int all_pt_cnt = 0;
    std::unordered_map<int, std::vector<int>> hit_info;
    ScorePose(match_sample.pose_sample.y, match_sample.pose_sample.heading,
              tracking_manager, &hit_info);
    if (hit_info.empty()) {
      HLOG_ERROR << "hit_info is empty";
      return false;
    }

    // convert hit info to match index
    for (const auto& item : hit_info) {
      int percept_id = item.first;
      if (item.second.empty()) {
        continue;
      }
      all_pt_cnt += static_cast<int>(item.second.size());

      if (tracking_manager->lane_lines.count(percept_id) == 0) {
        continue;
      }
      LaneType match_type =
          tracking_manager->lane_lines.at(percept_id).lane_type;
      SemanticType semantic_type = MatchTypeToSemanticType(match_type);

      // get match map ids, 1-to-n possible
      std::unordered_set<int> map_ids;  // remove redundant
      for (const auto& id : item.second) {
        if (id == -1) {
          continue;
        }
        ++hit_pt_cnt;
        map_ids.insert(id);
      }
      for (const auto& map_id : map_ids) {
        match_sample.match->AddSemanticMatch(
            semantic_type, std::make_pair(percept_id, map_id));
      }
    }
    if (all_pt_cnt == 0) {
      HLOG_ERROR << "why sampled percept points empty?";
      return false;
    }
    match_sample.hit_ratio =
        static_cast<double>(hit_pt_cnt) / static_cast<double>(all_pt_cnt);
    match_samples_.emplace_back(match_sample);
  }

  if (match_samples_.empty()) {
    HLOG_ERROR << "match_samples_ is empty";
    return false;
  }
  return true;
}

bool Reloc::PostConditionCheck(
    const std::shared_ptr<TrackingManager>& tracking_manager,
    const std::shared_ptr<MappingManager>& map_manager) {
  if (!GetMatchSamples(tracking_manager)) {
    HLOG_ERROR << "get match samples failed!";
    return false;
  }
  if (match_samples_.empty()) {
    HLOG_ERROR << "match_samples_.empty()";
    return false;
  }
  // matched perception element number statistic
  MatchIndex::Ptr match_index = match_samples_.front().match;
  if (match_index == nullptr) {
    HLOG_ERROR << "match_index is nullptr";
    return false;
  }

  MatchPairVec match_pairs = match_index->GetAllMatchInOneVec();
  if (match_pairs.empty()) {
    HLOG_ERROR << "match_pairs.empty()";
    return false;
  }

  matched_type_nums_.clear();
  std::vector<std::pair<int, int>> rs_matches;
  for (const auto& pair : match_pairs) {
    int p_id = pair.first;
    auto iter = tracking_manager->lane_lines.find(p_id);
    if (iter == tracking_manager->lane_lines.end()) {
      continue;
    }
    LaneType match_type = iter->second.lane_type;
    matched_type_nums_[match_type].insert(p_id);
    if (match_type == LaneType::Road_Edge) {
      rs_matches.emplace_back(pair);
    }
    HLOG_DEBUG << "match pair: " << p_id << " -> " << pair.second;
  }

  // get matched percept element number
  int match_rs_num =
      static_cast<int>(matched_type_nums_[LaneType::Road_Edge].size());
  int match_sl_num =
      static_cast<int>(matched_type_nums_[LaneType::SOLID_LINE].size());
  int match_dl_num =
      static_cast<int>(matched_type_nums_[LaneType::DASHED_LINE].size());
  HLOG_DEBUG << "match_rs_num " << match_rs_num;
  HLOG_DEBUG << "match_sl_num " << match_sl_num;
  HLOG_DEBUG << "match_dl_num " << match_dl_num;

  // get matched line style num
  int match_line_style_num = 0;
  if (match_rs_num != 0) {
    ++match_line_style_num;
  }
  if (match_sl_num != 0) {
    ++match_line_style_num;
  }
  if (match_dl_num != 0) {
    ++match_line_style_num;
  }

  // get total match elem num
  int total_match_elem_num = match_rs_num + match_sl_num + match_dl_num;
  for (const auto& lane_line : tracking_manager->lane_lines) {
    if (lane_line.second.lane_type == LaneType::Road_Edge) {
      percept_type_nums_[LaneType::Road_Edge]++;
    }
    if (lane_line.second.lane_type == LaneType::DASHED_LINE) {
      percept_type_nums_[LaneType::DASHED_LINE]++;
    }
    if (lane_line.second.lane_type == LaneType::SOLID_LINE) {
      percept_type_nums_[LaneType::SOLID_LINE]++;
    }
  }
  // get percept element number
  int percept_rs_num =
      static_cast<int>(percept_type_nums_[LaneType::Road_Edge]);
  int percept_ll_num =
      static_cast<int>(percept_type_nums_[LaneType::DASHED_LINE]) +
      static_cast<int>(percept_type_nums_[LaneType::DASHED_LINE]);
  int percept_line_num = percept_rs_num + percept_ll_num;
  HLOG_DEBUG << "m_rs_n " << match_rs_num << "/" << "m_sl_n " << match_sl_num
             << "/" << "m_dl_n " << match_dl_num;
  HLOG_DEBUG << "best reloc match with hit ratio "
             << match_samples_.front().hit_ratio;

  // check match percept pt ratio
  if (match_samples_.front().hit_ratio < 0.5) {
    HLOG_ERROR << "hit ratio too low";
    return false;
  }

  // consider check with matched pole/sign
  if (match_line_style_num >= 2 && total_match_elem_num >= 5) {
    return true;
  }

  // check if has 2 widest roadsides match
  HLOG_DEBUG << "percept_left_rs_id_ " << percept_left_rs_id_;
  HLOG_DEBUG << "percept_right_rs_id_ " << percept_right_rs_id_;
  bool two_roadside_matched = false;
  for (size_t i = 0; i < rs_matches.size(); ++i) {
    if (two_roadside_matched) {
      break;
    }
    int rs_i = rs_matches[i].first;
    int rs_map_i = rs_matches[i].second;
    for (size_t j = i + 1; j < rs_matches.size(); ++j) {
      int rs_j = rs_matches[j].first;
      int rs_map_j = rs_matches[j].second;
      bool widest_roadside_pair =
          (rs_i == percept_left_rs_id_ && rs_j == percept_right_rs_id_) ||
          (rs_i == percept_right_rs_id_ && rs_j == percept_left_rs_id_);
      if (!widest_roadside_pair) {
        continue;
      }
      double percept_rs_dist = 0;
      double map_rs_dist = 0;

      for (const auto& item : tracking_manager->lane_lines) {
        auto points = item.second.points;
        percept_dist_db_->AddElement(item.first, SemanticType::LaneLine,
                                     points);
      }
      for (const auto& item : map_manager->lane_lines) {
        auto points = item.second.points;
        map_dist_db_->AddElement(item.first, SemanticType::LaneLine, points);
      }
      if (!percept_dist_db_->GetDistBetweenElements(rs_i, rs_j,
                                                    &percept_rs_dist)) {
        continue;
      }
      if (!map_dist_db_->GetDistBetweenElements(rs_map_i, rs_map_j,
                                                &map_rs_dist)) {
        continue;
      }
      HLOG_DEBUG << "percept roadside " << rs_i << "/" << rs_j << " dist "
                 << percept_rs_dist;
      HLOG_DEBUG << "map roadside " << rs_map_i << "/" << rs_map_j << " dist"
                 << map_rs_dist;
      if (std::fabs(percept_rs_dist - map_rs_dist) <
          MatchParam::max_match_rs_inconsist) {
        two_roadside_matched = true;
        break;
      }
    }
  }

  // check matched line number
  // 1. at least 3 line match, 1 roadside match
  int match_line_num = match_rs_num + match_sl_num + match_dl_num;
  if (percept_line_num > 2 && match_line_num <= 2) {
    HLOG_ERROR << "too few line matched!";
    return false;
  }

  // 2. if 2 roadside matched, max unmatched line num is 2
  int unmatch_line_num = percept_line_num - match_line_num;
  if (two_roadside_matched && unmatch_line_num > 2) {
    HLOG_ERROR
        << "2 roadside matched, but still has too many unmatched lines !";
    return false;
  }
  for (const auto& lane_line : map_manager->lane_lines) {
    if (lane_line.second.lane_type == LaneType::Road_Edge) {
      map_type_nums_[MatchSemanticType::RoadSideLine]++;
    }
    if (lane_line.second.lane_type == LaneType::DASHED_LINE) {
      map_type_nums_[MatchSemanticType::DashedLine]++;
    }
    if (lane_line.second.lane_type == LaneType::SOLID_LINE) {
      map_type_nums_[MatchSemanticType::SolidLine]++;
    }
  }

  // 3. if only 1 roadside match
  if (!two_roadside_matched) {
    // matched lines should have different style
    int map_sl_num =
        static_cast<int>(map_type_nums_[MatchSemanticType::SolidLine]);
    int map_dl_num =
        static_cast<int>(map_type_nums_[MatchSemanticType::DashedLine]);
    int map_rs_num =
        static_cast<int>(map_type_nums_[MatchSemanticType::RoadSideLine]);
    if ((match_sl_num == 0 && map_sl_num != 0) ||
        (match_dl_num == 0 && map_dl_num != 0)) {
      HLOG_ERROR << "only single line style, match ambiguous possible!";
      return false;
    }
    // max unmatched line num is 1
    if (unmatch_line_num > 1) {
      HLOG_ERROR << "too many unmatched lines!";
      return false;
    }
    // other roadside should be matched
    int unmatch_percept_rs_num = percept_rs_num - match_rs_num;
    int unmatch_map_rs_num = map_rs_num - match_rs_num;
    if (unmatch_percept_rs_num != 0 && unmatch_map_rs_num != 0) {
      HLOG_ERROR << "unmached roadside is not allowed!";
      HLOG_ERROR << "percept_rs_num " << percept_rs_num << " map_rs_num "
                 << map_rs_num << " match_rs_num " << match_rs_num;
      return false;
    }
  }
  return true;
}

void Reloc::GetFinalMatchIndex() {
  // all check pass, split to multi camera match result
  MatchIndex::Ptr match_index = match_samples_.front().match;
  if (match_index == nullptr) {
    return;
  }
  auto multi_semantic_match_pairs = match_index->GetAllSemanticMatch();
  for (const auto& item : multi_semantic_match_pairs) {
    SemanticType semantic_type = item.first;
    const MatchPairVec& match_pairs = item.second;
    for (const auto& pair : match_pairs) {
      int percept_id = pair.first;
      int map_id = pair.second;
      HLOG_DEBUG << "smm reloc final match pairs, percept_id: " << percept_id
                 << ", map_id: " << map_id;
      if (percept_frame_sources_.count(percept_id) == 0) {
        continue;
      }
      auto cam_name = percept_frame_sources_.at(percept_id);
      if (multi_frame_match_.count(cam_name) == 0) {
        multi_frame_match_[cam_name] = std::make_shared<MatchIndex>();
      }
      multi_frame_match_[cam_name]->AddSemanticMatch(semantic_type, pair);
    }
  }
}

Sophus::SE3d Reloc::Node2SE3(const hozon::mp::loc::fc::Node& node) {
  return {Sophus::SO3d::exp(node.orientation), node.enu};
}

Sophus::SE3d Reloc::Node2SE3(
    const std::shared_ptr<hozon::mp::loc::fc::Node>& node) {
  return Node2SE3(*node);
}

void Reloc::LocalizationToNode(
    const ::hozon::localization::Localization& localization,
    hozon::mp::loc::fc::Node* const global_pose,
    hozon::mp::loc::fc::Node* const dr_pose) {
  Point3dToVector3d(localization.pose().angular_velocity(),
                    &global_pose->angular_velocity);
  Point3dToVector3d(localization.pose().linear_velocity(),
                    &global_pose->velocity);
  Point3dToVector3d(localization.pose().linear_acceleration(),
                    &global_pose->linear_accel);
  Point3dToVector3d(localization.pose().gcj02(), &global_pose->blh);
  Point3dToVector3d(localization.pose().position(), &global_pose->enu);
  Point3dToVector3d(localization.pose().euler_angle(),
                    &global_pose->orientation);

  global_pose->seq = localization.header().seq();
  global_pose->ticktime = localization.header().data_stamp();
  global_pose->heading = localization.pose().heading();

  Point3dToVector3d(localization.pose_local().angular_velocity(),
                    &dr_pose->angular_velocity);
  Point3dToVector3d(localization.pose_local().linear_velocity(),
                    &dr_pose->velocity);
  Point3dToVector3d(localization.pose_local().linear_acceleration(),
                    &dr_pose->linear_accel);
  Point3dToVector3d(localization.pose_local().gcj02(), &dr_pose->blh);
  Point3dToVector3d(localization.pose_local().position(), &dr_pose->enu);
  Point3dToVector3d(localization.pose_local().euler_angle(),
                    &dr_pose->orientation);

  dr_pose->seq = localization.header().seq();
  dr_pose->ticktime = localization.header().data_stamp();
  dr_pose->heading = localization.pose_local().heading();
}

template <typename T>
void Reloc::Point3dToVector3d(const T& enu, Eigen::Vector3d* const vector) {
  *vector = {enu.x(), enu.y(), enu.z()};
}

template <typename T>
double Reloc::FindGoodLane(const std::shared_ptr<T>& manager,
                           bool* const is_found, int* const left_rs_id,
                           int* const right_rs_id) {
  double left_rs_dist = -1e10;
  double right_rs_dist = 1e10;
  for (const auto& item : manager->lane_lines) {
    if (item.second.points.empty()) {
      continue;
    }
    double y_mean = 0;
    for (const auto& pt : item.second.points) {
      y_mean += pt.y();
    }
    y_mean /= static_cast<int>(item.second.points.size());
    // get roadside id
    if (y_mean > 0 && y_mean > left_rs_dist) {
      left_rs_dist = y_mean;
      *left_rs_id = item.first;
    }
    if (y_mean < 0 && y_mean < right_rs_dist) {
      right_rs_dist = y_mean;
      *right_rs_id = item.first;
    }
  }
  *is_found =
      (*left_rs_id != -1 && *right_rs_id != -1 && *left_rs_id != *right_rs_id);
  return left_rs_dist - right_rs_dist;
}

void Reloc::ComputeRelocPose(
    const std::shared_ptr<::hozon::localization::Localization>& localization) {
  if (!localization) {
    return;
  }
  // d_lateral_和d_heading_转换为变换矩阵T_v1_v
  Eigen::Matrix4d T_v1_v;
  T_v1_v << cos(d_heading_), -sin(d_heading_), 0, 0, sin(d_heading_),
      cos(d_heading_), 0, d_lateral_, 0, 0, 1, 0, 0, 0, 0, 1;
  // localization转为T_w_v1
  Eigen::Matrix4d T_w_v1 = Eigen::Matrix4d::Identity();
  T_w_v1.topRightCorner(3, 1) << localization->pose().position().x(),
      localization->pose().position().y(), localization->pose().position().z();
  Eigen::Quaterniond q_w_v1(localization->pose().quaternion().w(),
                            localization->pose().quaternion().x(),
                            localization->pose().quaternion().y(),
                            localization->pose().quaternion().z());
  q_w_v1.normalize();
  T_w_v1.topLeftCorner(3, 3) = q_w_v1.toRotationMatrix();
  // T_w_v1和T_v1_v转为T_w_v，重定位位姿
  T_w_v_ = Eigen::Affine3d(T_w_v1 * T_v1_v);
}

}  // namespace pe
}  // namespace loc
}  // namespace mp
}  // namespace hozon
