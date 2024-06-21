/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： calc_util.cc
 *   author     ： taoshaoyuan
 *   date       ： 2023.10
 ******************************************************************************/

#include "map_fusion/fusion_common/calc_util.h"
#include <depend/common/math/double_type.h>

#include <thread>

namespace hozon {
namespace mp {
namespace mf {

bool InCubicRange(float x, const em::LineCubic& cubic) {
  return x >= cubic.start && x <= cubic.end;
}

void SamplingCubic(const em::LineCubic& cubic, float step,
                   std::vector<em::Point>* pts) {
  if (pts == nullptr) return;

  pts->clear();
  float x = cubic.start;
  while (x < cubic.end) {
    float y = CubicResolve(cubic.c3, cubic.c2, cubic.c1, cubic.c0, x);
    em::Point pt(x, y, 0);
    pts->emplace_back(pt);

    x += step;
  }
  x = cubic.end;
  float end_y = CubicResolve(cubic.c3, cubic.c2, cubic.c1, cubic.c0, x);
  pts->emplace_back(em::Point(x, end_y, 0));
}

double NowInSec() {
  timespec ts{};
  clock_gettime(CLOCK_REALTIME, &ts);
  double secs =
      static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
  return secs;
}

void SplitSeconds(double secs, uint32_t* sec, uint32_t* nsec) {
  if (sec == nullptr || nsec == nullptr) {
    return;
  }
  if (secs < 0) {
    return;
  }
  auto s = static_cast<uint32_t>(secs);
  auto ns = static_cast<uint32_t>((secs - s) * 1e9);
  *sec = s;
  *nsec = ns;
}

void InterpPose(const Pose& T_a_0, const Pose& T_a_1, const Pose& T_b_0,
                Pose* T_b_1) {
  if (T_b_1 == nullptr) {
    return;
  }

  Eigen::Isometry3f T_fa_0;
  T_fa_0.setIdentity();
  T_fa_0.rotate(T_a_0.quat);
  T_fa_0.pretranslate(T_a_0.pos);
  Eigen::Isometry3f T_fa_1;
  T_fa_1.setIdentity();
  T_fa_1.rotate(T_a_1.quat);
  T_fa_1.pretranslate(T_a_1.pos);

  Eigen::Isometry3f T_fb_0;
  T_fb_0.setIdentity();
  T_fb_0.rotate(T_b_0.quat);
  T_fb_0.pretranslate(T_b_0.pos);
  Eigen::Isometry3f T_fb_1;
  T_fb_1.setIdentity();
  T_fb_1 = T_fb_0 * T_fa_0.inverse() * T_fa_1;

  T_b_1->quat = T_fb_1.rotation();
  T_b_1->pos = T_fb_1.translation();
}

int PoseQueue::CheckStamp(double stamp) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (queue_.empty()) {
    return 2;
  }
  if (stamp < queue_.front().stamp) {
    return -1;
  }
  if (stamp > queue_.back().stamp) {
    return 1;
  }
  return 0;
}

int PoseQueue::Interp(double stamp, Pose* pose) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (pose == nullptr) {
    return -1;
  }

  auto cmp = [](const Pose& x, const Pose& y) -> bool {
    return x.stamp < y.stamp;
  };
  Pose tmp;
  tmp.stamp = stamp;
  auto it = std::lower_bound(queue_.begin(), queue_.end(), tmp, cmp);
  if (it == queue_.end()) {
    return -1;
  }

  if (it == queue_.begin() && it->stamp > stamp) {
    return -1;
  }

  if (it->stamp == stamp) {
    *pose = *it;
    return 0;
  }

  auto it_prev = std::prev(it);

  auto ratio = (stamp - it_prev->stamp) / (it->stamp - it_prev->stamp);

  pose->stamp = stamp;
  if (ratio < 1e-3) {
    pose->pos = it_prev->pos;
    pose->quat = it_prev->quat;
  } else if ((1 - ratio) < 1e-3) {
    pose->pos = it->pos;
    pose->quat = it->quat;
  } else {
    pose->pos = (1 - ratio) * it_prev->pos + ratio * it->pos;
    pose->quat = it_prev->quat.slerp(ratio, it->quat);
  }

  return 0;
}

Rate::Rate(double hz) : expected_cycle_time_(1. / hz), actual_cycle_time_(0.) {
  start_ = NowInSec();
}

Rate::Rate(uint64_t us)
    : expected_cycle_time_(static_cast<double>(us) * 1e-6),
      actual_cycle_time_(0.) {
  start_ = NowInSec();
}

bool Rate::Sleep() {
  double expected_end = start_ + expected_cycle_time_;
  double actual_end = NowInSec();
  double sleep_time = expected_end - actual_end;
  actual_cycle_time_ = actual_end - start_;
  start_ = expected_end;
  if (sleep_time <= 0.) {
    if (actual_end > expected_end + expected_cycle_time_) {
      start_ = actual_end;
    }
    return false;
  }
  std::this_thread::sleep_for(
      std::chrono::nanoseconds(static_cast<int64_t>(sleep_time * 1e9)));
  return true;
}

void Rate::Reset() { start_ = NowInSec(); }

double Rate::CycleTime() const { return actual_cycle_time_; }

double Rate::ExpectedCycleTime() const { return expected_cycle_time_; }

void TransformMap(const em::ElementMap& map_source,
                  const Pose& T_source_to_target,
                  const std::string& target_frame_id,
                  em::ElementMap* map_target) {
  if (map_target == nullptr) {
    return;
  }
  map_target->map_info = map_source.map_info;
  map_target->map_info.frame_id = target_frame_id;

  for (const auto& it : map_source.arrows) {
    if (it.second == nullptr) {
      continue;
    }
    const auto& arrow_source = it.second;
    auto arrow_target = std::make_shared<em::Arrow>();
    arrow_target->id = arrow_source->id;
    arrow_target->type = arrow_source->type;
    arrow_target->lane_id = arrow_source->lane_id;
    arrow_target->polygon.points.reserve(arrow_source->polygon.points.size());
    for (const auto& pt : arrow_source->polygon.points) {
      auto ptt = T_source_to_target.TransformPoint(pt);
      arrow_target->polygon.points.emplace_back(ptt);
    }

    map_target->arrows.insert({it.first, arrow_target});
  }

  for (const auto& it : map_source.boundary_nodes) {
    if (it.second == nullptr) {
      continue;
    }
    const auto& node_source = it.second;
    auto node_target = std::make_shared<em::BoundaryNode>();
    node_target->id = node_source->id;
    node_target->dash_seg = node_source->dash_seg;
    node_target->point = T_source_to_target.TransformPoint(node_source->point);

    map_target->boundary_nodes.insert({it.first, node_target});
  }

  for (const auto& it : map_source.boundaries) {
    if (it.second == nullptr) {
      continue;
    }
    const auto& bound_source = it.second;
    auto bound_target = std::make_shared<em::Boundary>();
    bound_target->id = bound_source->id;
    bound_target->color = bound_source->color;
    bound_target->boundary_type = bound_source->boundary_type;
    bound_target->prev_boundary_ids.insert(
        bound_target->prev_boundary_ids.end(),
        bound_source->prev_boundary_ids.begin(),
        bound_source->prev_boundary_ids.end());
    bound_target->next_boundary_ids.insert(
        bound_target->next_boundary_ids.end(),
        bound_source->next_boundary_ids.begin(),
        bound_source->next_boundary_ids.end());
    for (const auto& node : bound_source->nodes) {
      if (node == nullptr) {
        continue;
      }
      if (map_target->boundary_nodes.find(node->id) ==
          map_target->boundary_nodes.end()) {
        continue;
      }
      bound_target->nodes.emplace_back(map_target->boundary_nodes[node->id]);
    }

    map_target->boundaries.insert({it.first, bound_target});
  }

  for (const auto& it : map_source.center_lines) {
    if (it.second == nullptr) {
      continue;
    }
    const auto& center_source = it.second;
    auto center_target = std::make_shared<em::CenterLine>();
    center_target->id = center_source->id;
    center_target->lane_id = center_source->lane_id;
    center_target->attr = center_source->attr;
    center_target->points.reserve(center_source->points.size());
    for (const auto& pt : center_source->points) {
      auto ptt = T_source_to_target.TransformPoint(pt);
      center_target->points.emplace_back(ptt);
    }

    map_target->center_lines.insert({it.first, center_target});
  }

  for (const auto& it : map_source.cross_walks) {
    if (it.second == nullptr) {
      continue;
    }
    const auto& cross_source = it.second;
    auto cross_target = std::make_shared<em::CrossWalk>();
    cross_target->id = cross_source->id;
    cross_target->polygon.points.reserve(cross_source->polygon.points.size());
    for (const auto& pt : cross_source->polygon.points) {
      auto ptt = T_source_to_target.TransformPoint(pt);
      cross_target->polygon.points.emplace_back(ptt);
    }

    map_target->cross_walks.insert({it.first, cross_target});
  }

  for (const auto& it : map_source.lanes) {
    if (it.second == nullptr) {
      continue;
    }
    const auto& lane_source = it.second;
    auto lane_target = std::make_shared<em::Lane>();
    lane_target->id = lane_source->id;
    lane_target->lane_type = lane_source->lane_type;
    lane_target->turn_type = lane_source->turn_type;
    lane_target->lane_status = lane_source->lane_status;
    lane_target->length = lane_source->length;
    lane_target->center_line_id = lane_source->center_line_id;
    lane_target->prev_lane_ids.insert(lane_target->prev_lane_ids.end(),
                                      lane_source->prev_lane_ids.begin(),
                                      lane_source->prev_lane_ids.end());
    lane_target->next_lane_ids.insert(lane_target->next_lane_ids.end(),
                                      lane_source->next_lane_ids.begin(),
                                      lane_source->next_lane_ids.end());
    lane_target->left_forward_lane_ids.insert(
        lane_target->left_forward_lane_ids.end(),
        lane_source->left_forward_lane_ids.begin(),
        lane_source->left_forward_lane_ids.end());
    lane_target->right_forward_lane_ids.insert(
        lane_target->right_forward_lane_ids.end(),
        lane_source->right_forward_lane_ids.begin(),
        lane_source->right_forward_lane_ids.end());
    lane_target->left_reverse_lane_ids.insert(
        lane_target->left_reverse_lane_ids.end(),
        lane_source->left_reverse_lane_ids.begin(),
        lane_source->left_reverse_lane_ids.end());
    lane_target->right_reverse_lane_ids.insert(
        lane_target->right_reverse_lane_ids.end(),
        lane_source->right_reverse_lane_ids.begin(),
        lane_source->right_reverse_lane_ids.end());
    lane_target->left_boundary_ids.insert(
        lane_target->left_boundary_ids.end(),
        lane_source->left_boundary_ids.begin(),
        lane_source->left_boundary_ids.end());
    lane_target->right_boundary_ids.insert(
        lane_target->right_boundary_ids.end(),
        lane_source->right_boundary_ids.begin(),
        lane_source->right_boundary_ids.end());

    map_target->lanes.insert({it.first, lane_target});
  }

  for (const auto& it : map_source.roads) {
    if (it.second == nullptr) {
      continue;
    }
    const auto& road_source = it.second;
    auto road_target = std::make_shared<em::Road>();
    road_target->id = road_source->id;
    road_target->road_type = road_source->road_type;
    road_target->scene_type = road_source->scene_type;
    road_target->lane_ids.insert(road_target->lane_ids.end(),
                                 road_source->lane_ids.begin(),
                                 road_source->lane_ids.end());
    road_target->prev_road_ids.insert(road_target->prev_road_ids.end(),
                                      road_source->prev_road_ids.begin(),
                                      road_source->prev_road_ids.end());
    road_target->next_road_ids.insert(road_target->next_road_ids.end(),
                                      road_source->next_road_ids.begin(),
                                      road_source->next_road_ids.end());
    road_target->left_boundary_id = road_source->left_boundary_id;
    road_target->right_boundary_id = road_source->right_boundary_id;

    map_target->roads.insert({it.first, road_target});
  }

  for (const auto& it : map_source.stop_lines) {
    if (it.second == nullptr) {
      continue;
    }
    const auto& stop_source = it.second;
    auto stop_target = std::make_shared<em::StopLine>();
    stop_target->id = stop_source->id;
    stop_target->lane_ids.insert(stop_target->lane_ids.end(),
                                 stop_source->lane_ids.begin(),
                                 stop_source->lane_ids.end());
    stop_target->points.reserve(stop_source->points.size());
    for (const auto& pt : stop_source->points) {
      auto ptt = T_source_to_target.TransformPoint(pt);
      stop_target->points.emplace_back(ptt);
    }

    map_target->stop_lines.insert({it.first, stop_target});
  }

  for (const auto& it : map_source.symbols) {
    if (it.second == nullptr) {
      continue;
    }
    const auto& symbol_source = it.second;
    auto symbol_target = std::make_shared<em::Symbol>();
    symbol_target->id = symbol_source->id;
    symbol_target->symbol_type = symbol_source->symbol_type;
    symbol_target->lane_id = symbol_source->lane_id;
    symbol_target->polygon.points.reserve(symbol_source->polygon.points.size());
    for (const auto& pt : symbol_source->polygon.points) {
      auto ptt = T_source_to_target.TransformPoint(pt);
      symbol_target->polygon.points.emplace_back(ptt);
    }

    map_target->symbols.insert({it.first, symbol_target});
  }

  for (const auto& it : map_source.traffic_lights) {
    if (it.second == nullptr) {
      continue;
    }
    const auto& light_source = it.second;
    auto light_target = std::make_shared<em::TrafficLight>();
    light_target->id = light_source->id;
    light_target->traffic_light_type = light_source->traffic_light_type;
    light_target->point =
        T_source_to_target.TransformPoint(light_source->point);
    light_target->lane_ids.insert(light_target->lane_ids.end(),
                                  light_source->lane_ids.begin(),
                                  light_source->lane_ids.end());

    map_target->traffic_lights.insert({it.first, light_target});
  }
}

namespace math {

void GenerateCenterPoint(
    const std::vector<hozon::common::math::Vec2d>& left_point,
    const std::vector<hozon::common::math::Vec2d>& right_point,
    std::vector<hozon::common::math::Vec2d>* cent_point) {
  if (left_point.size() >= 2 && right_point.size() >= 2) {
    if (left_point.size() >= right_point.size()) {
      CenterPoint(left_point, right_point, cent_point);
    } else {
      CenterPoint(right_point, left_point, cent_point);
    }
  } else {
    HLOG_ERROR << "point size error";
  }
}

void CenterPoint(const std::vector<hozon::common::math::Vec2d>& project_points,
                 const std::vector<hozon::common::math::Vec2d>& points,
                 std::vector<hozon::common::math::Vec2d>* cent_point) {
  cent_point->emplace_back(
      (project_points.front().x() + points.front().x()) / 2,
      (project_points.front().y() + points.front().y()) / 2);
  for (int i = 1; i < project_points.size() - 1; ++i) {
    bool found(false);
    int point_count(0);
    double lambda(0.0);
    while (!found) {
      if (point_count + 1 < points.size()) {
        if (CalculateLambda(project_points[i], project_points[i + 1],
                            points[point_count], points[point_count + 1],
                            &lambda)) {
          found = true;
          break;
        }
      } else {
        break;
      }
      ++point_count;
    }
    if (found) {
      hozon::common::math::Vec2d sub_point(
          lambda * points[point_count].x() +
              (1 - lambda) * points[point_count + 1].x(),
          lambda * points[point_count].y() +
              (1 - lambda) * points[point_count + 1].y());

      double l1(sub_point.DistanceTo(project_points[i]));
      double l2(points[point_count].DistanceTo(points[point_count + 1]));
      double theta =
          acos((project_points[i] - sub_point)
                   .InnerProd(points[point_count + 1] - points[point_count]) /
               (l1 * l2));
      double alpha = sin(theta) / (1 + sin(theta));
      cent_point->emplace_back((1 - alpha) * project_points[i] +
                               alpha * sub_point);
    } else {
      //      cent_point->clear();
      //      AERROR << "inital point error";
      //      return;
      continue;
    }
  }
  cent_point->emplace_back((project_points.back().x() + points.back().x()) / 2,
                           (project_points.back().y() + points.back().y()) / 2);
}

bool CalculateLambda(const hozon::common::math::Vec2d& p1,
                     const hozon::common::math::Vec2d& p2,
                     const hozon::common::math::Vec2d& p3,
                     const hozon::common::math::Vec2d& p4, double* lambda) {
  double delta = (p3.x() - p4.x()) * (p2.x() - p1.x()) -
                 (p3.y() - p4.y()) * (p1.y() - p2.y());
  if (!hozon::common::math::double_type::IsZero(delta)) {
    *lambda = ((p2.x() - p1.x()) * (p1.x() - p4.x()) +
               (p1.y() - p2.y()) * (p4.y() - p1.y())) /
              delta;
    return *lambda >= 0 && *lambda <= 1;
  }
  return false;
}

bool ComputeDiscretePointsProfile(
    const std::vector<hozon::common::math::Vec2d>& xy_points,
    std::vector<double>* headings, std::vector<double>* accumulated_s,
    std::vector<double>* kappas, std::vector<double>* dkappas) {
  // CHECK_NOTNULL(headings);
  // CHECK_NOTNULL(kappas);
  // CHECK_NOTNULL(dkappas);
  // CHECK_NOTNULL(accumulated_s);
  if (headings == nullptr || kappas == nullptr || dkappas == nullptr ||
      accumulated_s == nullptr) {
    return false;
  }
  headings->clear();
  kappas->clear();
  dkappas->clear();
  accumulated_s->clear();

  if (xy_points.size() < 2) {
    HLOG_ERROR << "xy_points size < 2!";
    return false;
  }
  std::vector<double> dxs;
  std::vector<double> dys;
  std::vector<double> y_over_s_first_derivatives;
  std::vector<double> x_over_s_first_derivatives;
  std::vector<double> y_over_s_second_derivatives;
  std::vector<double> x_over_s_second_derivatives;

  dxs.reserve(xy_points.size());
  dys.reserve(xy_points.size());
  y_over_s_first_derivatives.reserve(xy_points.size());
  x_over_s_first_derivatives.reserve(xy_points.size());
  y_over_s_second_derivatives.reserve(xy_points.size());
  x_over_s_second_derivatives.reserve(xy_points.size());

  // Get finite difference approximated dx and dy for heading and kappa
  // calculation
  std::size_t points_size = xy_points.size();
  for (std::size_t i = 0; i < points_size; ++i) {
    double x_delta = 0.0;
    double y_delta = 0.0;
    if (i == 0) {
      x_delta = (xy_points[i + 1].x() - xy_points[i].x());
      y_delta = (xy_points[i + 1].y() - xy_points[i].y());
    } else if (i == points_size - 1) {
      x_delta = (xy_points[i].x() - xy_points[i - 1].x());
      y_delta = (xy_points[i].y() - xy_points[i - 1].y());
    } else {
      x_delta = 0.5 * (xy_points[i + 1].x() - xy_points[i - 1].x());
      y_delta = 0.5 * (xy_points[i + 1].y() - xy_points[i - 1].y());
    }
    dxs.push_back(x_delta);
    dys.push_back(y_delta);
  }

  // Heading calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    headings->push_back(std::atan2(dys[i], dxs[i]));
  }

  // Get linear interpolated s for dkappa calculation
  double distance = 0.0;
  accumulated_s->push_back(distance);
  double fx = xy_points[0].x();
  double fy = xy_points[0].y();
  double nx = 0.0;
  double ny = 0.0;
  for (std::size_t i = 1; i < points_size; ++i) {
    nx = xy_points[i].x();
    ny = xy_points[i].y();
    double end_segment_s =
        std::sqrt((fx - nx) * (fx - nx) + (fy - ny) * (fy - ny));
    accumulated_s->push_back(end_segment_s + distance);
    distance += end_segment_s;
    fx = nx;
    fy = ny;
  }

  // Get finite difference approximated first derivative of y and x respective
  // to s for kappa calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    double xds = 0.0;
    double yds = 0.0;
    if (i == 0) {
      xds = (xy_points[i + 1].x() - xy_points[i].x()) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i));
      yds = (xy_points[i + 1].y() - xy_points[i].y()) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i));
    } else if (i == points_size - 1) {
      xds = (xy_points[i].x() - xy_points[i - 1].x()) /
            (accumulated_s->at(i) - accumulated_s->at(i - 1));
      yds = (xy_points[i].y() - xy_points[i - 1].y()) /
            (accumulated_s->at(i) - accumulated_s->at(i - 1));
    } else {
      xds = (xy_points[i + 1].x() - xy_points[i - 1].x()) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
      yds = (xy_points[i + 1].y() - xy_points[i - 1].y()) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
    }
    x_over_s_first_derivatives.push_back(xds);
    y_over_s_first_derivatives.push_back(yds);
  }

  // Get finite difference approximated second derivative of y and x respective
  // to s for kappa calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    double xdds = 0.0;
    double ydds = 0.0;
    if (i == 0) {
      xdds =
          (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i]) /
          (accumulated_s->at(i + 1) - accumulated_s->at(i));
      ydds =
          (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i]) /
          (accumulated_s->at(i + 1) - accumulated_s->at(i));
    } else if (i == points_size - 1) {
      xdds =
          (x_over_s_first_derivatives[i] - x_over_s_first_derivatives[i - 1]) /
          (accumulated_s->at(i) - accumulated_s->at(i - 1));
      ydds =
          (y_over_s_first_derivatives[i] - y_over_s_first_derivatives[i - 1]) /
          (accumulated_s->at(i) - accumulated_s->at(i - 1));
    } else {
      xdds = (x_over_s_first_derivatives[i + 1] -
              x_over_s_first_derivatives[i - 1]) /
             (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
      ydds = (y_over_s_first_derivatives[i + 1] -
              y_over_s_first_derivatives[i - 1]) /
             (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
    }
    x_over_s_second_derivatives.push_back(xdds);
    y_over_s_second_derivatives.push_back(ydds);
  }

  for (std::size_t i = 0; i < points_size; ++i) {
    double xds = x_over_s_first_derivatives[i];
    double yds = y_over_s_first_derivatives[i];
    double xdds = x_over_s_second_derivatives[i];
    double ydds = y_over_s_second_derivatives[i];
    double kappa =
        (xds * ydds - yds * xdds) /
        (std::sqrt(xds * xds + yds * yds) * (xds * xds + yds * yds) + 1e-6);
    kappas->push_back(kappa);
  }

  // Dkappa calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    double dkappa = 0.0;
    if (i == 0) {
      dkappa = (kappas->at(i + 1) - kappas->at(i)) /
               (accumulated_s->at(i + 1) - accumulated_s->at(i));
    } else if (i == points_size - 1) {
      dkappa = (kappas->at(i) - kappas->at(i - 1)) /
               (accumulated_s->at(i) - accumulated_s->at(i - 1));
    } else {
      dkappa = (kappas->at(i + 1) - kappas->at(i - 1)) /
               (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
    }
    dkappas->push_back(dkappa);
  }
  return true;
}

bool ComputeDiscretePoints(
    const std::vector<hozon::common::math::Vec2d>& xy_points,
    const std::vector<double> coeffs, std::vector<double>* kappas,
    std::vector<double>* dkappas) {
  if (kappas == nullptr || dkappas == nullptr) {
    return false;
  }

  kappas->clear();
  dkappas->clear();

  if (xy_points.size() < 2) {
    HLOG_ERROR << "xy_points size < 2!";
    return false;
  }

  std::size_t points_size = xy_points.size();
  for (std::size_t i = 0; i < points_size; ++i) {
    double first_deriv = 3 * coeffs[3] * std::pow(xy_points[i].x(), 2) +
                         2 * coeffs[2] * xy_points[i].x() + coeffs[1];
    double second_deriv = 6 * coeffs[3] * xy_points[i].x() + 2 * coeffs[2];
    double third_deriv = 6 * coeffs[3];
    double kappa =
        std::abs(second_deriv) / std::pow(1 + std::pow(first_deriv, 2), 1.5);
    kappas->emplace_back(kappa);

    double numerator = third_deriv * second_deriv;
    double denominator =
        std::pow(second_deriv, 2) + std::pow(1 + pow(first_deriv, 2), 1.5);
    double dkappa = denominator == 0. ? 0. : numerator / denominator;
    dkappas->emplace_back(dkappa);
  }
  return true;
}

bool DoubleHasSameSign(double first, double second) {
  return (hozon::common::math::double_type::ComparedToZero(first) >= 0 &&
          hozon::common::math::double_type::ComparedToZero(second) >= 0) ||
         (hozon::common::math::double_type::ComparedToZero(first) < 0 &&
          hozon::common::math::double_type::ComparedToZero(second) < 0);
}

void FitLaneLinePoint(const std::vector<hozon::common::math::Vec2d>& pts,
                      std::vector<double>* c) {
  int n = static_cast<int>(pts.size());
  Eigen::Matrix<double, Eigen::Dynamic, 4> A(n, 4);
  Eigen::VectorXd b(n);
  for (int i = 0; i < n; ++i) {
    double xi = pts[i].x();
    double yi = pts[i].y();
    A(i, 0) = 1.0;
    A(i, 1) = xi;
    A(i, 2) = xi * xi;
    A(i, 3) = xi * xi * xi;
    b[i] = yi;
  }
  // x = (A.transpose() * A).inverse() * A.transpose() * b;
  Eigen::VectorXd coeffs = A.householderQr().solve(b);
  c->emplace_back(coeffs[0]);
  c->emplace_back(coeffs[1]);
  c->emplace_back(coeffs[2]);
  c->emplace_back(coeffs[3]);
}
// 计算两直线交点坐标
Eigen::Vector2f FindPointOnExtendedLine(Eigen::Vector2f p1, Eigen::Vector2f p2,
                                        float angleRadians = 0.017f) {
  // 计算给定两点的直线斜率，处理垂直线情况
  double m1 =
      p1.x() == p2.x() ? INFINITY : (p2.y() - p1.y()) / (p2.x() - p1.x());
  double b1 = m1 * (-p1.x()) + p1.y();  // 使用任一点计算y轴截距
  double m2 = tan(angleRadians);

  // 由于经过原点，该直线的y轴截距为0
  double b2 = 0;

  // 解方程组求交点
  if (m1 == INFINITY) {  // 第一条直线垂直时
    return {p1.x(), m2 * p1.x()};
  } else if (m2 == INFINITY) {  // 不适用，因为我们已知m2是基于非垂直角度计算的
  } else {
    double x = (b2 - b1) / (m1 - m2);
    double y = m1 * x + b1;
    return {x, y};
  }

  return 0.5 * (p1 + p2);  // 如果没有交点或计算错误返回NaN
}

Eigen::Vector2f AddPointAlongHeading(Eigen::Vector2f origin, float headingRad,
                                     float distance) {
  /**
   * 在给定的heading方向上，从origin点出发，补充一个距离为distance的新点。
   *
   * @param origin: 原始点的坐标 (Eigen::Vector2f)
   * @param headingRad: 航向角，以弧度表示
   * @param distance: 新点与原点之间的距离
   * @return: 沿heading方向的新点坐标
   */
  // 根据heading计算x和y方向的偏移量
  float offsetX = distance * cos(headingRad);
  float offsetY = distance * sin(headingRad);

  // 计算新点坐标
  Eigen::Vector2f newPoint;
  newPoint.x() = origin.x() + offsetX;
  newPoint.y() = origin.y() + offsetY;

  return newPoint;
}

std::vector<hozon::common::math::Vec2d> LinearInterp(
    hozon::common::math::Vec2d start_point,
    hozon::common::math::Vec2d end_point, float interp_dist) {
  std::vector<hozon::common::math::Vec2d> out_interp_points;
  hozon::common::math::Vec2d dist = end_point - start_point;
  int interp_nums = std::floor(dist.Length() / interp_dist);
  hozon::common::math::Vec2d interval = dist / (interp_nums + 1);
  for (int i = 1; i <= interp_nums; ++i) {
    hozon::common::math::Vec2d interval_point = start_point + i * interval;
    out_interp_points.emplace_back(interval_point);
  }

  return out_interp_points;
}

double CalCubicCurveY(const std::vector<double> vehicle_curve,
                      const double& x) {
  double y = vehicle_curve[0] + vehicle_curve[1] * x +
             vehicle_curve[2] * x * x + vehicle_curve[3] * x * x * x;
  return y;
}

}  // namespace math

}  // namespace mf
}  // namespace mp
}  // namespace hozon
