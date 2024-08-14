/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： cut_point.h
 *   author     ： mahaijun
 *   date       ： 2024.06
 ******************************************************************************/

#include "modules/map_fusion_02/modules/lane/road_builder/ctp_util.h"

namespace hozon {
namespace mp {
namespace mf {

bool IsOverLapTwoLine(const std::vector<Point3D>& segment1,
                      const std::vector<Point3D>& segment2) {
  double overlap_value = 0.0;
  if (segment1.size() < 2 || segment2.size() < 2) {
    overlap_value = 0.0;
  } else {
    Point3D back_pt = segment1.back();
    Point3D front_pt = segment1.front();
    Point3D direction_v = back_pt - front_pt;
    HLOG_DEBUG << "back_x: " << back_pt.x << " front_x: " << front_pt.x;
    // line1:  f1 ~ - - - - - - - - - ~ b1
    // line2:                 f2 ~ - - - - ~ b2
    if (direction_v.Dot(segment2.front() - front_pt) > 0) {
      front_pt = segment2.front();
    }
    // line1:  f1 ~ - - - - - - - - - ~ b1
    // line2:        f2 ~ - - - - ~ b2
    if (direction_v.Dot(segment2.back() - back_pt) < 0) {
      back_pt = segment2.back();
    }
    HLOG_DEBUG << "back_x: " << back_pt.x << " front_x: " << front_pt.x;
    Point3D new_direction = back_pt - front_pt;
    double overlap = direction_v.Dot(new_direction);
    HLOG_DEBUG << "overlap: " << overlap;
    if (overlap <= 0) {
      overlap_value = 0.0;
    } else {
      double hd_length = direction_v.Norm2D();
      HLOG_DEBUG << "back_x: " << back_pt.x << " front_x: " << front_pt.x
                 << " hd_length: " << hd_length;
      overlap_value = overlap / direction_v.SquaredNorm();
    }
  }
  HLOG_DEBUG << "overlap_value: " << overlap_value;
  return overlap_value > 1e-3;
}

SMStatus DistBetweenTwoLine(const std::vector<Point3D>& points_a,
                            const std::vector<Point3D>& points_b,
                            double* _mean_d, double* _min_d,
                            int* _overlap_status) {
  HLOG_DEBUG << "## DistBetweenTwoLine";
  static TicToc timer;
  timer.Tic();

  if (points_a.size() < 2 || points_b.size() < 2) {
    HLOG_ERROR
        << "Check failed: (points_a.size() < 2 || points_b.size() < 2)";
    return SMStatus::ERROR;
  }
  *_overlap_status = 0;

  // for (size_t i = 0; i < points_a.size(); i++)
  //     SM_LTRACE() << "points_a, i: " << i << ", p: " << points_a[i];
  // for (size_t i = 0; i < points_b.size(); i++)
  //     SM_LTRACE() << "points_b, i: " << i << ", p: " << points_b[i];

  // 1. lineA is totally front of lineB
  const auto& start = points_a[0];
  const auto& pt1_s = points_b[points_b.size() - 2];
  const auto& pt2_s = points_b[points_b.size() - 1];
  Point3D proj_pt;
  float64_t coef = 0;
  Point2LineProject3D(start, pt1_s, pt2_s, &proj_pt, &coef);
  if (coef >= 1) {
    HLOG_DEBUG << "lineA front of lineB, coef: " << coef;
    // min_d, sometimes should consider that coef cannot be too big
    double dist = std::sqrt((start.x - proj_pt.x) * (start.x - proj_pt.x) +
                            (start.y - proj_pt.y) * (start.y - proj_pt.y));
    *_min_d = dist;
    *_overlap_status = 1;
    return SMStatus::SUCCESS;
  }

  // 2. lineA is totally back of lineB
  const auto& end = points_a[points_a.size() - 1];
  const auto& pt1_e = points_b[0];
  const auto& pt2_e = points_b[1];
  Point2LineProject3D(end, pt1_e, pt2_e, &proj_pt, &coef);
  if (coef <= 0) {  // end <= pt1_e
    HLOG_DEBUG << "lineA back of lineB, coef: " << coef;
    double dist = std::sqrt((end.x - proj_pt.x) * (end.x - proj_pt.x) +
                            (end.y - proj_pt.y) * (end.y - proj_pt.y));
    *_min_d = dist;
    *_overlap_status = 2;
    return SMStatus::SUCCESS;
  }

  // 3. there is overlap between AB
  // SM_LTRACE() << "compute dist for AB begin ...";

  double mean1 = 0.0;
  double min1 = 0.0;
  auto status = ComputeLinePointsDist(points_a, points_b, &mean1, &min1);
  if (!(status == SMStatus::SUCCESS)) {
    HLOG_ERROR << "Check failed: (status != SMStatus::SUCCESS)";
    return SMStatus::ERROR;
  }
  // SM_LTRACE() << "mean1: " << mean1 << ", min1: " << min1;

  double mean2 = 0.0;
  double min2 = 0.0;
  status = ComputeLinePointsDist(points_b, points_a, &mean2, &min2);
  if (!(status == SMStatus::SUCCESS)) {
    HLOG_ERROR << "Check failed: (status == SMStatus::SUCCESS)";
    return SMStatus::ERROR;
  }
  // SM_LTRACE() << "mean2: " << mean2 << ", min2: " << min2;

  *_mean_d = mean1 > mean2 ? mean2 : mean1;
  *_min_d = min1 > min2 ? min2 : min1;

  HLOG_DEBUG << "DistBetweenTwoLine cost: " << timer.Toc();
  HLOG_DEBUG << "DistBetweenTwoLine ave cost: " << timer.AveCost();
  return SMStatus::SUCCESS;
}

SMStatus Point2LineProject3D(const Point3D& pt, const Point3D& line_start_pt,
                             const Point3D& line_end_pt, Point3D* project_pt,
                             float64_t* coef) {
  if (project_pt == nullptr) {
    return SMStatus::NULL_PTR;
  }
  if (line_end_pt == line_start_pt) {
    return SMStatus::ERROR;
  }
  auto direction = line_end_pt - line_start_pt;
  auto line_len = direction.Norm();
  if (line_len < 1e-6) {
    return SMStatus::ERROR;
  }

  auto unit_direction = direction / line_len;
  auto v1 = pt - line_start_pt;
  auto project_len = InnerProd3d(v1, unit_direction);
  auto _coef = project_len / line_len;
  project_pt->x = (line_end_pt.x - line_start_pt.x) * _coef + line_start_pt.x;
  project_pt->y = (line_end_pt.y - line_start_pt.y) * _coef + line_start_pt.y;
  project_pt->z = (line_end_pt.z - line_start_pt.z) * _coef + line_start_pt.z;
  if (coef != nullptr) {
    *coef = _coef;
  }
  return SMStatus::SUCCESS;
}

float64_t InnerProd3d(const Point3D& v1, const Point3D& v2) {
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

SMStatus ComputeLinePointsDist(const std::vector<Point3D>& points_a,
                               const std::vector<Point3D>& points_b,
                               double* _mean_d, double* _min_d) {
  if (!(points_a.size() >= 2)) {
    HLOG_ERROR << "Check failed: (points_a.size() >= 2)";
    return SMStatus::ERROR;
  }
  if (!(points_b.size() >= 2)) {
    HLOG_ERROR << "Check failed: (points_b.size() >= 2)";
    return SMStatus::ERROR;
  }

  double mean_d = 0;
  double min_d = __DBL_MAX__;
  int count_num = 0;

  int start_index = static_cast<int>(points_b.size()) - 2;
  bool break_flag = false;

  // from end to start, time complexity is N
  for (int i = static_cast<int>(points_a.size()) - 1; i >= 0; i--) {
    const auto& pt0 = points_a[i];
    // SM_LTRACE() << "i: " << i << ", pt0: " << pt0
    //             << ", start_index: " << start_index
    //             << ", break_flag: " << break_flag;

    if (break_flag) {
      break;
    }

    for (int j = start_index; j >= 0; j--) {
      const auto& pt1 = points_b[j];
      const auto& pt2 = points_b[j + 1];
      // SM_LTRACE() << "j: " << j << ", pt1: " << pt1 << ", pt2: " <<
      // pt2;

      Point3D proj_pt;
      float64_t coef = 0;
      Point2LineProject3D(pt0, pt1, pt2, &proj_pt, &coef);

      if (start_index == 0 && coef < 0) {
        // need finish all loop
        break_flag = true;
      }

      if (coef > 1) {
        if (j == start_index) {
          // not found projection, start_index not update

          break;

        } else {
          double dist = std::sqrt((pt0.x - pt2.x) * (pt0.x - pt2.x) +
                                  (pt0.y - pt2.y) * (pt0.y - pt2.y));
          HLOG_DEBUG << "special case dist: " << dist;
          mean_d += dist;
          count_num++;
          if (dist < min_d) {
            min_d = dist;
          }
          start_index = j;
          break;
        }
      }

      // found target projection
      if (coef >= 0 && coef <= 1) {
        double dist = std::sqrt((pt0.x - proj_pt.x) * (pt0.x - proj_pt.x) +
                                (pt0.y - proj_pt.y) * (pt0.y - proj_pt.y));
        // HLOG_INFO << "found target projection dist: " << dist;
        mean_d += dist;
        count_num++;
        if (dist < min_d) {
          min_d = dist;
        }

        start_index = j;
        break;
      }
    }
  }

  if (min_d == __DBL_MAX__) {
    HLOG_WARN << "min_d == __DBL_MAX__";
    return SMStatus::ERROR;
  }

  mean_d /= count_num;
  *_min_d = min_d;
  *_mean_d = mean_d;
  HLOG_DEBUG << "count_num: " << count_num << ", mean_d: " << mean_d
             << ", min_d: " << min_d;

  return SMStatus::SUCCESS;
}

SMStatus GetProjectPoint(const std::vector<Point3D>& points,
                         const Point3D& ref_point, Point3D* _proj_point,
                         size_t* _next_idx, int* _pos_flag) {
  if (!(points.size() >= 2)) {
    HLOG_ERROR << "Check failed: (points.size() >= 2)";
    return SMStatus::ERROR;
  }

  double min_dist = __DBL_MAX__;
  size_t min_idx = 0;
  for (size_t i = 0; i < points.size(); i++) {
    const auto& point = points[i];
    double dist = (ref_point - point).Norm2D();
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }

  if (min_idx == 0) {
    const auto& p1 = points[0];
    const auto& p2 = points[1];

    Point3D proj_pt;
    float64_t coef = 0;
    Point2LineProject2D(ref_point, p1, p2, &proj_pt, &coef);
    HLOG_DEBUG << "coef: " << coef;
    if (coef <= 0) {
      HLOG_DEBUG << "coef <= 0";
      *_proj_point = p1;
      *_next_idx = 0;
      *_pos_flag = -1;
      return SMStatus::SUCCESS;

    } else if (coef > 0 && coef <= 1) {
      HLOG_DEBUG << "coef > 0 && coef <= 1";
      *_proj_point = proj_pt;
      *_next_idx = 1;
      *_pos_flag = 0;
      return SMStatus::SUCCESS;

    } else if (coef > 1) {
      HLOG_DEBUG << "coef > 1";
      *_proj_point = p2;
      *_next_idx = 1;
      *_pos_flag = 0;
      return SMStatus::SUCCESS;
    }

  } else if (min_idx == points.size() - 1) {
    const auto& p2 = points[points.size() - 1];
    const auto& p1 = points[points.size() - 2];

    Point3D proj_pt;
    float64_t coef = 0;
    Point2LineProject2D(ref_point, p1, p2, &proj_pt, &coef);
    HLOG_DEBUG << "coef: " << coef;
    if (coef <= 0) {
      HLOG_DEBUG << "coef <= 0";
      *_proj_point = p1;
      *_next_idx = points.size() - 2;
      *_pos_flag = 0;
      return SMStatus::SUCCESS;

    } else if (coef > 0 && coef <= 1) {
      HLOG_DEBUG << "coef > 0 && coef <= 1";
      *_proj_point = proj_pt;
      *_next_idx = points.size() - 1;
      *_pos_flag = 0;
      return SMStatus::SUCCESS;

    } else if (coef > 1) {
      HLOG_DEBUG << "coef > 1";
      *_proj_point = p2;
      *_next_idx = points.size() - 1;
      *_pos_flag = 1;
      return SMStatus::SUCCESS;
    }

  } else {
    // min_idx is not the start or end
    const auto& p1 = points[min_idx - 1];
    const auto& p2 = points[min_idx];
    const auto& p3 = points[min_idx + 1];

    Point3D proj_pt1;
    float64_t coef1 = 0;
    Point2LineProject2D(ref_point, p1, p2, &proj_pt1, &coef1);
    // HLOG_INFO << "check projection in p1 - p2, coef1: " << coef1;
    if (coef1 <= 0) {
      *_proj_point = p1;
      *_next_idx = min_idx - 1;
      *_pos_flag = 0;
      return SMStatus::SUCCESS;

    } else if (coef1 > 0 && coef1 <= 1) {
      *_proj_point = proj_pt1;
      *_next_idx = min_idx;
      *_pos_flag = 0;
      return SMStatus::SUCCESS;
    }

    Point3D proj_pt2;
    float64_t coef2 = 0;
    Point2LineProject2D(ref_point, p2, p3, &proj_pt2, &coef2);
    // HLOG_INFO << "check projection in p2 - p3, coef2: " << coef2;
    if (coef2 <= 0) {
      *_proj_point = p2;
      *_next_idx = min_idx;
      *_pos_flag = 0;
      return SMStatus::SUCCESS;

    } else if (coef2 > 0 && coef2 <= 1) {
      *_proj_point = proj_pt2;
      *_next_idx = min_idx + 1;
      *_pos_flag = 0;
      return SMStatus::SUCCESS;

    } else if (coef2 > 1) {
      *_proj_point = p3;
      *_next_idx = min_idx + 1;
      *_pos_flag = 0;
      return SMStatus::SUCCESS;
    }
  }

  return SMStatus::SUCCESS;
}

SMStatus Point2LineProject2D(const Point3D& pt, const Point3D& line_start_pt,
                             const Point3D& line_end_pt, Point3D* project_pt,
                             float64_t* coef) {
  auto direction = line_end_pt - line_start_pt;
  auto line_len = direction.Norm2D();
  if (line_len == 0) {
    *project_pt = line_start_pt;
    *coef = 0;
    HLOG_ERROR << "line_len == 0, start == end";
    return SMStatus::ERROR;
  }

  auto unit_direction = direction / line_len;
  auto v1 = pt - line_start_pt;
  auto project_len = InnerProd2d(v1, unit_direction);
  auto _coef = project_len / line_len;
  project_pt->x = (line_end_pt.x - line_start_pt.x) * _coef + line_start_pt.x;
  project_pt->y = (line_end_pt.y - line_start_pt.y) * _coef + line_start_pt.y;
  project_pt->z = (line_end_pt.z - line_start_pt.z) * _coef + line_start_pt.z;
  if (coef != nullptr) {
    *coef = _coef;
  }

  return SMStatus::SUCCESS;
}

float64_t InnerProd2d(const Point3D& v1, const Point3D& v2) {
  return v1.x * v2.x + v1.y * v2.y;
}

Point3D ToPoint(const Eigen::Vector3d& p) { return {p.x(), p.y(), p.z()}; }

SMStatus GetFootPointInLine(const Point3D& ref_point,
                            const std::vector<Point3D>& points,
                            Point3D* _foot_point) {
  if (!(points.size() >= 2)) {
    HLOG_ERROR << "check failed: (points.size() >= 2)";
    return SMStatus::ERROR;
  }

  double min_dist = __DBL_MAX__;
  size_t min_idx = 0;
  for (size_t i = 0; i < points.size(); i++) {
    const auto& point = points[i];
    double dist = (ref_point - point).Norm2D();
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }

  if (min_idx == 0) {
    const auto& p1 = points[0];
    const auto& p2 = points[1];

    Point3D proj_pt;
    float64_t coef = 0;
    Point2LineProject2D(ref_point, p1, p2, &proj_pt, &coef);
    HLOG_DEBUG << "coef: " << coef;
    *_foot_point = proj_pt;
    if (coef < 0) {
      return SMStatus::INVALID;
    }

  } else if (min_idx == points.size() - 1) {
    const auto& p1 = points[points.size() - 2];
    const auto& p2 = points[points.size() - 1];

    Point3D proj_pt;
    float64_t coef = 0;
    Point2LineProject2D(ref_point, p1, p2, &proj_pt, &coef);
    HLOG_DEBUG << "coef: " << coef;
    *_foot_point = proj_pt;
    if (coef > 1) {
      return SMStatus::INVALID;
    }

  } else {
    const auto& p1 = points[min_idx - 1];
    const auto& p2 = points[min_idx];
    const auto& p3 = points[min_idx + 1];

    Point3D proj_pt1;
    float64_t coef1 = 0;
    Point2LineProject2D(ref_point, p1, p2, &proj_pt1, &coef1);
    HLOG_DEBUG << "coef1: " << coef1;
    if (coef1 >= 0 && coef1 <= 1) {
      *_foot_point = proj_pt1;
      return SMStatus::SUCCESS;
    }

    Point3D proj_pt2;
    float64_t coef2 = 0;
    Point2LineProject2D(ref_point, p2, p3, &proj_pt2, &coef2);
    HLOG_DEBUG << "coef2: " << coef2;
    if (coef2 >= 0 && coef2 <= 1) {
      *_foot_point = proj_pt2;
      return SMStatus::SUCCESS;
    }

    *_foot_point = p2;
  }

  return SMStatus::SUCCESS;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
