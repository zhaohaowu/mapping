/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-09-06
 *****************************************************************************/
#pragma once

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "Eigen/src/Core/Matrix.h"
#include "depend/common/utm_projection/coordinate_convertor.h"
#include "depend/map/hdmap/hdmap.h"
#include "depend/proto/soc/sensor_image.pb.h"
#include "interface/adsfi_proto/viz/sensor_msgs.pb.h"
#include "modules/local_mapping/base/location/dr.h"
#include "modules/local_mapping/base/scene/laneline.h"
#include "modules/local_mapping/base/scene/roadedge.h"
#include "modules/map_fusion/include/map_fusion/map_service/global_hd_map.h"
#include "modules/util/include/util/geo.h"
#include "modules/util/include/util/mapping_log.h"
#include "modules/util/include/util/rviz_agent/rviz_agent.h"
namespace hozon {
namespace mp {
namespace lm {

class CommonUtil {
 public:
  // static bool IsOcclusionByObstacle(Eigen::Vector3d center,
  //                                   Eigen::Vector3d size,
  //                                   std::vector<Object> obstacles) {
  //   auto input_x1 = center.x() - size.x() / 2.0;
  //   auto input_y1 = center.y() - size.y() / 2.0;
  //   auto input_x2 = center.x() + size.x() / 2.0;
  //   auto input_y2 = center.y() + size.y() / 2.0;

  //   for (auto& item : obstacles) {
  //     auto compare_x1 = item.center.x() - item.size.x() / 2.0;
  //     auto compare_y1 = item.center.y() - item.size.y() / 2.0;
  //     auto compare_x2 = item.center.x() + item.size.x() / 2.0;
  //     auto compare_y2 = item.center.y() + item.size.y() / 2.0;

  //     if (input_x1 > compare_x1 && input_x1 < compare_x2 &&
  //         input_y1 > compare_y1 && input_y1 < compare_y2 &&
  //         input_x2 > compare_x1 && input_x2 < compare_x2 &&
  //         input_y2 > compare_y1 && input_y2 < compare_y2) {
  //       return true;
  //     }
  //   }

  //   return false;
  // }

  static void FitLaneLine(const std::vector<Eigen::Vector3d>& pts,
                          std::vector<double>* c) {
    int n = static_cast<int>(pts.size());
    Eigen::Matrix<double, Eigen::Dynamic, 4> A(n, 4);
    Eigen::VectorXd x(n);
    Eigen::VectorXd b(n);
    for (int i = 0; i < n; i++) {
      double xi = pts[i][0];
      double yi = pts[i][1];
      A(i, 0) = 1.0;
      A(i, 1) = xi;
      A(i, 2) = xi * xi;
      A(i, 3) = xi * xi * xi;
      b[i] = yi;
    }
    // x = (A.transpose() * A).inverse() * A.transpose() * b;
    x = A.householderQr().solve(b);
    c->at(0) = x[0];
    c->at(1) = x[1];
    c->at(2) = x[2];
    c->at(3) = x[3];
  }

  static double CalCubicCurveY(const LaneLineCurve& vehicle_curve,
                               const double& x) {
    double y = vehicle_curve.coeffs[0] + vehicle_curve.coeffs[1] * x +
               vehicle_curve.coeffs[2] * x * x +
               vehicle_curve.coeffs[3] * x * x * x;
    return y;
  }

  static double CalMainLaneHeading(
      const std::vector<Eigen::Vector3d>& left_points,
      const std::vector<Eigen::Vector3d>& right_points) {
    double left_theta = atan2(left_points[1].y() - left_points[0].y(),
                              left_points[1].x() - left_points[0].x());
    double right_theta = atan2(right_points[1].y() - right_points[0].y(),
                               right_points[1].x() - right_points[0].x());
    double min_yaw = 10.0 / 180 * M_PI;
    if (abs(left_theta - right_theta) < min_yaw) {
      return (left_theta + right_theta) / 2;
    }
    return abs(left_theta) < abs(right_theta) ? left_theta : right_theta;
  }

  static double CalTwoPointsDis(Eigen::Vector3d a, Eigen::Vector3d b) {
    double result = sqrt(pow(a.x() - b.x(), 2) + pow(a.y() - b.y(), 2) +
                         pow(a.z() - b.z(), 2));
    return result;
  }

  static double CalTwoPointsHeading(Eigen::Vector3d a, Eigen::Vector3d b) {
    double result = atan2(a.y() - b.y(), a.x() - b.x());
    return result;
  }

  static void EraseErrorPts(std::vector<Eigen::Vector3d>* pts) {
    for (int i = 0; i < static_cast<int>(pts->size() - 2); i++) {
      if ((pts->at(i + 1).y() - pts->at(i).y() > 1 &&
           pts->at(i + 2).y() - pts->at(i + 1).y() < 0) ||
          (pts->at(i + 1).y() - pts->at(i).y() > 0 &&
           pts->at(i + 2).y() - pts->at(i + 1).y() < -1) ||
          (pts->at(i + 1).y() - pts->at(i).y() < -1 &&
           pts->at(i + 2).y() - pts->at(i + 1).y() > 0) ||
          (pts->at(i + 1).y() - pts->at(i).y() < 0 &&
           pts->at(i + 2).y() - pts->at(i + 1).y() > 1)) {
        pts->at(i + 1).y() = (pts->at(i).y() + pts->at(i + 2).y()) / 2;
      }
    }
  }

  static bool IsConvex(const std::vector<Eigen::Vector3d>& points) {
    if (points.size() != 4) {
      return false;
    }
    auto IsOutsideTriangle =
        [](const Eigen::Vector3d& P, const Eigen::Vector3d& A,
           const Eigen::Vector3d& B, const Eigen::Vector3d& C) {
          Eigen::Vector2d AP = {P.x() - A.x(), P.y() - A.y()};
          Eigen::Vector2d BP = {P.x() - B.x(), P.y() - B.y()};
          Eigen::Vector2d CP = {P.x() - C.x(), P.y() - C.y()};
          Eigen::Vector2d AB = {B.x() - A.x(), B.y() - A.y()};
          Eigen::Vector2d BC = {C.x() - B.x(), C.y() - B.y()};
          Eigen::Vector2d CA = {A.x() - C.x(), A.y() - C.y()};
          double S1 = AP.x() * AB.y() - AP.y() * AB.x();
          double S2 = BP.x() * BC.y() - BP.y() * BC.x();
          double S3 = CP.x() * CA.y() - CP.y() * CA.x();
          return (S1 < 0 || S2 < 0 || S3 < 0) && (S1 > 0 || S2 > 0 || S3 > 0);
        };
    bool P0 = IsOutsideTriangle(points[0], points[1], points[2], points[3]);
    bool P1 = IsOutsideTriangle(points[1], points[0], points[2], points[3]);
    bool P2 = IsOutsideTriangle(points[2], points[0], points[1], points[3]);
    bool P3 = IsOutsideTriangle(points[3], points[0], points[1], points[2]);
    return P0 && P1 && P2 && P3;
  }

  static void CatmullRom(const std::vector<Eigen::Vector3d>& pts,
                         std::vector<Eigen::Vector3d>* fit_points, int num) {
    auto func = [](double p0, double p1, double p2, double p3, double t) {
      double s = 0.5;
      double a = -s * p0 + (2 - s) * p1 + (s - 2) * p2 + s * p3;
      double b = 2 * s * p0 + (s - 3) * p1 + (3 - 2 * s) * p2 - s * p3;
      double c = -s * p0 + s * p2;
      double d = p1;
      double t2 = t * t;
      double t3 = t2 * t;
      return (a * t3 + b * t2 + c * t + d);
    };
    for (size_t i = 0; i < pts.size() - 3; ++i) {
      double t = 0;
      while (t < 1) {
        double px =
            func(pts[i].x(), pts[i + 1].x(), pts[i + 2].x(), pts[i + 3].x(), t);
        double py =
            func(pts[i].y(), pts[i + 1].y(), pts[i + 2].y(), pts[i + 3].y(), t);
        Eigen::Vector3d point = {px, py, 0.0};
        fit_points->emplace_back(point);
        t += 1.0 / num;
      }
    }
  }

  //   static void FitLaneLines(std::vector<LaneLine>* map_lane_lines) {
  // for (auto& lane_line : *map_lane_lines) {
  //   lane_line.fit_points_.clear();
  //   lane_line.control_points_.clear();
  //   int n = static_cast<int>(lane_line.points_.size());
  //   if (n < 21) {
  //     continue;
  //   }
  //   std::vector<Eigen::Vector3d> pts;
  //   std::vector<Eigen::Vector3d> back_pts;
  //   for (int i = 0; i < n; i++) {
  //     if (i == 1 || i % 10 == 0 || i == n - 1) {
  //       pts.push_back(lane_line.points_[i]);
  //     }
  //     if (((n - 1) % 10 == 0 && i >= ((n - 1) / 10 - 1) * 10) ||
  //         ((n - 1) % 10 != 0 && i >= (n - 1) / 10 * 10)) {
  //       back_pts.push_back(lane_line.points_[i]);
  //     }
  //   }
  //   CommonUtil::EraseErrorPts(&pts);
  //   if (pts.size() < 4) {
  //     continue;
  //   }
  //   lane_line.control_points_ = pts;
  //   CatmullRom(pts, &lane_line.fit_points_);
  //   std::vector<double> c(4);
  //   CommonUtil::FitLaneLine(pts, &c);
  //   lane_line.c0_ = c[0];
  //   lane_line.c1_ = c[1];
  //   lane_line.c2_ = c[2];
  //   lane_line.c3_ = c[3];
  //   for (const auto& point : back_pts) {
  //     lane_line.fit_points_.push_back(point);
  //   }
  // }
  // map_lane_lines->erase(
  //     std::remove_if(map_lane_lines->begin(), map_lane_lines->end(),
  //                    [&](const LaneLine& lane_line) {
  //                      return lane_line.fit_points_.empty();
  //                    }),
  //     map_lane_lines->end());
  //   }

  //   static void FitRoadEdges(std::vector<RoadEdge>* map_road_edges) {
  // for (auto& road_edge : *map_road_edges) {
  //   if (!road_edge.ismature_) {
  //     continue;
  //   }
  //   road_edge.fit_points_.clear();
  //   road_edge.control_points_.clear();
  //   int n = static_cast<int>(road_edge.points_.size());
  //   if (n < 21) {
  //     continue;
  //   }
  //   std::vector<Eigen::Vector3d> pts;
  //   std::vector<Eigen::Vector3d> back_pts;
  //   for (int i = 0; i < n; i++) {
  //     if (i == 1 || i % 10 == 0 || i == n - 1) {
  //       pts.push_back(road_edge.points_[i]);
  //     }
  //     if (((n - 1) % 10 == 0 && i >= ((n - 1) / 10 - 1) * 10) ||
  //         ((n - 1) % 10 != 0 && i >= (n - 1) / 10 * 10)) {
  //       back_pts.push_back(road_edge.points_[i]);
  //     }
  //   }
  //   CommonUtil::EraseErrorPts(&pts);
  //   if (pts.size() < 4) {
  //     continue;
  //   }
  //   road_edge.control_points_ = pts;
  //   CatmullRom(pts, &road_edge.fit_points_);
  //   std::vector<double> c(4);
  //   CommonUtil::FitLaneLine(pts, &c);
  //   road_edge.c0_ = c[0];
  //   road_edge.c1_ = c[1];
  //   road_edge.c2_ = c[2];
  //   road_edge.c3_ = c[3];
  //   for (const auto& point : back_pts) {
  //     road_edge.fit_points_.push_back(point);
  //   }
  // }
  //   }

  // static DrData Interpolate(const double& scale, const DrData& start,
  //                           const DrData& end, const double& timestamp) {
  //   DrData res;
  //   res.timestamp = timestamp;
  //   res.pose = start.pose + (end.pose - start.pose) * scale;
  //   res.quaternion = start.quaternion.slerp(scale, end.quaternion);

  //   if (scale >= 0.5) {
  //     res.local_vel = end.local_vel;
  //     res.local_omg = end.local_omg;
  //   } else {
  //     res.local_vel = start.local_vel;
  //     res.local_omg = start.local_omg;
  //   }

  //   return res;
  // }

  static void TransVehiclePoint2Local(
      const std::vector<Eigen::Vector3d>& vehicle_points,
      std::vector<Eigen::Vector3d>* world_points,
      const Eigen::Affine3d& trans_pose) {
    world_points->clear();
    for (const auto& vehicle_pt : vehicle_points) {
      world_points->emplace_back(trans_pose * vehicle_pt);
    }
  }

  static void TransLocalPoint2Vehicle(
      const std::vector<Eigen::Vector3d>& world_points,
      std::vector<Eigen::Vector3d>* vehicle_points,
      const Eigen::Affine3d& trans_pose) {
    vehicle_points->clear();
    for (const auto& world_pt : world_points) {
      auto vehicle_pt = trans_pose.inverse() * world_pt;
      vehicle_points->emplace_back(vehicle_pt);
    }
  }

  static void Gcj02ToUtm(const Sophus::SE3d& T_G_V, Sophus::SE3d* T_U_V) {
    Eigen::Vector3d p_U_V = Eigen::Vector3d::Identity();
    util::Geo::LatLonToUtmXy(51, T_G_V.translation().y(),
                             T_G_V.translation().x(), &p_U_V);
    *T_U_V = Sophus::SE3d(T_G_V.so3().unit_quaternion(), p_U_V);
  }

  static Eigen::Vector3d UtmPointToVehicle(const Eigen::Vector3d& point_utm,
                                           const Eigen::Vector3d& ref_point,
                                           const Sophus::SE3d& T_G_V) {
    double x = point_utm.x();
    double y = point_utm.y();
    hozon::common::coordinate_convertor::UTM2GCS(51, &x, &y);
    Eigen::Vector3d point_gcj(y, x, 0);
    Eigen::Vector3d point_enu = util::Geo::Gcj02ToEnu(point_gcj, ref_point);

    Eigen::Vector3d p_enu_vehicle =
        util::Geo::Gcj02ToEnu(T_G_V.translation(), ref_point);
    Eigen::Quaterniond q_enu_vehicle = T_G_V.so3().unit_quaternion();
    Sophus::SE3d T_enu_vehicle = Sophus::SE3d(q_enu_vehicle, p_enu_vehicle);

    return T_enu_vehicle.inverse() * point_enu;
  }

  /// 判断两线段是否相交，如果相交计算出交点，参考：https://zhuanlan.zhihu.com/p/158533421
  /// 输入：
  ///   a: 线段1起点
  ///   b: 线段1终点
  ///   c: 线段2起点
  ///   d: 线段2终点
  ///   intersect_pt: 计算后的交点
  /// 返回值：
  ///   false，不相交
  ///   true，相交
  template <typename T>
  static bool SegmentIntersection(const Eigen::Vector3<T>& a,
                                  const Eigen::Vector3<T>& b,
                                  const Eigen::Vector3<T>& c,
                                  const Eigen::Vector3<T>& d,
                                  Eigen::Vector3<T>* intersect_pt) {
    if (intersect_pt == nullptr) {
      return false;
    }
    Eigen::Vector3<T> n2(d.y() - c.y(), c.x() - d.x(), 0);
    T dist_c_n2 = c.x() * n2.x() + c.y() * n2.y();
    T dist_a_n2 = a.x() * n2.x() + a.y() * n2.y();
    T dist_b_n2 = b.x() * n2.x() + b.y() * n2.y();
    const T EPSILON = 1e-5;
    if ((dist_a_n2 - dist_c_n2) * (dist_b_n2 - dist_c_n2) >= -EPSILON) {
      return false;
    }

    Eigen::Vector3<T> n1(b.y() - a.y(), a.x() - b.x(), 0);
    T dist_a_n1 = a.x() * n1.x() + a.y() * n1.y();
    T dist_c_n1 = c.x() * n1.x() + c.y() * n1.y();
    T dist_d_n1 = d.x() * n1.x() + d.y() * n1.y();
    if ((dist_c_n1 - dist_a_n1) * (dist_d_n1 - dist_a_n1) >= -EPSILON) {
      return false;
    }

    // 向量AB和CD
    Eigen::Vector3<T> AB(b.x() - a.x(), b.y() - a.y(), 0);
    Eigen::Vector3<T> CD(d.x() - c.x(), d.y() - c.y(), 0);

    // 计算外积的z分量判断是否平行
    T cross_product_z = AB.x() * CD.y() - AB.y() * CD.x();

    // 如果平行（包括共线），返回false
    if (std::abs(cross_product_z) < EPSILON - 9) {
      return false;
    }

    // 计算参数t和s
    T t =
        ((c.x() - a.x()) * CD.y() - (c.y() - a.y()) * CD.x()) / cross_product_z;
    T s =
        ((c.x() - a.x()) * AB.y() - (c.y() - a.y()) * AB.x()) / cross_product_z;

    // 检查t和s是否在[0, 1]区间内
    if (0 <= t && t <= 1 && 0 <= s && s <= 1) {
      intersect_pt->x() = a.x() + t * AB.x();
      intersect_pt->y() = a.y() + t * AB.y();
      return true;  // 有交点
    }
    return false;  // 无交点或不在线段上
  }

  // 计算车道线的平均heading
  static double CalMeanLineHeading(const std::vector<Eigen::Vector3d>& points) {
    if (points.size() < 2) {
      return 0;
    }
    double mean_theta = 0.;
    for (size_t i = 0; i < points.size() - 1; i++) {
      const Eigen::Vector2f pa(points[i].x(), points[i].y());
      const Eigen::Vector2f pb(points[i + 1].x(), points[i + 1].y());
      Eigen::Vector2f v = pb - pa;
      double theta = 0;
      if (std::abs(v.x()) > 1e-2) {
        theta = atan2(v.y(), v.x());  // atan2计算出的角度范围是[-pi, pi]
      }
      mean_theta += theta;
    }
    mean_theta /= static_cast<double>(points.size());
    return mean_theta;
  }

  static double NormalizeAngle(double angle) {
    if (angle > M_PI) {
      angle -= 2 * M_PI;
    }
    if (angle < -M_PI) {
      angle += 2 * M_PI;
    }
    return angle;
  }
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
