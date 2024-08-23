/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： calc_util.h
 *   author     ： taoshaoyuan
 *   date       ： 2023.10
 ******************************************************************************/

#pragma once

#include <depend/common/math/double_type.h>
#include <depend/common/math/vec2d.h>

#include <Eigen/Dense>
#include <algorithm>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "modules/map_fusion_02/base/element_base.h"
#include "modules/map_fusion_02/base/element_map.h"
#include "modules/map_fusion_02/base/group.h"
#include "modules/map_fusion_02/common/common_data.h"
#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace mf {

inline double DegToRad(double deg) { return deg * M_PI / 180; }
inline double RadToDeg(double rad) { return 180 * rad / M_PI; }

double NowInSec();

void SplitSeconds(double secs, uint32_t* sec, uint32_t* nsec);

/// 将内旋顺序为Z-X-Y的欧拉角转换为四元数
template <typename T>
Eigen::Quaternion<T> IntZXYToQuat(const Eigen::Vector3<T>& int_zxy_euler) {
  Eigen::Quaternion<T> q =
      Eigen::AngleAxis<T>(int_zxy_euler.z(), Eigen::Vector3<T>::UnitZ()) *
      Eigen::AngleAxis<T>(int_zxy_euler.x(), Eigen::Vector3<T>::UnitX()) *
      Eigen::AngleAxis<T>(int_zxy_euler.y(), Eigen::Vector3<T>::UnitY());
  return q;
}

/// 将内旋顺序为Z-X-Y的欧拉角转换为旋转矩阵
template <typename T>
Eigen::Matrix3<T> IntZXYToRot(const Eigen::Vector3<T>& int_zxy_euler) {
  Eigen::Quaternion<T> q = IntZXYToQuat(int_zxy_euler);
  return q.toRotationMatrix();
}

/// 将内旋顺序为Z-X-Y的欧拉角转换为外旋顺序为X-Y-Z的欧拉角
template <typename T>
Eigen::Vector3<T> IntZXYToExtXYZ(const Eigen::Vector3<T>& int_zxy_euler) {
  Eigen::Matrix3<T> rot = IntZXYToRot(int_zxy_euler);
  Eigen::Vector3<T> euler = rot.eulerAngles(2, 1, 0);
  Eigen::Vector3<T> ext_xyz_euler;
  ext_xyz_euler << euler[2], euler[1], euler[0];
  return ext_xyz_euler;
}

template <typename T>
Eigen::Matrix3<T> Skew(const Eigen::Vector3<T>& vec) {
  Eigen::Matrix3<T> m;
  m << 0, -vec.z(), vec.y(), vec.z(), 0, -vec.x(), -vec.y(), vec.x(), 0;
  return m;
}

template <typename T>
Eigen::Matrix<T, 4, 4> QL(const Eigen::Quaternion<T>& q) {
  Eigen::Matrix<T, 4, 4> m;
  m << q.w(), -q.x(), -q.y(), -q.z(), q.x(), q.w(), -q.z(), q.y(), q.y(), q.z(),
      q.w(), -q.x(), q.z(), -q.y(), q.x(), q.w();
  return m;
}

template <typename T>
Eigen::Matrix<T, 4, 4> QR(const Eigen::Quaternion<T>& q) {
  Eigen::Matrix<T, 4, 4> m;
  m << q.w(), -q.x(), -q.y(), -q.z(), q.x(), q.w(), q.z(), -q.y(), q.y(),
      -q.z(), q.w(), q.x(), q.z(), q.y(), -q.x(), q.w();
  return m;
}

template <typename T>
Eigen::Quaternion<T> ExpQ(const Eigen::Vector3<T>& rot_vec) {
  // rotation angle
  T phi = rot_vec.norm();
  // rotation axis
  Eigen::Vector3<T> u = rot_vec.normalized();
  T re = static_cast<T>(std::cos(phi / 2));
  Eigen::Vector3<T> im = u * std::sin(phi / 2);
  Eigen::Quaternion<T> q(re, im.x(), im.y(), im.z());
  q.normalize();
  return q;
}

template <typename T>
Eigen::Matrix3<T> ExpR(const Eigen::Vector3<T>& rot_vec) {
  // rotation angle
  T phi = rot_vec.norm();
  // rotation axis
  Eigen::Vector3<T> u = rot_vec.normalized();
  // Rodrigues formula
  Eigen::Matrix3<T> R = Eigen::Matrix3<T>::Identity() * std::cos(phi) +
                        Skew(u) * std::sin(phi) +
                        u * u.transpose() * (1 - std::cos(phi));
  return R;
}

template <typename T>
T Dist(const Eigen::Matrix<T, 3, 1>& p0, const Eigen::Matrix<T, 3, 1>& p1) {
  return (p1 - p0).norm();
}

/// 求pt与向量p0p1的距离.
template <typename T, int ROWS>
T PointToVectorDist(const Eigen::Matrix<T, ROWS, 1>& p0,
                    const Eigen::Matrix<T, ROWS, 1>& p1,
                    const Eigen::Matrix<T, ROWS, 1>& pt) {
  Eigen::Matrix<T, ROWS, 1> p0_p1 = p1 - p0;
  p0_p1.normalize();
  Eigen::Matrix<T, ROWS, 1> pt_p0 = p0 - pt;
  Eigen::Matrix<T, ROWS, 1> d = pt_p0 - (pt_p0.transpose() * p0_p1) * p0_p1;
  return d.norm();
}

/// 判断pt在向量p0p1的左侧还是右侧.
/// 返回值：
///  < 0：pt在p0p1左侧；
///  > 0：pt在p0p1右侧；
///  = 0：pt在p0p1所在直线上.
template <typename T>
T PointInVectorSide(const Eigen::Matrix<T, 2, 1>& p0,
                    const Eigen::Matrix<T, 2, 1>& p1,
                    const Eigen::Matrix<T, 2, 1>& pt) {
  Eigen::Matrix<T, 2, 1> p0_p1 = p1 - p0;
  Eigen::Matrix<T, 2, 1> p0_pt = pt - p0;

  T cross = p0_pt.x() * p0_p1.y() - p0_p1.x() * p0_pt.y();
  return cross;
}

/// 判断pt在p0p1的投影点是否在线段内
template <typename T>
bool ProjectedInSegment(const Eigen::Matrix<T, 2, 1>& p0,
                        const Eigen::Matrix<T, 2, 1>& p1,
                        const Eigen::Matrix<T, 2, 1>& pt) {
  // 分别计算：p0p1向量与p0pt向量的夹角，以及p1p0向量与p1pt向量的夹角，
  // 有一个为钝角就表示不在范围内
  Eigen::Matrix<T, 2, 1> p0p1 = p1 - p0;
  p0p1.normalize();
  Eigen::Matrix<T, 2, 1> p0pt = pt - p0;
  p0pt.normalize();
  double theta0 = std::acos(p0p1.transpose() * p0pt);

  Eigen::Matrix<T, 2, 1> p1p0 = p0 - p1;
  p1p0.normalize();
  Eigen::Matrix<T, 2, 1> p1pt = pt - p1;
  p1pt.normalize();
  double theta1 = std::acos(p1p0.transpose() * p1pt);

  return theta0 < M_PI_2 && theta1 < M_PI_2;
}

/// 求向量p0pt在向量p0p1上的投影长度
template <typename T>
T ProjectionToVector(const Eigen::Matrix<T, 2, 1>& p0,
                     const Eigen::Matrix<T, 2, 1>& p1,
                     const Eigen::Matrix<T, 2, 1>& pt) {
  Eigen::Matrix<T, 2, 1> p0p1 = p1 - p0;
  Eigen::Matrix<T, 2, 1> p0pt = pt - p0;
  T dot = p0p1.transpose() * p0pt;
  T p0p1_norm = p0p1.norm();
  T proj = 0;
  if (std::abs(p0p1_norm - 0) > 0.000001) {
    proj = dot / p0p1_norm;
  }
  return proj;
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
bool SegmentIntersection(const Eigen::Vector2<T>& a, const Eigen::Vector2<T>& b,
                         const Eigen::Vector2<T>& c, const Eigen::Vector2<T>& d,
                         Eigen::Vector2<T>* intersect_pt) {
  if (intersect_pt == nullptr) {
    return false;
  }

  Eigen::Vector2<T> n2(d.y() - c.y(), c.x() - d.x());
  T dist_c_n2 = c.x() * n2.x() + c.y() * n2.y();
  T dist_a_n2 = a.x() * n2.x() + a.y() * n2.y();
  T dist_b_n2 = b.x() * n2.x() + b.y() * n2.y();
  const T EPSILON = 1e-5;
  if ((dist_a_n2 - dist_c_n2) * (dist_b_n2 - dist_c_n2) >= -EPSILON) {
    return false;
  }

  Eigen::Vector2<T> n1(b.y() - a.y(), a.x() - b.x());
  T dist_a_n1 = a.x() * n1.x() + a.y() * n1.y();
  T dist_c_n1 = c.x() * n1.x() + c.y() * n1.y();
  T dist_d_n1 = d.x() * n1.x() + d.y() * n1.y();
  if ((dist_c_n1 - dist_a_n1) * (dist_d_n1 - dist_a_n1) >= -EPSILON) {
    return false;
  }

  T denominator = n1.x() * n2.y() - n1.y() * n2.x();
  if (std::abs(denominator) < EPSILON) {
    return false;
  }
  T fraction = (dist_a_n1 - dist_c_n2) / denominator;
  intersect_pt->x() = a.x() + fraction * n1.y();
  intersect_pt->y() = a.y() - fraction * n1.x();
  return true;
}

// #define CATMULL_ROM_EIGEN
#ifdef CATMULL_ROM_EIGEN
template <typename T, int DIM>
class CatmullRomSpline {
 public:
  CatmullRomSpline(const Eigen::Matrix<T, DIM, 1>& p0,
                   const Eigen::Matrix<T, DIM, 1>& p1,
                   const Eigen::Matrix<T, DIM, 1>& p2,
                   const Eigen::Matrix<T, DIM, 1>& p3, T alpha)
      : alpha_(alpha), p0_(p0), p1_(p1), p2_(p2), p3_(p3) {
    Eigen::Matrix<T, 4, 4> m_alpha;
    // clang-format off
    m_alpha << 0, 1, 0, 0,
               -alpha_, 0, alpha_, 0,
               2 * alpha_, alpha_ - 3, 3 - 2 * alpha_, -alpha_,
               -alpha_, 2 - alpha_, alpha_ - 2, alpha_;
    // clang-format on
    Eigen::Matrix<T, 4, DIM> m_pts;
    m_pts.template block<1, DIM>(0, 0) = p0_.transpose();
    m_pts.template block<1, DIM>(1, 0) = p1_.transpose();
    m_pts.template block<1, DIM>(2, 0) = p2_.transpose();
    m_pts.template block<1, DIM>(3, 0) = p3_.transpose();
    m_ = m_alpha * m_pts;
  }

  Eigen::Matrix<T, DIM, 1> Parameterize(T u) {
    Eigen::Matrix<T, 1, 4> uv;
    uv << 1, u, u * u, u * u * u;
    Eigen::Matrix<T, 1, DIM> put = uv * m_;
    return put.transpose();
  }

 private:
  T alpha_;
  Eigen::Matrix<T, DIM, 1> p0_;
  Eigen::Matrix<T, DIM, 1> p1_;
  Eigen::Matrix<T, DIM, 1> p2_;
  Eigen::Matrix<T, DIM, 1> p3_;
  Eigen::Matrix<T, 4, DIM> m_;
};
#else
/// 不用Eigen, 实测计算效率更高
template <typename T, int DIM>
class CatmullRomSpline {
 public:
  CatmullRomSpline(const Eigen::Matrix<T, DIM, 1>& p0,
                   const Eigen::Matrix<T, DIM, 1>& p1,
                   const Eigen::Matrix<T, DIM, 1>& p2,
                   const Eigen::Matrix<T, DIM, 1>& p3, T alpha)
      : alpha_(alpha) {
    for (int j = 0; j < DIM; ++j) {
      pts_[0][j] = p0[j];
      pts_[1][j] = p1[j];
      pts_[2][j] = p2[j];
      pts_[3][j] = p3[j];
    }

    std::array<std::array<T, 4>, 4> m_alpha = {
        {{0, 1, 0, 0},
         {-alpha_, 0, alpha_, 0},
         {2 * alpha_, alpha_ - 3, 3 - 2 * alpha_, -alpha_},
         {-alpha_, 2 - alpha_, alpha_ - 2, alpha_}}};

    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < DIM; ++j) {
        m_[i][j] = m_alpha[i][0] * pts_[0][j] + m_alpha[i][1] * pts_[1][j] +
                   m_alpha[i][2] * pts_[2][j] + m_alpha[i][3] * pts_[3][j];
      }
    }
  }

  Eigen::Matrix<T, DIM, 1> Parameterize(T u) {
    T uu = u * u;
    T uuu = uu * u;
    Eigen::Matrix<T, DIM, 1> put;
    for (int j = 0; j < DIM; ++j) {
      T v = 1 * m_[0][j] + u * m_[1][j] + uu * m_[2][j] + uuu * m_[3][j];
      put[j] = v;
    }
    return put;
  }

 private:
  T alpha_;
  std::array<std::array<T, DIM>, 4> pts_;
  std::array<std::array<T, DIM>, 4> m_;
};
#endif

// /// 已知a系下0、1时刻的位姿(T_a_0, T_a_1), b系下0时刻的位姿(T_b_0),
// /// 求b系下1时刻位姿(T_b_1):
// /// T_0_1 = T_a_0.inverse() * T_a_1;
// /// T_b_1 = T_b_0 * T_0_1
// void InterpPose(const Pose& T_a_0, const Pose& T_a_1, const Pose& T_b_0,
//                 Pose* T_b_1);

class TicToc {
 public:
  TicToc() { Tic(); }

  void Tic() { clock_gettime(CLOCK_REALTIME, &start_); }

  // in ms
  double Toc() {
    clock_gettime(CLOCK_REALTIME, &end_);
    double start_ms = static_cast<double>(start_.tv_sec) * 1e3 +
                      static_cast<double>(start_.tv_nsec) * 1e-6;
    double end_ms = static_cast<double>(end_.tv_sec) * 1e3 +
                    static_cast<double>(end_.tv_nsec) * 1e-6;
    time_cost_ = end_ms - start_ms;
    time_total_ += time_cost_;
    time_count_++;
    return time_cost_;
  }

  double AveCost() const { return time_total_ / time_count_; }

  double Cost() const { return time_cost_; }

 private:
  timespec start_{};
  timespec end_{};

  double time_cost_{0.0};
  double time_total_{0.0};
  int time_count_{0};
};

class Rate {
 public:
  explicit Rate(double hz);
  explicit Rate(uint64_t us);

  bool Sleep();
  void Reset();
  // Return actual cycle time, in seconds
  double CycleTime() const;
  // Return expected cycle time, in seconds
  double ExpectedCycleTime() const;

 private:
  double start_;
  double expected_cycle_time_;
  double actual_cycle_time_;
};

void TransformMap(const ElementMap& map_source, const Pose& T_source_to_target,
                  const std::string& target_frame_id, ElementMap* map_target);

namespace math {

void GenerateCenterPoint(
    const std::vector<hozon::common::math::Vec2d>& left_point,
    const std::vector<hozon::common::math::Vec2d>& right_point,
    std::vector<hozon::common::math::Vec2d>* cent_point);

void CenterPoint(const std::vector<hozon::common::math::Vec2d>& project_points,
                 const std::vector<hozon::common::math::Vec2d>& points,
                 std::vector<hozon::common::math::Vec2d>* cent_point);

bool CalculateLambda(const hozon::common::math::Vec2d& p1,
                     const hozon::common::math::Vec2d& p2,
                     const hozon::common::math::Vec2d& p3,
                     const hozon::common::math::Vec2d& p4, double* lambda);
bool ComputeDiscretePoints(
    const std::vector<hozon::common::math::Vec2d>& xy_points,
    const std::vector<double> coeffs, std::vector<double>* kappas,
    std::vector<double>* dkappas);
bool DoubleHasSameSign(double first, double second);
void FitLaneLinePoint(const std::vector<hozon::common::math::Vec2d>& pts,
                      std::vector<double>* c);

Eigen::Vector2f FindPointOnExtendedLine(Eigen::Vector2f point1,
                                        Eigen::Vector2f point2,
                                        float angleRadians);

Eigen::Vector2f AddPointAlongHeading(Eigen::Vector2f origin, float headingRad,
                                     float distance);

std::vector<hozon::common::math::Vec2d> LinearInterp(
    hozon::common::math::Vec2d start_point,
    hozon::common::math::Vec2d end_point, float interp_dist);

double CalCubicCurveY(const std::vector<double> vehicle_curve, const double& x);

Eigen::Vector3f Quat2EulerAngle(const Eigen::Quaternionf& q);
float AngleDiff(float angle_0, float angle_1);

double CalculateHeading(const Eigen::Quaternionf& q1,
                        const Eigen::Quaternionf& q2);

std::vector<double> FitLaneline(const std::vector<Point>& centerline);

std::vector<double> FitLanelinefront(const std::vector<Point>& centerline);

template <typename PointType>
void ComputerLineDis(const std::vector<PointType>& line_pts,
                     const std::vector<PointType>& right_line_pts,
                     std::vector<double>* line_dis);

bool IsRight(const Eigen::Vector3d& P, const Eigen::Vector3d& A,
             const Eigen::Vector3d& B);

template <typename T1, typename T2>
float evaluateHeadingDiff(const T1& x, const std::vector<T2>& params);
double CalMeanLineHeading(const std::vector<Eigen::Vector3d>& points);

template <typename T, typename M>
T CubicResolve(const std::vector<T>& coefs, M t);

template <typename T>
bool InRange(T x, T min_x, T max_x);

template <typename T>
bool OutRange(T x, T min_x, T max_x);

template <typename T>
bool InCubicRange(T x, const LineCubic& cubic);
// 采样三次多项式曲线

template <typename pointType>
double GetLength(const std::vector<pointType>& point_sets);

template <typename pointType>
float GetOverLayRatioBetweenTwoLane(const std::vector<pointType>& curve1,
                                    const std::vector<pointType>& curve2);

template <typename T>
void SamplingCubic(const LineCubic& cubic, float step,
                   std::vector<Eigen::Matrix<T, 3, 1>>* pts);

}  // namespace math

}  // namespace mf
}  // namespace mp
}  // namespace hozon

#include "modules/map_fusion_02/common/calc_util.hpp"
