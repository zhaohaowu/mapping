/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： cut_point.h
 *   author     ： mahaijun
 *   date       ： 2024.05
 ******************************************************************************/

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <ostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Sophus/se3.hpp>
#include <Sophus/so3.hpp>

namespace hozon {
namespace mp {
namespace mf {

// Define Smart ptr
#define DEFINE_SMART_PTR(...) \
  DEFINE_PTR(__VA_ARGS__)     \
  DEFINE_UNIQUE_PTR(__VA_ARGS__)

#define DEFINE_PTR(...)                     \
  using Ptr = std::shared_ptr<__VA_ARGS__>; \
  using ConstPtr = std::shared_ptr<const __VA_ARGS__>;

#define DEFINE_UNIQUE_PTR(...)                    \
  using UniquePtr = std::unique_ptr<__VA_ARGS__>; \
  using ConstUniquePtr = std::unique_ptr<const __VA_ARGS__>;

#define DEFINE_CREATE_PTR(...)                      \
  static std::shared_ptr<__VA_ARGS__> CreatePtr() { \
    return std::make_shared<__VA_ARGS__>();         \
  }

using float32_t = float;
using float64_t = double;
using id_t = int32_t;

enum class SMStatus {
  ERROR = 0,
  INVALID = 1,
  INVALID_PARAM = 2,
  FILE_NOT_FOUND = 3,
  NULL_PTR = 4,
  INIT_ERROR = 5,
  END_OF_FILE = 6,
  NO_INIT_ERROR = 7,
  ALLOC_MEMORY_FAILED = 8,
  INTERNAL_ERROR = 9,
  TIME_DISORDER = 10,
  OUT_OF_RANGE = 11,
  SUCCESS = 12,
  TIME_AHEAD = 13,
  TIME_DELAY = 14
};

enum class LMSFStatus {
  INVALID = 0,
  CONVERGING = 1,
  UNSTABLE = 2,
  OK = 3,
  GOOD = 4,
};

inline std::string StatusToString(const SMStatus status) {
  switch (status) {
    case SMStatus::SUCCESS:
      return "SUCCESS";
    case SMStatus::INVALID:
      return "INVALID";
    case SMStatus::INVALID_PARAM:
      return "INVALID_PARAM";
    case SMStatus::FILE_NOT_FOUND:
      return "FILE_NOT_FOUND";
    case SMStatus::NULL_PTR:
      return "NULL_PTR";
    case SMStatus::INIT_ERROR:
      return "INIT_ERROR";
    case SMStatus::END_OF_FILE:
      return "END_OF_FILE";
    case SMStatus::NO_INIT_ERROR:
      return "NO_INIT_ERROR";
    case SMStatus::ALLOC_MEMORY_FAILED:
      return "ALLOC_MEMORY_FAILED";
    case SMStatus::INTERNAL_ERROR:
      return "INTERNAL_ERROR";
    case SMStatus::TIME_DISORDER:
      return "TIME_DISORDER";
    case SMStatus::OUT_OF_RANGE:
      return "OUT_OF_RANGE";
    case SMStatus::ERROR:
      return "ERROR";
    case SMStatus::TIME_AHEAD:
      return "TIME_AHEAD";
    case SMStatus::TIME_DELAY:
      return "TIME_DELAY";
    default:
      return "";
  }
}

enum class CutPointType : std::uint8_t {
  Unknown = 0,
  Split = 1,
  Merge = 2,
  Multi2Single = 3,  // 少变多, 多变少点
  Single2Multi = 4,
  V_Shaped = 5,
  V_Shaped_Inv = 6,  // 倒V字形状
  Broken = 7,
  Broken_Start = 8
};

template <typename T>
bool FloatEqual(const T x, const T y) {
  const T max_val = std::max({1.0, std::fabs(x), std::fabs(y)});
  return std::fabs(x - y) <= std::numeric_limits<T>::epsilon() * max_val;
}

struct Point2D {
  float64_t x = 0;
  float64_t y = 0;
  float32_t weight = 1.0;
  Point2D() = default;
  Point2D(float64_t _x, float64_t _y, float32_t _weight = 1.0)
      : x(_x), y(_y), weight(_weight) {}
  float64_t Norm() const { return std::sqrt(x * x + y * y); }

  float64_t SquaredNorm() const { return (x * x + y * y); }

  float64_t Dot(const Point2D& p) const { return p.x * x + p.y * y; }

  float64_t Cross(const Point2D& p) const {
    return this->x * p.y - this->y * p.x;
  }
  // same as Cross
  float64_t Cross2D(const Point2D& p) const {
    return this->x * p.y - this->y * p.x;
  }

  float64_t CrossSqr(const Point2D& p) const {
    auto c = Cross(p);
    return c * c;
  }

  friend std::ostream& operator<<(std::ostream& out,  // NOLINT
                                  const Point2D& point) {
    out << "(" << point.x << ", " << point.y << ")";
    return out;
  }

  Point2D operator-() const {
    Point2D p;
    p.x = -this->x;
    p.y = -this->y;
    return p;
  }

  Point2D operator+(const Point2D& b) const {
    Point2D c;
    c.x = this->x + b.x;
    c.y = this->y + b.y;
    return c;
  }

  Point2D operator-(const Point2D& b) const {
    Point2D c;
    c.x = this->x - b.x;
    c.y = this->y - b.y;
    return c;
  }

  template <typename N, std::enable_if_t<std::is_arithmetic<N>::value, int> = 0>
  Point2D operator*(const N scalar) const {
    Point2D q;
    q.x = this->x * scalar;
    q.y = this->y * scalar;
    return q;
  }

  template <typename N, std::enable_if_t<std::is_arithmetic<N>::value, int> = 0>
  Point2D operator/(const N scalar) {
    Point2D c;
    c.x = this->x / scalar;
    c.y = this->y / scalar;
    return c;
  }

  bool operator==(const Point2D& b) const {
    return FloatEqual(this->x, b.x) && FloatEqual(this->y, b.y);
  }
};

struct Point3D {
  float64_t x = 0;
  float64_t y = 0;
  float64_t z = 0;
  float32_t weight = 1.0;

  Point3D() = default;
  Point3D(float64_t _x, float64_t _y, float64_t _z, float32_t _weight = 1.0)
      : x(_x), y(_y), z(_z), weight(_weight) {}
  Point3D(float64_t _x, float64_t _y) : x(_x), y(_y) {}

  float64_t Norm() const { return std::sqrt(x * x + y * y + z * z); }
  float64_t Norm2D() const { return std::sqrt(x * x + y * y); }
  float64_t SquaredNorm() const { return (x * x + y * y + z * z); }
  float64_t SquaredNorm2D() const { return (x * x + y * y); }

  float64_t Dot(const Point3D& p) { return p.x * x + p.y * y + p.z * z; }

  float64_t Cross2D(const Point3D& p) const {
    return this->x * p.y - this->y * p.x;
  }

  friend std::ostream& operator<<(std::ostream& out,  // NOLINT
                                  const Point3D& point) {
    out << std::fixed << std::setprecision(6) << "(" << point.x << ", "
        << point.y << ", " << point.z << ", " << point.weight << ")";
    return out;
  }

  Point3D operator-() const {
    Point3D p;
    p.x = -this->x;
    p.y = -this->y;
    p.z = -this->z;
    return p;
  }

  Point3D operator+(const Point3D& b) const {
    Point3D c;
    c.x = this->x + b.x;
    c.y = this->y + b.y;
    c.z = this->z + b.z;
    return c;
  }

  Point3D operator-(const Point3D& b) const {
    Point3D c;
    c.x = this->x - b.x;
    c.y = this->y - b.y;
    c.z = this->z - b.z;
    return c;
  }

  float64_t Cross(const Point3D& p) const {
    auto cross_x = -this->z * p.y + this->y * p.z;
    auto cross_y = this->z * p.x - this->x * p.z;
    auto cross_z = -this->y * p.x + this->x * p.y;
    float64_t dir = std::fabs(cross_z);
    if (dir > 1e-10) {
      dir = cross_z / dir;
    } else {
      dir = 1.0;
    }
    return std::sqrt(cross_x * cross_x + cross_y * cross_y +
                     cross_z * cross_z) *
           dir;
  }

  float64_t CrossSqr(const Point3D& p) const {
    auto cross_x = -this->z * p.y + this->y * p.z;
    auto cross_y = this->z * p.x - this->x * p.z;
    auto cross_z = -this->y * p.x + this->x * p.y;
    return cross_x * cross_x + cross_y * cross_y + cross_z * cross_z;
  }

  template <typename N, std::enable_if_t<std::is_arithmetic<N>::value, int> = 0>
  Point3D operator*(const N scalar) const {
    Point3D q;
    q.x = this->x * scalar;
    q.y = this->y * scalar;
    q.z = this->z * scalar;
    return q;
  }

  template <typename N, std::enable_if_t<std::is_arithmetic<N>::value, int> = 0>
  Point3D operator/(const N scalar) {
    Point3D c;
    c.x = this->x / scalar;
    c.y = this->y / scalar;
    c.z = this->z / scalar;
    return c;
  }

  bool operator==(const Point3D& b) const {
    return FloatEqual(this->x, b.x) && FloatEqual(this->y, b.y) &&
           FloatEqual(this->z, b.z);
  }

  bool operator!=(const Point3D& b) const {
    return !FloatEqual(this->x, b.x) || !FloatEqual(this->y, b.y) ||
           !FloatEqual(this->z, b.z);
  }
};

class CutPoint {
 public:
  DEFINE_SMART_PTR(CutPoint)

  CutPoint() = default;
  ~CutPoint() = default;

  void SetId(const id_t id);
  id_t GetId() const;

  void SetPoint(const Point3D& pt);
  Point3D GetPoint() const;

  void SetType(const CutPointType& type);
  CutPointType GetType() const;

  void SetMainLineId(const id_t id);
  id_t GetMainLineId() const;

  void SetTargetLineId(const id_t id);
  id_t GetTargetLineId() const;

  void SetLineIds(const std::vector<id_t>& line_ids);
  std::vector<id_t> GetLineIds() const;

  void SetParams(const Eigen::Vector3d& params);
  Eigen::Vector3d GetParams() const;

  void SetExtrePoint(const std::vector<Point3D>& pts) { extre_point_ = pts; }

  std::vector<Point3D> GetExtrePoint() const { return extre_point_; }

  void SetSubPosePath(const std::vector<Sophus::SE3d>& poses) {
    sub_pose_path_ = poses;
  }
  std::vector<Sophus::SE3d> GetSubPosePath() const { return sub_pose_path_; }

  void SetStamp(const double stamp) { stamp_ = stamp; }
  double GetStamp() const { return stamp_; }
  
 private:
  id_t id_ = 0;
  // split, merge , V, Y型 ,Multi2Single, Single2Multi
  CutPointType cut_type_ = CutPointType::Unknown;

  // 切分点坐标
  Point3D point_;

  // 切分点距离最近的历史path对应的时间
  double stamp_;

  // 当前切分点,处于哪根线上
  id_t main_line_id_ = 0;

  // 对于 split, merge , V/Y型 还有 target line id
  // 目前仅支持单根target line
  id_t target_line_id_ = 0;

  // 待切分的line ids
  std::vector<id_t> line_ids_;

  // 切分线的方向向量, 方程由切分点和切分方向向量组成
  Eigen::Vector3d cut_dir_params_;

  // 切分线段上的点,size >= 2
  std::vector<Point3D> extre_point_;

  // for memory cut
  std::vector<Sophus::SE3d> sub_pose_path_;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
