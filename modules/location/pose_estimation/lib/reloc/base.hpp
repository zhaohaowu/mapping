/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： base.hpp
 *   author     ： zhaohaowu/nihongjie
 *   date       ： 2024.04
 ******************************************************************************/

#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Sophus/se2.hpp>
#include <Sophus/se3.hpp>
#include <Sophus/so2.hpp>
#include <Sophus/so3.hpp>

#include "perception-base/base/utils/log.h"

namespace hozon {
namespace mp {
namespace loc {
namespace pe {
using int8_t = signed char;
// using int16_t = signed short;
using int32_t = signed int;
// using int64_t = signed long;
using uint8_t = unsigned char;
// using uint16_t = unsigned short;
using uint32_t = unsigned int;
// using uint64_t = unsigned long;
using float32_t = float;
using float64_t = double;
using float128_t = long double;
using MatchPair = std::pair<int, int>;
using MatchPairVec = std::vector<std::pair<int, int>>;
// SemanticType
enum class SemanticType {
  NONE = 0,
  LaneLine = 1,
  StopLine = 2,
  Pole = 3,
  TrafficSign = 5
};
struct EnumClassHash {
  template <typename T>
  std::size_t operator()(T t) const {
    return static_cast<std::size_t>(t);
  }
};
using FrameMatchPairVec =
    std::unordered_map<SemanticType, MatchPairVec, EnumClassHash>;

////////////////////////// System related data structure ///////////////////////

#define DEFINE_PTR(...)                     \
  using Ptr = std::shared_ptr<__VA_ARGS__>; \
  using ConstPtr = std::shared_ptr<const __VA_ARGS__>;

#define DEFINE_UNIQUE_PTR(...)                    \
  using UniquePtr = std::unique_ptr<__VA_ARGS__>; \
  using ConstUniquePtr = std::unique_ptr<const __VA_ARGS__>;

#define DEFINE_WEAK_PTR(...)                  \
  using WeakPtr = std::weak_ptr<__VA_ARGS__>; \
  using ConstWeakPtr = std::weak_ptr<const __VA_ARGS__>;

#define DEFINE_SMART_PTR(...)    \
  DEFINE_PTR(__VA_ARGS__)        \
  DEFINE_UNIQUE_PTR(__VA_ARGS__) \
  DEFINE_WEAK_PTR(__VA_ARGS__)

#define SMARTENUM_ENUM(name, value) name = (value),
#define SMARTENUM_STRING(name, value) \
  case name:                          \
    return #name;
#define SMARTENUM_VALUE(name, value) \
  if (str == #name) return name;
#define SMARTENUM_FIND(name, value)         \
  if (str.find(#name) != std::string::npos) \
    list.push_back(true);                   \
  else                                      \
    list.push_back(false);
#define SMARTENUM_DEFINE_ENUM(typeName, lists) \
  typedef enum { lists(SMARTENUM_ENUM) } typeName
#define SMARTENUM_DEFINE_TOSTRING(typeName, lists)                \
  inline std::string typeName##ToString(const typeName& err) {    \
    switch (err) { lists(SMARTENUM_STRING) default : return ""; } \
  }
#define SMARTENUM_DEFINE_FROMSTRING(typeName, lists)             \
  inline typeName typeName##FromString(const std::string& str) { \
    lists(SMARTENUM_VALUE);                                      \
    return typeName(0);                                          \
  }
#define SMARTENUM_DEFINE_FINDTYPEFROMSTRING(typeName, lists) \
  inline std::vector<bool> Find##typeName##FromString(       \
      const std::string& str) {                              \
    std::vector<bool> list;                                  \
    lists(SMARTENUM_FIND);                                   \
    return list;                                             \
  }

// snesor locator source
// Note: backend order matters
#define LocatorType_LIST(f)                                                 \
  f(NONE, 0) f(INS, 1) f(CAN, 2) f(IMU, 3) f(VO, 4) f(REPLAY, 5) f(VIO, 6)  \
      f(SMM, 7) f(FINS, 8) f(HEADING, 9) f(GNSS, 10) f(MOCK, 11) f(WIO, 12) \
          f(LIO, 13)
SMARTENUM_DEFINE_ENUM(LocatorType, LocatorType_LIST);
SMARTENUM_DEFINE_FROMSTRING(LocatorType, LocatorType_LIST);
SMARTENUM_DEFINE_TOSTRING(LocatorType, LocatorType_LIST);
SMARTENUM_DEFINE_FINDTYPEFROMSTRING(LocatorType, LocatorType_LIST);
using STATE_SOURCE = LocatorType;

// online: subscribe ros message, run computation, record localization results
// offline: subscribe ros message, run computation
// offline_replay: replay localization results with ipc
#define EXE_MODE_LIST(f) f(online, 0) f(offline, 1) f(offline_replay, 2)
SMARTENUM_DEFINE_ENUM(ExecuteMode, EXE_MODE_LIST);
SMARTENUM_DEFINE_FROMSTRING(ExecuteMode, EXE_MODE_LIST);

/////////////////////// Geometry based data structure //////////////////////////

template <typename T>
bool FloatEqual(const T x, const T y) {
  const T max_val = std::max({1.0, std::fabs(x), std::fabs(y)});
  return std::fabs(x - y) <= std::numeric_limits<T>::epsilon() * max_val;
}

// Base geometry structure: Point2D_t
struct Point2D_t {
  float64_t x = 0;
  float64_t y = 0;
  Point2D_t() = default;
  Point2D_t(float64_t _x, float64_t _y) : x(_x), y(_y) {}

  float64_t Norm() const { return std::sqrt(x * x + y * y); }
  float64_t SquaredNorm() const { return (x * x + y * y); }

  float64_t Dot(const Point2D_t& p) const { return p.x * x + p.y * y; }
  float64_t Cross(const Point2D_t& p) const {
    return this->x * p.y - this->y * p.x;
  }

  Point2D_t operator-() const {
    Point2D_t p;
    p.x = -this->x;
    p.y = -this->y;
    return p;
  }

  Point2D_t operator+(const Point2D_t& b) const {
    Point2D_t c;
    c.x = this->x + b.x;
    c.y = this->y + b.y;
    return c;
  }

  Point2D_t operator-(const Point2D_t& b) const {
    Point2D_t c;
    c.x = this->x - b.x;
    c.y = this->y - b.y;
    return c;
  }

  template <typename N, std::enable_if_t<std::is_arithmetic<N>::value, int> = 0>
  Point2D_t operator*(const N scalar) const {
    Point2D_t q;
    q.x = this->x * scalar;
    q.y = this->y * scalar;
    return q;
  }

  template <typename N, std::enable_if_t<std::is_arithmetic<N>::value, int> = 0>
  Point2D_t operator/(const N scalar) {
    Point2D_t c;
    c.x = this->x / scalar;
    c.y = this->y / scalar;
    return c;
  }

  bool operator==(const Point2D_t& b) const {
    return FloatEqual(this->x, b.x) && FloatEqual(this->y, b.y);
  }

  friend std::ostream& operator<<(std::ostream& out,  // NOLINT
                                  const Point2D_t& point) {
    out << "(" << point.x << ", " << point.y << ")";
    return out;
  }
};

// Base geometry structure: Point3D_t
struct Point3D_t {
  float64_t x = 0;
  float64_t y = 0;
  float64_t z = 0;
  Point3D_t() = default;
  Point3D_t(float64_t _x, float64_t _y, float64_t _z = 0)
      : x(_x), y(_y), z(_z) {}

  float64_t Norm() const { return std::sqrt(x * x + y * y + z * z); }
  float64_t Norm2D() const { return std::sqrt(x * x + y * y); }
  float64_t SquaredNorm() const { return (x * x + y * y + z * z); }
  float64_t SquaredNorm2D() const { return (x * x + y * y); }

  float64_t Dot(const Point3D_t& p) const {
    return p.x * x + p.y * y + p.z * z;
  }

  Point3D_t operator-() const {
    Point3D_t p;
    p.x = -this->x;
    p.y = -this->y;
    p.z = -this->z;
    return p;
  }

  Point3D_t operator+(const Point3D_t& b) const {
    Point3D_t c;
    c.x = this->x + b.x;
    c.y = this->y + b.y;
    c.z = this->z + b.z;
    return c;
  }

  Point3D_t operator-(const Point3D_t& b) const {
    Point3D_t c;
    c.x = this->x - b.x;
    c.y = this->y - b.y;
    c.z = this->z - b.z;
    return c;
  }

  template <typename N, std::enable_if_t<std::is_arithmetic<N>::value, int> = 0>
  Point3D_t operator*(const N scalar) const {
    Point3D_t q;
    q.x = this->x * scalar;
    q.y = this->y * scalar;
    q.z = this->z * scalar;
    return q;
  }

  template <typename N, std::enable_if_t<std::is_arithmetic<N>::value, int> = 0>
  Point3D_t operator/(const N scalar) {
    Point3D_t c;
    c.x = this->x / scalar;
    c.y = this->y / scalar;
    c.z = this->z / scalar;
    return c;
  }

  bool operator==(const Point3D_t& b) const {
    return FloatEqual(this->x, b.x) && FloatEqual(this->y, b.y) &&
           FloatEqual(this->z, b.z);
  }

  friend std::ostream& operator<<(std::ostream& out,  // NOLINT
                                  const Point3D_t& point) {
    out << "(" << point.x << ", " << point.y << ", " << point.z << ")";
    return out;
  }
};

// Position in the East North Up (ENU) coordinate system, origin defined by map
struct PointENU_t {
  float64_t x = 0;  // East from the origin, in meters.
  float64_t y = 0;  // North from the origin, in meters.
  float64_t z = 0;  // Up from the WGS-84 ellipsoid, in meters.
  PointENU_t() = default;
  PointENU_t(float64_t _x, float64_t _y, float64_t _z = 0)
      : x(_x), y(_y), z(_z) {}

  friend PointENU_t operator+(const PointENU_t& a, const PointENU_t& b) {
    PointENU_t c;
    c.x = a.x + b.x;
    c.y = a.y + b.y;
    c.z = a.z + b.z;
    return c;
  }
  friend PointENU_t operator-(const PointENU_t& a, const PointENU_t& b) {
    PointENU_t c;
    c.x = a.x - b.x;
    c.y = a.y - b.y;
    c.z = a.z - b.z;
    return c;
  }

  friend std::ostream& operator<<(std::ostream& out,  // NOLINT
                                  const PointENU_t& point) {
    out << "(" << point.x << ", " << point.y << ", " << point.z << ")";
    return out;
  }
};

// Position in the WGS84 reference ellipsoid coordinate system.
struct PointLLH_t {
  // Longitude in degrees, ranging from -180 to 180.
  float64_t lon = 0;
  // Latitude in degrees, ranging from -90 to 90.
  float64_t lat = 0;
  // WGS-84 ellipsoid height in meters.
  float64_t height = 0;
  PointLLH_t() = default;
  PointLLH_t(float64_t _lon, float64_t _lat, float64_t _height)
      : lon(_lon), lat(_lat), height(_height) {}

  friend std::ostream& operator<<(std::ostream& out,  // NOLINT
                                  const PointLLH_t& point) {
    out << "(" << point.lon << ", " << point.lat << ", " << point.height << ")";
    return out;
  }
};

// A unit Quaternion_t also represents a spatial rotation.
// Most of the time, qw = sqrt(1 - qx * qx - qy * qy - qz * qz).
struct Quaternion_t {
  float64_t qx{0.0};  // quaternion x
  float64_t qy{0.0};  // quaternion y
  float64_t qz{0.0};  // quaternion z
  float64_t qw{0.0};  // quaternion w
};

// A EulerAngles_t that represents a spatial rotation.
struct EulerAngles_t {
  float64_t roll{0.0};   // Right-handed rotation around x-axis, unit: rad
  float64_t pitch{0.0};  // Right-handed rotation around y‑axis, unit: rad
  float64_t yaw{0.0};    // Right-handed rotation around z-axis, unit: rad
};

// navigation state
struct NavState {
  uint64_t timestamp{0};                         // timestamp
  uint8_t nav_status;                            // msf status
  STATE_SOURCE state_source;                     // state source
  PointLLH_t origin;                             // origin LLA
  Sophus::SE3d pose;                             // position and rotation
  Eigen::Vector3d linear_speed{0, 0, 0};         // velocity
  Eigen::Vector3d linear_acceleration{0, 0, 0};  // acceleration
  Eigen::Vector3d angular_speed{0, 0, 0};        // angular speed
  Eigen::Matrix<double, 6, 6> pose_cov;          // pose covariance
  double gnss_bias;                              // gnss bias
  double gnss_bias_cov;                          // gnss bias covariance
};

// odometry state
struct OdomState {
  uint64_t timestamp{0};                         // timestamp
  uint8_t odom_status;                           // odom status
  STATE_SOURCE state_source;                     // state source
  uint64_t origin_id;                            // origin id
  Sophus::SE3d pose;                             // position and rotation
  Eigen::Vector3d linear_speed{0, 0, 0};         // velocity
  Eigen::Vector3d linear_acceleration{0, 0, 0};  // acceleration
  Eigen::Vector3d angular_speed{0, 0, 0};        // angular speed
  Eigen::Matrix<double, 6, 6> pose_cov;          // pose covariance
};

enum class NavStatus {
  INVALID = 0,     // invalid
  INITIALIZING,    // initializing, only as system is starting
  FATAL_ACCURACY,  // fatal accuracy, location is divergency
  LOW_ACCURACY,    // low accuracy, road level localization accuracy
  MID_ACCURACY,    // middle accuracy, lane level localization accuracy
  HIGH_ACCURACY    // high accuracy, 10cm level localization accuracy
};
static inline std::string GetNavStatusStr(const NavStatus& status) {
  std::string status_str;
  static const std::map<NavStatus, std::string> NavStatusTable{
      {NavStatus::INVALID, "INVALID"},
      {NavStatus::INITIALIZING, "INITIALIZING"},
      {NavStatus::FATAL_ACCURACY, "FATAL_ACCURACY"},
      {NavStatus::LOW_ACCURACY, "LOW_ACCURACY"},
      {NavStatus::MID_ACCURACY, "MID_ACCURACY"},
      {NavStatus::HIGH_ACCURACY, "HIGH_ACCURACY"}};
  const auto& it = NavStatusTable.find(status);
  if (it != NavStatusTable.end()) {
    status_str = it->second;
  }
  return status_str;
}
static inline std::string GetNavStatusShortStr(const NavStatus& status) {
  std::string status_str;
  static const std::map<NavStatus, std::string> NavStatusTableShort{
      {NavStatus::INVALID, "INV"},       {NavStatus::INITIALIZING, "INIT"},
      {NavStatus::FATAL_ACCURACY, "FA"}, {NavStatus::LOW_ACCURACY, "LA"},
      {NavStatus::MID_ACCURACY, "MA"},   {NavStatus::HIGH_ACCURACY, "HA"}};
  const auto& it = NavStatusTableShort.find(status);
  if (it != NavStatusTableShort.end()) {
    status_str = it->second;
  }
  return status_str;
}
static inline NavStatus GetNavStatusFromStr(const std::string& status_str) {
  NavStatus status;  // NOLINT
  static const std::map<std::string, NavStatus> NavStatusTable{
      {"INVALID", NavStatus::INVALID},
      {"INITIALIZING", NavStatus::INITIALIZING},
      {"FATAL_ACCURACY", NavStatus::FATAL_ACCURACY},
      {"LOW_ACCURACY", NavStatus::LOW_ACCURACY},
      {"MID_ACCURACY", NavStatus::MID_ACCURACY},
      {"HIGH_ACCURACY", NavStatus::HIGH_ACCURACY}};
  const auto& it = NavStatusTable.find(status_str);
  if (it != NavStatusTable.end()) {
    status = it->second;
  }
  return status;
}
static inline uint8_t GetNavStatusNum(const NavStatus& status) {
  switch (status) {
    case NavStatus::INVALID:
      return 0;
    case NavStatus::INITIALIZING:
      return 1;
    case NavStatus::FATAL_ACCURACY:
      return 2;
    case NavStatus::LOW_ACCURACY:
      return 3;
    case NavStatus::MID_ACCURACY:
      return 4;
    case NavStatus::HIGH_ACCURACY:
      return 5;
    default:
      return 0;
  }
  return 0;
}
static inline NavStatus GetNavStatusEnum(uint8_t num) {
  switch (num) {
    case 0:
      return NavStatus::INVALID;
    case 1:
      return NavStatus::INITIALIZING;
    case 2:
      return NavStatus::FATAL_ACCURACY;
    case 3:
      return NavStatus::LOW_ACCURACY;
    case 4:
      return NavStatus::MID_ACCURACY;
    case 5:
      return NavStatus::HIGH_ACCURACY;
    default:
      return NavStatus::INVALID;
  }
  return NavStatus::INVALID;
}

// odometry (dead reckoning) status related
enum class OdomStatus { INVALID = 0, FAILED = 1, GOOD = 2 };
static inline std::string GetOdomStatusStr(const OdomStatus& status) {
  std::string status_str;
  static const std::map<OdomStatus, std::string> OdomStatusTable{
      {OdomStatus::INVALID, "INVALID"},
      {OdomStatus::FAILED, "FAILED"},
      {OdomStatus::GOOD, "GOOD"}};
  const auto& it = OdomStatusTable.find(status);
  if (it != OdomStatusTable.end()) {
    status_str = it->second;
  }
  return status_str;
}
static inline OdomStatus GetOdomStatusFromStr(const std::string& status_str) {
  OdomStatus status;  // NOLINT
  static const std::map<std::string, OdomStatus> OdomStatusTable{
      {"INVALID", OdomStatus::INVALID},
      {"FAILED", OdomStatus::FAILED},
      {"GOOD", OdomStatus::GOOD}};
  const auto& it = OdomStatusTable.find(status_str);
  if (it != OdomStatusTable.end()) {
    status = it->second;
  }
  return status;
}
static inline uint8_t GetOdomStatusNum(const OdomStatus& status) {
  switch (status) {
    case OdomStatus::INVALID:
      return 0;
    case OdomStatus::FAILED:
      return 1;
    case OdomStatus::GOOD:
      return 2;
    default:
      return 0;
  }
  return 0;
}
static inline OdomStatus GetOdomStatusEnum(uint8_t num) {
  switch (num) {
    case 0:
      return OdomStatus::INVALID;
    case 1:
      return OdomStatus::FAILED;
    case 2:
      return OdomStatus::GOOD;
    default:
      return OdomStatus::INVALID;
  }
  return OdomStatus::INVALID;
}

class MatchIndex {
 public:
  DEFINE_SMART_PTR(MatchIndex)

  MatchIndex() = default;
  ~MatchIndex() = default;

  void Reset() { match_pairs_.clear(); }

  void AddSemanticMatch(SemanticType semantic_type,
                        const MatchPairVec& match_pairs) {
    auto& vec = match_pairs_[semantic_type];
    vec.insert(vec.end(), match_pairs.begin(), match_pairs.end());
  }
  void AddSemanticMatch(SemanticType semantic_type,
                        const MatchPair& match_pair) {
    auto& vec = match_pairs_[semantic_type];
    vec.emplace_back(match_pair);
  }

  bool GetSemanticMatch(SemanticType semantic_type,
                        MatchPairVec* match_pairs) const {
    auto iter = match_pairs_.find(semantic_type);
    if (iter == match_pairs_.end()) return false;
    *match_pairs = iter->second;
    return true;
  }

  size_t GetSemanticMatchNum(SemanticType semantic_type) const {
    auto iter = match_pairs_.find(semantic_type);
    if (iter == match_pairs_.end()) return 0;
    return iter->second.size();
  }

  FrameMatchPairVec GetAllSemanticMatch() const { return match_pairs_; }

  MatchPairVec GetAllMatchInOneVec() const {
    MatchPairVec vec;
    for (const auto& item : match_pairs_) {
      vec.insert(vec.end(), item.second.begin(), item.second.end());
    }
    return vec;
  }

 private:
  FrameMatchPairVec match_pairs_{};
};

// project check whether one point in line range
template <typename T>
bool IsPointInLineRange(const T& pt, const T& start_pt, const T& end_pt) {
  T direct = end_pt - start_pt;
  direct = direct / direct.Norm();
  double proj1 = (pt - start_pt).Dot(direct);
  double proj2 = (pt - end_pt).Dot(direct);
  return proj1 >= 0 && proj2 <= 0;
}

// binary search two nearest points in one line
template <typename T>
bool BinarySearchTwoNearestPoints(const T& search_pt,
                                  const std::vector<T>& points, size_t* index1,
                                  size_t* index2) {
  if (points.size() < 2 || !index1 || !index2) {
    return false;
  }

  T direct = points.back() - points.front();
  direct = direct / direct.Norm();

  double pt_proj = (search_pt - points.front()).Dot(direct);

  int left = 0;
  int right = static_cast<int>(points.size() - 1);
  while (right - left > 1) {
    int mid = left + (right - left) / 2;
    double proj = (points[mid] - points.front()).Dot(direct);
    if (proj < pt_proj) {
      left = mid;
    } else {
      right = mid;
    }
  }
  *index1 = left;
  *index2 = right;
  return true;
}

// point to line distance
template <typename T>
float64_t Point2LineDist(const T& p, const T& q1, const T q2,
                         bool signed_dist = false) {
  T line_q1q2 = q2 - q1;
  T line_q1p = p - q1;
  line_q1q2 = line_q1q2 / line_q1q2.Norm();
  float64_t project_dist = line_q1p.Dot(line_q1q2);
  if (line_q1p.Norm() <= project_dist) {
    return 0.0;
  }

  double sign = 1.0;
  if (signed_dist) {
    if (line_q1q2.Cross(line_q1p) < 0) {
      sign = -1;
    }
  }
  return sign * std::sqrt(line_q1p.SquaredNorm() - project_dist * project_dist);
}

// point to line 2D(X-Y) distance
template <typename T>
float64_t Point2LineDist2D(const T& p, const T& q1, const T q2,
                           bool signed_dist = false) {
  Point2D_t p_2d(p.x, p.y);
  Point2D_t q1_2d(q1.x, q1.y);
  Point2D_t q2_2d(q2.x, q2.y);
  return Point2LineDist(p_2d, q1_2d, q2_2d, signed_dist);
}

// line to line 2d(X-Y) distance
template <typename T>
float64_t Line2LineDistance2D(const std::vector<T>& src_line,
                              const std::vector<T>& tgt_line,
                              int* valid_num = nullptr,
                              bool signed_dist = false) {
  if (src_line.empty() || tgt_line.empty()) {
    if (valid_num) *valid_num = 0;
    return std::numeric_limits<double>::max();
  }

  int counter = 0;
  double ave_distance = 0.0;
  for (const auto& pt : src_line) {
    // check whether in target line view range
    if (!IsPointInLineRange(pt, tgt_line.front(), tgt_line.back())) {
      continue;
    }
    // serach the nearest two points in target line
    size_t idx1 = 0;
    size_t idx2 = 0;
    if (!BinarySearchTwoNearestPoints(pt, tgt_line, &idx1, &idx2)) {
      continue;
    }
    ave_distance +=
        Point2LineDist2D(pt, tgt_line[idx1], tgt_line[idx2], signed_dist);
    ++counter;
  }
  if (valid_num) {
    *valid_num = counter;
  }
  ave_distance = counter == 0 ? std::numeric_limits<double>::max()
                              : ave_distance / counter;
  return ave_distance;
}

struct MatchPairHash {
  size_t operator()(const std::pair<int, int>& idx) const {
    size_t seed = 0;
    hash_combine(&seed, idx.first);
    hash_combine(&seed, idx.second);
    return seed;
  }
  template <class T>
  void hash_combine(size_t* seed, const T& v) const {
    std::hash<T> hasher;
    *seed ^= hasher(v) + 0x9e3779b9 + (*seed << 6) + (*seed >> 2);
  }
};

// define semantic type for grid match
enum class MatchSemanticType {
  Unknown = 0,
  DashedLine = 1,
  SolidLine = 2,
  RoadSideLine = 3,
  Sign = 4,
  Pole = 5,
  Other = 255,
};

// data struct definition for internal use
struct PerceptPoint {
  PerceptPoint(const Point2D_t& pt, Eigen::Matrix2d cov, MatchSemanticType type)
      : point(pt), cov(std::move(cov)), type(type) {}
  PerceptPoint(const Point2D_t& pt, Eigen::Matrix2d cov, MatchSemanticType type,
               int obs_cnt)
      : point(pt), cov(std::move(cov)), type(type), observ_cnt(obs_cnt) {}
  Point2D_t point;
  Eigen::Matrix2d cov;
  MatchSemanticType type;
  int observ_cnt{1};
  double weight{};
};
struct Point2DWithCov {
  Point2DWithCov() = default;
  Point2DWithCov(const Point2D_t& pt, Eigen::Matrix2d cov)
      : point(pt), cov(std::move(cov)) {}
  Point2D_t point;
  Eigen::Matrix2d cov;
};

struct MapPoint {
  MapPoint(int id, const Point2D_t& pt) : id(id), point(pt) {}
  MapPoint(int id, const Point2D_t& pt, double heading)
      : id(id), point(pt), local_heading(heading) {}
  int id;
  Point2D_t point;
  double local_heading{0};
};

inline double CalculateDistbtwTwoPerceptPoints(
    const std::vector<Point2DWithCov>& percept_points1,
    const std::vector<Point2DWithCov>& percept_points2) {
  int percept_points1_size = static_cast<int>(percept_points1.size());
  int percept_points2_size = static_cast<int>(percept_points2.size());
  std::vector<Point2D_t> src_points;
  std::vector<Point2D_t> tgt_points;
  src_points.reserve(percept_points1_size);
  tgt_points.reserve(percept_points2_size);
  for (const auto& percept_point : percept_points1) {
    src_points.emplace_back(percept_point.point);
  }
  for (const auto& percept_point : percept_points2) {
    tgt_points.emplace_back(percept_point.point);
  }
  int valid_num = 0;
  double dist = Line2LineDistance2D(src_points, tgt_points, &valid_num, true);
  if (valid_num == 0) {
    return 1e10;
  }
  return dist;
}

inline double FourDegreePolyFunc(const Eigen::Vector4d& line_param, double x) {
  double y = 0;
  Eigen::Vector4d xs(x * x * x, x * x, x, 1);
  y = line_param.transpose() * xs;
  return y;
}

inline double CalculateDistbtwTwoPolyFitLine(
    const Eigen::Vector4d& line1_param, const Eigen::Vector4d& line2_param) {
  double y_mean = 0.;
  double threshold = 10.0;
  double step = 2.0;
  int points_size = static_cast<int>(std::floor((2 * threshold) / step) + 1);
  std::vector<double> y_diffs;
  y_diffs.reserve(points_size);
  for (double pt_x = -threshold; pt_x <= threshold; pt_x += step) {
    double pt_y1 = FourDegreePolyFunc(line1_param, pt_x);
    double pt_y2 = FourDegreePolyFunc(line2_param, pt_x);
    double y_diff = pt_y1 - pt_y2;
    y_diffs.emplace_back(y_diff);
    y_mean += y_diff;
  }
  y_mean /= points_size;
  return y_mean;
}

inline double CalculateDistbtwPerceptPointsMapLine(
    const std::vector<Point2DWithCov>& percept_points,
    const Eigen::Vector4d& map_line_param, const Sophus::SE2d& transform) {
  if (percept_points.empty()) {
    return 1e9;
  }

  int points_size = static_cast<int>(percept_points.size());
  std::vector<double> y_diffs;
  std::vector<double> weights;
  y_diffs.reserve(points_size);
  weights.reserve(points_size);

  double y_mean = 0.;
  double w_sum = 0.;
  for (const auto& percept_point : percept_points) {
    if (percept_point.point.x < -10. || percept_point.point.x > 20.) {
      continue;
    }
    Eigen::Vector2d p{percept_point.point.x, percept_point.point.y};
    Eigen::Vector2d t = transform * p;
    Point2D_t transformed_point;
    transformed_point.x = t.x();
    transformed_point.y = t.y();
    double map_y = FourDegreePolyFunc(map_line_param, transformed_point.x);
    double y_diff = transformed_point.y - map_y;
    double weight = 1.0 / std::max(percept_point.cov.diagonal().norm(), 1e-4);
    y_diffs.emplace_back(y_diff);
    weights.emplace_back(weight);
    w_sum += weight;
  }
  if (y_diffs.empty()) {
    return 1e9;
  }

  for (auto& weight : weights) {
    weight /= w_sum;
  }
  for (int i = 0; i < static_cast<int>(y_diffs.size()); i++) {
    y_mean += weights[i] * y_diffs[i];
  }
  return y_mean;
}

struct PerceptPointsToMapLineDist {
  DEFINE_SMART_PTR(PerceptPointsToMapLineDist)
  double GetDistbtwTwoPerceptPoints(
      int percept_id1, const std::vector<Point2DWithCov>& percept_points1,
      int percept_id2, const std::vector<Point2DWithCov>& percept_points2) {
    MatchPair match_pair(percept_id1, percept_id2);
    if (percept_points_dists_.count(match_pair) != 0U) {
      return percept_points_dists_.at(match_pair);
    }

    MatchPair inverse_match_pair(percept_id2, percept_id1);
    if (percept_points_dists_.count(inverse_match_pair) != 0U) {
      return -percept_points_dists_.at(inverse_match_pair);
    }

    double dist =
        CalculateDistbtwTwoPerceptPoints(percept_points1, percept_points2);
    percept_points_dists_.insert({match_pair, dist});
    return dist;
  }

  double GetDistbtwTwoMapLines(int map_id1,
                               const Eigen::Vector4d& map_line_params1,
                               int map_id2,
                               const Eigen::Vector4d& map_line_params2) {
    MatchPair match_pair(map_id1, map_id2);
    if (map_line_dists_.count(match_pair) != 0U) {
      return map_line_dists_.at(match_pair);
    }

    MatchPair inverse_match_pair(map_id2, map_id1);
    if (map_line_dists_.count(inverse_match_pair) != 0U) {
      return -map_line_dists_.at(inverse_match_pair);
    }

    double dist =
        CalculateDistbtwTwoPolyFitLine(map_line_params1, map_line_params2);
    map_line_dists_.insert({match_pair, dist});
    return dist;
  }

  double GetDistbtwPerceptPointsMapLine(
      int percept_id, const std::vector<Point2DWithCov>& percept_points,
      int map_id, const Eigen::Vector4d& map_line_param, double heading,
      double search_y = 0) {
    MatchPair match_pair(percept_id, map_id);
    if ((multi_heading_percept_points_to_map_line_dists_.count(heading) !=
         0U) &&
        (multi_heading_percept_points_to_map_line_dists_[heading].count(
             match_pair) != 0U)) {
      return multi_heading_percept_points_to_map_line_dists_[heading]
                                                            [match_pair] +
             search_y;
    }

    Sophus::SE2d T_map_percept(heading, Eigen::Vector2d::Zero());
    auto dist = CalculateDistbtwPerceptPointsMapLine(
        percept_points, map_line_param, T_map_percept);
    auto& percept_points_to_map_line_dists =
        multi_heading_percept_points_to_map_line_dists_[heading];
    percept_points_to_map_line_dists.insert({match_pair, dist});

    return dist + search_y;
  }

 private:
  std::unordered_map<MatchPair, double, MatchPairHash> percept_points_dists_;
  std::unordered_map<MatchPair, double, MatchPairHash> map_line_dists_;
  std::unordered_map<double,
                     std::unordered_map<MatchPair, double, MatchPairHash>>
      multi_heading_percept_points_to_map_line_dists_;
};

// transform points
inline std::vector<Point3D_t> TransformPoints(
    const std::vector<Point3D_t>& points, const Sophus::SE3d& pose) {
  std::vector<Point3D_t> trans_pts;
  trans_pts.reserve(points.size());
  for (const auto& pt : points) {
    Eigen::Vector3d p{pt.x, pt.y, pt.z};
    Eigen::Vector3d t = pose * p;
    trans_pts.emplace_back(t.x(), t.y(), t.z());
  }
  return trans_pts;
}

inline double NormalizeAngleDiff(double diff) {
  if (diff > M_PI) {
    diff -= 2 * M_PI;
  }
  if (diff < -M_PI) {
    diff += 2 * M_PI;
  }
  return diff;
}

inline bool PolyLineLeastSquareFitting(const std::vector<Point3D_t>& points,
                                       const std::vector<double>& weights,
                                       int order, Eigen::VectorXd* line_param,
                                       double* residual_mean,
                                       double* residual_max) {
  if (points.size() < 2) {
    HLOG_ERROR << "ORDER " << order << "points.size() < 2";
    return false;
  }
  if (order > static_cast<int>(points.size()) - 1) {
    HLOG_ERROR << "order > static_cast<int>(points.size()) - 1";
    return false;
  }
  if (points.size() != weights.size()) {
    HLOG_ERROR << "points.size() " << points.size() << " != weights.size() "
               << weights.size();
    return false;
  }

  Eigen::MatrixXd A(points.size(), order + 1);
  Eigen::VectorXd b(points.size());

  // construct A and b, fitting polyline: y = a*x^3 + b*x^2 + c*x + d
  for (int i = 0; i < static_cast<int>(points.size()); ++i) {
    A(i, order) = weights[i];
  }
  for (int i = 0; i < static_cast<int>(points.size()); ++i) {
    for (int j = order - 1; j >= 0; --j) {
      A(i, j) = std::pow(points[i].x, order - j) * weights[i];
      // A(i, j) = std::pow(points[i].x, order - j - 1) * weights[i];
    }
    b(i) = points[i].y * weights[i];
  }

  *line_param = (A.transpose() * A).llt().solve(A.transpose() * b);

  // evaluate residuals if needed
  auto PolyFunc = [](const Eigen::VectorXd& param, double x) {
    double y = 0;
    int param_size = static_cast<int>(param.size());
    for (int i = 0; i < param_size; ++i) {
      y += param[i] * std::pow(x, param_size - 1 - i);
      // y += param[i] * x / std::pow(2, i+1);
    }
    return y;
  };
  if (residual_mean != nullptr || residual_max != nullptr) {
    double mean_residual = 0;
    double max_residual = 0;
    for (const auto& pt : points) {
      double res = std::fabs(pt.y - PolyFunc(*line_param, pt.x));
      mean_residual += res;
      if (res > max_residual) {
        max_residual = res;
      }
    }
    mean_residual /= static_cast<double>(points.size());
    if (residual_mean != nullptr) *residual_mean = mean_residual;
    if (residual_max != nullptr) *residual_max = max_residual;
  }
  return true;
}

}  // namespace pe
}  // namespace loc
}  // namespace mp
}  // namespace hozon
