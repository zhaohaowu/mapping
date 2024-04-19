/*
 * Copyright (C) 2017-2020 by SenseTime Group Limited. All rights reserved.
 * YU Chendi <yuchendi@sensetime.com>
 * Chen Shengjie <chenshengjie@sensetime.com>
 * Wang Xiaofeng <wangxiaofeng@senseauto.com>
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <type_traits>
#include <vector>

#include <Sophus/se3.hpp>

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

using SE3d = Sophus::SE3<double>;
using SO3d = Sophus::SO3<double>;

namespace senseAD {
namespace localization {

////////////////////////// System related data structure ///////////////////////

#define SMARTENUM_ENUM(name, value) name = value,
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

// status for function return
enum adLocStatus_t {
  LOC_SUCCESS = 0,
  LOC_INVALID = -1,
  LOC_INVALID_PARAM = -2,
  LOC_ALLOC_MEMORY_FAILED = -3,
  LOC_INTERNAL_ERROR = -4,
  LOC_NULL_PTR = -5,
  LOC_LOCALIZATION_ERROR = -6,
  LOC_INIT_ERROR = -7,
  LOC_TIME_DISORDER = -8,
  LOC_TIME_AHEAD = -9,
  LOC_TIME_DELAY = -10,
  LOC_FILE_NOT_FOUND = -11,
  LOC_OUT_OF_RANGE = -12,
  LOC_NOT_INIT = -13
};
static inline std::string StatusToString(const adLocStatus_t status) {
  switch (status) {
    case adLocStatus_t::LOC_SUCCESS:
      return "LOC_SUCCESS";
    case adLocStatus_t::LOC_INVALID:
      return "LOC_INVALID";
    case adLocStatus_t::LOC_INVALID_PARAM:
      return "LOC_INVALID_PARAM";
    case adLocStatus_t::LOC_ALLOC_MEMORY_FAILED:
      return "LOC_ALLOC_MEMORY_FAILED";
    case adLocStatus_t::LOC_INTERNAL_ERROR:
      return "LOC_INTERNAL_ERROR";
    case adLocStatus_t::LOC_NULL_PTR:
      return "LOC_NULL_PTR";
    case adLocStatus_t::LOC_LOCALIZATION_ERROR:
      return "LOC_LOCALIZATION_ERROR";
    case adLocStatus_t::LOC_INIT_ERROR:
      return "LOC_INIT_ERROR";
    case adLocStatus_t::LOC_TIME_DISORDER:
      return "LOC_TIME_DISORDER";
    case adLocStatus_t::LOC_TIME_AHEAD:
      return "LOC_TIME_AHEAD";
    case adLocStatus_t::LOC_TIME_DELAY:
      return "LOC_TIME_DELAY";
    case adLocStatus_t::LOC_FILE_NOT_FOUND:
      return "LOC_FILE_NOT_FOUND";
    case adLocStatus_t::LOC_OUT_OF_RANGE:
      return "LOC_OUT_OF_RANGE";
    case adLocStatus_t::LOC_NOT_INIT:
      return "LOC_NOT_INIT";
    default:
      return "";
  }
}

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
  SE3d pose;                                     // position and rotation
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
  SE3d pose;                                     // position and rotation
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
  std::string status_str = "";
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
  std::string status_str = "";
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
  NavStatus status;
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
  std::string status_str = "";
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
  OdomStatus status;
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

}  // namespace localization
}  // namespace senseAD
