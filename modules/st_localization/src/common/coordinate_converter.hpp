/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Chen Shengjie <chenshengjie@sensetime.com>
 * Zhao Ming <zhaoming@senseauto.com>
 */
#pragma once

#include <Eigen/Core>
#include <type_traits>

#include "localization/common/log.hpp"
#include "localization/data_type/base.hpp"

namespace senseAD {
namespace localization {

class CoordinateConverter {
 public:
  CoordinateConverter(CoordinateConverter const&) = delete;

  CoordinateConverter& operator=(CoordinateConverter const&) = delete;

  static CoordinateConverter* GetInstance();

  void SetOrigin(const PointLLH_t& origin);

  adLocStatus_t GetOrigin(PointLLH_t* origin, bool is_old_origin = false) const;

  adLocStatus_t LLA2ENU(const PointLLH_t& lla, PointENU_t* enu,
                        bool is_old_origin = false) const;

  adLocStatus_t ENU2LLA(const PointENU_t& enu, PointLLH_t* lla,
                        bool is_old_origin = false) const;

  // convert enu position from old origin to new origin
  template <typename T>
  adLocStatus_t ConvertENUToNewOrigin(const T& old_enu, T* new_enu) {
    if (nullptr == new_enu) {
      return LOC_NULL_PTR;
    }
    if (!has_origin_) {
      LC_LERROR(COMMON) << "CoordinateConverter doesn't have origin yet";
      return LOC_LOCALIZATION_ERROR;
    }
    PointENU_t enu(old_enu.x, old_enu.y, old_enu.z);
    PointLLH_t lla;
    if (LOC_SUCCESS != ENU2LLA(enu, &lla, true)) {
      return LOC_LOCALIZATION_ERROR;
    }
    if (LOC_SUCCESS != LLA2ENU(lla, &enu, false)) {
      return LOC_LOCALIZATION_ERROR;
    }
    *new_enu = T(enu.x, enu.y, enu.z);
    return LOC_SUCCESS;
  }
  adLocStatus_t ConvertENUToNewOrigin(const Eigen::Vector3d& old_enu,
                                      Eigen::Vector3d* new_enu) {
    PointENU_t p_enu(old_enu.x(), old_enu.y(), old_enu.z());
    if (LOC_SUCCESS != ConvertENUToNewOrigin(p_enu, &p_enu)) {
      return LOC_LOCALIZATION_ERROR;
    }
    *new_enu = Eigen::Vector3d(p_enu.x, p_enu.y, p_enu.z);
    return LOC_SUCCESS;
  }

  // Compute distance by lla, return m
  double ComputeDistanceByLLH(const PointLLH_t& p1, const PointLLH_t& p2);

 private:
  CoordinateConverter() = default;

  // Convert Geodetic coordinates to ECEF coordinates.
  void LLA2ECEF(const PointLLH_t& lla, PointENU_t* ecef) const;

  // Convert ECEF coordinates to Geodetic coordinates.
  void ECEF2LLA(const PointENU_t& ecef, PointLLH_t* lla) const;

  // Converts ECEF coordinate position into local-tangent-plane ENU.
  adLocStatus_t ECEF2ENU(const PointENU_t& ecef, PointENU_t* enu,
                         bool is_old_origin = false) const;

  // Converts ENU(East/North/Up) to ECEF coordinates
  adLocStatus_t ENU2ECEF(const PointENU_t& enu, PointENU_t* ecef,
                         bool is_old_origin = false) const;

  inline float64_t Rad2Deg(float64_t radians) const {
    return (radians / M_PI) * 180.0;
  }

  inline float64_t Deg2Rad(float64_t degrees) const {
    return (degrees / 180.0) * M_PI;
  }

  PointLLH_t origin_, origin_old_;
  PointENU_t origin_ecef_m_, origin_ecef_m_old_;
  Eigen::Matrix3d matrix_ecef_to_enu_, matrix_ecef_to_enu_old_;
  Eigen::Matrix3d matrix_enu_to_ecef_, matrix_enu_to_ecef_old_;

  bool has_origin_ = false;

  // Geodetic system parameters
  // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion
  static constexpr float64_t kSemimajorAxis = 6378137;
  static constexpr float64_t kSemiminorAxis = 6356752.3142;
  static constexpr float64_t kFirstEccentricitySquared = 6.69437999014 * 0.001;
  static constexpr float64_t kSecondEccentricitySquared = 6.73949674228 * 0.001;
  static constexpr float64_t kFlattening = 1 / 298.257223563;
  static constexpr float64_t kEarthRadius = 6378.137;
};

}  // namespace localization
}  // namespace senseAD
