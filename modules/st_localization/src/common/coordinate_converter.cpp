/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Chen Shengjie <chenshengjie@sensetime.com>
 * Zhao Ming <zhaoming@senseauto.com>
 */

#include "common/coordinate_converter.hpp"

namespace senseAD {
namespace localization {

CoordinateConverter* CoordinateConverter::GetInstance() {
  static CoordinateConverter converter;
  return &converter;
}

void CoordinateConverter::SetOrigin(const PointLLH_t& origin) {
  origin_old_ = origin_;
  origin_ = origin;

  // Compute ECEF of ENU origin
  origin_ecef_m_old_ = origin_ecef_m_;
  LLA2ECEF(origin_, &origin_ecef_m_);

  // Compute ECEF<-->ENU's matrices
  // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion
  matrix_ecef_to_enu_old_ = matrix_ecef_to_enu_;
  const float64_t sLon = sin(Deg2Rad(origin_.lon));
  const float64_t sLat = sin(Deg2Rad(origin_.lat));
  const float64_t cLon = cos(Deg2Rad(origin_.lon));
  const float64_t cLat = cos(Deg2Rad(origin_.lat));
  matrix_ecef_to_enu_(0, 0) = -sLon;
  matrix_ecef_to_enu_(0, 1) = cLon;
  matrix_ecef_to_enu_(0, 2) = 0.0;
  matrix_ecef_to_enu_(1, 0) = -sLat * cLon;
  matrix_ecef_to_enu_(1, 1) = -sLat * sLon;
  matrix_ecef_to_enu_(1, 2) = cLat;
  matrix_ecef_to_enu_(2, 0) = cLat * cLon;
  matrix_ecef_to_enu_(2, 1) = cLat * sLon;
  matrix_ecef_to_enu_(2, 2) = sLat;

  matrix_enu_to_ecef_old_ = matrix_enu_to_ecef_;
  matrix_enu_to_ecef_ = matrix_ecef_to_enu_.transpose();

  has_origin_ = true;
}

adLocStatus_t CoordinateConverter::GetOrigin(PointLLH_t* origin,
                                             bool is_old_origin) const {
  if (nullptr == origin) {
    return LOC_NULL_PTR;
  }
  if (!has_origin_) {
    LC_LERROR(COMMON) << "CoordinateConverter doesn't have origin yet.";
    return LOC_LOCALIZATION_ERROR;
  }

  if (is_old_origin) {
    *origin = origin_old_;
  } else {
    *origin = origin_;
  }

  return LOC_SUCCESS;
}

adLocStatus_t CoordinateConverter::LLA2ENU(const PointLLH_t& lla,
                                           PointENU_t* enu,
                                           bool is_old_origin) const {
  if (nullptr == enu) {
    return LOC_NULL_PTR;
  }
  if (!has_origin_) {
    LC_LERROR(COMMON) << "CoordinateConverter doesn't have origin yet";
    return LOC_LOCALIZATION_ERROR;
  }

  PointENU_t ecef;
  LLA2ECEF(lla, &ecef);
  return ECEF2ENU(ecef, enu, is_old_origin);
}

adLocStatus_t CoordinateConverter::ENU2LLA(const PointENU_t& enu,
                                           PointLLH_t* lla,
                                           bool is_old_origin) const {
  if (nullptr == lla) {
    return LOC_NULL_PTR;
  }
  if (!has_origin_) {
    LC_LERROR(COMMON) << "CoordinateConverter doesn't have origin yet";
    return LOC_LOCALIZATION_ERROR;
  }

  PointENU_t ecef;
  if (LOC_SUCCESS != ENU2ECEF(enu, &ecef, is_old_origin)) {
    return LOC_LOCALIZATION_ERROR;
  }
  ECEF2LLA(ecef, lla);
  return LOC_SUCCESS;
}

double CoordinateConverter::ComputeDistanceByLLH(const PointLLH_t& p1,
                                                 const PointLLH_t& p2) {
  double lat1 = Deg2Rad(p1.lat);
  double lon1 = Deg2Rad(p1.lon);
  double lat2 = Deg2Rad(p2.lat);
  double lon2 = Deg2Rad(p2.lon);

  double d_lat = lat1 - lat2;
  double d_lon = lon1 - lon2;

  double d_sigma =
      2 * asin(sqrt(pow(sin(d_lat / 2), 2) +
                    cos(lat1) * cos(lat2) * pow(sin(d_lon / 2), 2)));

  return kEarthRadius * d_sigma * 1000;
}

void CoordinateConverter::LLA2ECEF(const PointLLH_t& lla,
                                   PointENU_t* ecef) const {
  // https://github.com/dpq/pysatel/blob/master/pysatel/coord.py#L44
  float64_t lon_rad = Deg2Rad(lla.lon);
  float64_t lat_rad = Deg2Rad(lla.lat);
  float64_t altitude = lla.height;
  float64_t xi =
      sqrt(1 - kFirstEccentricitySquared * sin(lat_rad) * sin(lat_rad));
  ecef->x = (kSemimajorAxis / xi + altitude) * cos(lat_rad) * cos(lon_rad);
  ecef->y = (kSemimajorAxis / xi + altitude) * cos(lat_rad) * sin(lon_rad);
  ecef->z = (kSemimajorAxis / xi * (1 - kFirstEccentricitySquared) + altitude) *
            sin(lat_rad);
}

void CoordinateConverter::ECEF2LLA(const PointENU_t& ecef,
                                   PointLLH_t* lla) const {
  // Convert ECEF coordinates to geodetic coordinates.
  // J. Zhu, "Conversion of Earth-centered Earth-fixed coordinates
  // to geodetic coordinates," IEEE Transactions on Aerospace and
  // Electronic Systems, vol. 30, pp. 957-961, 1994.

  float64_t r = sqrt(ecef.x * ecef.x + ecef.y * ecef.y);
  float64_t Esq =
      kSemimajorAxis * kSemimajorAxis - kSemiminorAxis * kSemiminorAxis;
  float64_t F = 54 * kSemiminorAxis * kSemiminorAxis * ecef.z * ecef.z;
  float64_t G = r * r + (1 - kFirstEccentricitySquared) * ecef.z * ecef.z -
                kFirstEccentricitySquared * Esq;
  float64_t C =
      (kFirstEccentricitySquared * kFirstEccentricitySquared * F * r * r) /
      pow(G, 3);
  float64_t S = cbrt(1 + C + sqrt(C * C + 2 * C));
  float64_t P = F / (3 * pow((S + 1 / S + 1), 2) * G * G);
  float64_t Q =
      sqrt(1 + 2 * kFirstEccentricitySquared * kFirstEccentricitySquared * P);
  float64_t r_0 = -(P * kFirstEccentricitySquared * r) / (1 + Q) +
                  sqrt(0.5 * kSemimajorAxis * kSemimajorAxis * (1 + 1.0 / Q) -
                       P * (1 - kFirstEccentricitySquared) * ecef.z * ecef.z /
                           (Q * (1 + Q)) -
                       0.5 * P * r * r);
  float64_t U =
      sqrt(pow((r - kFirstEccentricitySquared * r_0), 2) + ecef.z * ecef.z);
  float64_t V = sqrt(pow((r - kFirstEccentricitySquared * r_0), 2) +
                     (1 - kFirstEccentricitySquared) * ecef.z * ecef.z);
  float64_t Z_0 =
      kSemiminorAxis * kSemiminorAxis * ecef.z / (kSemimajorAxis * V);

  lla->height =
      U * (1 - kSemiminorAxis * kSemiminorAxis / (kSemimajorAxis * V));
  lla->lat = Rad2Deg(atan((ecef.z + kSecondEccentricitySquared * Z_0) / r));
  lla->lon = Rad2Deg(atan2(ecef.y, ecef.x));
}

adLocStatus_t CoordinateConverter::ECEF2ENU(const PointENU_t& ecef,
                                            PointENU_t* enu,
                                            bool is_old_origin) const {
  if (nullptr == enu) {
    return LOC_NULL_PTR;
  }
  if (!has_origin_) {
    LC_LERROR(COMMON) << "CoordinateConverter doesn't have origin yet";
    return LOC_LOCALIZATION_ERROR;
  }

  PointENU_t origin_ecef_m;
  Eigen::Matrix3d matrix_ecef_to_enu;
  if (is_old_origin) {
    origin_ecef_m = origin_ecef_m_old_;
    matrix_ecef_to_enu = matrix_ecef_to_enu_old_;
  } else {
    origin_ecef_m = origin_ecef_m_;
    matrix_ecef_to_enu = matrix_ecef_to_enu_;
  }

  Eigen::Vector3d delta_ecef;
  delta_ecef(0) = ecef.x - origin_ecef_m.x;
  delta_ecef(1) = ecef.y - origin_ecef_m.y;
  delta_ecef(2) = ecef.z - origin_ecef_m.z;
  Eigen::Vector3d vec_enu = matrix_ecef_to_enu * delta_ecef;
  enu->x = vec_enu(0);
  enu->y = vec_enu(1);
  enu->z = vec_enu(2);

  return LOC_SUCCESS;
}

adLocStatus_t CoordinateConverter::ENU2ECEF(const PointENU_t& enu,
                                            PointENU_t* ecef,
                                            bool is_old_origin) const {
  if (nullptr == ecef) {
    return LOC_NULL_PTR;
  }
  if (!has_origin_) {
    LC_LERROR(COMMON) << "CoordinateConverter doesn't have origin yet";
    return LOC_LOCALIZATION_ERROR;
  }

  PointENU_t origin_ecef_m;
  Eigen::Matrix3d matrix_enu_to_ecef;
  if (is_old_origin) {
    origin_ecef_m = origin_ecef_m_old_;
    matrix_enu_to_ecef = matrix_enu_to_ecef_old_;
  } else {
    origin_ecef_m = origin_ecef_m_;
    matrix_enu_to_ecef = matrix_enu_to_ecef_;
  }

  Eigen::Vector3d vec_enu(enu.x, enu.y, enu.z);
  Eigen::Vector3d delta_ecef = matrix_enu_to_ecef * vec_enu;
  ecef->x = delta_ecef(0) + origin_ecef_m.x;
  ecef->y = delta_ecef(1) + origin_ecef_m.y;
  ecef->z = delta_ecef(2) + origin_ecef_m.z;

  return LOC_SUCCESS;
}

}  // namespace localization
}  // namespace senseAD
