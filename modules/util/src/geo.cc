/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： geo.cc
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/util/include/util/geo.h"

namespace hozon {
namespace mp {
namespace util {

Eigen::Vector3d Geo::BlhToEnu(const Eigen::Vector3d& pos,
                              const Eigen::Vector3d& ref) {
  Eigen::Vector3d xyz = BlhToXyz(pos);
  Eigen::Vector3d refxyz = BlhToXyz(ref);
  Eigen::Vector3d delta = xyz - refxyz;
  double dgr2rad = M_PI / 180.0;
  double lat0 = ref[0] * dgr2rad;
  double lon0 = ref[1] * dgr2rad;
  double alt0 = ref[2];
  double sinlon = std::sin(lon0);
  double coslon = std::cos(lon0);
  double sinlat = std::sin(lat0);
  double coslat = std::cos(lat0);
  Eigen::Matrix3d S;
  S << -sinlon, coslon, 0, -sinlat * coslon, -sinlat * sinlon, coslat,
      coslat * coslon, coslat * sinlon, sinlat;
  Eigen::Vector3d enu = S * delta;
  return enu;
}

Eigen::Vector3d Geo::BlhToXyz(const Eigen::Vector3d& gps, double a_axis,
                              double b_axis) {
  Eigen::Vector3d xyz;
  double dgr2rad = M_PI / 180.0;
  double l = gps[0] * dgr2rad;
  double b = gps[1] * dgr2rad;
  double h = gps[2];
  double a_axis2 = std::pow(a_axis, 2);
  double e2 = (a_axis2 - std::pow(b_axis, 2)) / (a_axis2);
  double N = a_axis / std::sqrt(1.0 - e2 * std::pow(std::sin(l), 2));
  double x = (N + h) * std::cos(l) * std::cos(b);
  double y = (N + h) * std::cos(l) * std::sin(b);
  double z = (N * (1.0 - e2) + h) * std::sin(l);
  xyz << x, y, z;
  return xyz;
}

Eigen::Vector3d Geo::EnuToBlh(const Eigen::Vector3d& pos,
                              const Eigen::Vector3d& ref, double a_axis,
                              double b_axis) {
  double dgr2rad = M_PI / 180.0;
  double rad2dgr = 180.0 / M_PI;
  double lat0 = ref[0] * dgr2rad;
  double lon0 = ref[1] * dgr2rad;
  double alt0 = ref[2];
  double sinlon = std::sin(lon0);
  double coslon = std::cos(lon0);
  double sinlat = std::sin(lat0);
  double coslat = std::cos(lat0);
  Eigen::Matrix3d S;
  S << -sinlon, coslon, 0, -sinlat * coslon, -sinlat * sinlon, coslat,
      coslat * coslon, coslat * sinlon, sinlat;
  Eigen::Vector3d refxyz = BlhToXyz(ref);
  Eigen::Vector3d xyz = S.transpose() * pos + refxyz;
  double p = std::sqrt(std::pow(xyz[0], 2) + std::pow(xyz[1], 2));
  double a_axis2 = std::pow(a_axis, 2);
  double b_axis2 = std::pow(b_axis, 2);
  double e2_a = (a_axis2 - b_axis2) / (a_axis2);
  double e2_b = (a_axis2 - b_axis2) / (b_axis2);
  double theta = std::atan2(xyz[2] * a_axis, p * b_axis);
  double lon = std::atan2(xyz[1], xyz(0));
  double lat = std::atan2(xyz[2] + e2_b * b_axis * std::pow(std::sin(theta), 3),
                          p - e2_a * a_axis * std::pow(std::cos(theta), 3));
  double N = a_axis / std::sqrt(1.0 - e2_a * std::pow(std::sin(lat), 2));
  double h = p / std::cos(lat) - N;
  Eigen::Vector3d blh;
  blh << lat * rad2dgr, lon * rad2dgr, h;
  return blh;
}

Eigen::Vector3d Geo::Gcj02ToEnu(const Eigen::Vector3d& blh,
                                const Eigen::Vector3d& ref) {
  Eigen::Vector3d _ref = ref;
  _ref.z() = 0;
  Eigen::Vector3d _blh(blh.x(), blh.y(), 0);
  return BlhToEnu(_blh, _ref);
}

Eigen::Vector3d Geo::EnuToGcj02(const Eigen::Vector3d& blh,
                                const Eigen::Vector3d& ref) {
  Eigen::Vector3d _ref = ref;
  _ref.z() = 0;
  Eigen::Vector3d _blh(blh.x(), blh.y(), blh.z());
  return EnuToBlh(_blh, _ref);
}

void Geo::WgsToUtm(const Eigen::Vector3d& pos, Eigen::Vector3d* const xy) {
  if (!xy) {
    return;
  }
  int zone = std::floor((pos[1] + 180.0) / 6.0) + 1;
  MapWgsToUtm(pos[0] / 180 * PI_, pos[1] / 180 * PI_, UtmCentralMeridian(zone),
              xy);
  (*xy)[0] = (*xy)[0] * utm_scale_factor_ + 500000.0;
  (*xy)[1] = (*xy)[1] * utm_scale_factor_;
  if ((*xy)[1] < 0.0) {
    (*xy)[1] += 10000000.0;
  }
}

void Geo::LatLonToUtmXy(uint32_t zone, double lon, double lat,
                        Eigen::Vector3d* const xy) {
  if (!xy) {
    return;
  }
  MapWgsToUtm(lat / 180.0 * M_PI, lon / 180.0 * M_PI, UtmCentralMeridian(zone),
              xy);
  (*xy)[0] = (*xy)[0] * utm_scale_factor_ + 500000.0;
  (*xy)[1] = (*xy)[1] * utm_scale_factor_;
  if ((*xy)[1] < 0.0) {
    (*xy)[1] += 10000000.0;
  }
}

void Geo::MapWgsToUtm(double phi, double lambda, double lambda0,
                      Eigen::Vector3d* const xy) {
  if (!xy) {
    return;
  }
  double N, nu2, ep2, t, t2, l;
  double l3coef, l4coef, l5coef, l6coef, l7coef, l8coef;
  double tmp;
  ep2 = (std::pow(sm_a_, 2.0) - std::pow(sm_b_, 2.0)) / std::pow(sm_b_, 2.0);
  nu2 = ep2 * std::pow(std::cos(phi), 2.0);
  N = std::pow(sm_a_, 2.0) / (sm_b_ * std::sqrt(1 + nu2));
  t = std::tan(phi);
  t2 = t * t;
  tmp = (t2 * t2 * t2) - std::pow(t, 6.0);
  l = lambda - lambda0;
  l3coef = 1.0 - t2 + nu2;
  l4coef = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2);
  l5coef = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2 - 58.0 * t2 * nu2;
  l6coef = 61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2 - 330.0 * t2 * nu2;
  l7coef = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2);
  l8coef = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2);
  (*xy)[0] =
      N * std::cos(phi) * l +
      (N / 6.0 * std::pow(std::cos(phi), 3.0) * l3coef * std::pow(l, 3.0)) +
      (N / 120.0 * std::pow(std::cos(phi), 5.0) * l5coef * std::pow(l, 5.0));
  (*xy)[1] = ArcLengthOfMeridian(phi) +
             (t / 2.0 * N * std::pow(std::cos(phi), 2.0) * std::pow(l, 2.0)) +
             (t / 24.0 * N * std::pow(std::cos(phi), 4.0) * l4coef *
              std::pow(l, 4.0)) +
             (t / 720.0 * N * std::pow(std::cos(phi), 6.0) * l6coef *
              std::pow(l, 6.0));
}

double Geo::UtmCentralMeridian(int zone) {
  return DegToRad(-183.0 + (zone * 6.0));
}

double Geo::DegToRad(double deg) { return (deg / 180.0 * PI_); }

double Geo::ArcLengthOfMeridian(double phi) {
  double alpha, beta, gamma, delta, epsilon, n;
  double result;
  n = (sm_a_ - sm_b_) / (sm_a_ + sm_b_);
  alpha = ((sm_a_ + sm_b_) / 2.0) *
          (1.0 + (std::pow(n, 2.0) / 4.0) + (std::pow(n, 4.0) / 64.0));
  beta = (-3.0 * n / 2.0) + (9.0 * std::pow(n, 3.0) / 16.0) +
         (-3.0 * std::pow(n, 5.0) / 32.0);
  gamma = (15.0 * std::pow(n, 2.0) / 16.0) + (-15.0 * std::pow(n, 4.0) / 32.0);
  delta =
      (-35.0 * std::pow(n, 3.0) / 48.0) + (105.0 * std::pow(n, 5.0) / 256.0);
  epsilon = (315.0 * std::pow(n, 4.0) / 512.0);
  result = phi + (beta * std::sin(2.0 * phi)) + (gamma * std::sin(4.0 * phi)) +
           (delta * std::sin(6.0 * phi)) + (epsilon * std::sin(8.0 * phi));
  result = alpha * result;
  return result;
}

Vec3d Geo::Wgs84ToGcj02(const Vec3d& blh) {
  static double a = 6378245.0;
  static double ee = 0.00669342162296594323;
  double lat = blh.x();
  double lon = blh.y();
  double d_lat = TransformLat(lon - 105.0, lat - 35.0);
  double d_lon = TransformLon(lon - 105.0, lat - 35.0);
  double rad_lat = lat / 180.0 * M_PI;
  double magic = std::sin(rad_lat);
  magic = 1 - ee * magic * magic;
  double sqrt_magic = std::sqrt(magic);
  d_lat = (d_lat * 180.0) / ((a * (1 - ee)) / (magic * sqrt_magic) * M_PI);
  d_lon = (d_lon * 180.0) / (a / sqrt_magic * cos(rad_lat) * M_PI);
  double mg_lat = lat + d_lat;
  double mg_lon = lon + d_lon;
  return Vec3d(mg_lat, mg_lon, blh.z());
}

double Geo::TransformLat(double x, double y) {
  double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y +
               0.2 * std::sqrt(std::abs(x));
  ret += (20.0 * std::sin(6.0 * x * M_PI) + 20.0 * std::sin(2.0 * x * M_PI)) *
         2.0 / 3.0;
  ret +=
      (20.0 * std::sin(y * M_PI) + 40.0 * std::sin(y / 3.0 * M_PI)) * 2.0 / 3.0;
  ret += (160.0 * std::sin(y / 12.0 * M_PI) + 320 * std::sin(y * M_PI / 30.0)) *
         2.0 / 3.0;
  return ret;
}

double Geo::TransformLon(double x, double y) {
  double ret =
      300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * sqrt(abs(x));
  ret += (20.0 * sin(6.0 * x * M_PI) + 20.0 * sin(2.0 * x * M_PI)) * 2.0 / 3.0;
  ret += (20.0 * sin(x * M_PI) + 40.0 * sin(x / 3.0 * M_PI)) * 2.0 / 3.0;
  ret +=
      (150.0 * sin(x / 12.0 * M_PI) + 300.0 * sin(x / 30.0 * M_PI)) * 2.0 / 3.0;
  return ret;
}

double Geo::UtmXyToSpeed(
    const std::vector<std::tuple<double, Eigen::Vector3d>>& pos) {
  if (pos.size() < 3) {
    return -1;
  }
  std::vector<std::tuple<double, Eigen::Vector3d>> gradient;
  std::vector<double> tmp(4, 0);
  int c = 1;
  int count = 0;

  for (auto i = 0; i < pos.size(); ++i) {
    if (i == 0) {
      gradient.emplace_back(std::tuple<double, Eigen::Vector3d>{
          std::get<0>(pos[1]) - std::get<0>(pos[0]),
          Eigen::Vector3d(std::get<1>(pos[1])[0] - std::get<1>(pos[0])[0],
                          std::get<1>(pos[1])[1] - std::get<1>(pos[0])[1],
                          std::get<1>(pos[1])[2] - std::get<1>(pos[0])[2])});
    } else if (i + 1 == pos.size()) {
      gradient.emplace_back(std::tuple<double, Eigen::Vector3d>{
          std::get<0>(pos[2]) - std::get<0>(pos[1]),
          Eigen::Vector3d(
              std::get<1>(pos[i])[0] - std::get<1>(pos[i - 1])[0],
              std::get<1>(pos[i])[1] - std::get<1>(pos[i - 1])[1],
              std::get<1>(pos[i])[2] - std::get<1>(pos[i - 1])[2])});
    } else {
      gradient.emplace_back(std::tuple<double, Eigen::Vector3d>{
          (std::get<0>(pos[2]) - std::get<0>(pos[0])) / 2,
          Eigen::Vector3d(
              (std::get<1>(pos[i + 1])[0] - std::get<1>(pos[i - 1])[0]) / 2,
              (std::get<1>(pos[i + 1])[1] - std::get<1>(pos[i - 1])[1]) / 2,
              (std::get<1>(pos[i + 1])[2] - std::get<1>(pos[i - 1])[2]) / 2)});
    }
  }
  std::vector<double> speed(3, 0);
  for (auto& g : gradient) {
    if (std::get<0>(g) <= 0) {
      continue;
    }
    for (auto i = 0; i < speed.size(); ++i) {
      if (std::get<1>(g)[i] * speed[i] < 0) {
        return -1;
      }
      speed[i] += std::get<1>(g)[i] / std::get<0>(g);
    }
    ++count;
  }
  if (count == 0) {
    return -1;
  }
  double sum = 0;
  for (auto i = 0; i < speed.size(); ++i) {
    auto s = speed[i] / count;
    sum += s * s;
  }
  return std::sqrt(sum);
}

}  // namespace util
}  // namespace mp
}  // namespace hozon
