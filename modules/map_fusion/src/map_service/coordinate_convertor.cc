/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ：
 *   author     ： zhangrui
 *   date       ： 2024.5
 ******************************************************************************/

#include "map_fusion/map_service/coordinate_convertor.h"

#include <cmath>

// #include "common/file/file.h"
// #include "common/math/math_utils.h"

namespace hozon {
namespace mp {
namespace mf {

bool GCS2UTM(const int zone, double* x, double* y) {
  const double source_x = *x;
  const double source_y = *y;
  // std::string utm_str = "+proj=utm +zone=" + std::to_string(utm_zone) +
  //                       " +datum=WGS84 +units=m +no_defs";
  // const char* utm = utm_str.c_str();
  // CoordinateConvert(gcs_, utm, source_x, source_y, source_z, x, y, z);

  LatLonToUTMXY(zone, source_x, source_y, x, y);
  return true;
}

bool UTM2GCS(const int zone, double* x, double* y) {
  const double source_x = *x;
  const double source_y = *y;
  // std::string utm_str = "+proj=utm +zone=" + std::to_string(utm_zone) +
  //                       " +datum=WGS84 +units=m +no_defs";
  // const char* utm = utm_str.c_str();
  // CoordinateConvert(utm, gcs_, source_x, source_y, source_z, x, y, z);
  UTMXYToLatLon(source_x, source_y, zone, false, x, y);
  return true;
}

void Wgs84ToGcj02(const double wgs_Lon, const double wgs_Lat,
                  double* const gcj_Lon, double* const gcj_Lat) {
  if (OutOfChina(wgs_Lat, wgs_Lon)) {
    *gcj_Lat = wgs_Lat;
    *gcj_Lon = wgs_Lon;
    return;
  } else {
    double delta_Lat = TransformLat(wgs_Lon - 105.0, wgs_Lat - 35.0);
    double delta_Lon = TransformLon(wgs_Lon - 105.0, wgs_Lat - 35.0);
    double radLat = wgs_Lat / 180.0 * M_PI;
    double magic = std::sin(radLat);
    magic = 1 - kMagicCoeff * magic * magic;
    double sqrtMagic = std::sqrt(magic);
    delta_Lat = (delta_Lat * 180.0) / ((kEarthRadius * (1 - kMagicCoeff)) /
                                       (magic * sqrtMagic) * M_PI);
    delta_Lon = (delta_Lon * 180.0) /
                (kEarthRadius / sqrtMagic * std::cos(radLat) * M_PI);
    *gcj_Lat = wgs_Lat + delta_Lat;
    *gcj_Lon = wgs_Lon + delta_Lon;
    return;
  }
}

bool OutOfChina(const double lat, const double lon) {
  if (lon < 72.004 || lon > 137.8347) return true;
  if (lat < 0.8293 || lat > 55.8271) return true;
  return false;
}

double TransformLat(const double x, const double y) {
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

double TransformLon(const double x, const double y) {
  double ret =
      300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * sqrt(std::abs(x));
  ret += (20.0 * std::sin(6.0 * x * M_PI) + 20.0 * std::sin(2.0 * x * M_PI)) *
         2.0 / 3.0;
  ret +=
      (20.0 * std::sin(x * M_PI) + 40.0 * std::sin(x / 3.0 * M_PI)) * 2.0 / 3.0;
  ret +=
      (150.0 * std::sin(x / 12.0 * M_PI) + 300.0 * std::sin(x / 30.0 * M_PI)) *
      2.0 / 3.0;
  return ret;
}

inline double DegToRad(double deg) { return (deg / 180.0 * M_PI); }

inline double RadToDeg(double rad) { return (rad * 180.0 / M_PI); }

inline double UTMCentralMeridian(int zone) {
  return DegToRad(-183.0 + (zone * 6.0));
}

void LatLonToUTMXY(const int zone, const double lon, const double lat,
                   double* x, double* y) {
  // int zone = std::floor((wgs.lon + 180.0) / 6.0) + 1;
  MapLatLonToXY(lat / 180 * M_PI, lon / 180 * M_PI, UTMCentralMeridian(zone), x,
                y);

  /** Adjust easting and northing for UTM system. */
  *x = *x * kUTMScaleFactor + 500000.0;
  *y = *y * kUTMScaleFactor;
  if (*y < 0.0) *y += 10000000.0;
}

void MapLatLonToXY(double phi, double lambda, double lambda0, double* x,
                   double* y) {
  double N = 0.0;
  double nu2 = 0.0;
  double ep2 = 0.0;
  double t = 0.0;
  double t2 = 0.0;
  double l = 0.0;
  double l3coef = 0.0;
  double l4coef = 0.0;
  double l5coef = 0.0;
  double l6coef = 0.0;
  // double l7coef = 0.0;
  // double l8coef = 0.0;
  // double tmp = 0.0;

  /** Precalculate ep2 */
  // ep2 = (std::pow(kEarthARadius, 2.0) - std::pow(kEarthBRadius, 2.0)) /
  //       std::pow(kEarthBRadius, 2.0);
  ep2 = std::pow(e, 2.0) / (1 - std::pow(e, 2.0));

  /** Precalculate nu2 */
  nu2 = ep2 * std::pow(std::cos(phi), 2.0);

  /** Precalculate N */
  N = std::pow(kEarthARadius, 2.0) / (kEarthBRadius * std::sqrt(1 + nu2));

  /** Precalculate t */
  t = std::tan(phi);
  t2 = t * t;
  // tmp = (t2 * t2 * t2) - std::pow(t, 6.0);

  /** Precalculate l */
  l = lambda - lambda0;

  /** Precalculate coefficients for l**n in the equations below
  so a normal human being can read the expressions for easting
  and northing
  -- l**1 and l**2 have coefficients of 1.0 */
  l3coef = 1.0 - t2 + nu2;

  l4coef = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2);

  l5coef = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2 - 58.0 * t2 * nu2;

  l6coef = 61.0 - 58.0 * t2 + (t2 * t2);

  // l7coef = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2);

  // l8coef = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2);

  /** Calculate easting (x) */
  *x = N * std::cos(phi) * l +
       (N / 6.0 * std::pow(std::cos(phi), 3.0) * l3coef * std::pow(l, 3.0)) +
       (N / 120.0 * std::pow(std::cos(phi), 5.0) * l5coef * std::pow(l, 5.0));

  /** Calculate northing (y) */
  *y = ArcLengthOfMeridian(phi) +
       (t / 2.0 * N * std::pow(std::cos(phi), 2.0) * std::pow(l, 2.0)) +
       (t / 24.0 * N * std::pow(std::cos(phi), 4.0) * l4coef *
        std::pow(l, 4.0)) +
       (t / 720.0 * N * std::pow(std::cos(phi), 6.0) * l6coef *
        std::pow(l, 6.0));
}

double ArcLengthOfMeridian(double phi) {
  double alpha = 0.0;
  double beta = 0.0;
  double gamma = 0.0;
  double delta = 0.0;
  double epsilon = 0.0;
  double n = 0.0;
  double result = 0.0;

  /** Precalculate n */
  n = (kEarthARadius - kEarthBRadius) / (kEarthARadius + kEarthBRadius);

  /** Precalculate alpha */
  alpha = ((kEarthARadius + kEarthBRadius) / 2.0) *
          (1.0 + (std::pow(n, 2.0) / 4.0) + (std::pow(n, 4.0) / 64.0));

  /** Precalculate beta */
  beta = (-3.0 * n / 2.0) + (9.0 * std::pow(n, 3.0) / 16.0) +
         (-3.0 * std::pow(n, 5.0) / 32.0);

  /** Precalculate gamma */
  gamma = (15.0 * std::pow(n, 2.0) / 16.0) + (-15.0 * std::pow(n, 4.0) / 32.0);

  /** Precalculate delta */
  delta =
      (-35.0 * std::pow(n, 3.0) / 48.0) + (105.0 * std::pow(n, 5.0) / 256.0);

  /** Precalculate epsilon */
  epsilon = (315.0 * std::pow(n, 4.0) / 512.0);

  /** Now calculate the sum of the series and return */
  result = alpha *
           (phi + (beta * std::sin(2.0 * phi)) + (gamma * std::sin(4.0 * phi)) +
            (delta * std::sin(6.0 * phi)) + (epsilon * std::sin(8.0 * phi)));

  return result;
}

void UTMXYToLatLon(double x, double y, int zone, bool southhemi, double* lon,
                   double* lat) {
  double cmeridian = 0.0;

  // x -= 500000.0;
  // x /= kUTMScaleFactor;
  auto factor_x = (x - 500000.0) / kUTMScaleFactor;

  /** If in southern hemisphere, adjust y accordingly. */
  auto factor_y = y;
  if (southhemi) factor_y -= 10000000.0;

  factor_y /= kUTMScaleFactor;

  cmeridian = UTMCentralMeridian(zone);
  MapXYToLatLon(factor_x, factor_y, cmeridian, lon, lat);
  *lon = RadToDeg(*lon);
  *lat = RadToDeg(*lat);
}

void MapXYToLatLon(double x, double y, double lambda0, double* lon,
                   double* lat) {
  double phif = 0.0;
  double Nf = 0.0;
  double Nfpow = 0.0;
  double nuf2 = 0.0;
  double ep2 = 0.0;
  double tf = 0.0;
  double tf2 = 0.0;
  double tf4 = 0.0;
  double cf = 0.0;
  double x1frac = 0.0;
  double x2frac = 0.0;
  double x3frac = 0.0;
  double x4frac = 0.0;
  double x5frac = 0.0;
  double x6frac = 0.0;
  double x7frac = 0.0;
  double x8frac = 0.0;
  double x2poly = 0.0;
  double x3poly = 0.0;
  double x4poly = 0.0;
  double x5poly = 0.0;
  double x6poly = 0.0;
  double x7poly = 0.0;
  double x8poly = 0.0;

  /** Get the value of phif, the footpoint latitude. */
  phif = FootpointLatitude(y);

  /** Precalculate ep2 */
  // ep2 = (std::pow(kEarthARadius, 2.0) - std::pow(kEarthBRadius, 2.0)) /
  //       std::pow(kEarthBRadius, 2.0);
  ep2 = std::pow(e, 2.0) / (1 - std::pow(e, 2.0));

  /** Precalculate cos (phif) */
  cf = std::cos(phif);

  /** Precalculate nuf2 */
  nuf2 = ep2 * std::pow(cf, 2.0);

  /** Precalculate Nf and initialize Nfpow */
  Nf = std::pow(kEarthARadius, 2.0) / (kEarthBRadius * std::sqrt(1 + nuf2));
  Nfpow = Nf;

  /** Precalculate tf */
  tf = std::tan(phif);
  tf2 = tf * tf;
  tf4 = tf2 * tf2;

  /** Precalculate fractional coefficients for x**n in the equations
  below to simplify the expressions for latitude and longitude. */
  x1frac = 1.0 / (Nfpow * cf);

  Nfpow *= Nf; /* now equals Nf**2) */
  x2frac = tf / (2.0 * Nfpow);

  Nfpow *= Nf; /* now equals Nf**3) */
  x3frac = 1.0 / (6.0 * Nfpow * cf);

  Nfpow *= Nf; /* now equals Nf**4) */
  x4frac = tf / (24.0 * Nfpow);

  Nfpow *= Nf; /* now equals Nf**5) */
  x5frac = 1.0 / (120.0 * Nfpow * cf);

  Nfpow *= Nf; /* now equals Nf**6) */
  x6frac = tf / (720.0 * Nfpow);

  Nfpow *= Nf; /* now equals Nf**7) */
  x7frac = 1.0 / (5040.0 * Nfpow * cf);

  Nfpow *= Nf; /* now equals Nf**8) */
  x8frac = tf / (40320.0 * Nfpow);

  /** Precalculate polynomial coefficients for x**n.
  -- x**1 does not have a polynomial coefficient. */
  x2poly = -1.0 - nuf2;

  x3poly = -1.0 - 2 * tf2 - nuf2;

  x4poly = 5.0 + 3.0 * tf2 + 6.0 * nuf2 - 6.0 * tf2 * nuf2 -
           3.0 * (nuf2 * nuf2) - 9.0 * tf2 * (nuf2 * nuf2);

  x5poly = 5.0 + 28.0 * tf2 + 24.0 * tf4 + 6.0 * nuf2 + 8.0 * tf2 * nuf2;

  x6poly = -61.0 - 90.0 * tf2 - 45.0 * tf4 - 107.0 * nuf2 + 162.0 * tf2 * nuf2;

  x7poly = -61.0 - 662.0 * tf2 - 1320.0 * tf4 - 720.0 * (tf4 * tf2);

  x8poly = 1385.0 + 3633.0 * tf2 + 4095.0 * tf4 + 1575 * (tf4 * tf2);

  /** Calculate latitude */
  *lat = phif + x2frac * x2poly * (x * x) + x4frac * x4poly * std::pow(x, 4.0) +
         x6frac * x6poly * std::pow(x, 6.0) +
         x8frac * x8poly * std::pow(x, 8.0);

  /** Calculate longitude */
  *lon = lambda0 + x1frac * x + x3frac * x3poly * std::pow(x, 3.0) +
         x5frac * x5poly * std::pow(x, 5.0) +
         x7frac * x7poly * std::pow(x, 7.0);
}

double FootpointLatitude(double y) {
  double y_ = 0.0;
  double alpha_ = 0.0;
  double beta_ = 0.0;
  double gamma_ = 0.0;
  double delta_ = 0.0;
  double epsilon_ = 0.0;
  double n = 0.0;
  double result = 0.0;

  /** Precalculate n (Eq. 10.18) */
  n = (kEarthARadius - kEarthBRadius) / (kEarthARadius + kEarthBRadius);

  /** Precalculate alpha_ (Eq. 10.22) */
  /** (Same as alpha in Eq. 10.17) */
  alpha_ = ((kEarthARadius + kEarthBRadius) / 2.0) *
           (1 + (std::pow(n, 2.0) / 4) + (std::pow(n, 4.0) / 64));

  /** Precalculate y_ (Eq. 10.23) */
  y_ = y / alpha_;

  /** Precalculate beta_ (Eq. 10.22) */
  beta_ = (3.0 * n / 2.0) + (-27.0 * std::pow(n, 3.0) / 32.0) +
          (269.0 * std::pow(n, 5.0) / 512.0);

  /** Precalculate gamma_ (Eq. 10.22) */
  gamma_ = (21.0 * std::pow(n, 2.0) / 16.0) + (-55.0 * std::pow(n, 4.0) / 32.0);

  /** Precalculate delta_ (Eq. 10.22) */
  delta_ =
      (151.0 * std::pow(n, 3.0) / 96.0) + (-417.0 * std::pow(n, 5.0) / 128.0);

  /** Precalculate epsilon_ (Eq. 10.22) */
  epsilon_ = (1097.0 * std::pow(n, 4.0) / 512.0);

  /** Now calculate the sum of the series (Eq. 10.21) */
  result = y_ + (beta_ * std::sin(2.0 * y_)) + (gamma_ * std::sin(4.0 * y_)) +
           (delta_ * std::sin(6.0 * y_)) + (epsilon_ * std::sin(8.0 * y_));

  return result;
}

// double HeadingError(int zone, double lat, double lon) {
//   double x1 = lat;
//   double y1 = lon;
//   GCS2UTM(zone, &x1, &y1);
//   double x2 = lat;
//   double y2 = lon + 0.000001;
//   GCS2UTM(zone, &x2, &y2);
//   double heading_offset = std::atan2(y2 - y1, x2 - x1);
//   if (std::isnan(heading_offset)) {
//     return 0.0;
//   }
//   return common::math::Clamp(std::atan2(y2 - y1, x2 - x1) - M_PI_2,
//                              DegToRad(-3.0), DegToRad(3.0));
// }

}  // namespace mf
}  // namespace mp
}  // namespace hozon
