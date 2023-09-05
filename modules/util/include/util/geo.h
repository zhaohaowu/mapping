/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： geo.h
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <memory>
#include <tuple>
#include <vector>

#include "Eigen/Eigen"

namespace hozon {
namespace mp {
namespace util {

typedef Eigen::Matrix<double, 3, 1> Vec3d;
typedef Eigen::Quaterniond Quatd;

class Geo {
 public:
  Geo() = default;
  ~Geo() = default;

  /**
   * @brief converts blh to enu
   *
   * @param pos : blh coordinate
   * @param ref : reference point
   * @return get enu coordinate
   */
  static Eigen::Vector3d BlhToEnu(const Eigen::Vector3d& pos,
                                  const Eigen::Vector3d& ref);

  /**
   * @brief converts blh to ecef
   *
   * @param pos : blh coordinate
   * @param a_axis : equatorial radius of the Earth
   * @param b_axis : polar radius of the Earth
   * @return get ecef coordinate
   */
  static Eigen::Vector3d BlhToXyz(const Eigen::Vector3d& gps,
                                  double a_axis = 6378137.0,
                                  double b_axis = 6356752.314);

  /**
   * @brief converts enu to blh
   *
   * @param pos : enu coordinate
   * @param ref : reference point
   * @param a_axis : equatorial radius of the Earth
   * @param b_axis : polar radius of the Earth
   * @return get blh position
   */
  static Eigen::Vector3d EnuToBlh(const Eigen::Vector3d& pos,
                                  const Eigen::Vector3d& ref,
                                  double a_axis = 6378137.0,
                                  double b_axis = 6356752.314);

  /**
   * @brief converts gcj02 to enu
   *
   * @param pos : gcj02 coordinate
   * @param ref : reference point
   * @return get enu position
   */
  static Eigen::Vector3d Gcj02ToEnu(const Eigen::Vector3d& blh,
                                    const Eigen::Vector3d& ref);

  /**
   * @brief converts enu to gcj02
   *
   * @param pos : enu coordinate
   * @param ref : reference point
   * @return get gcj02 position
   */
  static Eigen::Vector3d EnuToGcj02(const Eigen::Vector3d& blh,
                                    const Eigen::Vector3d& ref);

  /**
   * @brief converts wgs to utm
   *
   * @param pos : wgs coordinate
   * @param ref : reference point
   * @return get utm coordinate
   */
  static void WgsToUtm(const Eigen::Vector3d& pos, Eigen::Vector3d* const xy);

  /**
   * @brief converts latlon to utmxy
   *
   * @param lon : longitude of the point
   * @param lat : latitude of the point
   * @param zone : an integer value designating the utm zone, range [1,60]
   * @param xy : utmxy coordinate
   * @return get utmxy coordinate
   */
  static void LatLonToUtmXy(uint32_t zone, double lon, double lat,
                            Eigen::Vector3d* const xy);

  /**
   * @brief converts wgs to utm
   *
   * @param phi : latitude of the point
   * @param lambda : longitude of the point
   * @param lambda0 : longitude of the central meridian to be used
   * @param xy : utmxy coordinate
   * @return get utmxy coordinate
   */
  static void MapWgsToUtm(double phi, double lambda, double lambda0,
                          Eigen::Vector3d* const xy);

  /**
   * @brief Determines the central meridian for the given utm zone.
   *
   * @param zone : an integer value designating the utm zone, range [1,60]
   * @return get radians value
   */
  static double UtmCentralMeridian(int zone);

  /**
   * @brief converts degrees to radians
   *
   * @param deg : degrees value
   * @return get radians value
   */
  static double DegToRad(double deg);

  /**
   * @brief  computes the ellipsoidal distance from the equator to a point at a
   * given latitude.
   *
   * @param phi : latitude of the point
   * @return the ellipsoidal distance of the point from the equator, in meters
   */
  static double ArcLengthOfMeridian(double phi);

  /**
   * @brief  converts wgs84 to gcj02
   *
   * @param blh : wgs84 coordinate
   * @return gcj02 coordinate
   */
  static Vec3d Wgs84ToGcj02(const Vec3d& blh);

  /**
   * @brief  transform latitude value
   *
   * @param x : latitude of the point
   * @param y :  longitude of the point
   * @return latitude of the point
   */
  static double TransformLat(double x, double y);

  /**
   * @brief  transform longitude value
   *
   * @param x : latitude of the point
   * @param y :  longitude of the point
   * @return longitude of the point
   */
  static double TransformLon(double x, double y);

  /**
   * @brief
   *
   * @param
   * @return
   */
  static double UtmXyToSpeed(
      const std::vector<std::tuple<double, Eigen::Vector3d>>& pos);

 private:
  static constexpr double sm_a_ = 6378137.0;
  static constexpr double sm_b_ = 6356752.314;
  static constexpr double sm_ecc_squared_ = 6.69437999013e-03;
  static constexpr double utm_scale_factor_ = 0.9996;
  static constexpr double PI_ = 3.14159265358979;
};

}  // namespace util
}  // namespace mp
}  // namespace hozon
