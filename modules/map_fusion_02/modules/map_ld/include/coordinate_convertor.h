/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ：
 *   author     ： zhangrui
 *   date       ： 2024.5
 ******************************************************************************/
#pragma once

#include <string>

namespace hozon {
namespace mp {
namespace mf {
/*
 * @brief WGS point to GCJ02 transition function
 */
static constexpr double kEarthRadius = 6378245.0;
static constexpr double kMagicCoeff = 0.00669342162296594323;
void Wgs84ToGcj02(const double wgs_Lon, const double wgs_Lat,
                  double* const gcj_Lon, double* const gcj_Lat);
bool OutOfChina(const double lat, const double lon);
double TransformLat(const double x, const double y);
double TransformLon(const double x, const double y);

/*
 * @brief Convert between latitude & longitude points and utm points
 */

bool GCS2UTM(const int zone, double* x, double* y);
bool UTM2GCS(const int zone, double* x, double* y);

static const double kEarthARadius = 6378137.0;
static const double kEarthBRadius = 6356752.314;
static const double kUTMScaleFactor = 0.9996;
static const double e = 0.0818192;

double DegToRad(double deg);

double RadToDeg(double rad);

double UTMCentralMeridian(int zone);

void LatLonToUTMXY(const int zone, const double lon, const double lat,
                   double* x, double* y);

void MapLatLonToXY(const double phi, const double lambda, const double lambda0,
                   double* x, double* y);

double ArcLengthOfMeridian(double phi);

void UTMXYToLatLon(const double x, const double y, int zone, bool southhemi,
                   double* lon, double* lat);

void MapXYToLatLon(double x, double y, double lambda0, double* lon,
                   double* lat);

double FootpointLatitude(double y);

// double HeadingError(const int zone, const double lat, const double lon);

}  // namespace mf
}  // namespace mp
}  // namespace hozon
