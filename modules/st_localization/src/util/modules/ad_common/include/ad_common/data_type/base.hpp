/*
 * Copyright (C) 2017 by SenseTime Group Limited. All rights reserved.
 * YU Chendi <yuchendi@sensetime.com>
 * Chen Shengjie <chenshengjie@sensetime.com>
 */

#pragma once
#include <iostream>
typedef signed char int8_t;
// typedef signed short int16_t;
typedef signed int int32_t;
// typedef signed long  int64_t;
typedef unsigned char uint8_t;
// typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
// typedef unsigned long  uint64_t;
typedef float float32_t;
typedef double float64_t;
typedef long double float128_t;

namespace senseAD {

#ifdef __aarch64__
typedef long long int senseStamp;  // NOLINT
#else
typedef long int senseStamp;  // NOLINT
#endif

/**
 * @brief struct for bgr means
 **/
typedef struct {
    float32_t blue;   // blue channel
    float32_t green;  // green channel
    float32_t red;    // red channel
} BgrValues_t;

//--------------------------- Geometry-related base ----------------------------
// A point in the map reference frame. The map defines an origin, whose
// coordinate is (0, 0, 0).
// Most modules, including localization and routing, generate results based on
// the map reference frame.
// Currently, the map uses Universal Transverse Mercator (UTM) projection.
// The z field of PointENU_t can be omitted. If so, it is a 2D location
// and we do not care its height.
struct PointENU_t {
    float64_t x = 0;  // East from the origin, in meters.
    float64_t y = 0;  // North from the origin, in meters.
    float64_t z = 0;  // Up from the WGS-84 ellipsoid, in meters.
    PointENU_t() = default;
    PointENU_t(float64_t _x, float64_t _y, float64_t _z = 0)
        : x(_x), y(_y), z(_z) {}
    friend std::ostream& operator<<(std::ostream& out,  // NOLINT
                                    const PointENU_t& point) {
        out << "(" << point.x << ", " << point.y << ", " << point.z << ")";
        return out;
    }
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
};

// A point in the global reference frame. Similar to PointENU_t, PointLLH_t
// allows omitting the height field for representing a 2D location.
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
        out << "(" << point.lon << ", " << point.lat << ", " << point.height
            << ")";
        return out;
    }
};

// A EulerAngles_t that represents a spatial rotation.
typedef struct {
    // Right-handed rotation from local level around y‑axis, in radius.
    float64_t roll;
    // Right-handed rotation from local level around x‑axis, in radius.
    float64_t pitch;
    // Left-handed rotation around z-axis clockwise from North, in radius.
    float64_t yaw;
} EulerAngles_t;

// A unit Quaternion_t also represents a spatial rotation.
// Most of the time, qw = sqrt(1 - qx * qx - qy * qy - qz * qz).
typedef struct {
    float64_t qx;
    float64_t qy;
    float64_t qz;
    float64_t qw;
} Quaternion_t;

// A general 2D point in double. Its meaning and units depend on context,
// and must be explained in comments.
typedef struct {
    float64_t x;
    float64_t y;
} Point2D_t;

// A general 2D point in float. Its meaning and units depend on context,
// and must be explained in comments.
typedef struct {
    float32_t x;
    float32_t y;
} Point2F_t;

// A general 3D point in double. Its meaning and units depend on context,
// and must be explained in comments.
using Point3D_t = PointENU_t;

// A general 3D point in float. Its meaning and units depend on context,
// and must be explained in comments.
typedef struct {
    float32_t x;
    float32_t y;
    float32_t z;
} Point3F_t;

//--------------------------- Geometry-related base ----------------------------

}  // namespace senseAD
