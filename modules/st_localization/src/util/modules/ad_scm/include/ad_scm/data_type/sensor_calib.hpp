/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Deng Long <denglong@sensetime.com>
 * Yue Dayu <yuedayu@sensetime.com>
 */

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <opencv2/opencv.hpp>

#include "ad_common/data_type/any.hpp"
#include "ad_common/data_type/base.hpp"

namespace senseAD {
namespace sensorconfig {

enum class DeviceType {
    CAMERA,
    LIDAR,
    GNSS,
    CAN,
    RADAR,
    RELATIONAL,
    GENERAL,
    VEHICLE,
    IMU,
    DUALANT,
    ODOMETER,
    UNKNOWN
};
enum class ParamType {
    INTRINSIC,
    EXTRINSIC,
    HOMOGRAPHY,
    HMATRIX,
    HARDWARE,
    CURRENT_CALIBRATION_RESULT,
    DESIGNINFO,
    UNKNOWN
};

inline std::string GetDeviceName(const DeviceType &type) {
    switch (type) {
        case DeviceType::CAMERA:
            return "camera";
        case DeviceType::LIDAR:
            return "lidar";
        case DeviceType::GNSS:
            return "gnss";
        case DeviceType::CAN:
            return "can";
        case DeviceType::RADAR:
            return "radar";
        case DeviceType::RELATIONAL:
            return "relational";
        case DeviceType::VEHICLE:
            return "vehicle";
        case DeviceType::IMU:
            return "imu";
        case DeviceType::DUALANT:
            return "dual_ant";
        case DeviceType::ODOMETER:
            return "odometer";
        default:
            return "UNKNOWN";
    }
}

inline std::string GetParamName(const ParamType &type) {
    switch (type) {
        case ParamType::INTRINSIC:
            return "intrinsic";
        case ParamType::EXTRINSIC:
            return "extrinsic";
        case ParamType::HOMOGRAPHY:
            return "homography";
        case ParamType::HMATRIX:
            return "h_matrix";
        case ParamType::HARDWARE:
            return "hardware";
        case ParamType::DESIGNINFO:
            return "designinfo";
        case ParamType::CURRENT_CALIBRATION_RESULT:
            return "current_calibration_result";
        default:
            return "UNKNOWN";
    }
}

#define DECLARE_IS_FUNCTION(value, type, name) \
    virtual bool Is##name() const final { return (value) == (type); }

class SensorInfo {
 public:
    // Default constructor
    SensorInfo()
        : sensor_name("unknown"),
          target_sensor_name("unknown"),
          device_type(DeviceType::UNKNOWN),
          param_type(ParamType::UNKNOWN) {}
    explicit SensorInfo(const std::string &sname,
                        const std::string &tname,
                        const DeviceType &dtype,
                        const ParamType &ptype)
        : sensor_name(sname),
          target_sensor_name(tname),
          device_type(dtype),
          param_type(ptype) {}
    virtual ~SensorInfo() = default;
    // Helper functions
    DECLARE_IS_FUNCTION(param_type, ParamType::INTRINSIC, Intrinsic);
    DECLARE_IS_FUNCTION(param_type, ParamType::EXTRINSIC, Extrinsic);
    DECLARE_IS_FUNCTION(param_type, ParamType::HOMOGRAPHY, Homography);
    DECLARE_IS_FUNCTION(param_type, ParamType::HMATRIX, Hmatrix);
    DECLARE_IS_FUNCTION(param_type, ParamType::HARDWARE, Hardware);
    DECLARE_IS_FUNCTION(param_type, ParamType::DESIGNINFO, DesignInfo);
    DECLARE_IS_FUNCTION(param_type,
                        ParamType::CURRENT_CALIBRATION_RESULT,
                        CurrentCalibrationResult);
    DECLARE_IS_FUNCTION(device_type, DeviceType::CAMERA, Camera);
    DECLARE_IS_FUNCTION(device_type, DeviceType::LIDAR, Lidar);
    DECLARE_IS_FUNCTION(device_type, DeviceType::CAN, Can);
    DECLARE_IS_FUNCTION(device_type, DeviceType::GNSS, Gnss);
    DECLARE_IS_FUNCTION(device_type, DeviceType::RADAR, Radar);
    DECLARE_IS_FUNCTION(device_type, DeviceType::RELATIONAL, Relational);
    DECLARE_IS_FUNCTION(device_type, DeviceType::VEHICLE, Vehicle);
    DECLARE_IS_FUNCTION(device_type, DeviceType::IMU, Imu);
    DECLARE_IS_FUNCTION(device_type, DeviceType::DUALANT, Dual_ant);
    DECLARE_IS_FUNCTION(device_type, DeviceType::ODOMETER, Odometer);
    // Getter
    virtual std::string GetDeviceName() const final {
        return sensorconfig::GetDeviceName(this->device_type);
    }
    virtual std::string GetParamName() const final {
        return sensorconfig::GetParamName(this->param_type);
    }

 public:
    std::string sensor_name;
    std::string target_sensor_name;
    DeviceType device_type;
    ParamType param_type;
    Any param;
};
#undef DECLARE_IS_FUNCTION

typedef struct StructExternalCalib {
    StructExternalCalib() = default;
    StructExternalCalib(const StructExternalCalib &other)
        : sensor_calib(other.sensor_calib.clone()), time_lag(other.time_lag) {}
    cv::Mat sensor_calib;
    int64_t time_lag;
} ExternalCalib;

typedef struct StructCamInternalCalib {
    StructCamInternalCalib() = default;
    StructCamInternalCalib(const StructCamInternalCalib &other)
        : img_dist_w(other.img_dist_w),
          img_dist_h(other.img_dist_h),
          img_new_w(other.img_new_w),
          img_new_h(other.img_new_h),
          img_dist_type(other.img_dist_type),
          cam_K(other.cam_K.clone()),
          cam_dist(other.cam_dist.clone()),
          cam_K_new(other.cam_K_new.clone()) {}
    uint32_t img_dist_w;
    uint32_t img_dist_h;
    uint32_t img_new_w;
    uint32_t img_new_h;
    std::string img_dist_type;
    cv::Mat cam_K;
    cv::Mat cam_dist;
    cv::Mat cam_K_new;
} CamInternalCalib;

typedef struct StructCamHomography {
    std::vector<cv::Point2f> src_quad;
    std::vector<cv::Point2f> dst_quad;
} CamHomography;

typedef struct StructHMatrix {
    cv::Mat h_matrix;
} HMatrix;

// Lidar calibration config
typedef struct StructLidarCalib {
    // lidar type; used to find calibration file.
    std::string lidar_type;
    // lidar on/off
    bool is_open;
    // input scan msg topic
    std::string lidar_ids;
    std::string target_frame;
    // input lidar ip & port
    std::string ip_addr;
    uint16_t port;
    // is fusion
    bool is_fusion;
    // use_undist
    bool use_undist;
    double max_range_m;
    double min_range_m;
    double view_direction_rad;
    double view_width_rad;
    int32_t buffer_size;
    int16_t max_output_count;

    int min_angle_1e2deg;  ///< minimum angle to publish
    int max_angle_1e2deg;  ///< maximum angle to publish
    uint8_t running_lidar_id;

    Any additional_param;
} LidarCalib;

typedef struct StructRsLidarAdditionalCalib {
    uint16_t status_port;
    int32_t status_buffer_size;
    bool is_open;
} RsLidarAdditionalCalib;

typedef struct StructHesaiLidarAdditionalCalib {
    int32_t time_zone;
    uint64_t tz_ns;
    bool is_gps_time;
} HesaiLidarAdditionalCalib;

// calibrate can with vehicle static state
typedef struct StructCanCalib {
    // angular_yaw_rate scale from calibration
    double can_yaw_rate_scale;
    // angular_yaw_rate offset from calibration(rad/s)
    double can_yaw_rate_offset;
    // vehicle_speed scale from calibration
    double can_velocity_scale;
    // vehicle_speed offset from calibration(m/s)
    double can_velocity_offset;
} CanCalib;

typedef struct StructGnssCalib {
    double roll;
    double pitch;
    double yaw;
    double heading;
} GnssCalib;

typedef struct StructHardwareInfo {
    std::string type;
    Any property;
} HardwareInfo;

typedef struct StructOdometerCalib {
    double speed_scale;
} OdometerCalib;

typedef struct StructDualantCalib {
    double heading_offset;
} DualantCalib;

typedef struct StructIMUCalib {
    double accel_bias[3];
    double gyro_bias[3];
    double accel_scale[3];
    double gyro_scale[3];
} IMUCalib;

typedef struct StructDesignInfo {
    // unit in meter
    float designed_vertical_offset;
    float designed_lateral_offset;
    float designed_height_offset;
    // unit in rad
    float designed_yaw_rad;
    float designed_pitch_rad;
    float designed_roll_rad;
    // unit in meter
    float threshold_vertical_offset;
    float threshold_lateral_offset;
    float threshold_height_offset;
    // unit in rad
    float threshold_yaw_rad;
    float threshold_pitch_rad;
    float threshold_roll_rad;
} DesignInfo;

typedef struct StructCalibrationStatus {
    uint8_t status_code;
    uint16_t failure_code;
} CalibrationStatus;

typedef struct StructRotation {
    float yaw;
    float pitch;
    float roll;
} Rotation;

typedef struct StructPose6Dof {
    Point3F_t position;
    StructRotation rotation;
} Pose6Dof;

typedef struct StructLocalTime {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} LocalTime;

typedef struct StructCalibrationResult {
    CalibrationStatus calibration_result;
    Pose6Dof error_to_design;
    LocalTime calibration_time;
} CalibrationResult;

}  // namespace sensorconfig
}  // namespace senseAD
