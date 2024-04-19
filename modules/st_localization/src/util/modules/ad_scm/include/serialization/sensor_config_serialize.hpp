/*
 * Copyright (C) 2019-2020 by SenseTime Group Limited. All rights reserved.
 * Deng Long <denglong@sensetime.com>
 */

#pragma once

// should always include following 4 cereal's archives
#include "cereal/archives/binary.hpp"
#include "cereal/archives/json.hpp"
#include "cereal/archives/xml.hpp"
#include "cereal/archives/portable_binary.hpp"
#include "cereal/types/string.hpp"
#include "cereal/types/vector.hpp"
#include "cereal/types/map.hpp"
// private user components header
#include "ad_scm/data_type/sensor_calib.hpp"
#include "ad_scm/data_type/camera_property.hpp"
#include "ad_scm/data_type/lidar_property.hpp"
#include "ad_scm/data_type/vehicle_property.hpp"
#include "serialization/camera_property_serialize.hpp"
#include "serialization/lidar_property_serialize.hpp"
// public user components header
#include "ad_common/config_utils/macros.hpp"
#include "ad_log/ad_log.hpp"

namespace senseAD {
namespace sensorconfig {

#define REGITSTER_SENSOR_DEVICE_TYPE(XX)     \
    XX(DeviceType::CAMERA, "camera")         \
    XX(DeviceType::LIDAR, "lidar")           \
    XX(DeviceType::CAN, "can")               \
    XX(DeviceType::GNSS, "gnss")             \
    XX(DeviceType::RADAR, "radar")           \
    XX(DeviceType::RELATIONAL, "relational") \
    XX(DeviceType::GENERAL, "general")       \
    XX(DeviceType::VEHICLE, "vehicle")       \
    XX(DeviceType::IMU, "imu")               \
    XX(DeviceType::DUALANT, "dual_ant")      \
    XX(DeviceType::ODOMETER, "odometer")     \
    XX(DeviceType::UNKNOWN, "unknown")

REGISTER_CEREAL_ENUM_SERIALIZE(DeviceType,
                               REGITSTER_SENSOR_DEVICE_TYPE)  // NOLINT

#define REGITSTER_SENSOR_PARAM_TYPE(XX)                                     \
    XX(ParamType::INTRINSIC, "intrinsic")                                   \
    XX(ParamType::EXTRINSIC, "extrinsic")                                   \
    XX(ParamType::HOMOGRAPHY, "homography")                                 \
    XX(ParamType::HMATRIX, "h_matrix")                                      \
    XX(ParamType::DESIGNINFO, "designinfo")                                 \
    XX(ParamType::CURRENT_CALIBRATION_RESULT, "current_calibration_result") \
    XX(ParamType::HARDWARE, "hardware")                                     \
    XX(ParamType::UNKNOWN, "unknown")

REGISTER_CEREAL_ENUM_SERIALIZE(ParamType,
                               REGITSTER_SENSOR_PARAM_TYPE)  // NOLINT

template <class Archive>
void save(Archive &archive, const SensorInfo &sensor_info) {  // NOLINT
    const auto &sensor_name = sensor_info.sensor_name;
    const auto &target_sensor_name = sensor_info.target_sensor_name;
    const auto &device_type = sensor_info.device_type;
    const auto &param_type = sensor_info.param_type;
    archive(cereal::make_nvp("sensor_name", sensor_name));
    archive(cereal::make_nvp("target_sensor_name", target_sensor_name));
    archive(cereal::make_nvp("device_type", device_type));
    archive(cereal::make_nvp("param_type", param_type));
    if (device_type == DeviceType::CAMERA &&
        param_type == ParamType::INTRINSIC) {
        const auto &param = sensor_info.param.GetValue<CamInternalCalib>();
        archive(cereal::make_nvp("param", *param));
    } else if (device_type == DeviceType::CAMERA &&
               param_type == ParamType::HOMOGRAPHY) {
        const auto &param = sensor_info.param.GetValue<CamHomography>();
        archive(cereal::make_nvp("param", *param));
    } else if (device_type == DeviceType::CAMERA &&
               param_type == ParamType::HMATRIX) {
        const auto &param = sensor_info.param.GetValue<HMatrix>();
        archive(cereal::make_nvp("param", *param));
    } else if (device_type == DeviceType::CAN &&
               param_type == ParamType::INTRINSIC) {
        const auto &param = sensor_info.param.GetValue<CanCalib>();
        archive(cereal::make_nvp("param", *param));
    } else if (device_type == DeviceType::GNSS &&
               param_type == ParamType::INTRINSIC) {
        const auto &param = sensor_info.param.GetValue<GnssCalib>();
        archive(cereal::make_nvp("param", *param));
    } else if (device_type == DeviceType::LIDAR &&
               param_type == ParamType::INTRINSIC) {
        const auto &param = sensor_info.param.GetValue<LidarCalib>();
        archive(cereal::make_nvp("param", *param));
    } else if (param_type == ParamType::EXTRINSIC) {
        const auto &param = sensor_info.param.GetValue<ExternalCalib>();
        archive(cereal::make_nvp("param", *param));
    } else if (param_type == ParamType::HARDWARE) {
        const auto &param = sensor_info.param.GetValue<HardwareInfo>();
        archive(cereal::make_nvp("param", *param));
    } else if (param_type == ParamType::DESIGNINFO) {
        const auto &param = sensor_info.param.GetValue<StructDesignInfo>();
        archive(cereal::make_nvp("param", *param));
    } else if (param_type == ParamType::CURRENT_CALIBRATION_RESULT) {
        const auto &param =
            sensor_info.param.GetValue<StructCalibrationResult>();
        archive(cereal::make_nvp("param", *param));
    }
}

template <class Archive>
void load(Archive &archive, SensorInfo &sensor_info) {  // NOLINT
    auto &sensor_name = sensor_info.sensor_name;
    auto &target_sensor_name = sensor_info.target_sensor_name;
    auto &device_type = sensor_info.device_type;
    auto &param_type = sensor_info.param_type;
    archive(cereal::make_nvp("sensor_name", sensor_name));
    archive(cereal::make_nvp("target_sensor_name", target_sensor_name));
    archive(cereal::make_nvp("device_type", device_type));
    archive(cereal::make_nvp("param_type", param_type));
    if (device_type == DeviceType::CAMERA &&
        param_type == ParamType::INTRINSIC) {
        CamInternalCalib param;
        archive(cereal::make_nvp("param", param));
        sensor_info.param = Any(param);
    } else if (device_type == DeviceType::CAMERA &&
               param_type == ParamType::HOMOGRAPHY) {
        CamHomography param;
        archive(cereal::make_nvp("param", param));
        sensor_info.param = Any(param);
    } else if (device_type == DeviceType::CAMERA &&
               param_type == ParamType::HMATRIX) {
        HMatrix param;
        archive(cereal::make_nvp("param", param));
        sensor_info.param = Any(param);
    } else if (device_type == DeviceType::CAN &&
               param_type == ParamType::INTRINSIC) {
        CanCalib param;
        archive(cereal::make_nvp("param", param));
        sensor_info.param = Any(param);
    } else if (device_type == DeviceType::GNSS &&
               param_type == ParamType::INTRINSIC) {
        GnssCalib param;
        archive(cereal::make_nvp("param", param));
        sensor_info.param = Any(param);
    } else if (device_type == DeviceType::LIDAR &&
               param_type == ParamType::INTRINSIC) {
        LidarCalib param;
        archive(cereal::make_nvp("param", param));
        sensor_info.param = Any(param);
    } else if (param_type == ParamType::EXTRINSIC) {
        ExternalCalib param;
        archive(cereal::make_nvp("param", param));
        sensor_info.param = Any(param);
    } else if (param_type == ParamType::HARDWARE) {
        HardwareInfo param;
        archive(cereal::make_nvp("param", param));
        sensor_info.param = Any(param);
    } else if (device_type == DeviceType::IMU &&
               param_type == ParamType::INTRINSIC) {
        IMUCalib param;
        archive(cereal::make_nvp("param", param));
        sensor_info.param = Any(param);
    } else if (device_type == DeviceType::ODOMETER &&
               param_type == ParamType::INTRINSIC) {
        OdometerCalib param;
        archive(cereal::make_nvp("param", param));
        sensor_info.param = Any(param);
    } else if (device_type == DeviceType::DUALANT &&
               param_type == ParamType::INTRINSIC) {
        DualantCalib param;
        archive(cereal::make_nvp("param", param));
        sensor_info.param = Any(param);
    } else if (param_type == ParamType::DESIGNINFO) {
        StructDesignInfo param;
        archive(cereal::make_nvp("param", param));
        sensor_info.param = Any(param);
    } else if (param_type == ParamType::CURRENT_CALIBRATION_RESULT) {
        StructCalibrationResult param;
        archive(cereal::make_nvp("param", param));
        sensor_info.param = Any(param);
    }
}

REGISTER_CEREAL_SERIALIZE(ExternalCalib &result) {  // NOLINT
    CEREAL_PAIR(result, time_lag);
    CEREAL_PAIR(result, sensor_calib);
}

REGISTER_CEREAL_SERIALIZE(CamHomography &result) {  // NOLINT
    CEREAL_PAIR(result, src_quad);
    CEREAL_PAIR(result, dst_quad);
}

REGISTER_CEREAL_SERIALIZE(HMatrix &result) {  // NOLINT
    CEREAL_PAIR(result, h_matrix);
}

REGISTER_CEREAL_SERIALIZE(CamInternalCalib &result) {  // NOLINT
    CEREAL_PAIR(result, img_dist_w);
    CEREAL_PAIR(result, img_dist_h);
    CEREAL_PAIR(result, img_new_w);
    CEREAL_PAIR(result, img_new_h);
    CEREAL_PAIR(result, img_dist_type);
    CEREAL_PAIR(result, cam_K);
    CEREAL_PAIR(result, cam_dist);
    CEREAL_OPTIONAL_PAIR(result, cam_K_new, cv::Mat());
}

REGISTER_CEREAL_SERIALIZE(StructDesignInfo &result) {  // NOLINT
    CEREAL_PAIR(result, designed_vertical_offset);
    CEREAL_PAIR(result, designed_lateral_offset);
    CEREAL_PAIR(result, designed_height_offset);
    CEREAL_PAIR(result, designed_yaw_rad);
    CEREAL_PAIR(result, designed_pitch_rad);
    CEREAL_PAIR(result, designed_roll_rad);
    CEREAL_PAIR(result, threshold_vertical_offset);
    CEREAL_PAIR(result, threshold_lateral_offset);
    CEREAL_PAIR(result, threshold_height_offset);
    CEREAL_PAIR(result, threshold_yaw_rad);
    CEREAL_PAIR(result, threshold_pitch_rad);
    CEREAL_PAIR(result, threshold_roll_rad);
}

template <class Archive>
void save(Archive &archive, const LidarCalib &result) {  // NOLINT
    CEREAL_PAIR(result, lidar_type);
    CEREAL_PAIR(result, is_open);
    CEREAL_PAIR(result, lidar_ids);
    CEREAL_PAIR(result, is_fusion);
    CEREAL_PAIR(result, use_undist);
    CEREAL_PAIR(result, max_range_m);
    CEREAL_PAIR(result, min_range_m);
    CEREAL_PAIR(result, view_direction_rad);
    CEREAL_PAIR(result, view_width_rad);
    CEREAL_PAIR(result, target_frame);
    CEREAL_PAIR(result, ip_addr);
    CEREAL_PAIR(result, port);
    CEREAL_PAIR(result, buffer_size);
    CEREAL_PAIR(result, max_output_count);

    if (result.lidar_type == "RS32") {
        const auto &param =
            result.additional_param.GetValue<RsLidarAdditionalCalib>();
        archive(cereal::make_nvp("additional_param", *param));
    } else if (result.lidar_type == "Pandar64" ||
               result.lidar_type == "Pandar40" ||
               result.lidar_type == "PandarQT") {
        const auto &param =
            result.additional_param.GetValue<HesaiLidarAdditionalCalib>();
        archive(cereal::make_nvp("additional_param", *param));
    }
}

template <class Archive>
void load(Archive &archive, LidarCalib &result) {  // NOLINT
    CEREAL_PAIR(result, lidar_type);
    CEREAL_PAIR(result, is_open);
    CEREAL_PAIR(result, lidar_ids);
    CEREAL_PAIR(result, is_fusion);
    CEREAL_PAIR(result, use_undist);
    CEREAL_PAIR(result, max_range_m);
    CEREAL_PAIR(result, min_range_m);
    CEREAL_PAIR(result, view_direction_rad);
    CEREAL_PAIR(result, view_width_rad);
    CEREAL_PAIR(result, target_frame);
    CEREAL_PAIR(result, ip_addr);
    CEREAL_PAIR(result, port);
    CEREAL_PAIR(result, buffer_size);
    CEREAL_PAIR(result, max_output_count);

    double tmp_min_angle_rad;
    double tmp_max_angle_rad;

    if (result.view_width_rad > 2 * M_PI) {
        result.view_width_rad = 2 * M_PI;
    }

    // converting angle parameters into the velodyne reference (rad)
    tmp_min_angle_rad = result.view_direction_rad + result.view_width_rad / 2;
    tmp_max_angle_rad = result.view_direction_rad - result.view_width_rad / 2;

    // computing positive modulo to keep theses angles into [0;2*M_PI]
    tmp_min_angle_rad =
        fmod(fmod(tmp_min_angle_rad, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
    tmp_max_angle_rad =
        fmod(fmod(tmp_max_angle_rad, 2 * M_PI) + 2 * M_PI, 2 * M_PI);

    result.min_angle_1e2deg =
        100 * (2. * M_PI - tmp_min_angle_rad) * 180. / M_PI + 0.5;
    result.max_angle_1e2deg =
        100 * (2. * M_PI - tmp_max_angle_rad) * 180. / M_PI + 0.5;
    if (result.min_angle_1e2deg == result.max_angle_1e2deg) {
        result.min_angle_1e2deg = 0;
        result.max_angle_1e2deg = 36000;
    }

    if (result.lidar_type == "RS32") {
        RsLidarAdditionalCalib additional_param;
        archive(cereal::make_nvp("additional_param", additional_param));
        result.additional_param = Any(additional_param);
    } else if (result.lidar_type == "Pandar64" ||
               result.lidar_type == "Pandar40" ||
               result.lidar_type == "PandarQT") {
        HesaiLidarAdditionalCalib additional_param;
        archive(cereal::make_nvp("additional_param", additional_param));
        result.additional_param = Any(additional_param);
    }
}

REGISTER_CEREAL_SERIALIZE(RsLidarAdditionalCalib &result) {  // NOLINT
    CEREAL_PAIR(result, status_port);
    CEREAL_PAIR(result, is_open);
    CEREAL_PAIR(result, status_buffer_size);
}

REGISTER_CEREAL_SERIALIZE(HesaiLidarAdditionalCalib &result) {  // NOLINT
    CEREAL_PAIR(result, is_gps_time);
    CEREAL_PAIR(result, time_zone);
    result.tz_ns = result.time_zone * 3600 * 1e9;
}

REGISTER_CEREAL_SERIALIZE(CanCalib &result) {  // NOLINT
    CEREAL_PAIR(result, can_yaw_rate_scale);
    CEREAL_PAIR(result, can_yaw_rate_offset);
    CEREAL_PAIR(result, can_velocity_scale);
    CEREAL_PAIR(result, can_velocity_offset);
}

REGISTER_CEREAL_SERIALIZE(StructGnssCalib &result) {  // NOLINT
    CEREAL_PAIR(result, roll);
    CEREAL_PAIR(result, pitch);
    CEREAL_PAIR(result, yaw);
    CEREAL_PAIR(result, heading);
}

REGISTER_CEREAL_SERIALIZE(IMUCalib &result) {  // NOLINT
    CEREAL_PAIR(result, accel_bias);
    CEREAL_PAIR(result, gyro_bias);
    CEREAL_PAIR(result, accel_scale);
    CEREAL_PAIR(result, gyro_scale);
}

REGISTER_CEREAL_SERIALIZE(DualantCalib &result) {  // NOLINT
    CEREAL_PAIR(result, heading_offset);
}

REGISTER_CEREAL_SERIALIZE(OdometerCalib &result) {  // NOLINT
    CEREAL_PAIR(result, speed_scale);
}

REGISTER_CEREAL_SERIALIZE(CalibrationStatus &result) {  // NOLINT
    CEREAL_PAIR(result, status_code);
    CEREAL_PAIR(result, failure_code);
}

REGISTER_CEREAL_SERIALIZE(Rotation &result) {  // NOLINT
    CEREAL_PAIR(result, yaw);
    CEREAL_PAIR(result, pitch);
    CEREAL_PAIR(result, roll);
}

REGISTER_CEREAL_SERIALIZE(Pose6Dof &result) {  // NOLINT
    CEREAL_PAIR(result, position);
    CEREAL_PAIR(result, rotation);
}

REGISTER_CEREAL_SERIALIZE(LocalTime &result) {  // NOLINT
    CEREAL_PAIR(result, year);
    CEREAL_PAIR(result, month);
    CEREAL_PAIR(result, day);
    CEREAL_PAIR(result, hour);
    CEREAL_PAIR(result, minute);
    CEREAL_PAIR(result, second);
}

REGISTER_CEREAL_SERIALIZE(CalibrationResult &result) {  // NOLINT
    CEREAL_PAIR(result, calibration_result);
    CEREAL_PAIR(result, calibration_time);
    CEREAL_PAIR(result, error_to_design);
}

template <class Archive>
void save(Archive &archive, const HardwareInfo &hardware_info) {  // NOLINT
    archive(cereal::make_nvp("type", hardware_info.type));
    if (hardware_info.type == "CameraIMX290") {
        const auto &property = hardware_info.property.GetValue<IMX290Attr>();
        archive(cereal::make_nvp("property", *property));
    } else if (hardware_info.type == "CameraIMXCustom") {
        const auto &property = hardware_info.property.GetValue<IMXCustomAttr>();
        archive(cereal::make_nvp("property", *property));
    } else if (hardware_info.type == "CameraPylon") {
        const auto &property =
            hardware_info.property.GetValue<CameraPylonAttr>();
        archive(cereal::make_nvp("property", *property));
    } else if (hardware_info.type == "CameraZMQ") {
        const auto &property = hardware_info.property.GetValue<CameraZMQAttr>();
        archive(cereal::make_nvp("property", *property));
    } else if (hardware_info.type == "CameraNvdw") {
        const auto &property = hardware_info.property.GetValue<NvdwAttr>();
        archive(cereal::make_nvp("property", *property));
    } else if (hardware_info.type == "CameraVirtual") {
        const auto &property = hardware_info.property.GetValue<VirtualAttrs>();
        archive(cereal::make_nvp("property", *property));
    } else if (hardware_info.type == "CameraIPC") {
        const auto &property = hardware_info.property.GetValue<IPCCameraAttr>();
        archive(cereal::make_nvp("property", *property));
    } else if (hardware_info.type == "Lidar") {
        const auto &property =
            hardware_info.property.GetValue<LidarHardwareConfig>();
        archive(cereal::make_nvp("property", *property));
    } else if (hardware_info.type == "Vehicle") {
        const auto &property = hardware_info.property.GetValue<VehicleConfig>();
        archive(cereal::make_nvp("property", *property));
    } else if (hardware_info.type == "CameraOffline") {
        const auto &property =
            hardware_info.property.GetValue<CameraOfflineAttr>();
        archive(cereal::make_nvp("property", *property));
    } else if (hardware_info.type == "CameraTda") {
        const auto &property = hardware_info.property.GetValue<CameraTdaAttr>();
        archive(cereal::make_nvp("property", *property));
    } else {
        std::cout << "\033[1;36m"
                  << "[SAVE] Unknown hardware type: " << hardware_info.type
                  << "\033[0m \t" << __FILENAME__ << ":" << __LINE__
                  << std::endl;
    }
}

template <class Archive>
void load(Archive &archive, HardwareInfo &hardware_info) {  // NOLINT
    auto &type = hardware_info.type;
    archive(cereal::make_nvp("type", type));
    if (type == "CameraIMX290") {
        IMX290Attr property;
        archive(cereal::make_nvp("property", property));
        hardware_info.property = Any(property);
    } else if (type == "CameraIMXCustom") {
        IMXCustomAttr property;
        archive(cereal::make_nvp("property", property));
        hardware_info.property = Any(property);
    } else if (type == "CameraPylon") {
        CameraPylonAttr property;
        archive(cereal::make_nvp("property", property));
        hardware_info.property = Any(property);
    } else if (type == "CameraZMQ") {
        CameraZMQAttr property;
        archive(cereal::make_nvp("property", property));
        hardware_info.property = Any(property);
    } else if (type == "CameraNvdw") {
        NvdwAttr property;
        archive(cereal::make_nvp("property", property));
        hardware_info.property = Any(property);
    } else if (type == "CameraOffline") {
        CameraOfflineAttr property;
        archive(cereal::make_nvp("property", property));
        hardware_info.property = Any(property);
    } else if (type == "CameraTda") {
        CameraTdaAttr property;
        archive(cereal::make_nvp("property", property));
        hardware_info.property = Any(property);
    } else if (type == "CameraVirtual") {
        VirtualAttrs property;
        archive(cereal::make_nvp("property", property));
        hardware_info.property = Any(property);
    } else if (hardware_info.type == "CameraIPC") {
        IPCCameraAttr property;
        archive(cereal::make_nvp("property", property));
        hardware_info.property = Any(property);
    } else if (hardware_info.type == "Lidar") {
        LidarHardwareConfig property;
        archive(cereal::make_nvp("property", property));
        hardware_info.property = Any(property);
    } else if (hardware_info.type == "Vehicle") {
        VehicleConfig property;
        archive(cereal::make_nvp("property", property));
        hardware_info.property = Any(property);
    } else {
        std::cout << "\033[1;36m"
                  << "[LOAD] Unknown hardware type to load: "
                  << hardware_info.type << "\033[0m \t" << __FILENAME__ << ":"
                  << __LINE__ << std::endl;
    }
}

}  // namespace sensorconfig
}  // namespace senseAD
