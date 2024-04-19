/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * Deng Long <denglong@sensetime.com>
 */

#pragma once

#include <string>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <opencv2/core/core.hpp>

#include "ad_scm/data_type/status.hpp"
#include "ad_scm/data_type/sensor_calib.hpp"
#include "ad_scm/data_type/camera_property.hpp"
#include "ad_scm/data_type/lidar_property.hpp"
#include "ad_scm/data_type/vehicle_property.hpp"

namespace senseAD {
namespace sensorconfig {

/*
 * @brief Global API for the initialization of sensor configuration manager
 * @param config_file [in] specify the config path. If null, try to get from
 * @return SCM_SUCCESS if succeed
 * @Note if sensor_folder[in] is empty, program will try to get from the
 * environment variable 'SENSOR_CONFIG_FOLDER'
 */
scmStatus_t InitializeSensorConfigManager(
    const std::string &sensor_folder = "");

/*
 * @brief Reset sensor configuration manager
 */
void ResetSensorConfigManager();

/*
 * @brief Get the camera control list
 * @param camera_control [out] output the camera control list
 * @param config_file [in] specify the config path. If null, try to get from
 * environment variable
 * @return SCM_SUCCESS if succeed
 */
scmStatus_t SpecifyCameraControlConfigFile(CameraControl *camera_control,
                                           const std::string &config_path = "");

/*
 * @brief Get the initialized sensor configuration folder
 * @return current initialized sensor configuration folder
 */
std::string GetSensorConfigFolder();

scmStatus_t SaveToJson(const std::string &save_path, const SensorInfo &data);

class SensorConfigManager {
 public:
    template <typename SpecificInfoType>
    using SensorMap_t = std::unordered_map<std::string, SpecificInfoType>;
    template <typename SpecificInfoType>
    using SensorArray_t =
        std::unordered_map<std::string,
                           std::unordered_map<std::string, SpecificInfoType>>;
    typedef std::shared_ptr<SensorInfo> SensorInfoPtr;
    typedef std::vector<SensorInfoPtr> SensorInfoList;

    ~SensorConfigManager() {}
    /*
     * @brief Get global sensor configuration manager instance
     */
    static SensorConfigManager *GetInstance();

    /*
     * @brief Initialize sensor config manager
     * @param config folder [in] specify the folder to search for configuration
     * files
     * @return flag that indicates succeed of failed
     */
    scmStatus_t Init(const std::string &config_folder);

    /*
     * @brief Get the extrinsic parameters from sensor to target sensor
     * @param sensor [in] specify the source sensor name
     * @param target_sensor [in] specify the target sensor name
     * @param trans_mat [out] output the transformation matrix
     * @return flag that indicates succeed of failed
     */
    scmStatus_t GetExternalCalibConfig(std::string sensor,
                                       std::string target_sensor,
                                       cv::Mat *trans_mat) const;

    /*
     * @brief Get the time lag from sensor to target sensor
     * @param sensor [in] specify the source sensor name
     * @param target_sensor [in] specify the target sensor name
     * @param time_lag [out] output the time lag
     * @return flag that indicates succeed of failed
     */
    scmStatus_t GetClockLagBetweenTwoSensors(std::string sensor,
                                             std::string target_sensor,
                                             int64_t &time_lag) const;

    /*
     * @brief Get the camera intrinsic parameter
     * @param cam_sensor [in] specify the camera sensor name
     * @param cam_internal_calib [out] output the specific camera intrinsic
     * parameter
     * @return flag that indicates succeed of failed
     */
    scmStatus_t GetCameraInternalCalibConfig(
        std::string cam_sensor, CamInternalCalib *cam_internal_calib) const;

    /*
     * @brief Get the sensor design info parameter
     * @param sensor_name [in] specify the sensor name
     * @param struct_design_calib [out] output the specific sensor design info
     * parameter
     * @return flag that indicates succeed of failed
     */
    scmStatus_t GetSensorDesignInfoConfig(
        std::string sensor_name, DesignInfo *struct_design_calib) const;
    /*
     * @brief Get the current calibration result info parameter
     * @param sensor_name [in] specify the sensor name
     * @param struct_calibration_result [out] output the specific sensor design
     * info parameter
     * @return flag that indicates succeed of failed
     */
    scmStatus_t GetCurrentCalibrationResultConfig(
        std::string sensor_name,
        std::string target_sensor,
        CalibrationResult *struct_calibration_result) const;
    /*
     * @brief Set the camera intrinsic parameter
     * @param cam_sensor [in] specify the camera sensor name
     * @param cam_internal_calib [in]  specific the camera intrinsic parameter
     * @return flag that indicates succeed of failed
     * @note: if the sensor has already existed, replace the old value with the
     * new value
     */
    scmStatus_t SetCameraInternalCalib(
        const std::string &cam_sensor,
        const CamInternalCalib &cam_internal_calib);

    /*
     * @brief Get the camera homography parameter
     * @param cam_sensor [in] specify the camera sensor name
     * @param cam_homography [out] output the specific camera cam_homography
     * parameter
     * @return flag that indicates succeed of failed
     */
    scmStatus_t GetHomographyConfig(std::string cam_sensor,
                                    CamHomography *cam_homography) const;
    /*
     * @brief Get the camera h_matrix parameter
     * @param cam_sensor [in] specify the camera sensor name
     * @param cam_homography [out] output the specific camera cam_h_matrix
     * parameter
     * @return flag that indicates succeed of failed
     */
    scmStatus_t GetHMatrixConfig(std::string cam_sensor,
                                 HMatrix *cam_h_matrix) const;

    /*
     * @brief Get the CAN intrinsic parameter
     * @param can_calib [out] output the CAN intrinsic parameter
     * @return flag that indicates succeed of failed
     * @note currently the CAN sensor name in configuration file must be 'can'
     */
    scmStatus_t GetCanCalibConfig(CanCalib *can_calib) const;

    /*
     * @brief Get the GNSS intrinsic parameter
     * @param gnss_calib [out] output the GNSS intrinsic parameter
     * @return flag that indicates succeed of failed
     * @note currently the GNSS sensor name in configuration file must be 'gnss'
     */
    scmStatus_t GetGnssCalibConfig(GnssCalib *gnss_calib) const;

    /*
     * @brief Get the IMU intrinsic parameter
     * @param imu_calib [out] output the IMU intrinsic parameter
     * @return flag that indicates succeed of failed
     * @note currently the IMU sensor name in configuration file must be 'imu'
     */
    scmStatus_t GetIMUCalibConfig(IMUCalib *imu_calib) const;

    /*
     * @brief Get the ODOMETER intrinsic parameter
     * @param odometer_calib [out] output the odometer intrinsic parameter
     * @return flag that indicates succeed of failed
     * @note currently the ODOMETER sensor name in configuration file must be
     * 'odometer'
     */
    scmStatus_t GetODOMETERCalibConfig(OdometerCalib *odometer_calib) const;

    /*
     * @brief Get the DUAL_ANT intrinsic parameter
     * @param dual_ant_calib [out] output the dual_ant intrinsic parameter
     * @return flag that indicates succeed of failed
     * @note currently the DUAL_ANT sensor name in configuration file must be
     * 'dual_ant'
     */
    scmStatus_t GetDUALANTCalibConfig(DualantCalib *dual_ant_calib) const;

    /*
     * @brief Get the name list of all registered sensors
     * @param sensor_list [out] output the name list of all registered sensors
     * @return flag that indicates succeed of failed
     */
    scmStatus_t GetAllSensorName(std::vector<std::string> *sensor_list) const;

    /*
     * @brief Get the relationship of sensors
     * @param relationship_list [out] output the relationship list of all sensor
     * pairs
     * @return flag that indicates succeed of failed
     */
    scmStatus_t GetRelationshipPair(
        std::vector<std::pair<std::string, std::string>> *relationship_list)
        const;

    /*
     * @brief Get the All hardware informations which satisfy the type and param
     * @param hardware_info [out] output the specific hardware information
     * @param param_type [in] return only the hardware info of parameter type
     * "param_type". UNKNWON means no selection
     * @param device_type [in] return only the hardware info of device type
     * "device_type". UNKNWON means no selection
     * @return flag that indicates succeed of failed
     */
    scmStatus_t GetAllCameraHardwareInfo(
        SensorMap_t<HardwareInfo> *hardware_infos) const;
    /*
     * @brief Get the hardware information
     * @param sensor_name [in] specify the sensor name
     * @param hardware_info [out] output the specific hardware information
     * @return flag that indicates succeed of failed
     */
    scmStatus_t GetCameraHardwareInfo(const std::string &sensor_name,
                                      HardwareInfo *hardware_info) const;

    /*
     * @brief Get the hardware information
     * @param sensor_name [in] specify the sensor name
     * @param hardware_info [out] output the specific hardware information
     * @return flag that indicates succeed of failed
     */
    scmStatus_t GetLidarHardwareInfo(const std::string &sensor_name,
                                     LidarHardwareConfig *hardware_info) const;

    /*
     * @brief Get the vehicle hardware information
     * @param sensor_name [in] specify the sensor name
     * @param hardware_info [out] output the specific hardware information
     * @return flag that indicates succeed of failed
     */
    scmStatus_t GetVehicleConfigInfo(const std::string &sensor_name,
                                     VehicleConfig *hardware_info) const;

    /*
     * @brief Get the lidar intrinsic parameter
     * @param sensor_name [in] specify the lidar sensor name
     * @param lidar_info [out] output the specific lidar intrinsic parameter
     * @return flag that indicates succeed of failed
     */
    scmStatus_t GetLidarInternalCalibConfig(const std::string &sensor_name,
                                            LidarCalib *lidar_info) const;

    /*
     * @brief Get all lidar intrinsic parameter
     * @param lidar_infos [out] output all lidars intrinsic parameter
     * @return flag that indicates succeed of failed
     */
    scmStatus_t GetAllLidarInternalCalibConfig(
        SensorMap_t<LidarCalib> *lidar_infos) const;

    /*
     * @brief Get the hardware information
     * @param source [in] specify the source sensor name
     * @param target [in] output the target sensor name
     * @param sensor_info [out] output the list specific sensor info
     * @return flag that indicates succeed of failed
     */
    scmStatus_t GetSensorInfoList(const std::string &source,
                                  const std::string &target,
                                  SensorInfoList *sensor_info_list) const;

    void Reset();

 private:
    SensorConfigManager() = default;
    SensorConfigManager(const SensorConfigManager &) = delete;
    SensorConfigManager &operator=(const SensorConfigManager &) = delete;
    template <typename SpecificInfoType>
    bool RestoreInfo(const SensorInfo &sensor_info,
                     SpecificInfoType *specific_info) const;
    template <typename SpecificInfoType>
    bool RegisterInfo(const std::string &sensor_name,
                      const std::string &target_sensor_name,
                      const SensorInfo &container,
                      SensorArray_t<SpecificInfoType> &map) const;
    void SetupLidarRunningId();

    std::vector<std::pair<std::string, std::string>> relationship_list_;
    std::map<std::pair<std::string, std::string>, cv::Mat> external_calib_;
    std::map<std::pair<std::string, std::string>, int64_t> sensor_time_lag_;
    std::map<std::string, CamInternalCalib> cam_internal_calib_;
    std::map<std::string, CamHomography> cam_homography_;
    std::map<std::string, HMatrix> cam_h_matrix_;
    CanCalib can_calib_;
    GnssCalib gnss_calib_;
    IMUCalib imu_calib_;
    DualantCalib dual_ant_calib_;
    OdometerCalib odometer_calib_;

    std::unordered_set<std::string> registered_sensor_names_;
    SensorArray_t<SensorInfoList> sensor_info_array_;
    SensorArray_t<CamInternalCalib> camera_intrinsic_array_;
    SensorArray_t<CamHomography> camera_homography_array_;
    SensorArray_t<HMatrix> camera_h_matrix_array_;
    // SensorArray_t<CanCalib> can_intrinsic_array_;
    // SensorArray_t<GnssCalib> gnss_intrinsic_array_;
    SensorArray_t<ExternalCalib> extrinsic_array_;
    SensorArray_t<HardwareInfo> camera_hardware_array_;
    SensorArray_t<LidarCalib> lidar_intrinsic_array_;
    SensorArray_t<HardwareInfo> lidar_hardware_array_;
    SensorArray_t<HardwareInfo> vehicle_hardware_array_;
    SensorArray_t<DesignInfo> sensor_designinfo_array_;
    SensorArray_t<CalibrationResult> current_calibration_result_array_;
};
}  // namespace sensorconfig
}  // namespace senseAD
