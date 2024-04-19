/*
 * Copyright (C) 2018 by SenseTime Group Limited. All rights reserved.
 * Deng Long <denglong@sensetime.com>
 */

#include "ad_scm/ad_scm.hpp"

#include <atomic>
#include <cstdlib>  // getenv
#include <iostream>

#include "path_util.hpp"
#include "serialization/tf_tree_serialize.hpp"
#include "serialization/sensor_config_serialize.hpp"
#include "serialization/camera_property_serialize.hpp"
#include "serialization/global_property_serialize.hpp"
#include "serialization/stitching_property_serialize.hpp"
#include "serialization/map_matching_property_serialize.hpp"
#include "serialization/gnss_transformer_property_serialize.hpp"
#include "serialization/vehicle_property_serialize.hpp"

#include "ad_common/config_utils/config_utils.hpp"

using ConfigReader = senseAD::common::utils::ConfigurationReader;
using FileSystem = senseAD::common::utils::FileSystem;

namespace senseAD {
namespace sensorconfig {

namespace {  // Internal usage
static bool kIsManagerInitialized = false;
static std::string kSensorConfigurationFolder;  // NOLINT
std::mutex g_mutex;
}  // namespace

scmStatus_t InitializeSensorConfigManager(const std::string &sensor_folder) {
    if (sensor_folder.empty()) {
        std::string folder_path = FileSystem::GetEnv("SENSOR_CONFIG_FOLDER");
        if (!folder_path.empty()) {
            auto ret = SensorConfigManager::GetInstance()->Init(folder_path);
            if (SCM_SUCCESS != ret) {
                AD_LERROR(ADSCM) << "Initialize sensor configuration manager: "
                                 << sensor_folder;
                return ret;
            }
            return SCM_SUCCESS;
        } else {
            AD_LERROR(ADSCM)
                << "Cannot get sensor configuration folder from env: "
                << "[SENSOR_CONFIG_FOLDER]";
            return SCM_INVALID_PARAM;
        }
    } else {
        return SensorConfigManager::GetInstance()->Init(sensor_folder);
    }
    return SCM_SUCCESS;
}

void ResetSensorConfigManager() {
    kIsManagerInitialized = false;
    kSensorConfigurationFolder = "";
    SensorConfigManager::GetInstance()->Reset();
}

scmStatus_t SpecifyCameraControlConfigFile(CameraControl *camera_control,
                                           const std::string &file_path) {
    if (file_path.empty()) {
        AD_LDEBUG(ADSCM) << "Try to get camera control config file from env.";
        char *p = getenv("CAMERA_CONTROL_CONFIG");
        if (p != nullptr) {
            std::string path(p);
            auto ret = ConfigReader::LoadJSON(path, camera_control);
            if (CONF_SUCCESS != ret) {
                AD_LWARN(ADSCM)
                    << "Fails to get camera control from env: " << path;
                return SCM_INVALID_PARAM;
            }
        } else {
            AD_LERROR(ADSCM) << "No path named [CAMERA_CONTROL_CONFIG] found.";
            return SCM_INVALID_PARAM;
        }
    } else {
        auto ret = ConfigReader::LoadJSON(file_path, camera_control);
        if (CONF_SUCCESS != ret) {
            AD_LWARN(ADSCM) << "Fails to get camera control " << file_path;
            return SCM_INVALID_PARAM;
        }
    }
    return SCM_SUCCESS;
}

std::string GetSensorConfigFolder() { return kSensorConfigurationFolder; }

scmStatus_t SaveToJson(const std::string &save_path, const SensorInfo &data) {
    if (confStatus_t::CONF_SUCCESS !=
        ConfigReader::WriteJSON(save_path, data)) {
        return SCM_INVALID_PARAM;
    }
    return SCM_SUCCESS;
}

#define CHECK_SENSOR_EXISTS(sensor_map, sensor, ret)                 \
    if ((sensor_map).find(sensor) == (sensor_map).end()) {           \
        AD_LERROR(ADSCM) << "Sensor: " << sensor << " not exists. "; \
        return ret;                                                  \
    }

void SensorConfigManager::Reset() {
    relationship_list_.clear();
    external_calib_.clear();
    sensor_time_lag_.clear();
    cam_internal_calib_.clear();
    cam_homography_.clear();
    cam_h_matrix_.clear();
    can_calib_ = CanCalib();
    gnss_calib_ = GnssCalib();

    registered_sensor_names_.clear();
    sensor_info_array_.clear();
    camera_intrinsic_array_.clear();
    camera_homography_array_.clear();
    camera_h_matrix_array_.clear();
    extrinsic_array_.clear();
    camera_hardware_array_.clear();
    lidar_intrinsic_array_.clear();
    lidar_hardware_array_.clear();
    vehicle_hardware_array_.clear();
}

template <typename SpecificInfoType>
bool SensorConfigManager::RestoreInfo(const SensorInfo &sensor_info,
                                      SpecificInfoType *specific_info) const {
    if (specific_info == nullptr) {
        AD_LWARN(ADSCM) << "Cannot restore sensor info. [nullptr]";
        return false;
    }
    auto specific_info_ptr = sensor_info.param.GetValue<SpecificInfoType>();
    if (specific_info_ptr == nullptr) {
        AD_LWARN(ADSCM) << "Cannot restore sensor info. [nullptr]";
        return false;
    }
    *specific_info = *specific_info_ptr;
    return true;
}

SensorConfigManager *SensorConfigManager::GetInstance() {
    static SensorConfigManager sensor_config_manager;
    return &sensor_config_manager;
}

template <typename SpecificInfoType>
bool SensorConfigManager::RegisterInfo(  // NOLINT
    const std::string &sensor_name,
    const std::string &target_sensor_name,
    const SensorInfo &container,
    SensorArray_t<SpecificInfoType> &array) const {
    SpecificInfoType info;
    if (!this->RestoreInfo(container, &info)) {
        return false;
    }
    if (array.find(sensor_name) == array.end()) {
        array[sensor_name] = SensorMap_t<SpecificInfoType>();
    }
    array[sensor_name][target_sensor_name] = info;
    AD_LDEBUG(ADSCM) << "[Registering info] sensor name: " << sensor_name
                     << "; "
                     << "target sensor name: " << target_sensor_name << " "
                     << "number: " << array.size();
    return true;
}

scmStatus_t SensorConfigManager::Init(const std::string &sensor_folder) {
    std::lock_guard<std::mutex> lock(g_mutex);
    if (kIsManagerInitialized) {
        AD_LWARN(ADSCM) << "Already initialized from folder: "
                        << kSensorConfigurationFolder;
        return SCM_SUCCESS;
    }
    if (sensor_folder.empty()) {
        AD_LERROR(SENSOR_CONFIG_MANANGER)
            << "Please specify sensor id list file.";
        return SCM_INVALID_PARAM;
    }
    AD_LINFO(ADSCM) << "\033[1;32m[SENSOR]\033[0m Sensor configuration folder: "
                    << sensor_folder;
    // Sensor name, sensor info container
    auto BuildSensorConfigMap =
        [](const std::vector<std::string> &config_files,
           SensorArray_t<SensorInfoList> &array) {  // NOLINT
            if (config_files.empty()) return;
            for (const auto &config : config_files) {
                SensorInfoPtr sensor_info(new SensorInfo());
                auto ret = ConfigReader::LoadJSON(config, sensor_info.get());
                if (ret != CONF_SUCCESS) {
                    AD_LWARN(ADSCM) << "Cannot load sensor info " << config;
                } else {
                    const auto &sensor_name = sensor_info->sensor_name;
                    const auto &target_sensor_name =
                        sensor_info->target_sensor_name;
                    if (array.find(sensor_name) == array.end()) {
                        array[sensor_name] = SensorMap_t<SensorInfoList>();
                    }
                    array[sensor_name][target_sensor_name].push_back(
                        sensor_info);
                }
            }
        };
    // Try to get Sensor connection list from file
    std::string sensor_connection_config_file =
        JOIN_PATH(sensor_folder, "sensor_connection.json");
    std::vector<std::pair<std::string, std::string>> sensor_connection_list;
    if (1 == FileSystem::FileStatus(sensor_connection_config_file)) {
        auto ret = ConfigReader::LoadJSON(sensor_connection_config_file,
                                          &sensor_connection_list);
        if (CONF_SUCCESS != ret) {
            AD_LERROR(ADSCM)
                << "Cannot read sensor connection information from: "
                << sensor_connection_config_file;
            return SCM_INVALID_PARAM;
        }
        AD_LDEBUG(ADSCM) << "Specify sensor connection from file: "
                         << sensor_connection_config_file;
    }
    // List directory
    std::vector<std::string> device_ids(
        FileSystem::ListDirectory(sensor_folder, 2));
    AD_LDEBUG(ADSCM) << "\033[1;33m[SENSOR]\033[0m Number of device: "
                     << device_ids.size();
    for (const auto &device_id : device_ids) {
        AD_LDEBUG(ADSCM) << "\033[1;33m"
                         << "[SENSOR]\033[0m device: " << device_id
                         << " found.";
        std::string &&device_folder = JOIN_PATH(sensor_folder, device_id);
        auto config_files = std::move(FileSystem::GetFileList(device_folder));
        BuildSensorConfigMap(config_files, sensor_info_array_);
    }
    bool is_succeed = true;
    for (const auto &sensor_infos : sensor_info_array_) {
        const auto &sensor_name = sensor_infos.first;
        const auto &sensor_map = sensor_infos.second;
        int32_t loaded_devices = sensor_map.size();
        auto it_map = sensor_map.begin();
        while (it_map != sensor_map.end()) {
            // Restore sensor info
            const auto &target_sensor_name = it_map->first;
            for (const auto &sensor_info_ptr : it_map->second) {
                const auto &sensor_info = *sensor_info_ptr;
                AD_LDEBUG(ADSCM)
                    << "\033[1;35m"
                    << "[LOADING]\033[0m [sensor]: " << sensor_name << " "
                    << "[target sensor]: " << target_sensor_name << " "
                    << "[param]: " << GetParamName(sensor_info.param_type);
                if (sensor_info.IsExtrinsic()) {
                    // TODO(guozhichong): might not filter the connection here?
                    AD_LTRACE(ADSCM) << "Loading extrinsic params";
                    bool is_matched = sensor_connection_list.empty();
                    for (const auto &pair : sensor_connection_list) {
                        const auto &one = pair.first;
                        const auto &two = pair.second;
                        if ((one == sensor_name && two == target_sensor_name) ||
                            (one == target_sensor_name && two == sensor_name)) {
                            is_matched = true;
                            break;
                        }
                    }
                    if (is_matched) {
                        is_succeed =
                            this->RegisterInfo(sensor_name, target_sensor_name,
                                               sensor_info, extrinsic_array_) &&
                            is_succeed;
                    }
                } else if (sensor_info.IsCurrentCalibrationResult()) {
                    AD_LTRACE(ADSCM)
                        << "Loading current calibration result info params";
                    is_succeed =
                        this->RegisterInfo(sensor_name, target_sensor_name,
                                           sensor_info,
                                           current_calibration_result_array_) &&
                        is_succeed;
                } else if (sensor_info.IsDesignInfo()) {
                    AD_LTRACE(ADSCM) << "Loading design info params";
                    is_succeed = this->RegisterInfo(
                                     sensor_name, target_sensor_name,
                                     sensor_info, sensor_designinfo_array_) &&
                                 is_succeed;
                } else if (sensor_info.IsCamera()) {
                    AD_LTRACE(ADSCM) << "Loading camera params";
                    if (sensor_info.IsIntrinsic()) {
                        AD_LTRACE(ADSCM) << "Loading camera intrinsic params";
                        is_succeed =
                            this->RegisterInfo(sensor_name, target_sensor_name,
                                               sensor_info,
                                               camera_intrinsic_array_) &&
                            is_succeed;
                        auto &cam_K_new =
                            camera_intrinsic_array_[sensor_name]
                                                   [target_sensor_name]
                                                       .cam_K_new;
                        if (cam_K_new.empty() == false) {
                            AD_LDEBUG(ADSCM) << sensor_name << " using new K";
                        }
                        // if input img new size is 1024*576 for fov120, fov30
                        // and rear camera, resize to 1920*1024 and adjust
                        // intrinsic
                        if (sensor_name == "center_camera_fov30" ||
                            sensor_name == "center_camera_fov120" ||
                            sensor_name == "rear_camera") {
                            auto &img_new_w =
                                camera_intrinsic_array_[sensor_name]
                                                       [target_sensor_name]
                                                           .img_new_w;
                            auto &img_new_h =
                                camera_intrinsic_array_[sensor_name]
                                                       [target_sensor_name]
                                                           .img_new_h;
                            if (img_new_w == 1024 && img_new_h == 576) {
                                // first resize to 1920*1080
                                float scale = 1920 / 1024.0;
                                cam_K_new.at<float>(0, 0) *= scale;
                                cam_K_new.at<float>(1, 1) *= scale;
                                cam_K_new.at<float>(0, 2) *= scale;
                                cam_K_new.at<float>(1, 2) *= scale;
                                // adjust to 1920*1024(crop top 28 lines)
                                cam_K_new.at<float>(1, 2) -= 28;
                                img_new_w = 1920;
                                img_new_h = 1024;
                                AD_LINFO(ADSCM)
                                    << "resize img new size to " << img_new_w
                                    << "*" << img_new_h << " for "
                                    << sensor_name
                                    << ", fx=" << cam_K_new.at<float>(0, 0)
                                    << ", fy=" << cam_K_new.at<float>(1, 1)
                                    << ", cx=" << cam_K_new.at<float>(0, 2)
                                    << ", cy=" << cam_K_new.at<float>(1, 2);
                            } else if (img_new_w == 1920 && img_new_h == 1024) {
                                AD_LINFO(ADSCM) << "input img new size already "
                                                   "is 1920*1024 for "
                                                << sensor_name;
                            } else {
                                AD_LERROR(ADSCM)
                                    << "input img new size error! w="
                                    << img_new_w << ", h=" << img_new_h
                                    << " for " << sensor_name;
                            }
                        }
                    } else if (sensor_info.IsHardware()) {
                        AD_LTRACE(ADSCM) << "Loading Camera hardware params";
                        is_succeed = this->RegisterInfo(
                                         sensor_name, target_sensor_name,
                                         sensor_info, camera_hardware_array_) &&
                                     is_succeed;
                    } else {
                        AD_LERROR(ADSCM) << "Invalid camera params";
                    }
                } else if (sensor_info.IsCan() && sensor_info.IsIntrinsic()) {
                    AD_LTRACE(ADSCM) << "Loading can intrinsic params";
                    is_succeed = this->RestoreInfo(sensor_info, &can_calib_) &&
                                 is_succeed;
                } else if (sensor_info.IsGnss() && sensor_info.IsIntrinsic()) {
                    AD_LTRACE(ADSCM) << "Loading gnss intrinsic params";
                    is_succeed = this->RestoreInfo(sensor_info, &gnss_calib_) &&
                                 is_succeed;
                } else if (sensor_info.IsImu() && sensor_info.IsIntrinsic()) {
                    AD_LTRACE(SENSOR_CONFIG_MANAGER)
                        << "Loading imu intrinsic params";
                    is_succeed = this->RestoreInfo(sensor_info, &imu_calib_) &&
                                 is_succeed;
                } else if (sensor_info.IsOdometer() &&
                           sensor_info.IsIntrinsic()) {
                    AD_LTRACE(SENSOR_CONFIG_MANAGER)
                        << "Loading odometer intrinsic params";
                    is_succeed =
                        this->RestoreInfo(sensor_info, &odometer_calib_) &&
                        is_succeed;
                } else if (sensor_info.IsDual_ant() &&
                           sensor_info.IsIntrinsic()) {
                    AD_LTRACE(SENSOR_CONFIG_MANAGER)
                        << "Loading dual ant intrinsic params";
                    is_succeed =
                        this->RestoreInfo(sensor_info, &dual_ant_calib_) &&
                        is_succeed;
                } else if (sensor_info.IsLidar()) {
                    if (sensor_info.IsIntrinsic()) {
                        AD_LTRACE(ADSCM) << "Loading lidar intrinsic params";
                        is_succeed = this->RegisterInfo(
                                         sensor_name, target_sensor_name,
                                         sensor_info, lidar_intrinsic_array_) &&
                                     is_succeed;
                    } else if (sensor_info.IsHardware()) {
                        AD_LTRACE(ADSCM) << "Loading lidar hardware params";
                        is_succeed = this->RegisterInfo(
                                         sensor_name, target_sensor_name,
                                         sensor_info, lidar_hardware_array_) &&
                                     is_succeed;
                    }
                } else if (sensor_info.IsVehicle()) {
                    if (sensor_info.IsHardware()) {
                        AD_LTRACE(ADSCM) << "Loading vehicle config params";
                        is_succeed =
                            this->RegisterInfo(sensor_name, target_sensor_name,
                                               sensor_info,
                                               vehicle_hardware_array_) &&
                            is_succeed;
                    }
                } else {
                    AD_LDEBUG(ADSCM) << "\033[1;34m device: " << sensor_name
                                     << " param type: "
                                     << GetParamName(sensor_info.param_type)
                                     << " not used\033[0m";
                    --loaded_devices;
                }
                registered_sensor_names_.insert(sensor_name);
                registered_sensor_names_.insert(target_sensor_name);
            }
            ++it_map;
        }
        AD_LDEBUG(ADSCM) << "\033[1;32m"
                         << "[SENSOR] [DONE]\033[0m " << sensor_name
                         << " loaded : "
                         << "params: " << loaded_devices << "/"
                         << sensor_map.size();
    }
    SetupLidarRunningId();
    kIsManagerInitialized = is_succeed;
    kSensorConfigurationFolder = std::move(sensor_folder);
    AD_LINFO(ADSCM) << "\033[1;32m[SENSOR]\033[0m Sensor configuration done.";
    return (is_succeed ? SCM_SUCCESS : SCM_INVALID_PARAM);
}

scmStatus_t SensorConfigManager::GetClockLagBetweenTwoSensors(
    std::string sensor, std::string target_sensor, int64_t &time_lag) const {
    CHECK_SENSOR_EXISTS(extrinsic_array_, sensor, SCM_INVALID_PARAM);
    CHECK_SENSOR_EXISTS(extrinsic_array_.find(sensor)->second, target_sensor,
                        SCM_INVALID_PARAM);
    auto sensor_map = extrinsic_array_.find(sensor);
    if (sensor_map != extrinsic_array_.end()) {
        auto extrinsic = sensor_map->second.find(target_sensor);
        if (extrinsic == sensor_map->second.end()) {
            AD_LERROR(ADSCM) << "Cannot find param time lag to target sensor: "
                             << target_sensor;
            return SCM_INVALID_PARAM;
        } else {
            time_lag = extrinsic->second.time_lag;
        }
    } else {
        AD_LERROR(ADSCM) << "Cannot find param time lag from source sensor: "
                         << sensor;
        return SCM_INVALID_PARAM;
    }
    return SCM_SUCCESS;
}

scmStatus_t SensorConfigManager::GetExternalCalibConfig(
    std::string sensor, std::string target_sensor, cv::Mat *trans_mat) const {
    if (trans_mat == nullptr) {
        return SCM_NULL_PTR;
    }
    CHECK_SENSOR_EXISTS(extrinsic_array_, sensor, SCM_INVALID_PARAM);
    CHECK_SENSOR_EXISTS(extrinsic_array_.find(sensor)->second, target_sensor,
                        SCM_INVALID_PARAM);
    auto sensor_map = extrinsic_array_.find(sensor);
    if (sensor_map != extrinsic_array_.end()) {
        auto extrinsic = sensor_map->second.find(target_sensor);
        if (extrinsic == sensor_map->second.end()) {
            AD_LERROR(ADSCM) << "Cannot find param extrinsic to target sensor: "
                             << target_sensor;
            return SCM_INVALID_PARAM;
        } else {
            *trans_mat = extrinsic->second.sensor_calib.clone();
        }
    } else {
        AD_LERROR(ADSCM) << "Cannot find param extrinsic from source sensor: "
                         << sensor;
        return SCM_INVALID_PARAM;
    }
    return SCM_SUCCESS;
}

scmStatus_t SensorConfigManager::GetCameraInternalCalibConfig(
    std::string cam_sensor, CamInternalCalib *cam_internal_calib) const {
    if (cam_internal_calib == nullptr) {
        return SCM_NULL_PTR;
    }
    CHECK_SENSOR_EXISTS(camera_intrinsic_array_, cam_sensor, SCM_INVALID_PARAM);
    auto camera_map = camera_intrinsic_array_.find(cam_sensor);
    if (camera_map != camera_intrinsic_array_.end()) {
        auto camera_intrinsic = camera_map->second.find(cam_sensor);
        if (camera_intrinsic == camera_map->second.end()) {
            AD_LERROR(ADSCM) << "Cannot find param camera intrinsic of sensor: "
                             << cam_sensor;
            return SCM_INVALID_PARAM;
        } else {
            *cam_internal_calib = camera_intrinsic->second;
        }
    } else {
        AD_LERROR(ADSCM) << "Cannot find param camera intrinsic of sensor: "
                         << cam_sensor;
        return SCM_INVALID_PARAM;
    }
    return SCM_SUCCESS;
}

scmStatus_t SensorConfigManager::GetSensorDesignInfoConfig(
    std::string sensor_name, DesignInfo *struct_design_calib) const {
    if (struct_design_calib == nullptr) {
        return SCM_NULL_PTR;
    }
    CHECK_SENSOR_EXISTS(sensor_designinfo_array_, sensor_name,
                        SCM_INVALID_PARAM);

    auto sensor_map = sensor_designinfo_array_.find(sensor_name);
    if (sensor_map != sensor_designinfo_array_.end()) {
        auto design_info = sensor_map->second.find(sensor_name);
        if (design_info == sensor_map->second.end()) {
            AD_LERROR(ADSCM)
                << "Cannot find param design info of sensor: " << sensor_name;
            return SCM_INVALID_PARAM;
        } else {
            *struct_design_calib = design_info->second;
        }
    } else {
        AD_LERROR(ADSCM) << "Cannot find param design info of sensor: "
                         << sensor_name;
        return SCM_INVALID_PARAM;
    }
    return SCM_SUCCESS;
}

scmStatus_t SensorConfigManager::GetCurrentCalibrationResultConfig(
    std::string sensor_name,
    std::string target_sensor,
    CalibrationResult *struct_calibration_result) const {
    if (struct_calibration_result == nullptr) {
        return SCM_NULL_PTR;
    }
    CHECK_SENSOR_EXISTS(current_calibration_result_array_, sensor_name,
                        SCM_INVALID_PARAM);
    CHECK_SENSOR_EXISTS(extrinsic_array_.find(sensor_name)->second,
                        target_sensor, SCM_INVALID_PARAM);
    auto sensor_map = current_calibration_result_array_.find(sensor_name);
    if (sensor_map != current_calibration_result_array_.end()) {
        auto info = sensor_map->second.find(target_sensor);
        if (info == sensor_map->second.end()) {
            AD_LERROR(ADSCM)
                << "Cannot find param design info of sensor: " << target_sensor;
            return SCM_INVALID_PARAM;
        } else {
            *struct_calibration_result = info->second;
        }
    } else {
        AD_LERROR(ADSCM)
            << "Cannot find param current calibration result info of sensor: "
            << sensor_name;
        return SCM_INVALID_PARAM;
    }
    return SCM_SUCCESS;
}

scmStatus_t SensorConfigManager::GetLidarInternalCalibConfig(
    const std::string &sensor_name, LidarCalib *lidar_info) const {
    if (lidar_info == nullptr) {
        return SCM_NULL_PTR;
    }
    CHECK_SENSOR_EXISTS(lidar_intrinsic_array_, sensor_name, SCM_INVALID_PARAM);
    auto lidar_map = lidar_intrinsic_array_.find(sensor_name);
    if (lidar_map != lidar_intrinsic_array_.end()) {
        auto lidar_intrinsic = lidar_map->second.find(sensor_name);
        if (lidar_intrinsic == lidar_map->second.end()) {
            AD_LERROR(ADSCM) << "Cannot find param lidar intrinsic of sensor: "
                             << sensor_name;
            return SCM_INVALID_PARAM;
        } else {
            *lidar_info = lidar_intrinsic->second;
        }
    } else {
        AD_LERROR(ADSCM) << "Cannot find param camera intrinsic of sensor: "
                         << sensor_name;
        return SCM_INVALID_PARAM;
    }
    return SCM_SUCCESS;
}

void SensorConfigManager::SetupLidarRunningId() {
    std::vector<std::string> all_lidar_name;
    for (auto lidar_map : lidar_intrinsic_array_) {
        auto lidar_intrinsic = lidar_map.second.find(lidar_map.first);
        if (lidar_intrinsic == lidar_map.second.end()) {
            continue;
        }
        all_lidar_name.push_back(lidar_map.first);
    }
    std::sort(all_lidar_name.begin(), all_lidar_name.end());
    for (size_t i = 0; i < all_lidar_name.size(); i++) {
        auto lidar_name = all_lidar_name[i];
        lidar_intrinsic_array_[lidar_name][lidar_name].running_lidar_id = i;
        AD_LDEBUG(ADSCM) << "\033[1;32m"
                         << "[LidarRunningId]\033[0m lidar: " << lidar_name
                         << " id: " << i;
    }
}

scmStatus_t SensorConfigManager::SetCameraInternalCalib(
    const std::string &cam_sensor, const CamInternalCalib &cam_internal_calib) {
    auto &array = camera_intrinsic_array_;
    if (array.find(cam_sensor) == array.end()) {
        array[cam_sensor] = SensorMap_t<CamInternalCalib>();
    }
    array[cam_sensor][cam_sensor] = cam_internal_calib;
    AD_LDEBUG(ADSCM) << "\033[1;36m"
                     << "New added camera intrinsic param: " << cam_sensor
                     << "\033[0m";
    AD_LDEBUG(ADSCM) << "New camera [" << cam_sensor << "]"
                     << " "
                     << "K: " << cam_internal_calib.cam_K << " "
                     << "Dist: " << cam_internal_calib.cam_dist;
    return SCM_SUCCESS;
}

scmStatus_t SensorConfigManager::GetCanCalibConfig(CanCalib *can_calib) const {
    if (can_calib == nullptr) {
        return SCM_NULL_PTR;
    }
    *can_calib = can_calib_;
    return SCM_SUCCESS;
}

scmStatus_t SensorConfigManager::GetGnssCalibConfig(
    GnssCalib *gnss_calib) const {
    if (gnss_calib == nullptr) {
        return SCM_NULL_PTR;
    }
    *gnss_calib = gnss_calib_;
    return SCM_SUCCESS;
}

scmStatus_t SensorConfigManager::GetIMUCalibConfig(IMUCalib *imu_calib) const {
    if (imu_calib == nullptr) {
        return SCM_NULL_PTR;
    }
    *imu_calib = imu_calib_;
    return SCM_SUCCESS;
}

scmStatus_t SensorConfigManager::GetODOMETERCalibConfig(
    OdometerCalib *odometer_calib) const {
    if (odometer_calib == nullptr) {
        return SCM_NULL_PTR;
    }
    *odometer_calib = odometer_calib_;
    return SCM_SUCCESS;
}

scmStatus_t SensorConfigManager::GetDUALANTCalibConfig(
    DualantCalib *dual_ant_calib) const {
    if (dual_ant_calib == nullptr) {
        return SCM_NULL_PTR;
    }
    *dual_ant_calib = dual_ant_calib_;
    return SCM_SUCCESS;
}

scmStatus_t SensorConfigManager::GetAllSensorName(
    std::vector<std::string> *sensor_list) const {
    if (sensor_list == nullptr) {
        return SCM_NULL_PTR;
    }
    sensor_list->resize(registered_sensor_names_.size());
    std::copy(registered_sensor_names_.begin(), registered_sensor_names_.end(),
              sensor_list->begin());
    return SCM_SUCCESS;
}

scmStatus_t SensorConfigManager::GetHomographyConfig(
    std::string cam_sensor, CamHomography *cam_homography) const {
    AD_LERROR(ADSCM)
        << "This API had been deleted, use GetHMatrixConfig instead";
    return SCM_INVALID_PARAM;
}

scmStatus_t SensorConfigManager::GetHMatrixConfig(std::string cam_sensor,
                                                  HMatrix *cam_h_matrix) const {
    if (cam_h_matrix == nullptr) {
        return SCM_NULL_PTR;
    }

    CamInternalCalib cam_intrinsic;
    if (GetCameraInternalCalibConfig(cam_sensor, &cam_intrinsic) !=
        SCM_SUCCESS) {
        AD_LERROR(ADSCM) << "Get Homography matrix failed: " << cam_sensor;
        return SCM_INVALID_PARAM;
    }

    cv::Mat Tvc;
    if (GetExternalCalibConfig(cam_sensor, "car_center", &Tvc) != SCM_SUCCESS) {
        AD_LERROR(ADSCM) << "Get Homography matrix failed: " << cam_sensor;
        return SCM_INVALID_PARAM;
    }

    cam_intrinsic.cam_K_new.convertTo(cam_intrinsic.cam_K_new, CV_64F);
    Tvc.convertTo(Tvc, CV_64F);
    cv::Mat Rvc = Tvc.rowRange(0, 3).colRange(0, 3).clone();
    cv::Mat tvc = Tvc.rowRange(0, 3).colRange(3, 4).clone();

    cv::Mat n = cv::Mat::zeros(3, 1, CV_64F);
    // downward ground plane normal
    n.at<float64_t>(0, 0) = 0;
    n.at<float64_t>(1, 0) = 0;
    n.at<float64_t>(2, 0) = -1;
    n = Rvc.t() * n;

    cv::Mat H;
    // moving to the 1m height virtual ground over head actual ground
    cv::Mat shift = cv::Mat::zeros(3, 1, CV_64F);
    shift.at<float64_t>(2, 0) = 1.0;
    H = Rvc + (tvc + shift) * n.t() / tvc.at<float64_t>(2, 0);
    cv::Mat K_inv = cam_intrinsic.cam_K_new.inv();
    K_inv.convertTo(K_inv, CV_64F);
    H = H * K_inv;
    cam_h_matrix->h_matrix = H.clone();

    return SCM_SUCCESS;
}

scmStatus_t SensorConfigManager::GetRelationshipPair(
    std::vector<std::pair<std::string, std::string>> *relationship_list) const {
    if (relationship_list == nullptr) {
        return SCM_NULL_PTR;
    }
    relationship_list->clear();
    for (const auto &sensor_map : extrinsic_array_) {
        const auto &sensor_name = sensor_map.first;
        if (!sensor_map.second.empty()) {
            for (const auto &pair : sensor_map.second) {
                const auto &target_name = pair.first;
                if (sensor_name != target_name) {
                    AD_LDEBUG(ADSCM) << "Relationship list: "
                                     << "source: " << sensor_name << " "
                                     << "target: " << target_name;
                    relationship_list->emplace_back(sensor_name, target_name);
                }
            }
        }
    }
    return SCM_SUCCESS;
}
scmStatus_t SensorConfigManager::GetAllCameraHardwareInfo(
    SensorMap_t<HardwareInfo> *hardware_infos) const {
    if (hardware_infos == nullptr) {
        return SCM_NULL_PTR;
    }
    hardware_infos->clear();
    auto it_array = camera_hardware_array_.begin();
    while (it_array != camera_hardware_array_.end()) {
        const auto &sensor_name = it_array->first;
        auto it_map = it_array->second.begin();
        while (it_map != it_array->second.end()) {
            const auto &target_sensor_name = it_map->first;
            if (sensor_name != target_sensor_name) {
                AD_LERROR(ADSCM) << "Invalid hardware info. " << sensor_name
                                 << " " << target_sensor_name;
                continue;
            }
            const auto &hardware_info = it_map->second;
            (*hardware_infos)[sensor_name] = hardware_info;
            ++it_map;
        }
        ++it_array;
    }
    return SCM_SUCCESS;
}

scmStatus_t SensorConfigManager::GetAllLidarInternalCalibConfig(
    SensorMap_t<LidarCalib> *lidar_infos) const {
    if (lidar_infos == nullptr) {
        return SCM_NULL_PTR;
    }
    lidar_infos->clear();
    auto it_array = lidar_intrinsic_array_.begin();
    while (it_array != lidar_intrinsic_array_.end()) {
        const auto &sensor_name = it_array->first;
        auto it_map = it_array->second.begin();
        while (it_map != it_array->second.end()) {
            const auto &target_sensor_name = it_map->first;
            if (sensor_name != target_sensor_name) {
                AD_LERROR(ADSCM) << "Invalid lidar internal calibration info. "
                                 << sensor_name << " " << target_sensor_name;
                continue;
            }
            const auto &lidar_info = it_map->second;
            (*lidar_infos)[sensor_name] = lidar_info;
            ++it_map;
        }
        ++it_array;
    }
    return SCM_SUCCESS;
}

scmStatus_t SensorConfigManager::GetCameraHardwareInfo(
    const std::string &sensor_name, HardwareInfo *hardware_info) const {
    if (hardware_info == nullptr) {
        return SCM_NULL_PTR;
    }

    CHECK_SENSOR_EXISTS(camera_hardware_array_, sensor_name, SCM_INVALID_PARAM);
    auto hardware_map = camera_hardware_array_.find(sensor_name);
    if (hardware_map != camera_hardware_array_.end()) {
        auto hardware = hardware_map->second.find(sensor_name);
        if (hardware == hardware_map->second.end()) {
            AD_LERROR(ADSCM)
                << "Cannot find param camera hardware information of sensor: "
                << sensor_name;
            return SCM_INVALID_PARAM;
        } else {
            *hardware_info = hardware->second;
        }
    } else {
        AD_LERROR(ADSCM)
            << "Cannot find param camera hardware information of sensor: "
            << sensor_name;
        return SCM_INVALID_PARAM;
    }
    return SCM_SUCCESS;
}

scmStatus_t SensorConfigManager::GetLidarHardwareInfo(
    const std::string &sensor_name, LidarHardwareConfig *hardware_info) const {
    if (hardware_info == nullptr) {
        return SCM_NULL_PTR;
    }
    CHECK_SENSOR_EXISTS(lidar_hardware_array_, sensor_name, SCM_INVALID_PARAM);
    auto hardware_map = lidar_hardware_array_.find(sensor_name);
    if (hardware_map != lidar_hardware_array_.end()) {
        auto hardware = hardware_map->second.find(sensor_name);
        if (hardware == hardware_map->second.end()) {
            AD_LERROR(ADSCM)
                << "Cannot find param lidar hardware information of sensor: "
                << sensor_name;
            return SCM_INVALID_PARAM;
        } else {
            *hardware_info =
                (*(hardware->second)
                      .property.GetValue<senseAD::LidarHardwareConfig>());
        }
    } else {
        AD_LERROR(ADSCM)
            << "Cannot find param camera hardware information of sensor: "
            << sensor_name;
        return SCM_INVALID_PARAM;
    }
    return SCM_SUCCESS;
}

scmStatus_t SensorConfigManager::GetVehicleConfigInfo(
    const std::string &sensor_name, VehicleConfig *hardware_info) const {
    if (hardware_info == nullptr) {
        return SCM_NULL_PTR;
    }
    CHECK_SENSOR_EXISTS(vehicle_hardware_array_, sensor_name,
                        SCM_INVALID_PARAM);
    auto hardware_map = vehicle_hardware_array_.find(sensor_name);
    if (hardware_map != vehicle_hardware_array_.end()) {
        auto hardware = hardware_map->second.find(sensor_name);
        if (hardware == hardware_map->second.end()) {
            AD_LERROR(ADSCM)
                << "Cannot find param vehicle hardware information of sensor: "
                << sensor_name;
            return SCM_INVALID_PARAM;
        } else {
            *hardware_info =
                (*(hardware->second)
                      .property.GetValue<senseAD::VehicleConfig>());
        }
    } else {
        AD_LERROR(ADSCM)
            << "Cannot find param camera hardware information of sensor: "
            << sensor_name;
        return SCM_INVALID_PARAM;
    }
    return SCM_SUCCESS;
}

scmStatus_t SensorConfigManager::GetSensorInfoList(
    const std::string &source,
    const std::string &target,
    SensorInfoList *sensor_info_list) const {
    if (sensor_info_list == nullptr) {
        return SCM_NULL_PTR;
    }
    CHECK_SENSOR_EXISTS(sensor_info_array_, source, SCM_INVALID_PARAM);
    auto sensor_map = sensor_info_array_.find(source);
    if (sensor_map != sensor_info_array_.end()) {
        auto sensor = sensor_map->second.find(target);
        if (sensor == sensor_map->second.end()) {
            AD_LERROR(ADSCM) << "Cannot find information of sensor: " << target;
            return SCM_INVALID_PARAM;
        } else {
            *sensor_info_list = sensor->second;
        }
    } else {
        AD_LERROR(ADSCM) << "Cannot find information of sensor: " << source;
        return SCM_INVALID_PARAM;
    }
    return SCM_SUCCESS;
}

}  // namespace sensorconfig
}  // namespace senseAD
