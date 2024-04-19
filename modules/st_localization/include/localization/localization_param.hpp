/*
 * Copyright (C) 2019-2020 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 */
#pragma once

#include <string>
#include <vector>

#include "ad_common/config_utils/macros.hpp"

namespace senseAD {
namespace localization {

struct LocalizationCommonParam {
  bool absolute_localization;
  bool relative_localization;
  bool sub_ins_data;
  std::string ins_device;
  std::string front_locator_type;
  std::string back_locator_type;
  bool enable_switch_origin;
  bool enable_loc_restart;
  bool enable_dr_restart;
  size_t window_size;
  std::string sensor_config_file;
  std::string execute_mode;
  std::string hdmap_file;
  bool using_map_origin;
  bool use_ehr_map;
  bool using_l4_metric_status_output;
  bool using_dual_antenna;
};

struct LocalizationCANParam {
  double can_minimum_speed;
  bool use_offline_calib;
};

struct LocalizationDRParam {
  std::string dead_reckoning_locator_type;
  int dr_velocity_update_freq;
};

struct LocalizationGNSSParam {
  bool enable_fault_diagnosis;
};

struct LocalizationSMMParam {
  std::vector<std::string> enable_camera_names;
  bool enable_percept_semantic;
  bool use_route_map;
  double map_laneline_sample_dist;
  bool enable_temporal_fusion;
  bool track_by_percept_tracker_id;
  bool enable_calib_homography;
  bool enable_strict_reloc_check;
  bool use_reloc_new_version;
};

struct LocalizationMSFParam {
  bool enable_multi_filters;
  bool enable_gnss_bias_estimate;
  bool enable_msf_serial;
  std::string msf_serial_save_path;
  // can noise parameters
  double can_sigma_vel;
  double can_sigma_omega;
  double can_sigma_vel_scale;
  double can_sigma_omega_bias;
  // imu nose parameters
  double imu_sigma_acc;
  double imu_sigma_gyro;
  double imu_sigma_acc_bias;
  double imu_sigma_gyro_bias;
  double imu_sigma_acc_scale;
  double imu_sigma_gyro_scale;
  double dq_imu_sigma_acc;
  double dq_imu_sigma_gyro;
  double dq_imu_sigma_acc_bias;
  double dq_imu_sigma_gyro_bias;
  double dq_imu_sigma_acc_scale;
  double dq_imu_sigma_gyro_scale;
  double imu_static_acc_var_thre;
  // fins noise parameters
  double fins_sigma_pos;
  double fins_sigma_rot;
  // gnss noise parameters
  double gnss_sigma_pos;
  double gnss_sigma_vel;
  // map matching noise parameter
  double map_matching_sigma_pos;
  double map_matching_sigma_rot;
  // for msf param
  double balance_factor_chi2_thre;
};

struct LocalizationCIParam {
  bool enable_evaluation;
  std::string results_save_dir;
  std::string setting_type;
  std::string testcase_id;
};

struct LocalizationVisualParam {
  bool enable_display;
  bool enable_video_record;
};

struct LocalizationParam {
  LocalizationCommonParam common_param;
  LocalizationDRParam dr_param;
  LocalizationCANParam can_param;
  LocalizationGNSSParam gnss_param;
  LocalizationSMMParam smm_param;
  LocalizationMSFParam msf_param;
  LocalizationCIParam ci_param;
  LocalizationVisualParam visual_param;
};

REGISTER_CEREAL_SERIALIZE(LocalizationCommonParam& common_param) {  // NOLINT
  CEREAL_PAIR(common_param, absolute_localization);
  CEREAL_PAIR(common_param, relative_localization);
  CEREAL_PAIR(common_param, sub_ins_data);
  CEREAL_PAIR(common_param, ins_device);
  CEREAL_PAIR(common_param, front_locator_type);
  CEREAL_PAIR(common_param, back_locator_type);
  CEREAL_PAIR(common_param, enable_switch_origin);
  CEREAL_PAIR(common_param, enable_loc_restart);
  CEREAL_PAIR(common_param, enable_dr_restart);
  CEREAL_PAIR(common_param, window_size);
  CEREAL_PAIR(common_param, sensor_config_file);
  CEREAL_PAIR(common_param, execute_mode);
  CEREAL_PAIR(common_param, hdmap_file);
  CEREAL_PAIR(common_param, using_map_origin);
  CEREAL_PAIR(common_param, use_ehr_map);
  CEREAL_PAIR(common_param, using_l4_metric_status_output);
  CEREAL_PAIR(common_param, using_dual_antenna);
}

REGISTER_CEREAL_SERIALIZE(LocalizationDRParam& dr_param) {  // NOLINT
  CEREAL_PAIR(dr_param, dead_reckoning_locator_type);
  CEREAL_PAIR(dr_param, dr_velocity_update_freq);
}

REGISTER_CEREAL_SERIALIZE(LocalizationCANParam& can_param) {  // NOLINT
  CEREAL_PAIR(can_param, can_minimum_speed);
  CEREAL_PAIR(can_param, use_offline_calib);
}

REGISTER_CEREAL_SERIALIZE(LocalizationGNSSParam& gnss_param) {  // NOLINT
  CEREAL_PAIR(gnss_param, enable_fault_diagnosis);
}

REGISTER_CEREAL_SERIALIZE(LocalizationSMMParam& smm_param) {  // NOLINT
  CEREAL_PAIR(smm_param, enable_camera_names);
  CEREAL_PAIR(smm_param, enable_percept_semantic);
  CEREAL_PAIR(smm_param, use_route_map);
  CEREAL_PAIR(smm_param, map_laneline_sample_dist);
  CEREAL_PAIR(smm_param, enable_temporal_fusion);
  CEREAL_PAIR(smm_param, track_by_percept_tracker_id);
  CEREAL_PAIR(smm_param, enable_calib_homography);
  CEREAL_PAIR(smm_param, enable_strict_reloc_check);
  CEREAL_PAIR(smm_param, use_reloc_new_version);
}

REGISTER_CEREAL_SERIALIZE(LocalizationMSFParam& msf_param) {  // NOLINT
  CEREAL_PAIR(msf_param, enable_multi_filters);
  CEREAL_PAIR(msf_param, enable_gnss_bias_estimate);
  CEREAL_PAIR(msf_param, enable_msf_serial);
  CEREAL_PAIR(msf_param, msf_serial_save_path);
  CEREAL_PAIR(msf_param, can_sigma_vel);
  CEREAL_PAIR(msf_param, can_sigma_omega);
  CEREAL_PAIR(msf_param, can_sigma_vel_scale);
  CEREAL_PAIR(msf_param, can_sigma_omega_bias);
  CEREAL_PAIR(msf_param, imu_sigma_acc);
  CEREAL_PAIR(msf_param, imu_sigma_gyro);
  CEREAL_PAIR(msf_param, imu_sigma_acc_bias);
  CEREAL_PAIR(msf_param, imu_sigma_gyro_bias);
  CEREAL_PAIR(msf_param, imu_sigma_acc_scale);
  CEREAL_PAIR(msf_param, imu_sigma_gyro_scale);
  CEREAL_PAIR(msf_param, dq_imu_sigma_acc);
  CEREAL_PAIR(msf_param, dq_imu_sigma_gyro);
  CEREAL_PAIR(msf_param, dq_imu_sigma_acc_bias);
  CEREAL_PAIR(msf_param, dq_imu_sigma_gyro_bias);
  CEREAL_PAIR(msf_param, dq_imu_sigma_acc_scale);
  CEREAL_PAIR(msf_param, dq_imu_sigma_gyro_scale);
  CEREAL_PAIR(msf_param, imu_static_acc_var_thre);
  CEREAL_PAIR(msf_param, fins_sigma_pos);
  CEREAL_PAIR(msf_param, fins_sigma_rot);
  CEREAL_PAIR(msf_param, gnss_sigma_pos);
  CEREAL_PAIR(msf_param, gnss_sigma_vel);
  CEREAL_PAIR(msf_param, map_matching_sigma_pos);
  CEREAL_PAIR(msf_param, map_matching_sigma_rot);
  CEREAL_PAIR(msf_param, balance_factor_chi2_thre);
}

REGISTER_CEREAL_SERIALIZE(LocalizationCIParam& ci_param) {  // NOLINT
  CEREAL_PAIR(ci_param, enable_evaluation);
  CEREAL_PAIR(ci_param, results_save_dir);
  CEREAL_PAIR(ci_param, setting_type);
  CEREAL_PAIR(ci_param, testcase_id);
}

REGISTER_CEREAL_SERIALIZE(LocalizationVisualParam& visual_param) {  // NOLINT
  CEREAL_PAIR(visual_param, enable_display);
  CEREAL_PAIR(visual_param, enable_video_record);
}

REGISTER_CEREAL_SERIALIZE(LocalizationParam& param) {  // NOLINT
  CEREAL_PAIR(param, common_param);
  CEREAL_PAIR(param, dr_param);
  CEREAL_PAIR(param, can_param);
  CEREAL_PAIR(param, gnss_param);
  CEREAL_PAIR(param, smm_param);
  CEREAL_PAIR(param, msf_param);
  CEREAL_PAIR(param, ci_param);
  CEREAL_PAIR(param, visual_param);
}

}  // namespace localization
}  // namespace senseAD
