/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming<zhaoming@senseauto.com>
 */

#include "eval/evaluator_smm.hpp"

namespace senseAD {
namespace localization {

adLocStatus_t EvaluatorSMM::WriteResult(double timestamp,
                                        const SMMEvalData& data) {
  // write timestamp, input pose, refined pose and variance
  constexpr size_t kPoseDataSize = 1 + 7 + 7 + 2;
  double pose_arr[kPoseDataSize];
  pose_arr[0] = timestamp;
  Eigen::Quaterniond q(data.input_pose.block<3, 3>(0, 0));
  q.normalize();
  Eigen::Vector3d t = data.input_pose.block<3, 1>(0, 3);
  for (int i = 0; i < 3; ++i) {
    pose_arr[i + 1] = t[i];
  }
  pose_arr[4] = q.x();
  pose_arr[5] = q.y();
  pose_arr[6] = q.z();
  pose_arr[7] = q.w();

  q = Eigen::Quaterniond(data.refined_pose.block<3, 3>(0, 0));
  q.normalize();
  t = data.refined_pose.block<3, 1>(0, 3);
  for (int i = 0; i < 3; ++i) {
    pose_arr[i + 8] = t[i];
  }
  pose_arr[11] = q.x();
  pose_arr[12] = q.y();
  pose_arr[13] = q.z();
  pose_arr[14] = q.w();

  pose_arr[15] = data.pose_cov(1, 1);  // lateral variance
  pose_arr[16] = data.pose_cov(5, 5);  // yaw variance

  output_stream_ << std::setprecision(6) << std::fixed;
  output_stream_.write(reinterpret_cast<char*>(pose_arr),
                       sizeof(double) * kPoseDataSize);

  // write smm status
  constexpr size_t kStatusDataSize = 3;
  bool smm_status[kStatusDataSize];
  smm_status[0] = data.is_perception_valid;
  smm_status[1] = data.is_map_valid;
  smm_status[2] = data.is_smm_success;
  output_stream_.write(reinterpret_cast<char*>(smm_status),
                       sizeof(bool) * kStatusDataSize);

  // write smm process time
  output_stream_.write(reinterpret_cast<const char*>(&data.process_time_ms),
                       sizeof(double));

  // write data statistics of perception
  for (const auto& camera_name : camera_names_) {
    char laneline_num = 0;
    char curb_num = 0;
    if (data.roadline_statistics.count(camera_name)) {
      const auto& roadline_num = data.roadline_statistics.at(camera_name);
      laneline_num = static_cast<char>(roadline_num.first);
      curb_num = static_cast<char>(roadline_num.second);
    }
    output_stream_.write(&laneline_num, 1);
    output_stream_.write(&curb_num, 1);
  }

  // write data statistics of map
  char laneline_num = 0;
  char roadside_num = 0;
  if (data.roadline_statistics.count("map")) {
    const auto& roadline_num = data.roadline_statistics.at("map");
    laneline_num = static_cast<char>(roadline_num.first);
    roadside_num = static_cast<char>(roadline_num.second);
  }
  output_stream_.write(&laneline_num, 1);
  output_stream_.write(&roadside_num, 1);
  output_stream_.flush();

  return LOC_SUCCESS;
}

adLocStatus_t EvaluatorSMM::WriteHeader(
    const std::vector<std::string>& enable_camera_names) {
  camera_names_ = enable_camera_names;

  // write header for timestamp, input pose, refined pose and variance
  char data_num = 1 + 7 + 7 + 2;
  char bytes_each_data = static_cast<char>(sizeof(double));
  output_stream_.write(&data_num, 1);
  output_stream_.write(&bytes_each_data, 1);

  // write header for smm status
  data_num = 3;
  bytes_each_data = static_cast<char>(sizeof(bool));
  output_stream_.write(&data_num, 1);
  output_stream_.write(&bytes_each_data, 1);

  // write header for smm process time
  data_num = 1;
  bytes_each_data = static_cast<char>(sizeof(double));
  output_stream_.write(&data_num, 1);
  output_stream_.write(&bytes_each_data, 1);

  // write header for data statistics of perception
  char camera_num = static_cast<char>(enable_camera_names.size());
  output_stream_.write(&camera_num, 1);
  for (const auto& camera_name : enable_camera_names) {
    char camera_name_length = static_cast<char>(camera_name.size());
    output_stream_.write(&camera_name_length, 1);
    output_stream_.write(camera_name.data(), camera_name.size());
  }
  data_num = 2;
  bytes_each_data = static_cast<char>(sizeof(char));
  output_stream_.write(&data_num, 1);
  output_stream_.write(&bytes_each_data, 1);

  // write header for data statistics of map
  data_num = 2;
  bytes_each_data = static_cast<char>(sizeof(char));
  output_stream_.write(&data_num, 1);
  output_stream_.write(&bytes_each_data, 1);
  output_stream_.flush();

  return LOC_SUCCESS;
}

}  // namespace localization
}  // namespace senseAD
