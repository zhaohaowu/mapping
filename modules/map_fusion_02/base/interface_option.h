/***************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： interface_option.h
 *   author     ： hozon
 *   date       ： 2024.01
 ******************************************************************************/

#pragma once

#include <Eigen/Eigen>
#include <string>
namespace hozon {
namespace mp {
namespace mf {

struct BaseProcessOption {
  double timestamp = 0.0;
};

// process 函数传递参数
// 各个pipeline可以定义自己的结构
struct ProcessOption : public BaseProcessOption {
  Eigen::Affine3d T_cur_last = Eigen::Affine3d::Identity();
};

struct LaneFusionProcessOption : public BaseProcessOption {
  float path_predict_range = 80.0;
  float path_back_range = 80.0;
  float path_interval = 1.0;
  int state_detect_range = 10;
  // 对上游车道线进行插值的点间距，-1时不插值
  float lane_line_interp_dist = 0.0;
  // 用于分割GroupSegment的分割线的1/2长度
  float half_slice_length = 0.0;
  // 从线聚合到lane的宽度阈值
  float min_lane_width = 0.0;
  float max_lane_width = 0.0;
  /// 以下用于填充proto里的lane和road限速:
  float lane_speed_limit_kmph = 0.0;
  float road_min_max_speed_kmph = 0.0;
  float road_max_max_speed_kmph = 0.0;

  // 路口判断参数
  float near_junction_dis_thresh = 20.0;

  // 使用OCC构建道路边沿
  bool use_occ = false;
  // 路口内预瞄最大距离，这个未加防抖机制
  float junction_predict_distance = 40.0;
  // 远端车道线预测距离，小于等于robust_percep_dist时不预测
  float predict_farthest_dist = 0.0;
  // 车道线末端点间距阈值
  float min_predict_interval = 0.0;
  // 车道线末端heading阈值
  float max_heading_rad = 0.0;  // in rad
  // 可信赖的感知区间距离
  float robust_percep_dist = 0.0;
};

struct MapServiceOption : public BaseProcessOption {
  int map_service_mode = 0;
  std::string map_dir;
  std::string work_mode;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
