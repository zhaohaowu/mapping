/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: shenliangchen
 *Date: 2023-09-25
 *****************************************************************************/
#include "modules/local_mapping/utils/compute_loss.h"

namespace hozon {
namespace mp {
namespace lm {

float Loss::MaxDistance(cv::flann::Index* kdtree, float x, float y) {
  int query_num = 1;
  std::vector<int> vec_index(query_num);
  std::vector<float> vec_dist(query_num);
  cv::flann::SearchParams params(32);

  std::vector<float> vec_query = {x, y};

  kdtree->knnSearch(vec_query, vec_index, vec_dist, query_num, params);
  return std::sqrt(vec_dist[0]);
}

float Loss::Process(const std::vector<Eigen::Vector3d>& hq_pts,
                    const std::vector<LaneLine>& lane_lines,
                    float dist_threshold) {
  std::vector<cv::Point2f> pts;
  for (const auto& pt : hq_pts) {
    cv::Point2f tmp = {static_cast<float>(pt.x()), static_cast<float>(pt.y())};
    pts.emplace_back(tmp);
  }
  cv::Mat source = cv::Mat(pts).reshape(1);
  source.convertTo(source, CV_32F);
  cv::flann::KDTreeIndexParams index_params(2);
  cv::flann::Index kdtree(source, index_params);

  float valid_num = 0;
  float sum_distance = 0;
  std::vector<Eigen::Vector3d> map_points;
  for (const auto& lane_line : lane_lines) {
    for (const auto& param : lane_line.lane_param_) {
      double start_x = param.start_point_x_;
      for (size_t k = 0; k < param.sample_x_.size(); k++) {
        double x = param.sample_x_[k];
        double y = param.c0_ + param.c1_ * (x - start_x) +
                   param.c2_ * pow(x - start_x, 2) +
                   param.c3_ * pow(x - start_x, 3);
        float max_dist =
            MaxDistance(&kdtree, static_cast<float>(x), static_cast<float>(y));
        // HLOG_ERROR << "loss max_dist " << max_dist;
        if (max_dist < dist_threshold) {
          valid_num++;
          sum_distance += max_dist;
        }
      }
    }
  }
  HLOG_ERROR << "valid_num " << valid_num;
  HLOG_ERROR << "distance " << sum_distance / valid_num;
  if (valid_num == 0) {
    HLOG_ERROR << "valid_num = 0";
    return 100000000.0;
  }
  return sum_distance / valid_num;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
