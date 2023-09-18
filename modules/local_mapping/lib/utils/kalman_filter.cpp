/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: luerwei
 *Date: 2023-09-08
 *****************************************************************************/
#include "modules/local_mapping/lib/utils/kalman_filter.h"

namespace hozon {
namespace mp {
namespace lm {

KFFilter::KFFilter(const Eigen::Vector2d& pt) : kf_(stateNum_, measureNum_, 0) {
  cv::Mat measurement = cv::Mat::zeros(measureNum_, 1, CV_32F);
  kf_.transitionMatrix = (cv::Mat_<float>(stateNum_, stateNum_) << 1, 0, 1, 0,
                          0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);  // F

  cv::setIdentity(kf_.measurementMatrix);                           // H
  cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(1e-5));      // Q
  cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(1e-1));  // R
  cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(1));            // P

  // randn(kf_.statePost, Scalar::all(0), Scalar::all(0.1));  //x
  kf_.statePost.at<float>(0) = pt(0);
  kf_.statePost.at<float>(1) = pt(1);
  kf_.statePost.at<float>(2) = 0.001;
  kf_.statePost.at<float>(3) = 0.001;
}

Eigen::Vector2d KFFilter::Predict() {
  cv::Mat prediction = kf_.predict();
  return Eigen::Vector2d(prediction.at<float>(0), prediction.at<float>(1));
}

void KFFilter::Update(const Eigen::Vector2d& pt) {
  cv::Mat measurement = cv::Mat::zeros(measureNum_, 1, CV_32F);
  measurement.at<float>(0) = pt(0);
  measurement.at<float>(1) = pt(1);
  kf_.correct(measurement);
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
