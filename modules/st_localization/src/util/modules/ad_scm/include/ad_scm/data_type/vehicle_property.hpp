/*
 * Copyright (C) 2019-2020 by SenseTime Group Limited. All rights reserved.
 * GUO Zhichong <guozhichong@sensetime.com>
 */

#pragma once

#include <opencv2/core/core.hpp>

#include "ad_common/data_type/base.hpp"

namespace senseAD {

struct VehicleConfig {
    float32_t car_center_height;
    float32_t wheel_track;
    float32_t wheel_base;
    cv::Point2f front_right_wheel;
    cv::Point2f front_left_wheel;
    cv::Point2f back_right_wheel;
    cv::Point2f back_left_wheel;
    float32_t car_width;
    float32_t car_length;
    float32_t car_height;
    float32_t front_axis_to_front_bumper;
    float32_t rear_axis_to_rear_bumper;
};

}  // namespace senseAD
