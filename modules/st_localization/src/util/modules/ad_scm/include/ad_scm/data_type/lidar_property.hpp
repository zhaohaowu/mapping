/*
 * Copyright (C) 2017 by SenseTime Group Limited. All rights reserved.
 * Yue Dayu <yuedayu@sensetime.com>
 */

#pragma once

#include <string>
#include <vector>
#include <map>

#include "ad_common/data_type/any.hpp"
#include "ad_common/data_type/base.hpp"

namespace senseAD {

struct LaserCorrection {
    float rot_correction;
    float vert_correction;
    float dist_correction;
    bool two_pt_correction_available;
    float dist_correction_x;
    float dist_correction_y;
    float vert_offset_correction;
    float horiz_offset_correction;
    int max_intensity;
    int min_intensity;
    float focal_distance;
    float focal_slope;

    /** cached values calculated when the calibration file is read */
    float cos_rot_correction;   ///< cosine of rot_correction
    float sin_rot_correction;   ///< sine of rot_correction
    float cos_vert_correction;  ///< cosine of vert_correction
    float sin_vert_correction;  ///< sine of vert_correction

    int laser_ring;  ///< ring number for this laser
};

struct RsLidarAdditionalConfig {
    std::vector<std::vector<float32_t>> intensity_cal;
    std::vector<std::vector<int32_t>> channel_num;
    std::vector<float32_t> curves_rate;
    std::vector<int32_t> vert_angle;
    std::vector<int32_t> hori_angle;
};

struct LidarHardwareConfig {
    std::string lidar_type;
    float distance_resolution_m;
    std::map<int, LaserCorrection> laser_corrections;
    int num_lasers;

    Any additional_config;
};

}  // namespace senseAD
