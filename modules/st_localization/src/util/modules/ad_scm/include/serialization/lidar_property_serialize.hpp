/*
 * Copyright (C) 2019~2020 by SenseTime Group Limited. All rights reserved.
 * Yue Dayu <yuedayu@sensetime.com>
 */
#pragma once

#include "cereal/archives/binary.hpp"
#include "cereal/archives/json.hpp"
#include "cereal/archives/xml.hpp"
#include "cereal/archives/portable_binary.hpp"
#include "cereal/types/vector.hpp"
// private user components header
#include "ad_scm/data_type/lidar_property.hpp"
#include "serialization/base_serialize.hpp"
// public user components header
#include "ad_common/config_utils/macros.hpp"

namespace senseAD {

REGISTER_CEREAL_SERIALIZE(LaserCorrection &result) {  // NOLINT
    CEREAL_PAIR(result, rot_correction);
    CEREAL_PAIR(result, vert_correction);
    CEREAL_PAIR(result, laser_ring);
    CEREAL_OPTIONAL_PAIR(result, dist_correction, 0.0);
    CEREAL_OPTIONAL_PAIR(result, dist_correction_x, 0.0);
    CEREAL_OPTIONAL_PAIR(result, dist_correction_y, 0.0);
    CEREAL_OPTIONAL_PAIR(result, vert_offset_correction, 0.0);
    CEREAL_OPTIONAL_PAIR(result, focal_distance, 0.0);
    CEREAL_OPTIONAL_PAIR(result, focal_slope, 0.0);
    CEREAL_OPTIONAL_PAIR(result, two_pt_correction_available, false);
    CEREAL_OPTIONAL_PAIR(result, horiz_offset_correction, 0.0);
    CEREAL_OPTIONAL_PAIR(result, max_intensity, 255);
    CEREAL_OPTIONAL_PAIR(result, min_intensity, 0);
    result.cos_rot_correction = cosf(result.rot_correction);
    result.sin_rot_correction = sinf(result.rot_correction);
    result.cos_vert_correction = cosf(result.vert_correction);
    result.sin_vert_correction = sinf(result.vert_correction);
}

REGISTER_CEREAL_SERIALIZE(RsLidarAdditionalConfig &result) {  // NOLINT
    CEREAL_PAIR(result, intensity_cal);
    CEREAL_PAIR(result, channel_num);
    CEREAL_PAIR(result, curves_rate);
    CEREAL_PAIR(result, vert_angle);
    CEREAL_PAIR(result, hori_angle);
}

template <class Archive>
void save(Archive &archive, const LidarHardwareConfig &result) {  // NOLINT
    CEREAL_PAIR(result, lidar_type);
    CEREAL_PAIR(result, distance_resolution_m);
    CEREAL_PAIR(result, laser_corrections);
    CEREAL_PAIR(result, num_lasers);

    if (result.lidar_type == "RS32") {
        const auto &param =
            result.additional_config.GetValue<RsLidarAdditionalConfig>();
        archive(cereal::make_nvp("additional_config", *param));
    }
}

template <class Archive>
void load(Archive &archive, LidarHardwareConfig &result) {  // NOLINT
    CEREAL_PAIR(result, lidar_type);
    CEREAL_PAIR(result, distance_resolution_m);
    CEREAL_PAIR(result, laser_corrections);
    CEREAL_PAIR(result, num_lasers);

    if (result.lidar_type == "RS32") {
        RsLidarAdditionalConfig additional_config;
        archive(cereal::make_nvp("additional_config", additional_config));
        result.additional_config = Any(additional_config);
    }
}

}  // namespace senseAD
