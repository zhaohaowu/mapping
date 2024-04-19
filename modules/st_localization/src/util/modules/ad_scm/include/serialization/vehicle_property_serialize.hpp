/*
 * Copyright (C) 2019~2020 by SenseTime Group Limited. All rights reserved.
 * Yue Dayu <yuedayu@sensetime.com>
 */

#pragma once

// should always include following 4 cereal's archives
#include "cereal/archives/binary.hpp"
#include "cereal/archives/json.hpp"
#include "cereal/archives/xml.hpp"
#include "cereal/archives/portable_binary.hpp"
// private user components header
#include "serialization/opencv_serialize.hpp"
// public user components header
#include "ad_scm/data_type/vehicle_property.hpp"

namespace senseAD {

REGISTER_CEREAL_SERIALIZE(VehicleConfig &result) {
    CEREAL_PAIR(result, car_center_height);
    CEREAL_PAIR(result, wheel_track);
    CEREAL_PAIR(result, wheel_base);
    CEREAL_PAIR(result, front_right_wheel);
    CEREAL_PAIR(result, front_left_wheel);
    CEREAL_PAIR(result, back_right_wheel);
    CEREAL_PAIR(result, back_left_wheel);
    CEREAL_PAIR(result, car_width);
    CEREAL_PAIR(result, car_length);
    CEREAL_PAIR(result, car_height);
    CEREAL_PAIR(result, front_axis_to_front_bumper);
    CEREAL_PAIR(result, rear_axis_to_rear_bumper);
}

}  // namespace senseAD
