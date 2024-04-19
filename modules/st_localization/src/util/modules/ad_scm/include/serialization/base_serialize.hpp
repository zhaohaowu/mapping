/*
 * Copyright (C) 2018~2020 by SenseTime Group Limited. All rights reserved.
 * Chen Shengjie <chenshengjie@sensetime.com>
 */
#pragma once

// should always include following 4 cereal's archives
#include "cereal/archives/binary.hpp"
#include "cereal/archives/json.hpp"
#include "cereal/archives/xml.hpp"
#include "cereal/archives/portable_binary.hpp"
// public user components header
#include "ad_common/config_utils/macros.hpp"
#include "ad_common/data_type/base.hpp"

namespace senseAD {

REGISTER_CEREAL_SERIALIZE(BgrValues_t &bgr) {  // NOLINT
    CEREAL_PAIR(bgr, blue);
    CEREAL_PAIR(bgr, green);
    CEREAL_PAIR(bgr, red);
}

REGISTER_CEREAL_SERIALIZE(PointENU_t &point) {  // NOLINT
    CEREAL_PAIR(point, x);
    CEREAL_PAIR(point, y);
    CEREAL_PAIR(point, z);
}

REGISTER_CEREAL_SERIALIZE(PointLLH_t &point) {  // NOLINT
    CEREAL_PAIR(point, lat);
    CEREAL_PAIR(point, lon);
    CEREAL_PAIR(point, height);
}

REGISTER_CEREAL_SERIALIZE(EulerAngles_t &rpy) {  // NOLINT
    CEREAL_PAIR(rpy, roll);
    CEREAL_PAIR(rpy, pitch);
    CEREAL_PAIR(rpy, yaw);
}

REGISTER_CEREAL_SERIALIZE(Quaternion_t &quaternion) {  // NOLINT
    CEREAL_PAIR(quaternion, qx);
    CEREAL_PAIR(quaternion, qy);
    CEREAL_PAIR(quaternion, qz);
    CEREAL_PAIR(quaternion, qw);
}

REGISTER_CEREAL_SERIALIZE(Point2D_t &point) {  // NOLINT
    CEREAL_PAIR(point, x);
    CEREAL_PAIR(point, y);
}

REGISTER_CEREAL_SERIALIZE(Point2F_t &point) {  // NOLINT
    CEREAL_PAIR(point, x);
    CEREAL_PAIR(point, y);
}

REGISTER_CEREAL_SERIALIZE(Point3F_t &point) {  // NOLINT
    CEREAL_PAIR(point, x);
    CEREAL_PAIR(point, y);
    CEREAL_PAIR(point, z);
}

}  // namespace senseAD
