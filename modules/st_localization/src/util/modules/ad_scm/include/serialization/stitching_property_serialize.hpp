/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * GUO Zhichong<guozhichong@sensetime.com>
 */
#pragma once

#include "cereal/archives/binary.hpp"
#include "cereal/archives/json.hpp"
#include "cereal/archives/xml.hpp"
#include "cereal/archives/portable_binary.hpp"
#include <cereal/types/vector.hpp>
#include <cereal/types/string.hpp>
// private user components header
#include "property/stitching_property.hpp"
// public user components header
#include "ad_common/config_utils/macros.hpp"

namespace senseAD {
namespace property {

REGISTER_CEREAL_SERIALIZE(StitchingProperty& property) {  // NOLINT
    CEREAL_PAIR(property, stitching_size);
    CEREAL_PAIR(property, input_size);
}

REGISTER_CEREAL_SERIALIZE(StitchingSize& property) {  // NOLINT
    CEREAL_PAIR(property, valid_width);
    CEREAL_PAIR(property, valid_height);
}

REGISTER_CEREAL_SERIALIZE(InputSize& property) {  // NOLINT
    CEREAL_PAIR(property, input_left_width);
    CEREAL_PAIR(property, input_left_height);
    CEREAL_PAIR(property, input_center_width);
    CEREAL_PAIR(property, input_center_height);
    CEREAL_PAIR(property, input_right_width);
    CEREAL_PAIR(property, input_right_height);
}

}  // namespace property
}  // namespace senseAD
