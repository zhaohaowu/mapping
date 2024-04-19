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
#include "cereal/types/string.hpp"
#include "cereal/types/array.hpp"
// private user components header
#include "property/gnss_transformer_property.hpp"
// public user components header
#include "ad_common/config_utils/macros.hpp"

namespace senseAD {
namespace property {

REGISTER_CEREAL_SERIALIZE(GnssTransformerProperty& property) {  // NOLINT
    CEREAL_PAIR(property, wgs84_source);
    CEREAL_PAIR(property, utm_target);
    CEREAL_PAIR(property, yaw_offset);
    CEREAL_PAIR(property, map_offset);
}

}  // namespace property
}  // namespace senseAD
