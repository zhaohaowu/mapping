/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * Alex KOng <kongzhenqiang@sensetime.com>
 */
#pragma once

// should always include following 4 cereal's archives
#include "cereal/archives/binary.hpp"
#include "cereal/archives/json.hpp"
#include "cereal/archives/xml.hpp"
#include "cereal/archives/portable_binary.hpp"
#include "cereal/types/string.hpp"
#include "cereal/types/vector.hpp"
// private user components header
#include "ad_scm/data_type/tf_tree.hpp"
#include "serialization/opencv_serialize.hpp"
// public user components header
#include "ad_common/config_utils/macros.hpp"

namespace senseAD {

REGISTER_CEREAL_SERIALIZE(FrameData& result) {  // NOLINT
    CEREAL_PAIR(result, frame_id);
    CEREAL_PAIR(result, parent_frame_id);
    CEREAL_PAIR(result, transform);
}

REGISTER_CEREAL_SERIALIZE(TFTree& result) {  // NOLINT
    CEREAL_PAIR(result, version);
    CEREAL_PAIR(result, tf_relationship);
}

}  // namespace senseAD
