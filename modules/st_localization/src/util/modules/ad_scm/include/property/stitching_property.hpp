/*
 * Copyright (C) 2018 by SenseTime Group Limited. All rights reserved.
 * Zhang Shu <zhangshu@sensetime.com>
 * Yue Dayu <yuedayu@sensetime.com>
 * GUO Zhichong <guozhichong@sensetime.com>
 */

#pragma once

#include "ad_common/data_type/base.hpp"

namespace senseAD {
namespace property {

typedef struct StructStitchingSize {
    uint32_t valid_width;
    uint32_t valid_height;
} StitchingSize;

typedef struct StructInputSize {
    uint32_t input_left_width;
    uint32_t input_left_height;
    uint32_t input_center_width;
    uint32_t input_center_height;
    uint32_t input_right_width;
    uint32_t input_right_height;
} InputSize;

typedef struct StructStitchingProperty {
    StitchingSize stitching_size;
    InputSize input_size;
} StitchingProperty;

}  // namespace property
}  // namespace senseAD
