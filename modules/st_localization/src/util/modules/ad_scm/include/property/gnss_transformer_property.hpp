/*
 * Copyright (C) 2017 by SenseTime Group Limited. All rights reserved.
 * WEI Zhengyuan <weizhengyuan@sensetime.com>
 * Chen Shengjie <chenshengjie@sensetime.com>
 */
#pragma once

#include <string>
#include <array>

namespace senseAD {
namespace property {

typedef struct {
    std::string wgs84_source;
    std::string utm_target;
    double yaw_offset;
    std::array<double, 3> map_offset;
} GnssTransformerProperty;

}  // namespace property
}  // namespace senseAD
