/*
 * Copyright (C) 2017 by SenseTime Group Limited. All rights reserved.
 * Chen Shengjie <chenshengjie@sensetime.com>
 */
#pragma once

#include <map>
#include <string>

namespace senseAD {
namespace property {

enum class VehicleBrand {
    LINCOLN_MKZ = 0,
    HONDA_ACCORD = 1,
};

// See the link below for the definition of road traffic direction
//   https://en.wikipedia.org/wiki/Left-_and_right-hand_traffic
enum class RoadTrafficDirection {
    // Hong Kong, Macau and Japan are Left-Hand Traffic
    LEFT_HAND = 0,
    // Mainland China is Right-Hand Traffic
    RIGHT_HAND = 1,
};

typedef struct {
    VehicleBrand brand = VehicleBrand::LINCOLN_MKZ;
    RoadTrafficDirection traffic_direction = RoadTrafficDirection::RIGHT_HAND;
} GlobalProperty;

typedef std::map<VehicleBrand, std::string> VehicleBrandTable_t;
static const VehicleBrandTable_t VehicleBrandTable{
    {VehicleBrand::LINCOLN_MKZ, "LINCOLN_MKZ"},
    {VehicleBrand::HONDA_ACCORD, "HONDA_ACCORD"},
};

static inline VehicleBrand FromVehicleBrandStr(const std::string& brand_str) {
    VehicleBrand brand = VehicleBrand::LINCOLN_MKZ;
    VehicleBrandTable_t::const_iterator iter = VehicleBrandTable.begin();
    for (; iter != VehicleBrandTable.end(); ++iter) {
        if (brand_str == iter->second) {
            brand = iter->first;
            break;
        }
    }
    return brand;
}

static inline std::string GetVehicleBrandStr(const VehicleBrand& brand) {
    return VehicleBrandTable.at(brand);
}

typedef std::map<RoadTrafficDirection, std::string> TrafficDirTable_t;
static const TrafficDirTable_t TrafficDirTable{
    {RoadTrafficDirection::LEFT_HAND, "LEFT_HAND"},
    {RoadTrafficDirection::RIGHT_HAND, "RIGHT_HAND"},
};

static inline RoadTrafficDirection FromRoadTrafficDirectionStr(
    const std::string& dir_str) {
    RoadTrafficDirection dir = RoadTrafficDirection::RIGHT_HAND;
    TrafficDirTable_t::const_iterator iter = TrafficDirTable.begin();
    for (; iter != TrafficDirTable.end(); ++iter) {
        if (dir_str == iter->second) {
            dir = iter->first;
            break;
        }
    }
    return dir;
}

static inline std::string GetRoadTrafficDirectionStr(
    const RoadTrafficDirection& dir) {
    return TrafficDirTable.at(dir);
}

}  // namespace property
}  // namespace senseAD
