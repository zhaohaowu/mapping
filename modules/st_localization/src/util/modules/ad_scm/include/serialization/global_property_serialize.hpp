/*
 * Copyright (C) 2019~2020 by SenseTime Group Limited. All rights reserved.
 * Chen Shengjie <chenshengjie@sensetime.com>
 */
#pragma once

#include <string>
// should always include following 4 cereal's archives
#include "cereal/archives/binary.hpp"
#include "cereal/archives/json.hpp"
#include "cereal/archives/xml.hpp"
#include "cereal/archives/portable_binary.hpp"
// private user components header
#include "property/global_property.hpp"
// public user components header
#include "ad_common/config_utils/macros.hpp"

namespace senseAD {
namespace property {

template <class Archive>
std::string save_minimal(const Archive& archive, const VehicleBrand& value) {
    return GetVehicleBrandStr(value);
}

template <class Archive>
void load_minimal(const Archive& archive,
                  VehicleBrand& value,  // NOLINT
                  const std::string& name) {
    value = FromVehicleBrandStr(name);
}

template <class Archive>
std::string save_minimal(const Archive& archive,
                         const RoadTrafficDirection& value) {
    return GetRoadTrafficDirectionStr(value);
}

template <class Archive>
void load_minimal(const Archive& archive,
                  RoadTrafficDirection& value,  // NOLINT
                  const std::string& name) {
    value = FromRoadTrafficDirectionStr(name);
}

REGISTER_CEREAL_SERIALIZE(GlobalProperty& property) {  // NOLINT
    CEREAL_PAIR(property, brand);
    CEREAL_PAIR(property, traffic_direction);
}

}  // namespace property
}  // namespace senseAD
