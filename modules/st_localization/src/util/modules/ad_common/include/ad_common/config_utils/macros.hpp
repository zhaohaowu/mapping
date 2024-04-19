/*
 * Copyright (C) 2017-2020 by SenseTime Group Limited. All rights reserved.
 * Sun Gefei <sungefei@sensetime.com>
 * Chen Kaige <chenkaige@sensetime.com>
 * Chen Shengjie <chenshengjie@sensetime.com>
 * Yue Dayu <yuedayu@sensetime.com>
 */

#pragma once

#include "ad_common/config_utils/optional/binary.hpp"
#include "ad_common/config_utils/optional/json.hpp"
#include "ad_common/config_utils/optional/xml.hpp"

namespace senseAD {

// Macros for Cereal-based serialization
#define REGISTER_CEREAL_SERIALIZE(parent) \
    template <class Archive>              \
    void serialize(Archive& archive, parent)  // NOLINT

#define REGISTER_CEREAL_SERIALIZE_TEMPLATE_TWO(DataType, parent, T, D) \
    template <class Archive, class T, class D>                         \
    void serialize(Archive& archive, DataType<T, D>& parent)  // NOLINT

#define CEREAL_BASE_CLASS(derived, base) \
    archive(cereal::make_nvp(#base, cereal::base_class<base>(&derived)));

#define CEREAL_PAIR(parent, child) \
    archive(cereal::make_nvp(#child, parent.child));

#define CEREAL_OPTIONAL_PAIR(parent, child, default_value) \
    archive(cereal::make_optional_nvp(#child, parent.child, default_value));

// Time33 HashMap
constexpr unsigned int StrHash(const char* str, int h = 0) {
    return !str[h] ? 5381 : (StrHash(str, h + 1) * 33) ^ str[h];
}

#define ENUM_CASE_NAME(value, name) \
    case StrHash((name)):           \
        return value;

#define ENUM_CASE_VALUE(value, name) \
    case value:                      \
        return name;

#define REGISTER_CEREAL_ENUM_SAVE(REGISTERED_TYPE, ENUM_DEF) \
    template <class Archive>                                 \
    std::string save_minimal(const Archive& archive,         \
                             const REGISTERED_TYPE& value) { \
        switch (value) {                                     \
            ENUM_DEF(ENUM_CASE_VALUE)                        \
            default:                                         \
                return "NONE";                               \
        }                                                    \
    }

#define REGISTER_CEREAL_ENUM_LOAD(REGISTERED_TYPE, ENUM_DEF)           \
    template <class Archive>                                           \
    void load_minimal(const Archive& archive, REGISTERED_TYPE& value,  \
                      const std::string& name) {                       \
        auto GetValueFunction = [name](const REGISTERED_TYPE& value) { \
            switch (StrHash(name.c_str())) {                           \
                ENUM_DEF(ENUM_CASE_NAME)                               \
                default:                                               \
                    return static_cast<REGISTERED_TYPE>(0);            \
            }                                                          \
        };                                                             \
        value = GetValueFunction(value);                               \
    }

#define REGISTER_CEREAL_ENUM_SERIALIZE(REGISTERED_TYPE, ENUM_DEF) \
    REGISTER_CEREAL_ENUM_SAVE(REGISTERED_TYPE, ENUM_DEF)          \
    REGISTER_CEREAL_ENUM_LOAD(REGISTERED_TYPE, ENUM_DEF)

}  // namespace senseAD
