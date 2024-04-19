/*
 * Copyright (C) 2017 by SenseTime Group Limited. All rights reserved.
 * Yue Dayu <yuedayu@sensetime.com>
 */

#pragma once

#include <utility>

#include "cereal/archives/json.hpp"
#include "ad_common/config_utils/optional/nvp.hpp"

namespace cereal {

template <class T>
void prologue(JSONInputArchive &, const OptionalNameValuePair<T> &) {}

template <class T>
void epilogue(JSONInputArchive &, const OptionalNameValuePair<T> &) {}

template <class T>
void prologue(JSONOutputArchive &, const OptionalNameValuePair<T> &) {}

template <class T>
void epilogue(JSONOutputArchive &, const OptionalNameValuePair<T> &) {}

template <class T>
void CEREAL_LOAD_FUNCTION_NAME(JSONInputArchive &archive,
                               OptionalNameValuePair<T> &nvp) {
    try {
        archive.setNextName(nvp.name);
        archive(nvp.value);
    } catch (cereal::Exception &) {
        nvp.value = std::move(nvp.default_value);
        archive.setNextName(nullptr);
    }
}

template <class T>
void CEREAL_SAVE_FUNCTION_NAME(JSONOutputArchive &archive,
                               OptionalNameValuePair<T> const &nvp) {
    archive.setNextName(nvp.name);
    archive(nvp.value);
}

}  // namespace cereal
