/*
 * Copyright (C) 2017 by SenseTime Group Limited. All rights reserved.
 * Yue Dayu <yuedayu@sensetime.com>
 */

#pragma once

#include <utility>

#include "cereal/archives/xml.hpp"
#include "ad_common/config_utils/optional/nvp.hpp"

namespace cereal {

template <class T>
void prologue(XMLInputArchive &, const OptionalNameValuePair<T> &) {}

template <class T>
void epilogue(XMLInputArchive &, const OptionalNameValuePair<T> &) {}

template <class T>
void prologue(XMLOutputArchive &, const OptionalNameValuePair<T> &) {}

template <class T>
void epilogue(XMLOutputArchive &, const OptionalNameValuePair<T> &) {}

template <class T>
void CEREAL_LOAD_FUNCTION_NAME(XMLInputArchive &archive,
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
void CEREAL_SAVE_FUNCTION_NAME(XMLOutputArchive &archive,
                               OptionalNameValuePair<T> const &nvp) {
    archive.setNextName(nvp.name);
    archive(nvp.value);
}

}  // namespace cereal
