/*
 * Copyright (C) 2017 by SenseTime Group Limited. All rights reserved.
 * Yue Dayu <yuedayu@sensetime.com>
 */

#pragma once

#include "cereal/archives/binary.hpp"
#include "ad_common/config_utils/optional/nvp.hpp"

namespace cereal {

template <class T>
void prologue(BinaryInputArchive &, OptionalNameValuePair<T> const &) {}

template <class T>
void epilogue(BinaryOutputArchive &, OptionalNameValuePair<T> const &) {}

template <class T>
void prologue(BinaryOutputArchive &, OptionalNameValuePair<T> const &) {}

template <class T>
void epilogue(BinaryInputArchive &, OptionalNameValuePair<T> const &) {}

template <class T>
void CEREAL_LOAD_FUNCTION_NAME(BinaryInputArchive &archive,
                               OptionalNameValuePair<T> &value) {
    archive(value.value);
}

template <class T>
void CEREAL_SAVE_FUNCTION_NAME(BinaryOutputArchive &archive,
                               OptionalNameValuePair<T> const &value) {
    archive(value.value);
}

}  // namespace cereal
