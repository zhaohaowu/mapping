/*
 * Copyright (C) 2017 by SenseTime Group Limited. All rights reserved.
 * Yue Dayu <yuedayu@sensetime.com>
 */

#pragma once

#include <string>
#include <utility>
#include "cereal/cereal.hpp"

namespace cereal {

template <class T>
struct OptionalNameValuePair {
 private:
    using Type = typename std::conditional<
        std::is_array<typename std::remove_reference<T>::type>::value,
        typename std::remove_cv<T>::type,
        typename std::conditional<std::is_lvalue_reference<T>::value,
                                  T,
                                  typename std::decay<T>::type>::type>::type;

    OptionalNameValuePair &operator=(OptionalNameValuePair const &) = delete;

 public:
    OptionalNameValuePair(char const *n, T &&v, T &&dv)
        : name(n),
          value(std::forward<T>(v)),
          default_value(std::forward<T>(dv)) {}

    OptionalNameValuePair(const OptionalNameValuePair &) = delete;

    char const *name;
    Type value;
    T default_value;
};

template <class T, class Tv>
OptionalNameValuePair<T> make_optional_nvp(const char *name,
                                           T &&option,
                                           Tv &&defaultValue) {
    return {name, std::forward<T>(option), T(defaultValue)};
}

template <class T, class Tv>
OptionalNameValuePair<T> make_optional_nvp(const std::string &name,
                                           T &&option,
                                           Tv &&defaultValue) {
    return {name.c_str(), std::forward<T>(option), T(defaultValue)};
}

}  // namespace cereal

#define CEREAL_OPTIONAL_NVP(T, V) ::cereal::make_optional_nvp(#T, T, V)
