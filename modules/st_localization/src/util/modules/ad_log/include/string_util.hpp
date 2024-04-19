/*
 * Copyright (C) 2018 by SenseTime Group Limited. All rights reserved.
 * Chen Shengjie <chenshengjie@sensetime.com>
 */
#pragma once

#include <string>

namespace senseAD {

constexpr unsigned int Str2Int(const char* str, int h = 0) {
    return !str[h] ? 5381 : (Str2Int(str, h + 1) * 33) ^ str[h];
}

inline bool StartsWith(const std::string& str, const std::string& prefix) {
    return (str.size() >= prefix.size()) &&
           (0 == str.compare(0, prefix.size(), prefix));
}

inline bool EndsWith(const std::string& str, const std::string& suffix) {
    return (str.size() >= suffix.size()) &&
           (0 ==
            str.compare(str.size() - suffix.size(), suffix.size(), suffix));
}

}  // namespace senseAD
