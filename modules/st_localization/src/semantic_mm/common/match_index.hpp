/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Fang Yu <fangyu1@senseauto.com>
 */

#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "localization/data_type/semantic_type.hpp"

namespace senseAD {
namespace localization {
namespace smm {

using MatchPair = std::pair<id_t, id_t>;
using MatchPairVec = std::vector<std::pair<id_t, id_t>>;
using FrameMatchPairVec =
    std::unordered_map<SemanticType, MatchPairVec, EnumClassHash>;

struct MatchPairHash {
  size_t operator()(const std::pair<id_t, id_t>& idx) const {
    size_t seed = 0;
    hash_combine(&seed, idx.first);
    hash_combine(&seed, idx.second);
    return seed;
  }
  template <class T>
  void hash_combine(size_t* seed, const T& v) const {
    std::hash<T> hasher;
    *seed ^= hasher(v) + 0x9e3779b9 + (*seed << 6) + (*seed >> 2);
  }
};
class MatchIndex {
 public:
  DEFINE_SMART_PTR(MatchIndex)

  MatchIndex() {}
  ~MatchIndex() {}

  void Reset() { match_pairs_.clear(); }

  void AddSemanticMatch(SemanticType semantic_type,
                        const MatchPairVec& match_pairs) {
    auto& vec = match_pairs_[semantic_type];
    vec.insert(vec.end(), match_pairs.begin(), match_pairs.end());
  }
  void AddSemanticMatch(SemanticType semantic_type,
                        const MatchPair& match_pair) {
    auto& vec = match_pairs_[semantic_type];
    vec.emplace_back(match_pair);
  }

  bool GetSemanticMatch(SemanticType semantic_type,
                        MatchPairVec* match_pairs) const {
    auto iter = match_pairs_.find(semantic_type);
    if (iter == match_pairs_.end()) return false;
    *match_pairs = iter->second;
    return true;
  }

  size_t GetSemanticMatchNum(SemanticType semantic_type) const {
    auto iter = match_pairs_.find(semantic_type);
    if (iter == match_pairs_.end()) return 0;
    return iter->second.size();
  }

  FrameMatchPairVec GetAllSemanticMatch() const { return match_pairs_; }

  MatchPairVec GetAllMatchInOneVec() const {
    MatchPairVec vec;
    for (const auto& item : match_pairs_) {
      vec.insert(vec.end(), item.second.begin(), item.second.end());
    }
    return vec;
  }

 private:
  FrameMatchPairVec match_pairs_;
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
