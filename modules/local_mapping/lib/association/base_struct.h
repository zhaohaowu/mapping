// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: wenhai.zhang
// @file: lane_matcher.h
// @brief: matcher for lane

#pragma once

#include <iomanip>
#include <list>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace hozon {
namespace mp {
namespace lm {

// track_index, detect_lane_index, match_score
using MatchScoreTuple = std::tuple<size_t, size_t, float>;

struct AssociationResult {
  std::vector<MatchScoreTuple> assignments;
  std::vector<size_t> unassigned_tracks;
  std::vector<size_t> unsigned_objects;
};

struct MatcherInitOptions {
  // LaneMatchParam lane_match_param;
};

struct MatcherOptions {
  double timestamp = 0.0;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
