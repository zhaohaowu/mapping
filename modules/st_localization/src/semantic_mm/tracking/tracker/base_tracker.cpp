/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Fang Yu <fangyu1@senseauto.com>
 */

#include "semantic_mm/tracking/tracker/base_tracker.hpp"

namespace senseAD {
namespace localization {
namespace smm {

std::atomic<id_t> BaseTracker::next_id_{0};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
