/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * GUO Zhichong <guozhichong@sensetime.com>
 */
#pragma once

// should always include following 4 cereal's archives
#include "cereal/archives/binary.hpp"
#include "cereal/archives/json.hpp"
#include "cereal/archives/xml.hpp"
#include "cereal/archives/portable_binary.hpp"
// private user components header
#include "property/map_matching_property.hpp"
// public user components header
#include "ad_common/config_utils/macros.hpp"

namespace senseAD {
namespace property {

REGISTER_CEREAL_SERIALIZE(MapMatchingProperty& property) {  // NOLINT
    CEREAL_PAIR(property, laneline_filter_property);
    CEREAL_PAIR(property, matching_property);
}

REGISTER_CEREAL_SERIALIZE(LanelineFilterProperty& property) {  // NOLINT
    CEREAL_PAIR(property, kCacheSize);
    CEREAL_PAIR(property, kAdditionalSamplePts);
    CEREAL_PAIR(property, kMergeThreshold);
}

REGISTER_CEREAL_SERIALIZE(MatchingProperty& property) {  // NOLINT
    CEREAL_PAIR(property, OpenLanelineMatching);
    CEREAL_PAIR(property, OpenStoplineMatching);
    CEREAL_PAIR(property, OpenRelocalization);
    CEREAL_PAIR(property, UseEgoline);
    CEREAL_PAIR(property, UseWLSE);
    CEREAL_PAIR(property, LanelineQueryLength);
    CEREAL_PAIR(property, HdmLongitudinalEffectiveRangeMin);
    CEREAL_PAIR(property, HdmLongitudinalEffectiveRangeMax);
    CEREAL_PAIR(property, PerceivedLongitudinalEffectiveRangeMin);
    CEREAL_PAIR(property, PerceivedLongitudinalEffectiveRangeMax);
    CEREAL_PAIR(property, PRLanelinesMinLength);
    CEREAL_PAIR(property, PRLinePtsMinInterval);
    CEREAL_PAIR(property, MaxPRHdmPtDist);
    CEREAL_PAIR(property, MinPtsEachPRLine);
    CEREAL_PAIR(property, LanelinesMaxDistance);
    CEREAL_PAIR(property, LanelinesQualifiedDistance);
    CEREAL_PAIR(property, LanelinesMaxAngle);
    CEREAL_PAIR(property, LanelinesMaxLateralShift);
    CEREAL_PAIR(property, StoplineMinimumLength);
    CEREAL_PAIR(property, StoplineMaximumLength);
    CEREAL_PAIR(property, StoplineMaxLengthDifference);
    CEREAL_PAIR(property, StoplineMaximumCorrection);
    CEREAL_PAIR(property, HdmSearchingRadius);
    CEREAL_PAIR(property, MinLostFrames);
    CEREAL_PAIR(property, RelocalizationMaxAngle);
    CEREAL_PAIR(property, RelocalizationMinIntersentionRatio);
    CEREAL_PAIR(property, RegularizationRatio);
}

}  // namespace property
}  // namespace senseAD
