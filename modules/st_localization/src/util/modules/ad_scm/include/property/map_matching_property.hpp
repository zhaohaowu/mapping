/*
 * Copyright (C) 2018 by SenseTime Group Limited. All rights reserved.
 * GUO Zhichong <guozhichong@sensetime.com>
 */

#pragma once

#include "ad_common/data_type/base.hpp"

namespace senseAD {
namespace property {

/**
 * @brief Laneline Filter property
 **/
typedef struct StructLanelineFilterProperty {
    // const int LANELINE_SLIDE_WINDOW_SIZE = 4;
    int32_t kCacheSize = 4;
    // const int LANELINE_COMPENSATE_EXTRA_SAMPLE_PT = 4;
    int32_t kAdditionalSamplePts = 4;
    // const float LANELINE_MERGE_THRES = 1.0;  // no more than 100000
    float32_t kMergeThreshold = 1.0;
} LanelineFilterProperty;

/*
 * @brief: Package of map matching Property
 */
typedef struct StructMatchingProperty {
    bool OpenLanelineMatching = true;
    bool OpenStoplineMatching = true;
    bool OpenRelocalization = true;
    bool UseEgoline = false;
    bool UseWLSE = false;
    int32_t LanelineQueryLength = 100;
    float32_t HdmLongitudinalEffectiveRangeMin = 0;
    float32_t HdmLongitudinalEffectiveRangeMax = 20;
    float32_t PerceivedLongitudinalEffectiveRangeMin = 4;
    float32_t PerceivedLongitudinalEffectiveRangeMax = 15;
    float32_t PRLanelinesMinLength = 3;
    float32_t PRLinePtsMinInterval = 0.8;
    float32_t MaxPRHdmPtDist = 1;
    float32_t MinPtsEachPRLine = 2;
    float32_t LanelinesMaxDistance = 1.0;
    float32_t LanelinesQualifiedDistance = 0.01;
    float32_t LanelinesMaxAngle = 0.1;
    float32_t LanelinesMaxLateralShift = 1.5;
    float32_t StoplineMinimumLength = 4;
    float32_t StoplineMaximumLength = 30;
    float32_t StoplineMaxLengthDifference = 8;
    float32_t StoplineMaximumCorrection = 12;
    float32_t HdmSearchingRadius = 40;
    float32_t MinLostFrames = 15;
    float32_t RelocalizationMaxAngle = 1.0;
    float32_t RelocalizationMinIntersentionRatio = 0.2;
    float32_t RegularizationRatio = 0.05;
} MatchingProperty;

/**
 * @brief Filter property for road structure
 **/
typedef struct StructMapMatchingProperty {
    LanelineFilterProperty laneline_filter_property;
    MatchingProperty matching_property;
} MapMatchingProperty;

}  // namespace property
}  // namespace senseAD
