/*
 * Copyright (C) 2018 by SenseTime Group Limited. All rights reserved.
 * Alex KOng <kongzhenqiang@sensetime.com>
 */
#pragma once

#include <string>
#include <vector>
#include <opencv2/core/core.hpp>

namespace senseAD {

typedef struct StructFramePair {
    std::string frame_id;
    std::string parent_frame_id;
    cv::Mat transform;
} FrameData;

typedef struct StructTFTree {
    std::string version;
    std::vector<FrameData> tf_relationship;
} TFTree;

typedef cv::Mat_<float> Transform;

typedef struct Frame {
    std::string frame_id = "";
    struct Frame* parent = nullptr;
    std::vector<struct Frame*> children;
    Transform trans;
} Frame;

}  // namespace senseAD
