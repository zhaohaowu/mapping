/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * WEI Zhengyuan <weizhengyuan@sensetime.com>
 * Guo Zhichong <guozhichong@sensetime.com>
 */
#pragma once

#include <vector>
#include <opencv2/core/core.hpp>

// should always include following 4 cereal's archives
#include "cereal/archives/binary.hpp"
#include "cereal/archives/json.hpp"
#include "cereal/archives/xml.hpp"
#include "cereal/archives/portable_binary.hpp"
#include "cereal/types/vector.hpp"
#include "cereal/details/traits.hpp"
// public user components header
#include "ad_common/config_utils/macros.hpp"
#include "ad_common/data_type/base.hpp"

namespace cv {
REGISTER_CEREAL_SERIALIZE(cv::Size &size) {  // NOLINT
    CEREAL_PAIR(size, width);
    CEREAL_PAIR(size, height);
}

REGISTER_CEREAL_SERIALIZE(cv::Point &point) {  // NOLINT
    CEREAL_PAIR(point, x);
    CEREAL_PAIR(point, y);
}

REGISTER_CEREAL_SERIALIZE(cv::Point2f &point) {  // NOLINT
    CEREAL_PAIR(point, x);
    CEREAL_PAIR(point, y);
}

REGISTER_CEREAL_SERIALIZE(cv::Point3f &point) {  // NOLINT
    CEREAL_PAIR(point, x);
    CEREAL_PAIR(point, y);
    CEREAL_PAIR(point, z);
}

REGISTER_CEREAL_SERIALIZE(cv::Point2d &point) {  // NOLINT
    CEREAL_PAIR(point, x);
    CEREAL_PAIR(point, y);
}

REGISTER_CEREAL_SERIALIZE(cv::Point3d &point) {  // NOLINT
    CEREAL_PAIR(point, x);
    CEREAL_PAIR(point, y);
    CEREAL_PAIR(point, z);
}

REGISTER_CEREAL_SERIALIZE(cv::Vec4i &vec) {  // NOLINT
    archive(cereal::make_nvp("v0", vec[0]));
    archive(cereal::make_nvp("v1", vec[1]));
    archive(cereal::make_nvp("v2", vec[2]));
    archive(cereal::make_nvp("v3", vec[3]));
}

REGISTER_CEREAL_SERIALIZE(cv::Rect &rect) {  // NOLINT
    CEREAL_PAIR(rect, x);
    CEREAL_PAIR(rect, y);
    CEREAL_PAIR(rect, width);
    CEREAL_PAIR(rect, height);
}

template <
    class Archive,
    cereal::traits::DisableIf<cereal::traits::is_text_archive<Archive>::value> =
        cereal::traits::sfinae>
void save(Archive &ar, const cv::Mat &mat) {  // NOLINT
    int rows, cols, type;
    bool continuous;
    rows = mat.rows;
    cols = mat.cols;
    type = mat.type();
    continuous = mat.isContinuous();
    ar &rows &cols &type &continuous;
    if (continuous) {
        const int data_size = rows * cols * static_cast<int>(mat.elemSize());
        auto mat_data = cereal::binary_data(mat.ptr(), data_size);
        ar &mat_data;
    } else {
        const int row_size = cols * static_cast<int>(mat.elemSize());
        for (int i = 0; i < rows; i++) {
            auto row_data = cereal::binary_data(mat.ptr(i), row_size);
            ar &row_data;
        }
    }
}

template <
    class Archive,
    cereal::traits::DisableIf<cereal::traits::is_text_archive<Archive>::value> =
        cereal::traits::sfinae>
void load(Archive &ar, cv::Mat &mat) {  // NOLINT
    int rows, cols, type;
    bool continuous;
    ar &rows &cols &type &continuous;
    if (continuous) {
        mat.create(rows, cols, type);
        const int data_size = rows * cols * static_cast<int>(mat.elemSize());
        auto mat_data = cereal::binary_data(mat.ptr(), data_size);
        ar &mat_data;
    } else {
        mat.create(rows, cols, type);
        const int row_size = cols * static_cast<int>(mat.elemSize());
        for (int i = 0; i < rows; i++) {
            auto row_data = cereal::binary_data(mat.ptr(i), row_size);
            ar &row_data;
        }
    }
}

template <
    class Archive,
    cereal::traits::EnableIf<cereal::traits::is_text_archive<Archive>::value> =
        cereal::traits::sfinae>
void save(Archive &ar, const cv::Mat &mat) {  // NOLINT
    int rows, cols, type;
    bool continuous;
    rows = mat.rows;
    cols = mat.cols;
    type = mat.type();
    continuous = mat.isContinuous();
    ar(cereal::make_nvp("rows", rows));
    ar(cereal::make_nvp("cols", cols));
    ar(cereal::make_nvp("type", type));
    ar(cereal::make_nvp("continuous", continuous));
    // assert(mat.dims == 2); // correct?
    if (continuous) {
        std::vector<std::vector<float>> mat_data;
        for (int i = 0; i < rows; i++) {
            Mat this_row = mat.row(i);
            mat_data.push_back(std::vector<float>(this_row.begin<float>(),
                                                  this_row.end<float>()));
        }
        ar &cereal::make_nvp("data", mat_data);
    }
}

template <
    class Archive,
    cereal::traits::EnableIf<cereal::traits::is_text_archive<Archive>::value> =
        cereal::traits::sfinae>
void load(Archive &ar, cv::Mat &mat) {  // NOLINT
    int32_t rows;
    int32_t cols;
    int32_t type;
    bool continuous;
    std::vector<std::vector<float32_t>> mat_data;
    ar(cereal::make_nvp("rows", rows));
    ar(cereal::make_nvp("cols", cols));
    ar(cereal::make_nvp("type", type));
    ar(cereal::make_nvp("continuous", continuous));
    ar(cereal::make_nvp("data", mat_data));
    if (rows != 0 && cols != 0) {
        assert(mat_data.size() > 0);
    }
    mat.create(rows, cols, CV_32FC1);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            mat.at<float32_t>(r, c) = mat_data[r][c];
        }
    }
}
}  // namespace cv
