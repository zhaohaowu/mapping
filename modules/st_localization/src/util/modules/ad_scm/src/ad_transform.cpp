/*
 * Copyright (C) 2018 by SenseTime Group Limited. All rights reserved.
 * Dai Xing <daixing@sensetime.com>
 * Zhou Siyu <zhousiyu@sensetime.com>
 */

#include "ad_scm/ad_transform.hpp"

#include <cmath>
#include <stack>
#include <queue>
#include <algorithm>

#include "ad_scm/ad_scm.hpp"
#include "serialization/tf_tree_serialize.hpp"
#include "serialization/sensor_config_serialize.hpp"
#include "serialization/camera_property_serialize.hpp"
#include "serialization/global_property_serialize.hpp"
#include "serialization/stitching_property_serialize.hpp"
#include "serialization/map_matching_property_serialize.hpp"
#include "serialization/gnss_transformer_property_serialize.hpp"
#include "serialization/vehicle_property_serialize.hpp"

#include "ad_common/config_utils/config_utils.hpp"
#include "ad_log/ad_log.hpp"

namespace senseAD {
namespace common {
namespace utils {

using senseAD::sensorconfig::InitializeSensorConfigManager;
using senseAD::sensorconfig::SensorConfigManager;

void warp_r2a(float32_t x,
              float32_t y,
              float32_t base_x,
              float32_t base_y,
              float32_t heading,
              float32_t *r_x,
              float32_t *r_y) {
    float32_t r_heading = heading;
    float32_t cos_val = cos(r_heading);
    float32_t sin_val = sin(r_heading);

    *r_x = x * cos_val - y * sin_val + base_x;
    *r_y = y * cos_val + x * sin_val + base_y;
}

void warp_a2r(float32_t x,
              float32_t y,
              float32_t base_x,
              float32_t base_y,
              float32_t heading,
              float32_t *r_x,
              float32_t *r_y) {
    float32_t r_heading = heading;
    float32_t cos_val = cos(r_heading);
    float32_t sin_val = sin(r_heading);
    x -= base_x;
    y -= base_y;

    *r_x = x * cos_val + y * sin_val;
    *r_y = y * cos_val - x * sin_val;
}

cv::Point2f rotate(const cv::Point2f &center_pt,
                   const float32_t &radian,
                   const cv::Point2f ori_pt) {
    cv::Point2f result_pt;
    float32_t cos_val = cos(radian);
    float32_t sin_val = sin(radian);

    float32_t x = ori_pt.x - center_pt.x;
    float32_t y = ori_pt.y - center_pt.y;

    result_pt.x = x * cos_val - y * sin_val + center_pt.x;
    result_pt.y = y * cos_val + x * sin_val + center_pt.y;
    return result_pt;
}

cv::Point2f utm2car(const cv::Point2f &ori_pt,
                    const float32_t &heading_radian,
                    const float32_t &car_heading_radian) {
    float32_t rotate_radian = car_heading_radian - heading_radian;
    cv::Point2f center_pt(0, 0);
    return rotate(center_pt, rotate_radian, ori_pt);
}

cv::Point2f TransformPointCoordbyRTMatrix(const cv::Mat &rotate,
                                          const cv::Mat &transform,
                                          const cv::Point2f point) {
    cv::Mat p(2, 1, CV_32FC1);
    p.at<float>(0, 0) = point.x;
    p.at<float>(1, 0) = point.y;
    cv::Mat out = rotate * p + transform;

    cv::Point2f transform_point(0, 0);
    transform_point.x = out.at<float>(0, 0);
    transform_point.y = out.at<float>(1, 0);
    return transform_point;
}

#define CHECK_UNINITIALIZE_RETURN(ret) \
    if (!CoordinateTransformUtility::initialized) return (ret)

bool CoordinateTransformUtility::initialized = false;

std::vector<senseAD::Frame *> CoordinateTransformUtility::frame_trees_;

std::map<std::string, senseAD::Transform>
    CoordinateTransformUtility::cached_frame_data_;

CoordinateTransformUtility &CoordinateTransformUtility::get_shared_instance() {
    static CoordinateTransformUtility util;
    return util;
}

// read configuration
senseAD::scmStatus_t CoordinateTransformUtility::init_with_configuration(
    const std::string &config) {
    senseAD::TFTree tf;
    CoordinateTransformUtility &inst = get_shared_instance();
    senseAD::confStatus_t status =
        senseAD::common::utils::ConfigurationReader::LoadJSON(config, &tf);
    if (status != CONF_SUCCESS) {
        return SCM_INVALID_PARAM;
    }
    for (auto item : tf.tf_relationship) {
        inst.update_frame(item.parent_frame_id, item.frame_id, item.transform);
    }
    initialized = true;
    return SCM_SUCCESS;
}

senseAD::scmStatus_t CoordinateTransformUtility::init_with_configuration() {
    const auto &sensor_config = *SensorConfigManager::GetInstance();
    std::vector<std::pair<std::string, std::string>> relationship_list;
    senseAD::scmStatus_t status;
    status = sensor_config.GetRelationshipPair(&relationship_list);
    if (status != senseAD::SCM_SUCCESS) {
        return SCM_TF_TREE_INIT_ERROR;
    }
    std::vector<std::string> parent;
    std::vector<std::string> children;
    std::stack<std::string> children_stack;
    cv::Mat identity = cv::Mat::eye(4, 4, CV_32F);
    parent.push_back("vehicle");
    children.push_back("car_center");
    children_stack.push("car_center");
    CoordinateTransformUtility &inst = get_shared_instance();
    inst.update_frame(parent[0], children[0], identity);
    while (!children_stack.empty()) {
        std::string parent_id = children_stack.top();
        children_stack.pop();
        auto iter = relationship_list.begin();
        while (!relationship_list.empty() && iter != relationship_list.end()) {
            cv::Mat tranfrom;
            status = sensor_config.GetExternalCalibConfig(
                iter->first, iter->second, &tranfrom);
            if (status != senseAD::SCM_SUCCESS) {
                return SCM_TF_TREE_INIT_ERROR;
            }
            if (parent_id == iter->first) {
                for (size_t i = 0; i < children.size(); i++) {
                    if (iter->second == children[i]) {
                        AD_LERROR(TFTree) << "two parents point to one child";
                        return SCM_TF_TREE_INIT_ERROR;
                    }
                }
                inst.update_frame(iter->first, iter->second, tranfrom.inv());
                children_stack.push(iter->second);
                parent.push_back(iter->first);
                children.push_back(iter->second);
                iter = relationship_list.erase(iter);
                continue;
            }
            if (parent_id == iter->second) {
                for (size_t i = 0; i < children.size(); i++) {
                    if (iter->first == children[i]) {
                        AD_LERROR(TFTree) << "two parents point to one child";
                        return SCM_TF_TREE_INIT_ERROR;
                    }
                }
                inst.update_frame(iter->second, iter->first, tranfrom);
                children_stack.push(iter->first);
                parent.push_back(iter->second);
                children.push_back(iter->first);
                iter = relationship_list.erase(iter);
                continue;
            }
            iter++;
        }
    }
    initialized = true;
    return SCM_SUCCESS;
}

void CoordinateTransformUtility::reset() {
    std::stack<senseAD::Frame *> frame_stack;
    for (auto tree : get_shared_instance().frame_trees_) {
        if (tree == nullptr) {
            continue;
        }
        frame_stack.push(tree);
    }
    while (!frame_stack.empty()) {
        auto item = frame_stack.top();
        frame_stack.pop();
        for (auto it = item->children.begin(); it != item->children.end();
             ++it) {
            if ((*it)->children.size() == 0) {
                std::string frame_name = (*it)->frame_id;
                delete (*it);
                (*it) = nullptr;
            } else {
                frame_stack.push(*it);
            }
        }
        delete item;
        item = nullptr;
    }
    initialized = false;
}

const std::vector<std::string> CoordinateTransformUtility::get_frame_list()
    const {
    std::vector<std::string> frame_ids;
    CHECK_UNINITIALIZE_RETURN(frame_ids);
    std::queue<senseAD::Frame *> frames;
    for (auto tree : frame_trees_) {
        frame_ids.emplace_back(tree->frame_id);
        frames.push(tree);
    }
    while (!frames.empty()) {
        senseAD::Frame *fm = frames.front();
        frames.pop();
        for (auto child : fm->children) {
            frame_ids.emplace_back(child->frame_id);
            frames.push(child);
        }
    }
    return frame_ids;
}

bool CoordinateTransformUtility::update_frame(const std::string &parent,
                                              const std::string &self,
                                              const senseAD::Transform &data) {
    senseAD::Frame *fm = get_frame(self);
    senseAD::Frame *pfm = get_frame(parent);
    if (pfm != nullptr && fm != nullptr &&
        fm->parent->frame_id != pfm->frame_id) {
        // change the parent frame is not support
        return false;
    }
    if (pfm == nullptr) {
        pfm = new senseAD::Frame();
        pfm->frame_id = parent;
        pfm->parent = nullptr;
        // add new frame to the system
        frame_trees_.emplace_back(pfm);
    }
    if (fm == nullptr) {
        fm = new senseAD::Frame();
        fm->frame_id = self;
        fm->parent = pfm;
        if (pfm != nullptr) {
            pfm->children.emplace_back(fm);
        }
    }
    fm->trans = data.clone();
    return true;
}

bool CoordinateTransformUtility::get_transform(const std::string &from,
                                               const std::string &to,
                                               senseAD::Transform &tsfm) {
    CHECK_UNINITIALIZE_RETURN(false);
    std::string key = from + "," + to;
    auto iter = cached_frame_data_.find(key);
    if (cached_frame_data_.end() != iter) {
        tsfm = iter->second.clone();
        return true;
    }
    senseAD::Frame *src = get_frame(from);
    if (src == nullptr) {
        AD_LERROR(TFTree) << "get src frame error";
        return false;
    }
    senseAD::Frame *dst = get_frame(to);
    if (dst == nullptr) {
        AD_LERROR(TFTree) << "get dst frame error";
        return false;
    }
    senseAD::Transform src_accum = cv::Mat::eye(4, 4, CV_32F);
    senseAD::Transform dst_accum = cv::Mat::eye(4, 4, CV_32F);
    while (src->parent != nullptr) {
        src_accum = src->trans * src_accum;
        src = src->parent;
    }
    while (dst->parent != nullptr) {
        dst_accum = dst->trans * dst_accum;
        dst = dst->parent;
    }
    auto result = cached_frame_data_.insert(
        std::make_pair(key, dst_accum.inv() * src_accum));
    tsfm = result.first->second.clone();
    return result.second;
}

bool CoordinateTransformUtility::get_transformed_positon(
    const std::string &from,
    const std::string &to,
    const cv::Point3f &src,
    cv::Point3f &dst) {
    CHECK_UNINITIALIZE_RETURN(false);
    senseAD::Transform tsfm;
    bool ret = get_transform(from, to, tsfm);
    if (ret) {
        cv::Mat_<float> in = (cv::Mat_<float>(4, 1) << src.x, src.y, src.z, 1);
        cv::Mat_<float> out = tsfm * in;
        dst.x = out(0);
        dst.y = out(1);
        dst.z = out(2);
    }
    return ret;
}

bool CoordinateTransformUtility::get_transformed_positons(
    const std::string &from,
    const std::string &to,
    const std::vector<cv::Point3f> &src,
    std::vector<cv::Point3f> &dst) {
    CHECK_UNINITIALIZE_RETURN(false);
    dst.clear();
    int size = src.size();
    dst.resize(size);
    senseAD::Transform tsfm;
    bool ret = get_transform(from, to, tsfm);
    cv::Mat_<float> in;
    cv::Mat_<float> out;
    if (ret) {
        for (int i = 0; i < size; ++i) {
            in = (cv::Mat_<float>(4, 1) << src[i].x, src[i].y, src[i].z, 1);
            out = tsfm * in;
            dst[i].x = out(0);
            dst[i].y = out(1);
            dst[i].z = out(2);
        }
    }
    return ret;
}

bool CoordinateTransformUtility::get_transformed_positons(
    const std::string &from,
    const std::string &to,
    const cv::Mat &src,
    cv::Mat &dst) {
    CHECK_UNINITIALIZE_RETURN(false);
    bool ret = true;
    senseAD::Transform trans;
    ret = get_transform(from, to, trans);
    if (ret) {
        cv::Mat ext_src(4, src.cols, CV_32F);
        cv::Mat_<float> ext_row = (cv::Mat_<float>(3, 3) << 1, 1, 1, 1);
        cv::vconcat(src, ext_row, ext_src);
        cv::Mat ext_dst(4, 4, CV_32F);
        ext_dst = trans * ext_src;
        dst = ext_dst.rowRange(0, dst.rows);
    }
    return ret;
}

bool CoordinateTransformUtility::get_frame(const std::string &frame_id,
                                           senseAD::Frame *&frame) {
    senseAD::Frame *fm = get_frame(frame_id);
    if (!fm) {
        return false;
    }
    frame = fm;
    return true;
}

senseAD::Frame *CoordinateTransformUtility::get_frame(
    const std::string &frame_id) {
    std::queue<senseAD::Frame *> frame_queue;
    for (auto item : frame_trees_) {
        frame_queue.push(item);
    }
    while (!frame_queue.empty()) {
        auto it = frame_queue.front();
        frame_queue.pop();
        if (it->frame_id == frame_id) {
            return it;
        }
        if (!it->children.empty()) {
            for (auto &nd : it->children) {
                frame_queue.push(nd);
            }
        }
    }
    return nullptr;
}

}  // namespace utils
}  // namespace common
}  // namespace senseAD
