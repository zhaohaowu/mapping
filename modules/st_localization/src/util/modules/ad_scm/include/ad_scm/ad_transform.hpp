/*
 * Copyright (C) 2018 by SenseTime Group Limited. All rights reserved.
 * NI Tianwei <nitianwei@sensetime.com>
 * YUE Dayu <yuedayu@sensetime.com>
 * Dai Xing <daixing@sensetime.com>
 * Zhou Siyu <zhousiyu@sensetime.com>
 * Alex Kong <kongzhenqiang@sensetime.com>
 */

#pragma once

#include <vector>
#include <map>
#include <string>
#include <opencv2/core/core.hpp>

#include "ad_common/data_type/base.hpp"
#include "ad_scm/data_type/status.hpp"
#include "ad_scm/data_type/tf_tree.hpp"

namespace senseAD {
namespace common {
namespace utils {

void warp_r2a(float32_t x,
              float32_t y,
              float32_t base_x,
              float32_t base_y,
              float32_t heading,
              float32_t *r_x,
              float32_t *r_y);

void warp_a2r(float32_t x,
              float32_t y,
              float32_t base_x,
              float32_t base_y,
              float32_t heading,
              float32_t *r_x,
              float32_t *r_y);

cv::Point2f rotate(const cv::Point2f &center_pt,
                   const float32_t &radian,
                   const cv::Point2f ori_pt);

cv::Point2f utm2can(const cv::Point2f &ori_pt,
                    const float32_t &heading_radian,
                    const float32_t &car_heading_radian);

cv::Point2f TransformPointCoordbyRTMatrix(const cv::Mat &rotate,
                                          const cv::Mat &transform,
                                          const cv::Point2f point);

/*
 * utility class for transform from a coordinate to another one
 *
 */
class CoordinateTransformUtility {
 public:
    /**
     *  @brief get singleton instance of the coordinate transform utility
     */
    static CoordinateTransformUtility &get_shared_instance();

    /*
     * @brief Init tf tree from sensor config manager
     * tree.
     * @return the initializaion result
     *          SCM_SUCCESS: initialization OK
     *          others:     initialization failed.
     *
     */
    static senseAD::scmStatus_t init_with_configuration();

    /*
     * @brief read configuration from configuration file
     * @param config [in] the configuration file used to setup the transform
     * tree.
     * @return the initializaion result
     *          SCM_SUCCESS: initialization OK
     *          others:     initialization failed.
     *
     */
    static senseAD::scmStatus_t init_with_configuration(
        const std::string &config);

    /**
     * @brief reset the tf transform system.
     * @return void
     */
    static void reset();

    /*
     * get the registered frame list
     * if not initialized, or, no frame setup, return empty list.
     */
    const std::vector<std::string> get_frame_list() const;

    /*
     * @brief update frame, if the frame exist, update the relation, otherwise
     *  create a new one and attach to its parent frame.
     * @param parent [in] the parent frame id
     * @param self    [in] the updated frame id
     * @param trans   [in] the updated transform from self to parent.
     * @return true for success, false for failure.
     */
    bool update_frame(const std::string &parent,
                      const std::string &self,
                      const senseAD::Transform &trans);

    /*
     * @brief Get the transform between the "from" frame and the "to" frame
     * @param from [in]  the source frame id
     * @param to   [in]  the target frame id
     * @param trans[out] the transform retrieved from source frame to target
     *                   frame.
     * @return true for success, false for failure.
     */
    bool get_transform(const std::string &from,
                       const std::string &to,
                       senseAD::Transform &trans);

    /*
     * @brief Get the point in the destination coordinate
     * @param from [in]  the source frame id
     * @param to   [in]  the target frame id
     * @param src  [in]  the point in source frame
     * @param dst  [out] the point in target frame
     * @return true for success, false for failure.
     */
    bool get_transformed_positon(const std::string &from,
                                 const std::string &to,
                                 const cv::Point3f &src,
                                 cv::Point3f &dst);

    /*
     * @brief Get the points in the destination coordinate
     * @param from [in]  the source frame id
     * @param to   [in]  the target frame id
     * @param src  [in]  the points in source frame
     * @param dst  [out] the points in target frame
     * @return true for success, false for failure.
     */
    bool get_transformed_positons(const std::string &from,
                                  const std::string &to,
                                  const std::vector<cv::Point3f> &src,
                                  std::vector<cv::Point3f> &dst);

    /*
     * Get the points in the destination coordinate
     * @param from [in]  the source frame id
     * @param to   [in]  the target frame id
     * @param src  [in]  the points in source frame(column as a point vector)
     * @param dst  [out] the points in target frame (column as a point vector)
     * @return true for success, false for failure.
     */
    bool get_transformed_positons(const std::string &from,
                                  const std::string &to,
                                  const cv::Mat &src,
                                  cv::Mat &dst);

    /*
     * Get the frame with specified frame id
     * @param from [in]  the frame id
     * @param frm  [out] the output frame
     * @return true for success, false for failure.
     */
    bool get_frame(const std::string &frame_id, senseAD::Frame *&frm);

 private:
    senseAD::Frame *get_frame(const std::string &frame_id);

 private:
    static std::vector<senseAD::Frame *> frame_trees_;

    static std::map<std::string, senseAD::Transform> cached_frame_data_;

    static bool initialized;
};

}  // namespace utils
}  // namespace common
}  // namespace senseAD
