/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： viz_util.h
 *   author     ： taoshaoyuan
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <adsfi_proto/viz/nav_msgs.pb.h>
#include <adsfi_proto/viz/sensor_msgs.pb.h>
#include <adsfi_proto/viz/visualization_msgs.pb.h>
#include <depend/proto/perception/transport_element.pb.h>
#include <depend/proto/soc/sensor_image.pb.h>

#include <memory>
#include <string>
#include <vector>

#include "map_fusion/fusion_common/common_data.h"
#include "map_fusion/fusion_common/element_map.h"

namespace hozon {
namespace mp {
namespace mf {
namespace viz {

struct Rgb {
  float r = 0;
  float g = 0;
  float b = 0;
};

enum Color {
  WHITE = 0,
  GREY,
  BLACK,
  RED,
  ORANGE,
  YELLOW,
  GREEN,
  BLUE,
  CYAN,
  PURPLE,
};

Rgb ColorRgb(Color color);

enum LineType {
  SINGLE_SOLID = 0,
  SINGLE_DASHED,
  SHORT_SINGLE_DASHED,
  DOUBLE_SOLID,
  DOUBLE_DASHED,
  LEFT_SOLID_RIGHT_DASHED,
  LEFT_DASHED_RIGHT_SOLID,
};

struct LineCubic {
  float start_x = 0;
  float end_x = 0;
  float c0 = 0;
  float c1 = 0;
  float c2 = 0;
  float c3 = 0;
};

void PoseToTf(const Pose& pose, const std::string& frame_id,
              const std::string& child_frame_id,
              adsfi_proto::viz::TransformStamped* tf);

void PoseToOdom(const Pose& pose, const std::string& frame_id,
                adsfi_proto::viz::Odometry* odom);

void LineCubicToMarker(const LineCubic& cubic, const Rgb& rgb, LineType type,
                       const std::string& frame_id, const std::string& ns,
                       double stamp, double lifetime, int32_t id,
                       float sample_dist, adsfi_proto::viz::Marker* marker);

using MarkerArrayPtr = std::shared_ptr<adsfi_proto::viz::MarkerArray>;
using CompressedImagePtr = std::shared_ptr<adsfi_proto::viz::CompressedImage>;
using PoseArrayPtr = std::shared_ptr<adsfi_proto::viz::PoseArray>;

MarkerArrayPtr LaneInfoToMarkers(const hozon::perception::LaneInfo& lane,
                                 const std::string& frame_id,
                                 const std::string& ns, double stamp,
                                 double lifetime);

MarkerArrayPtr TransportElementToMarkers(
    const hozon::perception::TransportElement& element,
    const std::string& frame_id, const std::string& ns, double lifetime);

/**
 * @brief Convert image in compressed jpeg format to compressed image for
 * RvizAgent visualization
 * @param sensor_img: input compressed jpeg image
 * @return converted image pointer, nullptr if converting failed
 */
CompressedImagePtr JpegImageToVizImage(
    const hozon::soc::CompressedImage& sensor_img);

/**
 * @brief Convert soc image in format YUV-NV12 to compressed image for RvizAgent
 * visualization
 * @param yuv_image: input soc image
 * @param quality: compression quality, should in range (0, 100]
 * @param resize_factor: resizing factor to shrink image, should in range
 * (0., 1.]
 * @return converted compressed image pointer, nullptr if converting failed
 */
CompressedImagePtr YUVNV12ImageToVizImage(
    const std::shared_ptr<hozon::soc::Image>& yuv_image, int quality = 70,
    double resize_factor = 1.0);

void ElementBoundaryNodesToMarker(
    const std::vector<em::BoundaryNode::Ptr>& nodes,
    const std::string& frame_id, const std::string& ns, int32_t id,
    double stamp, double lifetime, Color color,
    adsfi_proto::viz::Marker* marker);

void ElementBoundaryToMarker(const em::Boundary& boundary,
                             const std::string& frame_id, const std::string& ns,
                             int32_t id, double stamp, double lifetime,
                             Color color, adsfi_proto::viz::Marker* marker);

MarkerArrayPtr ElementMapToMarkers(const em::ElementMap& map,
                                   const std::string& frame_id,
                                   const std::string& ns, double stamp,
                                   double lifetime);

PoseArrayPtr PosesToPoseArray(const std::vector<Pose>& poses,
                              const std::string& frame_id, double stamp);

}  // namespace viz
}  // namespace mf
}  // namespace mp
}  // namespace hozon
