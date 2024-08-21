/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： viz_common.h
 *   author     ： taoshaoyuan
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <adsfi_proto/viz/nav_msgs.pb.h>
#include <adsfi_proto/viz/sensor_msgs.pb.h>
#include <adsfi_proto/viz/visualization_msgs.pb.h>
#include <depend/proto/perception/transport_element.pb.h>
#include <depend/proto/soc/sensor_image.pb.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "modules/map_fusion_02/base/element_map.h"
#include "modules/map_fusion_02/common/common_data.h"

namespace hozon {
namespace mp {
namespace mf {

struct RvizRgb {
  float r = 0;
  float g = 0;
  float b = 0;
};

enum RvizColor {
  R_WHITE = 0,
  R_GREY,
  R_BLACK,
  R_RED,
  R_ORANGE,
  R_YELLOW,
  R_GREEN,
  R_BLUE,
  R_CYAN,
  R_PURPLE,
};

RvizRgb ColorRgb(RvizColor color);

enum RvizLineType {
  R_SINGLE_SOLID = 0,
  R_SINGLE_DASHED,
  R_SHORT_SINGLE_DASHED,
  R_DOUBLE_SOLID,
  R_DOUBLE_DASHED,
  R_LEFT_SOLID_RIGHT_DASHED,
  R_LEFT_DASHED_RIGHT_SOLID,
};

struct RvizLineCubic {
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

void LineCubicToMarker(const RvizLineCubic& cubic, const RvizRgb& rgb,
                       RvizLineType type, const std::string& frame_id,
                       const std::string& ns, double stamp, double lifetime,
                       int32_t id, float sample_dist,
                       adsfi_proto::viz::Marker* marker);

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

void ElementBoundaryNodesToMarker(const std::vector<BoundaryNode::Ptr>& nodes,
                                  const std::string& frame_id,
                                  const std::string& ns, int32_t id,
                                  double stamp, double lifetime,
                                  RvizColor color,
                                  adsfi_proto::viz::Marker* marker);

void ElementBoundaryToMarker(const Boundary& boundary,
                             const std::string& frame_id, const std::string& ns,
                             int32_t id, double stamp, double lifetime,
                             RvizColor color, adsfi_proto::viz::Marker* marker);

void ElementOccEgoToMarker(const Boundary& boundary,
                           const std::string& frame_id, const std::string& ns,
                           int32_t id, double stamp, double lifetime,
                           RvizColor color, adsfi_proto::viz::Marker* marker);

void ElementOccRoadToMarker(const OccRoad& occ_road,
                            const std::string& frame_id, const std::string& ns,
                            int32_t id, double stamp, double lifetime,
                            RvizColor color, adsfi_proto::viz::Marker* marker);

MarkerArrayPtr ElementMapToMarkers(const ElementMap& map,
                                   const std::string& frame_id,
                                   const std::string& ns, double stamp,
                                   double lifetime);

PoseArrayPtr PosesToPoseArray(const std::vector<Pose>& poses,
                              const std::string& frame_id, double stamp);

}  // namespace mf
}  // namespace mp
}  // namespace hozon
