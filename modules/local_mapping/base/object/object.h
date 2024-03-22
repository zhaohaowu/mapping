/*================================================================
*   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
*   file       ：
*   author     zhangwenhai
*   date       ：2023.03.17
================================================================*/

#pragma once
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "modules/local_mapping/base/object/object_types.h"

namespace hozon {
namespace mp {
namespace lm {

struct Object {
  // @brief object id per frame, required
  int id = -1;

  // @brief existence confidence, required
  float confidence = 1.0;

  // @brief center of the boundingbox (cx, cy, cz), required
  Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);
  // @brief covariance matrix of the center uncertainty, required
  Eigen::Matrix3f center_uncertainty;

  // @brief anchor point, required
  Eigen::Vector3d anchor_point = Eigen::Vector3d(0, 0, 0);

  /* @brief size = [length, width, height] of boundingbox
     length is the size of the main direction, required
  */
  Eigen::Vector3f size = Eigen::Vector3f(0, 0, 0);
  // @brief size variance, required
  Eigen::Vector3f size_variance = Eigen::Vector3f(0, 0, 0);

  // @brief theta, required, [-pai,pai]
  float theta = 0.0;
  // @brief theta variance, required  朝向角方差
  float theta_variance = 0.0;

  // oriented boundingbox information
  // @brief main direction of the object, required
  Eigen::Vector3f direction = Eigen::Vector3f(1, 0, 0);

  // @brief object type, required
  ObjectType type = ObjectType::UNKNOWN;
  // @brief probability for each type, required
  std::vector<float> type_probs;

  // @brief object sub-type, optional
  ObjectSubType sub_type = ObjectSubType::UNKNOWN;
  // @brief probability for each sub-type, optional

  // @brief age of the tracked object, required
  double tracking_time = 0.0;
  // @brief timestamp of latest measurement, required
  double latest_tracked_time = 0.0;
};

using ObjectPtr = std::shared_ptr<Object>;
using ObjectConstPtr = std::shared_ptr<const Object>;

struct Objects {
  std::vector<ObjectPtr> objects;
};

using ObjectsPtr = std::shared_ptr<Objects>;

using ObjectsConstPtr = std::shared_ptr<const Objects>;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
