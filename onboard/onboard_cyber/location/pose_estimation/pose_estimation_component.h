/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： pose_estimation_component.h
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "depend/proto/localization/localization.pb.h"
#include "interface/adsfi_proto/internal/node_info.pb.h"
#include "interface/adsfi_proto/internal/slam_hd_submap.pb.h"
#include "interface/adsfi_proto/perception/lanes.pb.h"
#include "modules/location/pose_estimation/lib/pose_estimation.h"

namespace hozon {
namespace mp {
namespace loc {
enum NodeType { INS = 0, VIO = 1, WO = 2, MM = 3 };

struct Node {
  friend std::ostream &operator<<(std::ostream &of, Node &n) {
    of << "enu : " << n.enu.transpose() << std::endl;
    of << "orientation so3 : " << n.orientation.transpose() << std::endl;
    of << "velocity : " << n.velocity.transpose() << std::endl;
    return of;
  }
  double timestamp = 0.0;
  int index = -1;
  Eigen::Vector3d enu = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d blh = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d orientation = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d velocity = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d b_a = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d b_g = Eigen::Vector3d(0, 0, 0);
  Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Identity();
  NodeType type = NodeType::MM;
  int rtk_status = 4;
  bool active = true;
};

class PoseEstimationComponent : public apollo::cyber::Component<> {
 public:
  PoseEstimationComponent() = default;
  ~PoseEstimationComponent() override;
  bool Init() override;

 private:
  void OnPerception(
      const std::shared_ptr<const ::hozon::perception::TransportElement> &msg);
  void OnIns(
      const std::shared_ptr<const hozon::localization::HafNodeInfo> &msg);
  void pubMmMsg();
  // void OnLocation(const std::shared_ptr<location::HafLocation> &msg);
  // void OnMarkPole(const std::shared_ptr<::perception::Roadmarking> &msg);
  // void setup_mm_localization_estimate(
  //     const hozon::localization::HafNodeInfo &node_info);

 private:
  std::shared_ptr<apollo::cyber::Reader<hozon::localization::HafNodeInfo>>
      ins_reader_;
  std::shared_ptr<apollo::cyber::Reader<::hozon::perception::TransportElement>>
      road_marking_reader_;
  std::shared_ptr<apollo::cyber::Writer<::hozon::localization::HafNodeInfo>>
      node_info_writer_;
  //   std::shared_ptr<apollo::cyber::Reader<::hozon::localization::HafNodeInfo>>
  //       location_reader_;
  std::thread mm_pub_thread_;
  double last_pub_position_x_;
  double last_pub_position_y_;
  double last_pub_position_z_;
  std::string hdmap_topic_;
  std::string location_topic_;
  std::string lane_line_topic_;
  std::string node_info_topic_;
  std::string ins_topic_;
  std::string wo_topic_;
  std::shared_ptr<MapMatching> mm_ = nullptr;
};

CYBER_REGISTER_COMPONENT(PoseEstimationComponent);

}  // namespace loc
}  // namespace mp
}  // namespace hozon
