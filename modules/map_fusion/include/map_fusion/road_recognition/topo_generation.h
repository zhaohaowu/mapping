/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： topo_generation.h
 *   author     ： taoshaoyuan
 *   date       ： 2023.12
 ******************************************************************************/

#pragma once

#include <proto/localization/localization.pb.h>
#include <proto/map/map.pb.h>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "map_fusion/fusion_common/common_data.h"
#include "map_fusion/fusion_common/element_map.h"
#include "map_fusion/road_recognition/base_data.h"
#include "map_fusion/road_recognition/group_map.h"
namespace hozon {
namespace mp {
namespace mf {

class PathManager;

class TopoGeneration {
 public:
  TopoGeneration() = default;
  ~TopoGeneration() = default;
  bool Init(const YAML::Node& conf);
  void OnLocalization(
      const std::shared_ptr<hozon::localization::Localization>& msg);
  void OnElementMap(
      const std::shared_ptr<hozon::mp::mf::em::ElementMap>& ele_map);
  std::shared_ptr<hozon::hdmap::Map> GetPercepMap(
      const std::pair<double, double>& map_speed_limit);
  std::shared_ptr<hozon::mp::mf::em::ElementMapOut> GetEleMap();
  inline void SetRoadScene(RoadScene road_scene) { road_scene_ = road_scene; }

  inline RoadScene GetRoadScene() { return road_scene_; }

  inline void SetPose(const KinePose& curr_pose) { curr_pose_ = curr_pose; }

  inline KinePose GetPose() { return curr_pose_; }

 private:
  void VizEleMap(const std::shared_ptr<hozon::mp::mf::em::ElementMap>& ele_map);
  void VizPath(const std::vector<KinePose::Ptr>& path,
               const KinePose& curr_pose);
  void VizGroup(const std::vector<gm::Group::Ptr>& groups, double stamp);
  void VizGuidePoint(const std::vector<gm::Group::Ptr>& groups, double stamp);
  bool viz_ = false;
  std::string viz_topic_input_ele_map_;
  std::string viz_topic_output_ele_map_;
  std::string viz_topic_path_;
  std::string viz_topic_group_;
  std::string viz_topic_guidepoints_ = "/mapfusion/guide_points";
  double viz_lifetime_ = 0;
  double path_predict_range_ = 0.;
  gm::GroupMapConf gm_conf_;
  gm::IsCross is_cross_;
  gm::HistoryId history_id_;
  KinePose::Ptr last_pose_ = nullptr;

  std::shared_ptr<PathManager> path_ = nullptr;

  std::shared_ptr<gm::GroupMap> group_map_ = nullptr;
  std::shared_ptr<hozon::mp::mf::em::ElementMap> ele_map_ = nullptr;
  std::shared_ptr<hozon::mp::mf::em::ElementMapOut> ele_map_output_ = nullptr;
  bool ego_exist_ = false;
  std::vector<double> line_params_;
  RoadScene road_scene_;
  KinePose curr_pose_;
  bool IsValid(const std::vector<gm::Group::Ptr>& groups);
  void IsInCrossing(const std::vector<gm::Group::Ptr>& groups,
                    gm::IsCross* iscross);
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
