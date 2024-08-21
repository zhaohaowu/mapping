/***************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_fusion_rviz.h
 *   author     ： cuijiayu
 *   date       ： 2024.01
 ******************************************************************************/

#pragma once

#include <proto/localization/localization.pb.h>
#include <proto/map/map.pb.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/map_fusion_02/base/element_base.h"
#include "modules/map_fusion_02/common/calc_util.h"
#include "modules/map_fusion_02/modules/lane/road_builder/cut_point.h"
#include "modules/rviz/viz_common.h"
#include "modules/util/include/util/rviz_agent/rviz_agent.h"
#include "perception-base/base/utils/macros.h"

namespace hozon {
namespace mp {
namespace mf {

class MapFusionRviz {
 public:
  ~MapFusionRviz() = default;

  bool Init();

  void VizEleMap(const std::shared_ptr<ElementMap>& ele_map);
  void VizPath(const std::vector<KinePosePtr>& path, const KinePose& curr_pose);
  void VizGroup(const std::vector<Group::Ptr>& groups, double stamp);
  void VizGuidePoint(const std::vector<Group::Ptr>& groups, double stamp);
  void VizCutpoint(const std::vector<CutPoint>& cut_points, double stamp);
  void VizDistpoint(const std::vector<Eigen::Vector3f>& distpoints,
                    double stamp);

  std::string Name() const;

 private:
  bool inited_ = false;
  bool map_fusion_group_rviz_ = false;
  bool map_fusion_lane_loc_rviz_ = false;
  std::string viz_topic_input_ele_map_;
  std::string viz_topic_output_ele_map_;
  std::string viz_topic_path_;
  std::string viz_topic_group_;
  std::string viz_topic_guidepoints_;
  std::string viz_topic_cutpoints_;
  std::string viz_topic_distpoints_;
  float viz_lifetime_ = 0;
  DECLARE_SINGLETON_PERCEPTION(MapFusionRviz)
};

#define MF_RVIZ MapFusionRviz::Instance()

}  // namespace mf
}  // namespace mp
}  // namespace hozon
