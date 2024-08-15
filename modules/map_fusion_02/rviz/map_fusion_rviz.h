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
#include "modules/map_fusion_02/rviz/viz_common.h"
#include "perception-lib/lib/config_manager/config_manager.h"
#include "modules/util/include/util/rviz_agent/rviz_agent.h"

namespace hozon {
namespace mp {
namespace mf {

class MapFusionRviz {
 public:
  MapFusionRviz() = default;
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
  bool use_rviz_ = true;
  std::string rviz_addr_mfr_ = "ipc:///tmp/rviz_agent_mfr";
  std::string viz_topic_input_ele_map_;
  std::string viz_topic_output_ele_map_;
  std::string viz_topic_path_;
  std::string viz_topic_group_;
  std::string viz_topic_guidepoints_ = "/mapfusion/guide_points";
  std::string viz_topic_cutpoints_ = "/ccmapfusion/cut_points";
  std::string viz_topic_distpoints_ = "/ccmapfusion/dist_points";
  float viz_lifetime_ = 0;
};
using MapFusionRvizPtr = std::unique_ptr<MapFusionRviz>;

}  // namespace mf
}  // namespace mp
}  // namespace hozon
