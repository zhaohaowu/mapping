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
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/map_fusion/base/element_base.h"
#include "modules/map_fusion/common/calc_util.h"
#include "modules/map_fusion/modules/lane/road_builder/cut_point.h"
#include "modules/map_fusion/modules/lane_loc/base_lane_loc.h"
#include "modules/rviz/viz_common.h"
#include "modules/util/include/util/rviz_agent/rviz_agent.h"
#include "perception-base/base/utils/macros.h"

namespace hozon {
namespace mp {
namespace mf {
struct rviz_topo {
 public:
  rviz_topo(const RvizRgb& line_rgbt, const uint32_t& life_sect,
            const uint32_t& life_nsect, const std::string& grp_nst,
            const std::string& lane_nst,
            const adsfi_proto::hz_Adsfi::AlgHeader viz_headert)
      : line_rgb(line_rgbt),
        life_sec(life_sect),
        life_nsec(life_nsect),
        grp_ns(grp_nst),
        lane_ns(lane_nst),
        viz_header(viz_headert) {}

 public:
  /* data */
  RvizRgb line_rgb;
  uint32_t life_sec;
  uint32_t life_nsec;
  std::string grp_ns;
  std::string lane_ns;
  adsfi_proto::hz_Adsfi::AlgHeader viz_header;
};

class MapFusionRviz {
 public:
  ~MapFusionRviz() = default;

  bool Init();

  void VizEleMap(const std::shared_ptr<ElementMap>& ele_map);
  void VizGeoInputEleMap(const std::shared_ptr<ElementMap>& ele_map);
  void VizGeoOutputEleMap(const std::shared_ptr<ElementMap>& ele_map);
  void VizPath(const std::vector<KinePosePtr>& path, const KinePose& curr_pose);
  void VizJunctionStatus(int status, double stamp);
  void VizTopo(const hozon::mp::mf::Lane::Ptr& lane,
               const rviz_topo& tmprviz_topo, double& tmppose,  // NOLINT
               hozon::mp::mf::MarkerArrayPtr& marker_array);    // NOLINT
  void VizGroup(const std::vector<Group::Ptr>& groups, double stamp);
  void SetMarker(::adsfi_proto::viz::Marker* marker, const RvizRgb& color,
                 const double& scale, const uint32_t& life_sec,
                 const uint32_t& life_nsec);
  void VizGuidePoint(const std::vector<Group::Ptr>& groups, double stamp);
  void VizCutpoint(const std::vector<CutPoint>& cut_points, double stamp);
  void VizDistpoint(const std::vector<Eigen::Vector3f>& distpoints,
                    double stamp);
  void VizFitOcc(const std::deque<Line::Ptr>& occ_lines, double stamp);
  // 车道级定位
  void PubInsTf(const Eigen::Affine3d& T_W_V, uint64_t sec, uint64_t nsec,
                const std::string& topic);
  void PubInsPath(const Eigen::Affine3d& T_W_V, uint64_t sec, uint64_t nsec,
                  const std::string& topic);
  void PubInsOdom(const Eigen::Affine3d& T_W_V, uint64_t sec, uint64_t nsec,
                  const std::string& topic);
  void PubPerSection(const lane_loc::Section& section, uint64_t sec,
                     uint64_t nsec, const std::string& topic);
  void PubMapSection(const lane_loc::Section& section, uint64_t sec,
                     uint64_t nsec, const std::string& topic);
  void PubLaneLocInfo(const lane_loc::LaneLocInfo& lane_loc_info, uint64_t sec,
                      uint64_t nsec, const std::string& topic);
  void PubState(const std::string& lane_num, const std::string& road_edge_state,
                const std::string& measure_lane_index,
                const std::vector<double>& p_measure,
                const lane_loc::TurnState& turn_state,
                const std::vector<double>& p_predict,
                const std::string& fusion_lane_index,
                const std::vector<double>& p_fusion, uint64_t sec,
                uint64_t nsec, const std::string& topic);

  std::string Name() const;

 private:
  bool inited_ = false;
  bool map_fusion_group_rviz_ = false;
  bool viz_road_topo_option_ = false;
  std::string viz_topic_input_ele_map_;
  std::string viz_topic_output_ele_map_;
  std::string viz_topic_geo_input_ele_map_;
  std::string viz_topic_geo_output_ele_map_;
  std::string viz_topic_path_;
  std::string viz_topic_group_;
  std::string viz_topic_guidepoints_;
  std::string viz_topic_cutpoints_;
  std::string viz_topic_distpoints_;
  std::string viz_topic_fitocc_;
  std::string viz_topic_junction_status_;
  float viz_lifetime_ = 0;
  DECLARE_SINGLETON_PERCEPTION(MapFusionRviz)
};

#define MF_RVIZ MapFusionRviz::Instance()

}  // namespace mf
}  // namespace mp
}  // namespace hozon
