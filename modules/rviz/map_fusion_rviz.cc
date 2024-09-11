/***************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_fusion_rviz.h
 *   author     ： cuijiayu
 *   date       ： 2024.01
 ******************************************************************************/
#include "modules/rviz/map_fusion_rviz.h"

#include <string>
#include <unordered_map>

#include "base/utils/log.h"
#include "perception-lib/lib/config_manager/config_manager.h"

namespace hozon {
namespace mp {
namespace mf {

std::string MapFusionRviz::Name() const { return "MapFusionRviz"; }

// init没有放在构造函数中,因为init用到了其他单例,会有初始化顺序问题
MapFusionRviz::MapFusionRviz() = default;

bool MapFusionRviz::Init() {
  if (inited_) {
    return true;
  }
  auto* config_manager = hozon::perception::lib::ConfigManager::Instance();
  const hozon::perception::lib::ModelConfig* model_config = nullptr;
  if (!config_manager->GetModelConfig(Name(), &model_config)) {
    HLOG_ERROR << "Parse config failed! Name: " << Name();
    return false;
  }

  if (!model_config->get_value("map_fusion_group_rviz",
                               &map_fusion_group_rviz_)) {
    HLOG_ERROR << "Get map_fusion_group_rviz viz failed!";
    return false;
  }

  if (!model_config->get_value("viz_topic_input_ele_map",
                               &viz_topic_input_ele_map_)) {
    HLOG_ERROR << "Get viz_topic_input_ele_map_ failed!";
    return false;
  }

  if (!model_config->get_value("viz_topic_output_ele_map",
                               &viz_topic_output_ele_map_)) {
    HLOG_ERROR << "Get viz_topic_output_ele_map failed!";
    return false;
  }
  if (!model_config->get_value("viz_topic_geo_input_ele_map",
                               &viz_topic_geo_input_ele_map_)) {
    HLOG_ERROR << "Get viz_topic_geo_input_ele_map failed!";
    return false;
  }

  if (!model_config->get_value("viz_topic_geo_output_ele_map",
                               &viz_topic_geo_output_ele_map_)) {
    HLOG_ERROR << "Get viz_topic_geo_output_ele_map failed!";
    return false;
  }
  if (!model_config->get_value("viz_topic_path", &viz_topic_path_)) {
    HLOG_ERROR << "Get viz_topic_path failed!";
    return false;
  }
  if (!model_config->get_value("viz_topic_group", &viz_topic_group_)) {
    HLOG_ERROR << "Get viz_topic_group failed!";
    return false;
  }
  if (!model_config->get_value("viz_topic_guidepoints",
                               &viz_topic_guidepoints_)) {
    HLOG_ERROR << "Get viz_topic_guidepoints failed!";
    return false;
  }
  if (!model_config->get_value("viz_topic_cutpoints", &viz_topic_cutpoints_)) {
    HLOG_ERROR << "Get viz_topic_cutpoints failed!";
    return false;
  }
  if (!model_config->get_value("viz_topic_distpoints",
                               &viz_topic_distpoints_)) {
    HLOG_ERROR << "Get viz_topic_distpoints failed!";
    return false;
  }
  if (!model_config->get_value("viz_topic_junction_status",
                               &viz_topic_junction_status_)) {
    HLOG_ERROR << "Get viz_topic_junction_status failed!";
    return false;
  }
  if (!model_config->get_value("viz_lifetime", &viz_lifetime_)) {
    HLOG_ERROR << "Get viz_lifetime failed!";
    return false;
  }
  if (!model_config->get_value("viz_topic_junction_status",
                               &viz_topic_junction_status_)) {
    HLOG_ERROR << "Get viz_topic_junction_status failed!";
    return false;
  }

  std::vector<std::string> marker_topics = {viz_topic_input_ele_map_,
                                            viz_topic_output_ele_map_,
                                            viz_topic_group_,
                                            viz_topic_geo_input_ele_map_,
                                            viz_topic_geo_output_ele_map_,
                                            viz_topic_junction_status_};
  int ret = RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(marker_topics);
  if (ret < 0) {
    HLOG_FATAL << "RvizAgent register marker array failed";
    return false;
  }

  std::vector<std::string> pose_array_topics = {
      viz_topic_path_,
  };
  ret = RVIZ_AGENT.Register<adsfi_proto::viz::PoseArray>(pose_array_topics);
  if (ret < 0) {
    HLOG_FATAL << "RvizAgent register pose array failed";
    return false;
  }

  std::vector<std::string> guide_points_topic = {
      viz_topic_guidepoints_,
  };
  ret =
      RVIZ_AGENT.Register<adsfi_proto::viz::PointCloud>(viz_topic_guidepoints_);
  if (ret < 0) {
    HLOG_FATAL << "RvizAgent register guide point failed";
    return false;
  }

  std::vector<std::string> cut_points_topic = {
      viz_topic_cutpoints_,
  };
  ret = RVIZ_AGENT.Register<adsfi_proto::viz::PointCloud>(cut_points_topic);
  if (ret < 0) {
    HLOG_FATAL << "RvizAgent register cut point failed";
    return false;
  }

  std::vector<std::string> dist_points_topic = {
      viz_topic_distpoints_,
  };
  ret = RVIZ_AGENT.Register<adsfi_proto::viz::PointCloud>(dist_points_topic);
  if (ret < 0) {
    HLOG_FATAL << "RvizAgent register cut point failed";
    return false;
  }
  inited_ = true;
  return true;
}

template <typename T0, typename T1>
static void SetXYZ(const T0& t0, T1* const t1) {
  t1->set_x(t0.x());
  t1->set_y(t0.y());
  t1->set_z(t0.z());
}

template <typename T0, typename T1>
static void SetXYZW(const T0& t0, T1* const t1) {
  t1->set_w(t0.w());
  t1->set_x(t0.x());
  t1->set_y(t0.y());
  t1->set_z(t0.z());
}

void MapFusionRviz::VizEleMap(const std::shared_ptr<ElementMap>& ele_map) {
  if (!inited_ || !map_fusion_group_rviz_ || !RVIZ_AGENT.Ok()) {
    return;
  }

  auto m = ElementMapToMarkers(*ele_map, "vehicle", "topo_gen",
                               viz_topic_input_ele_map_,
                               ele_map->map_info.stamp, viz_lifetime_);
  if (m != nullptr) {
    RVIZ_AGENT.Publish(viz_topic_input_ele_map_, m);
  }
}

void MapFusionRviz::VizGeoInputEleMap(
    const std::shared_ptr<ElementMap>& ele_map) {
  if (!inited_ || !map_fusion_group_rviz_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  auto m = ElementMapToMarkers(*ele_map, "vehicle", "geo",
                               viz_topic_geo_input_ele_map_,
                               ele_map->map_info.stamp, viz_lifetime_);
  if (m != nullptr) {
    RVIZ_AGENT.Publish(viz_topic_geo_input_ele_map_, m);
  }
}

void MapFusionRviz::VizGeoOutputEleMap(
    const std::shared_ptr<ElementMap>& ele_map) {
  if (!inited_ || !map_fusion_group_rviz_ || !RVIZ_AGENT.Ok()) {
    return;
  }

  auto m = ElementMapToMarkers(*ele_map, "vehicle", "geo",
                               viz_topic_geo_output_ele_map_,
                               ele_map->map_info.stamp, viz_lifetime_);
  if (m != nullptr) {
    RVIZ_AGENT.Publish(viz_topic_geo_output_ele_map_, m);
  }
}

void MapFusionRviz::VizPath(const std::vector<KinePosePtr>& path,
                            const KinePose& curr_pose) {
  if (!inited_ || !map_fusion_group_rviz_ || !RVIZ_AGENT.Ok()) {
    return;
  }

  auto curr_local_to_veh = curr_pose.Inverse();
  std::vector<Pose> poses;
  for (const auto& p : path) {
    if (p == nullptr) {
      HLOG_ERROR << "found nullptr pose";
      continue;
    }
    Eigen::Isometry3f T_local_to_curr_veh;
    T_local_to_curr_veh.setIdentity();
    T_local_to_curr_veh.rotate(curr_local_to_veh.quat);
    T_local_to_curr_veh.pretranslate(curr_local_to_veh.pos);

    Eigen::Isometry3f T_veh_to_local;
    T_veh_to_local.setIdentity();
    T_veh_to_local.rotate(p->quat);
    T_veh_to_local.pretranslate(p->pos);

    Eigen::Isometry3f T_veh_to_curr_veh;
    T_veh_to_curr_veh.setIdentity();
    T_veh_to_curr_veh = T_local_to_curr_veh * T_veh_to_local;
    Pose cp;
    cp.pos = T_veh_to_curr_veh.translation();
    cp.quat = T_veh_to_curr_veh.rotation();
    poses.emplace_back(cp);
  }

  auto pose_array = PosesToPoseArray(poses, "vehicle", curr_pose.stamp);
  if (pose_array != nullptr) {
    RVIZ_AGENT.Publish(viz_topic_path_, pose_array);
  }
}

void MapFusionRviz::VizJunctionStatus(int status, double stamp) {
  if (!inited_ || !map_fusion_group_rviz_ || !RVIZ_AGENT.Ok()) {
    return;
  }

  adsfi_proto::hz_Adsfi::AlgHeader viz_header;
  viz_header.set_frameid("vehicle");
  uint32_t sec = 0;
  uint32_t nsec = 0;
  SplitSeconds(stamp, &sec, &nsec);
  viz_header.mutable_timestamp()->set_sec(sec);
  viz_header.mutable_timestamp()->set_nsec(nsec);
  uint32_t life_sec = 0;
  uint32_t life_nsec = 0;
  SplitSeconds(viz_lifetime_, &life_sec, &life_nsec);

  auto marker_array = std::make_shared<adsfi_proto::viz::MarkerArray>();
  auto* marker_junction_status = marker_array->add_markers();
  marker_junction_status->mutable_header()->CopyFrom(viz_header);
  marker_junction_status->set_ns("/junction_status");
  marker_junction_status->set_id(0);
  marker_junction_status->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker_junction_status->mutable_scale()->set_z(1.0);
  marker_junction_status->mutable_lifetime()->set_sec(life_sec);
  marker_junction_status->mutable_lifetime()->set_nsec(life_nsec);
  marker_junction_status->mutable_color()->set_a(1.0);

  RvizRgb junc_rgb = ColorRgb(RvizColor::R_RED);
  marker_junction_status->mutable_color()->set_r(junc_rgb.r);
  marker_junction_status->mutable_color()->set_g(junc_rgb.g);
  marker_junction_status->mutable_color()->set_b(junc_rgb.b);

  marker_junction_status->set_type(
      adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
  marker_junction_status->mutable_pose()->mutable_orientation()->set_w(1);
  marker_junction_status->mutable_pose()->mutable_orientation()->set_x(0);
  marker_junction_status->mutable_pose()->mutable_orientation()->set_y(0);
  marker_junction_status->mutable_pose()->mutable_orientation()->set_z(0);
  marker_junction_status->mutable_pose()->mutable_position()->set_x(-2.0);
  marker_junction_status->mutable_pose()->mutable_position()->set_y(10.0);
  marker_junction_status->mutable_pose()->mutable_position()->set_z(0.0);
  auto* junc_text = marker_junction_status->mutable_text();
  *junc_text = "junction_status: " + std::to_string(status);

  if (!marker_array->markers().empty()) {
    RVIZ_AGENT.Publish(viz_topic_junction_status_, marker_array);
  }
}

void MapFusionRviz::VizGroup(const std::vector<Group::Ptr>& groups,
                             double stamp) {
  if (!inited_ || !map_fusion_group_rviz_ || !RVIZ_AGENT.Ok()) {
    return;
  }

  adsfi_proto::hz_Adsfi::AlgHeader viz_header;
  viz_header.set_frameid("vehicle");
  uint32_t sec = 0;
  uint32_t nsec = 0;
  SplitSeconds(stamp, &sec, &nsec);
  viz_header.mutable_timestamp()->set_sec(sec);
  viz_header.mutable_timestamp()->set_nsec(nsec);
  uint32_t life_sec = 0;
  uint32_t life_nsec = 0;
  SplitSeconds(viz_lifetime_, &life_sec, &life_nsec);

  auto marker_array = std::make_shared<adsfi_proto::viz::MarkerArray>();
  std::vector<RvizColor> line_colors = {RvizColor::R_ORANGE,
                                        RvizColor::R_YELLOW, RvizColor::R_GREEN,
                                        RvizColor::R_CYAN};
  int grp_idx = -1;
  for (const auto& grp : groups) {
    grp_idx += 1;
    std::string grp_ns =
        "grp" + std::to_string(grp_idx) + "(" + grp->str_id + ")";
    if (grp == nullptr) {
      // HLOG_ERROR << "found nullptr group";
      continue;
    }

    RvizRgb line_rgb = ColorRgb(line_colors.at(grp_idx % line_colors.size()));

    if (grp->group_state != Group::GroupState::VIRTUAL) {
      // start_slice
      auto* marker_start_slice = marker_array->add_markers();
      marker_start_slice->mutable_header()->CopyFrom(viz_header);
      marker_start_slice->set_ns(grp_ns + "/slice");
      marker_start_slice->set_id(0);
      marker_start_slice->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
      marker_start_slice->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
      marker_start_slice->mutable_color()->set_a(0.1);
      double width = 1;
      SetMarker(marker_start_slice, line_rgb, width, life_sec, life_nsec);

      double scale = 1;
      Eigen::Vector3f left =
          (grp->start_slice.pl - grp->start_slice.po) * scale +
          grp->start_slice.po;
      Eigen::Vector3f center = grp->start_slice.po;
      Eigen::Vector3f right =
          (grp->start_slice.pr - grp->start_slice.po) * scale +
          grp->start_slice.po;
      std::vector<Eigen::Vector3f> pts = {left, center, right};
      for (const auto& p : pts) {
        auto* pt = marker_start_slice->add_points();
        pt->set_x(p.x());
        pt->set_y(p.y());
        pt->set_z(p.z());
      }
      // start slice cut type
      auto* marker_start_slice_type = marker_array->add_markers();
      marker_start_slice_type->mutable_header()->CopyFrom(viz_header);
      marker_start_slice_type->set_ns(grp_ns + "/slice_cut");
      marker_start_slice_type->set_id(0);
      marker_start_slice_type->set_action(
          adsfi_proto::viz::MarkerAction::MODIFY);
      marker_start_slice_type->set_type(
          adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
      marker_start_slice_type->mutable_color()->set_a(1.0);
      double text_size = 0.5;
      SetMarker(marker_start_slice_type, line_rgb, text_size, life_sec,
                life_nsec);

      marker_start_slice_type->mutable_pose()->mutable_position()->set_x(
          grp->start_slice.po.x() - 2);
      marker_start_slice_type->mutable_pose()->mutable_position()->set_y(
          grp->start_slice.po.y() - 1);
      marker_start_slice_type->mutable_pose()->mutable_position()->set_z(
          grp->start_slice.po.z());
      auto* text_start = marker_start_slice_type->mutable_text();
      *text_start = "cut_type: " + std::to_string(grp->start_slice.cut_type);

      // end_slice
      auto* marker_end_slice = marker_array->add_markers();
      marker_end_slice->CopyFrom(*marker_start_slice);
      marker_end_slice->set_id(1);
      marker_end_slice->clear_points();
      left =
          (grp->end_slice.pl - grp->end_slice.po) * scale + grp->end_slice.po;
      center = grp->end_slice.po;
      right =
          (grp->end_slice.pr - grp->end_slice.po) * scale + grp->end_slice.po;
      pts = std::vector<Eigen::Vector3f>{left, center, right};
      for (const auto& p : pts) {
        auto* pt = marker_end_slice->add_points();
        pt->set_x(p.x());
        pt->set_y(p.y());
        pt->set_z(p.z());
      }
      // end slice cut type
      auto* marker_end_slice_type = marker_array->add_markers();
      marker_end_slice_type->CopyFrom(*marker_start_slice_type);
      marker_end_slice_type->set_id(1);
      marker_end_slice_type->mutable_pose()->mutable_position()->set_x(
          grp->end_slice.po.x() - 2);
      marker_end_slice_type->mutable_pose()->mutable_position()->set_y(
          grp->end_slice.po.y() - 1);
      marker_end_slice_type->mutable_pose()->mutable_position()->set_z(
          grp->end_slice.po.z());
      auto* text_end = marker_end_slice_type->mutable_text();
      *text_end = "cut_type: " + std::to_string(grp->end_slice.cut_type);
    }

    // lanes
    if (!grp->lanes.empty()) {
      int lane_idx = -1;
      for (const auto& lane : grp->lanes) {
        lane_idx += 1;
        std::string lane_ns =
            "lane" + std::to_string(lane_idx) + "(" + lane->str_id + ")";
        // left_boundary
        auto* marker_left_bound = marker_array->add_markers();
        marker_left_bound->mutable_header()->CopyFrom(viz_header);
        marker_left_bound->set_ns(grp_ns + "/" + lane_ns);
        marker_left_bound->set_id(0);
        marker_left_bound->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
        marker_left_bound->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
        marker_left_bound->mutable_color()->set_a(1.0);
        double width = 0.2;
        SetMarker(marker_left_bound, line_rgb, width, life_sec, life_nsec);

        // right_boundary
        auto* marker_right_bound = marker_array->add_markers();
        marker_right_bound->CopyFrom(*marker_left_bound);
        marker_right_bound->set_id(1);

        // center_line_pts
        auto* marker_center_line = marker_array->add_markers();
        marker_center_line->CopyFrom(*marker_left_bound);
        marker_center_line->set_id(2);
        marker_center_line->mutable_scale()->set_x(width / 2);
        marker_center_line->mutable_color()->set_a(0.2);

        std::vector<Eigen::Vector3f> left_pred_pts;
        std::vector<Eigen::Vector3f> right_pred_pts;
        // 在同一根lane可能前端和后端都有虚拟车道，要区分开来
        std::vector<Eigen::Vector3f> left_virtual_pts;
        std::vector<Eigen::Vector3f> right_virtual_pts;
        std::vector<Eigen::Vector3f> left_virtual_pts_back;
        std::vector<Eigen::Vector3f> right_virtual_pts_back;
        adsfi_proto::viz::Marker temp_marker_left_pred;
        adsfi_proto::viz::Marker temp_marker_right_pred;
        adsfi_proto::viz::Marker temp_marker_left_virt;
        adsfi_proto::viz::Marker temp_marker_right_virt;
        adsfi_proto::viz::Marker temp_marker_left_virt_back;
        adsfi_proto::viz::Marker temp_marker_right_virt_back;
        temp_marker_left_pred.CopyFrom(*marker_left_bound);
        temp_marker_left_pred.set_id(3);
        temp_marker_left_pred.mutable_color()->set_a(0.2);
        temp_marker_right_pred.CopyFrom(*marker_right_bound);
        temp_marker_right_pred.set_id(4);
        temp_marker_right_pred.mutable_color()->set_a(0.2);
        temp_marker_left_virt.CopyFrom(*marker_left_bound);
        temp_marker_left_virt.set_id(5);
        temp_marker_left_virt.mutable_color()->set_a(0.5);
        temp_marker_right_virt.CopyFrom(*marker_right_bound);
        temp_marker_right_virt.set_id(6);
        temp_marker_right_virt.mutable_color()->set_a(0.5);
        temp_marker_left_virt_back.CopyFrom(*marker_left_bound);
        temp_marker_left_virt_back.set_id(7);
        temp_marker_left_virt_back.mutable_color()->set_a(0.5);
        temp_marker_right_virt_back.CopyFrom(*marker_right_bound);
        temp_marker_right_virt_back.set_id(8);
        temp_marker_right_virt_back.mutable_color()->set_a(0.5);
        int flag = 0;
        for (const auto& p : lane->left_boundary->pts) {
          if (p.type == PREDICTED) {
            left_pred_pts.emplace_back(p.pt);
          } else if (p.type == VIRTUAL) {
            if (flag == 0) {
              left_virtual_pts.emplace_back(p.pt);
            } else {
              left_virtual_pts_back.emplace_back(p.pt);
            }
          } else {
            auto* pt = marker_left_bound->add_points();
            pt->set_x(p.pt.x());
            pt->set_y(p.pt.y());
            pt->set_z(p.pt.z());
            flag = 1;
          }
        }
        flag = 0;
        for (const auto& p : lane->right_boundary->pts) {
          if (p.type == PREDICTED) {
            right_pred_pts.emplace_back(p.pt);
          } else if (p.type == VIRTUAL) {
            if (flag == 0) {
              right_virtual_pts.emplace_back(p.pt);
            } else {
              right_virtual_pts_back.emplace_back(p.pt);
            }
          } else {
            auto* pt = marker_right_bound->add_points();
            pt->set_x(p.pt.x());
            pt->set_y(p.pt.y());
            pt->set_z(p.pt.z());
            flag = 1;
          }
        }
        for (const auto& p : lane->center_line_pts) {
          auto* pt = marker_center_line->add_points();
          pt->set_x(p.pt.x());
          pt->set_y(p.pt.y());
          pt->set_z(p.pt.z());
        }

        if (left_pred_pts.size() > 1) {
          auto* marker_left_pred = marker_array->add_markers();
          marker_left_pred->CopyFrom(temp_marker_left_pred);
          if (!marker_left_bound->points().empty()) {
            auto* pt = marker_left_pred->add_points();
            pt->CopyFrom(*marker_left_bound->points().rbegin());
          }
          for (const auto& p : left_pred_pts) {
            auto* pt = marker_left_pred->add_points();
            pt->set_x(p.x());
            pt->set_y(p.y());
            pt->set_z(p.z());
          }
        }
        if (right_pred_pts.size() > 1) {
          auto* marker_right_pred = marker_array->add_markers();
          marker_right_pred->CopyFrom(temp_marker_right_pred);
          if (!marker_right_bound->points().empty()) {
            auto* pt = marker_right_pred->add_points();
            pt->CopyFrom(*marker_right_bound->points().rbegin());
          }
          for (const auto& p : right_pred_pts) {
            auto* pt = marker_right_pred->add_points();
            pt->set_x(p.x());
            pt->set_y(p.y());
            pt->set_z(p.z());
          }
        }
        if (left_virtual_pts.size() > 1) {
          auto* marker_left_virt = marker_array->add_markers();
          marker_left_virt->CopyFrom(temp_marker_left_virt);
          for (const auto& p : left_virtual_pts) {
            auto* pt = marker_left_virt->add_points();
            pt->set_x(p.x());
            pt->set_y(p.y());
            pt->set_z(p.z());
          }
        }
        if (right_virtual_pts.size() > 1) {
          auto* marker_right_virt = marker_array->add_markers();
          marker_right_virt->CopyFrom(temp_marker_right_virt);
          for (const auto& p : right_virtual_pts) {
            auto* pt = marker_right_virt->add_points();
            pt->set_x(p.x());
            pt->set_y(p.y());
            pt->set_z(p.z());
          }
        }
        if (left_virtual_pts_back.size() > 1) {
          auto* marker_left_virt = marker_array->add_markers();
          marker_left_virt->CopyFrom(temp_marker_left_virt_back);
          for (const auto& p : left_virtual_pts_back) {
            auto* pt = marker_left_virt->add_points();
            pt->set_x(p.x());
            pt->set_y(p.y());
            pt->set_z(p.z());
          }
        }
        if (right_virtual_pts_back.size() > 1) {
          auto* marker_right_virt = marker_array->add_markers();
          marker_right_virt->CopyFrom(temp_marker_right_virt_back);
          for (const auto& p : right_virtual_pts_back) {
            auto* pt = marker_right_virt->add_points();
            pt->set_x(p.x());
            pt->set_y(p.y());
            pt->set_z(p.z());
          }
        }

        auto* marker_str_id_with_group = marker_array->add_markers();
        marker_str_id_with_group->mutable_header()->CopyFrom(viz_header);
        marker_str_id_with_group->set_ns(grp_ns + "/" + lane_ns);
        marker_str_id_with_group->set_id(9);
        marker_str_id_with_group->set_action(
            adsfi_proto::viz::MarkerAction::MODIFY);
        marker_str_id_with_group->set_type(
            adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
        marker_str_id_with_group->mutable_color()->set_a(0.2);
        double text_size = 0.5;
        SetMarker(marker_str_id_with_group, line_rgb, text_size, life_sec,
                  life_nsec);

        marker_str_id_with_group->mutable_pose()->mutable_position()->set_x(
            lane->center_line_pts.front().pt.x());
        marker_str_id_with_group->mutable_pose()->mutable_position()->set_y(
            lane->center_line_pts.front().pt.y());
        marker_str_id_with_group->mutable_pose()->mutable_position()->set_z(
            lane->center_line_pts.front().pt.z());
        auto* text = marker_str_id_with_group->mutable_text();
        *text = lane->str_id_with_group + " broken id " +
                std::to_string(grp->broken_id);
        if (!lane->next_lane_str_id_with_group.empty() &&
            lane->center_line_pts.size() > 2) {
          marker_str_id_with_group = marker_array->add_markers();
          marker_str_id_with_group->mutable_header()->CopyFrom(viz_header);
          marker_str_id_with_group->set_ns(grp_ns + "/" + lane_ns + "/next");
          marker_str_id_with_group->set_id(9);
          marker_str_id_with_group->set_action(
              adsfi_proto::viz::MarkerAction::MODIFY);
          marker_str_id_with_group->set_type(
              adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
          marker_str_id_with_group->mutable_color()->set_a(0.2);
          double text_size = 0.5;
          SetMarker(marker_str_id_with_group, line_rgb, text_size, life_sec,
                    life_nsec);

          marker_str_id_with_group->mutable_pose()->mutable_position()->set_x(
              lane->center_line_pts[lane->center_line_pts.size() - 3].pt.x());
          marker_str_id_with_group->mutable_pose()->mutable_position()->set_y(
              lane->center_line_pts[lane->center_line_pts.size() - 3].pt.y());
          marker_str_id_with_group->mutable_pose()->mutable_position()->set_z(
              lane->center_line_pts[lane->center_line_pts.size() - 3].pt.z());
          text = marker_str_id_with_group->mutable_text();
          for (int i = 0; i < lane->next_lane_str_id_with_group.size(); ++i) {
            *text = *text + lane->next_lane_str_id_with_group[i] + "  ";
          }
        }
      }
    } else {
      auto* marker_str_id_with_group = marker_array->add_markers();
      marker_str_id_with_group->mutable_header()->CopyFrom(viz_header);
      marker_str_id_with_group->set_ns(grp_ns + "/no_lane");
      marker_str_id_with_group->set_id(0);
      marker_str_id_with_group->set_action(
          adsfi_proto::viz::MarkerAction::MODIFY);
      marker_str_id_with_group->set_type(
          adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
      marker_str_id_with_group->mutable_color()->set_a(0.2);
      double text_size = 0.5;
      SetMarker(marker_str_id_with_group, line_rgb, text_size, life_sec,
                life_nsec);

      if (!grp->line_segments.empty()) {
        int idx = static_cast<int>(grp->line_segments.front()->pts.size() / 2);
        marker_str_id_with_group->mutable_pose()->mutable_position()->set_x(
            grp->line_segments.front()->pts[idx].pt.x());
        marker_str_id_with_group->mutable_pose()->mutable_position()->set_y(
            grp->line_segments.front()->pts[idx].pt.y() - 1);
        marker_str_id_with_group->mutable_pose()->mutable_position()->set_z(
            grp->line_segments.front()->pts[idx].pt.z());
        auto* text = marker_str_id_with_group->mutable_text();
        *text = grp->str_id + " broken id " + std::to_string(grp->broken_id);
      }
    }

    // road edges
    if (!grp->road_edges.empty()) {
      // HLOG_INFO << "road edges size: " << grp->road_edges.size();

      int edge_id = -1;
      for (const auto& road_edge : grp->road_edges) {
        if (road_edge == nullptr) {
          continue;
        }
        if (road_edge->points.empty()) {
          continue;
        }
        // HLOG_INFO << "road edge points size: " << road_edge->points.size();

        edge_id += 1;
        std::string edge_ns = "road_edge_" + std::to_string(edge_id);
        auto* marker_edge_text = marker_array->add_markers();

        // text
        marker_edge_text->mutable_header()->CopyFrom(viz_header);
        marker_edge_text->set_ns(grp_ns + "/" + edge_ns);
        marker_edge_text->set_id(edge_id++);
        marker_edge_text->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
        marker_edge_text->set_type(
            adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
        marker_edge_text->mutable_color()->set_a(1.0);
        double text_size = 1.0;
        if (road_edge->road_edge_type == LINE) {
          RvizRgb line_edge_rgb = ColorRgb(RvizColor::R_GREY);
          SetMarker(marker_edge_text, line_edge_rgb, text_size, life_sec,
                    life_nsec);
        } else if (road_edge->road_edge_type == OCC) {
          RvizRgb occ_edge_rgb = ColorRgb(RvizColor::R_PURPLE);
          SetMarker(marker_edge_text, occ_edge_rgb, text_size, life_sec,
                    life_nsec);
        } else if (road_edge->road_edge_type == MODEL) {
          RvizRgb model_edge_rgb = ColorRgb(RvizColor::R_RED);
          SetMarker(marker_edge_text, model_edge_rgb, text_size, life_sec,
                    life_nsec);
        }

        marker_edge_text->mutable_pose()->mutable_position()->set_x(
            road_edge->points[0].x());
        if (road_edge->is_left) {
          marker_edge_text->mutable_pose()->mutable_position()->set_y(
              road_edge->points[0].y() + 2);
        }
        if (road_edge->is_right) {
          marker_edge_text->mutable_pose()->mutable_position()->set_y(
              road_edge->points[0].y() - 2);
        }
        marker_edge_text->mutable_pose()->mutable_position()->set_z(
            road_edge->points[0].z());
        auto* text = marker_edge_text->mutable_text();
        *text = "edge \n id: " + std::to_string(road_edge->id) +
                "\n left: " + std::to_string(road_edge->is_left) +
                "\n right: " + std::to_string(road_edge->is_right) +
                "\n type: " + std::to_string(road_edge->road_edge_type);

        // points
        auto* marker_road_edge = marker_array->add_markers();
        marker_road_edge->mutable_header()->CopyFrom(viz_header);
        marker_road_edge->set_ns(grp_ns + "/" + edge_ns);
        marker_road_edge->set_id(edge_id++);
        marker_road_edge->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
        marker_road_edge->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
        marker_road_edge->mutable_color()->set_a(1.0);
        double width = 0.2;
        if (road_edge->road_edge_type == LINE) {
          RvizRgb line_edge_rgb = ColorRgb(RvizColor::R_GREY);
          SetMarker(marker_road_edge, line_edge_rgb, width, life_sec,
                    life_nsec);
        } else if (road_edge->road_edge_type == OCC) {
          RvizRgb occ_edge_rgb = ColorRgb(RvizColor::R_PURPLE);
          SetMarker(marker_road_edge, occ_edge_rgb, width, life_sec, life_nsec);
        } else if (road_edge->road_edge_type == MODEL) {
          RvizRgb model_edge_rgb = ColorRgb(RvizColor::R_RED);
          SetMarker(marker_road_edge, model_edge_rgb, width, life_sec,
                    life_nsec);
        }
        for (const auto& p : road_edge->points) {
          auto* pt = marker_road_edge->add_points();
          pt->set_x(p.x());
          pt->set_y(p.y());
          pt->set_z(p.z());
        }
      }
    }
  }
  if (!marker_array->markers().empty()) {
    RVIZ_AGENT.Publish(viz_topic_group_, marker_array);
  }
}

void MapFusionRviz::SetMarker(::adsfi_proto::viz::Marker* marker,
                              const RvizRgb& color, const double& scale,
                              const uint32_t& life_sec,
                              const uint32_t& life_nsec) {
  marker->mutable_pose()->mutable_orientation()->set_w(1);
  marker->mutable_pose()->mutable_orientation()->set_x(0);
  marker->mutable_pose()->mutable_orientation()->set_y(0);
  marker->mutable_pose()->mutable_orientation()->set_z(0);
  marker->mutable_color()->set_r(color.r);
  marker->mutable_color()->set_g(color.g);
  marker->mutable_color()->set_b(color.b);
  marker->mutable_scale()->set_x(scale);
  marker->mutable_scale()->set_y(scale);
  marker->mutable_scale()->set_z(scale);
  marker->mutable_lifetime()->set_sec(life_sec);
  marker->mutable_lifetime()->set_nsec(life_nsec);
}

void MapFusionRviz::VizGuidePoint(const std::vector<Group::Ptr>& groups,
                                  double stamp) {
  if (!inited_ || !map_fusion_group_rviz_ || !RVIZ_AGENT.Ok()) {
    return;
  }

  adsfi_proto::viz::PointCloud points_msg;
  // adsfi_proto::hz_Adsfi::AlgHeader viz_header;
  // viz_header.set_frameid("vehicle");
  // points_msg.mutable_header(CopyFrom(viz_header));
  uint32_t sec = 0;
  uint32_t nsec = 0;
  SplitSeconds(stamp, &sec, &nsec);
  points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
  points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  points_msg.mutable_header()->set_frameid("vehicle");
  auto* channels = points_msg.add_channels();
  channels->set_name("rgb");

  // // 增加引导点的可视化
  if (groups.empty()) {
    return;
  }
  auto guide_points = groups.front()->guide_points_toviz;
  for (const auto& p : guide_points) {
    auto* point_msg = points_msg.add_points();
    point_msg->set_x(p.pt.x());
    point_msg->set_y(p.pt.y());
    point_msg->set_z(p.pt.z());
  }

  RVIZ_AGENT.Publish(viz_topic_guidepoints_, points_msg);
}

void MapFusionRviz::VizCutpoint(const std::vector<mp::mf::CutPoint>& cut_points,
                                double stamp) {
  if (!inited_ || !map_fusion_group_rviz_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  // HLOG_ERROR << "viz cutpoint size " << cut_points.size();
  adsfi_proto::viz::PointCloud points_msg;
  uint32_t sec = 0;
  uint32_t nsec = 0;
  SplitSeconds(stamp, &sec, &nsec);
  points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
  points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  points_msg.mutable_header()->set_frameid("vehicle");
  auto* channels = points_msg.add_channels();
  channels->set_name("rgb");

  // // 增加切分点的可视化
  if (cut_points.empty()) {
    return;
  }
  for (const auto& p : cut_points) {
    auto* point_msg = points_msg.add_points();
    point_msg->set_x(p.GetPoint().x);
    point_msg->set_y(p.GetPoint().y);
    point_msg->set_z(p.GetPoint().z);
  }

  RVIZ_AGENT.Publish(viz_topic_cutpoints_, points_msg);
}

void MapFusionRviz::VizDistpoint(const std::vector<Eigen::Vector3f>& distpoints,
                                 double stamp) {
  if (!inited_ || !map_fusion_group_rviz_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  std::vector<Eigen::Vector3f> distpoints_t = distpoints;
  // HLOG_ERROR << "viz dist point size " << distpoints.size();
  adsfi_proto::viz::PointCloud points_msg;
  uint32_t sec = 0;
  uint32_t nsec = 0;
  SplitSeconds(stamp, &sec, &nsec);
  points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
  points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  points_msg.mutable_header()->set_frameid("vehicle");
  auto* channels = points_msg.add_channels();
  channels->set_name("rgb");

  // // 增加切分点的可视化
  if (distpoints_t.empty()) {
    return;
  }
  for (const auto& p : distpoints_t) {
    auto* point_msg = points_msg.add_points();
    point_msg->set_x(static_cast<float>(p.x()));
    point_msg->set_y(static_cast<float>(p.y()));
    point_msg->set_z(static_cast<float>(p.z()));
    // HLOG_ERROR << "viz dist dp: " << p.x() << " " << p.y();
  }

  RVIZ_AGENT.Publish(viz_topic_distpoints_, points_msg);
  distpoints_t.clear();
}

void MapFusionRviz::PubInsTf(const Eigen::Affine3d& T_W_V, uint64_t sec,
                             uint64_t nsec, const std::string& topic) {
  if (!inited_ || !map_fusion_group_rviz_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  Eigen::Vector3d p = T_W_V.translation();
  Eigen::Quaterniond q(T_W_V.rotation());
  static bool register_flag = true;
  if (register_flag) {
    RVIZ_AGENT.Register<adsfi_proto::viz::TransformStamped>(topic);
    register_flag = false;
  }
  int seq = 0;
  adsfi_proto::viz::TransformStamped tf_msg;
  tf_msg.mutable_header()->set_seq(seq++);
  tf_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
  tf_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  tf_msg.mutable_header()->set_frameid("map");
  tf_msg.set_child_frame_id("vehicle");
  SetXYZ(p, tf_msg.mutable_transform()->mutable_translation());
  SetXYZW(q, tf_msg.mutable_transform()->mutable_rotation());
  RVIZ_AGENT.Publish(topic, tf_msg);
}

void MapFusionRviz::PubInsPath(const Eigen::Affine3d& T_W_V, uint64_t sec,
                               uint64_t nsec, const std::string& topic) {
  if (!inited_ || !map_fusion_group_rviz_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  Eigen::Vector3d p = T_W_V.translation();
  Eigen::Quaterniond q(T_W_V.rotation());
  static bool register_flag = true;
  if (register_flag) {
    RVIZ_AGENT.Register<adsfi_proto::viz::Path>(topic);
    register_flag = false;
  }
  static adsfi_proto::viz::Path path_msg;
  auto* pose = path_msg.add_poses();
  path_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
  path_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  path_msg.mutable_header()->set_frameid("map");
  pose->mutable_header()->mutable_timestamp()->set_sec(sec);
  pose->mutable_header()->mutable_timestamp()->set_nsec(nsec);
  pose->mutable_header()->set_frameid("map");
  SetXYZ(p, pose->mutable_pose()->mutable_position());
  SetXYZW(q, pose->mutable_pose()->mutable_orientation());
  if (path_msg.poses().size() > 250) {
    path_msg.mutable_poses()->DeleteSubrange(0, 1);
  }
  RVIZ_AGENT.Publish(topic, path_msg);
}

void MapFusionRviz::PubInsOdom(const Eigen::Affine3d& T_W_V, uint64_t sec,
                               uint64_t nsec, const std::string& topic) {
  if (!inited_ || !map_fusion_group_rviz_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  static bool register_flag = true;
  if (register_flag) {
    RVIZ_AGENT.Register<adsfi_proto::viz::Odometry>(topic);
    register_flag = false;
  }
  Eigen::Vector3d p = T_W_V.translation();
  Eigen::Quaterniond q(T_W_V.rotation());
  adsfi_proto::viz::Odometry odom_msg;
  odom_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
  odom_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  odom_msg.mutable_header()->set_frameid("map");
  odom_msg.set_child_frame_id("vehicle");
  SetXYZ(p, odom_msg.mutable_pose()->mutable_pose()->mutable_position());
  SetXYZW(q, odom_msg.mutable_pose()->mutable_pose()->mutable_orientation());
  for (size_t i = 0; i < 36; ++i) {
    odom_msg.mutable_pose()->add_covariance(0.);
  }
  RVIZ_AGENT.Publish(topic, odom_msg);
}

void MapFusionRviz::PubPerSection(const lane_loc::Section& section,
                                  uint64_t sec, uint64_t nsec,
                                  const std::string& topic) {
  if (!inited_ || !map_fusion_group_rviz_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  static bool register_flag = true;
  if (register_flag) {
    RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(topic);
    register_flag = false;
  }
  std::unordered_map<std::string, lane_loc::Section::Lane::LaneLine> lane_lines;
  adsfi_proto::viz::MarkerArray markers;
  int id = 0;
  for (const auto& lane : section.sorted_lanes) {
    if (lane.second.left_line.points.empty()) {
      continue;
    }
    lane_lines.insert({lane.second.left_line.line_id, lane.second.left_line});
    lane_lines.insert({lane.second.right_line.line_id, lane.second.right_line});
    adsfi_proto::viz::Marker lane_marker;
    lane_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
    lane_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
    lane_marker.set_id(id++);
    lane_marker.mutable_lifetime()->set_sec(0);
    lane_marker.mutable_lifetime()->set_nsec(200000000);
    lane_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
    lane_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    lane_marker.mutable_header()->set_frameid("vehicle");
    lane_marker.mutable_pose()->mutable_orientation()->set_x(0.);
    lane_marker.mutable_pose()->mutable_orientation()->set_y(0.);
    lane_marker.mutable_pose()->mutable_orientation()->set_z(0.);
    lane_marker.mutable_pose()->mutable_orientation()->set_w(1.);
    lane_marker.mutable_pose()->mutable_position()->set_x(0);
    lane_marker.mutable_pose()->mutable_position()->set_y(lane.first);
    lane_marker.mutable_pose()->mutable_position()->set_z(0);
    lane_marker.mutable_color()->set_r(1);
    lane_marker.mutable_color()->set_g(0);
    lane_marker.mutable_color()->set_b(0);
    lane_marker.mutable_color()->set_a(1);
    lane_marker.set_text(lane.second.lane_id);
    lane_marker.mutable_scale()->set_x(0.5);
    lane_marker.mutable_scale()->set_y(0.5);
    lane_marker.mutable_scale()->set_z(0.5);
    markers.add_markers()->CopyFrom(lane_marker);
  }
  for (auto lane_line : lane_lines) {
    if (lane_line.second.points.size() <= 1) {
      continue;
    }
    if (lane_line.second.points.size() % 2 != 0) {
      lane_line.second.points.push_back(*lane_line.second.points.rbegin());
    }
    adsfi_proto::viz::Marker point_marker;
    point_marker.mutable_header()->set_frameid("vehicle");
    point_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
    point_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    point_marker.set_id(id++);
    if (lane_line.second.line_type == lane_loc::Section::Lane::DOTTED_WHITE ||
        lane_line.second.line_type == lane_loc::Section::Lane::DOTTED_YELLOW) {
      point_marker.set_type(adsfi_proto::viz::MarkerType::LINE_LIST);
    } else {
      point_marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
    }
    point_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
    point_marker.mutable_pose()->mutable_orientation()->set_x(0.);
    point_marker.mutable_pose()->mutable_orientation()->set_y(0.);
    point_marker.mutable_pose()->mutable_orientation()->set_z(0.);
    point_marker.mutable_pose()->mutable_orientation()->set_w(1.);
    point_marker.mutable_scale()->set_x(0.05);
    point_marker.mutable_scale()->set_y(0.05);
    point_marker.mutable_scale()->set_z(0.05);
    point_marker.mutable_lifetime()->set_sec(0);
    point_marker.mutable_lifetime()->set_nsec(200000000);
    point_marker.mutable_color()->set_a(1.0);
    if (lane_line.second.line_type == lane_loc::Section::Lane::DOTTED_WHITE ||
        lane_line.second.line_type == lane_loc::Section::Lane::SOLID_WHITE) {
      point_marker.mutable_color()->set_r(1);
      point_marker.mutable_color()->set_g(1);
      point_marker.mutable_color()->set_b(1);
    } else {
      point_marker.mutable_color()->set_r(1);
      point_marker.mutable_color()->set_g(1);
      point_marker.mutable_color()->set_b(0);
    }
    for (const auto& point : lane_line.second.points) {
      auto* point_msg = point_marker.add_points();
      SetXYZ(point, point_msg);
    }
    markers.add_markers()->CopyFrom(point_marker);
    adsfi_proto::viz::Marker txt_marker;
    txt_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
    txt_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
    txt_marker.set_id(id++);
    txt_marker.mutable_lifetime()->set_sec(0);
    txt_marker.mutable_lifetime()->set_nsec(200000000);
    txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
    txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    txt_marker.mutable_header()->set_frameid("vehicle");
    txt_marker.mutable_pose()->mutable_orientation()->set_x(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_y(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_z(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_w(1.);
    txt_marker.mutable_pose()->mutable_position()->set_x(
        lane_line.second.points[0].x());
    txt_marker.mutable_pose()->mutable_position()->set_y(
        lane_line.second.points[0].y());
    txt_marker.mutable_pose()->mutable_position()->set_z(0);
    txt_marker.mutable_color()->set_r(1);
    txt_marker.mutable_color()->set_g(0);
    txt_marker.mutable_color()->set_b(0);
    txt_marker.mutable_color()->set_a(1);
    std::string txt;
    switch (lane_line.second.line_type) {
      case lane_loc::Section::Lane::UNKNOWN:
        txt = "UNKNOWN ";
        break;
      case lane_loc::Section::Lane::DOTTED_YELLOW:
        txt = "DOTTED_YELLOW ";
        break;
      case lane_loc::Section::Lane::DOTTED_WHITE:
        txt = "DOTTED_WHITE ";
        break;
      case lane_loc::Section::Lane::SOLID_YELLOW:
        txt = "SOLID_YELLOW ";
        break;
      case lane_loc::Section::Lane::SOLID_WHITE:
        txt = "SOLID_WHITE ";
        break;
      case lane_loc::Section::Lane::DOUBLE_YELLOW:
        txt = "DOUBLE_YELLOW ";
        break;
    }
    if (lane_line.second.is_near_road_edge) {
      txt += " road_edge";
    }
    txt_marker.set_text(lane_line.first + " " + txt);
    txt_marker.mutable_scale()->set_x(0.5);
    txt_marker.mutable_scale()->set_y(0.5);
    txt_marker.mutable_scale()->set_z(0.5);
    markers.add_markers()->CopyFrom(txt_marker);
  }
  RVIZ_AGENT.Publish(topic, markers);
}

void MapFusionRviz::PubMapSection(const lane_loc::Section& section,
                                  uint64_t sec, uint64_t nsec,
                                  const std::string& topic) {
  if (!inited_ || !map_fusion_group_rviz_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  static bool register_flag = true;
  if (register_flag) {
    RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(topic);
    register_flag = false;
  }
  std::unordered_map<std::string, lane_loc::Section::Lane::LaneLine> lane_lines;
  adsfi_proto::viz::MarkerArray markers;
  int id = 0;
  for (const auto& lane : section.sorted_lanes) {
    if (lane.second.left_line.points.empty()) {
      continue;
    }
    lane_lines.insert({lane.second.left_line.line_id, lane.second.left_line});
    lane_lines.insert({lane.second.right_line.line_id, lane.second.right_line});
    adsfi_proto::viz::Marker lane_marker;
    lane_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
    lane_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
    lane_marker.set_id(id++);
    lane_marker.mutable_lifetime()->set_sec(0);
    lane_marker.mutable_lifetime()->set_nsec(200000000);
    lane_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
    lane_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    lane_marker.mutable_header()->set_frameid("vehicle");
    lane_marker.mutable_pose()->mutable_orientation()->set_x(0.);
    lane_marker.mutable_pose()->mutable_orientation()->set_y(0.);
    lane_marker.mutable_pose()->mutable_orientation()->set_z(0.);
    lane_marker.mutable_pose()->mutable_orientation()->set_w(1.);
    lane_marker.mutable_pose()->mutable_position()->set_x(0);
    lane_marker.mutable_pose()->mutable_position()->set_y(lane.first);
    lane_marker.mutable_pose()->mutable_position()->set_z(0);
    lane_marker.mutable_color()->set_r(1);
    lane_marker.mutable_color()->set_g(1);
    lane_marker.mutable_color()->set_b(1);
    lane_marker.mutable_color()->set_a(1);
    lane_marker.set_text(lane.second.lane_id);
    lane_marker.mutable_scale()->set_x(0.5);
    lane_marker.mutable_scale()->set_y(0.5);
    lane_marker.mutable_scale()->set_z(0.5);
    markers.add_markers()->CopyFrom(lane_marker);
  }
  for (auto lane_line : lane_lines) {
    if (lane_line.second.points.size() <= 1) {
      continue;
    }
    if (lane_line.second.points.size() % 2 != 0) {
      lane_line.second.points.push_back(*lane_line.second.points.rbegin());
    }
    adsfi_proto::viz::Marker point_marker;
    point_marker.mutable_header()->set_frameid("vehicle");
    point_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
    point_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    point_marker.set_id(id++);
    if (lane_line.second.line_type == lane_loc::Section::Lane::DOTTED_WHITE ||
        lane_line.second.line_type == lane_loc::Section::Lane::DOTTED_YELLOW) {
      point_marker.set_type(adsfi_proto::viz::MarkerType::LINE_LIST);
    } else {
      point_marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
    }
    point_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
    point_marker.mutable_pose()->mutable_orientation()->set_x(0.);
    point_marker.mutable_pose()->mutable_orientation()->set_y(0.);
    point_marker.mutable_pose()->mutable_orientation()->set_z(0.);
    point_marker.mutable_pose()->mutable_orientation()->set_w(1.);
    point_marker.mutable_scale()->set_x(0.2);
    point_marker.mutable_scale()->set_y(0.2);
    point_marker.mutable_scale()->set_z(0.2);
    point_marker.mutable_lifetime()->set_sec(0);
    point_marker.mutable_lifetime()->set_nsec(200000000);
    point_marker.mutable_color()->set_a(0.5);
    if (lane_line.second.line_type == lane_loc::Section::Lane::DOTTED_WHITE ||
        lane_line.second.line_type == lane_loc::Section::Lane::SOLID_WHITE) {
      point_marker.mutable_color()->set_r(1);
      point_marker.mutable_color()->set_g(1);
      point_marker.mutable_color()->set_b(1);
    } else {
      point_marker.mutable_color()->set_r(1);
      point_marker.mutable_color()->set_g(1);
      point_marker.mutable_color()->set_b(0);
    }
    for (const auto& point : lane_line.second.points) {
      auto* point_msg = point_marker.add_points();
      SetXYZ(point, point_msg);
    }
    markers.add_markers()->CopyFrom(point_marker);
    adsfi_proto::viz::Marker txt_marker;
    txt_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
    txt_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
    txt_marker.set_id(id++);
    txt_marker.mutable_lifetime()->set_sec(0);
    txt_marker.mutable_lifetime()->set_nsec(200000000);
    txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
    txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    txt_marker.mutable_header()->set_frameid("vehicle");
    txt_marker.mutable_pose()->mutable_orientation()->set_x(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_y(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_z(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_w(1.);
    txt_marker.mutable_pose()->mutable_position()->set_x(
        lane_line.second.points[0].x());
    txt_marker.mutable_pose()->mutable_position()->set_y(
        lane_line.second.points[0].y());
    txt_marker.mutable_pose()->mutable_position()->set_z(0);
    txt_marker.mutable_color()->set_r(1);
    txt_marker.mutable_color()->set_g(1);
    txt_marker.mutable_color()->set_b(1);
    txt_marker.mutable_color()->set_a(1);
    std::string txt;
    switch (lane_line.second.line_type) {
      case lane_loc::Section::Lane::UNKNOWN:
        txt = "UNKNOWN ";
        break;
      case lane_loc::Section::Lane::DOTTED_YELLOW:
        txt = "DOTTED_YELLOW ";
        break;
      case lane_loc::Section::Lane::DOTTED_WHITE:
        txt = "DOTTED_WHITE ";
        break;
      case lane_loc::Section::Lane::SOLID_YELLOW:
        txt = "SOLID_YELLOW ";
        break;
      case lane_loc::Section::Lane::SOLID_WHITE:
        txt = "SOLID_WHITE ";
        break;
      case lane_loc::Section::Lane::DOUBLE_YELLOW:
        txt = "DOUBLE_YELLOW ";
        break;
    }
    txt_marker.set_text(lane_line.first + " " + txt);
    txt_marker.mutable_scale()->set_x(0.5);
    txt_marker.mutable_scale()->set_y(0.5);
    txt_marker.mutable_scale()->set_z(0.5);
    markers.add_markers()->CopyFrom(txt_marker);
  }
  RVIZ_AGENT.Publish(topic, markers);
}

void MapFusionRviz::PubLaneLocInfo(const lane_loc::LaneLocInfo& lane_loc_info,
                                   uint64_t sec, uint64_t nsec,
                                   const std::string& topic) {
  if (!inited_ || !map_fusion_group_rviz_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  static bool register_flag = true;
  if (register_flag) {
    RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(topic);
    register_flag = false;
  }
  std::unordered_map<std::string, lane_loc::Section::Lane::LaneLine> lane_lines;
  adsfi_proto::viz::MarkerArray markers;
  int id = 0;
  for (const auto& lane_info : lane_loc_info.next_lanes_info) {
    if (lane_info.next_lane.left_line.points.empty()) {
      continue;
    }
    lane_lines.insert(
        {lane_info.next_lane.left_line.line_id, lane_info.next_lane.left_line});
    lane_lines.insert({lane_info.next_lane.right_line.line_id,
                       lane_info.next_lane.right_line});
    adsfi_proto::viz::Marker lane_marker;
    lane_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
    lane_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
    lane_marker.set_id(id++);
    lane_marker.mutable_lifetime()->set_sec(0);
    lane_marker.mutable_lifetime()->set_nsec(200000000);
    lane_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
    lane_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    lane_marker.mutable_header()->set_frameid("vehicle");
    lane_marker.mutable_pose()->mutable_orientation()->set_x(0.);
    lane_marker.mutable_pose()->mutable_orientation()->set_y(0.);
    lane_marker.mutable_pose()->mutable_orientation()->set_z(0.);
    lane_marker.mutable_pose()->mutable_orientation()->set_w(1.);
    lane_marker.mutable_pose()->mutable_position()->set_x(
        lane_info.next_lane.left_line.points[0].x());
    lane_marker.mutable_pose()->mutable_position()->set_y(
        (lane_info.next_lane.left_line.points[0].y() +
         lane_info.next_lane.right_line.points[0].y()) /
        2);
    lane_marker.mutable_pose()->mutable_position()->set_z(0);
    lane_marker.mutable_color()->set_r(1);
    lane_marker.mutable_color()->set_g(1);
    lane_marker.mutable_color()->set_b(1);
    lane_marker.mutable_color()->set_a(1);
    lane_marker.set_text(lane_info.next_lane.lane_id);
    lane_marker.mutable_scale()->set_x(0.5);
    lane_marker.mutable_scale()->set_y(0.5);
    lane_marker.mutable_scale()->set_z(0.5);
    markers.add_markers()->CopyFrom(lane_marker);

    adsfi_proto::viz::Marker lane_num_marker;
    lane_num_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
    lane_num_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
    lane_num_marker.set_id(id++);
    lane_num_marker.mutable_lifetime()->set_sec(0);
    lane_num_marker.mutable_lifetime()->set_nsec(200000000);
    lane_num_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
    lane_num_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    lane_num_marker.mutable_header()->set_frameid("vehicle");
    lane_num_marker.mutable_pose()->mutable_orientation()->set_x(0.);
    lane_num_marker.mutable_pose()->mutable_orientation()->set_y(0.);
    lane_num_marker.mutable_pose()->mutable_orientation()->set_z(0.);
    lane_num_marker.mutable_pose()->mutable_orientation()->set_w(1.);
    lane_num_marker.mutable_pose()->mutable_position()->set_x(
        lane_info.next_lane.left_line.points[0].x() + 2);
    lane_num_marker.mutable_pose()->mutable_position()->set_y(
        (lane_info.next_lane.left_line.points[0].y() +
         lane_info.next_lane.right_line.points[0].y()) /
        2);
    lane_num_marker.mutable_pose()->mutable_position()->set_z(0);
    lane_num_marker.mutable_color()->set_r(0);
    lane_num_marker.mutable_color()->set_g(1);
    lane_num_marker.mutable_color()->set_b(0);
    lane_num_marker.mutable_color()->set_a(1);
    lane_num_marker.set_text("next_lane_nums: " +
                             std::to_string(lane_info.lane_num));
    lane_num_marker.mutable_scale()->set_x(1);
    lane_num_marker.mutable_scale()->set_y(1);
    lane_num_marker.mutable_scale()->set_z(1);
    markers.add_markers()->CopyFrom(lane_num_marker);
  }
  for (auto lane_line : lane_lines) {
    if (lane_line.second.points.size() <= 1) {
      continue;
    }
    if (lane_line.second.points.size() % 2 != 0) {
      lane_line.second.points.push_back(*lane_line.second.points.rbegin());
    }
    adsfi_proto::viz::Marker point_marker;
    point_marker.mutable_header()->set_frameid("vehicle");
    point_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
    point_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    point_marker.set_id(id++);
    if (lane_line.second.line_type == lane_loc::Section::Lane::DOTTED_WHITE ||
        lane_line.second.line_type == lane_loc::Section::Lane::DOTTED_YELLOW) {
      point_marker.set_type(adsfi_proto::viz::MarkerType::LINE_LIST);
    } else {
      point_marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
    }
    point_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
    point_marker.mutable_pose()->mutable_orientation()->set_x(0.);
    point_marker.mutable_pose()->mutable_orientation()->set_y(0.);
    point_marker.mutable_pose()->mutable_orientation()->set_z(0.);
    point_marker.mutable_pose()->mutable_orientation()->set_w(1.);
    point_marker.mutable_scale()->set_x(0.2);
    point_marker.mutable_scale()->set_y(0.2);
    point_marker.mutable_scale()->set_z(0.2);
    point_marker.mutable_lifetime()->set_sec(0);
    point_marker.mutable_lifetime()->set_nsec(200000000);
    point_marker.mutable_color()->set_a(1);
    point_marker.mutable_color()->set_r(0);
    point_marker.mutable_color()->set_g(1);
    point_marker.mutable_color()->set_b(0);
    for (const auto& point : lane_line.second.points) {
      auto* point_msg = point_marker.add_points();
      SetXYZ(point, point_msg);
    }
    markers.add_markers()->CopyFrom(point_marker);
    adsfi_proto::viz::Marker txt_marker;
    txt_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
    txt_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
    txt_marker.set_id(id++);
    txt_marker.mutable_lifetime()->set_sec(0);
    txt_marker.mutable_lifetime()->set_nsec(200000000);
    txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
    txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    txt_marker.mutable_header()->set_frameid("vehicle");
    txt_marker.mutable_pose()->mutable_orientation()->set_x(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_y(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_z(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_w(1.);
    txt_marker.mutable_pose()->mutable_position()->set_x(
        lane_line.second.points[0].x());
    txt_marker.mutable_pose()->mutable_position()->set_y(
        lane_line.second.points[0].y());
    txt_marker.mutable_pose()->mutable_position()->set_z(0);
    txt_marker.mutable_color()->set_r(0);
    txt_marker.mutable_color()->set_g(1);
    txt_marker.mutable_color()->set_b(0);
    txt_marker.mutable_color()->set_a(1);
    std::string txt;
    switch (lane_line.second.line_type) {
      case lane_loc::Section::Lane::UNKNOWN:
        txt = "UNKNOWN ";
        break;
      case lane_loc::Section::Lane::DOTTED_YELLOW:
        txt = "DOTTED_YELLOW ";
        break;
      case lane_loc::Section::Lane::DOTTED_WHITE:
        txt = "DOTTED_WHITE ";
        break;
      case lane_loc::Section::Lane::SOLID_YELLOW:
        txt = "SOLID_YELLOW ";
        break;
      case lane_loc::Section::Lane::SOLID_WHITE:
        txt = "SOLID_WHITE ";
        break;
      case lane_loc::Section::Lane::DOUBLE_YELLOW:
        txt = "DOUBLE_YELLOW ";
        break;
    }
    txt_marker.set_text(lane_line.first + " " + txt);
    txt_marker.mutable_scale()->set_x(0.5);
    txt_marker.mutable_scale()->set_y(0.5);
    txt_marker.mutable_scale()->set_z(0.5);
    markers.add_markers()->CopyFrom(txt_marker);
  }
  RVIZ_AGENT.Publish(topic, markers);
}

void MapFusionRviz::PubState(
    const std::string& lane_num, const std::string& road_edge_state,
    const std::string& measure_lane_index, const std::vector<double>& p_measure,
    const lane_loc::TurnState& turn_state, const std::vector<double>& p_predict,
    const std::string& fusion_lane_index, const std::vector<double>& p_fusion,
    uint64_t sec, uint64_t nsec, const std::string& topic) {
  if (!inited_ || !map_fusion_group_rviz_ || !RVIZ_AGENT.Ok()) {
    return;
  }
  static bool register_flag = true;
  if (register_flag) {
    RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(topic);
    register_flag = false;
  }
  adsfi_proto::viz::MarkerArray markers;
  adsfi_proto::viz::Marker text_marker;
  int id = 0;
  text_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
  text_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
  text_marker.set_id(id++);
  text_marker.mutable_lifetime()->set_sec(0);
  text_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
  text_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  text_marker.mutable_header()->set_frameid("vehicle");
  text_marker.mutable_pose()->mutable_position()->set_x(-5);
  text_marker.mutable_pose()->mutable_position()->set_y(0);
  text_marker.mutable_pose()->mutable_position()->set_z(2);
  text_marker.mutable_pose()->mutable_orientation()->set_x(0);
  text_marker.mutable_pose()->mutable_orientation()->set_y(0);
  text_marker.mutable_pose()->mutable_orientation()->set_z(0);
  text_marker.mutable_pose()->mutable_orientation()->set_w(1);
  text_marker.mutable_color()->set_r(1);
  text_marker.mutable_color()->set_g(1);
  text_marker.mutable_color()->set_b(0);
  text_marker.mutable_color()->set_a(1);
  std::string turn_state_msg;
  switch (turn_state) {
    case lane_loc::STARIGHT:
      turn_state_msg = "STARIGHT";
      break;
    case lane_loc::TURN_LEFT:
      turn_state_msg = "TURN_LEFT";
      break;
    case lane_loc::TURN_RIGHT:
      turn_state_msg = "TURN_RIGHT";
      break;
  }
  std::string txt;
  txt = "lane_num: " + lane_num + "\n" + road_edge_state +
        "\nmeasure_lane_index: " + measure_lane_index + "\np_measure: ";
  for (const auto& p : p_measure) {
    txt += std::to_string(p) + " ";
  }
  txt += "\nturn_state: " + turn_state_msg + "\np_predict: ";
  for (const auto& p : p_predict) {
    txt += std::to_string(p) + " ";
  }
  text_marker.set_text(txt);
  text_marker.mutable_scale()->set_x(1);
  text_marker.mutable_scale()->set_y(1);
  text_marker.mutable_scale()->set_z(1);

  auto result_marker = text_marker;
  result_marker.set_id(id++);
  result_marker.mutable_pose()->mutable_position()->set_x(5);
  result_marker.mutable_color()->set_r(0);
  result_marker.mutable_color()->set_g(1);
  result_marker.mutable_color()->set_b(0);
  std::string result_txt =
      "fusion_lane_index " + fusion_lane_index + "\np_fusion: ";
  for (const auto& p : p_fusion) {
    result_txt += std::to_string(p) + " ";
  }
  result_marker.set_text(result_txt);

  markers.add_markers()->CopyFrom(text_marker);
  markers.add_markers()->CopyFrom(result_marker);
  hozon::mp::util::RvizAgent::Instance().Publish(topic, markers);
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
