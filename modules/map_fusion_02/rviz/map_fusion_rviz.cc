/***************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_fusion_rviz.h
 *   author     ： cuijiayu
 *   date       ： 2024.01
 ******************************************************************************/
#include "modules/map_fusion_02/rviz/map_fusion_rviz.h"

#include "base/utils/log.h"

namespace hozon {
namespace mp {
namespace mf {

std::string MapFusionRviz::Name() const { return "MapFusionRviz"; }

bool MapFusionRviz::Init() {
  auto* config_manager = hozon::perception::lib::ConfigManager::Instance();
  const hozon::perception::lib::ModelConfig* model_config = nullptr;
  if (!config_manager->GetModelConfig(Name(), &model_config)) {
    HLOG_ERROR << "Parse config failed! Name: " << Name();
    return false;
  }

  if (!model_config->get_value("viz", &use_rviz_)) {
    HLOG_ERROR << "Get viz failed!";
    return false;
  }

  if (!model_config->get_value("rviz_addr_mfr", &rviz_addr_mfr_)) {
    HLOG_ERROR << "Get rviz_addr_mfr failed!";
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
  if (!model_config->get_value("viz_topic_path", &viz_topic_path_)) {
    HLOG_ERROR << "Get viz_topic_path failed!";
    return false;
  }
  if (!model_config->get_value("viz_topic_group", &viz_topic_group_)) {
    HLOG_ERROR << "Get viz_topic_group failed!";
    return false;
  }
  if (!model_config->get_value("viz_lifetime", &viz_lifetime_)) {
    HLOG_ERROR << "Get viz_lifetime failed!";
    return false;
  }

  if (use_rviz_) {
    HLOG_FATAL << "Start RvizAgent!!!";
    int ret = RVIZ_AGENT.Init(rviz_addr_mfr_);
    if (ret < 0) {
      HLOG_FATAL << "RvizAgent start failed";
    }

    if (!RVIZ_AGENT.Ok()) {
      HLOG_FATAL << "RvizAgent not Init";
    } else {
      std::vector<std::string> marker_topics = {viz_topic_input_ele_map_,
                                                viz_topic_output_ele_map_,
                                                viz_topic_group_};
      int ret =
          RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(marker_topics);
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
      ret = RVIZ_AGENT.Register<adsfi_proto::viz::PointCloud>(
          viz_topic_guidepoints_);
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
      ret =
          RVIZ_AGENT.Register<adsfi_proto::viz::PointCloud>(dist_points_topic);
      if (ret < 0) {
        HLOG_FATAL << "RvizAgent register cut point failed";
        return false;
      }
    }
  }
  HLOG_FATAL << "MapFusionRviz::Init true";
  return true;
}

void MapFusionRviz::VizEleMap(const std::shared_ptr<ElementMap>& ele_map) {
  if (!use_rviz_ || !RVIZ_AGENT.Ok()) {
    return;
  }

  auto m = ElementMapToMarkers(*ele_map, "vehicle", "topo_gen",
                               ele_map->map_info.stamp, viz_lifetime_);
  if (m != nullptr) {
    RVIZ_AGENT.Publish(viz_topic_input_ele_map_, m);
  }
}

void MapFusionRviz::VizPath(const std::vector<KinePosePtr>& path,
                            const KinePose& curr_pose) {
  if (!use_rviz_ || !RVIZ_AGENT.Ok()) {
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

void MapFusionRviz::VizGroup(const std::vector<Group::Ptr>& groups,
                             double stamp) {
  if (!use_rviz_ || !RVIZ_AGENT.Ok()) {
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
      HLOG_ERROR << "found nullptr group";
      continue;
    }

    RvizRgb line_rgb = ColorRgb(line_colors.at(grp_idx % line_colors.size()));

    // start_slice
    auto* marker_start_slice = marker_array->add_markers();
    marker_start_slice->mutable_header()->CopyFrom(viz_header);
    marker_start_slice->set_ns(grp_ns + "/slice");
    marker_start_slice->set_id(0);
    marker_start_slice->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
    double width = 1;
    marker_start_slice->mutable_scale()->set_x(width);
    marker_start_slice->mutable_lifetime()->set_sec(life_sec);
    marker_start_slice->mutable_lifetime()->set_nsec(life_nsec);
    marker_start_slice->mutable_color()->set_a(0.1);
    marker_start_slice->mutable_color()->set_r(line_rgb.r);
    marker_start_slice->mutable_color()->set_g(line_rgb.g);
    marker_start_slice->mutable_color()->set_b(line_rgb.b);
    marker_start_slice->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
    marker_start_slice->mutable_pose()->mutable_orientation()->set_w(1);
    marker_start_slice->mutable_pose()->mutable_orientation()->set_x(0);
    marker_start_slice->mutable_pose()->mutable_orientation()->set_y(0);
    marker_start_slice->mutable_pose()->mutable_orientation()->set_z(0);
    double scale = 1;
    Eigen::Vector3f left = (grp->start_slice.pl - grp->start_slice.po) * scale +
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

    // end_slice
    auto* marker_end_slice = marker_array->add_markers();
    marker_end_slice->CopyFrom(*marker_start_slice);
    marker_end_slice->set_id(1);
    marker_end_slice->clear_points();
    left = (grp->end_slice.pl - grp->end_slice.po) * scale + grp->end_slice.po;
    center = grp->end_slice.po;
    right = (grp->end_slice.pr - grp->end_slice.po) * scale + grp->end_slice.po;
    pts = std::vector<Eigen::Vector3f>{left, center, right};
    for (const auto& p : pts) {
      auto* pt = marker_end_slice->add_points();
      pt->set_x(p.x());
      pt->set_y(p.y());
      pt->set_z(p.z());
    }

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
      double width = 0.2;
      marker_left_bound->mutable_scale()->set_x(width);
      marker_left_bound->mutable_scale()->set_y(width);
      marker_left_bound->mutable_scale()->set_z(width);
      marker_left_bound->mutable_lifetime()->set_sec(life_sec);
      marker_left_bound->mutable_lifetime()->set_nsec(life_nsec);
      marker_left_bound->mutable_color()->set_a(1.0);
      marker_left_bound->mutable_color()->set_r(line_rgb.r);
      marker_left_bound->mutable_color()->set_g(line_rgb.g);
      marker_left_bound->mutable_color()->set_b(line_rgb.b);
      marker_left_bound->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
      marker_left_bound->mutable_pose()->mutable_orientation()->set_w(1);
      marker_left_bound->mutable_pose()->mutable_orientation()->set_x(0);
      marker_left_bound->mutable_pose()->mutable_orientation()->set_y(0);
      marker_left_bound->mutable_pose()->mutable_orientation()->set_z(0);

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
      marker_center_line->mutable_color()->set_r(line_rgb.r);
      marker_center_line->mutable_color()->set_g(line_rgb.g);
      marker_center_line->mutable_color()->set_b(line_rgb.b);

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
      double text_size = 0.5;
      marker_str_id_with_group->mutable_scale()->set_z(text_size);
      marker_str_id_with_group->mutable_lifetime()->set_sec(life_sec);
      marker_str_id_with_group->mutable_lifetime()->set_nsec(life_nsec);
      marker_str_id_with_group->mutable_color()->set_a(0.2);
      marker_str_id_with_group->mutable_color()->set_r(line_rgb.r);
      marker_str_id_with_group->mutable_color()->set_g(line_rgb.g);
      marker_str_id_with_group->mutable_color()->set_b(line_rgb.b);
      marker_str_id_with_group->set_type(
          adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
      marker_str_id_with_group->mutable_pose()->mutable_orientation()->set_w(1);
      marker_str_id_with_group->mutable_pose()->mutable_orientation()->set_x(0);
      marker_str_id_with_group->mutable_pose()->mutable_orientation()->set_y(0);
      marker_str_id_with_group->mutable_pose()->mutable_orientation()->set_z(0);
      marker_str_id_with_group->mutable_pose()->mutable_position()->set_x(
          lane->center_line_pts.front().pt.x());
      marker_str_id_with_group->mutable_pose()->mutable_position()->set_y(
          lane->center_line_pts.front().pt.y());
      marker_str_id_with_group->mutable_pose()->mutable_position()->set_z(
          lane->center_line_pts.front().pt.z());
      auto* text = marker_str_id_with_group->mutable_text();
      *text = lane->str_id_with_group + " broken id " + std::to_string(grp->broken_id);
      if (!lane->next_lane_str_id_with_group.empty() &&
          lane->center_line_pts.size() > 2) {
        marker_str_id_with_group = marker_array->add_markers();
        marker_str_id_with_group->mutable_header()->CopyFrom(viz_header);
        marker_str_id_with_group->set_ns(grp_ns + "/" + lane_ns + "/next");
        marker_str_id_with_group->set_id(9);
        marker_str_id_with_group->set_action(
            adsfi_proto::viz::MarkerAction::MODIFY);
        double text_size = 0.5;
        marker_str_id_with_group->mutable_scale()->set_z(text_size);
        marker_str_id_with_group->mutable_lifetime()->set_sec(life_sec);
        marker_str_id_with_group->mutable_lifetime()->set_nsec(life_nsec);
        marker_str_id_with_group->mutable_color()->set_a(0.2);
        marker_str_id_with_group->mutable_color()->set_r(line_rgb.r);
        marker_str_id_with_group->mutable_color()->set_g(line_rgb.g);
        marker_str_id_with_group->mutable_color()->set_b(line_rgb.b);
        marker_str_id_with_group->set_type(
            adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
        marker_str_id_with_group->mutable_pose()->mutable_orientation()->set_w(
            1);
        marker_str_id_with_group->mutable_pose()->mutable_orientation()->set_x(
            0);
        marker_str_id_with_group->mutable_pose()->mutable_orientation()->set_y(
            0);
        marker_str_id_with_group->mutable_pose()->mutable_orientation()->set_z(
            0);
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
  }
  if (!marker_array->markers().empty()) {
    RVIZ_AGENT.Publish(viz_topic_group_, marker_array);
  }
}

void MapFusionRviz::VizGuidePoint(const std::vector<Group::Ptr>& groups,
                                  double stamp) {
  if (!use_rviz_ || !RVIZ_AGENT.Ok()) {
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
  if (!use_rviz_ || !RVIZ_AGENT.Ok()) {
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
  if (!use_rviz_ || !RVIZ_AGENT.Ok()) {
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

}  // namespace mf
}  // namespace mp
}  // namespace hozon
