/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： topo_generation.cc
 *   author     ： taoshaoyuan
 *   date       ： 2023.12
 ******************************************************************************/

#include "map_fusion/road_recognition/topo_generation.h"

#include <cstddef>
#include <string>
#include <vector>

#include "map_fusion/fusion_common/calc_util.h"
#include "map_fusion/fusion_common/viz_util.h"
#include "map_fusion/road_recognition/group_map.h"
#include "map_fusion/road_recognition/path_manager.h"
#include "util/rviz_agent/rviz_agent.h"

namespace hozon {
namespace mp {
namespace mf {

bool TopoGeneration::Init(const YAML::Node& conf) {
  std::vector<std::string> required = {
      "viz",
      "viz_topic_input_ele_map",
      "viz_topic_output_ele_map",
      "viz_topic_path",
      "viz_topic_group",
      "viz_lifetime",
      "path_predict_range",
      "path_back_range",
      "path_interval",
      "lane_line_interp_dist",
      "half_slice_length",
      "min_lane_width",
      "max_lane_width",
      "lane_speed_limit_kmph",
      "road_min_max_speed_kmph",
      "road_max_max_speed_kmph",
      "predict_farthest_dist",
      "robust_percep_dist",
      "max_heading_std_dev",
      "min_predict_interval",
      "max_heading_degree",
      "junction_heading_diff",
  };
  for (const auto& it : required) {
    if (!conf[it].IsDefined()) {
      HLOG_ERROR << "cannot find conf key " << it;
      return false;
    }
  }

  viz_ = conf["viz"].as<bool>();
  viz_topic_input_ele_map_ = conf["viz_topic_input_ele_map"].as<std::string>();
  viz_topic_output_ele_map_ =
      conf["viz_topic_output_ele_map"].as<std::string>();
  viz_topic_path_ = conf["viz_topic_path"].as<std::string>();
  viz_topic_group_ = conf["viz_topic_group"].as<std::string>();
  viz_lifetime_ = conf["viz_lifetime"].as<double>();

  if (viz_) {
    if (!RVIZ_AGENT.Ok()) {
      HLOG_WARN << "RvizAgent not Init";
    } else {
      std::vector<std::string> marker_topics = {viz_topic_input_ele_map_,
                                                viz_topic_output_ele_map_,
                                                viz_topic_group_};
      int ret =
          RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(marker_topics);
      if (ret < 0) {
        HLOG_ERROR << "RvizAgent register marker array failed";
        return false;
      }

      std::vector<std::string> pose_array_topics = {
          viz_topic_path_,
      };
      ret = RVIZ_AGENT.Register<adsfi_proto::viz::PoseArray>(pose_array_topics);
      if (ret < 0) {
        HLOG_ERROR << "RvizAgent register pose array failed";
        return false;
      }
    }
  }

  path_predict_range_ = conf["path_predict_range"].as<double>();
  PathManagerConf pm_conf;
  pm_conf.back_range = conf["path_back_range"].as<float>();
  pm_conf.interval = conf["path_interval"].as<float>();
  gm_conf_.lane_line_interp_dist = conf["lane_line_interp_dist"].as<float>();
  gm_conf_.half_slice_length = conf["half_slice_length"].as<float>();
  gm_conf_.min_lane_width = conf["min_lane_width"].as<float>();
  gm_conf_.max_lane_width = conf["max_lane_width"].as<float>();
  gm_conf_.lane_speed_limit_kmph = conf["lane_speed_limit_kmph"].as<float>();
  gm_conf_.road_min_max_speed_kmph =
      conf["road_min_max_speed_kmph"].as<float>();
  gm_conf_.road_max_max_speed_kmph =
      conf["road_max_max_speed_kmph"].as<float>();

  gm_conf_.predict_farthest_dist = conf["predict_farthest_dist"].as<float>();
  gm_conf_.robust_percep_dist = conf["robust_percep_dist"].as<float>();
  gm_conf_.max_heading_std_dev = conf["max_heading_std_dev"].as<float>();
  gm_conf_.min_predict_interval = conf["min_predict_interval"].as<float>();
  gm_conf_.max_heading_rad =
      static_cast<float>(DegToRad(conf["max_heading_degree"].as<double>()));
  gm_conf_.junction_heading_diff =
      static_cast<float>(DegToRad(conf["junction_heading_diff"].as<double>()));
  is_cross_.cross_after_lane_ = 0;
  is_cross_.cross_before_lane_ = 0;
  is_cross_.is_crossing_ = 0;
  is_cross_.along_path_dis_.x() = 0.0;
  is_cross_.along_path_dis_.y() = 0.0;
  is_cross_.along_path_dis_.z() = 0.0;
  is_cross_.next_lane_left = -1000;
  is_cross_.next_lane_right = -1000;
  history_id_.lane_id = 0;
  history_id_.road_id = 0;
  history_id_.cicle = 2000;
  path_ = std::make_shared<PathManager>(pm_conf);

  return true;
}

void TopoGeneration::OnLocalization(
    const std::shared_ptr<hozon::localization::Localization>& msg) {
  KinePose curr;
  curr.stamp = msg->header().data_stamp();
  curr.pos << static_cast<float>(msg->pose_local().position().x()),
      static_cast<float>(msg->pose_local().position().y()),
      static_cast<float>(msg->pose_local().position().z());
  curr.quat.w() = msg->pose_local().quaternion().w();
  curr.quat.x() = msg->pose_local().quaternion().x();
  curr.quat.y() = msg->pose_local().quaternion().y();
  curr.quat.z() = msg->pose_local().quaternion().z();
  // 注意：上游发过来的速度、加速度、角速度都是车体系下的
  curr.vel << static_cast<float>(msg->pose_local().linear_velocity().x()),
      static_cast<float>(msg->pose_local().linear_velocity().y()),
      static_cast<float>(msg->pose_local().linear_velocity().z());
  curr.acc << static_cast<float>(msg->pose_local().linear_acceleration().x()),
      static_cast<float>(msg->pose_local().linear_acceleration().y()),
      static_cast<float>(msg->pose_local().linear_acceleration().z());
  curr.ang_vel << static_cast<float>(msg->pose_local().angular_velocity().x()),
      static_cast<float>(msg->pose_local().angular_velocity().y()),
      static_cast<float>(msg->pose_local().angular_velocity().z());
  path_->AddPose(curr);
}

void TopoGeneration::OnElementMap(
    const std::shared_ptr<hozon::mp::mf::em::ElementMap>& ele_map) {
  if (ele_map == nullptr) {
    HLOG_ERROR << "nullptr input ele_map";
    return;
  }

  VizEleMap(ele_map);

  ele_map_ = ele_map;
}

std::shared_ptr<hozon::mp::mf::em::ElementMapOut> TopoGeneration::GetEleMap() {
  return ele_map_output_;
}

std::shared_ptr<hozon::hdmap::Map> TopoGeneration::GetPercepMap() {
  auto path = std::make_shared<std::vector<KinePose::Ptr>>();
  // if(ego_exist_){
  //   path_->GetPath(path.get(), static_cast<float>(path_predict_range_),
  //   line_params_);
  // }else{
  //   path_->GetPath(path.get(), static_cast<float>(path_predict_range_));
  // }
  path_->GetPath(path.get(), static_cast<float>(path_predict_range_));
  auto curr_pose = path_->LatestPose();

  VizPath(*path, *curr_pose);

  std::shared_ptr<hozon::hdmap::Map> proto_map = nullptr;

  gm::GroupMap group_map(gm_conf_);
  auto ret = group_map.Build(path, curr_pose, ele_map_, &is_cross_);
  if (!ret) {
    HLOG_ERROR << "Build group map failed";
    return nullptr;
  }

  // if (group_map.ego_line_exist_) {
  //   ego_exist_ = true;
  //   line_params_ = group_map.predict_line_params_;
  // } else {
  //   ego_exist_ = false;
  //   line_params_.clear();
  // }
  std::vector<gm::Group::Ptr> groups;
  group_map.GetGroups(&groups);
  IsInCrossing(groups, &is_cross_);
  VizGroup(groups, ele_map_->map_info.stamp);

  proto_map = group_map.Export(ele_map_, &history_id_);
  ele_map_output_ = group_map.AddElementMap(ele_map_);
  return proto_map;
}

void TopoGeneration::VizEleMap(
    const std::shared_ptr<hozon::mp::mf::em::ElementMap>& ele_map) {
  if (!viz_ || !RVIZ_AGENT.Ok()) {
    return;
  }

  auto m = viz::ElementMapToMarkers(*ele_map, "vehicle", "topo_gen",
                                    ele_map->map_info.stamp, viz_lifetime_);
  if (m != nullptr) {
    RVIZ_AGENT.Publish(viz_topic_input_ele_map_, m);
  }
}

void TopoGeneration::VizPath(const std::vector<KinePose::Ptr>& path,
                             const KinePose& curr_pose) {
  if (!viz_ || !RVIZ_AGENT.Ok()) {
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
  auto pose_array = viz::PosesToPoseArray(poses, "vehicle", curr_pose.stamp);
  if (pose_array != nullptr) {
    RVIZ_AGENT.Publish(viz_topic_path_, pose_array);
  }
}

void TopoGeneration::VizGroup(const std::vector<gm::Group::Ptr>& groups,
                              double stamp) {
  if (!viz_ || !RVIZ_AGENT.Ok()) {
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
  std::vector<viz::Color> line_colors = {viz::ORANGE, viz::YELLOW, viz::GREEN,
                                         viz::CYAN};
  int grp_idx = -1;
  for (const auto& grp : groups) {
    grp_idx += 1;
    std::string grp_ns =
        "grp" + std::to_string(grp_idx) + "(" + grp->str_id + ")";
    if (grp == nullptr) {
      HLOG_ERROR << "found nullptr group";
      continue;
    }

    viz::Rgb line_rgb =
        viz::ColorRgb(line_colors.at(grp_idx % line_colors.size()));
    int seg_idx = -1;
    for (const auto& seg : grp->group_segments) {
      seg_idx += 1;
      std::string seg_ns =
          "seg" + std::to_string(seg_idx) + "(" + seg->str_id + ")";
      // start_slice
      auto* marker_start_slice = marker_array->add_markers();
      marker_start_slice->mutable_header()->CopyFrom(viz_header);
      marker_start_slice->set_ns(grp_ns + "/" + seg_ns + "/slice");
      marker_start_slice->set_id(0);
      marker_start_slice->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
      double width = 0.05;
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
      em::Point left = (seg->start_slice.pl - seg->start_slice.po) * scale +
                       seg->start_slice.po;
      em::Point center = seg->start_slice.po;
      em::Point right = (seg->start_slice.pr - seg->start_slice.po) * scale +
                        seg->start_slice.po;
      std::vector<em::Point> pts = {left, center, right};
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
      left =
          (seg->end_slice.pl - seg->end_slice.po) * scale + seg->end_slice.po;
      center = seg->end_slice.po;
      right =
          (seg->end_slice.pr - seg->end_slice.po) * scale + seg->end_slice.po;
      pts = std::vector<em::Point>{left, center, right};
      for (const auto& p : pts) {
        auto* pt = marker_end_slice->add_points();
        pt->set_x(p.x());
        pt->set_y(p.y());
        pt->set_z(p.z());
      }
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

      std::vector<em::Point> left_pred_pts;
      std::vector<em::Point> right_pred_pts;
      // 在同一根lane可能前端和后端都有虚拟车道，要区分开来
      std::vector<em::Point> left_virtual_pts;
      std::vector<em::Point> right_virtual_pts;
      std::vector<em::Point> left_virtual_pts_back;
      std::vector<em::Point> right_virtual_pts_back;
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
        if (p.type == gm::PREDICTED) {
          left_pred_pts.emplace_back(p.pt);
        } else if (p.type == gm::VIRTUAL) {
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
        if (p.type == gm::PREDICTED) {
          right_pred_pts.emplace_back(p.pt);
        } else if (p.type == gm::VIRTUAL) {
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
      *text = lane->str_id_with_group;
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

bool TopoGeneration::IsValid(const std::vector<gm::Group::Ptr>& groups) {
  int is_in_group = 0;  // 是否在所构建的group里
  for (size_t i = 0; i < groups.size(); ++i) {
    auto group = groups[i];
    size_t group_segments_size = group->group_segments.size();
    if (group_segments_size < 2) {
      continue;
    }
    if (group->group_segments[0]->start_slice.po.x() < 0 &&
        group->group_segments[group_segments_size - 1]->end_slice.po.x() > 0) {
      is_in_group = 1;
      if (group->str_id.back() == 'V') {
        return false;
      }
      if (i == groups.size() - 1) {
        // 判断路口前还是路口后
        if (group->group_segments[group_segments_size - 1]->end_slice.po.x() <
            10) {
          return false;
        }
      }
      // if(i > 0 && groups[i-1]->str_id.back() =='V' &&
      // group->group_segments[0]->start_slice.po.x()<){

      // }
      break;
    }
  }
  if (is_in_group == 0) {
    return false;
  }
  return true;
}

void TopoGeneration::IsInCrossing(const std::vector<gm::Group::Ptr>& groups,
                                  gm::IsCross* iscross) {
  if (groups.size() < 1) {
    return;
  }
  int index = groups.size() - 1;
  while (index >= 0 && groups[index]->group_segments.size() < 2) {
    index--;
  }
  if (index >= 0) {
    if (groups[index]->group_segments.back()->end_slice.po.x() < -5.0) {
      iscross->is_crossing_ = 1;
      iscross->along_path_dis_ =
          groups[index]->group_segments.back()->end_slice.po;
    } else if (groups[index]->group_segments.back()->end_slice.po.x() > 5.0) {
      for (int i = index; i > 0; i--) {
        if (groups[i]->group_segments.size() < 2) {
          // 因为虚拟group没有group_segments
          int curr_group_v = 0;
          if (groups[i]->str_id.back() == 'V' && i > 0 && i < index) {
            for (const auto& lane : groups[i]->lanes) {
              if (lane->center_line_pts.size() > 0 &&
                  lane->center_line_pts[0].pt.x() < 0.0) {
                size_t crs_before = groups[i - 1]->lanes.size();
                size_t crs_after = groups[i + 1]->lanes.size();
                if ((groups[i + 1]->group_segments.front()->start_slice.po.x() >
                     1.0)) {
                  //(crs_before != iscross->cross_before_lane_ ||
                  //  crs_after != iscross->cross_after_lane_) &&
                  iscross->along_path_dis_ =
                      groups[i - 1]->group_segments.back()->end_slice.po;
                  iscross->cross_before_lane_ = crs_before;
                  iscross->cross_after_lane_ = crs_after;
                }
                curr_group_v = 1;
                break;
              }
            }
          }
          if (curr_group_v) {
            break;
          }
        } else if (groups[i]->group_segments[0]->start_slice.po.x() < 0.0) {
          //  如果找到车所在的group
          HLOG_ERROR << "groups[i]->str_id = " << groups[i]->str_id;
          iscross->is_crossing_ = 0;
          iscross->next_lane_left = -1000;
          iscross->next_lane_right = -1000;
          break;
        }
      }
    }
  }
}
}  // namespace mf
}  // namespace mp
}  // namespace hozon
