/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 * Zhu Kaiying <zhukaiying@sensetime.com>
 */

#include "system/localization_visualizer.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <fstream>
#include <string>
#include <tuple>

#include <opencv2/video.hpp>

#include "ad_time/ad_time.hpp"
#include "common/coordinate_converter.hpp"
#include "common/path_util.hpp"
#include "common/transform_config.hpp"
#include "common/utility.hpp"
#include "localization/common/log.hpp"

namespace senseAD {
namespace localization {

LocalizationVisualizer::~LocalizationVisualizer() {
  if (thread_ && thread_->joinable()) thread_->join();
}

adLocStatus_t LocalizationVisualizer::Init(const LocalizationParam& param) {
  param_ = param;

  // init thread
  thread_.reset(new std::thread(&LocalizationVisualizer::Run, this));
  if (nullptr == thread_) {
    LC_LERROR(SYSTEM) << "Failed to create visualizer thread.";
    return LOC_ALLOC_MEMORY_FAILED;
  }
  SetThreadName();

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationVisualizer::Restart(bool is_abs_loc_mode) {
  if (is_abs_loc_mode) {
    history_loc_pose_.clear();
    history_ins_pose_.clear();
  } else {
    history_dr_pose_.clear();
  }
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationVisualizer::SwitchOriginProc() {
  // convert loc info related to new origin
  {
    std::lock_guard<std::mutex> lock(loc_info_mu_);
    CoordinateConverter::GetInstance()->ConvertENUToNewOrigin(
        loc_info_.position, &loc_info_.position);
  }
  {
    std::lock_guard<std::mutex> lock(ins_info_mu_);
    CoordinateConverter::GetInstance()->ConvertENUToNewOrigin(
        ins_state_.pose.translation(), &ins_state_.pose.translation());
  }
  {
    std::lock_guard<std::mutex> lock(gnss_info_mu_);
    CoordinateConverter::GetInstance()->ConvertENUToNewOrigin(
        gnss_state_.pose.translation(), &gnss_state_.pose.translation());
  }
  // convert map point to new origin
  {
    std::lock_guard<std::mutex> lock(local_map_data_mu_);
    for (auto& line : local_map_data_.second->semantic_map_data.lines) {
      for (auto& segment : line.line_segments) {
        for (auto& point : segment.points) {
          CoordinateConverter::GetInstance()->ConvertENUToNewOrigin(point,
                                                                    &point);
        }
      }
    }
    for (auto& lane : local_map_data_.second->route_map_data.lanes) {
      for (auto& point : lane.center_points) {
        CoordinateConverter::GetInstance()->ConvertENUToNewOrigin(point,
                                                                  &point);
      }
    }
    for (auto& pole : local_map_data_.second->semantic_map_data.poles) {
      CoordinateConverter::GetInstance()->ConvertENUToNewOrigin(
          pole.top_point, &pole.top_point);
      CoordinateConverter::GetInstance()->ConvertENUToNewOrigin(
          pole.bottom_point, &pole.bottom_point);
    }
    for (auto& sign : local_map_data_.second->semantic_map_data.traffic_signs) {
      CoordinateConverter::GetInstance()->ConvertENUToNewOrigin(sign.centroid,
                                                                &sign.centroid);
    }
  }
  // convert history trajectory to new origin
  for (auto& item : history_loc_pose_) {
    auto& pose = item.first;
    CoordinateConverter::GetInstance()->ConvertENUToNewOrigin(
        pose.translation(), &pose.translation());
  }
  for (auto& pose : history_ins_pose_) {
    CoordinateConverter::GetInstance()->ConvertENUToNewOrigin(
        pose.translation(), &pose.translation());
  }

  return LOC_SUCCESS;
}

void LocalizationVisualizer::SetThreadName() {
  pthread_setname_np(thread_->native_handle(), "LC_visualizer");
}

void LocalizationVisualizer::SetLocalizationInfo(
    const NavStateInfo& loc_info, const std::vector<NavStatus>& ms_nav_status) {
  std::lock_guard<std::mutex> lock(loc_info_mu_);
  loc_info_ = loc_info;
  ms_nav_status_ = ms_nav_status;
}

void LocalizationVisualizer::SetDRInfo(const OdomStateInfo& dr_info) {
  std::lock_guard<std::mutex> lock(dr_info_mu_);
  dr_info_ = dr_info;
}

void LocalizationVisualizer::SetInsInfo(const NavState& ins_state,
                                        const InsStatus& ins_status) {
  std::lock_guard<std::mutex> lock(ins_info_mu_);
  ins_state_ = ins_state;
  ins_status_ = ins_status;
}

void LocalizationVisualizer::SetGnssInfo(const NavState& gnss_state,
                                         const GnssStatus& gnss_status) {
  std::lock_guard<std::mutex> lock(gnss_info_mu_);
  gnss_state_ = gnss_state;
  gnss_status_ = gnss_status;
}

void LocalizationVisualizer::SetMMVisImage(const cv::Mat& vis_image) {
  std::lock_guard<std::mutex> lock(mm_vis_image_mu_);
  mm_vis_image_ = vis_image;
}

void LocalizationVisualizer::SetPerceptData(
    uint64_t timestamp, const std::shared_ptr<PerceptData>& percept_data) {
  std::lock_guard<std::mutex> lock(percept_data_mu_);
  percept_data_list_[percept_data->camera_name] =
      std::make_pair(timestamp, percept_data);
}

void LocalizationVisualizer::SetLocalMapData(
    uint64_t timestamp, const std::shared_ptr<RoadStructure>& local_map_data) {
  std::lock_guard<std::mutex> lock(local_map_data_mu_);
  local_map_data_ = std::make_pair(
      timestamp,
      std::make_shared<RoadStructure>(*local_map_data));  // deep copy
}

void LocalizationVisualizer::Run() {
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(40));

    // check finish request
    if (CheckFinishRequest()) break;

    // check pause request
    if (CheckPauseRequest()) {
      SetPause();
      while (IsPaused()) WaitReleasePaused(1000);
    }

    // draw process core
    uint64_t timestamp = 0;
    cv::Mat draw_image;
    if (LOC_SUCCESS != DrawProcess(&timestamp, &draw_image)) continue;

    if (param_.visual_param.enable_video_record) {
      SaveVedio(timestamp, draw_image);
    }

    // uncomment this line if want to fix gui's position
    // cv::moveWindow("Localization GUI", 0, 0);
    cv::imshow("Localization GUI", draw_image);
    cv::waitKey(1);
  }
  SetFinish();
}

adLocStatus_t LocalizationVisualizer::DrawProcess(uint64_t* timestamp,
                                                  cv::Mat* draw_image) {
  if (!timestamp || !draw_image) return LOC_NULL_PTR;

  {
    std::lock_guard<std::mutex> lock(loc_info_mu_);
    *timestamp = loc_info_.measurement_time_ns;
  }

  *draw_image = cv::Mat::zeros(window_height_, window_width_, CV_8UC3);

  cv::Mat global_view;
  DrawGlobalView(&global_view);
  cv::Mat image_view;
  DrawImageView(&image_view);

  if (!image_view.empty() && !global_view.empty()) {
    image_view.copyTo(
        global_view(cv::Rect(0, map_window_height_ - image_view.rows - 10,
                             image_view.cols, image_view.rows)));
  }

  if (!global_view.empty()) {
    global_view.copyTo((*draw_image)(
        cv::Rect(mm_window_width_, 0, global_view.cols, global_view.rows)));
  }

  std::lock_guard<std::mutex> lock(mm_vis_image_mu_);
  if (!mm_vis_image_.empty()) {
    cv::Mat mm_view;
    cv::resize(mm_vis_image_, mm_view,
               cv::Size(mm_window_width_, mm_window_height_));
    mm_view.copyTo((*draw_image)(cv::Rect(0, 0, mm_view.cols, mm_view.rows)));
  }

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationVisualizer::DrawGlobalView(cv::Mat* draw_image) {
  if (!draw_image) return LOC_NULL_PTR;

  *draw_image = cv::Mat::zeros(map_window_height_, map_window_width_, CV_8UC3);
  DrawMap(draw_image);
  DrawPerception(draw_image);
  DrawTrajectory(draw_image);
  DrawLocInfo(draw_image);

  std::vector<double> longitudinal_metric{-120, -100, -80, -60, -40, -20, 0,
                                          20,   40,   60,  80,  100, 120};
  std::vector<double> lateral_metric{-25, -20, -15, -10, -5, 0,
                                     5,   10,  15,  20,  25};
  VisualizerUtil::DrawTickMark(longitudinal_metric, lateral_metric,
                               global_view_matric_, draw_image);

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationVisualizer::DrawImageView(cv::Mat* draw_image) {
  if (!draw_image) return LOC_NULL_PTR;

  static const double scale = 5.0;
  static const cv::Scalar laneline_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::RED);
  static const cv::Scalar curb_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::GREEN);
  static const cv::Scalar sign_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::YELLOW);
  static const cv::Scalar pole_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::GREEN);
  static const cv::Scalar camera_name_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::WHITE);
  static std::unordered_map<std::string, cv::Mat> last_images;

  std::lock_guard<std::mutex> lock(percept_data_mu_);
  *draw_image = cv::Mat::zeros(img_window_height_ * percept_data_list_.size(),
                               img_window_width_, CV_8UC3);
  int cnt = 0;
  for (const auto& item : percept_data_list_) {
    const auto& percept_data = item.second.second;
    const std::string& camera_name = percept_data->camera_name;
    cv::Mat raw_image = percept_data->raw_image.clone();
    if (raw_image.empty()) {
      if (last_images.find(camera_name) != last_images.end()) {
        last_images.at(camera_name)
            .copyTo(
                (*draw_image)(cv::Rect(0, cnt * img_window_height_,
                                       img_window_width_, img_window_height_)));
      }
      ++cnt;
      continue;
    }
    // draw camera name
    cv::putText(raw_image, camera_name, cv::Point2d(0, 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, camera_name_color);

    // draw percept lanelines
    const auto& lanelines = percept_data->lane_lines;
    for (const auto& item : lanelines) {
      std::vector<cv::Point2d> draw_pts;
      draw_pts.reserve(item.second.img_pts.size());
      for (size_t i = 0; i < item.second.img_pts.size(); i += 10) {
        auto& pt = item.second.img_pts[i];
        draw_pts.emplace_back(pt.x / scale, pt.y / scale);
      }
      cv::Scalar color = laneline_color;
      if (item.second.line_type == LineType::Curb) color = curb_color;
      VisualizerUtil::DrawLine(draw_pts, VisualizerUtil::DrawLineStyle::Point,
                               color, 1, &raw_image);
    }
    // draw percept traffic signs
    const auto& traffic_signs = percept_data->traffic_signs;
    for (const auto& item : traffic_signs) {
      auto& bbox = item.second.rect;
      cv::Point2d tl(bbox.center.x - bbox.width / 2,
                     bbox.center.y - bbox.length / 2);
      cv::Point2d br(bbox.center.x + bbox.width / 2,
                     bbox.center.y + bbox.length / 2);
      cv::rectangle(raw_image, cv::Point2d(tl.x / scale, tl.y / scale),
                    cv::Point2d(br.x / scale, br.y / scale), sign_color);
      cv::putText(raw_image, std::to_string(item.first),
                  cv::Point2d(tl.x / scale, tl.y / scale),
                  cv::FONT_HERSHEY_SIMPLEX, 0.4, sign_color);
    }
    // draw percept poles
    const auto& poles = percept_data->poles;
    for (const auto& item : poles) {
      auto& bbox = item.second.rect;
      cv::Point2d tl(bbox.center.x - bbox.width / 2,
                     bbox.center.y - bbox.length / 2);
      cv::Point2d br(bbox.center.x + bbox.width / 2,
                     bbox.center.y + bbox.length / 2);
      cv::rectangle(raw_image, cv::Point2d(tl.x / scale, tl.y / scale),
                    cv::Point2d(br.x / scale, br.y / scale), pole_color);
      cv::putText(raw_image, std::to_string(item.first),
                  cv::Point2d(tl.x / scale, tl.y / scale),
                  cv::FONT_HERSHEY_SIMPLEX, 0.4, pole_color);
    }

    if (!raw_image.empty()) {
      cv::resize(raw_image, raw_image,
                 cv::Size(img_window_width_, img_window_height_));
      raw_image.copyTo((*draw_image)(cv::Rect(
          0, cnt * img_window_height_, img_window_width_, img_window_height_)));
      last_images[camera_name] = raw_image.clone();
    }
    ++cnt;
  }

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationVisualizer::DrawTrajectory(cv::Mat* draw_image) {
  if (!draw_image) return LOC_NULL_PTR;

  static const cv::Scalar loc_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::YELLOW);
  static const cv::Scalar dr_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::GREEN);
  static const cv::Scalar ins_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::GRAY);
  static const cv::Scalar warning_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::RED);
  static const size_t buffer_size = 100;

  // set global localization pose
  {
    std::lock_guard<std::mutex> lock(loc_info_mu_);
    Eigen::Vector3d ypr{loc_info_.euler_angle.yaw, loc_info_.euler_angle.pitch,
                        loc_info_.euler_angle.roll};
    Eigen::Vector3d position{loc_info_.position.x, loc_info_.position.y,
                             loc_info_.position.z};
    // SE3d pose = SE3d(SO3d(ypr), position); old Sophus
    SE3d pose = SE3d(SO3d::exp(ypr), position);
    cv::Scalar draw_loc_color = loc_color;
    if (!history_loc_pose_.empty()) {
      static uint64_t last_frontend_timestamp = 0;
      // check lateral jump (>=3m/s) and longitudinal jump (>=40m/s,
      // 144km/h)
      double delta_time = (static_cast<double>(loc_info_.measurement_time_ns) -
                           static_cast<double>(last_frontend_timestamp)) *
                          1e-9;
      if (delta_time > 0.0) {
        SE3d& last_pose = history_loc_pose_.back().first;
        SE3d delta_pose = last_pose.inverse() * pose;
        double lateral_velocity =
            std::fabs(delta_pose.translation()(1) / delta_time);
        double longitudinal_velocity =
            std::fabs(delta_pose.translation()(0) / delta_time);
        if (lateral_velocity >= 3.0 || longitudinal_velocity >= 40.0) {
          draw_loc_color = warning_color;
        }
      }
      last_frontend_timestamp = loc_info_.measurement_time_ns;
    }
    history_loc_pose_.emplace_back(pose, draw_loc_color);
    if (history_loc_pose_.size() > buffer_size) {
      history_loc_pose_.erase(history_loc_pose_.begin());
    }
  }

  // set dr pose
  {
    std::lock_guard<std::mutex> lock(dr_info_mu_);
    Eigen::Vector3d ypr{dr_info_.euler_angle.yaw, dr_info_.euler_angle.pitch,
                        dr_info_.euler_angle.roll};
    Eigen::Vector3d position{dr_info_.position.x, dr_info_.position.y,
                             dr_info_.position.z};
    // SE3d dr_pose = SE3d(SO3d(ypr), position); old Sophus
    SE3d dr_pose = SE3d(SO3d::exp(ypr), position);
    cv::Scalar draw_dr_color = dr_color;
    if (!history_dr_pose_.empty()) {
      static uint64_t last_dr_timestamp = 0;
      // check lateral jump (>=3m/s) and longitudinal jump (>=40m/s,
      // 144km/h)
      double delta_time =
          (dr_info_.measurement_time_ns - last_dr_timestamp) * 1e-9;
      if (delta_time > 0.0) {
        SE3d& last_pose = history_dr_pose_.back().first;
        SE3d delta_pose = last_pose.inverse() * dr_pose;
        double lateral_velocity =
            std::fabs(delta_pose.translation()(1) / delta_time);
        double longitudinal_velocity =
            std::fabs(delta_pose.translation()(0) / delta_time);
        if (lateral_velocity >= 3.0 || longitudinal_velocity >= 40.0) {
          draw_dr_color = warning_color;
        }
      }
      last_dr_timestamp = dr_info_.measurement_time_ns;
    }
    history_dr_pose_.emplace_back(dr_pose, draw_dr_color);
    if (history_dr_pose_.size() > buffer_size) {
      history_dr_pose_.erase(history_dr_pose_.begin());
    }
  }

  // set ins pose
  {
    std::lock_guard<std::mutex> lock(ins_info_mu_);
    history_ins_pose_.emplace_back(ins_state_.pose);
    if (history_ins_pose_.size() > buffer_size) {
      history_ins_pose_.erase(history_ins_pose_.begin());
    }
  }

  // draw ins trajectory
  std::vector<cv::Point2d> draw_ins_points;
  draw_ins_points.reserve(history_ins_pose_.size());
  SE3d ref_pose = history_loc_pose_.back().first.inverse();
  for (int i = history_ins_pose_.size() - 1; i >= 0; i -= 3) {
    SE3d local_pose = ref_pose * history_ins_pose_[i];
    cv::Point2d pixel_pt;
    if (VisualizerUtil::CvtLocalPoint2Image(
            Point2D_t(local_pose.translation()(0), local_pose.translation()(1)),
            global_view_matric_, &pixel_pt)) {
      draw_ins_points.emplace_back(pixel_pt);
    }
  }
  for (int i = 0; i < draw_ins_points.size(); ++i) {
    cv::circle(*draw_image, draw_ins_points[i], 2, ins_color, 2);
    if (i < draw_ins_points.size() - 1) {
      cv::line(*draw_image, draw_ins_points[i], draw_ins_points[i + 1],
               ins_color, 1);
    }
  }

  // draw dr trajectory
  std::vector<std::pair<cv::Point2d, cv::Scalar>> draw_dr_points;
  draw_dr_points.reserve(history_dr_pose_.size());
  for (int i = history_dr_pose_.size() - 1; i >= 0; i -= 3) {
    SE3d local_pose =
        history_dr_pose_.back().first.inverse() * history_dr_pose_[i].first;
    cv::Point2d pixel_pt;
    if (VisualizerUtil::CvtLocalPoint2Image(
            Point2D_t(local_pose.translation()(0), local_pose.translation()(1)),
            global_view_matric_, &pixel_pt)) {
      draw_dr_points.emplace_back(pixel_pt, history_dr_pose_[i].second);
    }
  }
  for (int i = 0; i < draw_dr_points.size(); ++i) {
    cv::circle(*draw_image, draw_dr_points[i].first, 2,
               draw_dr_points[i].second, 2);
    if (i < draw_dr_points.size() - 1) {
      cv::line(*draw_image, draw_dr_points[i].first,
               draw_dr_points[i + 1].first, draw_dr_points[i].second, 1);
    }
  }

  // draw global localization trajectory
  std::vector<std::pair<cv::Point2d, cv::Scalar>> draw_loc_points;
  draw_loc_points.reserve(history_loc_pose_.size());
  for (int i = history_loc_pose_.size() - 1; i >= 0; i -= 3) {
    SE3d local_pose =
        history_loc_pose_.back().first.inverse() * history_loc_pose_[i].first;
    cv::Point2d pixel_pt;
    if (VisualizerUtil::CvtLocalPoint2Image(
            Point2D_t(local_pose.translation()(0), local_pose.translation()(1)),
            global_view_matric_, &pixel_pt)) {
      draw_loc_points.emplace_back(pixel_pt, history_loc_pose_[i].second);
    }
  }
  for (int i = 0; i < draw_loc_points.size(); ++i) {
    cv::circle(*draw_image, draw_loc_points[i].first, 2,
               draw_loc_points[i].second, 2);
    if (i < draw_loc_points.size() - 1) {
      cv::line(*draw_image, draw_loc_points[i].first,
               draw_loc_points[i + 1].first, draw_loc_points[i].second, 1);
    }
  }

  DrawCovEllipse(draw_image);
  DrawVehicle(draw_loc_points.front().first, 0.0, loc_color, draw_image);
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationVisualizer::DrawMap(cv::Mat* draw_image) {
  if (!draw_image) return LOC_NULL_PTR;

  static const cv::Scalar map_laneline_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::WHITE);
  static const cv::Scalar map_curb_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::GRAY);
  static const cv::Scalar map_pole_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::WHITE);
  static const cv::Scalar map_traffic_sign_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::WHITE);

  std::lock_guard<std::mutex> lock(local_map_data_mu_);
  if (!local_map_data_.second) return LOC_NULL_PTR;

  Eigen::Matrix4d T_vw;
  {
    std::lock_guard<std::mutex> lock(loc_info_mu_);
    Eigen::Vector3d ypr{loc_info_.euler_angle.yaw, loc_info_.euler_angle.pitch,
                        loc_info_.euler_angle.roll};
    Eigen::Vector3d position{loc_info_.position.x, loc_info_.position.y,
                             loc_info_.position.z};
    // SE3d pose = SE3d(SO3d(ypr), position); old Sophus
    SE3d pose = SE3d(SO3d::exp(ypr), position);
    T_vw = pose.inverse().matrix();
  }

  const auto& map_lines = local_map_data_.second->semantic_map_data.lines;
  const auto& map_poles = local_map_data_.second->semantic_map_data.poles;
  const auto& map_traffic_signs =
      local_map_data_.second->semantic_map_data.traffic_signs;

  // draw map lines
  for (const auto& item : map_lines) {
    std::unordered_map<int, std::tuple<std::vector<cv::Point2d>, cv::Scalar,
                                       VisualizerUtil::DrawLineStyle>>
        draw_pts_list;
    int draw_id = 0;  // increase when line type or line style changes
    bool last_is_curb = false;
    bool last_is_dashed = false;
    for (const auto& line_segment : item.line_segments) {
      if (line_segment.points.empty()) continue;

      bool is_curb = IsRoadSideLine(line_segment.line_type);
      bool is_dashed = IsDashedLine(line_segment.line_style);
      if (!draw_pts_list.empty()) {
        if ((last_is_curb != is_curb) || (last_is_dashed != is_dashed)) {
          ++draw_id;
        }
      }
      last_is_curb = is_curb;
      last_is_dashed = is_dashed;

      auto& draw_pts = draw_pts_list[draw_id];
      if (is_curb) {
        std::get<1>(draw_pts) = map_curb_color;
      } else {
        std::get<1>(draw_pts) = map_laneline_color;
      }
      if (is_dashed) {
        std::get<2>(draw_pts) = VisualizerUtil::DrawLineStyle::Dashed;
      } else {
        std::get<2>(draw_pts) = VisualizerUtil::DrawLineStyle::Solid;
      }

      double sample_dis = 40.0;
      if (is_dashed) sample_dis = 4.0;
      Eigen::Vector4d last_local_pt =
          T_vw * Eigen::Vector4d{line_segment.points[0].x,
                                 line_segment.points[0].y,
                                 line_segment.points[0].z, 1};
      for (const auto& map_pt : line_segment.points) {
        Eigen::Vector4d local_pt =
            T_vw * Eigen::Vector4d{map_pt.x, map_pt.y, map_pt.z, 1};
        // do point sample
        Eigen::Vector4d direction = (local_pt - last_local_pt).normalized();
        Eigen::Vector4d sample_pt = last_local_pt;
        while (std::fabs(sample_pt(0) - local_pt(0)) > sample_dis) {
          sample_pt = sample_pt + sample_dis * direction;
          cv::Point2d pixel_pt;
          if (VisualizerUtil::CvtLocalPoint2Image(
                  Point2D_t(sample_pt(0), sample_pt(1)), global_view_matric_,
                  &pixel_pt)) {
            std::get<0>(draw_pts).emplace_back(pixel_pt);
          }
        }
        last_local_pt = local_pt;
        cv::Point2d pixel_pt;
        if (VisualizerUtil::CvtLocalPoint2Image(
                Point2D_t(local_pt(0), local_pt(1)), global_view_matric_,
                &pixel_pt)) {
          std::get<0>(draw_pts).emplace_back(pixel_pt);
        }
      }
    }
    for (const auto& item : draw_pts_list) {
      const auto& pts = std::get<0>(item.second);
      const VisualizerUtil::DrawLineStyle& draw_style =
          std::get<2>(item.second);
      const cv::Scalar& color = std::get<1>(item.second);
      VisualizerUtil::DrawLine(pts, draw_style, color, 2.0, draw_image);
    }
  }
  // draw map poles
  for (const auto& item : map_poles) {
    Eigen::Vector4d local_pt =
        T_vw * Eigen::Vector4d{item.bottom_point.x, item.bottom_point.y,
                               item.bottom_point.z, 1};
    cv::Point2d pixel_pt;
    if (VisualizerUtil::CvtLocalPoint2Image(Point2D_t(local_pt(0), local_pt(1)),
                                            global_view_matric_, &pixel_pt)) {
      cv::circle(*draw_image, pixel_pt, 5, map_pole_color, -1);
    }
  }
  // draw map traffic signs
  for (const auto& item : map_traffic_signs) {
    Eigen::Vector4d local_pt =
        T_vw *
        Eigen::Vector4d{item.centroid.x, item.centroid.y, item.centroid.z, 1};
    cv::Point2d pixel_pt;
    if (VisualizerUtil::CvtLocalPoint2Image(Point2D_t(local_pt(0), local_pt(1)),
                                            global_view_matric_, &pixel_pt)) {
      cv::drawMarker(*draw_image, pixel_pt, map_traffic_sign_color,
                     cv::MARKER_SQUARE, 10, 2);
    }
  }
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationVisualizer::DrawPerception(cv::Mat* draw_image) {
  if (!draw_image) return LOC_NULL_PTR;

  static const cv::Scalar percept_laneline_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::RED);
  static const cv::Scalar percept_curb_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::GREEN);

  std::lock_guard<std::mutex> lock(percept_data_mu_);
  if (percept_data_list_.empty()) return LOC_NULL_PTR;

  for (const auto& item : percept_data_list_) {
    // draw percept lines
    const auto& percept_data = item.second.second;
    const auto& lane_lines = percept_data->lane_lines;
    for (const auto& lane_line : lane_lines) {
      const auto& poly_coef = lane_line.second.poly_coef;
      const auto& start_point = lane_line.second.start_point;
      const auto& end_point = lane_line.second.end_point;
      std::vector<cv::Point2d> draw_pts;
      for (double x = start_point.x; x < end_point.x; x += 4.0) {
        double y = poly_coef[3] + poly_coef[2] * x + poly_coef[1] * x * x +
                   poly_coef[0] * x * x * x;
        cv::Point2d pixel_pt;
        if (VisualizerUtil::CvtLocalPoint2Image(
                Point2D_t(x, y), global_view_matric_, &pixel_pt)) {
          draw_pts.emplace_back(pixel_pt);
        }
      }
      bool is_curb = IsRoadSideLine(lane_line.second.line_type);
      bool is_dashed = IsDashedLine(lane_line.second.line_style);
      VisualizerUtil::DrawLineStyle draw_style =
          is_dashed ? VisualizerUtil::DrawLineStyle::Dashed
                    : VisualizerUtil::DrawLineStyle::Solid;
      cv::Scalar color = is_curb ? percept_curb_color : percept_laneline_color;
      VisualizerUtil::DrawLine(draw_pts, draw_style, color, 2, draw_image);
    }

    // TODO(xxx): draw percept poles and traffic signs
  }

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationVisualizer::DrawCovEllipse(cv::Mat* draw_image) {
  if (!draw_image) return LOC_NULL_PTR;

  static const cv::Scalar loc_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::PINK);
  static const double ellipse_factor =
      global_view_matric_.lateral_scale_factor /
      20.0;  // magnification factor for better visualization

  std::lock_guard<std::mutex> lock(loc_info_mu_);
  cv::Point2d ellipse_center;
  if (VisualizerUtil::CvtLocalPoint2Image(Point2D_t(0, 0), global_view_matric_,
                                          &ellipse_center)) {
    double half_ellipse_x_axis = loc_info_.position_std.y / ellipse_factor;
    double half_ellipse_y_axis = loc_info_.position_std.x / ellipse_factor;
    half_ellipse_x_axis = std::min(half_ellipse_x_axis, 300.0);
    half_ellipse_y_axis = std::min(half_ellipse_y_axis, 300.0);

    // draw smm pose cov
    cv::ellipse(*draw_image, ellipse_center,
                cv::Size(half_ellipse_x_axis, half_ellipse_y_axis), 0, 0, 360,
                loc_color, 1);
  }

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationVisualizer::DrawVehicle(const cv::Point2d& position,
                                                  const double heading_angle,
                                                  const cv::Scalar& color,
                                                  cv::Mat* draw_image) const {
  if (!draw_image) return LOC_NULL_PTR;

  static const Eigen::Vector3d vehicle_size = TransformConfig::GetVehicleSize();
  static const double vehicle_lenght_in_image =
      vehicle_size(0) / global_view_matric_.longitudinal_scale_factor;
  static const double vehicle_width_in_image =
      vehicle_size(1) / global_view_matric_.lateral_scale_factor;

  std::vector<cv::Point2f> rect_points{
      cv::Point2f(position.x - vehicle_width_in_image * 0.5,
                  position.y - vehicle_lenght_in_image * 0.5),
      cv::Point2f(position.x + vehicle_width_in_image * 0.5,
                  position.y - vehicle_lenght_in_image * 0.5),
      cv::Point2f(position.x + vehicle_width_in_image * 0.5,
                  position.y + vehicle_lenght_in_image * 0.5),
      cv::Point2f(position.x - vehicle_width_in_image * 0.5,
                  position.y + vehicle_lenght_in_image * 0.5)};

  cv::Mat rot_mat = cv::getRotationMatrix2D(position, heading_angle, 1.0);
  cv::transform(rect_points, rect_points, rot_mat);

  for (size_t i = 1; i < rect_points.size(); ++i) {
    cv::line(*draw_image, rect_points[i - 1], rect_points[i], color);
  }
  cv::line(*draw_image, rect_points[0], rect_points[3], color);
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationVisualizer::DrawLocInfo(cv::Mat* draw_image) {
  if (!draw_image) return LOC_NULL_PTR;

  auto ToStringWithPrecision = [](double v, int n = 3) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(n) << v;
    return out.str();
  };

  static const cv::Scalar info_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::WHITE);
  static const cv::Scalar warning_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::RED);
  static const cv::Scalar dividing_line_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::GREEN);

  std::string ins_status_str;
  {
    std::lock_guard<std::mutex> ins_lock(ins_info_mu_);
    ins_status_str = GetInsStatusStr(ins_status_);
    ins_status_str = "INS status: " + ins_status_str;
  }

  std::string gnss_status_str;
  {
    std::lock_guard<std::mutex> gnss_lock(gnss_info_mu_);
    gnss_status_str = GetGnssStatusStr(gnss_status_);
    gnss_status_str = "GNSS status: " + gnss_status_str;
  }

  std::vector<std::pair<cv::Scalar, std::string>> nav_info;
  {
    std::lock_guard<std::mutex> lock(loc_info_mu_);
    // timestamp info
    std::string utc_time_text(
        "Localization UTC time: " +
        senseAD::Time::ToString(loc_info_.measurement_time_ns));
    nav_info.emplace_back(info_color, utc_time_text);

    // msf status
    std::string nav_status_str = GetNavStatusStr(loc_info_.nav_status);
    nav_status_str = "MSF status: " + nav_status_str;
    cv::Scalar nav_status_color = info_color;
    if (static_cast<int>(loc_info_.nav_status) < 4) {
      nav_status_color = warning_color;
    }
    if (ms_nav_status_.size() == 4) {
      nav_status_str += " (FE: " + GetNavStatusShortStr(ms_nav_status_[0]);
      nav_status_str += " |BE: " + GetNavStatusShortStr(ms_nav_status_[1]);
      nav_status_str += " |MSF: " + GetNavStatusShortStr(ms_nav_status_[2]);
      nav_status_str +=
          " |OUT: " + GetNavStatusShortStr(ms_nav_status_[3]) + ")";
    }

    nav_info.emplace_back(nav_status_color, nav_status_str);

    // gnss status
    nav_info.emplace_back(info_color, gnss_status_str);

    // ins status
    nav_info.emplace_back(info_color, ins_status_str);

    // origin(lng/lat/alt) info
    std::string origin_text(
        "Origin(lng/lat/alt): (" +
        ToStringWithPrecision(loc_info_.origin_position_lla.lon, 5) + "," +
        ToStringWithPrecision(loc_info_.origin_position_lla.lat, 5) + "," +
        ToStringWithPrecision(loc_info_.origin_position_lla.height, 5) + ")");
    nav_info.emplace_back(info_color, origin_text);

    // wgs84(lng/lat/alt) info
    std::string wgs84_text(
        "WGS84(lng/lat/alt): (" +
        ToStringWithPrecision(loc_info_.position_lla.lon, 5) + "," +
        ToStringWithPrecision(loc_info_.position_lla.lat, 5) + "," +
        ToStringWithPrecision(loc_info_.position_lla.height, 5) + ")");
    nav_info.emplace_back(info_color, wgs84_text);

    // angle info
    std::string rpy_text(
        "RPY: (" + ToStringWithPrecision(loc_info_.euler_angle.roll, 4) + "," +
        ToStringWithPrecision(loc_info_.euler_angle.pitch, 4) + "," +
        ToStringWithPrecision(loc_info_.euler_angle.yaw, 4) + ")");
    nav_info.emplace_back(info_color, rpy_text);

    // angle std info
    std::string rpy_std_text(
        "RPY std: (" + ToStringWithPrecision(loc_info_.attitude_std.x, 4) +
        "," + ToStringWithPrecision(loc_info_.attitude_std.y, 4) + "," +
        ToStringWithPrecision(loc_info_.attitude_std.z, 4) + ")");
    nav_info.emplace_back(info_color, rpy_std_text);

    // position info
    std::string position_text(
        "Position: (" + ToStringWithPrecision(loc_info_.position.x, 4) + "," +
        ToStringWithPrecision(loc_info_.position.y, 4) + "," +
        ToStringWithPrecision(loc_info_.position.z, 4) + ")");
    nav_info.emplace_back(info_color, position_text);

    // position std info
    std::string position_std_text(
        "Position std: (" + ToStringWithPrecision(loc_info_.position_std.x, 4) +
        "," + ToStringWithPrecision(loc_info_.position_std.y, 4) + "," +
        ToStringWithPrecision(loc_info_.position_std.z, 4) + ")");
    nav_info.emplace_back(info_color, position_std_text);

    // linear velocity info
    std::string linear_velocity_text(
        "Linear Velocity: (" +
        ToStringWithPrecision(loc_info_.linear_velocity.x, 4) + "," +
        ToStringWithPrecision(loc_info_.linear_velocity.y, 4) + "," +
        ToStringWithPrecision(loc_info_.linear_velocity.z, 4) + ")");
    nav_info.emplace_back(info_color, linear_velocity_text);

    // angle velocity info
    std::string angular_velocity_text(
        "Angular Velocity: (" +
        ToStringWithPrecision(loc_info_.angular_velocity.x, 4) + "," +
        ToStringWithPrecision(loc_info_.angular_velocity.y, 4) + "," +
        ToStringWithPrecision(loc_info_.angular_velocity.z, 4) + ")");
    nav_info.emplace_back(info_color, angular_velocity_text);

    // linear acceleration info
    std::string linear_accel_text(
        "Linear Accelera: (" +
        ToStringWithPrecision(loc_info_.linear_acceleration.x, 4) + "," +
        ToStringWithPrecision(loc_info_.linear_acceleration.y, 4) + "," +
        ToStringWithPrecision(loc_info_.linear_acceleration.z, 4) + ")");
    nav_info.emplace_back(info_color, linear_accel_text);
  }

  cv::Point draw_pt(10, 20);
  for (size_t i = 0; i < nav_info.size(); ++i) {
    draw_pt.y = draw_pt.y + 20;
    cv::putText(*draw_image, nav_info[i].second, draw_pt,
                cv::FONT_HERSHEY_TRIPLEX, 0.4, nav_info[i].first);
  }

  // draw a dividing line
  cv::line(*draw_image, cv::Point(0, draw_pt.y + 5),
           cv::Point(300, draw_pt.y + 5), dividing_line_color, 1);

  std::vector<std::pair<cv::Scalar, std::string>> dr_info;
  {
    std::lock_guard<std::mutex> lock(dr_info_mu_);
    // timestamp info
    std::string utc_time_text(
        "DR UTC time: " +
        senseAD::Time::ToString(dr_info_.measurement_time_ns));
    dr_info.emplace_back(info_color, utc_time_text);

    // dr status info
    std::string odo_status_str = GetOdomStatusStr(dr_info_.odom_status);
    std::string odom_status_text("ODOM status: " + odo_status_str);
    cv::Scalar odo_status_color = info_color;
    if (static_cast<int>(dr_info_.odom_status) < 2) {
      odo_status_color = warning_color;
    }
    dr_info.emplace_back(odo_status_color, odom_status_text);

    // orign id
    std::string odom_origin_text("Origin id: " +
                                 std::to_string(dr_info_.origin_id));
    dr_info.emplace_back(info_color, odom_origin_text);

    // position info
    std::string position_text(
        "Position: (" + ToStringWithPrecision(dr_info_.position.x, 4) + "," +
        ToStringWithPrecision(dr_info_.position.y, 4) + "," +
        ToStringWithPrecision(dr_info_.position.z, 4) + ")");
    dr_info.emplace_back(info_color, position_text);

    // position std info
    std::string position_std_text(
        "Position std: (" + ToStringWithPrecision(dr_info_.position_std.x, 4) +
        "," + ToStringWithPrecision(dr_info_.position_std.y, 4) + "," +
        ToStringWithPrecision(dr_info_.position_std.z, 4) + ")");
    dr_info.emplace_back(info_color, position_std_text);

    // angle info
    std::string rpy_text(
        "RPY: (" + ToStringWithPrecision(dr_info_.euler_angle.roll, 4) + "," +
        ToStringWithPrecision(dr_info_.euler_angle.pitch, 4) + "," +
        ToStringWithPrecision(dr_info_.euler_angle.yaw, 4) + ")");
    dr_info.emplace_back(info_color, rpy_text);

    // angle std info
    std::string rpy_std_text(
        "RPY std: (" + ToStringWithPrecision(dr_info_.attitude_std.x, 4) + "," +
        ToStringWithPrecision(dr_info_.attitude_std.y, 4) + "," +
        ToStringWithPrecision(dr_info_.attitude_std.z, 4) + ")");
    dr_info.emplace_back(info_color, rpy_std_text);

    // linear velocity info
    std::string linear_velocity_text(
        "Linear Velocity: (" +
        ToStringWithPrecision(dr_info_.linear_velocity.x, 4) + "," +
        ToStringWithPrecision(dr_info_.linear_velocity.y, 4) + "," +
        ToStringWithPrecision(dr_info_.linear_velocity.z, 4) + ")");
    dr_info.emplace_back(info_color, linear_velocity_text);

    // angle velocity info
    std::string angular_velocity_text(
        "Angular Velocity: (" +
        ToStringWithPrecision(dr_info_.angular_velocity.x, 4) + "," +
        ToStringWithPrecision(dr_info_.angular_velocity.y, 4) + "," +
        ToStringWithPrecision(dr_info_.angular_velocity.z, 4) + ")");
    dr_info.emplace_back(info_color, angular_velocity_text);

    // linear acceleration info
    std::string linear_accel_text(
        "Linear Acceleration: (" +
        ToStringWithPrecision(dr_info_.linear_acceleration.x, 4) + "," +
        ToStringWithPrecision(dr_info_.linear_acceleration.y, 4) + "," +
        ToStringWithPrecision(dr_info_.linear_acceleration.z, 4) + ")");
    dr_info.emplace_back(info_color, linear_accel_text);
  }

  for (size_t i = 0; i < dr_info.size(); ++i) {
    draw_pt.y = draw_pt.y + 20;
    cv::putText(*draw_image, dr_info[i].second, draw_pt,
                cv::FONT_HERSHEY_TRIPLEX, 0.4, dr_info[i].first);
  }

  return LOC_SUCCESS;
}

void LocalizationVisualizer::SaveVedio(const uint64_t& timestamp_ns,
                                       const cv::Mat& draw_image) const {
#if CV_MAJOR_VERSION == 4
#define XVID_FOURCC cv::VideoWriter::fourcc
#else
#define XVID_FOURCC CV_FOURCC
#endif

  static const double fps = 30.0;
  static const cv::Size frame_size(window_width_, window_height_);
  static cv::VideoWriter vedio_writer(
      JOIN_PATH(param_.ci_param.results_save_dir, "video_record.avi"),
      XVID_FOURCC('X', 'V', 'I', 'D'), fps, frame_size);
  static std::ofstream timestamp_writer(
      JOIN_PATH(param_.ci_param.results_save_dir, "video_timestamp.txt"));
  static uint64_t last_timestamp_ns = 0;

  if (timestamp_ns == last_timestamp_ns || draw_image.empty()) return;
  if (vedio_writer.isOpened()) {
    vedio_writer << draw_image;
  } else {
    LC_LINFO_EVERY(SYSTEM, 100) << "video open fail";
  }
  if (timestamp_writer.is_open()) {
    timestamp_writer << timestamp_ns << "\n";
    timestamp_writer.flush();
  } else {
    LC_LINFO_EVERY(SYSTEM, 100) << "video timestamp txt open fail";
  }
  last_timestamp_ns = timestamp_ns;
}

}  // namespace localization
}  // namespace senseAD
