/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */
#include "semantic_mm/visual/visualization.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include <Sophus/se3.hpp>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>

#include "ad_time/ad_time.hpp"
#include "semantic_mm/base/lane_line.hpp"
#include "semantic_mm/base/line_segment.hpp"
#include "semantic_mm/common/configure.hpp"
#include "semantic_mm/common/math_tools.hpp"
#include "semantic_mm/frame/frame.hpp"
#include "semantic_mm/frame/frame_package.hpp"
#include "semantic_mm/map/map_manager.hpp"
#include "semantic_mm/tracking/tracking_manager.hpp"

namespace senseAD {
namespace localization {
namespace smm {

cv::Mat Visualization::Draw(const FramePackage::Ptr& frame_package,
                            const MapManager::Ptr& map_manager,
                            const TrackingManager::Ptr& tracking_manager) {
  cv::Mat draw_image = cv::Mat::zeros(image_height_, image_width_, CV_8UC3);
  DrawRoadLineDataOptimPose(frame_package, map_manager, tracking_manager,
                            &draw_image);
  auto smm_param = Configure::GetInstance()->GetLocalizationSMMParam();
  if (smm_param.enable_calib_homography) {
    DrawRoadLineDataCalibHomo(frame_package, &draw_image);
  }
  DrawPoseAndCovEllipse(frame_package, &draw_image);
  MakeSMMInfo(frame_package);
  DrawSMMInfo(&draw_image);

  std::vector<double> longitudinal_metric{-20, 0, 20, 40};
  std::vector<double> lateral_metric{-15, -10, -5, 0, 5, 10, 15};
  VisualizerUtil::DrawTickMark(longitudinal_metric, lateral_metric,
                               bv_view_matric_, &draw_image);

  if (save_image_ && frame_package->GetSMMStatus()) {
    SaveImage(draw_image);
  }

  return draw_image;
}

void Visualization::MakeSMMInfo(
    const std::shared_ptr<FramePackage>& frame_package) {
  if (!frame_package) return;

  auto ToStringWithPrecision = [](double v, int n = 3) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(n) << v;
    return out.str();
  };

  static const cv::Scalar info_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::WHITE);
  static const cv::Scalar warning_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::RED);

  smm_info_.clear();
  uint64_t timestamp = frame_package->GetFrames().front()->GetTimestamp();
  std::string time_str =
      std::to_string(timestamp) + "/" + senseAD::Time::ToString(timestamp);

  if (frame_package->GetSMMStatus()) {
    smm_info_.emplace_back(info_color, "SMM Success, timestamp " + time_str);
  } else {
    smm_info_.emplace_back(info_color, "SMM Fail, timestamp " + time_str);
  }

  // generate info of input pose error
  NavState gt_state;
  frame_package->GetGTState(&gt_state);
  if (gt_state.timestamp > 0) {
    SE3d input_pose = frame_package->GetInitPoseState();
    SE3d Tgt_input = gt_state.pose.inverse() * input_pose;
    Eigen::Vector3d to_gt_translation = Tgt_input.translation().matrix();
    double long_error = to_gt_translation(0);
    double lateral_error = to_gt_translation(1);
    Eigen::Matrix3d to_gt_rotation = Tgt_input.so3().matrix();
    double yaw_error =
        std::atan2(to_gt_rotation(1, 0), to_gt_rotation(0, 0)) * 57.3;
    std::string info =
        "init_long_err: " + ToStringWithPrecision(long_error) +
        " init_lat_err: " + ToStringWithPrecision(lateral_error) +
        ", init_yaw_err: " + ToStringWithPrecision(yaw_error);
    if (fabs(lateral_error) > 0.5 || fabs(yaw_error) > 1.5) {
      smm_info_.emplace_back(warning_color, info);
    } else {
      smm_info_.emplace_back(info_color, info);
    }
  } else {
    std::string info =
        "init_long_err: nan, init_lat_err: nan, init_yaw_err: nan";
    smm_info_.emplace_back(info_color, info);
  }

  // generate info of refined pose error
  bool smm_success = frame_package->GetSMMStatus();
  if (gt_state.timestamp > 0 && smm_success) {
    SE3d refined_pose = frame_package->GetPoseState();
    SE3d Tgt_refined = gt_state.pose.inverse() * refined_pose;
    Eigen::Vector3d to_gt_translation = Tgt_refined.translation().matrix();
    double long_error = to_gt_translation(0);
    double lateral_error = to_gt_translation(1);
    Eigen::Matrix3d to_gt_rotation = Tgt_refined.so3().matrix();
    double yaw_error =
        std::atan2(to_gt_rotation(1, 0), to_gt_rotation(0, 0)) * 57.3;
    std::string info = "opt_long_err: " + ToStringWithPrecision(long_error) +
                       " opt_lat_err: " + ToStringWithPrecision(lateral_error) +
                       ", opt_yaw_err: " + ToStringWithPrecision(yaw_error);
    if (fabs(lateral_error) > 0.5 || fabs(yaw_error) > 1.5) {
      smm_info_.emplace_back(warning_color, info);
    } else {
      smm_info_.emplace_back(info_color, info);
    }
    lateral_err_ = lateral_error;
    heading_err_ = yaw_error;
  } else {
    std::string info = "opt_long_err: nan, opt_lat_err: nan, opt_yaw_err: nan";
    smm_info_.emplace_back(info_color, info);
    lateral_err_ = 0;
    heading_err_ = 0;
  }

  // generate info of refined pose standard deviation
  if (smm_success) {
    Eigen::Matrix<double, 6, 6> refined_pose_cov = frame_package->GetPoseCov();
    double long_std = std::sqrt(refined_pose_cov(0, 0));
    double lateral_std = std::sqrt(refined_pose_cov(1, 1));
    double yaw_std = std::sqrt(refined_pose_cov(5, 5)) * 57.3;
    std::string info = "opt_long_std: " + ToStringWithPrecision(long_std) +
                       ", opt_lat_std: " + ToStringWithPrecision(lateral_std) +
                       ", opt_yaw_std: " + ToStringWithPrecision(yaw_std);
    cv::Scalar color = info_color;
    if (lateral_std < std::fabs(lateral_err_) ||
        yaw_std < std::fabs(heading_err_))
      color = warning_color;
    smm_info_.emplace_back(color, info);
  } else {
    std::string info = "opt_long_std: nan, opt_lat_std: nan, opt_yaw_std: nan";
    smm_info_.emplace_back(info_color, info);
  }

  // generate info of gnss error
  NavState gnss_state;
  frame_package->GetGnssState(&gnss_state);
  if (gnss_state.timestamp > 0 && gt_state.timestamp > 0) {
    SE3d Tgt_gnss = gt_state.pose.inverse() * gnss_state.pose;
    Eigen::Vector3d to_gt_translation = Tgt_gnss.translation().matrix();
    double lateral_error = to_gt_translation(1);
    std::string info = "gnss_lat_err: " + ToStringWithPrecision(lateral_error);
    if (fabs(lateral_error) > 0.5) {
      smm_info_.emplace_back(warning_color, info);
    } else {
      smm_info_.emplace_back(info_color, info);
    }
  } else {
    std::string info = "gnss_lat_err: nan";
    smm_info_.emplace_back(info_color, info);
  }

  // generate info of camera names
  const auto& frames = frame_package->GetFrames();
  for (size_t i = 0; i < frames.size(); ++i) {
    std::string cam_info = frames[i]->GetCameraName();
    auto smm_param = Configure::GetInstance()->GetLocalizationSMMParam();
    if (smm_param.enable_calib_homography) {
      std::string h_pitch =
          ToStringWithPrecision(frames[i]->GetHomoPitch() * 57.3);
      cam_info += ", H pitch(deg) " + h_pitch;
    }
    smm_info_.emplace_back(
        VisualizerUtil::ColorRender(static_cast<VisualizerUtil::Color>(i)),
        cam_info);

    if (smm_param.enable_calib_homography) {
      auto chi2_errs = frames[i]->GetOptimHomoChi2Errors();
      std::string cam_info = "chi2 mean/max " +
                             ToStringWithPrecision(chi2_errs.first, 4) + "/" +
                             ToStringWithPrecision(chi2_errs.second, 4);
      smm_info_.emplace_back(
          VisualizerUtil::ColorRender(static_cast<VisualizerUtil::Color>(i)),
          cam_info);
    }
  }

  // generate time cost info
  std::vector<std::pair<std::string, double>> module_time_cost =
      frame_package->GetModuleTimeCost();
  smm_info_.emplace_back(info_color, "Time cost: ");
  double total = 0;
  for (const auto& cost : module_time_cost) {
    cv::Scalar color = cost.second > 1.0 ? warning_color : info_color;
    std::string cost_info =
        cost.first + ": " + ToStringWithPrecision(cost.second) + "ms";
    smm_info_.emplace_back(color, cost_info);
    total += cost.second;
  }
  smm_info_.emplace_back(info_color,
                         "Total: " + ToStringWithPrecision(total) + "ms");

  return;
}

void Visualization::DrawRoadLineDataOptimPose(
    const std::shared_ptr<FramePackage>& frame_package,
    const std::shared_ptr<MapManager>& map_manager,
    const std::shared_ptr<TrackingManager>& tracking_manager,
    cv::Mat* draw_image) const {
  if (!draw_image || !frame_package || !map_manager) return;

  static const cv::Scalar map_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::GRAY);
  static const cv::Scalar matching_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::WHITE);
  static const cv::Scalar percept_sign_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::YELLOW);
  static const cv::Scalar percept_pole_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::GREEN);

  auto smm_param = Configure::GetInstance()->GetLocalizationSMMParam();

  bool smm_success = frame_package->GetSMMStatus();
  SE3d Tw_main, T_optim_predict;
  if (smm_success) {
    Tw_main = frame_package->GetPoseState();
    T_optim_predict = Tw_main.inverse() * frame_package->GetInitPoseState();
  } else {
    Tw_main = frame_package->GetInitPoseState();
    T_optim_predict =
        SE3d(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  }

  static const std::vector<cv::Scalar> rgb_list{
      cv::Scalar(0, 0, 255),   cv::Scalar(0, 255, 0),
      cv::Scalar(255, 0, 0),   cv::Scalar(255, 255, 0),
      cv::Scalar(255, 0, 255), cv::Scalar(0, 255, 255)};

  // 1. draw local map data
  // map lanelines
  for (const auto& item : map_manager->GetLocalLaneLines()) {
    id_t ll_id = item.first;
    for (const auto& line_segment : item.second->GetLineSegments()) {
      const auto& seg_points = line_segment->GetPoints();
      if (seg_points.empty()) continue;

      std::vector<cv::Point2d> draw_pts;
      int sum_cnt = 0;
      cv::Point2d draw_id_pt(0, 0);
      draw_pts.reserve(seg_points.size());
      for (const auto& pt_pred : seg_points) {
        Point3D_t pt = T_optim_predict * pt_pred;
        cv::Point2d cv_pt;
        if (VisualizerUtil::CvtLocalPoint2Image(Point2D_t(pt.x, pt.y),
                                                bv_view_matric_, &cv_pt)) {
          draw_pts.emplace_back(cv_pt);
          draw_id_pt += cv_pt;
          ++sum_cnt;
        }
      }
      if (draw_pts.empty()) continue;

      LineType line_type = line_segment->GetLineType();
      LineStyle line_style = line_segment->GetLineStyle();
      bool is_curb = IsRoadSideLine(line_type);
      bool is_dashed = IsDashedLine(line_style);
      bool is_laneline = line_type == LineType::LaneMarking;
      for (size_t i = 0; i < draw_pts.size(); ++i) {
        // line type marker
        if (is_laneline) {
          cv::circle(*draw_image, draw_pts[i], 3, map_color, 1);
        } else if (is_curb) {
          cv::drawMarker(*draw_image, draw_pts[i], map_color,
                         cv::MARKER_TILTED_CROSS, 10, 1);
        }
        // line style marker
        if (is_dashed) {
          if (i > 0 && i % 2)
            cv::line(*draw_image, draw_pts[i - 1], draw_pts[i], map_color, 2);
        } else {
          if (i > 0)
            cv::line(*draw_image, draw_pts[i - 1], draw_pts[i], map_color, 2);
        }
      }
      draw_id_pt.x /= sum_cnt;
      draw_id_pt.y /= sum_cnt;
      draw_id_pt.x -= 35.0;
      cv::putText(*draw_image,
                  std::to_string(ll_id) + "/" +
                      std::to_string(static_cast<int>(line_style)),
                  draw_id_pt, cv::FONT_HERSHEY_TRIPLEX, 0.5,
                  cv::Scalar(255, 255, 255));
    }
  }

  // map traffic signs
  std::unordered_map<id_t, cv::Point2d> draw_map_semantic_pixels;
  for (const auto& sign : map_manager->GetLocalTrafficSigns()) {
    auto center = T_optim_predict * sign.second->GetCenter();
    cv::Point2d cv_pt;
    if (VisualizerUtil::CvtLocalPoint2Image(Point2D_t(center.x, center.y),
                                            bv_view_matric_, &cv_pt)) {
      cv::drawMarker(*draw_image, cv_pt, map_color, cv::MARKER_SQUARE, 15, 2);
      draw_map_semantic_pixels.insert({sign.first, cv_pt});
    }
  }

  // map traffic poles
  for (const auto& pole : map_manager->GetLocalPoles()) {
    auto bot_pt = T_optim_predict * pole.second->GetBottomPoint();
    cv::Point2d cv_pt;
    if (VisualizerUtil::CvtLocalPoint2Image(Point2D_t(bot_pt.x, bot_pt.y),
                                            bv_view_matric_, &cv_pt)) {
      cv::circle(*draw_image, cv_pt, 10, map_color, -1);
      draw_map_semantic_pixels.insert({pole.first, cv_pt});
    }
  }

  // 2. draw matched map laneline
  // const auto& matched_map_pts = frame_package->GetMatchedMapLanelineData();
  // for (size_t i = 0; i < matched_map_pts.size(); i += 2) {
  //     cv::Point2d cv_pt0, cv_pt1;
  //     auto pt0 = T_optim_predict * matched_map_pts[i];
  //     auto pt1 = T_optim_predict * matched_map_pts[i + 1];
  //     if (!VisualizerUtil::CvtLocalPoint2Image(Point2D_t(pt0.x, pt0.y),
  //                                              bv_view_matric_, &cv_pt0))
  //         continue;
  //     if (!VisualizerUtil::CvtLocalPoint2Image(Point2D_t(pt1.x, pt1.y),
  //                                              bv_view_matric_, &cv_pt1))
  //         continue;
  //     cv::circle(*draw_image, cv_pt0, 4, matching_color, -1);
  //     cv::circle(*draw_image, cv_pt1, 4, matching_color, -1);
  //     cv::line(*draw_image, cv_pt0, cv_pt1, matching_color, 2);
  // }

  // 3. draw matched perception laneline
  for (const Point3D_t& pt : frame_package->GetMatchedLanelineData()) {
    cv::Point2d cv_pt;
    if (VisualizerUtil::CvtLocalPoint2Image(Point2D_t(pt.x, pt.y),
                                            bv_view_matric_, &cv_pt)) {
      cv::drawMarker(*draw_image, cv_pt, matching_color, cv::MARKER_SQUARE, 10,
                     1);
    }
  }

  // 4. draw preprocessed perception data
  const auto& frames = frame_package->GetFrames();
  for (size_t i = 0; i < frames.size(); ++i) {
    const auto& frame = frames[i];
    SE3d Tmain_sub = frames.front()->GetNavState().pose.inverse() *
                     frame->GetNavState().pose;
    const auto& lanelines = frame->GetPerceptionData()->lane_lines;
    for (const auto& item : lanelines) {
      const auto& points = item.second.processed_bv_points;
      std::vector<cv::Point2d> draw_pts;
      draw_pts.reserve(points.size());
      Point3D_t last_pt;
      for (const auto& p : points) {
        Point3D_t pt = Tmain_sub * p;
        cv::Point2d cv_pt;
        if (VisualizerUtil::CvtLocalPoint2Image(Point2D_t(pt.x, pt.y),
                                                bv_view_matric_, &cv_pt)) {
          draw_pts.emplace_back(cv_pt);
          last_pt = pt;
        }
      }
      LineType line_type = item.second.line_type;
      LineStyle line_style = item.second.line_style;
      bool is_dashed = IsDashedLine(line_style);
      cv::Scalar color =
          VisualizerUtil::ColorRender(static_cast<VisualizerUtil::Color>(i));
      for (size_t j = 0; j < draw_pts.size(); ++j) {
        // line type marker
        if (line_type == LineType::LaneMarking) {
          cv::circle(*draw_image, draw_pts[j], 2, color, 1);
        } else if (line_type == LineType::Curb) {
          cv::drawMarker(*draw_image, draw_pts[j], color,
                         cv::MARKER_TILTED_CROSS, 10, 1);
        }
        if (is_dashed) {
          if (j > 0 && j % 2)
            cv::line(*draw_image, draw_pts[j - 1], draw_pts[j], color, 1);
        } else {
          if (j > 0)
            cv::line(*draw_image, draw_pts[j - 1], draw_pts[j], color, 1);
        }
      }
    }
  }

  if (!smm_param.enable_temporal_fusion) return;

  // 5. draw fused perception data
  if (tracking_manager) {
    const std::unordered_map<std::string, PerceptLaneLine::PtrUMap>&
        local_lanelines = tracking_manager->GetFusedLaneLines();
    for (const auto& cam_ll_pair : local_lanelines) {
      const auto& camera_name = cam_ll_pair.first;
      const auto& local_ll_single_cam = cam_ll_pair.second;
      for (const auto& item : local_ll_single_cam) {
        const auto& points = item.second->processed_bv_points;
        const auto& covs = item.second->processed_bv_point_covs;
        if (points.empty() || covs.empty() || points.size() != covs.size()) {
          continue;
        }

        std::vector<cv::Point2d> draw_pts;
        std::vector<std::pair<double, double>> draw_pt_stds;
        draw_pts.reserve(points.size());
        draw_pt_stds.reserve(points.size());
        for (size_t i = 0; i < points.size(); ++i) {
          const auto& pt = points[i];
          cv::Point2d cv_pt;
          if (VisualizerUtil::CvtLocalPoint2Image(Point2D_t(pt.x, pt.y),
                                                  bv_view_matric_, &cv_pt)) {
            draw_pts.emplace_back(cv_pt);
            double long_std = std::sqrt(covs[i](0, 0));
            double lat_std = std::sqrt(covs[i](1, 1));
            draw_pt_stds.emplace_back(long_std, lat_std);
          }
        }
        if (draw_pts.empty()) continue;

        id_t tracker_id = item.first;
        cv::Scalar color = rgb_list[tracker_id % 6];
        LineType line_type = item.second->line_type;

        cv::Point2d draw_id_pt(0, 0);
        int sum_cnt = 0;
        for (size_t i = 0; i < draw_pts.size(); ++i) {
          if (draw_pt_cov_) {
            // draw bv point cov
            int el_x = static_cast<int>(
                draw_pt_stds[i].second / bv_view_matric_.lateral_scale_factor +
                0.5);
            int el_y =
                static_cast<int>(draw_pt_stds[i].first /
                                     bv_view_matric_.longitudinal_scale_factor +
                                 0.5);
            el_x = std::max(el_x, 10);
            el_y = std::max(el_y, 10);
            cv::ellipse(*draw_image, draw_pts[i], cv::Size(el_x, el_y), 0, 0,
                        360, color * 0.45, 1);
          }
          if (line_type == LineType::LaneMarking) {
            cv::circle(*draw_image, draw_pts[i], 3, color, -1);
          } else if (line_type == LineType::Curb) {
            cv::drawMarker(*draw_image, draw_pts[i], color,
                           cv::MARKER_TILTED_CROSS, 10, 1);
          }
          draw_id_pt.x += draw_pts[i].x;
          draw_id_pt.y += draw_pts[i].y;
          ++sum_cnt;
        }
        draw_id_pt.x /= sum_cnt;
        draw_id_pt.y /= sum_cnt;
        draw_id_pt.x += 10.0;
        cv::putText(*draw_image, std::to_string(tracker_id), draw_id_pt,
                    cv::FONT_HERSHEY_SIMPLEX, 0.75, color);
      }
    }

    // tracked percept signs
    const std::unordered_map<std::string, PerceptTrafficSign::PtrUMap>&
        local_signs = tracking_manager->GetFusedSigns();
    std::unordered_map<id_t, cv::Point2d> draw_percept_semantic_pixels;
    for (const auto& item : local_signs) {
      const std::string& cam_name = item.first;
      const PerceptTrafficSign::PtrUMap& signs = item.second;
      cv::Scalar color =
          VisualizerUtil::ColorRender(VisualizerUtil::Color::RED);
      if (cam_name.find("30") != std::string::npos) {
        color = VisualizerUtil::ColorRender(VisualizerUtil::Color::GREEN);
      }
      for (const auto& id_and_data : signs) {
        const auto& id = id_and_data.first;
        Point3D_t& local_pt = id_and_data.second->processed_center;
        cv::Point2d curr_cv_pt;
        if (VisualizerUtil::CvtLocalPoint2Image(
                Point2D_t(local_pt.x, local_pt.y), bv_view_matric_,
                &curr_cv_pt)) {
          cv::drawMarker(*draw_image, curr_cv_pt, color, cv::MARKER_SQUARE, 15,
                         2);
          draw_percept_semantic_pixels.insert({id, curr_cv_pt});
        }
      }
    }

    // tracked percept poles
    const std::unordered_map<std::string, PerceptPole::PtrUMap>& local_poles =
        tracking_manager->GetFusedPoles();
    for (const auto& item : local_poles) {
      const std::string& cam_name = item.first;
      const PerceptPole::PtrUMap& poles = item.second;
      cv::Scalar color =
          VisualizerUtil::ColorRender(VisualizerUtil::Color::RED);
      if (cam_name.find("30") != std::string::npos) {
        color = VisualizerUtil::ColorRender(VisualizerUtil::Color::GREEN);
      }
      for (const auto& id_and_data : poles) {
        const auto& id = id_and_data.first;
        Point3D_t& local_pt = id_and_data.second->processed_center;
        cv::Point2d curr_cv_pt;
        if (VisualizerUtil::CvtLocalPoint2Image(
                Point2D_t(local_pt.x, local_pt.y), bv_view_matric_,
                &curr_cv_pt)) {
          cv::circle(*draw_image, curr_cv_pt, 10, color, -1);
          draw_percept_semantic_pixels.insert({id, curr_cv_pt});
        }
      }
    }

    // draw semantic match with map
    auto match_indices = tracking_manager->GetMatchIndex();
    for (auto match_frame : match_indices) {
      for (auto match_pair : match_frame.second->GetAllMatchInOneVec()) {
        id_t p_id = match_pair.first, m_id = match_pair.second;
        auto p_iter = draw_percept_semantic_pixels.find(p_id);
        if (p_iter == draw_percept_semantic_pixels.end()) continue;
        auto m_iter = draw_map_semantic_pixels.find(m_id);
        if (m_iter == draw_map_semantic_pixels.end()) continue;
        cv::line(*draw_image, p_iter->second, m_iter->second, matching_color,
                 2);
      }
    }
  }

  // 6. draw for relocalization mode
  if (!frame_package->GetRelocalizationMode()) return;

  // 8. draw percept polyfit line
  if (smm_param.use_reloc_new_version) {
    DrawPerceptPolyfitLine(frame_package, draw_image);
    DrawMapPolyfitLine(frame_package, T_optim_predict, draw_image);
    DrawSearchHeadingValidArray(frame_package, T_optim_predict, draw_image);
  }

  if (map_manager->GetLocalLaneLines().empty()) return;
  auto reloc_hf_distribs = frame_package->GetRelocalDistributions();
  if (reloc_hf_distribs.empty()) return;
  // draw prior
  {
    auto color = VisualizerUtil::ColorRender(VisualizerUtil::Color::YELLOW);
    std::vector<cv::Point2d> cv_pts;
    for (const auto& item : reloc_hf_distribs[0]) {
      double y = (T_optim_predict * Point3D_t(0, item.first, 0)).y;
      double score = item.second;
      double x_score = -19.9 + score * 60;
      cv::Point2d cv_pt;
      if (VisualizerUtil::CvtLocalPoint2Image(Point2D_t(x_score, y),
                                              bv_view_matric_, &cv_pt))
        cv_pts.emplace_back(cv_pt);
    }
    for (size_t i = 1; i < cv_pts.size(); ++i) {
      cv::line(*draw_image, cv_pts[i - 1], cv_pts[i], color, 2);
    }
  }
  // draw likelihood
  {
    auto color = VisualizerUtil::ColorRender(VisualizerUtil::Color::RED);
    std::vector<cv::Point2d> cv_pts;
    for (const auto& item : reloc_hf_distribs[1]) {
      double y = (T_optim_predict * Point3D_t(0, item.first, 0)).y;
      double score = item.second;
      double x_score = -19.9 + score * 60;
      cv::Point2d cv_pt;
      if (VisualizerUtil::CvtLocalPoint2Image(Point2D_t(x_score, y),
                                              bv_view_matric_, &cv_pt))
        cv_pts.emplace_back(cv_pt);
    }
    for (size_t i = 1; i < cv_pts.size(); ++i) {
      cv::line(*draw_image, cv_pts[i - 1], cv_pts[i], color, 2);
    }
  }
  // draw posterior
  {
    auto color = VisualizerUtil::ColorRender(VisualizerUtil::Color::GREEN);
    std::vector<cv::Point2d> cv_pts;
    for (const auto& item : reloc_hf_distribs[2]) {
      double y = (T_optim_predict * Point3D_t(0, item.first, 0)).y;
      double score = item.second;
      double x_score = -19.9 + score * 60;
      cv::Point2d cv_pt;
      if (VisualizerUtil::CvtLocalPoint2Image(Point2D_t(x_score, y),
                                              bv_view_matric_, &cv_pt))
        cv_pts.emplace_back(cv_pt);
    }
    for (size_t i = 1; i < cv_pts.size(); ++i) {
      cv::line(*draw_image, cv_pts[i - 1], cv_pts[i], color, 2);
    }
  }
  // draw peaks
  for (const auto& item : reloc_hf_distribs[3]) {
    double y = (T_optim_predict * Point3D_t(0, item.first, 0)).y;
    double score = item.second;
    double bar_x_start = -20;
    double bar_x_end = bar_x_start + score * 60;
    cv::Scalar color =
        VisualizerUtil::ColorRender(VisualizerUtil::Color::GREEN);
    cv::Point2d top, bot;
    VisualizerUtil::CvtLocalPoint2Image(Point2D_t(bar_x_end, y),
                                        bv_view_matric_, &top);
    VisualizerUtil::CvtLocalPoint2Image(Point2D_t(bar_x_start, y),
                                        bv_view_matric_, &bot);
    cv::line(*draw_image, bot, top, color, 3);
  }

  // draw relocal pose, in optim coordinate
  {
    SE3d T_init_correct =
        T_optim_predict * frame_package->GetInitialPoseCorrection().pose;
    double y_init = T_init_correct.translation()(1);
    // double yaw_init = T_init_correct.so3().toYPR()(0); old Sophus
    double yaw_init =
        T_init_correct.so3().unit_quaternion().toRotationMatrix().eulerAngles(
            2, 1, 0)(0);
    double corrected_heading = 0 + yaw_init;
    Point2D_t norm(1.0, std::tan(corrected_heading));
    norm = norm / norm.Norm();
    Point2D_t heading_end = Point2D_t(0, y_init) + norm * 10;
    auto color = VisualizerUtil::ColorRender(VisualizerUtil::Color::GREEN);
    cv::Point2d cv_end;
    VisualizerUtil::CvtLocalPoint2Image(heading_end, bv_view_matric_, &cv_end);
    cv::Point2d cv_start;
    VisualizerUtil::CvtLocalPoint2Image(Point2D_t(0, y_init), bv_view_matric_,
                                        &cv_start);
    cv::arrowedLine(*draw_image, cv_start, cv_end, color, 2);
  }

  return;
}

void Visualization::DrawPerceptPolyfitLine(
    const std::shared_ptr<FramePackage>& frame_package,
    cv::Mat* draw_image) const {
  Eigen::Vector4d left_param, right_param;
  bool valid = frame_package->GetPerceptPolyfitParam(&left_param, &right_param);
  auto color = valid ? VisualizerUtil::ColorRender(VisualizerUtil::Color::BLUE)
                     : VisualizerUtil::ColorRender(VisualizerUtil::Color::RED);

  SE3d transform(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

  // left line
  std::vector<cv::Point2d> left_cv_points;
  GeneratePolyfitLinePoints(left_param, transform, &left_cv_points);
  for (int j = 0; j < left_cv_points.size(); j++) {
    if (j == 0) continue;
    cv::line(*draw_image, left_cv_points[j - 1], left_cv_points[j], color, 5);
  }

  // right line
  std::vector<cv::Point2d> right_cv_points;
  GeneratePolyfitLinePoints(right_param, transform, &right_cv_points);
  for (int j = 0; j < right_cv_points.size(); j++) {
    if (j == 0) continue;
    cv::line(*draw_image, right_cv_points[j - 1], right_cv_points[j], color, 5);
  }
}

void Visualization::DrawMapPolyfitLine(
    const std::shared_ptr<FramePackage>& frame_package, const SE3d& transform,
    cv::Mat* draw_image) const {
  const auto& map_line_params = frame_package->GetMapLinePolyfitParam();
  static auto color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::ORANGE);
  for (const auto& map_line_param : map_line_params) {
    std::vector<cv::Point2d> cv_points;
    GeneratePolyfitLinePoints(map_line_param, transform, &cv_points);
    for (int j = 0; j < cv_points.size(); j++) {
      if (j == 0) continue;
      cv::line(*draw_image, cv_points[j - 1], cv_points[j], color, 2.);
    }
  }
}

void Visualization::DrawSearchHeadingValidArray(
    const std::shared_ptr<FramePackage>& frame_package, const SE3d& transform,
    cv::Mat* draw_image) const {
  static auto valid_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::GREEN);
  static auto invalid_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::GRAY);
  const auto& heading_valid_array = frame_package->GetSearchHeadingValidArray();
  int search_size = heading_valid_array.size();
  std::vector<std::pair<cv::Point2d, bool>> cv_points_valid_array;
  cv_points_valid_array.reserve(search_size);
  for (const auto item : heading_valid_array) {
    Point3D_t transformed_point = transform * Point3D_t(-5., item.first, 0);
    cv::Point2d cv_pt;
    if (VisualizerUtil::CvtLocalPoint2Image(
            Point2D_t(transformed_point.x, transformed_point.y),
            bv_view_matric_, &cv_pt)) {
      cv_points_valid_array.emplace_back(cv_pt, item.second);
    }
  }
  for (int i = 1; i < cv_points_valid_array.size(); i++) {
    if (cv_points_valid_array[i].second != cv_points_valid_array[i - 1].second)
      continue;
    if (cv_points_valid_array[i].second) {
      cv::line(*draw_image, cv_points_valid_array[i - 1].first,
               cv_points_valid_array[i].first, valid_color, 2.);
    } else {
      cv::line(*draw_image, cv_points_valid_array[i - 1].first,
               cv_points_valid_array[i].first, invalid_color, 2.);
    }
  }
}

void Visualization::GeneratePolyfitLinePoints(
    const Eigen::Vector4d& line_param, const SE3d& transform,
    std::vector<cv::Point2d>* cv_points) const {
  cv_points->clear();
  for (double pt_x = -10.0; pt_x <= 10.0; pt_x += 0.5) {
    int param_size = line_param.size();
    double pt_y = 0;
    for (int i = 0; i < param_size; i++) {
      pt_y += line_param[i] * std::pow(pt_x, param_size - 1 - i);
    }
    Point3D_t transformed_point = transform * Point3D_t(pt_x, pt_y, 0);
    cv::Point2d cv_pt;
    if (VisualizerUtil::CvtLocalPoint2Image(
            Point2D_t(transformed_point.x, transformed_point.y),
            bv_view_matric_, &cv_pt)) {
      cv_points->emplace_back(cv_pt);
    }
  }
}

void Visualization::DrawRoadLineDataCalibHomo(
    const std::shared_ptr<FramePackage>& frame_package,
    cv::Mat* draw_image) const {
  if (!draw_image || !frame_package) return;

  // set parameters for converting metric point to pixel
  static const int sub_image_width = 300;
  static const int sub_image_height = 400;
  static const double scale = 1.0;
  static const int sub_x = 10;
  static const int sub_y = image_height_ - sub_image_height - 20;
  static const VisualizerUtil::Metric2PixelPara convert_para{
      20.0 / sub_image_width, 45.0 / sub_image_height,
      Point2D_t(sub_image_width / 2.0, sub_image_height),
      Point2D_t(0.0, sub_image_width), Point2D_t(0.0, sub_image_height)};

  cv::Mat sub_image =
      cv::Mat::zeros(sub_image_height, sub_image_width, CV_8UC3);
  SE3d Tw_main = frame_package->GetInitPoseState();

  // 1. draw matched local map data
  const auto& matched_map_pts =
      frame_package->GetMatchedMapLanelineDataForHomoCalib();
  for (size_t i = 0; i < matched_map_pts.size(); i += 2) {
    cv::Point2d cv_pt0, cv_pt1;
    const auto& pt0 = matched_map_pts[i];
    const auto& pt1 = matched_map_pts[i + 1];
    if (!VisualizerUtil::CvtLocalPoint2Image(Point2D_t(pt0.x, pt0.y),
                                             convert_para, &cv_pt0))
      continue;
    if (!VisualizerUtil::CvtLocalPoint2Image(Point2D_t(pt1.x, pt1.y),
                                             convert_para, &cv_pt1))
      continue;
    cv::line(sub_image, cv_pt0, cv_pt1,
             VisualizerUtil::ColorRender(VisualizerUtil::Color::WHITE),
             2 * scale);
  }

  // 2. draw original/H calibrated perception data
  const auto& frames = frame_package->GetFrames();
  for (size_t i = 0; i < frames.size(); ++i) {
    const auto& frame = frames[i];
    const auto& camera_name = frame->GetCameraName();
    SE3d Tmain_sub = frames.front()->GetNavState().pose.inverse() *
                     frame->GetNavState().pose;
    const auto& lanelines = frame->GetPerceptionData()->lane_lines;

    // BV points use original H
    for (const auto& item : lanelines) {
      const auto& img_pts = item.second.processed_img_points;
      Eigen::Matrix3d H;
      Configure::GetInstance()->GetCameraHomography(camera_name, &H);
      std::vector<cv::Point2d> draw_pts;
      draw_pts.reserve(img_pts.size());
      for (const auto& img_pt : img_pts) {
        Point2D_t pt_bv_proj = H * img_pt;
        Point3D_t pt = Tmain_sub * Point3D_t(pt_bv_proj.x, pt_bv_proj.y, 0);
        cv::Point2d cv_pt;
        if (pt.x <= 40.0 && VisualizerUtil::CvtLocalPoint2Image(
                                Point2D_t(pt.x, pt.y), convert_para, &cv_pt)) {
          draw_pts.emplace_back(cv_pt);
        }
      }
      LineType line_type = item.second.line_type;
      cv::Scalar color(180, 180, 180);
      for (size_t j = 0; j < draw_pts.size(); ++j) {
        if (line_type == LineType::LaneMarking) {
          cv::circle(sub_image, draw_pts[j], 3 * scale, color, 1);
        } else if (line_type == LineType::Curb) {
          cv::drawMarker(sub_image, draw_pts[j], color, cv::MARKER_TILTED_CROSS,
                         10 * scale, 1);
        }
        if (j > 0)
          cv::line(sub_image, draw_pts[j - 1], draw_pts[j], color, 1 * scale);
      }
    }

    // BV points use calibrated H
    for (const auto& item : lanelines) {
      const auto& img_pts = item.second.processed_img_points;
      Eigen::Matrix3d H = frame->GetAdaptedHomography();
      std::vector<cv::Point2d> draw_pts;
      draw_pts.reserve(img_pts.size());
      for (const auto& img_pt : img_pts) {
        Point2D_t pt_bv_proj = H * img_pt;
        Point3D_t pt = Tmain_sub * Point3D_t(pt_bv_proj.x, pt_bv_proj.y, 0);
        cv::Point2d cv_pt;
        if (pt.x <= 40.0 && VisualizerUtil::CvtLocalPoint2Image(
                                Point2D_t(pt.x, pt.y), convert_para, &cv_pt)) {
          draw_pts.emplace_back(cv_pt);
        }
      }
      LineType line_type = item.second.line_type;
      cv::Scalar color =
          VisualizerUtil::ColorRender(static_cast<VisualizerUtil::Color>(i));
      for (size_t j = 0; j < draw_pts.size(); ++j) {
        if (line_type == LineType::LaneMarking) {
          cv::circle(sub_image, draw_pts[j], 3 * scale, color, 1);
        } else if (line_type == LineType::Curb) {
          cv::drawMarker(sub_image, draw_pts[j], color, cv::MARKER_TILTED_CROSS,
                         10 * scale, 1);
        }
        if (j > 0)
          cv::line(sub_image, draw_pts[j - 1], draw_pts[j], color, 1 * scale);
      }
    }
  }

  // attach sub image to draw image
  cv::Mat draw_region =
      (*draw_image)(cv::Rect(sub_x, sub_y, sub_image.cols, sub_image.rows));
  cv::addWeighted(sub_image, 0.8, draw_region, 0.2, 0, draw_region);
}

void Visualization::DrawVehicle(const cv::Point2d& position,
                                const double heading_angle,
                                const cv::Scalar& color,
                                cv::Mat* draw_image) const {
  if (!draw_image) return;

  static const int vehicle_mark_lenght = 20;
  static const int vehicle_mark_width = 5;

  double x_offset = std::tan(heading_angle) * vehicle_mark_lenght;
  cv::line(*draw_image,
           cv::Point(position.x - x_offset, position.y - vehicle_mark_lenght),
           cv::Point(position.x + x_offset, position.y + vehicle_mark_lenght),
           color, 2);
  cv::line(*draw_image, cv::Point(position.x - vehicle_mark_width, position.y),
           cv::Point(position.x + vehicle_mark_width, position.y), color, 2);
}

void Visualization::DrawPoseAndCovEllipse(
    const std::shared_ptr<FramePackage>& frame_package,
    cv::Mat* draw_image) const {
  if (!draw_image || !frame_package) return;

  static const cv::Scalar gnss_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::ORANGE);
  static const cv::Scalar smm_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::PINK);
  static const cv::Scalar gt_color =
      VisualizerUtil::ColorRender(VisualizerUtil::Color::CHARTREUSE);
  static const double ellipse_factor =
      bv_view_matric_.lateral_scale_factor /
      8.0;  // magnification factor for better visualization

  SE3d smm_pose;
  if (frame_package->GetSMMStatus()) {
    smm_pose = frame_package->GetPoseState();
  } else {
    smm_pose = frame_package->GetInitPoseState();
  }
  // draw gnss covariance ellipse
  NavState gt_state;
  frame_package->GetGTState(&gt_state);
  NavState gnss_state;
  frame_package->GetGnssState(&gnss_state);
  if (gnss_state.timestamp > 0) {
    // get ellipse main axes
    Eigen::Matrix3d gnss_position_cov = gnss_state.pose_cov.block<3, 3>(0, 0);
    Eigen::Matrix3d Rwv = smm_pose.so3().matrix();
    Eigen::Matrix3d local_cov = Rwv.transpose() * gnss_position_cov * Rwv;
    double half_ellipse_x_axis = std::sqrt(local_cov(1, 1)) / ellipse_factor;
    double half_ellipse_y_axis = std::sqrt(local_cov(0, 0)) / ellipse_factor;
    half_ellipse_x_axis = std::min(half_ellipse_x_axis, 300.0);
    half_ellipse_y_axis = std::min(half_ellipse_y_axis, 300.0);

    // get ellipse center
    Eigen::Vector3d delta_translation =
        smm_pose.inverse() * gnss_state.pose.translation();
    // fix position_x of gnss, because gnss frequency is too small
    delta_translation(0) = 0.0;
    cv::Point2d ellipse_center;
    if (VisualizerUtil::CvtLocalPoint2Image(
            Point2D_t(delta_translation(0), delta_translation(1)),
            bv_view_matric_, &ellipse_center)) {
      // draw gnss pose
      double yaw = 0.0;
      DrawVehicle(ellipse_center, yaw, gnss_color, draw_image);
      // draw gnss pose cov
      cv::ellipse(*draw_image, ellipse_center,
                  cv::Size(half_ellipse_x_axis, half_ellipse_y_axis), 0, 0, 360,
                  gnss_color, 2);
    }
  }

  // draw smm center and covariance ellipse
  cv::Point2d smm_center;
  if (VisualizerUtil::CvtLocalPoint2Image(Point2D_t(0, 0), bv_view_matric_,
                                          &smm_center)) {
    DrawVehicle(smm_center, 0.0, smm_color, draw_image);
  }
  if (frame_package->GetSMMStatus()) {
    Eigen::Matrix<double, 6, 6> smm_cov =
        frame_package->GetPoseCov();  // this covariance is in the
                                      // vehicle coordinate
    double lateral_std = std::sqrt(smm_cov(1, 1));
    double half_ellipse_x_axis = lateral_std / ellipse_factor;
    half_ellipse_x_axis = std::min(half_ellipse_x_axis, 300.0);
    double half_ellipse_y_axis = 0.0;
    cv::ellipse(*draw_image, smm_center,
                cv::Size(half_ellipse_x_axis, half_ellipse_y_axis), 0, 0, 360,
                smm_color, 2);
  }

  // draw gt(ins) pose
  if (gt_state.timestamp > 0) {
    SE3d T_smm_gt = smm_pose.inverse() * gt_state.pose;
    Eigen::Vector3d delta_translation = T_smm_gt.translation().matrix();
    Eigen::Matrix3d delta_rotation = T_smm_gt.so3().matrix();
    cv::Point2d gt_point;
    if (VisualizerUtil::CvtLocalPoint2Image(
            Point2D_t(delta_translation(0), delta_translation(1)),
            bv_view_matric_, &gt_point)) {
      double yaw = std::atan2(delta_rotation(1, 0), delta_rotation(0, 0));
      DrawVehicle(gt_point, yaw, gt_color, draw_image);
    }
  }
}

void Visualization::DrawSMMInfo(cv::Mat* draw_image) const {
  if (!draw_image) return;

  for (size_t i = 0; i < smm_info_.size(); ++i) {
    std::string info = smm_info_[i].second;
    cv::putText(*draw_image, info, cv::Point(10, 30 * i + 40),
                cv::FONT_HERSHEY_TRIPLEX, 0.45, smm_info_[i].first);
  }
  return;
}

void Visualization::SaveImage(const cv::Mat& draw_image) {
  static int save_cnt_{0};
  ++save_cnt_;
  if (std::fabs(lateral_err_) > 0.5 || std::fabs(heading_err_) > 1.0) {
    static int last_save_cnt_{0};
    if (save_cnt_ - last_save_cnt_ >= 10) {
      std::string out_dir = "/tmp/logs/smm_visual";
      if (!boost::filesystem::is_directory(out_dir)) {
        boost::filesystem::create_directories(out_dir);
      }
      char file_name[1024];
      snprintf(file_name, sizeof(file_name), "/smm_%d.jpg", save_cnt_);
      cv::imwrite(out_dir + file_name, draw_image);
      last_save_cnt_ = save_cnt_;
    }
  }
}

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
