/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 * Zhu Kaiying <zhukaiying@sensetime.com>
 */

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

#include "common/thread_base.hpp"
#include "common/visualizer_util.hpp"
#include "localization/data_type/base.hpp"
#include "localization/data_type/gnss.hpp"
#include "localization/data_type/ins.hpp"
#include "localization/data_type/navstate_info.hpp"
#include "localization/data_type/odomstate_info.hpp"
#include "localization/data_type/percept_data.hpp"
#include "localization/data_type/road_structure.hpp"
#include "localization/localization_param.hpp"

namespace senseAD {
namespace localization {

// struct holds the localization visualizer
class LocalizationVisualizer : public ThreadBase {
 public:
  DEFINE_PTR(LocalizationVisualizer)

  LocalizationVisualizer() = default;
  LocalizationVisualizer(const LocalizationVisualizer&) = delete;
  LocalizationVisualizer& operator=(const LocalizationVisualizer&) = delete;
  ~LocalizationVisualizer();

  adLocStatus_t Init(const LocalizationParam& param);
  void SetThreadName() final;

  adLocStatus_t Restart(bool is_abs_loc_mode);

  adLocStatus_t SwitchOriginProc();

  void SetLocalizationInfo(const NavStateInfo& loc_info,
                           const std::vector<NavStatus>& ms_nav_status);
  void SetDRInfo(const OdomStateInfo& dr_info);
  void SetInsInfo(const NavState& ins_state, const InsStatus& ins_status);
  void SetGnssInfo(const NavState& gnss_state, const GnssStatus& gnss_status);

  void SetMMVisImage(const cv::Mat& vis_image);

  void SetPerceptData(uint64_t timestamp,
                      const std::shared_ptr<PerceptData>& percept_data);
  void SetLocalMapData(uint64_t timestamp,
                       const std::shared_ptr<RoadStructure>& local_map_data);

 private:
  // @brief: thread main
  void Run();

  adLocStatus_t DrawProcess(uint64_t* timestamp, cv::Mat* draw_image);

  adLocStatus_t DrawGlobalView(cv::Mat* draw_image);

  adLocStatus_t DrawImageView(cv::Mat* draw_image);

  adLocStatus_t DrawTrajectory(cv::Mat* draw_image);

  adLocStatus_t DrawMap(cv::Mat* draw_image);

  adLocStatus_t DrawPerception(cv::Mat* draw_image);

  adLocStatus_t DrawCovEllipse(cv::Mat* draw_image);

  // @brief: draw vehicle box
  adLocStatus_t DrawVehicle(const cv::Point2d& position,
                            const double heading_angle, const cv::Scalar& color,
                            cv::Mat* draw_image) const;

  // @brief: draw global and DR localization info
  adLocStatus_t DrawLocInfo(cv::Mat* draw_image);

  // @brief: save images to a vedio
  void SaveVedio(const uint64_t& timestamp_ns, const cv::Mat& draw_image) const;

 private:
  // Localization config parameters
  LocalizationParam param_;

  // displayed related data buffer
  std::mutex loc_info_mu_;
  NavStateInfo loc_info_;  // global localization infomation
  std::vector<NavStatus> ms_nav_status_;

  std::mutex dr_info_mu_;
  OdomStateInfo dr_info_;  // relative localization infomation

  std::mutex ins_info_mu_;
  NavState ins_state_;    // ins state
  InsStatus ins_status_;  // ins status

  std::mutex gnss_info_mu_;
  NavState gnss_state_;     // gnss state
  GnssStatus gnss_status_;  // gnss status

  std::mutex mm_vis_image_mu_;
  cv::Mat mm_vis_image_;  // map matching visualization image

  std::mutex percept_data_mu_;  // perception data
  std::unordered_map<std::string,
                     std::pair<uint64_t, std::shared_ptr<PerceptData>>>
      percept_data_list_;

  std::mutex local_map_data_mu_;  // local hdmap data
  std::pair<uint64_t, std::shared_ptr<RoadStructure>> local_map_data_;

  // window size parameters, unit: pixel
  const int img_window_width_ = 350;
  const int img_window_height_ = 250;
  const int mm_window_width_ = 800;
  const int mm_window_height_ = 1000;
  const int map_window_width_ = 900;
  const int map_window_height_ = 1000;
  const int window_width_ = mm_window_width_ + map_window_width_;
  const int window_height_ = 1000;

  // parameters for converting metric point to pixel
  const VisualizerUtil::Metric2PixelPara global_view_matric_{
      80.0 / map_window_width_, 250.0 / map_window_height_,
      Point2D_t(map_window_width_ * 0.5 + 150, map_window_height_ * 0.5 - 50),
      Point2D_t(200.0, map_window_width_), Point2D_t(50.0, map_window_height_)};

  // for trajectory visulization
  std::vector<std::pair<SE3d, cv::Scalar>> history_loc_pose_;
  std::vector<std::pair<SE3d, cv::Scalar>> history_dr_pose_;
  std::vector<SE3d> history_ins_pose_;
};

}  // namespace localization
}  // namespace senseAD
