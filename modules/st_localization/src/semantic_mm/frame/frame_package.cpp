/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */
#include "semantic_mm/frame/frame_package.hpp"

#include <algorithm>

#include "localization/common/log.hpp"
#include "semantic_mm/common/configure.hpp"
#include "semantic_mm/frame/frame.hpp"

namespace senseAD {
namespace localization {
namespace smm {

std::atomic<uint64_t> FramePackage::next_id_{0};

FramePackage::FramePackage(const std::vector<Frame::Ptr>& frames)
    : id_(next_id_++), frames_(frames) {
  init_pose_state_correction_.pose = SE3d();
  init_pose_state_correction_.pose_cov.setZero();
}

void FramePackage::SetId(uint64_t id) { id_ = id; }

uint64_t FramePackage::GetId() const { return id_; }

void FramePackage::SetInitPoseState(const SE3d& pose_state) {
  init_pose_state_ = pose_state;
}

SE3d FramePackage::GetInitPoseState() const { return init_pose_state_; }

void FramePackage::SetInitialPoseCorrection(const NavState& pose_state) {
  init_pose_state_correction_ = pose_state;
}

NavState FramePackage::GetInitialPoseCorrection() const {
  return init_pose_state_correction_;
}

void FramePackage::SetPoseState(const SE3d& pose_state) {
  pose_state_ = pose_state;
}

SE3d FramePackage::GetPoseState() const { return pose_state_; }

void FramePackage::SetPoseCov(const Eigen::Matrix<double, 6, 6>& pose_cov) {
  pose_cov_ = pose_cov;
}

Eigen::Matrix<double, 6, 6> FramePackage::GetPoseCov() const {
  return pose_cov_;
}

void FramePackage::AddFrame(const Frame::Ptr& frame) {
  frames_.push_back(frame);
}

void FramePackage::SetFrames(const std::vector<Frame::Ptr>& frames) {
  frames_ = frames;
}

adLocStatus_t FramePackage::GetFrame(const std::string& name,
                                     Frame::Ptr* frame) const {
  if (!frame) return LOC_NULL_PTR;
  auto it = std::find_if(
      frames_.begin(), frames_.end(),
      [&name](const Frame::Ptr& frm) { return frm->GetCameraName() == name; });
  if (it == frames_.end()) return LOC_OUT_OF_RANGE;
  *frame = *it;
  return LOC_SUCCESS;
}

void FramePackage::SetGTState(const NavState& gt_state) {
  gt_state_ = gt_state;
}

adLocStatus_t FramePackage::GetGTState(NavState* state) const {
  if (nullptr == state) return LOC_NULL_PTR;
  *state = gt_state_;
  return LOC_SUCCESS;
}

void FramePackage::SetGnssState(const NavState& gnss_state) {
  gnss_state_ = gnss_state;
}

adLocStatus_t FramePackage::GetGnssState(NavState* state) const {
  if (nullptr == state) return LOC_NULL_PTR;
  *state = gnss_state_;
  return LOC_SUCCESS;
}

const std::vector<Frame::Ptr>& FramePackage::GetFrames() const {
  return frames_;
}

void FramePackage::SetSMMStatus(bool smm_success) {
  smm_success_ = smm_success;
}

bool FramePackage::GetSMMStatus() const { return smm_success_; }

std::unordered_map<std::string, SE3d> FramePackage::GetTransformSubToMain(
    bool use_odom_state) const {
  std::unordered_map<std::string, SE3d> T_main_subs;
  for (const auto& frame : frames_) {
    SE3d Tw_main = use_odom_state ? frames_.front()->GetOdomState().pose
                                  : frames_.front()->GetNavState().pose;
    SE3d Tw_sub =
        use_odom_state ? frame->GetOdomState().pose : frame->GetNavState().pose;
    SE3d Tmain_sub = Tw_main.inverse() * Tw_sub;
    T_main_subs.insert({frame->GetCameraName(), Tmain_sub});
  }
  return T_main_subs;
}

std::unordered_map<std::string, double> FramePackage::GetHomoPitch() const {
  std::unordered_map<std::string, double> homo_pitch;
  for (const auto& frame : frames_) {
    homo_pitch.insert({frame->GetCameraName(), frame->GetHomoPitch()});
  }
  return homo_pitch;
}

adLocStatus_t FramePackage::PreProcess() {
  for (auto& frame : frames_) {
    auto status = frame->PreProcess();
    if (status != LOC_SUCCESS) {
      LC_LERROR(FRAME) << "frame preprocess falied. timestamp and camera name: "
                       << frame->GetTimestamp() << " "
                       << frame->GetCameraName();
      return status;
    }
  }

  auto smm_param = Configure::GetInstance()->GetLocalizationSMMParam();
  if (smm_param.enable_calib_homography) {
    CheckMultiCameraHCalibConsistency();
  }
  return LOC_SUCCESS;
}

void FramePackage::CheckMultiCameraHCalibConsistency() {
  if (frames_.size() < 2) return;
  double h_pitch_30 = 0, h_pitch_120 = 0;
  Frame::Ptr frame_30 = nullptr, frame_120 = nullptr;
  for (const auto& frame : frames_) {
    if (frame->GetCameraName().find("30") != std::string::npos) {
      h_pitch_30 = frame->GetHomoPitch();
      frame_30 = frame;
    }
    if (frame->GetCameraName().find("120") != std::string::npos) {
      h_pitch_120 = frame->GetHomoPitch();
      frame_120 = frame;
    }
  }
  if (frame_30 == nullptr || frame_120 == nullptr) return;

  double diff_mean = 0, diff_var = 0;
  int cnt = 0;
  Configure::GetInstance()->GetMultiCameraDiffPitch(&diff_mean, &diff_var,
                                                    &cnt);
  LC_LDEBUG(FRAME) << "diff pitch(deg) mean/std/cnt " << diff_mean * 57.3
                   << ", " << std::sqrt(diff_var) * 57.3 << ", " << cnt;
  if (cnt < 100) return;  // at least 100 frames for stability

  bool has_fov30 = std::fabs(h_pitch_30) > 1e-4;
  bool has_fov120 = std::fabs(h_pitch_120) > 1e-4;
  double diff_thres = std::max(2.0 * std::fabs(diff_mean), 0.25 * M_PI / 180.0);
  if (has_fov30 && has_fov120) {
    // both frame has estimation, check consistency
    if (std::fabs(h_pitch_30 - h_pitch_120) < std::fabs(diff_thres)) return;
    LC_LDEBUG(FRAME) << "multi camera pitch estimation from vanishing point "
                        "inconsistent!";
    if (std::fabs(h_pitch_30) > std::fabs(h_pitch_120)) {
      h_pitch_30 = h_pitch_120 + diff_mean;
      frame_30->SetHomoPitch(h_pitch_30);
    } else {
      h_pitch_120 = h_pitch_30 - diff_mean;
      frame_120->SetHomoPitch(h_pitch_120);
    }
  } else if (has_fov120) {
    // has only frame 120
    h_pitch_30 = h_pitch_120 + diff_mean;
    frame_30->SetHomoPitch(h_pitch_30);
  } else if (has_fov30) {
    // has only frame 30
    h_pitch_120 = h_pitch_30 - diff_mean;
    frame_120->SetHomoPitch(h_pitch_120);
  }
}

adLocStatus_t FramePackage::CheckPerceptionData(SMMEvalData* eval_data) const {
  if (!eval_data) {
    LC_LERROR(SMM) << "evaluation data object is nullptr.";
    return LOC_NULL_PTR;
  }

  for (const auto& frame : frames_) {
    // check pass if any frame has valid data
    auto status = frame->CheckPerceptionData(eval_data);
  }
  return LOC_SUCCESS;
}

void FramePackage::SetRelocalizationMode(bool relocal_mode) {
  relocalization_mode_ = relocal_mode;
}
bool FramePackage::GetRelocalizationMode() const {
  return relocalization_mode_;
}

void FramePackage::SetMatchedLanelineData(
    const std::vector<Point3D_t>& matched_points) {
  matched_laneline_points_ = matched_points;
}
const std::vector<Point3D_t>& FramePackage::GetMatchedLanelineData() const {
  return matched_laneline_points_;
}

void FramePackage::SetMatchedMapLanelineData(
    const std::vector<Point3D_t>& matched_map_points) {
  matched_map_laneline_points_ = matched_map_points;
}
const std::vector<Point3D_t>& FramePackage::GetMatchedMapLanelineData() const {
  return matched_map_laneline_points_;
}

void FramePackage::SetMatchedMapLanelineDataForHomoCalib(
    const std::vector<Point3D_t>& matched_map_points) {
  matched_map_laneline_points_homo_calib_ = matched_map_points;
}
const std::vector<Point3D_t>&
FramePackage::GetMatchedMapLanelineDataForHomoCalib() const {
  return matched_map_laneline_points_homo_calib_;
}

void FramePackage::SetRelocalDistributions(
    const std::vector<std::vector<std::pair<double, double>>>& distribs) {
  relocal_distribs_ = distribs;
}
const std::vector<std::vector<std::pair<double, double>>>&
FramePackage::GetRelocalDistributions() const {
  return relocal_distribs_;
}

void FramePackage::SetModuleTimeCost(
    const std::vector<std::pair<std::string, double>>& time_cost) {
  time_cost_ = time_cost;
}

std::vector<std::pair<std::string, double>> FramePackage::GetModuleTimeCost()
    const {
  return time_cost_;
}

void FramePackage::SetPerceptPolyfitParam(const Eigen::Vector4d& left_param,
                                          const Eigen::Vector4d& right_param,
                                          bool valid) {
  left_percept_line_param_ = left_param;
  right_percept_line_param_ = right_param;
  percept_line_param_valid_ = valid;
}

bool FramePackage::GetPerceptPolyfitParam(Eigen::Vector4d* left_param,
                                          Eigen::Vector4d* right_param) const {
  *left_param = left_percept_line_param_;
  *right_param = right_percept_line_param_;
  return percept_line_param_valid_;
}

void FramePackage::SetMapLinePolyfitParam(
    const std::vector<Eigen::Vector4d>& map_line_params) {
  map_line_params_ = map_line_params;
}

const std::vector<Eigen::Vector4d>& FramePackage::GetMapLinePolyfitParam()
    const {
  return map_line_params_;
}

void FramePackage::SetSearchHeadingValidArray(
    const std::vector<std::pair<double, bool>>& heading_valid_array) {
  heading_valid_array_ = heading_valid_array;
}

const std::vector<std::pair<double, bool>>&
FramePackage::GetSearchHeadingValidArray() const {
  return heading_valid_array_;
}
}  // namespace smm
}  // namespace localization
}  // namespace senseAD
