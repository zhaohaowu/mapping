/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */
#include "semantic_mm/optim/optim_manager.hpp"
#include <ceres/ceres.h>

#include <algorithm>
#include <limits>
#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "factor_optimizer/factor/edge.hpp"
#include "factor_optimizer/factor/edge_p2l_2dspace.hpp"
#include "factor_optimizer/factor/edge_p2l_2dspace_ba.hpp"
#include "factor_optimizer/factor/edge_p2p_2dspace.hpp"
#include "factor_optimizer/factor/vertex.hpp"
#include "factor_optimizer/factor/vertex_1dof.hpp"
#include "factor_optimizer/factor/vertex_2dof.hpp"
#include "factor_optimizer/factor/vertex_homo_pitch.hpp"
#include "factor_optimizer/factor_optimizer.hpp"
#include "localization/common/log.hpp"
#include "semantic_mm/common/configure.hpp"
#include "semantic_mm/common/math_tools.hpp"
#include "semantic_mm/frame/frame.hpp"
#include "semantic_mm/frame/frame_package.hpp"
#include "semantic_mm/map/map_manager.hpp"
#include "semantic_mm/matching/matching_manager.hpp"
#include "semantic_mm/tracking/tracking_manager.hpp"

namespace senseAD {
namespace localization {
namespace smm {

static constexpr double r2d = 180 / M_PI;

OptimManager* OptimManager::GetInstance() {
  static OptimManager instance;
  return &instance;
}

adLocStatus_t OptimManager::OptimizePoseOnlyLaneLineInSpace(
    const std::shared_ptr<FramePackage>& frame_package,
    const std::shared_ptr<MapManager>& map_manager,
    const std::shared_ptr<MatchingManager>& matching_manager,
    const std::shared_ptr<TrackingManager>& tracking_manager) {
  auto smm_param = Configure::GetInstance()->GetLocalizationSMMParam();
  const auto& frames = frame_package->GetFrames();
  if (frames.empty()) {
    LC_LDEBUG(OPTIM) << "frames size is empty.";
    return LOC_INVALID;
  }
  if (map_manager == nullptr || matching_manager == nullptr ||
      (smm_param.enable_temporal_fusion && tracking_manager == nullptr)) {
    LC_LDEBUG(OPTIM) << "nullptr of instance module";
    return LOC_NULL_PTR;
  }

  // get local map elements
  const LaneLine::PtrUMap& map_local_lanelines =
      map_manager->GetLocalLaneLines();
  const Pole::PtrUMap& map_local_poles = map_manager->GetLocalPoles();
  const TrafficSign::PtrUMap& map_local_signs =
      map_manager->GetLocalTrafficSigns();

  // get perceptions and matches
  std::unordered_map<std::string, PerceptLaneLine::PtrUMap>
      multi_cam_percept_lanelines;
  std::unordered_map<std::string, PerceptPole::PtrUMap> multi_cam_percept_poles;
  std::unordered_map<std::string, PerceptTrafficSign::PtrUMap>
      multi_cam_percept_signs;
  std::unordered_map<std::string, MatchIndex::Ptr> multi_frame_match_index;
  if (smm_param.enable_temporal_fusion) {
    // get perceptions
    multi_cam_percept_lanelines = tracking_manager->GetFusedLaneLines();
    multi_cam_percept_poles = tracking_manager->GetFusedPoles();
    multi_cam_percept_signs = tracking_manager->GetFusedSigns();

    // get matches
    for (const auto& frame : multi_cam_percept_lanelines) {
      const auto& camera_name = frame.first;
      MatchIndex::Ptr match;
      if (LOC_SUCCESS != tracking_manager->GetMatchIndex(camera_name, &match))
        continue;
      multi_frame_match_index.insert({camera_name, match});
    }
  } else {
    for (size_t i = 0; i < frames.size(); ++i) {
      const auto& frame = frames[i];
      const auto& camera_name = frame->GetCameraName();
      // get perceptions
      const auto& percept_lines = frame->GetPerceptionData()->lane_lines;
      // TODO(xxx): save copy here
      for (const auto& line : percept_lines) {
        PerceptLaneLine::Ptr line_ptr(new PerceptLaneLine(line.second));
        multi_cam_percept_lanelines[camera_name][line.first] = line_ptr;
      }

      // get matches
      MatchIndex::Ptr match = frame->GetMatchIndex();
      multi_frame_match_index.insert({camera_name, match});
    }
  }

  bool only_fov30_perceptions = multi_cam_percept_lanelines.size() == 1 &&
                                multi_cam_percept_lanelines.begin()->first.find(
                                    "30") != std::string::npos;
  if (only_fov30_perceptions) {
    LC_LDEBUG(OPTIM) << "only has camera fov 30 perception, not optimize!";
    return LOC_LOCALIZATION_ERROR;
  }

  NavState nav_state_init_correct = frame_package->GetInitialPoseCorrection();
  bool relocalization_mode = frame_package->GetRelocalizationMode();
  SE3d T_init_correct = nav_state_init_correct.pose;
  if (relocalization_mode && smm_param.use_reloc_new_version) {
    SE3d reloc_pose = frame_package->GetInitPoseState() * T_init_correct;
    Eigen::Matrix<double, 6, 6> reloc_pose_cov =
        nav_state_init_correct.pose_cov;
    frame_package->SetPoseState(reloc_pose);
    frame_package->SetPoseCov(reloc_pose_cov);
    LC_LDEBUG(SMM) << "smm reloc new version process skip optimization!";
    return LOC_SUCCESS;
  }
  std::unordered_map<std::string, SE3d> T_main_subs =
      frame_package->GetTransformSubToMain(smm_param.enable_temporal_fusion);

  // find laneline point2line correlations for total frames
  static std::vector<P2LCorr> point2line_corrs;
  point2line_corrs.clear();
  for (const auto& cam_ll_pair : multi_cam_percept_lanelines) {
    const auto& camera_name = cam_ll_pair.first;
    const auto& percept_lls = cam_ll_pair.second;
    if (!multi_frame_match_index.count(camera_name)) {
      LC_LDEBUG(OPTIM) << "why not exist match for " << camera_name;
      continue;
    }
    if (!T_main_subs.count(camera_name)) {
      LC_LDEBUG(OPTIM) << "why not exist Tmain_sub for " << camera_name;
      continue;
    }
    MatchPairVec match_pairs;
    multi_frame_match_index.at(camera_name)
        ->GetSemanticMatch(SemanticType::LaneLine, &match_pairs);
    // NOTE: perception should be initially aligned to map with
    // T_map_percept
    SE3d T_map_percept = T_init_correct * T_main_subs.at(camera_name);
    FindLaneLineP2LAssociations(percept_lls, map_local_lanelines, match_pairs,
                                T_map_percept, &point2line_corrs);
  }
  if (point2line_corrs.size() < 5) {
    LC_LDEBUG(OPTIM) << "point 2 line correspodences less than 5!";
    return LOC_LOCALIZATION_ERROR;
  }

  // find pole point2point correlations for total frames
  static std::vector<P2PCorr> pole_corrs;
  pole_corrs.clear();
  FindPoleP2PAssociations(multi_cam_percept_poles, map_local_poles,
                          multi_frame_match_index, &pole_corrs);

  // find traffic sign point2point correlations for total frames
  static std::vector<P2PCorr> sign_corrs;
  sign_corrs.clear();
  FindTrafficSignP2PAssociations(multi_cam_percept_signs, map_local_signs,
                                 multi_frame_match_index, &sign_corrs);

  ////////////////////// construct problem and optimize //////////////////////
  // optim params
  static constexpr int max_iter_num = 5;
  FactorOptimizer::Ptr optimizer(new FactorOptimizer);
  optimizer->SetNumIters(max_iter_num);
  optimizer->SetLinearSolverType(ceres::DENSE_QR);

  // add vertex_2dof: y and heading
  static Vertex2DOF::Ptr vertex_2dof{nullptr};
  if (!vertex_2dof) {
    vertex_2dof = std::make_shared<Vertex2DOF>(0);
    static double constexpr y_range = 1.2;    // m
    static double constexpr yaw_range = 0.8;  // deg
    vertex_2dof->SetParameterLowerBound(0, -y_range);
    vertex_2dof->SetParameterUpperBound(0, y_range);
    vertex_2dof->SetParameterLowerBound(1, -yaw_range / r2d);
    vertex_2dof->SetParameterUpperBound(1, yaw_range / r2d);
  }
  vertex_2dof->SetParameter(Eigen::Vector2d::Zero());

  // add vertex_1dof: x
  static Vertex1DOF::Ptr vertex_1dof{nullptr};
  if (!vertex_1dof) {
    vertex_1dof = std::make_shared<Vertex1DOF>(1);
    static double constexpr x_range = 2.0;  // m
    vertex_1dof->SetParameterLowerBound(0, -x_range);
    vertex_1dof->SetParameterUpperBound(0, x_range);
  }
  vertex_1dof->SetParameter(Eigen::Matrix<double, 1, 1>::Zero());

  optimizer->AddVertex(vertex_2dof);
  optimizer->AddVertex(vertex_1dof);

  // add edge: P2L2DSpace
  int edge_cnt = 0;
  static std::vector<EdgeP2L2DSpace::Ptr> edge_p2ls;
  for (size_t i = 0; i < point2line_corrs.size(); ++i) {
    const auto& corr = point2line_corrs[i];
    // reduce weight of curb in optim
    double cov_factor = corr.type == LineType::Curb ? 2.0 : 1.0;
    EdgeP2L2DSpace::Ptr edge_p2l;
    if (i < edge_p2ls.size()) {
      edge_p2l = edge_p2ls[i];
      if (edge_p2l == nullptr) {
        LC_LERROR(SMM) << "why p2l edge is nullptr?";
        continue;
      }
      edge_p2l->Reset(edge_cnt, Eigen::Vector2d(corr.p.x, corr.p.y),
                      Eigen::Vector2d(corr.p_map_s.x, corr.p_map_s.y),
                      Eigen::Vector2d(corr.p_map_e.x, corr.p_map_e.y),
                      cov_factor * corr.cov);
    } else {
      edge_p2l = std::make_shared<EdgeP2L2DSpace>(
          edge_cnt, Eigen::Vector2d(corr.p.x, corr.p.y),
          Eigen::Vector2d(corr.p_map_s.x, corr.p_map_s.y),
          Eigen::Vector2d(corr.p_map_e.x, corr.p_map_e.y),
          cov_factor * corr.cov);
      edge_p2ls.emplace_back(edge_p2l);
    }
    edge_p2l->AddVertex(vertex_2dof);
    edge_p2l->SetOutlier(!corr.inlier);
    // NOTE: robust kernel may slow down convergency
    // edge_p2l->SetLossFunction(std::make_shared<ceres::CauchyLoss>(0.5));
    optimizer->AddEdge(edge_p2l);
    ++edge_cnt;
  }
  // add edge: P2P2DSpace
  static std::vector<EdgeP2P2DSpace::Ptr> edge_pole_p2ps;
  for (size_t i = 0; i < pole_corrs.size(); ++i) {
    const auto& corr = pole_corrs[i];
    EdgeP2P2DSpace::Ptr edge_p2p;
    if (i < edge_pole_p2ps.size()) {
      edge_p2p = edge_pole_p2ps[i];
      if (edge_p2p == nullptr) {
        LC_LERROR(SMM) << "why pole p2p edge is nullptr?";
        continue;
      }
      edge_p2p->Reset(edge_cnt, Eigen::Vector2d(corr.p.x, corr.p.y),
                      Eigen::Vector2d(corr.p_map.x, corr.p_map.y), corr.cov);
    } else {
      edge_p2p = std::make_shared<EdgeP2P2DSpace>(
          edge_cnt, Eigen::Vector2d(corr.p.x, corr.p.y),
          Eigen::Vector2d(corr.p_map.x, corr.p_map.y), corr.cov);
      edge_pole_p2ps.emplace_back(edge_p2p);
    }
    edge_p2p->AddVertex(vertex_2dof);
    edge_p2p->AddVertex(vertex_1dof);
    edge_p2p->SetOutlier(!corr.inlier);
    optimizer->AddEdge(edge_p2p);
    ++edge_cnt;
  }
  static std::vector<EdgeP2P2DSpace::Ptr> edge_sign_p2ps;
  for (size_t i = 0; i < sign_corrs.size(); ++i) {
    const auto& corr = sign_corrs[i];
    EdgeP2P2DSpace::Ptr edge_p2p;
    if (i < edge_sign_p2ps.size()) {
      edge_p2p = edge_sign_p2ps[i];
      if (edge_p2p == nullptr) {
        LC_LERROR(SMM) << "why sign p2p edge is nullptr?";
        continue;
      }
      edge_p2p->Reset(edge_cnt, Eigen::Vector2d(corr.p.x, corr.p.y),
                      Eigen::Vector2d(corr.p_map.x, corr.p_map.y), corr.cov);
    } else {
      edge_p2p = std::make_shared<EdgeP2P2DSpace>(
          edge_cnt, Eigen::Vector2d(corr.p.x, corr.p.y),
          Eigen::Vector2d(corr.p_map.x, corr.p_map.y), corr.cov);
      edge_sign_p2ps.emplace_back(edge_p2p);
    }
    edge_p2p->AddVertex(vertex_2dof);
    edge_p2p->AddVertex(vertex_1dof);
    edge_p2p->SetOutlier(!corr.inlier);
    optimizer->AddEdge(edge_p2p);
    ++edge_cnt;
  }

  // Optimize it!
  if (!optimizer->Optimize(true)) return LOC_LOCALIZATION_ERROR;
  size_t inlier_nums = point2line_corrs.size();
  double inlier_ratio = 1.0;
  int iter_cnt = 0;
  while (iter_cnt++ < 2) {
    auto& edges = optimizer->GetEdges();
    for (int outlier_remove_count = 0; outlier_remove_count < 3;
         ++outlier_remove_count) {
      inlier_nums = 0;
      for (auto edge : edges) {
        uint64_t id = edge.first;
        if (id >= point2line_corrs.size()) {
          // LC_LERROR(OPTIM) << "why beyond inline range.";
          continue;
        }
        bool is_inlier =
            edge.second->Chi2() <= chi2_2d_[outlier_remove_count + 2];
        edge.second->SetOutlier(!is_inlier);
        point2line_corrs.at(id).inlier = is_inlier;
        if (is_inlier) ++inlier_nums;
      }
      inlier_ratio = inlier_nums / static_cast<double>(point2line_corrs.size());
      if (inlier_ratio > 0.5) break;
    }
    if (inlier_nums < 3) break;

    if (!optimizer->Optimize(true)) return LOC_LOCALIZATION_ERROR;
  }

  // check inlier ratio
  LC_LDEBUG(OPTIM) << "point to line corrs inlier ratio/num: " << inlier_ratio
                   << "/" << inlier_nums;
  if (inlier_ratio < 0.2 || inlier_nums < 3) return LOC_LOCALIZATION_ERROR;

  // NOTE: transform perception points back to vehicle coordinate
  if (relocalization_mode) {
    for (auto& corr : point2line_corrs)
      corr.p = T_init_correct.inverse() * corr.p;
  }

  // pts for visualization
  auto& param = Configure::GetInstance()->GetLocalizationParam();
  if (param.visual_param.enable_display) {
    std::vector<Point3D_t> inlier_points;
    inlier_points.reserve(inlier_nums);
    std::vector<Point3D_t> inlier_map_points;
    inlier_map_points.reserve(inlier_nums * 2);
    for (size_t i = 0; i < point2line_corrs.size(); ++i) {
      auto& corr = point2line_corrs[i];
      if (!corr.inlier) continue;
      inlier_points.emplace_back(corr.p);
      inlier_map_points.emplace_back(corr.p_map_s);
      inlier_map_points.emplace_back(corr.p_map_e);
    }
    frame_package->SetMatchedLanelineData(inlier_points);
    frame_package->SetMatchedMapLanelineData(inlier_map_points);
  }

  // get optim pose
  Eigen::Vector2d yheading = vertex_2dof->GetParameter();
  double x = vertex_1dof->GetParameter()(0);
  // SE3d optimed_se3(SO3d(0, 0, yheading(1)),
  //                         Eigen::Vector3d(x, yheading(0), 0)); old Sophus
  SE3d optimed_se3(SO3d::exp(Eigen::Vector3d(0, 0, yheading(1))),
                   Eigen::Vector3d(x, yheading(0), 0));
  SE3d Tw_main = frame_package->GetInitPoseState();
  SE3d optimed_main_pose = Tw_main * optimed_se3 * T_init_correct;

  // estimated smm covariance
  // count matching errors first
  SE3d T_optim_predict =
      optimed_main_pose.inverse() * frame_package->GetInitPoseState();
  for (auto& corr : point2line_corrs) {
    corr.opt_p_map_s = T_optim_predict * corr.p_map_s;
    corr.opt_p_map_e = T_optim_predict * corr.p_map_e;
    double error_sign =
        corr.p.y > (corr.opt_p_map_s.y + corr.opt_p_map_e.y) / 2 ? 1.0 : -1.0;
    corr.error = error_sign *
                 Point2LineDist2D(corr.p, corr.opt_p_map_s, corr.opt_p_map_e);
  }
  Eigen::Matrix<double, 6, 6> pose_cov;
  auto status = EstimateSMMCovariance(optimizer, point2line_corrs, pole_corrs,
                                      sign_corrs, &pose_cov);
  if (status != LOC_SUCCESS) return status;
  pose_cov(1, 1) += nav_state_init_correct.pose_cov(1, 1);
  pose_cov(5, 5) += nav_state_init_correct.pose_cov(5, 5);

  frame_package->SetPoseState(optimed_main_pose);
  frame_package->SetPoseCov(pose_cov);

  LC_LDEBUG(OPTIM) << "solve optimized x, y and yaw: " << x << " "
                   << yheading(0) << " " << yheading(1) * r2d
                   << ", x_std, y_std and yaw_std: "
                   << std::sqrt(pose_cov(0, 0)) << " "
                   << std::sqrt(pose_cov(1, 1)) << " "
                   << std::sqrt(pose_cov(5, 5)) * r2d;

  return LOC_SUCCESS;
}

adLocStatus_t OptimManager::OptimizeHomographyOnlyLaneLineInSpace(
    const std::shared_ptr<FramePackage>& frame_package,
    const std::shared_ptr<MapManager>& map_manager,
    const std::shared_ptr<MatchingManager>& matching_manager) {
  auto smm_param = Configure::GetInstance()->GetLocalizationSMMParam();
  if (frame_package == nullptr || map_manager == nullptr ||
      matching_manager == nullptr) {
    LC_LDEBUG(OPTIM) << "nullptr of instance module";
    return LOC_NULL_PTR;
  }
  const auto& frames = frame_package->GetFrames();
  if (frames.empty()) {
    LC_LDEBUG(OPTIM) << "frames size is empty.";
    return LOC_INVALID;
  }

  // get local map lanelines
  const LaneLine::PtrUMap& map_local_lanelines =
      map_manager->GetLocalLaneLines();

  // get perceptions and matches
  std::unordered_map<std::string, PerceptLaneLine::PtrUMap> percept_lanelines;
  std::unordered_map<std::string, MatchIndex::Ptr> multi_frame_match_index;
  for (size_t i = 0; i < frames.size(); ++i) {
    const auto& frame = frames[i];
    const auto& camera_name = frame->GetCameraName();
    // get perceptions
    const auto& percept_lines = frame->GetPerceptionData()->lane_lines;
    // TODO(xxx): save copy here
    for (const auto& line : percept_lines) {
      PerceptLaneLine::Ptr line_ptr(new PerceptLaneLine(line.second));
      percept_lanelines[camera_name][line.first] = line_ptr;
    }

    // get matches
    MatchIndex::Ptr match = frame->GetMatchIndex();
    multi_frame_match_index.insert({camera_name, match});
  }

  // object-level match check
  double yaw_rate = std::fabs(frames.front()->GetNavState().angular_speed(2));
  int min_line_num = yaw_rate > 0.05 ? 1 : 2;
  for (auto iter = percept_lanelines.begin();
       iter != percept_lanelines.end();) {
    // check if has enough perception lines
    auto& percept_lls = iter->second;
    if (percept_lls.size() < min_line_num) {
      LC_LDEBUG(OPTIM) << iter->first << " has percept lines < " << min_line_num
                       << ", not calib H.";
      percept_lanelines.erase(iter++);
      continue;
    }

    // use only main perception lanelines
    if (percept_lls.size() > 2) {
      double left_line_distance = 100, right_line_distance = -100;
      id_t invalid = std::numeric_limits<id_t>::min();
      id_t left_id = invalid, right_id = invalid;
      for (const auto& id_line_pair : percept_lls) {
        id_t id = id_line_pair.first;
        auto line = id_line_pair.second;
        if (line->processed_bv_points.empty()) continue;
        double dist = 0;
        double cnt = 0;
        for (const auto& pt : line->processed_bv_points) {
          dist += pt.y;
          ++cnt;
          if (cnt >= 5) break;
        }
        dist /= cnt;
        if (dist > 0 && dist < left_line_distance) {
          left_line_distance = dist;
          left_id = id;
        }
        if (dist < 0 && dist > right_line_distance) {
          right_line_distance = dist;
          right_id = id;
        }
      }
      if (left_id != invalid && right_id != invalid &&
          (left_line_distance - right_line_distance) < 10) {
        PerceptLaneLine::Ptr left_line = percept_lls.at(left_id);
        PerceptLaneLine::Ptr right_line = percept_lls.at(right_id);
        percept_lls.clear();
        percept_lls.insert({left_id, left_line});
        percept_lls.insert({right_id, right_line});
      } else {
        LC_LDEBUG(OPTIM) << iter->first
                         << " cannot find main lanelines, not calib "
                            "H.";
        percept_lanelines.erase(iter++);
        continue;
      }
    }
    iter++;
  }

  // find all point2line correlations for total frames
  std::unordered_map<std::string, SE3d> T_main_subs =
      frame_package->GetTransformSubToMain(false);
  std::unordered_map<std::string, std::vector<P2LCorr>>
      multicam_point2line_corrs;
  for (const auto& cam_ll_pair : percept_lanelines) {
    const auto& camera_name = cam_ll_pair.first;
    const auto& percept_lls = cam_ll_pair.second;
    if (!multi_frame_match_index.count(camera_name)) {
      LC_LDEBUG(OPTIM) << "why not exist match for " << camera_name;
      continue;
    }
    if (!T_main_subs.count(camera_name)) {
      LC_LDEBUG(OPTIM) << "why not exist Tmain_sub " << camera_name;
      continue;
    }
    MatchPairVec match_pairs_all;
    multi_frame_match_index.at(camera_name)
        ->GetSemanticMatch(SemanticType::LaneLine, &match_pairs_all);
    MatchPairVec match_pairs;
    for (const auto& pair : match_pairs_all) {
      if (percept_lls.count(pair.first)) match_pairs.emplace_back(pair);
    }
    std::vector<P2LCorr> p2l_corrs;
    FindLaneLineP2LAssociations(percept_lls, map_local_lanelines, match_pairs,
                                T_main_subs.at(camera_name), &p2l_corrs, true);
    if (p2l_corrs.size() < 5) {
      LC_LDEBUG(OPTIM) << camera_name << " has too few p2l corrs for H calib.";
      continue;
    }
    multicam_point2line_corrs[camera_name] = std::move(p2l_corrs);
  }
  if (multicam_point2line_corrs.empty()) return LOC_LOCALIZATION_ERROR;

  ////////////////////// construct problem and optimize //////////////////////

  FactorOptimizer::Ptr optimizer(new FactorOptimizer);
  optimizer->SetNumIters(5);
  optimizer->SetLinearSolverType(ceres::DENSE_QR);

  // add vertex_2dof: y and heading, fixed
  Vertex2DOF::Ptr vertex_2dof(new Vertex2DOF(0));
  vertex_2dof->SetFixed(true);
  optimizer->AddVertex(vertex_2dof);

  // add homography: pitch only
  std::unordered_map<std::string, VertexHomoPitch::Ptr>
      multicam_vertex_homo_pitch;
  size_t vertex_cnt = 1;
  std::unordered_map<std::string, double> initial_homo_pitchs =
      frame_package->GetHomoPitch();
  for (const auto& corr : multicam_point2line_corrs) {
    VertexHomoPitch::Ptr vertex_homo_pitch =
        std::make_shared<VertexHomoPitch>(vertex_cnt++);
    Eigen::VectorXd param(1);
    param(0) = 0;
    if (initial_homo_pitchs.count(corr.first)) {
      param(0) = initial_homo_pitchs.at(corr.first);
    }
    vertex_homo_pitch->SetParameter(param);
    optimizer->AddVertex(vertex_homo_pitch);
    multicam_vertex_homo_pitch[corr.first] = vertex_homo_pitch;
  }

  // add edge: EdgeP2L2DSpaceBA
  const auto& multicam_H_bases =
      Configure::GetInstance()->GetCameraHomographyBaseComponents();
  size_t edge_cnt = 0;

  std::unordered_map<std::string, std::vector<EdgeP2L2DSpaceBA::Ptr>>
      multicam_edges;
  for (const auto& corr : multicam_point2line_corrs) {
    const auto& camera_name = corr.first;
    if (!multicam_H_bases.count(camera_name)) {
      LC_LDEBUG(OPTIM) << "why not have H bases for " << camera_name;
      continue;
    }
    Eigen::Matrix3d H_base = multicam_H_bases.at(camera_name).first;
    Eigen::Matrix3d K_inv = multicam_H_bases.at(camera_name).second;

    // add point2line correlations to optimizer
    VertexHomoPitch::Ptr vertex_homo_pitch =
        multicam_vertex_homo_pitch.at(camera_name);
    const auto& point2line_corrs = corr.second;
    std::vector<EdgeP2L2DSpaceBA::Ptr> edges;
    edges.reserve(point2line_corrs.size());
    for (size_t i = 0; i < point2line_corrs.size(); ++i) {
      const auto& corr = point2line_corrs[i];
      double cov_factor = 1.0;
      if (corr.type == LineType::Curb) cov_factor = 2.0;
      EdgeP2L2DSpaceBA::Ptr edge_p2l(new EdgeP2L2DSpaceBA(
          edge_cnt++, Eigen::Vector2d(corr.p.x, corr.p.y),
          Eigen::Vector2d(corr.p_map_s.x, corr.p_map_s.y),
          Eigen::Vector2d(corr.p_map_e.x, corr.p_map_e.y),
          cov_factor * Eigen::Matrix2d::Identity(), H_base, K_inv));
      edge_p2l->AddVertex(vertex_2dof);
      edge_p2l->AddVertex(vertex_homo_pitch);
      edge_p2l->SetLossFunction(std::make_shared<ceres::HuberLoss>(1.0));
      optimizer->AddEdge(edge_p2l);
      edges.emplace_back(edge_p2l);
    }
    multicam_edges[camera_name] = std::move(edges);
  }

  // optimize homography
  if (!optimizer->Optimize(true)) return LOC_LOCALIZATION_ERROR;

  // check optimized pitch and chi2 error
  for (const auto& frame : frames) {
    const auto& camera_name = frame->GetCameraName();
    if (!multicam_vertex_homo_pitch.count(camera_name)) continue;
    double* h_pitch =
        multicam_vertex_homo_pitch.at(camera_name)->RawParameter();
    if (std::fabs(h_pitch[0]) * 180.0 / M_PI > 1.5) {
      LC_LDEBUG(OPTIM) << camera_name << " optim homo pitch too large "
                       << h_pitch[0];
      multicam_vertex_homo_pitch.erase(camera_name);
      continue;
    }

    double chi2_mean = 0.0, chi2_max = 0;
    for (const auto& edge : multicam_edges.at(camera_name)) {
      double chi2 = edge->Chi2();
      chi2_mean += chi2;
      chi2_max = std::max(chi2_max, chi2);
    }
    chi2_mean /= multicam_edges.at(camera_name).size();
    frame->SetOptimHomoChi2Errors(std::make_pair(chi2_mean, chi2_max));
    if (chi2_mean > 0.5 || (chi2_mean > 0.1 && chi2_max > 0.5) ||
        chi2_max > 1.0) {
      LC_LDEBUG(OPTIM) << camera_name << " homo optim chi2 error too large "
                       << chi2_mean << "/" << chi2_max;
      multicam_vertex_homo_pitch.erase(camera_name);
    }
  }
  if (multicam_vertex_homo_pitch.empty()) {
    LC_LDEBUG(OPTIM) << "all homo vertex are reset, no optim results";
    return LOC_LOCALIZATION_ERROR;
  }

  // update diff estimation if has multi camera optim result
  double h_pitch_30 = 0, h_pitch_120 = 0;
  bool has_30 = false, has_120 = false;
  for (const auto& vertex : multicam_vertex_homo_pitch) {
    if (vertex.first.find("30") != std::string::npos) {
      h_pitch_30 = vertex.second->GetParameter()(0);
      has_30 = true;
    }
    if (vertex.first.find("120") != std::string::npos) {
      h_pitch_120 = vertex.second->GetParameter()(0);
      has_120 = true;
    }
  }
  if (has_30 && has_120)
    Configure::GetInstance()->UpdateMultiCameraDiffPitch(h_pitch_30,
                                                         h_pitch_120);

  // check consistency
  double diff_mean = 0, diff_var = 0;
  int cnt = 0;
  Configure::GetInstance()->GetMultiCameraDiffPitch(&diff_mean, &diff_var,
                                                    &cnt);
  LC_LDEBUG(FRAME) << "diff pitch(deg) mean/std/cnt " << diff_mean * r2d << ", "
                   << std::sqrt(diff_var) * r2d << ", " << cnt;
  if (cnt > 100) {
    if (has_30 && has_120) {
      double diff_thres =
          std::max(2.0 * std::fabs(diff_mean), 0.25 * M_PI / 180.0);
      // at least 100 frames for stability
      if (std::fabs(NormalizeAngleDiff(h_pitch_30 - h_pitch_120)) >
          std::fabs(diff_thres)) {
        LC_LDEBUG(FRAME) << "multi camera pitch optimization inconsistent!";
        return LOC_LOCALIZATION_ERROR;
      }
    } else if (has_30) {  // has only 30
      h_pitch_120 = h_pitch_30 - diff_mean;
    } else if (has_120) {  // has only 120
      h_pitch_30 = h_pitch_120 + diff_mean;
    }
  }

  // get optimized homo pitch
  for (const auto& frame : frames) {
    const auto& camera_name = frame->GetCameraName();
    if (camera_name.find("30") != std::string::npos)
      frame->SetHomoPitch(h_pitch_30);
    if (camera_name.find("120") != std::string::npos)
      frame->SetHomoPitch(h_pitch_120);
  }

  // pts for visualization
  auto& param = Configure::GetInstance()->GetLocalizationParam();
  if (param.visual_param.enable_display) {
    std::vector<Point3D_t> match_map_points;
    for (const auto p2l_corrs : multicam_point2line_corrs) {
      SE3d T_main_sub = T_main_subs.at(p2l_corrs.first);
      for (const auto corr : p2l_corrs.second) {
        if (!corr.inlier) continue;
        match_map_points.emplace_back(T_main_sub * corr.p_map_s);
        match_map_points.emplace_back(T_main_sub * corr.p_map_e);
      }
    }
    frame_package->SetMatchedMapLanelineDataForHomoCalib(match_map_points);
  }

  // re-project lanelines bv points
  double max_long_range = smm_param.enable_temporal_fusion ? 30.0 : 40.0;
  double sample_dist = 1.0;
  double homo_proj_cov_coeff = 0.0004;
  for (auto frame : frame_package->GetFrames()) {
    auto pre_status = Frame::ProjectLaneLineBVPoints(
        frame->GetAdaptedHomography(), frame->GetNavState(), max_long_range,
        sample_dist, homo_proj_cov_coeff,
        &frame->GetPerceptionData()->lane_lines);
    if (pre_status != LOC_SUCCESS) {
      LC_LDEBUG(FRAME) << "re-project lanelines bv points failed!";
    }
  }

  return LOC_SUCCESS;
}

void OptimManager::FindLaneLineP2LAssociations(
    const PerceptLaneLine::PtrUMap& percept_lls,
    const std::unordered_map<id_t, std::shared_ptr<LaneLine>>& map_lls,
    const std::vector<std::pair<id_t, id_t>>& match_pairs,
    const SE3d& Tmap_percept, std::vector<P2LCorr>* p2l_corrs,
    bool in_image_space) {
  // iterate through matches
  for (const auto& match : match_pairs) {
    id_t p_id = match.first, m_id = match.second;
    auto pecept_ll_iter = percept_lls.find(p_id);
    auto map_ll_iter = map_lls.find(m_id);
    if (pecept_ll_iter == percept_lls.end() || map_ll_iter == map_lls.end()) {
      LC_LDEBUG(OPTIM) << "why not exist matching id " << p_id << " -> "
                       << m_id;
      continue;
    }
    const auto& percept_ll = *pecept_ll_iter;
    const auto& map_ll = *map_ll_iter;
    const auto& m_pts = map_ll.second->GetLineSegments().front()->GetPoints();
    const auto& p_pts = percept_ll.second->processed_bv_points;
    if (p_pts.empty() || m_pts.empty()) {
      continue;
    }

    // for down sample correspodences
    bool enable_corres_downsample = true;
    std::unordered_map<size_t, P2LCorr> map_line_corres;

    // transform perception points to map coordinate
    auto trans_points = TransformPoints(p_pts, Tmap_percept);
    SE3d Tpercept_map = Tmap_percept.inverse();
    // find percept point to map line correlations
    for (size_t k = 0; k < trans_points.size(); ++k) {
      P2LCorr corr;
      corr.p = trans_points[k];
      corr.cov.noalias() = percept_ll.second->processed_bv_point_covs[k];
      if (!IsPointInLineRange(corr.p, m_pts.front(), m_pts.back())) continue;
      size_t idx1, idx2;
      if (!BinarySearchTwoNearestPoints(corr.p, m_pts, &idx1, &idx2)) continue;
      if (Point2LineMahDist2D(corr.p, m_pts[idx1], m_pts[idx2],
                              corr.cov.inverse()) > chi2_2d_[2])
        continue;
      if ((m_pts[idx1] - m_pts[idx2]).Norm2D() < 1e-6) continue;

      corr.id_map = m_id;
      corr.type = percept_ll.second->line_type;
      corr.p_map_s = m_pts[idx1];
      corr.p_map_e = m_pts[idx2];
      if (in_image_space) {
        // save image point instead of bv point
        corr.p.x = percept_ll.second->processed_img_points[k].x;
        corr.p.y = percept_ll.second->processed_img_points[k].y;
        corr.p.z = 1.0;
        // and transform map point to percept frame
        corr.p_map_s = Tpercept_map * corr.p_map_s;
        corr.p_map_e = Tpercept_map * corr.p_map_e;
      }

      if (enable_corres_downsample) {
        auto iter = map_line_corres.find(idx1);
        if (iter == map_line_corres.end()) {
          map_line_corres[idx1] = std::move(corr);
        } else if (corr.cov.diagonal().norm() <
                   iter->second.cov.diagonal().norm()) {
          iter->second = std::move(corr);
        }
      } else {
        p2l_corrs->emplace_back(std::move(corr));
      }
    }

    if (enable_corres_downsample) {
      for (auto& item : map_line_corres)
        p2l_corrs->emplace_back(std::move(item.second));
    }
  }
}

adLocStatus_t OptimManager::FindPoleP2PAssociations(
    const std::unordered_map<std::string, PerceptPole::PtrUMap>&
        multi_cam_percept_poles,
    const std::unordered_map<id_t, std::shared_ptr<Pole>>& map_poles,
    const std::unordered_map<std::string, std::shared_ptr<MatchIndex>>&
        multi_cam_match_index,
    std::vector<P2PCorr>* p2p_corrs) const {
  if (!p2p_corrs) return LOC_NULL_PTR;

  for (const auto& item : multi_cam_percept_poles) {
    const auto& camera_name = item.first;
    const auto& percept_poles = item.second;
    if (!multi_cam_match_index.count(camera_name)) {
      LC_LERROR(OPTIM) << "why not exist match for " << camera_name;
      continue;
    }
    MatchPairVec match_pairs;
    multi_cam_match_index.at(camera_name)
        ->GetSemanticMatch(SemanticType::Pole, &match_pairs);
    for (const auto& pair : match_pairs) {
      id_t pid = pair.first;
      const auto p_pole_iter = percept_poles.find(pid);
      if (p_pole_iter == percept_poles.end()) {
        LC_LERROR(OPTIM) << "find pole association lose percept id " << pid;
        continue;
      }
      id_t mid = pair.second;
      const auto m_pole_iter = map_poles.find(mid);
      if (m_pole_iter == map_poles.end()) {
        LC_LERROR(OPTIM) << "find pole association lose map id, pid->mid: "
                         << pid << " -> " << mid;
        continue;
      }
      P2PCorr corr;
      corr.id_map = mid;
      corr.p = p_pole_iter->second->processed_center;
      corr.cov.noalias() =
          p_pole_iter->second->processed_center_cov.topLeftCorner(2, 2) +
          m_pole_iter->second->GetPointCov().topLeftCorner(2, 2);
      corr.p_map = m_pole_iter->second->GetBottomPoint();
      corr.type = SemanticType::Pole;
      p2p_corrs->emplace_back(corr);
    }
  }
  return LOC_SUCCESS;
}

adLocStatus_t OptimManager::FindTrafficSignP2PAssociations(
    const std::unordered_map<std::string, PerceptTrafficSign::PtrUMap>&
        multi_cam_percept_signs,
    const std::unordered_map<id_t, std::shared_ptr<TrafficSign>>& map_signs,
    const std::unordered_map<std::string, std::shared_ptr<MatchIndex>>&
        multi_cam_match_index,
    std::vector<P2PCorr>* p2p_corrs) const {
  if (!p2p_corrs) return LOC_NULL_PTR;

  for (const auto& item : multi_cam_percept_signs) {
    const auto& camera_name = item.first;
    const auto& percept_signs = item.second;
    if (!multi_cam_match_index.count(camera_name)) {
      LC_LERROR(OPTIM) << "why not exist match for " << camera_name;
      continue;
    }
    MatchPairVec match_pairs;
    multi_cam_match_index.at(camera_name)
        ->GetSemanticMatch(SemanticType::TrafficSign, &match_pairs);
    for (const auto& pair : match_pairs) {
      id_t pid = pair.first;
      const auto p_sign_iter = percept_signs.find(pid);
      if (p_sign_iter == percept_signs.end()) {
        LC_LERROR(OPTIM) << "find sign association lose percept id " << pid;
        continue;
      }
      id_t mid = pair.second;
      const auto m_sign_iter = map_signs.find(mid);
      if (m_sign_iter == map_signs.end()) {
        LC_LERROR(OPTIM) << "find sign association lose map id, pid->mid: "
                         << pid << " -> " << mid;
        continue;
      }
      P2PCorr corr;
      corr.id_map = mid;
      corr.p = p_sign_iter->second->processed_center;
      corr.cov.noalias() =
          p_sign_iter->second->processed_center_cov.topLeftCorner(2, 2) +
          m_sign_iter->second->GetCenterCov().topLeftCorner(2, 2);
      corr.p_map = m_sign_iter->second->GetCenter();
      corr.type = SemanticType::TrafficSign;
      p2p_corrs->emplace_back(corr);
    }
  }
  return LOC_SUCCESS;
}

adLocStatus_t OptimManager::EstimateSMMCovariance(
    const FactorOptimizer::Ptr& optimizer,
    const std::vector<P2LCorr>& laneline_p2l_corrs,
    const std::vector<P2PCorr>& pole_p2p_corrs,
    const std::vector<P2PCorr>& sign_p2p_corrs,
    Eigen::Matrix<double, 6, 6>* pose_cov) {
  if (!pose_cov || !optimizer) return LOC_NULL_PTR;

  struct MatchLineInfo {
    int pt_num = 0;
    int outlier_num = 0;
    double percept_y_sum = 0.0;
    double map_y_sum = 0.0;
    double error_sum = 0.0;
    double y_info_sum = 0.0;
  };

  // get optim covariance first
  Eigen::MatrixXd optim_covariance;
  if (!optimizer->GetCovariance(0, &optim_covariance))
    return LOC_LOCALIZATION_ERROR;

  // count laneline match info
  int p2l_inlier_num = 0;
  std::unordered_map<id_t, MatchLineInfo> match_line_info(16);
  for (size_t i = 0; i < laneline_p2l_corrs.size(); ++i) {
    auto& corr = laneline_p2l_corrs[i];
    if (!corr.inlier) {
      match_line_info[corr.id_map].outlier_num += 1;
      continue;
    }
    ++p2l_inlier_num;
    double y_info = 1 / (corr.cov(1, 1) + 1.e-8);
    if (corr.type == LineType::Curb) y_info /= 2.0;
    match_line_info[corr.id_map].pt_num += 1;
    match_line_info[corr.id_map].percept_y_sum += corr.p.y;
    match_line_info[corr.id_map].map_y_sum +=
        0.5 * (corr.opt_p_map_s.y + corr.opt_p_map_e.y);
    match_line_info[corr.id_map].error_sum += y_info * corr.error;
    match_line_info[corr.id_map].y_info_sum += y_info;
  }

  int remain_matched_map_lines = static_cast<int>(match_line_info.size());
  if (remain_matched_map_lines < 1 || p2l_inlier_num == 0) {
    LC_LDEBUG(OPTIM) << "after optimization, matched map lines < 1";
    return LOC_LOCALIZATION_ERROR;
  }

  // count pole & traffic sign match info
  int p2p_inlier_num = 0;
  for (size_t i = 0; i < pole_p2p_corrs.size(); ++i) {
    auto& corr = pole_p2p_corrs[i];
    if (!corr.inlier) continue;
    ++p2p_inlier_num;
  }
  for (size_t i = 0; i < sign_p2p_corrs.size(); ++i) {
    auto& corr = sign_p2p_corrs[i];
    if (!corr.inlier) continue;
    ++p2p_inlier_num;
  }

  // calculate optimization related covariance item
  double opt_lateral_factor = 0.3;
  double opt_yaw_factor = 0.3;
  double opt_lateral_offset = 0.08;
  double opt_yaw_offset = 0.014;
  Eigen::Vector2d optim_std{
      opt_lateral_factor * std::sqrt(optim_covariance(0, 0)) +
          opt_lateral_offset,
      opt_yaw_factor * std::sqrt(optim_covariance(1, 1)) + opt_yaw_offset};
  optim_std(0) = std::min(1.0, optim_std(0));
  optim_std(1) = std::min(0.052, optim_std(1));

  // calculate matched line number related covariance item
  Eigen::Vector2d line_num_std{0.2, 0.0052};  // std: 0.2 m, 0.3 degree
  double valid_line_num = 0.0;
  std::vector<std::pair<double, int>>
      valid_line_lateral;  // each pair: {mean lateral, point num}
  for (const auto& line_info : match_line_info) {
    if (line_info.second.pt_num > 0) {
      double mean_lateral =
          line_info.second.map_y_sum / line_info.second.pt_num;
      bool has_same_line = false;
      for (auto& item : valid_line_lateral) {
        if (std::fabs(mean_lateral - item.first) < 0.5) {
          item.first = (item.first * item.second + line_info.second.map_y_sum) /
                       (item.second + line_info.second.pt_num);
          item.second += line_info.second.pt_num;
          has_same_line = true;
          break;
        }
      }
      if (!has_same_line) {
        valid_line_lateral.push_back({mean_lateral, line_info.second.pt_num});
      }
    }
  }
  for (const auto& item : valid_line_lateral) {
    if (item.second > 3.0) ++valid_line_num;
  }
  double line_num_thres = 3.0;
  double line_num_factor =
      (1 - std::min(line_num_thres, valid_line_num) / line_num_thres);
  line_num_std = line_num_factor * line_num_std;

  // calculate line distribution related covariance item
  Eigen::Vector2d line_distribute_std{0.5, 0.035};  // std: 0.5 m, 2.0 degree
  int left_pt_num = 0;
  int right_pt_num = 0;
  double left_line_error_sum = 0;
  double right_line_error_sum = 0;
  double left_y_info_sum = 0;
  double right_y_info_sum = 0;
  for (const auto& item : match_line_info) {
    if (item.second.percept_y_sum >= 0.0) {
      left_pt_num += item.second.pt_num;
      left_line_error_sum += item.second.error_sum;
      left_y_info_sum += item.second.y_info_sum;
    } else {
      right_pt_num += item.second.pt_num;
      right_line_error_sum += item.second.error_sum;
      right_y_info_sum += item.second.y_info_sum;
    }
  }
  double error_sum = 0.0;
  if (left_pt_num > 3 && right_pt_num > 3) {
    double left_line_error_mean = left_line_error_sum / left_y_info_sum;
    double right_line_error_mean = right_line_error_sum / right_y_info_sum;
    double diff_error = std::fabs(left_line_error_mean + right_line_error_mean);
    error_sum =
        std::fabs(left_line_error_mean) + std::fabs(right_line_error_mean);
    if (left_line_error_mean * right_line_error_mean < 0.0) {
      diff_error *= 0.6;
    } else if (std::fabs(left_line_error_mean) > 0.03 &&
               std::fabs(right_line_error_mean) > 0.03) {
      diff_error *= 1.5;
    }
    if (valid_line_num < 3) {
      diff_error *= 1.2;
    }
    double exp_error = std::exp(diff_error);
    double exp_negative_error = std::exp(-diff_error);
    double line_distri_factor =
        (exp_error - exp_negative_error) / (exp_error + exp_negative_error);

    line_distribute_std = line_distri_factor * line_distribute_std;
  }
  if (error_sum > 0.9 || left_pt_num <= 3 || right_pt_num <= 3) {
    line_distribute_std += Eigen::Vector2d{0.5, 0.0};
  }

  Eigen::Vector2d total_std = optim_std + line_num_std + line_distribute_std;

  // adaptive covariance, consider the optimed and matched covariance
  (*pose_cov).setZero();
  (*pose_cov).block<3, 3>(0, 0) = 10000.0 * Eigen::Matrix3d::Identity();
  (*pose_cov).block<3, 3>(3, 3) = 10000.0 * Eigen::Matrix3d::Identity();
  (*pose_cov)(1, 1) = total_std(0) * total_std(0);
  (*pose_cov)(5, 5) = total_std(1) * total_std(1);

  // TODO(xxx): estimate longitudinal variance
  if (p2p_inlier_num > 0) {
    (*pose_cov)(0, 0) = 4.0;
  }

  LC_LDEBUG(SMM) << "smm y std " << std::sqrt((*pose_cov)(1, 1)) << " opt std "
                 << optim_std(0) << " line num std " << line_num_std(0)
                 << " line distri std " << line_distribute_std(0);
  LC_LDEBUG(SMM) << "smm yaw std " << std::sqrt((*pose_cov)(5, 5)) * r2d
                 << " opt std " << optim_std(1) * r2d << " line num std "
                 << line_num_std(1) * r2d << " line distri std "
                 << line_distribute_std(1) * r2d;
  return LOC_SUCCESS;
}

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
