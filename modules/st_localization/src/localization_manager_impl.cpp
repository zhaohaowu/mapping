/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 * Zhao Ming <zhaoming@senseauto.com>
 */

#include "localization_manager_impl.hpp"

#include <fstream>
#include <limits>
#include <utility>

#include <Sophus/se3.hpp>
#include <opencv2/core/eigen.hpp>

#include "ad_scm/ad_scm.hpp"
#include "ad_scm/ad_transform.hpp"
#include "common/coordinate_converter.hpp"
#include "common/transform_config.hpp"
#include "common/utility.hpp"
#include "eval/evaluator_localziation.hpp"
#include "initialization/initialization.hpp"
#include "msf/msf_fusion/msf_fusion_factory.hpp"
#include "system/backend/localization_backend_can.hpp"
#include "system/backend/localization_backend_gnss.hpp"
#include "system/backend/localization_backend_heading.hpp"
#include "system/backend/localization_backend_ins.hpp"
#include "system/backend/localization_backend_smm.hpp"
#include "system/frontend/localization_frontend_can.hpp"
#include "system/frontend/localization_frontend_imu.hpp"
#include "system/frontend/localization_frontend_ins.hpp"
#include "system/frontend/localization_frontend_replay.hpp"
#include "system/localization_backend.hpp"
#include "system/localization_dead_reckoning.hpp"
#include "system/localization_frontend.hpp"
#include "system/localization_state_monitor.hpp"
#include "system/localization_visualizer.hpp"

namespace senseAD {
namespace localization {

using typename senseAD::common::utils::CoordinateTransformUtility;
using typename senseAD::sensorconfig::SensorConfigManager;

adLocStatus_t LocalizationManagerImpl::Init(const LocalizationParam& param) {
  param_ = param;

  // load offline transform config
  if (TransformConfig::LoadTransformConfig(param_) != LOC_SUCCESS) {
    LC_LERROR(LOCALIZATION) << "Failed to read extrinsic parameters";
    return LOC_LOCALIZATION_ERROR;
  }

  // create relative localization related module
  if (param_.common_param.relative_localization) {
    // create and init dead reckoning locator
    if (CreateLocalizationDR() != LOC_SUCCESS) {
      LC_LERROR(LOCALIZATION) << "Failed to create localization DR.";
      return LOC_LOCALIZATION_ERROR;
    }
  }

  // create absolute localization related module
  if (param_.common_param.absolute_localization) {
    // load hdmap origin (ENU origin)
    if (param.common_param.using_map_origin) {
      if (LOC_SUCCESS != LoadMapOrigin(param_.common_param.hdmap_file)) {
        LC_LERROR(LOCALIZATION) << "Load map origin failed";
        return LOC_LOCALIZATION_ERROR;
      }
      set_origin_flag_ = true;
    }

    front_locator_type_ =
        LocatorTypeFromString(param_.common_param.front_locator_type);
    back_locator_type_ =
        FindLocatorTypeFromString(param_.common_param.back_locator_type);
    auto execute_mode = ExecuteModeFromString(param_.common_param.execute_mode);
    if (execute_mode == offline_replay) front_locator_type_ = REPLAY;

    // create and init frontend locator
    if (CreateLocalizationFrontend() != LOC_SUCCESS) {
      LC_LERROR(LOCALIZATION) << "Failed to create localization frontend.";
      return LOC_LOCALIZATION_ERROR;
    }

    // create and init backend locator
    if (CreateLocalizationBackend() != LOC_SUCCESS) {
      LC_LERROR(LOCALIZATION) << "Failed to create localization backend.";
      return LOC_LOCALIZATION_ERROR;
    }

    // create multi-sensor-fusion instance
    if (CreateMSF() != LOC_SUCCESS) {
      LC_LERROR(LOCALIZATION) << "Failed to create localization MSF fusion.";
      return LOC_LOCALIZATION_ERROR;
    }
  }

  // create localization state monitor
  if (CreateLocalizationMonitor() != LOC_SUCCESS) {
    LC_LERROR(LOCALIZATION) << "Failed to create localization monitor";
    return LOC_LOCALIZATION_ERROR;
  }

  // create localization visualizer
  if (param_.visual_param.enable_display) {
    if (CreateLocalizationVisualizer() != LOC_SUCCESS) {
      LC_LERROR(LOCALIZATION) << "Failed to create localization visualizer";
      return LOC_LOCALIZATION_ERROR;
    }
  }

  // set module instance pointer link
  if (SetModulePointerLink() != LOC_SUCCESS) {
    LC_LERROR(LOCALIZATION) << "Failed to set module pointer link.";
    return LOC_LOCALIZATION_ERROR;
  }

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationManagerImpl::Restart() { return LOC_SUCCESS; }

adLocStatus_t LocalizationManagerImpl::ShutDown() {
  LC_LINFO(LOCALIZATION) << "system shutdown, all threads will be killing...";

  if (param_.common_param.relative_localization) {
    // request dead reckoning thread finish
    if (loc_dead_reckoning_) {
      loc_dead_reckoning_->RequestFinish();
      while (!loc_dead_reckoning_->IsFinished()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    }
  }
  if (param_.common_param.absolute_localization) {
    // request frontend thread finish
    if (loc_frontend_) {
      loc_frontend_->RequestFinish();
      while (!loc_frontend_->IsFinished()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    }
    // request backend thread finish
    for (auto& loc_backend : loc_backends_) {
      if (loc_backend.second) {
        loc_backend.second->RequestFinish();
        while (!loc_backend.second->IsFinished()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
      }
    }
  }
  if (param_.visual_param.enable_display && loc_visualizer_) {
    // request visualizer thread finish
    loc_visualizer_->RequestFinish();
    while (!loc_visualizer_->IsFinished()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  if (loc_monitor_) {
    loc_monitor_->RequestFinish();
    while (!loc_monitor_->IsFinished()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  LC_LINFO(LOCALIZATION) << "system has shutdown.";

  return LOC_SUCCESS;
}

void LocalizationManagerImpl::SetCanData(uint64_t timestamp,
                                         const VehicleInfo& can_data) {
  // check can data
  if (LOC_SUCCESS != loc_monitor_->CheckCanData(timestamp, can_data)) return;
  if (param_.common_param.relative_localization) {
    // for DR
    if (loc_dead_reckoning_ && !is_dr_stop_data_) {
      loc_dead_reckoning_->SetCanData(timestamp, can_data);
    }
  }
  if (param_.common_param.absolute_localization) {
    // check whether has set localization origin
    if (!set_origin_flag_) return;
    // checking whether global localization system is on restarting
    if (is_loc_stop_data_) return;
    // for frontend using
    if (front_locator_type_ == CAN) {
      auto frontend_can =
          std::dynamic_pointer_cast<LocalizationFrontendCAN>(loc_frontend_);
      frontend_can->SetInputData(timestamp, can_data);
      // for initilizer
      auto initilizer = loc_frontend_->GetInitilizer();
      if (initilizer->GetInitStage() != InitStage::INIT_DONE) {
        initilizer->AddVehicleInfo(timestamp, can_data);
      }
    }
    if (front_locator_type_ == IMU) {
      auto frontend_imu =
          std::dynamic_pointer_cast<LocalizationFrontendIMU>(loc_frontend_);
      frontend_imu->SetCanData(timestamp, can_data);
    }
    // for backend using
    if (loc_backends_.count(CAN)) {
      auto backend_can =
          std::dynamic_pointer_cast<LocalizationBackendCAN>(loc_backends_[CAN]);
      backend_can->SetInputData(timestamp, can_data);
    }
  }
}

void LocalizationManagerImpl::SetImuData(uint64_t timestamp,
                                         const Imu& raw_imu) {
  Imu imu_data = raw_imu;
  // convert from IMU frame (sensor frame) to body frame (IMU center, RFU)
  ConvertImuToBodyFrame(raw_imu, &imu_data);
  // check imu data
  if (LOC_SUCCESS != loc_monitor_->CheckImuData(timestamp, imu_data)) return;
  if (param_.common_param.relative_localization) {
    // for DR
    if (loc_dead_reckoning_ && !is_dr_stop_data_) {
      loc_dead_reckoning_->SetImuData(timestamp, imu_data);
    }
  }
  if (param_.common_param.absolute_localization) {
    // check whether has set localization origin
    if (!set_origin_flag_) return;
    // checking whether global localization system is on restarting
    if (is_loc_stop_data_) return;
    if (front_locator_type_ == IMU || front_locator_type_ == INS) {
      // for initilizer
      auto initilizer = loc_frontend_->GetInitilizer();
      if (initilizer->GetInitStage() != InitStage::INIT_DONE) {
        initilizer->AddImu(timestamp, imu_data);
      }
    }
    // for frontend using
    if (front_locator_type_ == IMU) {
      auto frontend_imu =
          std::dynamic_pointer_cast<LocalizationFrontendIMU>(loc_frontend_);
      frontend_imu->SetInputData(timestamp, imu_data);
    }
    if (front_locator_type_ == INS) {
      auto frontend_ins =
          std::dynamic_pointer_cast<LocalizationFrontendINS>(loc_frontend_);
      frontend_ins->SetImuData(timestamp, imu_data);
    }
  }
}

void LocalizationManagerImpl::SetInsData(uint64_t timestamp,
                                         const Ins& fused_ins) {
  if (!param_.common_param.absolute_localization) return;
  if (!param_.common_param.sub_ins_data) return;
  // check ins data
  if (LOC_SUCCESS != loc_monitor_->CheckInsData(timestamp, fused_ins)) return;
  // check whether has set localization origin
  if (!set_origin_flag_) return;
  // for eval and SMM debugging
  if (param_.ci_param.enable_evaluation || param_.visual_param.enable_display)
    loc_frontend_->GtLocatorForEval(timestamp, fused_ins);
  // checking whether global localization system is on restarting
  if (is_loc_stop_data_) return;
  // for initilizer
  auto initilizer = loc_frontend_->GetInitilizer();
  auto init_status = initilizer->GetInitStage();
  if (init_status != InitStage::INIT_DONE) {
    if (front_locator_type_ == INS || front_locator_type_ == CAN) {
      initilizer->AddIns(timestamp, fused_ins);
    }
    // for ins-aided fast init
    if (front_locator_type_ == IMU) {
      auto frontend_imu =
          std::dynamic_pointer_cast<LocalizationFrontendIMU>(loc_frontend_);
      frontend_imu->SetInsData(timestamp, fused_ins);
    }
  }
  // for frontend using
  if (front_locator_type_ == INS) {
    auto frontend_ins =
        std::dynamic_pointer_cast<LocalizationFrontendINS>(loc_frontend_);
    frontend_ins->SetInputData(timestamp, fused_ins);
  }
  // for backend using
  if (loc_backends_.count(INS)) {
    auto backend_ins =
        std::dynamic_pointer_cast<LocalizationBackendINS>(loc_backends_[INS]);
    backend_ins->SetInputData(timestamp, fused_ins);
  }
  // for visualizer
  if (loc_visualizer_) {
    NavState ins_state, gnss_state;
    loc_frontend_->GetGtState(&ins_state, &gnss_state);
    loc_visualizer_->SetInsInfo(ins_state, fused_ins.status);
  }
}

void LocalizationManagerImpl::SetGnssData(uint64_t timestamp,
                                          const Gnss& raw_gnss) {
  if (!param_.common_param.absolute_localization) return;
  // check gnss data
  if (LOC_SUCCESS != loc_monitor_->CheckGnssData(timestamp, raw_gnss)) return;
  // check whether has set localization origin, using first gnss data set
  {
    std::lock_guard<std::mutex> lock(gnss_data_for_origin_mutex_);
    gnss_data_for_origin_ = raw_gnss;
  }
  if (!set_origin_flag_) {
    PointLLH_t origin(gnss_data_for_origin_.position.lon,
                      gnss_data_for_origin_.position.lat,
                      gnss_data_for_origin_.position.height);
    CoordinateConverter::GetInstance()->SetOrigin(origin);
    if (param_.ci_param.enable_evaluation) SaveLocOriginLLA(origin);
    set_origin_flag_ = true;
  }
  // for eval and SMM debugging
  loc_frontend_->GtRtkLocatorForEval(timestamp, raw_gnss);
  // checking whether global localization system is on restarting
  if (is_loc_stop_data_) return;
  if (front_locator_type_ == IMU) {
    // for initilizer
    auto initilizer = loc_frontend_->GetInitilizer();
    if (initilizer->GetInitStage() != InitStage::INIT_DONE) {
      initilizer->AddGnss(timestamp, raw_gnss);
    }
  }
  // for backend using
  if (loc_backends_.count(GNSS)) {
    auto backend_gnss =
        std::dynamic_pointer_cast<LocalizationBackendGNSS>(loc_backends_[GNSS]);
    backend_gnss->SetInputData(timestamp, raw_gnss);
  }
  // for visualizer
  if (loc_visualizer_) {
    NavState ins_state, gnss_state;
    loc_frontend_->GetGtState(&ins_state, &gnss_state);
    loc_visualizer_->SetGnssInfo(gnss_state, raw_gnss.status);
  }
}

void LocalizationManagerImpl::SetPerceptData(
    uint64_t timestamp, const std::shared_ptr<PerceptData>& percept_data) {
  // check whether has set absolute localization
  if (!param_.common_param.absolute_localization) return;
  // check percept data
  if (LOC_SUCCESS != loc_monitor_->CheckPerceptData(timestamp, percept_data))
    return;
  // check whether has set localization origin
  if (!set_origin_flag_) return;
  // checking whether global localization system is on restarting
  if (is_loc_stop_data_) return;
  // for backend using
  if (loc_backends_.count(SMM)) {
    auto backend_smm =
        std::dynamic_pointer_cast<LocalizationBackendSMM>(loc_backends_[SMM]);
    backend_smm->SetInputData(timestamp, percept_data);
  }
  // for visualizer
  if (loc_visualizer_) {
    loc_visualizer_->SetPerceptData(timestamp, percept_data);
  }
}

void LocalizationManagerImpl::SetLocalMapData(
    uint64_t timestamp, const std::shared_ptr<RoadStructure>& local_map_data) {
  if (!param_.common_param.absolute_localization) return;
  // check localmap data
  if (LOC_SUCCESS != loc_monitor_->CheckLocalMapData(timestamp, local_map_data))
    return;
  // check whether has set localization origin
  if (!set_origin_flag_) return;
  // checking whether global localization system is on restarting
  if (is_loc_stop_data_) return;
  // convert local map data
  // TODO(xxx): save computation of overlapping area
  ConvertLocalMapCoord(timestamp, local_map_data);
  // for backend using
  if (loc_backends_.count(SMM)) {
    auto backend_smm =
        std::dynamic_pointer_cast<LocalizationBackendSMM>(loc_backends_[SMM]);
    backend_smm->SetLocalMapData(timestamp, local_map_data);
  }
  // for visualizer
  if (loc_visualizer_) {
    loc_visualizer_->SetLocalMapData(timestamp, local_map_data);
  }
}

void LocalizationManagerImpl::SetNavData(uint64_t timestamp,
                                         const NavState& ns) {
  if (!param_.common_param.absolute_localization) return;
  // check nav data
  if (LOC_SUCCESS != loc_monitor_->CheckNavData(timestamp, ns)) return;
  // check whether has set localization origin
  if (!set_origin_flag_) return;
  // checking whether global localization system is on restarting
  if (is_loc_stop_data_) return;
  // for frontend using
  if (front_locator_type_ == REPLAY) {
    auto frontend_replay =
        std::dynamic_pointer_cast<LocalizationFrontendReplay>(loc_frontend_);
    frontend_replay->SetInputData(timestamp, ns);
  }
}

void LocalizationManagerImpl::SetDualAntHeadingData(
    uint64_t timestamp, const DualAntennaHeading& dual_ant_heading) {
  if (!param_.common_param.absolute_localization) return;
  // check dualant data
  if (LOC_SUCCESS !=
      loc_monitor_->CheckDualAntHeadingData(timestamp, dual_ant_heading))
    return;
  // check whether has set localization origin
  if (!set_origin_flag_) return;
  // checking whether global localization system is on restarting
  if (is_loc_stop_data_) return;
  // compensation of base line to body frame's forward direction offset
  DualAntennaHeading app_imu_heading = dual_ant_heading;
  app_imu_heading.heading = dual_ant_heading.heading - M_PI_2;
  if (app_imu_heading.heading < -M_PI) {
    app_imu_heading.heading += 2 * M_PI;
  }
  if (front_locator_type_ == IMU) {
    // for initilizer
    auto initilizer = loc_frontend_->GetInitilizer();
    if (initilizer->GetInitStage() != InitStage::INIT_DONE) {
      initilizer->AddDualAntennaHeading(timestamp, app_imu_heading);
    }
  }
  // for backend using
  if (loc_backends_.count(HEADING)) {
    auto backend_heading =
        std::dynamic_pointer_cast<LocalizationBackendHeading>(
            loc_backends_[HEADING]);
    backend_heading->SetInputData(timestamp, app_imu_heading);
  }
}

adLocStatus_t LocalizationManagerImpl::GetNavStateInfo(
    NavStateInfo* info) const {
  if (info == nullptr) {
    LC_LERROR(LOCALIZATION) << "nullptr input.";
    return LOC_NULL_PTR;
  }
  if (!param_.common_param.absolute_localization) return LOC_LOCALIZATION_ERROR;
  uint64_t cur_time =
      std::chrono::steady_clock::now().time_since_epoch().count();  // nanosec
  static std::pair<uint64_t, NavStateInfo> last_get_navinfo = {0,
                                                               NavStateInfo()};
  adLocStatus_t ret_status = LOC_SUCCESS;
  // TODO(xx): in restart mode, also should publish localization info;
  // in switch origin mode, need extrapolate pose ?
  if (param_.common_param.enable_loc_restart && is_loc_stop_data_) {
    if (last_get_navinfo.first == 0) return LOC_LOCALIZATION_ERROR;
    if (cur_time < last_get_navinfo.first) return LOC_LOCALIZATION_ERROR;

    uint64_t time_gap = cur_time - last_get_navinfo.first;
    *info = last_get_navinfo.second;
    info->measurement_time_ns = last_get_navinfo.second.measurement_time_ns +
                                (time_gap / 10000000) * 10000000;
    info->nav_status = NavStatus::FATAL_ACCURACY;

    cur_time = last_get_navinfo.first + (time_gap / 10000000) * 10000000;
    last_get_navinfo.first = cur_time;
  } else {  // normal mode
    ret_status = loc_frontend_->GetNavStateInfo(info);
    last_get_navinfo.first = cur_time;
    last_get_navinfo.second = *info;
  }
  if (ret_status == LOC_SUCCESS) loc_monitor_->CheckNavStateInfoData(*info);
  return ret_status;
}

adLocStatus_t LocalizationManagerImpl::GetOdomStateInfo(
    OdomStateInfo* info) const {
  if (info == nullptr) {
    LC_LERROR(LOCALIZATION) << "nullptr input.";
    return LOC_NULL_PTR;
  }
  if (!param_.common_param.relative_localization) return LOC_LOCALIZATION_ERROR;

  adLocStatus_t ret_status = loc_dead_reckoning_->GetOdomStateInfo(info);
  if (ret_status == LOC_SUCCESS) loc_monitor_->CheckOdomStateInfoData(*info);
  return ret_status;
}

adLocStatus_t LocalizationManagerImpl::DRRestartProc() {
  LC_LINFO(LOCALIZATION) << "DRRestart start, request system thread paused.";
  StopSetDataAndThread(false);

  LC_LINFO(LOCALIZATION) << "DRRestart...";

  // restart dr
  if (loc_dead_reckoning_->Restart() != LOC_SUCCESS) {
    LC_LERROR(LOCALIZATION) << "Failed to restart dr.";
    return LOC_LOCALIZATION_ERROR;
  }

  // restart localization visualizer
  if (loc_visualizer_ && loc_visualizer_->Restart(false) != LOC_SUCCESS) {
    LC_LERROR(LOCALIZATION) << "Failed to restart visualizer.";
    return LOC_LOCALIZATION_ERROR;
  }

  ContinueSetDataAndThread(false);
  LC_LINFO(LOCALIZATION) << "DRRestart done, continue system thread.";

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationManagerImpl::LocRestartProc() {
  LC_LINFO(LOCALIZATION) << "LocRestart start, request system thread paused.";
  StopSetDataAndThread(true);

  LC_LINFO(LOCALIZATION) << "LocRestart...";

  // restart localization frontend
  if (loc_frontend_->Restart() != LOC_SUCCESS) {
    LC_LERROR(LOCALIZATION) << "Failed to restart frontend.";
    return LOC_LOCALIZATION_ERROR;
  }

  // restart localization backend
  for (auto& loc_backend : loc_backends_) {
    if (loc_backend.second->Restart() != LOC_SUCCESS) {
      LC_LERROR(LOCALIZATION) << "Failed to restart backend: "
                              << LocatorTypeToString(loc_backend.first);
      return LOC_LOCALIZATION_ERROR;
    }
  }

  // restart localization visualizer
  if (loc_visualizer_ && loc_visualizer_->Restart(true) != LOC_SUCCESS) {
    LC_LERROR(LOCALIZATION) << "Failed to restart visualizer.";
    return LOC_LOCALIZATION_ERROR;
  }

  // restart msf
  if (CreateMSF() != LOC_SUCCESS) {
    LC_LERROR(LOCALIZATION) << "Failed to restart MSF fusion.";
    return LOC_LOCALIZATION_ERROR;
  }

  // set module instance pointer link
  if (SetModulePointerLink() != LOC_SUCCESS) {
    LC_LERROR(LOCALIZATION) << "Failed to restart set module pointer link.";
    return LOC_LOCALIZATION_ERROR;
  }

  ContinueSetDataAndThread(true);
  LC_LINFO(LOCALIZATION) << "LocRestart done, continue system thread.";

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationManagerImpl::LocSwitchOriginProc() {
  LC_LINFO(LOCALIZATION) << "SwitchOrigin start, request system thread paused.";
  StopSetDataAndThread(true);

  LC_LINFO(LOCALIZATION) << "SwitchOrigin...";

  // reset origin using latest gnss LLA
  {
    std::lock_guard<std::mutex> lock(gnss_data_for_origin_mutex_);
    PointLLH_t origin(gnss_data_for_origin_.position.lon,
                      gnss_data_for_origin_.position.lat,
                      gnss_data_for_origin_.position.height);
    CoordinateConverter::GetInstance()->SetOrigin(origin);
    if (param_.ci_param.enable_evaluation) SaveLocOriginLLA(origin);
  }

  // process for frontend
  if (loc_frontend_->SwitchOriginProc() != LOC_SUCCESS) {
    LC_LERROR(LOCALIZATION) << "Failed to switch origin process for frontend.";
    return LOC_LOCALIZATION_ERROR;
  }

  // process for backend
  for (auto& loc_backend : loc_backends_) {
    if (loc_backend.second->SwitchOriginProc() != LOC_SUCCESS) {
      LC_LERROR(LOCALIZATION) << "Failed to switch origin process for backend: "
                              << LocatorTypeToString(loc_backend.first);
      return LOC_LOCALIZATION_ERROR;
    }
  }

  // process for visualizer
  if (loc_visualizer_ && loc_visualizer_->SwitchOriginProc() != LOC_SUCCESS) {
    LC_LERROR(LOCALIZATION)
        << "Failed to switch origin process for visualizer.";
    return LOC_LOCALIZATION_ERROR;
  }

  // process for msf
  if (msf_fusion_->SwitchOriginProc() != LOC_SUCCESS) {
    LC_LERROR(LOCALIZATION) << "Failed to switch origin process for msf.";
    return LOC_LOCALIZATION_ERROR;
  }

  ContinueSetDataAndThread(true);
  LC_LINFO(LOCALIZATION) << "SwitchOrigin done, continue system thread.";

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationManagerImpl::LoadMapOrigin(
    const std::string& file_path) {
  std::ifstream fin(file_path, std::fstream::in | std::fstream::binary);
  if (!fin.is_open()) {
    LC_LERROR(LOCALIZATION) << "failed to load file path: " << file_path;
    return LOC_LOCALIZATION_ERROR;
  }

  PointLLH_t origin;

  std::string s;
  std::getline(fin, s);
  // get lon
  auto pos_lon = s.find("\"lon\": ");
  if (pos_lon == s.npos) {
    LC_LERROR(LOCALIZATION) << "can't find lon";
    return LOC_LOCALIZATION_ERROR;
  }
  s = s.substr(pos_lon + 7, s.size() - pos_lon - 7);
  auto lon_str = s.substr(0, s.find(","));
  origin.lon = std::stod(lon_str);

  // get lat
  auto pos_lat = s.find("\"lat\": ");
  if (pos_lat == s.npos) {
    LC_LERROR(LOCALIZATION) << "can't find lat";
    return LOC_LOCALIZATION_ERROR;
  }
  s = s.substr(pos_lat + 7, s.size() - pos_lat - 7);
  auto lat_str = s.substr(0, s.find(","));
  origin.lat = std::stod(lat_str);

  // get height
  auto pos_height = s.find("\"height\": ");
  if (pos_height == s.npos) {
    LC_LERROR(LOCALIZATION) << "can't find height";
    return LOC_LOCALIZATION_ERROR;
  }
  s = s.substr(pos_height + 10, s.size() - pos_height - 10);
  auto height_str = s.substr(0, s.find("}"));
  origin.height = std::stod(height_str);

  // set and save origin
  CoordinateConverter::GetInstance()->SetOrigin(origin);
  if (param_.ci_param.enable_evaluation) SaveLocOriginLLA(origin);

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationManagerImpl::CreateLocalizationFrontend() {
  // create frontend instance
  switch (front_locator_type_) {
    case CAN:
      loc_frontend_ = std::make_shared<LocalizationFrontendCAN>();
      break;
    case IMU:
      loc_frontend_ = std::make_shared<LocalizationFrontendIMU>();
      break;
    case INS:
      loc_frontend_ = std::make_shared<LocalizationFrontendINS>();
      break;
    case REPLAY:
      loc_frontend_ = std::make_shared<LocalizationFrontendReplay>();
    default:
      LC_LERROR(LOCALIZATION) << "Unsupported frontend locator type.";
      return LOC_LOCALIZATION_ERROR;
  }
  if (nullptr == loc_frontend_) {
    LC_LERROR(LOCALIZATION) << "Failed to create frontend locator.";
    return LOC_ALLOC_MEMORY_FAILED;
  }
  if (loc_frontend_->Init(param_) != LOC_SUCCESS) {
    LC_LERROR(LOCALIZATION) << "Failed to init frontend locator.";
    return LOC_LOCALIZATION_ERROR;
  }

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationManagerImpl::BackendEngineFactory(LocatorType type) {
  LocalizationBackend::Ptr backend = nullptr;
  switch (type) {
    case GNSS:
      backend = std::make_shared<LocalizationBackendGNSS>();
      break;
    case SMM:
      backend = std::make_shared<LocalizationBackendSMM>();
      break;
    case HEADING:
      backend = std::make_shared<LocalizationBackendHeading>();
      break;
    case INS:
      backend = std::make_shared<LocalizationBackendINS>();
      break;
    case CAN:
      backend = std::make_shared<LocalizationBackendCAN>();
      break;
    default:
      LC_LERROR(LOCALIZATION)
          << "Unsupported backend locator type: " << LocatorTypeToString(type);
      return LOC_LOCALIZATION_ERROR;
  }
  if (nullptr == backend) {
    LC_LERROR(LOCALIZATION) << "Failed to create backend locator type: "
                            << LocatorTypeToString(type);
    return LOC_ALLOC_MEMORY_FAILED;
  }
  if (backend->Init(param_) != LOC_SUCCESS) {
    LC_LERROR(LOCALIZATION)
        << "Failed to init backend locator type: " << LocatorTypeToString(type);
    return LOC_LOCALIZATION_ERROR;
  }
  loc_backends_.insert(std::make_pair(type, backend));
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationManagerImpl::CreateLocalizationBackend() {
  // frontend/backend type check
  if (front_locator_type_ == INS || front_locator_type_ == REPLAY)
    return LOC_SUCCESS;
  if (back_locator_type_[NONE]) return LOC_SUCCESS;

  // push all backend types
  std::vector<LocatorType> types;
  if (back_locator_type_[GNSS]) types.emplace_back(GNSS);
  if (back_locator_type_[SMM]) types.emplace_back(SMM);
  if (front_locator_type_ != IMU && back_locator_type_[FINS])
    types.emplace_back(INS);
  if (front_locator_type_ == IMU && back_locator_type_[SMM])
    types.emplace_back(CAN);
  if (param_.common_param.using_dual_antenna && back_locator_type_[GNSS])
    types.emplace_back(HEADING);

  // create all backends
  for (size_t i = 0; i < types.size(); ++i) {
    adLocStatus_t status = BackendEngineFactory(types[i]);
    if (status != LOC_SUCCESS) return status;
  }

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationManagerImpl::CreateMSF() {
  // check whether MSF instance is needed
  if (front_locator_type_ == INS || front_locator_type_ == REPLAY)
    return LOC_SUCCESS;
  if (back_locator_type_[NONE]) return LOC_SUCCESS;
  if (loc_backends_.empty()) return LOC_SUCCESS;

  std::shared_ptr<BaseLocator> locator;
  switch (front_locator_type_) {
    case CAN: {
      locator =
          std::dynamic_pointer_cast<LocalizationFrontendCAN>(loc_frontend_)
              ->GetLocator();
      break;
    }
    case IMU: {
      locator =
          std::dynamic_pointer_cast<LocalizationFrontendIMU>(loc_frontend_)
              ->GetLocator();
      break;
    }
    default:
      LC_LERROR(LOCALIZATION) << "Unsupported frontend locator type";
      return LOC_LOCALIZATION_ERROR;
  }
  msf_fusion_ = msf::MSFFusionFactory::CreateMSFFusion(front_locator_type_,
                                                       locator, param_);
  if (nullptr == msf_fusion_) {
    LC_LERROR(LOCALIZATION) << "Failed to create MSF fusion.";
    return LOC_ALLOC_MEMORY_FAILED;
  }
  msf_fusion_->SetStart(true);
  if (msf_fusion_->Init() != LOC_SUCCESS) {
    LC_LERROR(LOCALIZATION) << "Failed to init MSF fusion.";
    return LOC_LOCALIZATION_ERROR;
  }

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationManagerImpl::CreateLocalizationDR() {
  loc_dead_reckoning_ = std::make_shared<LocalizationDeadReckoning>();
  if (nullptr == loc_dead_reckoning_) {
    LC_LERROR(LOCALIZATION) << "Failed to create DR locator.";
    return LOC_ALLOC_MEMORY_FAILED;
  }
  if (loc_dead_reckoning_->Init(param_) != LOC_SUCCESS) {
    LC_LERROR(LOCALIZATION) << "Failed to init DR locator.";
    return LOC_LOCALIZATION_ERROR;
  }

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationManagerImpl::CreateLocalizationMonitor() {
  loc_monitor_ = std::make_shared<LocalizationStateMonitor>(this);
  if (nullptr == loc_monitor_) {
    LC_LERROR(LOCALIZATION) << "Failed to create state monitor.";
    return LOC_ALLOC_MEMORY_FAILED;
  }
  if (loc_monitor_->Init(param_) != LOC_SUCCESS) {
    LC_LERROR(LOCALIZATION) << "Failed to init localziation monitor.";
    return LOC_LOCALIZATION_ERROR;
  }
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationManagerImpl::CreateLocalizationVisualizer() {
  loc_visualizer_ = std::make_shared<LocalizationVisualizer>();
  if (nullptr == loc_visualizer_) {
    LC_LERROR(LOCALIZATION) << "Failed to create visualizer.";
    return LOC_ALLOC_MEMORY_FAILED;
  }
  if (loc_visualizer_->Init(param_) != LOC_SUCCESS) {
    LC_LERROR(LOCALIZATION) << "Failed to init localziation visualizer.";
    return LOC_LOCALIZATION_ERROR;
  }
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationManagerImpl::SetModulePointerLink() {
  if (param_.common_param.absolute_localization) {
    loc_frontend_->SetMSFFusion(msf_fusion_);
    loc_frontend_->SetLocalizationBackends(loc_backends_);
    loc_frontend_->SetLocalizationVisualizer(loc_visualizer_);
    if (front_locator_type_ == LocatorType::IMU) {
      auto initializer = loc_frontend_->GetInitilizer();
      initializer->SetLocalizationDeadReckoning(loc_dead_reckoning_);
    }
    for (auto& loc_backend : loc_backends_) {
      loc_backend.second->SetMSFFusion(msf_fusion_);
      loc_backend.second->SetLocalizationFrontend(loc_frontend_);
      loc_backend.second->SetLocalizationDeadReckoning(loc_dead_reckoning_);
      loc_backend.second->SetLocalizationVisualizer(loc_visualizer_);
    }
  }
  if (param_.common_param.relative_localization) {
    loc_dead_reckoning_->SetLocalizationVisualizer(loc_visualizer_);
  }

  return LOC_SUCCESS;
}

void LocalizationManagerImpl::StopSetDataAndThread(bool is_abs_loc_mode) {
  if (!is_abs_loc_mode) {
    is_dr_stop_data_ = true;
    // waiting enough time stopping set data
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    // request dr thread paused
    loc_dead_reckoning_->RequestPause();
    while (!loc_dead_reckoning_->IsPaused())
      loc_dead_reckoning_->WaitPaused(1000);
  } else {
    is_loc_stop_data_ = true;
    // waiting enough time stopping set data
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    // request frontend, backend and visualizer thread paused
    loc_frontend_->RequestPause();
    while (!loc_frontend_->IsPaused()) loc_frontend_->WaitPaused(1000);
    for (auto& item : loc_backends_) {
      auto& loc_backend = item.second;
      loc_backend->RequestPause();
      while (!loc_backend->IsPaused()) loc_backend->WaitPaused(1000);
    }
  }
  if (loc_visualizer_) {
    loc_visualizer_->RequestPause();
    while (!loc_visualizer_->IsPaused()) loc_visualizer_->WaitPaused(1000);
  }
}

void LocalizationManagerImpl::ContinueSetDataAndThread(bool is_abs_loc_mode) {
  if (loc_visualizer_ && loc_visualizer_->IsPaused()) {
    loc_visualizer_->ReleasePause();
  }
  if (!is_abs_loc_mode) {
    // continue dr thread
    if (loc_dead_reckoning_->IsPaused()) loc_dead_reckoning_->ReleasePause();
    is_dr_stop_data_ = false;
  } else {
    // continue frontend, backend and visualizer thread
    if (loc_frontend_->IsPaused()) loc_frontend_->ReleasePause();
    for (auto& loc_bkend : loc_backends_) loc_bkend.second->ReleasePause();
    is_loc_stop_data_ = false;
  }
}

adLocStatus_t LocalizationManagerImpl::ConvertLocalMapCoord(
    uint64_t timestamp, std::shared_ptr<RoadStructure> local_map_data) {
  static double vehicle_height = 0.0;  // wgs84 altitude
  NavState cur_pose;
  if (LOC_SUCCESS == loc_frontend_->QueryPoseByTime(timestamp, &cur_pose)) {
    SE3d Tveh_ground;
    TransformConfig::GetTVehGround(&Tveh_ground);
    Eigen::Vector3d pst = (cur_pose.pose * Tveh_ground).translation();
    PointENU_t enu(pst(0), pst(1), pst(2));
    PointLLH_t lla;
    CoordinateConverter::GetInstance()->ENU2LLA(enu, &lla);
    vehicle_height = lla.height;
  } else {
    LC_LWARN(LOCALIZATION) << "search pose failed for localmap, timestamp: "
                           << timestamp;
  }

  static auto LLA2ENUPoint3D = [](const Point3D_t& point, double height) {
    PointLLH_t lla(point.x, point.y, height);
    PointENU_t enu;
    CoordinateConverter::GetInstance()->LLA2ENU(lla, &enu);
    return Point3D_t(enu.x, enu.y, enu.z);
  };

  // convert point coordinate from LLA to ENU
  auto coord_type = local_map_data->header.coord_type;
  bool is_wgs_coord =
      coord_type == senseAD::localization::CoordinateType::WGS ||
      coord_type == senseAD::localization::CoordinateType::GCJ02;
  for (auto& line : local_map_data->semantic_map_data.lines) {
    for (auto& segments : line.line_segments) {
      for (auto& point : segments.points) {
        if (is_wgs_coord) point = LLA2ENUPoint3D(point, vehicle_height);
      }
    }
  }
  for (auto& lane : local_map_data->route_map_data.lanes) {
    for (auto& point : lane.center_points) {
      if (is_wgs_coord) point = LLA2ENUPoint3D(point, vehicle_height);
    }
  }

  if (param_.smm_param.enable_percept_semantic) {
    // TODO(xxx): require ehr module publishing LLA for
    //            pole and traffic sign points
    for (auto& pole : local_map_data->semantic_map_data.poles) {
      if (is_wgs_coord) {
        // TODO(xx): compensate for height relative to the ground?
        pole.top_point = LLA2ENUPoint3D(pole.top_point, vehicle_height);
        pole.bottom_point = LLA2ENUPoint3D(pole.bottom_point, vehicle_height);
      }
    }
    for (auto& sign : local_map_data->semantic_map_data.traffic_signs) {
      if (is_wgs_coord) {
        // TODO(xx): compensate for height relative to the ground?
        sign.centroid = LLA2ENUPoint3D(sign.centroid, vehicle_height);
      }
    }
  }
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationManagerImpl::ConvertImuToBodyFrame(const Imu& raw_imu,
                                                             Imu* out_imu) {
  if (out_imu == nullptr) {
    LC_LERROR(LOCALIZATION) << "nullptr input.";
    return LOC_LOCALIZATION_ERROR;
  }
  Matrix3d Rbv = TransformConfig::GetTvb().so3().matrix().inverse();
  Matrix3d Rvi = TransformConfig::GetTvi().so3().matrix();
  Eigen::Vector3d raw_imu_acc;
  Eigen::Vector3d raw_imu_gyro;
  raw_imu_acc << raw_imu.linear_acceleration.x, raw_imu.linear_acceleration.y,
      raw_imu.linear_acceleration.z;
  raw_imu_gyro << raw_imu.angular_velocity.x, raw_imu.angular_velocity.y,
      raw_imu.angular_velocity.z;
  Eigen::Vector3d body_frame_imu_acc = Rbv * Rvi * raw_imu_acc;
  Eigen::Vector3d body_frame_imu_gyro = Rbv * Rvi * raw_imu_gyro;

  *out_imu = raw_imu;
  out_imu->linear_acceleration.x = body_frame_imu_acc(0);
  out_imu->linear_acceleration.y = body_frame_imu_acc(1);
  out_imu->linear_acceleration.z = body_frame_imu_acc(2);
  out_imu->angular_velocity.x = body_frame_imu_gyro(0);
  out_imu->angular_velocity.y = body_frame_imu_gyro(1);
  out_imu->angular_velocity.z = body_frame_imu_gyro(2);

  return LOC_SUCCESS;
}

void LocalizationManagerImpl::SaveLocOriginLLA(const PointLLH_t& origin_lla) {
  static uint64_t origin_id = 0;
  std::string tmp = origin_id == 0 ? "" : ("_" + std::to_string(origin_id));
  ++origin_id;
  std::string file_name = "FT-loc_origin" + tmp + ".txt";
  std::string file_path =
      JOIN_PATH(param_.ci_param.results_save_dir, file_name);

  std::ofstream output_stream;
  output_stream.open(file_path, std::ofstream::out | std::ofstream::trunc);
  if (!output_stream.is_open()) {
    LC_LERROR(SYSTEM) << "save origin cannot open file: " << file_path;
    return;
  }

  // write origin
  output_stream << std::setprecision(12) << std::fixed;
  output_stream << origin_lla.lon << " " << origin_lla.lat << " "
                << origin_lla.height;
  output_stream.close();
}

}  // namespace localization
}  // namespace senseAD
