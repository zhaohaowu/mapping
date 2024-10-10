/*
 * Copyright (c) Hozon Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  ehp
 */
#include "amap/DataDefine.h"
#ifdef _MSC_VER
#define timegm _mkgmtime
#endif
#include <memory.h>

#include <algorithm>
#include <cmath>
#include <ctime>
#include <iostream>

#include "common/configs/config_gflags.h"
#include "common/utm_projection/coordinate_convertor.h"
#include "modules/map_fusion/data_manager/location_data_manager.h"
#include "modules/map_fusion/modules/map_hd/amap_core.h"
#include "util/mapping_log.h"
namespace hozon {
namespace mp {
namespace mf {
void locSignalCallback(const LocSignDataT& signData) {
  ::hdmap::service::GHDMapService::getInstance().setLocSignal(signData);
}

AmapAdapter::AmapAdapter()
    : is_init_(false),
      start_timestamp_(0.0),
      message_state_(false),
      current_time_(0.0),
      message_open_(false) {}

AmapAdapter::~AmapAdapter() {
  ::hdmap::service::GHDMapService::getInstance().deInit();
  HLOG_INFO << "~EhpComponent";
}

void AmapAdapter::SetUUID(const std::string& uuid) { uid_ = uuid; }

bool AmapAdapter::Init() {
  start_timestamp_ = hozon::common::Clock::NowInSeconds();
  std::string default_work_root = "/app/";
  std::string work_root =
      hozon::perception::lib::GetEnv("ADFLITE_ROOT_PATH", default_work_root);
  std::string data_root_path = work_root + "/data/map/amap_all/hd_map";
#ifdef ISMDC
  data_root_path = "/opt/usr/hd_map";
#endif
#ifdef ISORIN
  data_root_path = "/hd_map";
#endif

  // std::string log_path =
  //     data_root_path + "/fake_shape_order_new.loc";  // log路径
  // step 1: 获取 GHDMapService 单例
  ::hdmap::service::GHDMapService& service =
      ::hdmap::service::GHDMapService::getInstance();
  // step 2: 设置高精 sdk 的配置文件路径
  ::hdmap::service::HDMapConfig config;
  config.basicDataRootDir = data_root_path;
#if (defined ISMDC) || (defined ISORIN)
  config.logRootDir = FLAGS_ehp_log_root_dir;  // NOLINT
#endif
  // step 3: 设置 adasisV3 message impl 实例
  ehp_data_listener_ = std::make_shared<AdasisV3DataListenerImp>();
  if (ehp_data_listener_) {
    service.setAdasisV3MessageReceiver(ehp_data_listener_.get());
  }
  // step 4: 设置系统接口 impl 实例

  ehp_system_ = std::make_shared<SystemDeviceImp>();
  if (ehp_system_) {
    service.setSystemDevice(ehp_system_.get());
  }
  // step 5: 设置系统接口 impl 实例
  ehp_diagnose_ = std::make_shared<DiagnosisImp>();
  if (ehp_diagnose_) {
    service.setHDMapDiagnosisReceiver(ehp_diagnose_.get());
  }

  // 设置ota实例
  ehp_ota_ = std::make_shared<OtaDeviceImp>();
  if (ehp_ota_) {
    service.setDOTAReceiver(ehp_ota_.get());
  }

  // service.setResimLocSignalCallback(locSignalCallback);
  // step 7: GHDMapService 初始化
  if (!service.init(config, false)) {
    service.deInit();  // init 和 deInit 方法要成对出现
    HLOG_ERROR << "GHDMapService init error";
    is_init_ = false;
    return true;
  }
  is_init_ = true;
  MessageClose();
  HLOG_INFO << "GHDMapService init success";

  current_time_ = 0.0;
  message_open_ = false;
  HLOG_INFO << "sdk vision: " << ::hdmap::service::GHDMapService::mapVersion();
  return true;
}

// NOLINTBEGIN
bool AmapAdapter::Process(
    const hozon::localization::HafNodeInfo& localization_input,
    std::vector<EhpData>* const ehp_data) {
  if (!is_init_) {
    return false;
  }
  ehp_system_->SetUUID(uid_);
  // const auto& localization = localization_input;
  current_time_ = hozon::common::Clock::NowInSeconds();
  bool over_time(false);
  // if (FLAGS_ehp_monitor == 2 || FLAGS_ehp_monitor == 0) {  // NOLINT
  //   if (adc_trajectory.has_header()) {
  //     over_time = IsOverTime(adc_trajectory.header().publish_stamp());
  //     if (over_time) {
  //       HLOG_ERROR << "adc overtime";
  //     }
  //   }
  // }

  // if (!FLAGS_enable_planning_self_simulator && !over_time) {  // NOLINT
  //   if (localization_input.has_header()) {
  //     over_time = IsOverTime(localization_input.header().publish_stamp());
  //     if (over_time) {
  //       HLOG_ERROR << "loc overtime";
  //     }
  //   }
  // }
  if (localization_input.has_header()) {
    over_time = IsOverTime(localization_input.header().publish_stamp());
    if (over_time) {
      HLOG_INFO << "loc overtime";
    }
  }

  if (over_time) {
    if (message_open_) {
      MessageClose();
      message_open_ = false;
      HLOG_INFO << "amap sdk close";
    }
    return false;
  }
  if (!message_open_) {
    ehp_data_listener_->ReInit();
    MessageOpen();
    message_open_ = true;
    HLOG_INFO << "amap sdk open";
  }

  LocSignDataT sign_data{};
  sign_data.dataType = LocDataGnss_T;
  sign_data.gnss.isValid = true;
  sign_data.gnss.sourType = 0;
  sign_data.gnss.mode = '0';
  sign_data.gnss.status = 'A';
  sign_data.gnss.isEncrypted = 1;
  sign_data.gnss.isNS = '0';
  sign_data.gnss.isEW = 'w';

  sign_data.gnss.lon =
      static_cast<int>(localization_input.pos_gcj02().y() * 1e6);
  sign_data.gnss.lat =
      static_cast<int>(localization_input.pos_gcj02().x() * 1e6);
  sign_data.gnss.lonS =
      static_cast<int>(localization_input.pos_wgs().y() * 1e6);
  sign_data.gnss.latS =
      static_cast<int>(localization_input.pos_wgs().x() * 1e6);
  auto speed = sqrt(pow(localization_input.linear_velocity().x(), 2) +
                    pow(localization_input.linear_velocity().y(), 2));
  // HLOG_ERROR << "current lon " << sign_data.gnss.lon << " lat "
  //            << sign_data.gnss.lat << "zr123 localization_input.heading: "
  //            << localization_input.heading();

  if (speed < 0.1) {
    speed = 10;
  } else {
    speed *= 3.6;
  }
  sign_data.gnss.speed = speed;

  // sign_data.gnss.course = 90.0 - localization.heading() * 180 / M_PI;
  // if (sign_data.gnss.course < 0) {
  //   sign_data.gnss.course += 360;
  // }
  // if (sign_data.gnss.course > 360) {
  //   sign_data.gnss.course -= 360;
  // }
  sign_data.gnss.course = localization_input.heading();
  // HLOG_ERROR << "ehp heading: " << sign_data.gnss.course;
  sign_data.gnss.num = 20;
  sign_data.gnss.hdop = 1.0;
  sign_data.gnss.vdop = -1.0;
  sign_data.gnss.pdop = -1.0;
  auto time = hozon::common::Clock::NowInSeconds();
  time_t time_stamp = static_cast<int>(time);

  struct tm timeinfo {};

  gmtime_r(&time_stamp, &timeinfo);
  sign_data.gnss.year = timeinfo.tm_year + 1900;
  sign_data.gnss.month = timeinfo.tm_mon + 1;
  sign_data.gnss.day = timeinfo.tm_mday;
  sign_data.gnss.hour = timeinfo.tm_hour;
  sign_data.gnss.minute = timeinfo.tm_min;
  sign_data.gnss.second = timeinfo.tm_sec;
  sign_data.gnss.milliSec = static_cast<int>((time - time_stamp) * 1000);
  sign_data.gnss.accuracy = 0;
  sign_data.gnss.courseAccuracy = 0;
  // sqrt(pow(localization.uncertainty().position_std_dev().x(), 2) +
  //      pow(localization.uncertainty().position_std_dev().y(), 2));
  // HLOG_ERROR << "uncertainty: "
  //        << sqrt(pow(localization.uncertainty().position_std_dev().x(),
  //        2) +
  //                pow(localization.uncertainty().position_std_dev().y(),
  //                2));

  // HLOG_ERROR << "courseAccuracy: "
  //        << localization.uncertainty().position_std_dev().z();
  int timestamp = static_cast<int>(
      (hozon::common::Clock::NowInSeconds() - start_timestamp_) * 1000);
  sign_data.gnss.tickTime = timestamp;
  sign_data.gnss.tickTimeSys = timestamp;
  // NOLINTEND

  ::hdmap::service::GHDMapService::getInstance().setLocSignal(sign_data);

  // int planing_counter = INT_MAX;
  // int loc_counter = INT_MAX;
  // int min_counter = INT_MAX;

  // if (adc_trajectory.has_received_ehp_counter() &&
  //     adc_trajectory.received_ehp_counter() != -1) {
  //   planing_counter = adc_trajectory.received_ehp_counter();
  // }
  // if (localization.has_received_ehp_counter() &&
  //     localization.received_ehp_counter() != -1) {
  //   loc_counter = localization.received_ehp_counter();
  // }

  // if (FLAGS_enable_planning_self_simulator) {  // NOLINT
  //   min_counter = planing_counter;
  // } else if (FLAGS_ehp_monitor == 1) {  // NOLINT
  //   min_counter = loc_counter;
  // } else {
  //   min_counter = std::min(loc_counter, planing_counter);
  // }

  ehp_data_listener_->GetEhpData(ehp_data);
  return true;
}
// NOLINTEND

bool AmapAdapter::IsOverTime(double new_time) const {
  if (std::abs(current_time_ - new_time) > 3.0 &&
      std::abs(current_time_ - new_time) < 200.0) {
    return true;
  }
  if (std::abs(current_time_ - new_time) > 200.0) {
    HLOG_INFO << "play record data";
    return false;
  }
  return false;
}

bool AmapAdapter::GetInitState() const { return is_init_; }

// void AmapAdapter::MessageClose() {
//   if (ehp_data_listener_->GetListener() == NULL) {
//   } else {
//     ehp_data_listener_->GetListener()->closeMessageOutput();
//   }
// }

// void AmapAdapter::MessageOpen() {
//   if (ehp_data_listener_->GetListener() == NULL) {
//   } else {
//     ehp_data_listener_->GetListener()->openMessageOutput();
//   }
// }
void AmapAdapter::MessageClose() const {
  if (!is_init_) {
    HLOG_ERROR << "GHDMapService  not init";
  } else {
    ::hdmap::service::GHDAv3Service::getService()->closeMessageOutput();
    HLOG_INFO << "Close AV3 Service Successful";
  }
}

void AmapAdapter::MessageOpen() const {
  if (!is_init_) {
    HLOG_ERROR << "GHDMapService  not init";
  } else {
    ::hdmap::service::GHDAv3Service::getService()->openMessageOutput();
    HLOG_INFO << "Open AV3 Service Successful";
  }
}

bool AmapAdapter::GetDiagnose(::hdmap::service::DiagnosisState* state,
                              ::hdmap::service::DiagnosisType* type,
                              std::string* detailMessage) {
  if (!is_init_) {
    HLOG_ERROR << "ehp init fail";
    return false;
  }
  *state = ehp_diagnose_->GetDiagnosisState();
  *type = ehp_diagnose_->GetDiagnosisType();
  *detailMessage = ehp_diagnose_->GetDetailMessage();
  return true;
}
}  // namespace mf

}  // namespace mp
}  // namespace hozon
