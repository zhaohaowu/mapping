/*
 * Copyright (c) Hozon Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  ehp
 */
#include "map_fusion/map_service/ehp/amap_data.h"
#include <memory.h>

#include <cmath>
#include <ctime>
#include <iostream>

#include "common/configs/config_gflags.h"
#include "util/mapping_log.h"

namespace hozon {
namespace mp {
namespace mf {

// adasis V3 协议透出 demo
AdasisV3DataListenerImp::AdasisV3DataListenerImp()
    : listener_(nullptr), counter_(0) {}

AdasisV3DataListenerImp::~AdasisV3DataListenerImp() { listener_ = nullptr; }

bool AdasisV3DataListenerImp::shouldReceiveMessage() {
  if (ehp_data_.size() == ehp_data_.capacity()) {
    ehp_data_.reserve(10);
  }
  return true;
}

bool AdasisV3DataListenerImp::receiveGlobalData(
    const ::hdmap::service::GlobalData& globalData) {
  HLOG_DEBUG << "globalData";
  std::string glaobal_data;
  glaobal_data.assign(
      reinterpret_cast<const char*>(globalData.getData()),  // NOLINT
      globalData.getLength());

  // HLOG_ERROR << "globaldatalength: " << globalData.getLength();
  // apollo::adasisv3::MessageOnBus message;

  // message.ParseFromString(glaobal_data);
  // HLOG_ERROR << "globalmessage: " << message.DebugString();

  // 解析 globalData 消息成功,使用 message 实例对象获取相关属性
  std::lock_guard<std::mutex> lock(mutex_);
  ehp_data_.push_back(glaobal_data);
  return true;
}

bool AdasisV3DataListenerImp::receivePosition(
    const ::hdmap::service::PositionData& positionData) {
  HLOG_DEBUG << "positionData";
  std::string position_data;
  position_data.assign(
      reinterpret_cast<const char*>(positionData.getData()),  // NOLINT
      positionData.getLength());
  // HLOG_ERROR << "positiondatalength: " << positionData.getLength();
  // apollo::adasisv3::MessageOnBus message;

  // message.ParseFromString(position_data);
  // HLOG_ERROR << "positionmessage: " << message.DebugString();

  // 解析 position 消息成功,使用 message 实例对象获取相关属性
  std::lock_guard<std::mutex> lock(mutex_);
  ehp_data_.push_back(position_data);
  return true;
}

bool AdasisV3DataListenerImp::receivePathControl(
    const ::hdmap::service::PathControlData& pathControlData) {
  HLOG_DEBUG << "pathControlData";
  std::string pathcontrol_data;
  pathcontrol_data.assign(
      reinterpret_cast<const char*>(pathControlData.getData()),  // NOLINT
      pathControlData.getLength());

  // HLOG_ERROR << "pathControlDatalength: " << pathControlData.getLength();
  // apollo::adasisv3::MessageOnBus message;

  // message.ParseFromString(pathcontrol_data);
  // HLOG_ERROR << "pathcontrolmessage: " << message.DebugString();

  // 解析 pathcontrol 消息成功,使用 message 实例对象获取相关属性
  std::lock_guard<std::mutex> lock(mutex_);
  ehp_data_.push_back(pathcontrol_data);
  return true;
}

bool AdasisV3DataListenerImp::receiveProfileControl(
    const ::hdmap::service::ProfileControlData& profileControlData) {
  HLOG_DEBUG << "profileControlData";
  std::string profilecontrol_data;
  profilecontrol_data.assign(
      reinterpret_cast<const char*>(profileControlData.getData()),  // NOLINT
      profileControlData.getLength());

  // HLOG_ERROR << "profileControlDatalength: " <<
  // profileControlData.getLength(); apollo::adasisv3::MessageOnBus message;

  // message.ParseFromString(profilecontrol_data);
  // HLOG_ERROR << "profileControlmessage: " << message.DebugString();

  // 解析 profileControl 消息成功,使用 message 实例对象获取相关属性
  std::lock_guard<std::mutex> lock(mutex_);
  ehp_data_.push_back(profilecontrol_data);
  return true;
}

bool AdasisV3DataListenerImp::receiveProfile(
    const ::hdmap::service::ProfileData& profileData) {
  HLOG_DEBUG << "profiledata";
  // apollo::adasisv3::MessageOnBus message;
  std::string profile_data;
  profile_data.assign(
      reinterpret_cast<const char*>(profileData.getData()),  // NOLINT
      profileData.getLength());

  // HLOG_ERROR << "profileDatalength: " << profileData.getLength();
  // apollo::adasisv3::MessageOnBus message;

  // message.ParseFromString(profile_data);
  // HLOG_ERROR << "profilemessage: " << message.DebugString();

  // 解析 profile 消息成功,使用 message 实例对象获取相关属性
  std::lock_guard<std::mutex> lock(mutex_);
  ehp_data_.push_back(profile_data);
  std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(5)));
  return true;
}

bool AdasisV3DataListenerImp::receiveReason(
    const ::hdmap::service::ReasonData& reasonData) {
  HLOG_DEBUG << "reason message";
  // apollo::adasisv3::MessageOnBus message;
  std::string reason_message;
  reason_message.assign(
      reinterpret_cast<const char*>(reasonData.getData()),  // NOLINT
      reasonData.getLength());

  // HLOG_ERROR << "profileDatalength: " << profileData.getLength();
  // apollo::adasisv3::MessageOnBus message;

  // message.ParseFromString(profile_data);
  // HLOG_ERROR << "profilemessage: " << message.DebugString();

  // 解析 profile 消息成功,使用 message 实例对象获取相关属性
  std::lock_guard<std::mutex> lock(mutex_);
  ehp_data_.push_back(reason_message);
  return true;
}

void AdasisV3DataListenerImp::setAdasisV3Listener(
    ::hdmap::service::IAdasisV3MessageListener* adasisV3Listener) {
  listener_ = adasisV3Listener;
}

bool AdasisV3DataListenerImp::GetEhpData(std::vector<EhpData>* const ehp_data) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!ehp_data_.empty()) {
    ++counter_;
    ehp_data_list_.emplace_back(counter_, ehp_data_);
    ehp_data_.clear();
  }
  // DeleteDataByCounter(min_counter);
  *ehp_data = ehp_data_list_;
  ehp_data_list_.clear();
  return true;
}

::hdmap::service::IAdasisV3MessageListener*
AdasisV3DataListenerImp::GetListener() {
  return listener_;
}

void AdasisV3DataListenerImp::DeleteDataByCounter(int min_counter) {
  static constexpr int ehp_data_max_number = 500;
  if (!ehp_data_list_.empty() && ehp_data_list_.back().first > min_counter) {
    for (auto data_it = ehp_data_list_.begin();
         data_it != ehp_data_list_.end();) {
      if (data_it->first <= min_counter) {
        data_it = ehp_data_list_.erase(data_it);
      } else {
        ++data_it;
      }
    }
  }
  if (ehp_data_list_.size() > ehp_data_max_number) {
    int range = static_cast<int>(ehp_data_list_.size()) - ehp_data_max_number;
    ehp_data_list_.erase(ehp_data_list_.begin(),
                         ehp_data_list_.begin() + range);
  }
}

bool AdasisV3DataListenerImp::ReInit() {
  counter_ = 0;
  ehp_data_list_.clear();
  ehp_data_.clear();
  return true;
}
}  // namespace mf
}  // namespace mp
}  // namespace hozon
