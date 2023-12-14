/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： rviz_agent.h
 *   author     ： taoshaoyuan
 *   date       ： 2022.10
 ******************************************************************************/

#pragma once

#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <zmq/zmq.hpp>

// auto generated
#include "adsfi_proto/viz/geometry_msgs.pb.h"
#include "adsfi_proto/viz/nav_msgs.pb.h"
#include "adsfi_proto/viz/sensor_msgs.pb.h"
#include "adsfi_proto/viz/tf2_msgs.pb.h"
#include "adsfi_proto/viz/visualization_msgs.pb.h"
#include "modules/util/include/util/mapping_log.h"
#include "modules/util/include/util/nodelink/core/pub_worker.h"
#include "modules/util/include/util/rviz_agent/msg_alias.h"

namespace hozon {
namespace mp {
namespace util {

class RvizAgent {
 public:
  //! Not thread safe
  int Init(const std::string& addr);

  template <typename ProtoType>
  typename Checker<ProtoType>::Int Register(const std::string& topic) {
    if (topic.empty()) {
      HLOG_ERROR << "empty topic";
      return -1;
    }

    std::string type_alias = GetTypeAlias<ProtoType>();
    if (type_alias.empty()) {
      HLOG_ERROR << "unsupported type";
      return -1;
    }

    std::lock_guard<std::mutex> lock(ctrl_mtx_);
    if (reg_msgs_.find(topic) != reg_msgs_.end()) {
      HLOG_ERROR << "topic " << topic << " already registered";
      return -1;
    }
    reg_msgs_[topic] = type_alias;
    return 0;
  }

  template <typename ProtoType>
  typename Checker<ProtoType>::Int Register(
      const std::vector<std::string>& topics) {
    for (const auto& t : topics) {
      int ret = Register<ProtoType>(t);
      if (ret < 0) {
        HLOG_ERROR << "register topics failed";
        return -1;
      }
    }
    return 0;
  }

  void UnRegister(const std::string& topic);

  bool Registered(const std::string& topic);

  template <typename ProtoType>
  typename Checker<ProtoType>::Int Publish(const std::string& topic,
                                           const ProtoType& msg) {
    if (!CheckPub(topic)) {
      HLOG_ERROR << "PubCheck error";
      return -1;
    }

    std::string ser = msg.SerializeAsString();
    pub_->AddData(topic, reinterpret_cast<void*>(ser.data()), ser.size());
    return 0;
  }

  template <typename ProtoType>
  typename Checker<ProtoType>::Int Publish(
      const std::string& topic, const std::shared_ptr<ProtoType>& msg_ptr) {
    if (!CheckPub(topic)) {
      HLOG_ERROR << "PubCheck error";
      return -1;
    }

    if (!msg_ptr) {
      HLOG_ERROR << "nullptr msg ptr";
      return -1;
    }

    std::string ser = msg_ptr->SerializeAsString();
    pub_->AddData(topic, reinterpret_cast<void*>(ser.data()), ser.size());
    return 0;
  }

  int Publish(const std::string& topic, const std::string& data_str);

  void Term();
  bool Ok() const;

 private:
  std::unique_ptr<std::thread> ctrl_thread_ = nullptr;
  std::mutex ctrl_mtx_;
  bool running_ = false;
  // <topic, type_alias>
  std::map<std::string, std::string> reg_msgs_;
  void CtrlLoop();

  bool CheckPub(const std::string& topic);
  static bool CheckAddr(const std::string& addr);

  std::shared_ptr<PubWorker> pub_ = nullptr;

  // declare singleton
 private:
  RvizAgent() = default;
  ~RvizAgent() noexcept;
  RvizAgent(const RvizAgent&);
  RvizAgent& operator=(const RvizAgent&);

 public:
  static RvizAgent& Instance() {
    static RvizAgent instance;
    return instance;
  }
};

#define RVIZ_AGENT hozon::mp::util::RvizAgent::Instance()

}  // namespace util
}  // namespace mp
}  // namespace hozon
