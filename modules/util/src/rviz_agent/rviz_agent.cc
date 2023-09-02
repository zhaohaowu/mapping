/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： rviz_agent.cc
 *   author     ： taoshaoyuan
 *   date       ： 2022.10
 ******************************************************************************/

#include "util/rviz_agent/rviz_agent.h"

#include <adsfi_proto/viz/msg_info.pb.h>

#include "util/nodelink/core.h"

namespace hozon {
namespace mp {
namespace util {

using namespace std::chrono_literals;

RvizAgent::~RvizAgent() noexcept { Term(); }

int RvizAgent::Init(const std::string& addr) {
  if (running_) {
    HLOG_WARN << "RvizAgent already started";
    return -1;
  }

  if (!CheckAddr(addr)) {
    HLOG_ERROR << "invalid addr " << addr;
    return -1;
  }

  std::vector<std::string> pub_addrs = {addr};
  void* ctx = Context::Instance().Get();
  pub_ = PubWorker::Create(ctx, pub_addrs);
  if (pub_ && pub_->Init() < 0) {
    HLOG_ERROR << "PubWorker Init() failed";
    return -1;
  }

  running_ = true;
  ctrl_thread_ = std::make_unique<std::thread>(&RvizAgent::CtrlLoop, this);

  return 0;
}

void RvizAgent::UnRegister(const std::string& topic) {
  if (topic.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(ctrl_mtx_);
  if (reg_msgs_.find(topic) != reg_msgs_.end()) {
    reg_msgs_.erase(topic);
  }
}

bool RvizAgent::Registered(const std::string& topic) {
  if (topic.empty()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(ctrl_mtx_);
  if (reg_msgs_.find(topic) == reg_msgs_.end()) {
    return false;
  }
  return true;
}

int RvizAgent::Publish(const std::string& topic, const std::string& data_str) {
  if (!CheckPub(topic)) {
    HLOG_ERROR << "PubCheck error";
    return -1;
  }

  pub_->AddData(topic, (void*)data_str.data(), data_str.size());
  return 0;
}

void RvizAgent::Term() {
  running_ = false;

  if (ctrl_thread_ && ctrl_thread_->joinable()) {
    ctrl_thread_->join();
    ctrl_thread_ = nullptr;
  }

  if (pub_) {
    pub_->Term();
    pub_ = nullptr;
  }

  reg_msgs_.clear();
}

void RvizAgent::CtrlLoop() {
  pthread_setname_np(pthread_self(), "rviz_agent_ctrl_loop");

  std::string prefix = "CtrlLoop: ";
  while (running_) {
    if (!pub_) {
      HLOG_ERROR << prefix << "PubWorker nullptr";
      std::this_thread::sleep_for(1s);
      continue;
    }

    adsfi_proto::viz::MsgInfoArray msgs;
    {
      std::lock_guard<std::mutex> lock(ctrl_mtx_);
      for (const auto& it : reg_msgs_) {
        auto msg = msgs.mutable_msgs()->Add();
        msg->set_topic(it.first);
        msg->set_alias(it.second);
      }
    }
    std::string ser_msgs = msgs.SerializeAsString();
    if (!ser_msgs.empty()) {
      pub_->AddData(kCtrlTopic, (void*)ser_msgs.data(), ser_msgs.size());
    }

    std::this_thread::sleep_for(1s);
  }
}

bool RvizAgent::Ok() const { return running_; }

bool RvizAgent::CheckPub(const std::string& topic) {
  if (topic.empty() || !Registered(topic)) {
    HLOG_ERROR << "topic is empty or unregistered";
    return false;
  }

  if (!Ok()) {
    HLOG_ERROR << "RvizAgent not running";
    return false;
  }

  if (!pub_) {
    HLOG_ERROR << "nullptr PubWorker";
    return false;
  }

  return true;
}

bool RvizAgent::CheckAddr(const std::string& addr) {
  if (addr.empty()) {
    return false;
  }

  if (addr.rfind("tcp://", 0) != 0 && addr.rfind("ipc:///", 0) != 0 &&
      addr.rfind("inproc://", 0) != 0) {
    return false;
  }

  return true;
}

}  // namespace util
}  // namespace mp
}  // namespace hozon