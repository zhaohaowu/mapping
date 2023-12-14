/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： node.cc
 *   author     ： taoshaoyuan
 *   date       ： 2023.01
 ******************************************************************************/

#include "util/nodelink/core/node.h"

#include "util/mapping_log.h"
#include "util/nodelink/core/context.h"
#include "util/nodelink/core/pub_worker.h"

namespace hozon {
namespace mp {

const std::string kLinkerBindAddr = "inproc://LinkerBindAddr";  // NOLINT

int Node::InitNode(const std::string& config) {
  class_name_ = typeid(*this).name();
  if (Init(config) < 0) {
    HLOG_ERROR << class_name_ << " Init() failed";
    return -1;
  }

  if (sub_ && sub_->Init() < 0) {
    HLOG_ERROR << class_name_ << " SubWorker Init() failed";
    return -1;
  }

  if (pub_ && pub_->Init() < 0) {
    HLOG_ERROR << class_name_ << " PubWorker Init() failed";
    return -1;
  }

  return 0;
}

int Node::StartNode() {
  if (Start() < 0) {
    HLOG_ERROR << class_name_ << " Start() failed";
    return -1;
  }

  if (sub_) {
    sub_->Start();
  }
  return 0;
}

void Node::Term() {
  Stop();

  if (sub_) {
    sub_->Term();
    sub_ = nullptr;
  }
  if (pub_) {
    pub_->Term();
    pub_ = nullptr;
  }
}

std::vector<std::string> Node::SubAddrs() {
  if (!sub_) {
    return {};
  }
  return sub_->Addrs();
}

std::vector<std::string> Node::PubAddrs() {
  if (!pub_) {
    return {};
  }
  return pub_->Addrs();
}

std::vector<std::string> Node::SubTopics() {
  std::vector<std::string> ret(sub_topics_.begin(), sub_topics_.end());
  return ret;
}

std::vector<std::string> Node::PubTopics() {
  std::vector<std::string> ret(pub_topics_.begin(), pub_topics_.end());
  return ret;
}

PublisherPtr Node::Advertise(const std::string& topic) {
  if (!pub_) {
    std::string class_name(typeid(*this).name());
    std::vector<std::string> pub_addrs = {//        "inproc://" + Name()
                                          "inproc://" + class_name};
    void* ctx = Context::Instance().Get();
    pub_ = PubWorker::Create(ctx, pub_addrs);
  }

  if (pub_topics_.count(topic) != 0) {
    HLOG_ERROR << "topic " << topic << " already advertised";
    return nullptr;
  }

  auto p = Publisher::Create(topic, pub_);
  pub_topics_.insert(topic);

  return p;
}

void Node::Subscribe(const std::string& topic, SubCallback cbk) {
  if (!sub_) {
    void* ctx = Context::Instance().Get();
    std::vector<std::string> sub_addrs = {kLinkerBindAddr};
    sub_ = SubWorker::Create(ctx, sub_addrs);
  }

  if (sub_topics_.count(topic) != 0) {
    HLOG_ERROR << "topic " << topic << " already subscribed";
    return;
  }

  sub_->Reg(topic, std::move(cbk));
  sub_topics_.insert(topic);
}

}  // namespace mp
}  // namespace hozon
