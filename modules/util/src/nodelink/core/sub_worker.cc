/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： sub_worker.cc
 *   author     ： taoshaoyuan
 *   date       ： 2023.01
 ******************************************************************************/

#include "util/nodelink/core/sub_worker.h"
#include <zmq/zmq.h>

#include <utility>

#include "util/temp_log.h"

namespace hozon {
namespace mp {

int SubWorker::Init() {
  if (running_.load()) {
    HLOG_ERROR << "SubWorker already running";
    return -1;
  }

  if ((ctx_ == nullptr) || addrs_.empty()) {
    return -1;
  }
  for (const auto& addr : addrs_) {
    if (addr.empty()) {
      return -1;
    }
  }

  term_skt_ = zmq_socket(ctx_, ZMQ_PUB);
  term_addr_ = "inproc://" + Id();
  term_content_ = Id();
  HLOG_INFO << "SubWorker term_addr_ " << term_addr_;
  int ret = zmq_bind(term_skt_, term_addr_.c_str());
  if (ret < 0 && (term_skt_ != nullptr)) {
    HLOG_ERROR << "zmq_bind term addr " << term_addr_ << " error";
    zmq_close(term_skt_);
    term_skt_ = nullptr;
    return -1;
  }

  running_.store(true);
  sub_thread_ = std::make_unique<std::thread>(&SubWorker::Loop, this);

  return 0;
}

void SubWorker::Start() { started_.store(true); }

void SubWorker::Term() {
  running_.store(false);
  started_.store(false);
  if (SendTerm() < 0) {
    HLOG_ERROR << "SendTerm failed, something unexpected may happen";
  }

  if (sub_thread_ && sub_thread_->joinable()) {
    sub_thread_->join();
    sub_thread_ = nullptr;
  }

  if (term_skt_ != nullptr) {
    zmq_close(term_skt_);
    term_skt_ = nullptr;
  }

  {
    std::lock_guard<std::mutex> lock(mtx_);
    all_cbk_.clear();
    all_topics_cbk_ = nullptr;
  }

  term_addr_.clear();
  term_content_.clear();
}

std::vector<std::string> SubWorker::Addrs() { return addrs_; }

void SubWorker::Reg(const std::string& topic, SubCallback callback) {
  std::lock_guard<std::mutex> lock(mtx_);
  all_cbk_[topic] = std::move(callback);
}

void SubWorker::UnReg(const std::string& topic) {
  std::lock_guard<std::mutex> lock(mtx_);
  auto it = all_cbk_.find(topic);
  if (it != all_cbk_.end()) {
    all_cbk_.erase(it);
  }
}

void SubWorker::RegAll(SubAllCallback callback) {
  all_topics_cbk_ = std::move(callback);
}

void SubWorker::UnRegAll() { all_topics_cbk_ = nullptr; }

void SubWorker::Loop() {
  if (!OpenSocket()) {
    HLOG_ERROR << "OpenSocket failed";
    return;
  }

  while (running_.load()) {
    zmq_msg_t recv_topic;
    zmq_msg_t recv_content;
    if (!RecvMsg(&recv_topic, &recv_content)) {
      HLOG_ERROR << "RecvMsg failed";
      continue;
    }

    void* topic_data = zmq_msg_data(&recv_topic);
    size_t topic_size = zmq_msg_size(&recv_topic);
    void* content_data = zmq_msg_data(&recv_content);
    size_t content_size = zmq_msg_size(&recv_content);

    std::string topic(static_cast<const char*>(topic_data), topic_size);

    if (topic == kTermTopic &&
        std::string(static_cast<const char*>(content_data), content_size) ==
            term_content_) {
      HLOG_INFO << "receive term msg";
      zmq_msg_close(&recv_topic);
      zmq_msg_close(&recv_content);
      break;
    }

    if (all_topics_cbk_ != nullptr) {
      std::lock_guard<std::mutex> lock(mtx_);
      all_topics_cbk_(topic, content_data, content_size);
    }

    {
      std::lock_guard<std::mutex> lock(mtx_);
      auto it = all_cbk_.find(topic);
      if (started_.load() && it != all_cbk_.end()) {
        auto cbk = it->second;
        cbk(content_data, content_size);
      }
    }
    zmq_msg_close(&recv_topic);
    zmq_msg_close(&recv_content);
  }

  if (skt_ != nullptr) {
    zmq_close(skt_);
    skt_ = nullptr;
  }
}

void SubWorker::GenId() {
  const auto* this_ptr = static_cast<const void*>(this);
  std::stringstream ss;
  ss << this_ptr;
  id_ = ss.str();
}

std::string SubWorker::Id() { return id_; }

int SubWorker::SendTerm() {
  if (term_skt_ == nullptr) {
    HLOG_INFO << "invalid term_skt_, maybe already termed";
    return 0;
  }
  int ret =
      zmq_send(term_skt_, kTermTopic.c_str(), kTermTopic.length(), ZMQ_SNDMORE);
  if (ret < 0) {
    HLOG_ERROR << "zmq_send " << kTermTopic << " error";
    return -1;
  }
  ret = zmq_send(term_skt_, term_content_.c_str(), term_content_.length(), 0);
  if (ret < 0) {
    HLOG_ERROR << "zmq_send " << term_content_ << " error";
    return -1;
  }
  return 0;
}

bool SubWorker::OpenSocket() {
  std::vector<std::string> all_addr(addrs_);
  all_addr.push_back(term_addr_);

  std::vector<std::string> all_topic;
  all_topic.push_back(kTermTopic);
  if (all_topics_cbk_ != nullptr) {
    all_topic.emplace_back("");
  }
  {
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto& it : all_cbk_) {
      all_topic.push_back(it.first);
    }
  }

  skt_ = zmq_socket(ctx_, ZMQ_SUB);
  bool err = false;
  int ret = 0;
  for (const auto& addr : all_addr) {
    ret = zmq_connect(skt_, addr.c_str());
    if (ret < 0) {
      HLOG_ERROR << "zmq_connect " << addr << " error";
      err = true;
    }
  }

  for (const auto& t : all_topic) {
    ret = zmq_setsockopt(skt_, ZMQ_SUBSCRIBE, t.c_str(), t.length());
    if (ret < 0) {
      HLOG_ERROR << "zmq_setsockopt error";
      err = true;
    }
  }

  if (err) {
    if (skt_ != nullptr) {
      zmq_close(skt_);
      skt_ = nullptr;
    }
    return false;
  }
  return true;
}

bool SubWorker::RecvMsg(zmq_msg_t* recv_topic, zmq_msg_t* recv_content) {
  if (recv_topic == nullptr || recv_content == nullptr) {
    return false;
  }
  zmq_msg_init(recv_topic);

  int ret = zmq_msg_recv(recv_topic, skt_, 0);
  if (ret < 0) {
    HLOG_ERROR << "zmq_msg_recv topic error";
    zmq_msg_close(recv_topic);
    return false;
  }

  int more = zmq_msg_more(recv_topic);
  if (more == 0) {
    HLOG_ERROR << "No more message after received topic";
    zmq_msg_close(recv_topic);
    return false;
  }

  zmq_msg_init(recv_content);
  ret = zmq_msg_recv(recv_content, skt_, 0);
  if (ret < 0) {
    HLOG_ERROR << "zmq_msg_recv content error";
    zmq_msg_close(recv_topic);
    zmq_msg_close(recv_content);
    return false;
  }
  more = zmq_msg_more(recv_content);
  if (more != 0) {
    HLOG_ERROR << "More messages after already received one message";
    zmq_msg_close(recv_topic);
    zmq_msg_close(recv_content);
    return false;
  }

  return true;
}

}  // namespace mp
}  // namespace hozon
