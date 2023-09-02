/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： linker.cc
 *   author     ： taoshaoyuan
 *   date       ： 2023.01
 ******************************************************************************/

#include "util/nodelink/core/linker.h"

#include <zmq/zmq.h>

#include "util/nodelink/core/node.h"
#include "util/temp_log.h"

namespace hozon {
namespace mp {

using namespace std::chrono_literals;

// Should execute before Start()
int Linker::LinkNode(const std::shared_ptr<Node>& node) {
  auto sub_addrs = node->SubAddrs();
  auto pub_addrs = node->PubAddrs();

  for (const auto& ad : sub_addrs) {
    if (LinkBackend(ad) < 0) return -1;
  }

  for (const auto& ad : pub_addrs) {
    if (LinkFrontend(ad) < 0) return -1;
  }

  return 0;
}

// Should execute before Start()
void Linker::UnLinkNode(const std::shared_ptr<Node>& node) {
  auto sub_addrs = node->SubAddrs();
  auto pub_addrs = node->PubAddrs();

  for (const auto& ad : sub_addrs) {
    UnLinkBackend(ad);
  }

  for (const auto& ad : pub_addrs) {
    UnLinkFrontend(ad);
  }
}

// Should execute before Start()
int Linker::LinkFrontend(const std::string& addr) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (bind_addrs_.find(addr) != bind_addrs_.end()) {
    HLOG_ERROR
        << addr
        << " already in bind address, cannot bind and connect on same address";
    return -1;
  }

  if (conn_addrs_.find(addr) == conn_addrs_.end()) {
    conn_addrs_.insert(addr);
  }

  return 0;
}

// Should execute before Start()
void Linker::UnLinkFrontend(const std::string& addr) {
  std::lock_guard<std::mutex> lock(mtx_);
  auto it = conn_addrs_.find(addr);
  if (it != conn_addrs_.end()) {
    conn_addrs_.erase(it);
  }
}

// Should execute before Start()
int Linker::LinkBackend(const std::string& addr) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (conn_addrs_.find(addr) != conn_addrs_.end()) {
    HLOG_ERROR
        << addr
        << " already in connect address, cannot bind and connect on same "
           "address";
    return -1;
  }

  if (bind_addrs_.find(addr) == bind_addrs_.end()) {
    bind_addrs_.insert(addr);
  }

  return 0;
}

// Should execute before Start()
void Linker::UnLinkBackend(const std::string& addr) {
  std::lock_guard<std::mutex> lock(mtx_);
  auto it = bind_addrs_.find(addr);
  if (it != bind_addrs_.end()) {
    bind_addrs_.erase(it);
  }
}

int Linker::Start() {
  if (!ctx_) {
    HLOG_ERROR << "Linker start error, invalid context";
    return -1;
  }

  if (bind_addrs_.empty()) {
    HLOG_INFO << "no bind address (i.e. no nodes pub), no need to link";
    return 0;
  }

  if (conn_addrs_.empty()) {
    HLOG_INFO << "no connect address (i.e. no nodes sub), no need to link";
    return 0;
  }

  for (const auto& ad : bind_addrs_) {
    if (ad.empty()) {
      HLOG_ERROR << "Linker start error, empty bind address";
      return -1;
    }
  }
  for (const auto& ad : conn_addrs_) {
    if (ad.empty()) {
      HLOG_ERROR << "Linker start error, empty connect address";
      return -1;
    }
  }

  auto this_ptr = static_cast<const void*>(this);
  std::stringstream ss;
  ss << this_ptr;
  std::string this_str = ss.str();

  term_addr_ = "inproc://" + this_str;
  HLOG_INFO << "Linker term addr " << term_addr_;

  term_cmd_skt_ = zmq_socket(ctx_, ZMQ_PUB);
  int ret = zmq_bind(term_cmd_skt_, term_addr_.c_str());
  if (ret < 0 && term_cmd_skt_) {
    HLOG_ERROR << "zmq_bind term addr " << term_addr_ << " error";
    zmq_close(term_cmd_skt_);
    term_cmd_skt_ = nullptr;
    return -1;
  }

  auto cnt = bind_addrs_.size() + conn_addrs_.size();

  proxy_thread_ = std::make_unique<std::thread>(&Linker::Loop, this);

  int sleep_cnt = 50;
  auto every_sleep_ms = 100ms;
  auto max_sleep_ms = sleep_cnt * every_sleep_ms;
  while (sleep_cnt > 0) {
    if (loop_prepare_over_.load()) {
      break;
    }
    std::this_thread::sleep_for(every_sleep_ms);
    sleep_cnt--;
  }

  if (!loop_prepare_over_.load()) {
    HLOG_ERROR << "Loop prepare timeout, exceed " << max_sleep_ms.count()
               << "ms";
    return -1;
  }

  if (loop_prepare_error_.load()) {
    HLOG_ERROR << "Loop prepare error";
    return -1;
  }

  //! TBD：这里是个trick，目的是让Linker::Start()阻塞一会儿，等link的socket都
  //! 结束mute state，这样Linker::Start()后续的socket pub就不会丢消息了。
  //! 后面看有没有更稳妥的方式。
  auto t = cnt * 100ms;
  HLOG_INFO << "sleep " << t.count() << "ms for waiting sockets end mute state";
  std::this_thread::sleep_for(t);

  return 0;
}

void Linker::Term() {
  if (!term_cmd_skt_) {
    HLOG_INFO << "invalid term_cmd_skt_, maybe already termed";
    return;
  }

  int ret = zmq_send(term_cmd_skt_, "TERMINATE", 9, 0);
  if (ret < 0) {
    HLOG_ERROR << "zmq_send TERMINATE error";
    return;
  }

  if (proxy_thread_ && proxy_thread_->joinable()) {
    proxy_thread_->join();
    proxy_thread_ = nullptr;
  }

  if (term_cmd_skt_) {
    zmq_close(term_cmd_skt_);
    term_cmd_skt_ = nullptr;
  }
}

void Linker::Loop() {
  std::set<std::string> conn_addrs, bind_addrs;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    conn_addrs = conn_addrs_;
    bind_addrs = bind_addrs_;
  }

  int ret = 0;
  frontend_skt_ = zmq_socket(ctx_, ZMQ_XSUB);
  for (const auto& ad : conn_addrs) {
    ret = zmq_connect(frontend_skt_, ad.c_str());
    HLOG_INFO << "zmq_connect " << ad;
    if (ret < 0) {
      HLOG_ERROR << "zmq_connect " << ad << " error";
      loop_prepare_error_.store(true);
    }
  }

  backend_skt_ = zmq_socket(ctx_, ZMQ_XPUB);
  for (const auto& ad : bind_addrs) {
    ret = zmq_bind(backend_skt_, ad.c_str());
    if (ret < 0) {
      HLOG_ERROR << "zmq_bind " << ad << " error";
      loop_prepare_error_.store(true);
    }
  }

  term_skt_ = zmq_socket(ctx_, ZMQ_SUB);
  ret = zmq_connect(term_skt_, term_addr_.c_str());
  if (ret < 0) {
    HLOG_ERROR << "zmq_connect " << term_addr_ << " error";
    loop_prepare_error_.store(true);
  }
  ret = zmq_setsockopt(term_skt_, ZMQ_SUBSCRIBE, "", 0);
  if (ret < 0) {
    HLOG_ERROR << "zmq_setsockopt error";
    loop_prepare_error_.store(true);
  }

  auto close_skt = [&] {
    if (frontend_skt_) {
      zmq_close(frontend_skt_);
      frontend_skt_ = nullptr;
    }
    if (backend_skt_) {
      zmq_close(backend_skt_);
      backend_skt_ = nullptr;
    }
    if (term_skt_) {
      zmq_close(term_skt_);
      term_skt_ = nullptr;
    }
  };

  if (loop_prepare_error_.load()) {
    close_skt();
    loop_prepare_over_.store(true);
    HLOG_ERROR << "Linker Loop start failed";
    return;
  }

  loop_prepare_over_.store(true);

  // block, wait term msg
  zmq_proxy_steerable(frontend_skt_, backend_skt_, nullptr, term_skt_);

  // zmq_proxy_steerable return, close socket
  close_skt();
}

}  // namespace mp
}  // namespace hozon