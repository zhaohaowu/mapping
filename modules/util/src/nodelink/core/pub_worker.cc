/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： pub_worker.cc
 *   author     ： taoshaoyuan
 *   date       ： 2023.01
 ******************************************************************************/

#include "util/nodelink/core/pub_worker.h"
#include <zmq/zmq.h>

#include <algorithm>

#include "util/mapping_log.h"
#include "util/nodelink/core/publisher.h"

namespace hozon {
namespace mp {

using namespace std::chrono_literals;  // NOLINT

int PubWorker::Init() {
  if (running_.load()) {
    HLOG_ERROR << "PubWorker already running";
    return -1;
  }
  if ((ctx_ == nullptr) || addrs_.empty()) {
    return -1;
  }
  if (std::any_of(addrs_.begin(), addrs_.end(),
                  [](const auto& addr) { return addr.empty(); })) {
    return -1;
  }

  running_.store(true);
  pub_thread_ = std::make_unique<std::thread>(&PubWorker::Loop, this);

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

  return 0;
}

void PubWorker::Term() {
  running_.store(false);
  AddData(kTermTopic, nullptr, 0);
  if (pub_thread_ && pub_thread_->joinable()) {
    pub_thread_->join();
    pub_thread_ = nullptr;
  }
  data_.clear();
  loop_prepare_over_.store(false);
  loop_prepare_error_.store(false);
}

std::vector<std::string> PubWorker::Addrs() { return addrs_; }

void PubWorker::AddData(const std::string& topic, const void* data,
                        size_t size) {
  PubData pub_data;
  pub_data.topic = topic;
  pub_data.data.resize(size);
  if ((data != nullptr) && size > 0) {
    memcpy(pub_data.data.data(), data, size);
  }

  std::unique_lock<std::mutex> lock(mtx_);
  while (data_.size() >= kMaxQueueSize) {
    HLOG_WARN << "PubWorker data queue exceed max size " << kMaxQueueSize
              << " now pop " << data_.front().topic;
    data_.pop_front();
  }
  data_.push_back(pub_data);
  cv_.notify_all();
}

void PubWorker::Loop() {
  OpenSocket();
  while (running_.load()) {
    std::unique_lock<std::mutex> lock(mtx_);
    cv_.wait(lock);
    while (!data_.empty()) {
      auto data = data_.front();
      data_.pop_front();

      if (data.topic == kTermTopic) {
        HLOG_INFO << "get term topic " << kTermTopic;
        break;
      }

      int ret =
          zmq_send(skt_, data.topic.c_str(), data.topic.length(), ZMQ_SNDMORE);
      if (ret < 0) {
        HLOG_ERROR << "zmq_send topic " << data.topic << " error";
        continue;
      }
      ret = zmq_send(skt_, data.data.data(), data.data.size(), 0);
      if (ret < 0) {
        HLOG_ERROR << "zmq_send data error";
        continue;
      }
    }
  }

  if (skt_ != nullptr) {
    zmq_close(skt_);
    skt_ = nullptr;
  }
}

void PubWorker::OpenSocket() {
  skt_ = zmq_socket(ctx_, ZMQ_PUB);
  for (const auto& addr : addrs_) {
    //! 注意：zmq_bind()后socket处于mute
    //! state，直到有连接建立，连接建立后即处于ready state； 处于mute
    //! state的socket，如果使用其pub消息的话消息会丢。即如果data_队列非空，
    //! 但此时还没有连接建立，那zmq_send()的data_数据都会丢掉。
    //! 参考：http://api.zeromq.org/4-0:zmq-bind
    int ret = zmq_bind(skt_, addr.c_str());
    if (ret < 0) {
      HLOG_ERROR << "zmq_bind " << addr << " error";
      loop_prepare_error_.store(true);
    }
  }
  if (loop_prepare_error_.load()) {
    if (skt_ != nullptr) {
      zmq_close(skt_);
      skt_ = nullptr;
    }
    loop_prepare_over_.store(true);
    return;
  }

  loop_prepare_over_.store(true);
}

}  // namespace mp
}  // namespace hozon
