/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： pub_worker.h
 *   author     ： taoshaoyuan
 *   date       ： 2023.01
 ******************************************************************************/

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {

class Publisher;

class PubWorker {
 public:
  PubWorker(void* zmq_ctx, const std::vector<std::string>& addrs)
      : addrs_(addrs), ctx_(zmq_ctx) {}

  ~PubWorker() { Term(); }

  PubWorker(const PubWorker&) = delete;
  PubWorker& operator=(const PubWorker&) = delete;

  int Init();

  void Term();

  std::vector<std::string> Addrs();

  void AddData(const std::string& topic, const void* data, size_t size);

  static std::shared_ptr<PubWorker> Create(
      void* zmq_ctx, const std::vector<std::string>& addrs) {
    return std::make_shared<PubWorker>(zmq_ctx, addrs);
  }

 private:
  void Loop();
  void OpenSocket();

 private:
  std::vector<std::string> addrs_;
  void* ctx_ = nullptr;
  void* skt_ = nullptr;
  std::atomic_bool running_ = {false};
  std::atomic_bool loop_prepare_over_ = {false};
  std::atomic_bool loop_prepare_error_ = {false};

  struct PubData {
    std::string topic;
    std::vector<char> data;
  };

  std::deque<PubData> data_;
  std::mutex mtx_;
  std::condition_variable cv_;
  std::unique_ptr<std::thread> pub_thread_ = nullptr;

  const size_t kMaxQueueSize = 1000;
  const std::string kTermTopic = "TOPIC_TERM";
};

}  // namespace mp
}  // namespace hozon
