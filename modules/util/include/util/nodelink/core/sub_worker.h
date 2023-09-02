/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： sub_worker.h
 *   author     ： taoshaoyuan
 *   date       ： 2023.01
 ******************************************************************************/

#pragma once

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace hozon {
namespace mp {

using SubCallback = std::function<void(void*, size_t)>;
using SubAllCallback = std::function<void(const std::string&, void*, size_t)>;

class SubWorker {
 public:
  SubWorker(void* zmq_ctx, const std::vector<std::string>& addrs)
      : addrs_(addrs), ctx_(zmq_ctx) {
    GenId();
  }

  ~SubWorker() { Term(); }

  SubWorker(const SubWorker&) = delete;
  SubWorker& operator=(const SubWorker&) = delete;

  int Init();
  void Start();

  void Term();

  std::vector<std::string> Addrs();

  // Should execute before Init()
  void Reg(const std::string& topic, SubCallback callback);

  // Should execute before Init()
  void UnReg(const std::string& topic);

  // Subscribe all topics into one callback
  void RegAll(SubAllCallback callback);
  void UnRegAll();

  static std::shared_ptr<SubWorker> Create(
      void* zmq_ctx, const std::vector<std::string>& addrs) {
    return std::make_shared<SubWorker>(zmq_ctx, addrs);
  }

 private:
  void Loop();
  void GenId();
  std::string Id();
  int SendTerm();

 private:
  std::vector<std::string> addrs_;
  void* ctx_ = nullptr;
  void* skt_ = nullptr;
  void* term_skt_ = nullptr;
  std::atomic_bool running_ = {false};
  std::atomic_bool started_ = {false};
  std::unique_ptr<std::thread> sub_thread_ = nullptr;

  std::mutex mtx_;
  std::map<std::string, SubCallback> all_cbk_;
  SubAllCallback all_topics_cbk_ = nullptr;

  std::string id_;
  std::string term_addr_;
  const std::string kTermTopic = "TOPIC_TERM";
  std::string term_content_;
};

}  // namespace mp
}  // namespace hozon