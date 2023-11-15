/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： linker.h
 *   author     ： taoshaoyuan
 *   date       ： 2023.01
 ******************************************************************************/

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>

namespace hozon {
namespace mp {

class Node;

class Linker {
 public:
  explicit Linker(void* zmq_ctx) : ctx_(zmq_ctx) {}

  ~Linker() { Term(); }

  Linker(const Linker&) = delete;
  Linker& operator=(const Linker&) = delete;

  // Should execute before Start()
  int LinkNode(const std::shared_ptr<Node>& node);

  // Should execute before Start()
  void UnLinkNode(const std::shared_ptr<Node>& node);

  // Should execute before Start()
  int LinkFrontend(const std::string& addr);

  // Should execute before Start()
  void UnLinkFrontend(const std::string& addr);

  // Should execute before Start()
  int LinkBackend(const std::string& addr);

  // Should execute before Start()
  void UnLinkBackend(const std::string& addr);

  int Start();

  void Term();

  static std::shared_ptr<Linker> Create(void* zmq_ctx) {
    return std::make_shared<Linker>(zmq_ctx);
  }

 private:
  void Loop();
  bool NeedLink();
  bool ValidAddrs();
  void OpenSockets();
  void CloseSockets();

 private:
  std::mutex mtx_;
  std::set<std::string> bind_addrs_;
  std::set<std::string> conn_addrs_;

  std::unique_ptr<std::thread> proxy_thread_ = nullptr;

  void* ctx_ = nullptr;
  void* frontend_skt_ = nullptr;
  void* backend_skt_ = nullptr;
  void* term_skt_ = nullptr;
  void* term_cmd_skt_ = nullptr;
  std::string term_addr_;

  std::atomic_bool loop_prepare_over_ = {false};
  std::atomic_bool loop_prepare_error_ = {false};
};

using LinkerPtr = std::shared_ptr<Linker>;

}  // namespace mp
}  // namespace hozon
