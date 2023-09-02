/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： context.h
 *   author     ： taoshaoyuan
 *   date       ： 2023.01
 ******************************************************************************/

#pragma once

#include <zmq/zmq.h>

namespace hozon {
namespace mp {

class Context {
 public:
  void* Get() { return ctx_; }

 private:
  void* ctx_ = nullptr;

  // declare singleton
 private:
  Context() { ctx_ = zmq_ctx_new(); }

  ~Context() {
    if (ctx_) {
      zmq_ctx_destroy(ctx_);
    }
    ctx_ = nullptr;
  }

  Context(const Context&);
  Context& operator=(const Context&);

 public:
  static Context& Instance() {
    static Context instance;
    return instance;
  }
};

}  // namespace mp
}  // namespace hozon