/*
 * Copyright (C) 2021 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */
#pragma once

#include <pthread.h>

#include <atomic>
#include <chrono>              // NOLINT
#include <condition_variable>  // NOLINT
#include <limits>
#include <memory>
#include <mutex>   // NOLINT
#include <thread>  // NOLINT

namespace senseAD {
namespace localization {

class ThreadBase {
 public:
  ThreadBase() = default;
  virtual ~ThreadBase() = default;

  virtual void SetThreadName() {
    pthread_setname_np(thread_->native_handle(), "LC_threadBase");
  }

  // @brief: request finish
  void RequestFinish() { finish_req_ = true; }

  // @brief: is finished check
  bool IsFinished() {
    if (thread_ == nullptr) is_finished_ = true;
    return is_finished_;
  }

  // @brief: request pause
  void RequestPause() { pause_req_ = true; }

  // @brief: reset pause related flag
  void ReleasePause() {
    pause_req_ = false;
    is_paused_ = false;
    release_pause_cond_.notify_all();
  }

  // @biref: is paused check
  bool IsPaused() { return is_paused_; }

  // @brief: waiting for paused (if has paused, go on)
  void WaitPaused(int wait_ns = std::numeric_limits<int>::max()) {
    std::unique_lock<std::mutex> lg(pause_mutex_);
    pause_cond_.wait_for(lg, std::chrono::milliseconds(wait_ns));
  }

  // @brief: waiting for release paused (if has released paused, go on)
  void WaitReleasePaused(int wait_ns = std::numeric_limits<int>::max()) {
    std::unique_lock<std::mutex> lg(pause_mutex_);
    release_pause_cond_.wait_for(lg, std::chrono::milliseconds(wait_ns));
  }

 protected:
  // @brief: check finish request
  bool CheckFinishRequest() { return finish_req_; }

  // @brief: set finish
  void SetFinish() { is_finished_ = true; }

  // @brief: check pause request
  bool CheckPauseRequest() { return pause_req_; }

  // @brief: set pause
  void SetPause() {
    is_paused_ = true;
    pause_cond_.notify_all();
  }

 protected:
  std::mutex pause_mutex_;                      // pause mutex
  std::atomic<bool> pause_req_{false};          // pause request flag
  std::atomic<bool> is_paused_{false};          // is pause flag
  std::condition_variable pause_cond_;          // pause condition variable
  std::condition_variable release_pause_cond_;  // release pause condition var

  std::atomic<bool> finish_req_{false};   // finish request flag
  std::atomic<bool> is_finished_{false};  // is finish flag

  std::unique_ptr<std::thread> thread_;  // thread instance
};

}  // namespace localization
}  // namespace senseAD
