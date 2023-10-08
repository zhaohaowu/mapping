/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： atomic_rw_lock.h 
 *   author     ： zhuxiaolin
 *   date       ： 2023.10
 ******************************************************************************/

#ifndef COMMON_BASE_ATOMIC_RW_LOCK_H_
#define COMMON_BASE_ATOMIC_RW_LOCK_H_

#include <unistd.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <thread>

#include "modules/dr/util/lock/rw_lock_guard.h"

namespace hozon {
namespace common {
namespace base {

class AtomicRWLock {
  friend class ReadLockGuard<AtomicRWLock>;
  friend class WriteLockGuard<AtomicRWLock>;

 public:
  static const int32_t RW_LOCK_FREE = 0;
  static const int32_t WRITE_EXCLUSIVE = -1;
  static const uint32_t MAX_RETRY_TIMES = 5;
  AtomicRWLock() = default;
  ~AtomicRWLock() = default;

  explicit AtomicRWLock(bool write_first) : write_first_(write_first) {}

  AtomicRWLock(const AtomicRWLock&) = delete;
  AtomicRWLock& operator=(const AtomicRWLock&) = delete;
  AtomicRWLock(AtomicRWLock&&) = delete;
  AtomicRWLock operator=(const AtomicRWLock&&) = delete;

 private:
  // all these function only can used by ReadLockGuard/WriteLockGuard;
  void ReadLock();
  void WriteLock();

  void ReadUnlock();
  void WriteUnlock();
  std::atomic<uint32_t> write_lock_wait_num_ = {0};
  std::atomic<int32_t> lock_num_ = {0};
  bool write_first_ = true;
};

inline void AtomicRWLock::ReadLock() {  // NOLINT
  uint32_t retry_times = 0;
  int32_t lock_num = lock_num_.load();
  if (write_first_) {
    while (true) {
      while (lock_num < RW_LOCK_FREE || write_lock_wait_num_.load() > 0) {
        if (++retry_times == MAX_RETRY_TIMES) {
          // saving cpu
          std::this_thread::yield();
          retry_times = 0;
        }
        lock_num = lock_num_.load();
      }
      if (lock_num_.compare_exchange_weak(lock_num, lock_num + 1,
                                          std::memory_order_acq_rel,
                                          std::memory_order_relaxed)) {
        break;
      }
    }
  } else {
    while (true) {
      while (lock_num < RW_LOCK_FREE) {
        if (++retry_times == MAX_RETRY_TIMES) {
          // saving cpu
          std::this_thread::yield();
          retry_times = 0;
        }
        lock_num = lock_num_.load();
      }
      if (lock_num_.compare_exchange_weak(lock_num, lock_num + 1,
                                          std::memory_order_acq_rel,
                                          std::memory_order_relaxed)) {
        break;
      }
    }
  }
}

inline void AtomicRWLock::WriteLock() {
  int32_t rw_lock_free = RW_LOCK_FREE;
  uint32_t retry_times = 0;
  write_lock_wait_num_.fetch_add(1);
  while (!lock_num_.compare_exchange_weak(rw_lock_free, WRITE_EXCLUSIVE,
                                          std::memory_order_acq_rel,
                                          std::memory_order_relaxed)) {
    // rw_lock_free will change after CAS fail, so init agin
    rw_lock_free = RW_LOCK_FREE;
    if (++retry_times == MAX_RETRY_TIMES) {
      // saving cpu
      std::this_thread::yield();
      retry_times = 0;
    }
  }
  write_lock_wait_num_.fetch_sub(1);
}

inline void AtomicRWLock::ReadUnlock() {
  lock_num_.fetch_sub(1);
}

inline void AtomicRWLock::WriteUnlock() {
  lock_num_.fetch_add(1);
}

}  // namespace base
}  // namespace common
}  // namespace hozon

#endif  // CYBER_BASE_ATOMIC_RW_LOCK_H_
