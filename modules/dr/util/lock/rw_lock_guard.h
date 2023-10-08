/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： rw_lock_guard.h
 *   author     ： lilanxing
 *   date       ： 2023.09
 ******************************************************************************/

#ifndef COMMON_BASE_RW_LOCK_GUARD_H_
#define COMMON_BASE_RW_LOCK_GUARD_H_

#include <unistd.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <thread>

namespace hozon {
namespace common {
namespace base {

template <typename RWLock>
class ReadLockGuard {
 public:
  explicit ReadLockGuard(RWLock& lock) : rw_lock_(lock) { rw_lock_.ReadLock(); }

  ~ReadLockGuard() { rw_lock_.ReadUnlock(); }

  ReadLockGuard(const ReadLockGuard& other) = delete;
  ReadLockGuard& operator=(const ReadLockGuard& other) = delete;

 private:
  RWLock& rw_lock_;
};

template <typename RWLock>
class WriteLockGuard {
 public:
  explicit WriteLockGuard(RWLock& lock) : rw_lock_(lock) {
    rw_lock_.WriteLock();
  }

  ~WriteLockGuard() { rw_lock_.WriteUnlock(); }

  WriteLockGuard(const WriteLockGuard& other) = delete;
  WriteLockGuard& operator=(const WriteLockGuard& other) = delete;

 private:
  RWLock& rw_lock_;
};

}  // namespace base
}  // namespace common
}  // namespace hozon

#endif  // CYBER_BASE_RW_LOCK_GUARD_H_
