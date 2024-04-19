/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Yue Dayu <yuedayu@sensetime.com>
 */

#pragma once

#include <fcntl.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <unistd.h>
#include <string>
#include "ad_log/ad_log.hpp"
#include "ad_common/data_type/base.hpp"

namespace senseAD {
namespace ad_utils {

class SharedMemIPC {
 public:
    // max pool size is 4 Mega bytes
    const static int32_t MAX_DATA_SIZE = 2048 * 2000;

    explicit SharedMemIPC(const std::string& name,
                          const int32_t& size = MAX_DATA_SIZE,
                          const bool& multiple_set = false);
    ~SharedMemIPC();

    // wait for buffer time_ms milli-seconds
    bool set(uint8_t* data,
             const int32_t& size,
             const uint64_t& cap_time_ns,
             const int32_t& time_ms = 1000);
    int32_t get(uint8_t* data, const int32_t& size, uint64_t* cap_time_ns);
    int32_t timed_get(uint8_t* data,
                      const int32_t& size,
                      uint64_t* cap_time_ns,
                      const int32_t& time_ms = 1000);

    int32_t size();
    bool is_ready() { return is_ready_; }

    void clean_sys();

 private:
    struct MemHeader {
        bool is_mutex_init;
        pthread_mutex_t mutex;
        int32_t cur_client;
        int32_t memory_size;
        int32_t cap_len;
        uint64_t cap_timestamp_ns;
    };

    std::string name_;

    sem_t* sem_;
    sem_t* sem_empty_;
    int32_t size_;

    uint8_t* data_;
    MemHeader* header_;

    bool multiple_set_ = false;
    bool is_ready_ = false;
};

}  // namespace ad_utils
}  // namespace senseAD
