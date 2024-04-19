/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Yue Dayu <yuedayu@sensetime.com>
 */

#include "ad_utils/shared_memory_ipc.hpp"

#include <cstring>

namespace senseAD {
namespace ad_utils {

SharedMemIPC::SharedMemIPC(const std::string& name,
                           const int32_t& size,
                           const bool& multiple_set) {
    is_ready_ = false;
    size_ = size;
    if (size_ > MAX_DATA_SIZE) {
        AD_LERROR(SHARED_MEMORY_IPC) << "size_ over MAX_DATA_SIZE(" << size_
                                     << " : " << MAX_DATA_SIZE << ")";
        size_ = MAX_DATA_SIZE;
    }
    name_ = name;
    multiple_set_ = multiple_set;

    int fd = shm_open(name.c_str(), O_RDWR | O_CREAT, 0666);
    if (fd < 0) {
        return;
    }
    sem_ = sem_open((name + "_sem").c_str(), O_CREAT, 0666, 0);
    if (!multiple_set_) {
        sem_empty_ = sem_open((name + "_sem_empty").c_str(), O_CREAT, 0666, 0);
        if (sem_empty_ == SEM_FAILED) {
            AD_LERROR(SHARED_MEMORY_IPC) << "Create empty semaphore failed";
            close(fd);
            return;
        }
    }
    if (sem_ == SEM_FAILED) {
        close(fd);
        return;
    }

    int ret = ftruncate(fd, sizeof(MemHeader) + size_);
    if (-1 == ret) {
        close(fd);
        return;
    }

    auto* memPtr = reinterpret_cast<uint8_t*>(
        mmap(nullptr, sizeof(MemHeader) + size_, PROT_READ | PROT_WRITE,
             MAP_SHARED, fd, 0));
    close(fd);

    header_ = reinterpret_cast<MemHeader*>(memPtr);
    data_ = memPtr + sizeof(MemHeader);

    if (!header_->is_mutex_init) {
        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
        pthread_mutex_init(&header_->mutex, &attr);
        header_->is_mutex_init = true;
    }

    pthread_mutex_lock(&header_->mutex);

    if (!multiple_set_) {
        int v;
        sem_getvalue(sem_empty_, &v);
        if (v == 0) {
            sem_post(sem_empty_);
        }
    }

    if (header_->cur_client != 0) {
        if (size_ != header_->memory_size) {
            AD_LERROR(SHARED_MEMORY_IPC)
                << "Shared Memory Size Error: " << size_ << " : "
                << header_->memory_size;
        }
    }
    header_->cur_client++;
    header_->memory_size = size_;
    pthread_mutex_unlock(&header_->mutex);

    is_ready_ = true;
}

SharedMemIPC::~SharedMemIPC() {
    sem_close(sem_);
    if (!multiple_set_) {
        sem_close(sem_empty_);
    }
    header_->cur_client--;
    clean_sys();
}

void SharedMemIPC::clean_sys() {
    if (header_->cur_client == 0) {
        sem_unlink((name_ + "_sem").c_str());
        sem_unlink((name_ + "_sem_empty").c_str());
        shm_unlink(name_.c_str());
    }
}

bool SharedMemIPC::set(uint8_t* data,
                       const int32_t& size,
                       const uint64_t& cap_time_ns,
                       const int32_t& time_ms) {
    if (!is_ready_) {
        return false;
    }
    if (size > size_) {
        return false;
    }
    timespec ts;
    if (clock_gettime(CLOCK_REALTIME, &ts) == -1) {
        AD_LERROR(SHARED_MEMORY_IPC) << "clock_gettime";
        return false;
    }
    if (!multiple_set_) {
        uint64_t t_add = uint64_t(ts.tv_nsec) + uint64_t(time_ms) * 1000000;
        ts.tv_sec += (t_add / 1000000000);
        ts.tv_nsec = (t_add % 1000000000);
        if (sem_timedwait(sem_empty_, &ts) == -1) {
            AD_LINFO(SHARED_MEMORY_IPC) << "timeout";
            return false;
        }
    }

    pthread_mutex_lock(&header_->mutex);
    memcpy(data_, data, size); /* Flawfinder: ignore */
    header_->cap_len = size;
    header_->cap_timestamp_ns = cap_time_ns;
    int v;
    sem_getvalue(sem_, &v);
    if (v == 0) {
        sem_post(sem_);
    } else {
        AD_LWARN(SHARED_MEMORY_IPC) << "multiple set";
    }
    pthread_mutex_unlock(&header_->mutex);
    return true;
}

int32_t SharedMemIPC::timed_get(uint8_t* data,
                                const int32_t& size,
                                uint64_t* cap_time_ns,
                                const int32_t& time_ms) {
    if (!is_ready_) {
        AD_LERROR(SHARED_MEMORY_IPC) << "Not ready";
        return -1;
    }
    timespec ts;
    if (clock_gettime(CLOCK_REALTIME, &ts) == -1) {
        AD_LERROR(SHARED_MEMORY_IPC) << "clock_gettime";
        return -1;
    }
    uint64_t t_add = uint64_t(ts.tv_nsec) + uint64_t(time_ms) * 1000000;
    ts.tv_sec += (t_add / 1000000000);
    ts.tv_nsec = (t_add % 1000000000);
    if (sem_timedwait(sem_, &ts) == -1) {
        AD_LTRACE(SHARED_MEMORY_IPC) << "timeout";
        return -1;
    }
    pthread_mutex_lock(&header_->mutex);
    int32_t cap_len = header_->cap_len;
    if (size < cap_len) {
        AD_LERROR(SHARED_MEMORY_IPC)
            << "invalid size: " << size << ", should be " << cap_len;
        pthread_mutex_unlock(&header_->mutex);
        return -1;
    }
    memcpy(data, data_, cap_len); /* Flawfinder: ignore */
    if (cap_time_ns != nullptr) {
        *cap_time_ns = header_->cap_timestamp_ns;
    }
    if (!multiple_set_) {
        int v;
        sem_getvalue(sem_empty_, &v);
        if (v == 0) {
            sem_post(sem_empty_);
        }
    }
    pthread_mutex_unlock(&header_->mutex);
    return cap_len;
}

int32_t SharedMemIPC::get(uint8_t* data,
                          const int32_t& size,
                          uint64_t* cap_time_ns) {
    if (!is_ready_) {
        return -1;
    }
    sem_wait(sem_);
    pthread_mutex_lock(&header_->mutex);
    int32_t cap_len = header_->cap_len;
    if (size < cap_len) {
        AD_LERROR(SHARED_MEMORY_IPC)
            << "invalid size: " << size << ", should be " << cap_len;
        pthread_mutex_unlock(&header_->mutex);
        return -1;
    }
    memcpy(data, data_, cap_len); /* Flawfinder: ignore */
    if (cap_time_ns != nullptr) {
        *cap_time_ns = header_->cap_timestamp_ns;
    }
    if (!multiple_set_) {
        int v;
        sem_getvalue(sem_empty_, &v);
        if (v == 0) {
            sem_post(sem_empty_);
        }
    }
    pthread_mutex_unlock(&header_->mutex);
    return cap_len;
}

int32_t SharedMemIPC::size() { return header_->memory_size; }

}  // namespace ad_utils
}  // namespace senseAD
