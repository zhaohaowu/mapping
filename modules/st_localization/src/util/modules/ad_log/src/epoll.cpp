/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Guo Zhichong <guozhichong@sensetime.com>
 */

#ifndef WITH_TDA_QNX
#include <sys/epoll.h>  // epoll_create1()/epoll_ctl/epoll_wait()
#else
#include <sys/poll.h>
#include <array>
#endif
#include <stdio.h>

#include <unistd.h>  // read
#include <atomic>    // std::atomic<T>
#include <cstring>   // std::memset

#include "epoll.hpp"

namespace senseAD {
namespace common {
namespace utils {

class Epoll::Impl {
 public:
    Impl();

    virtual ~Impl();

    bool Watch(int32_t fd);

    bool Unwatch(int32_t fd);

    EventQueueSharedPtr GetEventQueue();

    void ClearQueue();

    bool SpinOnce(int32_t timeout_ms);

 private:
    static constexpr std::size_t kMaxEpollEvent = 10;
#ifndef WITH_TDA_QNX
    int32_t handle_;
    std::array<struct epoll_event, kMaxEpollEvent> events_;
#else
    std::array<struct pollfd, kMaxEpollEvent> watched_fd_;
#endif
    EventQueueSharedPtr event_queue_;
};

#ifdef WITH_TDA_QNX
Epoll::Impl::Impl() : event_queue_(new EventQueue()) {
    for (int32_t i = 0; i < kMaxEpollEvent; ++i) {
        watched_fd_[i].fd = -1;
        watched_fd_[i].events = POLLIN | POLLPRI | POLLOUT;
        watched_fd_[i].revents = 0;
    }
}
#else
Epoll::Impl::Impl()
    : handle_(epoll_create1(0)), event_queue_(new EventQueue()) {}
#endif

Epoll::Impl::~Impl() {}

bool Epoll::Impl::Watch(int32_t fd) {
#ifndef WITH_TDA_QNX
    if (handle_ < 0) {
        return false;
    }
    struct epoll_event event;
    memset(&event, 0, sizeof(epoll_event));
    event.data.fd = fd;
    event.events = EPOLLET | EPOLLIN;
    int32_t ret = epoll_ctl(handle_, EPOLL_CTL_ADD, fd, &event);

    if (ret != 0) {
        perror("Failed to add epoll control.");
        return false;
    }
    return ret == 0;
#else
    if (fd <= 0) {
        perror("Failed to add epoll control.");
        return false;
    }

    for (int32_t i = 0; i < kMaxEpollEvent; ++i) {
        if (watched_fd_[i].fd < 0) {
            watched_fd_[i].fd = fd;
            return true;
        } else {
            if (watched_fd_[i].fd == fd) {
                return true;
            }
        }
    }

    perror("Failed to add epoll control.");
    return false;
#endif
}

bool Epoll::Impl::Unwatch(int32_t fd) {
#ifndef WITH_TDA_QNX
    if (handle_ < 0) {
        return false;
    }
    int32_t ret = epoll_ctl(handle_, EPOLL_CTL_DEL, fd, NULL);
    if (ret != 0) {
        perror("Failed to add epoll control.");
        return false;
    }
    return ret == 0;
#else
    for (int32_t i = 0; i < kMaxEpollEvent; i++) {
        if (watched_fd_[i].fd > 0 && watched_fd_[i].fd == fd) {
            watched_fd_[i].fd = -1;
            return true;
        }
    }
    perror("Failed to add epoll control.");
    return false;

#endif
}

bool Epoll::Impl::SpinOnce(int32_t timeout_ms) {
#ifndef WITH_TDA_QNX
    if (handle_ < 0) {
        return false;
    }
    auto fds = epoll_wait(handle_, events_.begin(), kMaxEpollEvent, timeout_ms);
    for (int32_t i = 0; i < fds; ++i) {
        const auto &event = events_[i];
        const auto &fd = events_[i].data.fd;
        if (event.events & EPOLLHUP) {
            this->event_queue_->push_back(
                std::make_pair(EpollEvent::REMOVABLE, fd));
        } else if (event.events & EPOLLIN) {
            this->event_queue_->push_back(
                std::make_pair(EpollEvent::READABLE, fd));
        } else if (event.events & EPOLLOUT) {
            this->event_queue_->push_back(
                std::make_pair(EpollEvent::WRITABLE, fd));
        }
    }
#else
    poll(watched_fd_.begin(), kMaxEpollEvent, timeout_ms);

    for (int32_t i = 0; i < kMaxEpollEvent; i++) {
        if (watched_fd_[i].revents & POLLHUP) {
            this->event_queue_->push_back(
                std::make_pair(EpollEvent::REMOVABLE, watched_fd_[i].fd));
        } else if (watched_fd_[i].revents & (POLLIN | POLLPRI)) {
            this->event_queue_->push_back(
                std::make_pair(EpollEvent::READABLE, watched_fd_[i].fd));
        } else if (watched_fd_[i].revents & POLLOUT) {
            this->event_queue_->push_back(
                std::make_pair(EpollEvent::WRITABLE, watched_fd_[i].fd));
        } else {
        }
    }
#endif
    auto size = event_queue_->size();
    while (size-- > kMaxEpollEvent) {
        event_queue_->pop_front();
    }
    return true;
}

EventQueueSharedPtr Epoll::Impl::GetEventQueue() { return event_queue_; }

void Epoll::Impl::ClearQueue() { event_queue_->clear(); }

Epoll::Epoll() : impl_(new Epoll::Impl()) {}

Epoll::~Epoll() {}

bool Epoll::Watch(int32_t fd) { return this->impl_->Watch(fd); }

bool Epoll::Unwatch(int32_t fd) { return this->impl_->Unwatch(fd); }

bool Epoll::SpinOnce(int32_t timeout_ms) {
    return this->impl_->SpinOnce(timeout_ms);
}

EventQueueSharedPtr Epoll::GetEventQueue() { return impl_->GetEventQueue(); }

void Epoll::ClearQueue() { impl_->ClearQueue(); }

}  // namespace utils
}  // namespace common
}  // namespace senseAD
