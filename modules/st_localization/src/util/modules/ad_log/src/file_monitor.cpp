/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Guo Zhichong <guozhichong@sensetime.com>
 */

#ifdef WITH_TDA_QNX
#include <sys/poll.h>  // epoll_create1()/epoll_ctl/epoll_wait()
#else
#include <sys/epoll.h>
#endif
#include <sys/inotify.h>  // inotify_init/inotify_add_watch/inotify_rm_watch
#include <sys/types.h>    // inotify_event
#include <unistd.h>       // read

#include <map>  // std::map

#include "epoll.hpp"
#include "file_monitor.hpp"

namespace senseAD {
namespace common {
namespace utils {

class FileMonitor::Impl {
 public:
    Impl();

    virtual ~Impl();

    bool Init();

    bool WatchFileObject(const std::string &path);

    bool UnwatchFileObject(const std::string &path);

    bool ProcessOnce(int32_t timeout_ms);

    std::vector<std::string> GetCreated() { return created_; }

    std::vector<std::string> GetModified() { return modified_; }

    std::vector<std::string> GetDeleted() { return deleted_; }

 private:
    bool HandleInotifyEvent(int32_t fd);
    int32_t fd_;
    std::map<std::string, int32_t> filepaths_;
    std::vector<std::string> created_;
    std::vector<std::string> modified_;
    std::vector<std::string> deleted_;
    Epoll epoll_;
};

FileMonitor::Impl::Impl() : fd_(inotify_init()) {}

FileMonitor::Impl::~Impl() {
    const auto &filepaths = filepaths_;
    for (const auto &kv : filepaths) {
        UnwatchFileObject(kv.first);
    }
    close(fd_);
}

bool FileMonitor::Impl::Init() {
    auto ret = epoll_.Watch(fd_);
    return ret;
}

bool FileMonitor::Impl::WatchFileObject(const std::string &filepath) {
    auto wd = inotify_add_watch(fd_, filepath.c_str(),
                                IN_MODIFY | IN_CREATE | IN_DELETE);
    auto added = filepaths_.insert(std::make_pair(filepath, wd));
    if (!added.second) {
        fprintf(stdout, "Already under watching %s\n", filepath.c_str());
    }
    return true;
}

bool FileMonitor::Impl::UnwatchFileObject(const std::string &filepath) {
    if (filepaths_.count(filepath) < 1) {
        return false;
    }
    auto it = filepaths_.find(filepath);
    inotify_rm_watch(fd_, it->second);
    filepaths_.erase(filepath);
    return true;
}

bool FileMonitor::Impl::ProcessOnce(int32_t timeout_ms) {
    created_.clear();
    modified_.clear();
    deleted_.clear();
    if (!epoll_.SpinOnce(timeout_ms)) {
        return false;
    }
    auto queue = epoll_.GetEventQueue();
    if (queue == nullptr) {
        return false;
    }
    auto iter = queue->rbegin();
    while (iter != queue->rend()) {
        if (iter->first & EpollEvent::READABLE) {
            const auto &fd = iter->second;
            return HandleInotifyEvent(fd);
        }
        ++iter;
    }
    epoll_.ClearQueue();
    return true;
}

bool FileMonitor::Impl::HandleInotifyEvent(int32_t fd) {
    constexpr std::size_t kEventSize = sizeof(struct inotify_event);
    char buffer[4096]
        __attribute__((aligned(__alignof__(struct inotify_event))));
    int32_t bytes = read(fd, buffer, sizeof(buffer));
    if (bytes <= 0) {
        return false;
    }
    char *cursor = nullptr;
    const struct inotify_event *event;
    for (cursor = buffer; cursor < buffer + bytes;
         cursor += kEventSize + event->len) {
        event = reinterpret_cast<struct inotify_event *>(cursor);
        if (event->mask & IN_ISDIR) {
            fprintf(stdout, "File ");
        } else {
            fprintf(stdout, "Directory ");
        }
        if (event->mask & IN_CREATE) {
            created_.push_back(event->name);
            fprintf(stdout, "%s created\n", event->name);
        } else if (event->mask & IN_MODIFY) {
            modified_.push_back(event->name);
            fprintf(stdout, "%s modified\n", event->name);
        } else if (event->mask & IN_DELETE) {
            deleted_.push_back(event->name);
            fprintf(stdout, "%s deleted\n", event->name);
        } else {
            fprintf(stdout, "%s other operations\n", event->name);
        }
    }
    return true;
}

FileMonitor::FileMonitor() : impl_(new FileMonitor::Impl()) {}

FileMonitor::~FileMonitor() {}

bool FileMonitor::Init() { return impl_->Init(); }

bool FileMonitor::WatchFileObject(const std::string &filepath) {
    return this->impl_->WatchFileObject(filepath);
}

bool FileMonitor::UnwatchFileObject(const std::string &filepath) {
    return this->impl_->UnwatchFileObject(filepath);
}

bool FileMonitor::ProcessOnce(int32_t timeout_ms) {
    return this->impl_->ProcessOnce(timeout_ms);
}

std::vector<std::string> FileMonitor::GetCreated() {
    return this->impl_->GetCreated();
}

std::vector<std::string> FileMonitor::GetModified() {
    return this->impl_->GetModified();
}

std::vector<std::string> FileMonitor::GetDeleted() {
    return this->impl_->GetDeleted();
}

}  // namespace utils
}  // namespace common
}  // namespace senseAD
