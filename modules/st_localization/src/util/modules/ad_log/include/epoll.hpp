/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Guo Zhichong <guozhichong@sensetime.com>
 */
#pragma once

#include <deque>    // std::deque<T>
#include <memory>   // std::unique_ptr
#include <utility>  // std::pair

namespace senseAD {
namespace common {
namespace utils {

enum EpollEvent : uint8_t {
    READABLE = (1U << 0),
    WRITABLE = (1U << 1),
    REMOVABLE = (1U << 2)
};

typedef std::deque<std::pair<EpollEvent, int32_t>> EventQueue;
typedef std::shared_ptr<EventQueue> EventQueueSharedPtr;

/*
 * @brief Wrapper of epoll
 *
 */
class Epoll {
 public:
    Epoll();

    virtual ~Epoll();

    /*
     * @brief Add a file descriptor into monitoring list
     * @fd specify the file descriptor be monitored
     * @return true if succeed else false
     */
    bool Watch(int32_t fd);

    /*
     * @brief Remove monitoring on the specific file
     * @fd specify the file descriptor then cancel the monitoring on this file
     * @return true if succeed else false
     */
    bool Unwatch(int32_t fd);

    /*
     * @brief Get the event queue
     * @return event queue
     */
    EventQueueSharedPtr GetEventQueue();

    /*
     * @brief Clear the event queue
     */
    void ClearQueue();

    /*
     * @brief Wait for maximum 'timeout_ms' and fill the queue
     * @timeout_ms specify the maximum processing time in milliseconds
     * @return true if succeed else false
     */
    bool SpinOnce(int32_t timeout_ms);

 private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace utils
}  // namespace common
}  // namespace senseAD
