/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Guo Zhichong <guozhichong@sensetime.com>
 */
#pragma once

#include <vector>
#include <string>
#include <memory>  // std::unique_ptr

namespace senseAD {
namespace common {
namespace utils {

/*
 * @brief Monitor changes of files(regular files or directories)
 *
 */
class FileMonitor {
 public:
    FileMonitor();

    virtual ~FileMonitor();

    /*
     * @brief Initialize file monitor
     * @return true if succeed else false
     */
    bool Init();

    /*
     * @brief Add a file object into monitoring list
     * @path specify the file to be monitored
     * @return true if succeed else false
     */
    bool WatchFileObject(const std::string &path);

    /*
     * @brief Remove monitoring on the specific file
     * @path specify the file then cancel the monitoring on this file
     * @return true if succeed else false
     */
    bool UnwatchFileObject(const std::string &path);

    /*
     * @brief Monitor the changes within 'timeout_ms'
     * @timeout_ms specify timeout in milliseconds
     * @return true if succeed else false
     */
    bool ProcessOnce(int32_t timeout_ms);

    /*
     * @brief Get the created files
     * @return true if succeed else false
     */
    std::vector<std::string> GetCreated();

    /*
     * @brief Get the modified files
     * @return true if succeed else false
     */
    std::vector<std::string> GetModified();

    /*
     * @brief Get the deleted files
     * @return true if succeed else false
     */
    std::vector<std::string> GetDeleted();

 private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace utils
}  // namespace common
}  // namespace senseAD
