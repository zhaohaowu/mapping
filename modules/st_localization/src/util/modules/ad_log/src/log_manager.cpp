/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * LiangYu<liangyu@sensetime.com>
 */

#include <sys/types.h>  // pid_t
#include <stdio.h>      // fprint
#include <unistd.h>     // getpid
#include <fcntl.h>      // open
#include <sstream>

#ifdef WITH_TDA_QNX
#include <chrono>
#endif

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/stdout_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/sinks/null_sink.h"
#include "spdlog/async.h"

#include "ad_log/log_manager.hpp"
#include "file_monitor.hpp"

namespace senseAD {
using typename ::senseAD::common::utils::FileMonitor;

#ifdef __cplusplus
extern "C" {
#endif

extern char *__progname;
static char *ProcessName() { return __progname; }

#ifdef __cplusplus
}
#endif

template <typename CallableT>
class ScopeExit {
 public:
    explicit ScopeExit(CallableT &&callable) : callable_(std::move(callable)) {}
    ~ScopeExit() {
        if (is_active_) {
            callable_();
        }
    }

    void Dismiss() { is_active_ = false; }

 private:
    CallableT callable_;
    bool is_active_ = true;
};

// Function for type deduction
template <typename CallableT>
ScopeExit<CallableT> MakeScopeExit(CallableT &&callable) {
    return ScopeExit<CallableT>(std::forward<CallableT>(callable));
}

#define _JOIN_STRING(A, B) A##B
#define JOIN_STRING(A, B) _JOIN_STRING(A, B)
#define SCOPE_EXIT(code)                     \
    auto JOIN_STRING(scope_exit, __LINE__) = \
        senseAD::MakeScopeExit([&] { code })

template <typename T>
static inline T GetEnv(const char *name, T default_value) {
    char *p = getenv(name);
    if (p) {
        std::string str(p);
        std::istringstream iss(str);
        T ret;
        iss >> ret;
        return ret;
    } else {
        return default_value;
    }
}

template <typename T,
          typename U = typename std::enable_if<
              std::is_const<T>::value,
              typename std::remove_const<T>::type>::type>
static inline U GetEnv(const char *name, U default_value) {
    char *p = getenv(name);
    if (p) {
        std::string str(p);
        std::istringstream iss(str);
        U ret;
        iss >> ret;
        return ret;
    } else {
        return default_value;
    }
}

static bool DirectoryExists(const char *path) {
    if (path == NULL) return false;
    DIR *pdir;
    bool exists = false;
    pdir = opendir(path);
    if (pdir != NULL) {
        exists = true;
        (void)closedir(pdir);
    }
    return exists;
}

static std::string TimePidString() {
    struct ::tm tm_time;

#ifdef WITH_TDA_QNX
    auto time_now = std::chrono::system_clock::now();
    time_t stamp = std::chrono::system_clock::to_time_t(time_now);
#else
    auto time_now = std::chrono::high_resolution_clock::now();
    time_t stamp = std::chrono::high_resolution_clock::to_time_t(time_now);
#endif
    localtime_r(&stamp, &tm_time);

    // The logfile's filename will have the date/time & pid in it
    std::ostringstream time_pid_stream;
    time_pid_stream.fill('0');
    time_pid_stream << '.' << 1900 + tm_time.tm_year << std::setw(2)
                    << 1 + tm_time.tm_mon << std::setw(2) << tm_time.tm_mday
                    << '-' << std::setw(2) << tm_time.tm_hour << std::setw(2)
                    << tm_time.tm_min << std::setw(2) << tm_time.tm_sec << '.'
                    << getpid();
    return time_pid_stream.str();
}

StartLogTime &StartLogTime::Instance() {
    static StartLogTime startLogTime;
    return startLogTime;
}
StartLogTime::StartLogTime() { this->SetStartTime(TimePidString()); }

LogManager &LogManager::Instance() {
    static LogManager log;
    return log;
}

LogManager::~LogManager() { this->Flush(); }

void LogManager::Flush() {
    if (nullptr != logger_) {
        this->logger_->flush();
    }
}

LogManager::LogManager() {
    // log level [trace, debug, info, warning, error, critical, off]
    config_.severity = GetEnv("LOG_level", std::string("error"));
    /**
     * If specified, logfiles are written into this directory instead of the
     *     default logging directorydefault logging
     */
    config_.path = GetEnv("LOG_path", std::string("/tmp"));
    // whether using async log
    config_.async = GetEnv("LOG_async", false);
    /**
     * set log messages to temrminal close/std::cout/sdt::cerr
     *     0 close; 1 std::cout; 3 std::cerr
     */
    config_.tostderr = GetEnv("LOG_tostderr", 0);
    // set number of async log's thread
    config_.async_thread_nums = GetEnv("LOG_async_thread_nums", 1);
    // set number of async log's buffer length
    config_.async_queue_length = GetEnv("LOG_async_queue_length", 8192);
    // set async overflow policy 0 block; 1 overrun_oldest
    config_.async_overflow_policy = GetEnv("LOG_async_overflow_policy", 0);
    // whether using rotating log
    config_.rotating_file = GetEnv("LOG_rotating_file", false);
    // set the maximum rotating log file size
    config_.max_size = GetEnv("LOG_rotating_file_max_size", 8192);
    // set the maximum rotating log number of file
    config_.max_files = GetEnv("LOG_rotating_file_max_files", 3);
    // whether using daily log
    config_.daily_file = GetEnv("LOG_daily_file", false);
    // whether send messages to simple_file
    config_.simple_file = GetEnv("LOG_simple_file", true);
    // set daily log time
    config_.hour = GetEnv("LOG_daily_file_hour", 0);
    // set daily log time
    config_.minute = GetEnv("LOG_daily_file_minute", false);
    // set log pattern
    config_.pattern =
        GetEnv("LOG_pattern", std::string("%m%d %T.%e [%L]%$ %v"));
    // whether print log with color
    config_.color = GetEnv("LOG_color", true);

    if (level_map_.count(config_.severity) < 1) {
        config_.severity = "error";
    }
    // spdlog level(reverse): 6 5 4 3 2 1 0 = off critical error warn info debug
    // trace
    level_ = static_cast<int>(level_map_[config_.severity]);

    std::vector<spdlog::sink_ptr> sinks;
    bool path_ok = DirectoryExists(config_.path.c_str());

    if (config_.color) {
        if (1 == config_.tostderr) {
            sinks.push_back(
                std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
        } else if (2 == config_.tostderr) {
            sinks.push_back(
                std::make_shared<spdlog::sinks::stderr_color_sink_mt>());
        } else if (0 == config_.tostderr) {
            auto sk = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
            sk->set_level(
                static_cast<spdlog::level::level_enum>(level_map_["error"]));
            sinks.push_back(sk);
        }
    } else {
        if (1 == config_.tostderr) {
            sinks.push_back(std::make_shared<spdlog::sinks::stdout_sink_mt>());
        } else if (2 == config_.tostderr) {
            sinks.push_back(std::make_shared<spdlog::sinks::stderr_sink_mt>());
        } else if (0 == config_.tostderr) {
            auto sk = std::make_shared<spdlog::sinks::stdout_sink_mt>();
            sk->set_level(
                static_cast<spdlog::level::level_enum>(level_map_["error"]));
            sinks.push_back(sk);
        }
    }
    // const std::string time_pid_str = StartLogTime::Instance().GetStartTime();
    if (config_.path.back() != '/') {
        config_.path += '/';
    }
    const std::string suffix(".txt");
    std::string log_basename = GetEnv("LOG_basename", std::string());
    const std::string basename(log_basename.empty() ? ProcessName()
                                                    : log_basename);

    // if (config_.simple_file && path_ok && !basename.empty()) {
    //     const std::string file =
    //         config_.path + basename + time_pid_str + suffix;

    //     const std::string syslink_name = config_.path + basename + suffix;
    //     unlink(syslink_name.c_str());
    //     sinks.push_back(
    //         std::make_shared<spdlog::sinks::basic_file_sink_mt>(file, true));
    //     if (symlink(file.c_str(), syslink_name.c_str()) != 0) {
    //         // silently ignore failures
    //     }
    // }

    if (config_.rotating_file && path_ok && !basename.empty()) {
        const std::string sub_folder = config_.path + "rotating_log/";
        if (DirectoryExists(sub_folder.c_str()) ||
            0 == mkdir(sub_folder.c_str(), 0777)) {
            std::string file = sub_folder + basename + suffix;
            sinks.push_back(
                std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
                    file, config_.max_size, config_.max_files));
        }
    }
    if (config_.daily_file && path_ok) {
        const std::string sub_folder = config_.path + "daily_log/";
        if (DirectoryExists(sub_folder.c_str()) ||
            0 == mkdir(sub_folder.c_str(), 0777)) {
            std::string file = sub_folder + basename + suffix;
            sinks.push_back(std::make_shared<spdlog::sinks::daily_file_sink_mt>(
                file, config_.hour, config_.minute));
        }
    }
    if (sinks.empty()) {
        sinks.push_back(std::make_shared<spdlog::sinks::null_sink_st>());
    }
    for (auto &sink : sinks) {
        sink->set_pattern(config_.pattern);
    }
    if (config_.async) {
        auto &registry_inst = spdlog::details::registry::instance();
        // create global thread pool if not already exists..
        auto &mutex = registry_inst.tp_mutex();
        std::lock_guard<std::recursive_mutex> tp_lock(mutex);
        auto tp = registry_inst.get_tp();
        if (tp == nullptr) {
            tp = std::make_shared<spdlog::details::thread_pool>(
                config_.async_queue_length, config_.async_thread_nums);
            registry_inst.set_tp(tp);
        }
        spdlog::async_overflow_policy policy;
        policy = config_.async_overflow_policy == 0
                     ? spdlog::async_overflow_policy::block
                     : spdlog::async_overflow_policy::overrun_oldest;

        logger_ = SetupLogger<spdlog::async_logger>("_", sinks.begin(),
                                                    sinks.end(), tp, policy);
    } else {
        logger_ = SetupLogger<spdlog::logger>("_", sinks.begin(), sinks.end());
        logger_->set_level(static_cast<spdlog::level::level_enum>(
            level_map_[config_.severity]));
    }
}

FileLogManager &FileLogManager::Instance() {
    static FileLogManager log;
    return log;
}

FileLogManager::~FileLogManager() { this->Flush(); }

void FileLogManager::Flush() {
    if (nullptr != logger_) {
        this->logger_->flush();
    }
}

FileLogManager::FileLogManager() {
    // todo(zhangconghai): This is using hard code
    // set log pattern
    config_.pattern =
        GetEnv("LOG_pattern", std::string("%m%d %T.%e [%L]%$ %v"));
    /**
     * If specified, logfiles are written into this directory instead of the
     *     default logging directorydefault logging
     */
    config_.path = GetEnv("LOG_path", std::string("/tmp"));
    // whether using async log
    config_.async = GetEnv("LOG_async", false);
    // log level [trace, debug, info, warning, error, critical, off]
    config_.severity = GetEnv("FLOG_level", std::string("info"));
    /**
     * set log messages to temrminal close/std::cout/sdt::cerr
     *     0 close; 1 std::cout; 3 std::cerr
     */
    config_.tostderr = GetEnv("LOG_tostderr", 0);
    // set number of async log's thread
    config_.async_thread_nums = GetEnv("LOG_async_thread_nums", 1);
    // set number of async log's buffer length
    config_.async_queue_length = GetEnv("LOG_async_queue_length", 8192);
    // set async overflow policy 0 block; 1 overrun_oldest
    config_.async_overflow_policy = GetEnv("LOG_async_overflow_policy", 0);
    // whether using rotating log
    config_.rotating_file = GetEnv("LOG_rotating_file", false);
    // set the maximum rotating log file size
    config_.max_size = GetEnv("LOG_rotating_file_max_size", 8192);
    // set the maximum rotating log number of file
    config_.max_files = GetEnv("LOG_rotating_file_max_files", 3);
    // whether using daily log
    config_.daily_file = GetEnv("LOG_daily_file", false);
    // whether send messages to simple_file
    config_.simple_file = GetEnv("LOG_simple_file", true);
    // set daily log time
    config_.hour = GetEnv("LOG_daily_file_hour", 0);
    // set daily log time
    config_.minute = GetEnv("LOG_daily_file_minute", false);
    // whether print log with color
    config_.color = GetEnv("LOG_color", true);

    if (level_map_.count(config_.severity) < 1) {
        config_.severity = "error";
    }
    // spdlog level(reverse): 6 5 4 3 2 1 0 = off critical error warn info debug
    // trace
    level_ = static_cast<int>(level_map_[config_.severity]);

    std::vector<spdlog::sink_ptr> sinks;
    bool path_ok = DirectoryExists(config_.path.c_str());

    const std::string time_pid_str = StartLogTime::Instance().GetStartTime();

    if (config_.path.back() != '/') {
        config_.path += '/';
    }
    const std::string suffix(".txt");
    std::string log_basename = GetEnv("LOG_basename", std::string());
    const std::string basename(log_basename.empty() ? ProcessName()
                                                    : log_basename);

    // will only generate single file simple log or rotate log
    if (config_.simple_file && path_ok && !basename.empty()) {
        const std::string sub_folder = config_.path + "background_log/";
        if (DirectoryExists(sub_folder.c_str()) ||
            0 == mkdir(sub_folder.c_str(), 0777)) {
            const std::string log_basename(basename + "_background");
            std::string file =
                sub_folder + log_basename + time_pid_str + suffix;
            sinks.push_back(std::make_shared<spdlog::sinks::basic_file_sink_mt>(
                file, true));

            const std::string syslink_name =
                config_.path + log_basename + suffix;
            unlink(syslink_name.c_str());
            if (symlink(file.c_str(), syslink_name.c_str()) != 0) {
                // silently ignore failures
            }
        }
    } else if (config_.rotating_file && path_ok && !basename.empty()) {
        const std::string sub_folder = config_.path + "rotating_log/";
        if (DirectoryExists(sub_folder.c_str()) ||
            0 == mkdir(sub_folder.c_str(), 0777)) {
            const std::string log_basename(log_basename + "_rotating");
            std::string file =
                sub_folder + log_basename + time_pid_str + suffix;
            sinks.push_back(
                std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
                    file, config_.max_size, config_.max_files));
            const std::string syslink_name =
                config_.path + log_basename + suffix;
            unlink(syslink_name.c_str());
            if (symlink(file.c_str(), syslink_name.c_str()) != 0) {
                // silently ignore failures
            }
        }
    }

    if (sinks.empty()) {
        for (int i = 0; i < 10; i++) {
            std::cerr << "*** ERROR BACKGROUND FILE LOG INIT FAIL ***"
                      << std::endl;
        }
        sinks.push_back(std::make_shared<spdlog::sinks::null_sink_st>());
    }
    for (auto &sink : sinks) {
        sink->set_pattern(config_.pattern);
    }

    if (config_.async) {
        auto &registry_inst = spdlog::details::registry::instance();
        // create global thread pool if not already exists..
        auto &mutex = registry_inst.tp_mutex();
        std::lock_guard<std::recursive_mutex> tp_lock(mutex);
        auto tp = registry_inst.get_tp();
        if (tp == nullptr) {
            tp = std::make_shared<spdlog::details::thread_pool>(
                config_.async_queue_length, config_.async_thread_nums);
            registry_inst.set_tp(tp);
        }
        spdlog::async_overflow_policy policy;
        policy = config_.async_overflow_policy == 0
                     ? spdlog::async_overflow_policy::block
                     : spdlog::async_overflow_policy::overrun_oldest;

        logger_ = SetupLogger<spdlog::async_logger>("_", sinks.begin(),
                                                    sinks.end(), tp, policy);
        logger_->set_level(static_cast<spdlog::level::level_enum>(
            level_map_[config_.severity]));
    } else {
        logger_ = SetupLogger<spdlog::logger>("_", sinks.begin(), sinks.end());
        logger_->set_level(static_cast<spdlog::level::level_enum>(
            level_map_[config_.severity]));
    }
}

}  // namespace senseAD
