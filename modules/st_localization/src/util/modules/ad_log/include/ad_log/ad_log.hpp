
/*
 * Copyright (C) 2017 by SenseTime Group Limited. All rights reserved.
 * Sun Gefei <sungefei@sensetime.com>
 * Chen Kaige <chenkaige@sensetime.com>
 * Guo Zhichong <guozhichong@sensetime.com>
 */

#pragma once

#include <sys/time.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <chrono>

#include "ad_log/log_interface.hpp"

#ifndef JOIN_STRING
#define _JOIN_STRING(A, B) A##B
#define JOIN_STRING(A, B) _JOIN_STRING(A, B)
#endif

#define AD_LOG_EVERY_SEC(LOG_FUNC, SEC)                                        \
    static std::size_t JOIN_STRING(count, __LINE__) = 0;                       \
    static auto JOIN_STRING(last_time, __LINE__) =                             \
        std::chrono::high_resolution_clock::time_point(                        \
            std::chrono::seconds(0));                                          \
    auto JOIN_STRING(now, __LINE__) =                                          \
        std::chrono::high_resolution_clock::now();                             \
    auto JOIN_STRING(timeout, __LINE__) =                                      \
        (JOIN_STRING(now, __LINE__) >=                                         \
         JOIN_STRING(last_time, __LINE__) + std::chrono::seconds(SEC));        \
    auto JOIN_STRING(count_shadow, __LINE__) = ++JOIN_STRING(count, __LINE__); \
    if (JOIN_STRING(timeout, __LINE__)) {                                      \
        JOIN_STRING(last_time, __LINE__) =                                     \
            std::chrono::high_resolution_clock::now();                         \
        JOIN_STRING(count, __LINE__) = 0;                                      \
    }                                                                          \
    if (JOIN_STRING(timeout, __LINE__))                                        \
    LOG_FUNC << JOIN_STRING(count_shadow, __LINE__) << " "

#define AD_LOG_EVERY_N(LOG_FUNC, N)                      \
    static std::size_t JOIN_STRING(count, __LINE__) = 0; \
    ++JOIN_STRING(count, __LINE__);                      \
    if (JOIN_STRING(count, __LINE__) > (N)) {            \
        JOIN_STRING(count, __LINE__) -= (N);             \
    }                                                    \
    if (JOIN_STRING(count, __LINE__) == 1) LOG_FUNC

#define AD_LTRACE(tag) \
    if (false) std::cout
#define AD_LDEBUG(tag) \
    if (false) std::cout
#define ADAS_LDEBUG(tag) LOGS_DEBUG(tag)
#define AD_LINFO(tag) LOGS_INFO(tag)
#define AD_LERROR(tag) LOGS_ERROR(tag)
#define AD_LWARN(tag) LOGS_WARN(tag)
#define AD_LFATAL(tag) LOGS_CRITICAL(tag)
// with the prefix of AD_F  it will log into background log file
#define AD_FTRACE(tag) \
    if (false) std::cout
#define AD_FDEBUG(tag) \
    if (false) std::cout
#define AD_FINFO(tag) FLOGS_INFO(tag)
#define AD_FERROR(tag) FLOGS_ERROR(tag)
#define AD_FWARN(tag) FLOGS_WARN(tag)
#define AD_FFATAL(tag) FLOGS_CRITICAL(tag)

// Every N
#define AD_LTRACE_EVERY(tag, n) AD_LOG_EVERY_N(AD_LTRACE(tag), n)
#define AD_LDEBUG_EVERY(tag, n) AD_LOG_EVERY_N(AD_LDEBUG(tag), n)
#define AD_LINFO_EVERY(tag, n) AD_LOG_EVERY_N(AD_LINFO(tag), n)
#define AD_LERROR_EVERY(tag, n) AD_LOG_EVERY_N(AD_LERROR(tag), n)
#define AD_LWARN_EVERY(tag, n) AD_LOG_EVERY_N(AD_LWARN(tag), n)
#define AD_LFATAL_EVERY(tag, n) AD_LOG_EVERY_N(AD_LFATAL(tag), n)
#define AD_FTRACE_EVERY(tag, n) AD_LOG_EVERY_N(AD_FTRACE(tag), n)
#define AD_FDEBUG_EVERY(tag, n) AD_LOG_EVERY_N(AD_FDEBUG(tag), n)
#define AD_FINFO_EVERY(tag, n) AD_LOG_EVERY_N(AD_FINFO(tag), n)
#define AD_FERROR_EVERY(tag, n) AD_LOG_EVERY_N(AD_FERROR(tag), n)
#define AD_FWARN_EVERY(tag, n) AD_LOG_EVERY_N(AD_FWARN(tag), n)
#define AD_FFATAL_EVERY(tag, n) AD_LOG_EVERY_N(AD_FFATAL(tag), n)

#define AD_LTRACE_EVERY_SEC(tag, n) AD_LOG_EVERY_SEC(AD_LTRACE(tag), n)
#define AD_LDEBUG_EVERY_SEC(tag, n) AD_LOG_EVERY_SEC(AD_LDEBUG(tag), n)
#define AD_LINFO_EVERY_SEC(tag, n) AD_LOG_EVERY_SEC(AD_LINFO(tag), n)
#define AD_LERROR_EVERY_SEC(tag, n) AD_LOG_EVERY_SEC(AD_LERROR(tag), n)
#define AD_LWARN_EVERY_SEC(tag, n) AD_LOG_EVERY_SEC(AD_LWARN(tag), n)
#define AD_FTRACE_EVERY_SEC(tag, n) AD_LOG_EVERY_SEC(AD_FTRACE(tag), n)
#define AD_FDEBUG_EVERY_SEC(tag, n) AD_LOG_EVERY_SEC(AD_FDEBUG(tag), n)
#define AD_FINFO_EVERY_SEC(tag, n) AD_LOG_EVERY_SEC(AD_FINFO(tag), n)
#define AD_FERROR_EVERY_SEC(tag, n) AD_LOG_EVERY_SEC(AD_FERROR(tag), n)
#define AD_FWARN_EVERY_SEC(tag, n) AD_LOG_EVERY_SEC(AD_FWARN(tag), n)

#define START_FUNCTION(NAME)                                     \
    timeval M_PROCEDURE_##NAME##_START_TIME;                     \
    do {                                                         \
        LOGS_WARN(TIME) << "[" << #NAME << "] started.";         \
        gettimeofday(&M_PROCEDURE_##NAME##_START_TIME, nullptr); \
    } while (0)
#define FINISH_FUNCTION(NAME)                                              \
    do {                                                                   \
        LOGS_WARN(TIME) << "[" << #NAME << "] finished.";                  \
        timeval M_PROCEDURE_##NAME##_FINISH_TIME;                          \
        gettimeofday(&M_PROCEDURE_##NAME##_FINISH_TIME, nullptr);          \
        LOGS_WARN(TIME) << "[" << #NAME << "] totally time "               \
                        << (M_PROCEDURE_##NAME##_FINISH_TIME.tv_sec -      \
                            M_PROCEDURE_##NAME##_START_TIME.tv_sec) *      \
                                   1000.0 +                                \
                               (M_PROCEDURE_##NAME##_FINISH_TIME.tv_usec - \
                                M_PROCEDURE_##NAME##_START_TIME.tv_usec) / \
                                   1000.0                                  \
                        << " ms.";                                         \
    } while (0)

#define INIT_FRAME_TIMER(NAME)      \
    double M_TOTAL_TIME_##NAME = 0; \
    int M_TOTAL_COUNT_##NAME = 0;

#define START_FRAME_TIMER(NAME)                             \
    timeval M_FRAME_START_TIME_##NAME;                      \
    do {                                                    \
        LOGS_WARN(TIME) << "\"" << #NAME << "\": started."; \
        gettimeofday(&M_FRAME_START_TIME_##NAME, nullptr);  \
    } while (0)

#define FINISH_FRAME_TIMER(NAME, COUNT)                               \
    do {                                                              \
        LOGS_WARN(TIME) << "\"" << #NAME << "\": finished.";          \
        timeval M_FRAME_FINISH_TIME_##NAME;                           \
        gettimeofday(&M_FRAME_FINISH_TIME_##NAME, nullptr);           \
        double M_FRAME_TIME_COST_##NAME =                             \
            (M_FRAME_FINISH_TIME_##NAME.tv_sec -                      \
             M_FRAME_START_TIME_##NAME.tv_sec) *                      \
                1000.0 +                                              \
            (M_FRAME_FINISH_TIME_##NAME.tv_usec -                     \
             M_FRAME_START_TIME_##NAME.tv_usec) /                     \
                1000.0;                                               \
        M_TOTAL_TIME_##NAME += M_FRAME_TIME_COST_##NAME;              \
        M_TOTAL_COUNT_##NAME += COUNT;                                \
        LOGS_WARN(TIME) << "\"" << #NAME << "\": totally time "       \
                        << M_FRAME_TIME_COST_##NAME << " ms.";        \
        LOGS_WARN(TIME) << "\"" << #NAME << "\": average time "       \
                        << M_TOTAL_TIME_##NAME / M_TOTAL_COUNT_##NAME \
                        << " ms.";                                    \
    } while (0)

namespace senseAD {

class LoggingHelper {
 public:
    static void InitLogging(const std::string log_dir = "./",
                            const char* prog_name = "",
                            const std::string symbol_link_file_basename = "");

 private:
    static LoggingHelper* Instance();

    void Init(const std::string log_dir,
              const char* prog_name,
              const std::string symbol_link_file_basename);
};

}  // namespace senseAD
