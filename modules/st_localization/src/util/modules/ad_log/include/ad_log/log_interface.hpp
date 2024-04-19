/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * LiangYu <liangyu@sensetime.com>
 */

#pragma once

#include "ad_log/log_manager.hpp"
#include "ad_log/log_stream.hpp"

#define __FILENAME__ (strrchr("/" __FILE__, '/') + 1)

#define CHECK_SEVERITY(severity) \
    senseAD::LogManager::Instance().ShouldLogging(LOG_LEVEL_##severity)

#define LOGSTREAM_SEVERITY_IF(severity, condition)                      \
    static_cast<void>(0), (!condition)                                  \
                              ? (void)0                                 \
                              : senseAD::LogStreamHelper() &            \
                                    senseAD::LoggerStream##severity<>() \
                                        << __FILENAME__ << ":" << __LINE__

#define LOGSTREAM_SEVERITY(severity)                   \
    LOGSTREAM_SEVERITY_IF(severity,                    \
                          (CHECK_SEVERITY(severity) || \
                           SYSLOG_LEVEL_##severity <= SYSLOG_LEVEL_INFO))

#define LOGPRINT_SEVERITY(severity, fmt, ...)                            \
    senseAD::LogManager::Instance().severity("{}:{} " fmt, __FILENAME__, \
                                             __LINE__, __VA_ARGS__)

#ifndef SENSEAD_LOG_ACTIVE_LEVEL
#define SENSEAD_LOG_ACTIVE_LEVEL LOG_LEVEL_TRACE
#endif

#if SENSEAD_LOG_ACTIVE_LEVEL <= LOG_LEVEL_TRACE
#define LOGP_TRACE(fmt, ...) LOGPRINT_SEVERITY(TRACE, fmt, __VA_ARGS__)
#define LOGS_TRACE(tag) LOGSTREAM_SEVERITY(TRACE) << " [" << (#tag) << "] "
#else
#define LOGP_TRACE(fmt, ...) (void)0
#define LOGS_TRACE(tag) senseAD::LoggerStreamNULL()
#endif

#if SENSEAD_LOG_ACTIVE_LEVEL <= LOG_LEVEL_DEBUG
#define LOGP_DEBUG(fmt, ...) LOGPRINT_SEVERITY(DEBUG, fmt, __VA_ARGS__)
#define LOGS_DEBUG(tag) LOGSTREAM_SEVERITY(DEBUG) << " [" << (#tag) << "] "
#else
#define LOGP_DEBUG(fmt, ...) (void)0
#define LOGS_DEBUG(tag) senseAD::LoggerStreamNULL()
#endif

#if SENSEAD_LOG_ACTIVE_LEVEL <= LOG_LEVEL_INFO
#define LOGP_INFO(fmt, ...) LOGPRINT_SEVERITY(INFO, fmt, __VA_ARGS__)
#define LOGS_INFO(tag) LOGSTREAM_SEVERITY(INFO) << " [" << (#tag) << "] "
#else
#define LOGP_INFO(fmt, ...) (void)0
#define LOGS_INFO(tag) senseAD::LoggerStreamNULL()
#endif

#if SENSEAD_LOG_ACTIVE_LEVEL <= LOG_LEVEL_WARN
#define LOGP_WARN(fmt, ...) LOGPRINT_SEVERITY(WARN, fmt, __VA_ARGS__)
#define LOGS_WARN(tag) LOGSTREAM_SEVERITY(WARN) << " [" << (#tag) << "] "
#else
#define LOGP_WARN(fmt, ...) (void)0
#define LOGS_WARN(tag) senseAD::LoggerStreamNULL()
#endif

#if SENSEAD_LOG_ACTIVE_LEVEL <= LOG_LEVEL_ERROR
#define LOGP_ERROR(fmt, ...) LOGPRINT_SEVERITY(ERROR, fmt, __VA_ARGS__)
#define LOGS_ERROR(tag) LOGSTREAM_SEVERITY(ERROR) << " [" << (#tag) << "] "
#else
#define LOGP_ERROR(fmt, ...) (void)0
#define LOGS_ERROR(tag) senseAD::LoggerStreamNULL()
#endif

#if SENSEAD_LOG_ACTIVE_LEVEL <= LOG_LEVEL_CRITICAL
#define LOGS_CRITICAL(tag) \
    LOGSTREAM_SEVERITY(CRITICAL) << " [" << (#tag) << "] "
#define LOGP_CRITICAL(fmt, ...) LOGPRINT_SEVERITY(CRITICAL, fmt, __VA_ARGS__)
#else
#define LOGP_CRITICAL(fmt, ...) (void)0
#define LOGS_CRITICAL(tag) senseAD::LoggerStreamNULL()
#endif

#define LOGPRINT_SEVERITY_IF(severity, cond, fmt, ...) \
    if (cond) {                                        \
        LOGP_##severity(fmt, __VA_ARGS__);             \
    }

#define LOGP_TRACE_IF(cond, fmt, ...) \
    LOGPRINT_SEVERITY_IF(TRACE, cond, fmt, __VA_ARGS__)
#define LOGP_DEBUG_IF(cond, fmt, ...) \
    LOGPRINT_SEVERITY_IF(DEBUG, cond, fmt, __VA_ARGS__)
#define LOGP_INFO_IF(cond, fmt, ...) \
    LOGPRINT_SEVERITY_IF(INFO, cond, fmt, __VA_ARGS__)
#define LOGP_WARN_IF(cond, fmt, ...) \
    LOGPRINT_SEVERITY_IF(WARN, cond, fmt, __VA_ARGS__)
#define LOGP_ERROR_IF(cond, fmt, ...) \
    LOGPRINT_SEVERITY_IF(ERROR, cond, fmt, __VA_ARGS__)
#define LOGP_CRITICAL_IF(cond, fmt, ...) \
    LOGPRINT_SEVERITY_IF(CRITICAL, cond, fmt, __VA_ARGS__)

/* FILE LOG MANAGER
 * Remove default file:line output (which can be config by spdlog's config
   if file:line was needed someday.
 * The log level is the same as the above logger
 * */
#define CHECK_FILE_SEVERITY(severity) \
    senseAD::FileLogManager::Instance().ShouldLogging(LOG_LEVEL_##severity)

#define LOGFILESTREAM_SEVERITY_IF(severity, condition)       \
    static_cast<void>(0), (!condition)                       \
                              ? (void)0                      \
                              : senseAD::LogStreamHelper() & \
                                    senseAD::FileLoggerStream##severity<>()

#define LOGFILESTREAM_SEVERITY(severity) \
    LOGFILESTREAM_SEVERITY_IF(severity, CHECK_FILE_SEVERITY(severity))

#define FILELOGPRINT_SEVERITY(severity, fmt, ...) \
    senseAD::LogManager::Instance().severity(fmt, __VA_ARGS__)

#ifndef SENSEAD_FILE_LOG_ACTIVE_LEVEL
#define SENSEAD_FILE_LOG_ACTIVE_LEVEL LOG_LEVEL_TRACE
#endif

#if SENSEAD_LOG_ACTIVE_LEVEL <= LOG_LEVEL_TRACE
#define FLOGP_TRACE(fmt, ...) FILELOGPRINT_SEVERITY(TRACE, fmt, __VA_ARGS__)
#define FLOGS_TRACE(tag) LOGFILESTREAM_SEVERITY(TRACE) << "[" << (#tag) << "] "
#else
#define FLOGP_TRACE(fmt, ...) (void)0
#define FLOGS_TRACE(tag) senseAD::LoggerStreamNULL()
#endif

#if SENSEAD_LOG_ACTIVE_LEVEL <= LOG_LEVEL_DEBUG
#define FLOGP_DEBUG(fmt, ...) FILELOGPRINT_SEVERITY(DEBUG, fmt, __VA_ARGS__)
#define FLOGS_DEBUG(tag) LOGFILESTREAM_SEVERITY(DEBUG) << "[" << (#tag) << "] "
#else
#define FLOGP_DEBUG(fmt, ...) (void)0
#define FLOGS_DEBUG(tag) senseAD::LoggerStreamNULL()
#endif

#if SENSEAD_LOG_ACTIVE_LEVEL <= LOG_LEVEL_INFO
#define FLOGP_INFO(fmt, ...) FILELOGPRINT_SEVERITY(INFO, fmt, __VA_ARGS__)
#define FLOGS_INFO(tag) LOGFILESTREAM_SEVERITY(INFO) << "[" << (#tag) << "] "
#else
#define FLOGP_INFO(fmt, ...) (void)0
#define FLOGS_INFO(tag) senseAD::LoggerStreamNULL()
#endif

#if SENSEAD_LOG_ACTIVE_LEVEL <= LOG_LEVEL_WARN
#define FLOGP_WARN(fmt, ...) FILELOGPRINT_SEVERITY(WARN, fmt, __VA_ARGS__)
#define FLOGS_WARN(tag) LOGFILESTREAM_SEVERITY(WARN) << "[" << (#tag) << "] "
#else
#define FLOGP_WARN(fmt, ...) (void)0
#define FLOGS_WARN(tag) senseAD::LoggerStreamNULL()
#endif

#if SENSEAD_LOG_ACTIVE_LEVEL <= LOG_LEVEL_ERROR
#define FLOGP_ERROR(fmt, ...) FILELOGPRINT_SEVERITY(ERROR, fmt, __VA_ARGS__)
#define FLOGS_ERROR(tag) LOGFILESTREAM_SEVERITY(ERROR) << "[" << (#tag) << "] "
#else
#define FLOGP_ERROR(fmt, ...) (void)0
#define FLOGS_ERROR(tag) senseAD::LoggerStreamNULL()
#endif

#if SENSEAD_LOG_ACTIVE_LEVEL <= LOG_LEVEL_CRITICAL
#define FLOGS_CRITICAL(tag) \
    LOGFILESTREAM_SEVERITY(CRITICAL) << "[" << (#tag) << "] "
#define FLOGP_CRITICAL(fmt, ...) \
    FILELOGPRINT_SEVERITY(CRITICAL, fmt, __VA_ARGS__)
#else
#define FLOGP_CRITICAL(fmt, ...) (void)0
#define FLOGS_CRITICAL(tag) senseAD::LoggerStreamNULL()
#endif
