/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * LiangYu <liangyu@sensetime.com>
 */

#pragma once

#include <syslog.h>
#include <string>
#include <sstream>

#include "ad_log/log_manager.hpp"

namespace senseAD {

class LogStreamBuf : public std::streambuf {
 public:
    LogStreamBuf(char* buf, uint32_t len) { setp(buf, buf + len - 1); }
    virtual int_type overflow(int_type ch) { return ch; }
    size_t pcount() const { return pptr() - pbase(); }
    char* pbase() const { return std::streambuf::pbase(); }
};

template <uint32_t LEN = 2048>
class LoggerStreamBase : public std::ostream {
 public:
    LoggerStreamBase() : std::ostream(NULL), stringbuf_(buffer_, LEN) {
        rdbuf(&stringbuf_);
    }
    LoggerStreamBase(const LoggerStreamBase&) = delete;
    LoggerStreamBase& operator=(const LoggerStreamBase&) = delete;
    ~LoggerStreamBase() {}

 protected:
    void Flush() {}
    char* str() {
        const_cast<char*>(buffer_)[stringbuf_.pcount()] = '\0';
        return reinterpret_cast<char*>(&buffer_);
    }

 private:
    LogStreamBuf stringbuf_;
    char buffer_[LEN] = {0};
};

class LoggerStreamNULL : public LoggerStreamBase<1> {};

template <typename T>
inline LoggerStreamNULL& operator<<(LoggerStreamNULL& stream, const T& t) {
    return stream;
}

#define CREATE_STREAM_LOG(severity)                             \
    template <typename Write = LogManager>                      \
    class LoggerStream##severity : public LoggerStreamBase<> {  \
     public:                                                    \
        ~LoggerStream##severity() { Flush(); }                  \
                                                                \
     protected:                                                 \
        inline void Flush() {                                   \
            const auto& msg = str();                            \
            if (SYSLOG_LEVEL_##severity <= SYSLOG_LEVEL_INFO) { \
                syslog(SYSLOG_LEVEL_##severity, "%s\n", msg);   \
            }                                                   \
                                                                \
            if (senseAD::LogManager::Instance().ShouldLogging(  \
                    LOG_LEVEL_##severity)) {                    \
                Write::Instance().severity(msg);                \
            }                                                   \
        }                                                       \
    }

CREATE_STREAM_LOG(TRACE);
CREATE_STREAM_LOG(DEBUG);
CREATE_STREAM_LOG(INFO);
CREATE_STREAM_LOG(WARN);
CREATE_STREAM_LOG(ERROR);
CREATE_STREAM_LOG(CRITICAL);

class LogStreamHelper {
 public:
    LogStreamHelper() = default;
    void operator&(const std::ostream&) {}
    void operator&(std::ostream&) {}
};

#define CREATE_FILE_STREAM_LOG(severity)                           \
    template <typename Write = FileLogManager>                     \
    class FileLoggerStream##severity : public LoggerStreamBase<> { \
     public:                                                       \
        ~FileLoggerStream##severity() { Flush(); }                 \
                                                                   \
     protected:                                                    \
        inline void Flush() { Write::Instance().severity(str()); } \
    }

CREATE_FILE_STREAM_LOG(TRACE);
CREATE_FILE_STREAM_LOG(DEBUG);
CREATE_FILE_STREAM_LOG(INFO);
CREATE_FILE_STREAM_LOG(WARN);
CREATE_FILE_STREAM_LOG(ERROR);
CREATE_FILE_STREAM_LOG(CRITICAL);

}  // namespace senseAD
