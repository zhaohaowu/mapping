/*
 * Copyright (C) 2021 by SenseTime Group Limited. All rights reserved.
 * Kit Fung <fengzhongjie@sensetime.com>
 */

#pragma once

#include <cstdint>
#include <chrono>
#include <vector>
#include <array>
#include <mutex>
#include <memory>
#include <map>
#include <functional>
#include <utility>
#include <string>

namespace senseAD {
namespace profiler {

class BaseTimer {
 public:
    /**
     * @brief Start the Timer
     */
    virtual int64_t Start() = 0;
    /**
     * @brief Return the time interval started from the Start()
     * Unit: ms
     */
    virtual int64_t CountDuration() const = 0;
};

/**
 * @brief Timer Class for time interval measurement
 */
class Timer : public BaseTimer {
 public:
    explicit Timer(int tid = -1);
    int64_t Start() final;
    int64_t CountDuration() const final;

 private:
    using Clock = std::chrono::steady_clock;
    using TimePoint = std::chrono::time_point<Clock>;

    TimePoint begin_time_;
    int tid_;
};

/**
 * @brief Timer Class for cpu time interval measurement
 */
class CpuTimer : public BaseTimer {
 public:
    explicit CpuTimer(int tid = -1);
    int64_t Start() final;
    int64_t CountDuration() const final;

 private:
    using Clock = std::chrono::steady_clock;
    using TimePoint = std::chrono::time_point<Clock>;

    int64_t begin_time_;
    int tid_;
};

/**
 * @brief Memory Usage Info. Unit: KB
 */
struct RssInfo {
    // Virtual memory
    double vm_size;
    // Actual used memory
    double resident;
    double shared;
};

class ResourceProfiler {
 public:
    /**
     * @brief Get the Current Memory Usage Info
     *
     * @return RssInfo
     */
    static RssInfo GetCurrentRss();
    /**
     * @brief Get the Processor ID for current thread
     *
     * @return uint8_t
     */
    static uint8_t GetCpuId();
};

// Unit: ns
struct ProfilingStat {
 public:
    double min_ms = 0;
    double max_ms = 0;
    double mean_ms = 0;
    double variance = 0;
    uint64_t sample_cnt = 0;

    void AddSample(double measure_ms);
    void Print() const;
};

class ScopedFuncCall {
 public:
    ScopedFuncCall();
    explicit ScopedFuncCall(std::function<void()>&& at_exit);
    ScopedFuncCall(std::function<void()>&& at_start,
                   std::function<void()>&& at_exit);

    ~ScopedFuncCall();

 private:
    std::function<void()> at_exit_;
};

enum class TimeProfileOutputWhat : uint8_t {
    // Log every arrive
    kTracePointLog = 0,
    // kTracePointFile = 1,
    // Log only when signal trigger or destruction
    kSummaryLog = 2,
    // kSummaryFile = 3,
};

/**
 * @brief Provide Time Comsumption Profile Feature
 *
 *   Disabled by default.
 *   Three way to enable it:
 *    - set `PROFILING` env before run
 *    - manuall call TimeProfiler::Instance().Enable();
 *    - send 34 signal to the process
 */
class TimeProfiler {
 public:
    static TimeProfiler& Instance();

    // Taking the nvtx function name as reference

    /**
     * @brief Start a scoped section
     *
     * @param name section name
     * @return ScopedFuncCall RAII object
     */
    ScopedFuncCall Range(const std::string& name);
    /**
     * @brief Start a profile section
     * !IMPORTANT: Please don't split a section across thread
     * @param name The section name
     */
    void PushSection(const std::string& name);
    /**
     * @brief End the previous started section
     */
    void PopSection(const std::string& name = "");

    void PrintSummary();
    inline void Enable() { enable_ = true; }
    inline void Disable() { enable_ = false; }
    inline bool IsEnabled() { return enable_; }

 private:
    bool enable_ = false;
    int time_type_ = 0;  // 0 for counting actual time, 1 for counting cpu time,
    pid_t pid_ = -1;
    TimeProfileOutputWhat output_content_ =
        TimeProfileOutputWhat::kTracePointLog;

    using ProfilingStatMap = std::map<std::string, ProfilingStat>;
    static constexpr int kExpectedMaxThread = 64;

    std::array<ProfilingStatMap, kExpectedMaxThread> stats_;
    std::array<std::mutex, kExpectedMaxThread> stat_mtxs;

    //* Possible improvement, remove the vector, use circular buffer instead
    std::array<int, kExpectedMaxThread> section_cur_tid_;
    std::array<std::string, kExpectedMaxThread> section_cur_key_;
    std::array<std::string, kExpectedMaxThread> section_cur_func_;
    std::array<std::vector<uint32_t>, kExpectedMaxThread>
        section_keysize_stack_;
    std::array<std::vector<std::unique_ptr<BaseTimer>>, kExpectedMaxThread>
        section_timer_stack_;
    std::array<std::vector<int64_t>, kExpectedMaxThread> section_ts_stack_;

    TimeProfiler();
    void RewindStack(int32_t tid);
    void ThreadWorker();
};

}  // namespace profiler
}  // namespace senseAD

#define TIME_PROFILE_FORCE_ENABLE() \
    senseAD::profiler::TimeProfiler::Instance().Enable();

#define TIME_PROFILE_RANGE(name) \
    auto _profiler_ =            \
        senseAD::profiler::TimeProfiler::Instance().Range((#name));
#define TIME_PROFILE_PUSH(name) \
    senseAD::profiler::TimeProfiler::Instance().PushSection((#name));
#define TIME_PROFILE_POP_0_ARGS() \
    senseAD::profiler::TimeProfiler::Instance().PopSection();
#define TIME_PROFILE_POP_1_ARGS(name) \
    senseAD::profiler::TimeProfiler::Instance().PopSection((#name));
#define GET_2ND_ARG(arg1, arg2, FUNC, ...) FUNC
#define TIME_PROFILE_POP(...)                                          \
    GET_2ND_ARG(, ##__VA_ARGS__, TIME_PROFILE_POP_1_ARGS(__VA_ARGS__), \
                TIME_PROFILE_POP_0_ARGS(__VA_ARGS__))

// TODO(xuchenxiang): use macro overload
#define TIME_PROFILE_RANGE_STR(name) \
    auto _profiler_ = senseAD::profiler::TimeProfiler::Instance().Range(name);
#define TIME_PROFILE_PUSH_STR(name) \
    senseAD::profiler::TimeProfiler::Instance().PushSection(name);
#define TIME_PROFILE_POP_STR_0_ARGS() \
    senseAD::profiler::TimeProfiler::Instance().PopSection();
#define TIME_PROFILE_POP_STR_1_ARGS(name) \
    senseAD::profiler::TimeProfiler::Instance().PopSection(name);
#define TIME_PROFILE_POP_STR(...)                                          \
    GET_2ND_ARG(, ##__VA_ARGS__, TIME_PROFILE_POP_STR_1_ARGS(__VA_ARGS__), \
                TIME_PROFILE_POP_STR_0_ARGS(__VA_ARGS__))
