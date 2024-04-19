/*
 * Copyright (C) 2021 by SenseTime Group Limited. All rights reserved.
 * Kit Fung <fengzhongjie@sensetime.com>
 */

#include "ad_log/profiler.hpp"

#ifdef WITH_TDA_QNX
#include <sys/process.h>
#include <sys/neutrino.h>
#include <sys/procfs.h>
#include <devctl.h>
#else
#include <sys/syscall.h>
#endif
#include <unistd.h>
#include <sched.h>
#include <cstdlib>
#include <csignal>

#include <iostream>
#include <thread>
#include <fstream>
#include <atomic>
#include <unordered_map>

#include "ad_log/ad_log.hpp"

#define likely(x) __builtin_expect((x), 1)
#define unlikely(x) __builtin_expect((x), 0)

namespace senseAD {
namespace profiler {

namespace {
constexpr uint64_t PROC_CPU_LOG_INTERVAL = 1000;  // ms

std::atomic<int32_t> thread_counter(0);
thread_local int32_t t_thread_index = -1;

int32_t GetThreadIndex() {
    if (likely(t_thread_index != -1)) {
        return t_thread_index;
    }
    t_thread_index = thread_counter++;
    return t_thread_index;
}
}  // namespace

Timer::Timer(int tid) : tid_(tid) {}

int64_t Timer::Start() {
    begin_time_ = Timer::Clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               begin_time_.time_since_epoch())
        .count();
}

int64_t Timer::CountDuration() const {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               Timer::Clock::now() - begin_time_)
        .count();
}

CpuTimer::CpuTimer(int tid) : tid_(tid) {}

int64_t CpuTimer::Start() {
    begin_time_ = 0;
    struct timespec tspec;
    if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &tspec) != 0) {
        LOGFILESTREAM_SEVERITY(ERROR) << "clock_gettime() failed";
    } else {
        begin_time_ = tspec.tv_sec * 1000000000 + tspec.tv_nsec;
    }
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               CpuTimer::Clock::now().time_since_epoch())
        .count();
}

int64_t CpuTimer::CountDuration() const {
    struct timespec tspec;
    if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &tspec) != 0) {
        LOGFILESTREAM_SEVERITY(ERROR) << "clock_gettime() failed";
        return 0.0;
    } else {
        return tspec.tv_sec * 1000000000 + tspec.tv_nsec - begin_time_;
    }
}

void ProfilingStat::AddSample(double measure_ms) {
    if (sample_cnt++ == 0) {
        min_ms = measure_ms;
        max_ms = measure_ms;
        mean_ms = measure_ms;
    } else {
        min_ms = std::min(min_ms, measure_ms);
        max_ms = std::max(max_ms, measure_ms);
        double new_mean_ms = mean_ms + (measure_ms - mean_ms) / sample_cnt;
        variance =
            variance + (measure_ms - mean_ms) * (measure_ms - new_mean_ms);
        mean_ms = new_mean_ms;
    }
}

void ProfilingStat::Print() const {
    LOGFILESTREAM_SEVERITY(INFO) << min_ms << "," << max_ms << "," << mean_ms
                                 << "," << variance << "," << sample_cnt;
}

ScopedFuncCall::ScopedFuncCall() {}
ScopedFuncCall::ScopedFuncCall(std::function<void()>&& at_exit)
    : at_exit_(at_exit) {}
ScopedFuncCall::ScopedFuncCall(std::function<void()>&& at_start,
                               std::function<void()>&& at_exit)
    : at_exit_(at_exit) {
    at_start();
}

ScopedFuncCall::~ScopedFuncCall() {
    if (at_exit_) {
        at_exit_();
    }
}

RssInfo ResourceProfiler::GetCurrentRss() {
    int64_t vm_size = 0, resident = 0, share = 0;
    std::ifstream buffer("/proc/self/statm");
    buffer >> vm_size >> resident >> share;
    buffer.close();

    int64_t page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024;

    RssInfo info;
    info.vm_size = vm_size * page_size_kb;
    info.resident = resident * page_size_kb;
    info.shared = share * page_size_kb;
    return std::move(info);
}

uint8_t ResourceProfiler::GetCpuId() {
#ifdef WITH_TDA_QNX
    return SchedGetCpuNum();
#else
    return sched_getcpu();
#endif
}

TimeProfiler& TimeProfiler::Instance() {
    static TimeProfiler profiler;
    return profiler;
}

void ToggleTimeProfiler(int sig) {
    std::cout << "ToggleTimeProfiler" << std::endl;
    auto& instance = TimeProfiler::Instance();
    if (!instance.IsEnabled()) {
        TimeProfiler::Instance().Enable();
    } else {
        TimeProfiler::Instance().Disable();
    }
}

void TriggerSummayProfile(int sig) {
    auto& instance = TimeProfiler::Instance();
    if (instance.IsEnabled()) {
        instance.PrintSummary();
    }
}

TimeProfiler::TimeProfiler() {
    // Initial check
    const char* env_e = std::getenv("PROFILING_ENABLE");
    enable_ = env_e != nullptr;
    const char* env_m = std::getenv("PROFILING_MODE");
    const char* env_time = std::getenv("PROFILING_TIME");
    if (env_m != nullptr) {
        int flag = atoi(env_m);
        output_content_ = static_cast<TimeProfileOutputWhat>(flag);
    }
    if (env_time != nullptr) {
        time_type_ = atoi(env_time);
    }
    LOGFILESTREAM_SEVERITY(INFO) << "PROFILING_TIME: " << time_type_;

#ifdef WITH_TDA_QNX
    pid_ = getpid();
#else
    pid_ = syscall(__NR_getpid);
#endif

    // Allow enable dynamically
    signal(34, ToggleTimeProfiler);
    signal(35, TriggerSummayProfile);

    std::thread t([this] { ThreadWorker(); });
    t.detach();
}

ScopedFuncCall TimeProfiler::Range(const std::string& name) {
    PushSection(name);
    return enable_ ? ScopedFuncCall([this, name]() { this->PopSection(name); })
                   : ScopedFuncCall();
}

void TimeProfiler::PushSection(const std::string& name) {
    if (!enable_) {
        return;
    }
    auto thread_idx = GetThreadIndex();
    section_cur_func_[thread_idx] = name;
    section_cur_key_[thread_idx] = section_cur_key_[thread_idx] + "/" + name;
#ifdef WITH_TDA_QNX
    section_cur_tid_[thread_idx] = pthread_self();
#else
    section_cur_tid_[thread_idx] = syscall(__NR_gettid);
#endif
    // In Stack
    section_keysize_stack_[thread_idx].emplace_back(name.size());
    if (time_type_ == 0) {
        section_timer_stack_[thread_idx].emplace_back(
            std::make_unique<Timer>(section_cur_tid_[thread_idx]));
    } else {
        section_timer_stack_[thread_idx].emplace_back(
            std::make_unique<CpuTimer>(section_cur_tid_[thread_idx]));
    }
    section_ts_stack_[thread_idx].emplace_back(
        section_timer_stack_[thread_idx].back()->Start());
}

void TimeProfiler::PopSection(const std::string& name) {
    if (!enable_) {
        return;
    }

    auto thread_idx = GetThreadIndex();
    if (unlikely(section_timer_stack_[thread_idx].empty())) {
        return;
    }

    auto& key = section_cur_key_[thread_idx];
    if (name.length() > 0) {
        if (section_cur_key_[thread_idx].find(name) == std::string::npos) {
            // func already poped
            return;
        }
        for (;;) {
            auto func = key.substr(key.rfind('/') + 1);
            if (func == name) {
                break;
            }
            // pop incomplete Push/Pop pair
            RewindStack(thread_idx);
        }
    }

    // Update stat
    auto& tid = section_cur_tid_[thread_idx];
    auto& timer = section_timer_stack_[thread_idx].back();
    auto& ts = section_ts_stack_[thread_idx].back();
    auto measure = timer->CountDuration();

    auto& stat = stats_[thread_idx];
    auto itr = stat.find(key);
    if (itr == stat.end()) {
        std::lock_guard<std::mutex> lock(stat_mtxs[thread_idx]);
        itr = stat.emplace(std::make_pair(key, ProfilingStat())).first;
    }
    itr->second.AddSample(measure / 1000000.0);
    // Update cur key
    auto key_size = key.size();
    auto level_key_size = section_keysize_stack_[thread_idx].back();
    // Handle on arrive output
    switch (output_content_) {
        case TimeProfileOutputWhat::kTracePointLog:
            LOGFILESTREAM_SEVERITY(INFO)
                << " " << pid_ << " " << tid << " " << key << " : " << measure
                << " ns " << ts;
            break;
        case TimeProfileOutputWhat::kSummaryLog:
            LOGFILESTREAM_SEVERITY(INFO)
                << " " << pid_ << " " << tid << " " << key;
            itr->second.Print();
            break;
        default:
            break;
    }

    RewindStack(thread_idx);
}

void TimeProfiler::PrintSummary() {
    for (int i = 0; i < kExpectedMaxThread; ++i) {
        auto& tid = section_cur_tid_[i];
        std::lock_guard<std::mutex> lock(stat_mtxs[i]);
        for (const auto& itr : stats_[i]) {
            LOGFILESTREAM_SEVERITY(INFO)
                << " " << pid_ << " " << tid << " " << itr.first;
            itr.second.Print();
        }
    }
}

void TimeProfiler::RewindStack(int32_t tid) {
    auto& key = section_cur_key_[tid];
    auto key_size = key.size();
    auto level_key_size = section_keysize_stack_[tid].back();
    section_cur_key_[tid] = key.substr(0, key_size - level_key_size - 1);
    section_timer_stack_[tid].pop_back();
    section_keysize_stack_[tid].pop_back();
    section_ts_stack_[tid].pop_back();
}

void TimeProfiler::ThreadWorker() {
    int64_t last_time = 0;

#ifdef WITH_TDA_QNX
    char proc_file[_POSIX_PATH_MAX] = {0};
    snprintf(proc_file, _POSIX_PATH_MAX, "/proc/%d/as", pid_);
    int fd = open(proc_file, O_RDONLY);
    if (fd == -1) {
        LOGFILESTREAM_SEVERITY(ERROR)
            << "Open file " << proc_file << "error: " << errno << " "
            << strerror(errno);
    }
    std::unordered_map<uint32_t, procfs_status> thread_info;
    int tid = 1;
    for (;;) {
        procfs_status t_info;
        t_info.tid = tid;

        if (devctl(fd, DCMD_PROC_TIDSTATUS, &t_info, sizeof(t_info), 0) !=
            EOK) {
            break;
        }

        thread_info.emplace(t_info.tid, std::move(t_info));
        tid = t_info.tid + 1;
    }
    close(fd);
#endif

    for (;;) {
        auto start = std::chrono::steady_clock::now();
        if (enable_) {
            struct timespec tspec;
            if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tspec) != 0) {
                LOGFILESTREAM_SEVERITY(ERROR) << "clock_gettime() failed";
            } else {
                int64_t cur_time = tspec.tv_sec * 1000000000 + tspec.tv_nsec;
                if (last_time > 0) {
                    LOGFILESTREAM_SEVERITY(INFO)
                        << "process cpu time: " << cur_time - last_time
                        << " ns "
                        << std::chrono::duration_cast<std::chrono::nanoseconds>(
                               start.time_since_epoch())
                               .count();
                }
                last_time = cur_time;
            }
#ifdef WITH_TDA_QNX
            fd = open(proc_file, O_RDONLY);
            tid = 1;
            if (fd != -1) {
                for (;;) {
                    procfs_status t_info;
                    t_info.tid = tid;

                    if (devctl(fd, DCMD_PROC_TIDSTATUS, &t_info, sizeof(t_info),
                               0) != EOK) {
                        break;
                    }

                    if (thread_info.count(t_info.tid) == 1) {
                        LOGFILESTREAM_SEVERITY(INFO)
                            << "thread " << t_info.tid << " cpu time: "
                            << t_info.sutime - thread_info[t_info.tid].sutime
                            << " ns "
                            << std::chrono::duration_cast<
                                   std::chrono::nanoseconds>(
                                   start.time_since_epoch())
                                   .count();
                        thread_info[t_info.tid].sutime = t_info.sutime;
                    } else {
                        thread_info.emplace(t_info.tid, std::move(t_info));
                    }
                    tid = t_info.tid + 1;
                }
            }
            close(fd);
#endif
        }
        uint64_t dur = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::steady_clock::now() - start)
                           .count();
        if (dur > PROC_CPU_LOG_INTERVAL) {
            LOGFILESTREAM_SEVERITY(ERROR)
                << "Cost too many time to get proc cpu: " << dur << " ms";
            continue;
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(PROC_CPU_LOG_INTERVAL - dur));
    }
}

}  // namespace profiler
}  // namespace senseAD
