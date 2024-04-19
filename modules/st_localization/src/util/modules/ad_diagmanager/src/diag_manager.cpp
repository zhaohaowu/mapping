/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Xu Minhao <xuminhao@senseauto.com>
 */

#include "ad_diagmanager/diag_manager.hpp"
#include "ad_log/ad_log.hpp"

namespace senseAD {
namespace diag {

/**
 * @brief  转换字符串至16进进制(4字节)
 *
 * @param s 代表16进制数的字符串
 * @param d 转换完成后的16进制数
 * @return true 转换成功
 * @return false 转换失败
 */
static bool convertHexString(const std::string &s, uint32_t *d) {
    if (d == NULL) {
        return false;
    }
    auto len = s.length();
    if (len > 8) {
        return false;
    }
    *d = 0U;
    for (auto i = 0; i < len; i++) {
        *d = *d << 4;
        if (s[i] >= '0' && s[i] <= '9') {
            *d += s[i] - '0';
        } else if (s[i] >= 'a' && s[i] <= 'f') {
            *d += (s[i] - 'a') + 10;
        } else if (s[i] >= 'A' && s[i] <= 'F') {
            *d += (s[i] - 'A') + 10;
        } else {
            AD_LERROR(convertHexString) << "invalid hex string " << s << "!";
            return false;
        }
    }
    return true;
}

Fault::Fault(FAULT_TYPE type,
             const std::string &fault_name,
             uint8_t bit,
             uint32_t set_filter_time_ms,
             uint32_t reset_filter_time_ms,
             uint32_t reset_count,
             uint32_t init_timout_s)
    : type_(type),
      fault_name_(fault_name),
      bit_(bit),
      set_filter_time_ms_(set_filter_time_ms),
      reset_filter_time_ms_(reset_filter_time_ms),
      reset_count_(reset_count < 1 ? 1 : reset_count),
      init_timout_s_(init_timout_s),
      init_timeout_(true) {}

Fault::~Fault() { disable(); }

bool Fault::enable() {
    {
        std::lock_guard<std::mutex> lock(m_);
        if (!dtc_ || enabled_) {
            return false;
        } else {
            enabled_ = true;
            set_ = false;
        }
    }
    thread_.reset(new std::thread(&Fault::faultUpdateWorker, this));
    return true;
}

bool Fault::disable() {
    {
        std::lock_guard<std::mutex> lock(m_);
        if (!dtc_ || !enabled_) {
            return false;
        } else {
            enabled_ = false;
        }
    }
    cv_.notify_one();
    thread_->join();
    return true;
}

void Fault::refresh() {
    std::lock_guard<std::mutex> lock(m_);
    if (type_ != DATA) {
        return;
    }
    /*使用初次timeout时间*/
    if (init_timeout_ && init_timout_s_ > 0) {
        next_timeout_ = start_time + std::chrono::seconds(init_timout_s_) +
                        std::chrono::milliseconds(set_filter_time_ms_);
        init_timeout_ = false;
        AD_LDEBUG(Fault) << fault_name_ << " first time out refresh "
                         << convertTime2Str(next_timeout_);
    } else {
        /*使用一般timeout时间*/
        if (next_timeout_ <
            std::chrono::system_clock::now() +
                std::chrono::milliseconds(set_filter_time_ms_)) {
            next_timeout_ = std::chrono::system_clock::now() +
                            std::chrono::milliseconds(set_filter_time_ms_);
        }

        AD_LDEBUG(Fault) << fault_name_ << "  time out refresh "
                         << convertTime2Str(next_timeout_);
    }
    cv_.notify_one();
    return;
}

void Fault::refreshNoLock() {
    if (type_ != DATA) {
        return;
    }
    next_timeout_ = std::chrono::system_clock::now() +
                    std::chrono::milliseconds(set_filter_time_ms_);
    AD_LDEBUG(Fault) << fault_name_ << "  time out refresh "
                     << convertTime2Str(next_timeout_);
    return;
}

void Fault::set() {
    std::lock_guard<std::mutex> lock(m_);
    set_ = true;
    cv_.notify_one();
    return;
}

void Fault::unset() {
    std::lock_guard<std::mutex> lock(m_);
    set_ = false;
    cv_.notify_one();
}

void Fault::faultUpdateWorker() {
    if (type_ == DATA) {
        faultUpdateData();
    } else {
        faultUpdateNoneData();
    }
}

void Fault::faultUpdateNoneData() {
    AD_LINFO(Fault) << fault_name() << " faultUpdateNoneData start ";
    while (enabled_) {
        std::unique_lock<std::mutex> lock(m_);
        auto prev_set = set_;
        cv_.wait(lock, [&] { return !enabled_ || prev_set != set_; });
        if (!enabled_) {
            break;
        }
        prev_set = set_;

        AD_LDEBUG(Fault) << fault_name() << " set = " << prev_set
                         << " start filtering ";
        // 进行滤波如果在filter_time时间内故障的状态没有变换，进行故障的上报
        auto filter_time =
            prev_set ? set_filter_time_ms_ : reset_filter_time_ms_;
        if (cv_.wait_for(lock, std::chrono::milliseconds(filter_time),
                         [&] { return !enabled_ || prev_set != set_; })) {
            if (!enabled_) {
                break;
            }
            AD_LDEBUG(Fault) << fault_name() << " prev_set = " << prev_set
                             << " set = " << set_ << " within " << filter_time
                             << " ms fault filtered!";

        } else {
            AD_LDEBUG(Fault)
                << fault_name() << " set = " << set_ << " sustained within "
                << filter_time << " ms fault to be reported";
            if (set_) {
                dtc_->set(this);
            } else {
                dtc_->reset(this);
            }
        }
    }
    AD_LINFO(Fault) << fault_name() << " faultUpdateNoneData exit ";
    return;
}

void Fault::faultUpdateData() {
    AD_LINFO(Fault) << fault_name() << " faultUpdateData start ";
    refresh();
    // switch_pt 代表着在这个时间点之前如果，如果故障的状态没有出现变化，
    // 就应该上报这个故障
    std::chrono::system_clock::time_point switch_pt =
        std::chrono::system_clock::time_point::max();
    // 用于记录在故障回复的时间(reset_filter_time_ms_)内
    // 接收到的数据帧数
    uint32_t reset_count = 0;
    while (enabled_) {
        bool need_report = false;
        std::unique_lock<std::mutex> lock(m_);
        if (next_timeout_ < std::chrono::system_clock::now()) {
            // 如果目前已经处于超时状态,则更新超时时间至下一个超时时间点
            refreshNoLock();
        }
        auto prev_set = set_;
        auto prev_timeout = next_timeout_;
        if (cv_.wait_until(lock, prev_timeout, [&] {
                return !enabled_ || prev_timeout != next_timeout_;
            })) {
            if (!enabled_) {
                break;
            }
            // 当出现超时状态的变化时，刷新switch_pt的时间
            if (prev_set) {
                set_ = false;
                switch_pt = std::chrono::system_clock::now() +
                            std::chrono::milliseconds(reset_filter_time_ms_);
                reset_count = 1;
                AD_LDEBUG(Fault) << fault_name() << " reset start filtering ";
            } else if (reset_count < reset_count_) {
                reset_count++;
            }

        } else {
            if (!prev_set) {
                set_ = true;
                switch_pt = std::chrono::system_clock::now();
                AD_LDEBUG(Fault) << fault_name() << " set = " << set_;
            }
        }

        if (std::chrono::system_clock::now() >= switch_pt) {
            AD_LDEBUG(Fault)
                << fault_name() << " set_ = " << set_ << " sustained within "
                << set_filter_time_ms_ << " ms fault to be reported";
            if (set_) {
                dtc_->set(this);
            } else if (reset_count >= reset_count_) {
                dtc_->reset(this);
            }
            switch_pt = std::chrono::system_clock::time_point::max();
        }
    }
    AD_LINFO(Fault) << fault_name() << " faultUpdateData exit ";
    return;
}

DTC::DTC(const std::string &dtc_name) : dtc_(dtc_name), event_id_(0) {}

DTC *DTC::clone(DTC *src) {
    if (src == NULL) {
        return NULL;
    }
    DTC *dst = new DTC(src->dtc());
    dst->event_id(src->event_id());
    return dst;
}

uint32_t DTC::dtcHex() const {
    uint32_t dtc_hex = 0;
    convertHexString(dtc(), &dtc_hex);
    return dtc_hex;
}

uint32_t DTC::event_id(uint32_t e) { return event_id_.exchange(e); }

bool DTC::addFault(const FaultPtr &p) {
    std::lock_guard<std::mutex> guard(m_);
    if (faults_.count(p->fault_name())) {
        return false;
    }
    faults_.insert({p->fault_name(), p});
    return true;
}

bool DTC::delFault(const std::string &fault_name) {
    std::lock_guard<std::mutex> guard(m_);
    if (faults_.count(fault_name) == 0) {
        return false;
    }
    return faults_.erase(fault_name) > 0;
}

DTC::FaultPtr DTC::getFault(std::string fault_name) {
    std::lock_guard<std::mutex> guard(m_);
    auto it = faults_.find(fault_name);
    if (it == faults_.end()) {
        return NULL;
    } else {
        return it->second;
    }
}

void DTC::set(Fault *f) {
    std::lock_guard<std::mutex> guard(m_);
    if (f && !isSet(f)) {
        set(f->bit());
        DiagManager::GetInstance()->Notify(this);
    }
}

void DTC::reset(Fault *f) {
    std::lock_guard<std::mutex> guard(m_);
    if (f && isSet(f)) {
        reset(f->bit());
        DiagManager::GetInstance()->Notify(this);
    }
}

bool DTC::enable(const std::string &fault_name) {
    std::lock_guard<std::mutex> guard(m_);
    auto it = faults_.find(fault_name);
    if (it != faults_.end()) {
        it->second->enable();
        return true;
    }
    return false;
}

bool DTC::disable(const std::string &fault_name) {
    std::lock_guard<std::mutex> guard(m_);
    auto it = faults_.find(fault_name);
    if (it != faults_.end()) {
        it->second->disable();
        return true;
    }
    return false;
}

bool DTC::isEnabled(const std::string &fault_name) {
    std::lock_guard<std::mutex> guard(m_);
    auto it = faults_.find(fault_name);
    if (it != faults_.end()) {
        return it->second->enabled();
    }
    return false;
}

void DTC::clear() { event_id_.store(0UL); }

DiagManager::~DiagManager() { Stop(); }

bool DiagManager::registerFault(const std::string &dtc_name,
                                Fault::FAULT_TYPE type,
                                const std::string &fault_name,
                                uint32_t event_bit,
                                uint32_t set_time_ms,
                                uint32_t reset_time_ms,
                                uint32_t reset_count,
                                uint32_t initial_timeout_s) {
    if (dtc_group_.count(dtc_name) == 0) {
        dtc_group_.insert({dtc_name, DTCPtr(new DTC(dtc_name))});
    }

    auto it = dtc_group_.find(dtc_name);

    if (it == dtc_group_.end()) {
        return false;
    }

    if (it->second->getFault(fault_name) != nullptr) {
        AD_LERROR(DiagManager) << "fault_name " << fault_name
                               << " already exisited  register fail!";
        return false;
    }

    it->second->addFault(DTC::FaultPtr(
        new Fault(type, fault_name, event_bit, set_time_ms, reset_time_ms,
                  reset_count, initial_timeout_s)));

    auto fault = it->second->getFault(fault_name);
    fault->setDTC(it->second.get());

    return true;
}

void DiagManager::reportFault(const std::string &dtc_name,
                              const std::string &fault_name,
                              bool abnormal) {
    auto it = dtc_group_.find(dtc_name);
    if (it == dtc_group_.end()) {
        AD_LERROR(DiagManager) << " dtc " << dtc_name << " is no registered !";
        return;
    }

    auto dtc = it->second;
    if (NULL == dtc) {
        AD_LERROR(DiagManager) << "empty DTC for " << dtc_name;
        return;
    }

    auto f = dtc->getFault(fault_name);

    if (NULL == f) {
        AD_LERROR(DiagManager) << fault_name << " is not found in " << dtc_name;
        return;
    }

    if (f->type() == Fault::DATA) {
        f->refresh();
    } else {
        if (abnormal) {
            f->set();
        } else {
            f->unset();
        }
    }
}

void DiagManager::Pause() {
    if (!idle_) {
        idle_ = true;
        Notify();
    }
    return;
}

void DiagManager::Resume() {
    if (idle_) {
        idle_ = false;
        Notify();
    }
    return;
}

bool DiagManager::Init(const dtc_sender &sender) {
    if (!sender) {
        AD_LERROR(DiagManager) << "init with empty sender!";
        return false;
    }
    sender_ = sender;
    AD_LINFO(DiagManager) << "message sender set!";
    return true;
}

void DiagManager::Start() {
    stop_.store(false);
    message_sender_thread_ = std::make_shared<std::thread>(
        std::bind(&DiagManager::messageSender, this));

    return;
}

void DiagManager::Stop() {
    if (!stop_.load()) {
        stop_.store(true);
        Notify();
    }
    if (message_sender_thread_) {
        message_sender_thread_->join();
        message_sender_thread_.reset();
    }
    while (!dtc_to_send_.empty()) {
        DTC *dtc = dtc_to_send_.front();
        delete dtc;
        dtc_to_send_.pop_front();
    }
}

void DiagManager::Notify(DTC *dtc_to_report) {
    std::lock_guard<std::mutex> lock(m_);
    if (dtc_to_report != NULL) {
        DTC *dtc = DTC::clone(dtc_to_report);
        AD_LDEBUG(DiagManager) << "notify from sender from " << dtc->dtc()
                               << " event_id = " << dtc->event_id();
        dtc_to_send_.push_back(dtc);
    }
    cv_.notify_one();
    return;
}

bool DiagManager::enable(const std::string &dtc,
                         const std::string &fault_name) {
    auto g_it = dtc_group_.find(dtc);
    if (g_it == dtc_group_.end()) {
        return false;
    }

    if (!fault_name.empty()) {
        return g_it->second->enable(fault_name);
    }

    bool res = true;
    for (auto &fault_it : g_it->second->faults_) {
        res = res && g_it->second->enable(fault_it.first);
        if (!res) {
            break;
        }
    }
    return res;
}

bool DiagManager::disable(const std::string &dtc,
                          const std::string &fault_name) {
    auto g_it = dtc_group_.find(dtc);
    if (g_it == dtc_group_.end()) {
        return false;
    }

    if (!fault_name.empty()) {
        return g_it->second->disable(fault_name);
    }

    bool res = true;
    for (auto &fault_it : g_it->second->faults_) {
        res = res && g_it->second->disable(fault_it.first);
        if (!res) {
            break;
        }
    }
    return res;
}

uint32_t DiagManager::event_id(const std::string dtc) {
    uint32_t event = 0;
    auto g_it = dtc_group_.find(dtc);
    if (g_it != dtc_group_.end()) {
        event = g_it->second->event_id();
    }
    return event;
}

void DiagManager::sendOnce() {
    if (!dtc_to_send_.empty() && sender_) {
        DTC *dtc = dtc_to_send_.front();
        if (dtc == NULL) {
            return;
        }
        if (sender_(dtc)) {
            AD_LINFO(DiagManager)
                << "DTC code send success: "
                << "dtc = " << dtc->dtc()
                << "status = " << static_cast<uint32_t>(dtc->event_id() != 0)
                << "event_id = " << static_cast<uint32_t>(dtc->event_id());
        } else {
            AD_LERROR(DiagManager)
                << "DTC code send fail:"
                << "dtc = " << dtc->dtc()
                << "status = " << static_cast<uint32_t>(dtc->event_id() != 0)
                << "event_id = " << static_cast<uint32_t>(dtc->event_id());
        }
        dtc_to_send_.pop_front();
        delete dtc;
    }
}

void DiagManager::messageSender() {
    if (sender_ == NULL) {
        AD_LERROR(DiagManager) << "msg sender not set !";
        return;
    }

    AD_LINFO(DiagManager) << "start diag message sending thread ";

    while (!sender_ && !stop_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        AD_LDEBUG(DiagManager) << "waiting for sender to set !";
    }
    AD_LINFO(DiagManager) << "diag message sending thread start ";
    while (!stop_.load()) {
        std::unique_lock<std::mutex> lock(m_);
        // 如果当前处于swc暂定状态，等到该状态解除再进行上报
        if (idle_.load()) {
            cv_.wait(lock, [&] { return stop_.load() || !idle_.load(); });
        }

        cv_.wait(lock, [&] {
            return stop_.load() || idle_.load() || !dtc_to_send_.empty();
        });
        if (stop_.load()) {
            break;
        }
        if (idle_.load()) {
            continue;
        }
        sendOnce();
    }

    while (!dtc_to_send_.empty()) {
        sendOnce();
    }

    AD_LINFO(DiagManager) << "exiting diag message sending thread";
    return;
}

}  // namespace diag
}  // namespace senseAD
