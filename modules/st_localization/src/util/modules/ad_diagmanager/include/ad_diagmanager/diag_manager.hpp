/*
 * Copyright (C) 2022 by SenseTime DTC Limited. All rights reserved.
 * Xu Minhao <xuminhao@senseauto.com>
 */

#pragma once

#include <atomic>
#include <chrono>
#include <iomanip>
#include <condition_variable>
#include <fstream>
#include <sstream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <list>
#include <utility>
#include <functional>

namespace senseAD {
namespace diag {
const std::chrono::system_clock::time_point start_time =
    std::chrono::system_clock::now();

static std::string convertTime2Str(std::chrono::system_clock::time_point t) {
    auto epoch = std::chrono::time_point_cast<std::chrono::milliseconds>(t)
                     .time_since_epoch();
    auto ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(epoch).count() %
        1000;

    auto timer = std::chrono::system_clock::to_time_t(t);

    // convert to broken time
    std::tm bt = *std::localtime(&timer);

    std::ostringstream oss;

    oss << std::put_time(&bt, "%Y-%m-%d-%H:%S");  // HH:MM:SS
    oss << '.' << std::setfill('0') << std::setw(3) << ms;

    return oss.str();
}

class DiagManager;
class DTC;

/*
Fault 代表某一个具体的故障事件
具体的故障内容请参考　
https://ones.ainewera.com/wiki/#/team/Ttz6FJha/space/FkzEHmYf/page/Xm5hLRDT
*/
class Fault {
 public:
    friend class DTC;
    friend class DiagManager;
    enum FAULT_TYPE {
        DATA = 0,  // 　数据缺失故障
        SDK = 1,   //   SDK故障
        SWC = 2,   //   SWC故障
    };

    /**
     * @brief 新建一个故障对象
     *
     * @param type  FAULT_TYPE中的一种
     * @param fault_name  该故障的名称
     * @param bit         该故障对应的位
     * @param set_filter_time_ms  该故障置位需要的滤波时间(ms)
     * 对于数据缺失类型的故障在这个时间内如果没有收到1帧数据故障置位
     * @param reset_filter_time_ms 该故障复位需要的滤波时间(ms)
     * @param reset_count
     * 对于数据类型的故障，判定故障恢复需要在reset_filter_time_ms_时间内收到超过reset_count_帧的数据
     * @param init_timout_s
     * 该故障的初次timeout时间(仅针对FAULTTYPE=DATA的故障有效)
     */

    Fault(FAULT_TYPE type,
          const std::string &fault_name,
          uint8_t bit,
          uint32_t set_filter_time_ms = 0,
          uint32_t reset_filter_time_ms = 0,
          uint32_t reset_count = 0,
          uint32_t init_timout_s = 0);

    ~Fault();
    /**
     * @brief 返回故障类型
     */
    FAULT_TYPE type() const { return type_; }
    /**
     * @brief 返回故障名称
     */
    std::string fault_name() const { return fault_name_; }
    /**
     * @brief 返回故障码对应的比特位
     */
    uint8_t bit() const { return bit_; }
    /**
     * @brief 返回故障置位需要的时间
     */
    uint32_t set_time() const { return set_filter_time_ms_; }
    /**
     * @brief 返回故障复位需要的时间
     */
    uint32_t reset_time() const { return reset_filter_time_ms_; }
    /**
     * @brief 返回数据故障缺失初次timout时间单位(s)
     */
    uint32_t init_timout_s() const { return init_timout_s_; }

    /**
     * @brief 返回当前故障码是否被使能
     */
    bool enabled() const { return enabled_; }

 private:
    /**
     * @brief 使能该故障的上报
     *
     */
    bool enable();
    /**
     * @brief 抑制故障的上报
     *
     */
    bool disable();

    /**
     * @brief 故障发生
     *
     */
    void set();

    /**
     * @brief 故障消失
     *
     */
    void unset();

    /**
     * @brief 刷新数据下次一次timeout时间
     *  在类内部使用，需要获得锁的情况下调用
     *
     */
    void refreshNoLock();

 public:
    /**
     * @brief 比较两个故障码，当两个故障码的名称相同时，认为两者相同
     */
    bool operator==(const Fault &rhs) const {
        return fault_name_ == rhs.fault_name_;
    }

    /**
     * @brief 设置该子故障所属的dtc
     */
    void setDTC(DTC *dtc) { dtc_ = dtc; }

    /**
     * @brief 返回该子故障所属的DTC
     */
    DTC *getDTC() { return dtc_; }

    /**
     * @brief 刷新该故障的计时器，如果在timeout_ms_未被刷新，故障码置位
     */
    void refresh();

 private:
    /**
     * @brief 故障监视线程
     *
     */
    void faultUpdateWorker();
    /**
     * @brief 监视本故障的状态，适用于data类型的故障
     *
     */
    void faultUpdateData();

    /**
     * @brief 监视本故障的状态适用于非data类型的状态
     *
     */

    void faultUpdateNoneData();

 private:
    Fault &operator=(const Fault &) = delete;
    Fault(const Fault &other) = delete;
    const FAULT_TYPE type_;         // 故障类型
    const std::string fault_name_;  // 故障名称
    const uint8_t bit_;             // 故障对应的比特位
    const uint32_t
        set_filter_time_ms_;  // 故障置位滤波时间该故障置位需要的滤波时间(ms)
    // 对于数据缺失类型的故障在这个时间内如果没有收到1帧数据故障置位
    const uint32_t reset_filter_time_ms_;  // 故障恢复滤波时间
    const uint32_t reset_count_;
    const uint32_t init_timout_s_;
    bool init_timeout_;                 // 是否初次超时时间
    std::atomic<bool> enabled_{false};  // 是否使能当前故障的上报，默认不上报
    DTC *dtc_{NULL};                    //  该子故障所属的dtc_;
    std::shared_ptr<std::thread> thread_{
        NULL};  // 在本故障的设置/取消时进行相应的滤波

    std::condition_variable cv_;
    std::mutex m_;
    bool set_{false};  // 该故障是否发生

    std::chrono::system_clock::time_point
        next_timeout_;  //  下次timeout时间，仅对数据缺失类型故障
};

/*
代表某一个DTC号对应的故障诊断码
*/

class DTC {
 public:
    friend class DiagManager;
    typedef std::shared_ptr<Fault> FaultPtr;

    explicit DTC(const std::string &dtc_name);

    /**
     * @brief 创建新的DTC，并从已有的DTC中复event_id
     *
     * @return 如果src不为空 返回和src内容一直的DTC
     */
    static DTC *clone(DTC *src);

    ~DTC(){};
    /**
     * @brief 返回DTC号
     *
     * @return std::string
     */
    std::string dtc() const { return dtc_; }
    /**
     * @brief 转化DTC号作为16进制字符转为数字
     *
     * @return uiint32_t
     */
    uint32_t dtcHex() const;

    /**
     * @brief 返回该DTC对应的故障码
     *
     * @return uint32_t
     */
    uint32_t event_id() const { return event_id_.load(); }
    /**
     ** @brief 设置event_id
     * @return uint32_t 设置前的event_id
     */
    uint32_t event_id(uint32_t e);

    /**
     * @brief 在该dtc中添加子故障
     *
     * @param p 需要添加的子故障
     * @return p已经存在->返回失败 , p不存在->返回成功
     */
    bool addFault(const FaultPtr &p);

    /**
     * @brief 在该dtc中删除故障
     *
     * @param fault_name　子故障名称
     * @return f已经存在-返回成功 , f不存在->返回失败
     */
    bool delFault(const std::string &fault_name);

    /**
     * @brief 返回子故障对象
     *
     * @param fault_name　子故障名称
     * @return FaultPtr　子故障存在返回该子故障的指针，否则返回null
     */
    FaultPtr getFault(std::string fault_name);

    /**
     * @brief 置位子故障
     *
     * @param f　出现故障的子故障
     */
    void set(Fault *f);

    /**
     * @brief 复位子故障
     *
     * @param f　故障恢复的子故障
     */
    void reset(Fault *f);

    /**
     * @brief 判断某个子故障是否置位
     *
     * @param f　子故障　
     * @return　如果f对应的子故障置位　返回true否则返回false
     */
    bool isSet(Fault *f) { return (event_id_ & (1UL << f->bit())); }
    /**
     * @brief　使能子故障
     *
     * @param fault_name　需要使能的子故障名称
     * @return 该子故障存在返回true,否在返回false
     */
    bool enable(const std::string &fault_name);

    /**
     * @brief　禁用子故障，子故障在被禁用后依然会被监视,只是不会被上报
     *
     * @param fault_name　需要禁用的子故障名称
     * @return 该子故障存在返回true,否在返回false
     */
    bool disable(const std::string &fault_name);

    /**
     * @brief 某特定的子故障是否被使能
     *
     * @param fault_name 自故障名称
     * @return true
     * @return false
     */
    bool isEnabled(const std::string &fault_name);

 private:
    /**
     * @brief 重置故障码
     *
     */
    void clear();

    DTC &operator=(const DTC &other) = delete;
    DTC(const DTC &other) = delete;

    void set(uint8_t bit) {
        if (bit < 32) {
            event_id_ |= (1UL << bit);
        }
    }

    void reset(uint8_t bit) {
        if (bit < 32) {
            event_id_ &= ~(1UL << bit);
        }
    }

    std::mutex m_;
    std::unordered_map<std::string, FaultPtr> faults_;  // 该DTC包含的故障集合
    const std::string dtc_;                             // DTC号
    std::atomic<uint32_t> event_id_{0};  // 当前DTC号对应的event_id
};

using DTCPtr = std::shared_ptr<DTC>;
using dtc_sender = std::function<bool(DTC *)>;

/*故障诊断码管理类*/
class DiagManager {
 public:
    /**
     * @brief 获取全局唯一的故障诊断码管理对象
     *
     * @return DiagManager*　
     */
    static DiagManager *GetInstance() {
        static DiagManager instance;
        return &instance;
    }

    ~DiagManager();

    DiagManager(const DiagManager &) = delete;
    DiagManager &operator=(const DiagManager &) = delete;
    /**
     * @brief 注册需要监视的子故障
     *
     * @param dtc_name       DTC号
     * @param type　　    　　子故障类型
     * @param fault_name　　　子故障名称
     * @param event_bit　　　 子故障对应的比特位
     * @param set_time_ms    自故障置位需要的时间
     ** @param reset_time_ms  自故障复位需要的时间
     * @param reset_count    自故障复位需要的帧数（仅针对数据故障缺失故障）
     * @param initial_timeout_s　初次timeout时间（仅针对数据故障缺失故障）
     * @return 　true->添加成功　false->添加失败
     */
    bool registerFault(const std::string &dtc_name,
                       Fault::FAULT_TYPE type,
                       const std::string &fault_name,
                       uint32_t event_bit,
                       uint32_t set_time_ms,
                       uint32_t reset_time_ms,
                       uint32_t reset_count = 0,
                       uint32_t initial_timeout_s = 0);
    /**
     * @brief 向故障管理汇报故障的状态
     * 对于数据缺失故障，在接收到数据后调用该接口来刷新数据缺失的计时器
     *
     * @param dtc_name 　　　　　故障属于的DTC号
     * @param fault_name　　　子故障名称
     * @param abnormal　　　　子故障发生/恢复
     */
    void reportFault(const std::string &dtc_name,
                     const std::string &fault_name,
                     bool abnormal = false);
    /**
     * @brief 暂定故障上报
     */
    void Pause();

    /**
     * @brief 回复故障的上报
     */
    void Resume();

    /**
     * @brief 数据发送类型
     *
     * @param sender 用于发送DTC消息的回调函数
     */
    bool Init(const dtc_sender &sender);

    /**
     * @brief 开始故障上报
     *
     */
    void Start();

    /**
     * @brief 结束故障上报
     */
    void Stop();

    /**
     * @brief 使能指定的子故障
     *
     * @param dtc 　　　 子故障所属的DTC号
     * @param fault_name　　子故障名称 如果为空则使能dtc下所有的故障
     * @return true->使能成功　false->使能失败
     */
    bool enable(const std::string &dtc, const std::string &fault_name = "");
    /**
     * @brief 禁用指定的子故障
     *
     * @param dtc 　　　 子故障所属的DTC号
     * @param fault_name　　子故障名称 如果为空则禁用该dtc下所有的故障
     * @return true->禁用成功　false->禁用失败
     */
    bool disable(const std::string &DTC_id, const std::string &fault_name = "");

    /**
     * @brief 返回指定DTC的event_id
     *
     * @param DTC_id 　
     * @return dtc_name DTC_id 对应的event_id
     */
    uint32_t event_id(const std::string DTC_id);

    /**
     * @brief 通知故障处理线程可能的更新
     * @param dtc_to_report
     * 如果需要通知的事件是有新的dtc需要上报那么，该指针指向需要上报的dtc
     */
    void Notify(DTC *dtc_to_report = NULL);

 private:
    DiagManager() : stop_(true) {}

    /**
     * @brief 监视所有的数据缺失故障
     */
    void dataMonitor();

    /**
     * @brief 进行一次故障上报
     */
    void sendOnce();

    /**
     * @brief 以一定的周期进行故障的上报
     *
     */
    void messageSender();

 private:
    std::atomic<bool> stop_;
    std::atomic<bool> idle_;

    dtc_sender sender_;

    std::mutex m_;
    std::condition_variable cv_;

    std::list<DTC *> dtc_to_send_;

    std::unordered_map<std::string, DTCPtr> dtc_group_;
    //  故障码发送线程
    std::shared_ptr<std::thread> message_sender_thread_;
};

#define INIT_DIAG_MANAGER(SENDER)                                       \
    {                                                                   \
        auto diag_instance = senseAD::diag::DiagManager::GetInstance(); \
        diag_instance->Init(SENDER);                                    \
    }

#define START_DIAG_MANAGER                                              \
    {                                                                   \
        auto diag_instance = senseAD::diag::DiagManager::GetInstance(); \
        diag_instance->Start();                                         \
    }

#define STOP_DIAG_MANAGER                                               \
    {                                                                   \
        auto diag_instance = senseAD::diag::DiagManager::GetInstance(); \
        diag_instance->Stop();                                          \
    }

#define REGISTER_DIAG_MESSAGE(DTC, TYPE, MESSAGE, BIT, SET_TIME, RESET_TIME, \
                              RESET_COUNT, INIT_TIME)                        \
    {                                                                        \
        auto diag_instance = senseAD::diag::DiagManager::GetInstance();      \
        diag_instance->registerFault(DTC, TYPE, MESSAGE, BIT, SET_TIME,      \
                                     RESET_TIME, RESET_COUNT, INIT_TIME);    \
    }

#define REGISTER_DIAG_DATA(DTC, MESSAGE, BIT, SET_TIME, RESET_TIME,          \
                           RESET_COUNT, INIT_TIME)                           \
    {                                                                        \
        REGISTER_DIAG_MESSAGE(DTC, senseAD::diag::Fault::DATA, MESSAGE, BIT, \
                              SET_TIME, RESET_TIME, RESET_COUNT, INIT_TIME)  \
    }

#define REGISTER_DIAG_SWC(DTC, MESSAGE, BIT, SET_TIME, RESET_TIME)      \
    REGISTER_DIAG_MESSAGE(DTC, senseAD::diag::Fault::SWC, MESSAGE, BIT, \
                          SET_TIME, RESET_TIME, 0, 0)

#define REGISTER_DIAG_SDK(DTC, MESSAGE, BIT, SET_TIME, RESET_TIME)      \
    REGISTER_DIAG_MESSAGE(DTC, senseAD::diag::Fault::SDK, MESSAGE, BIT, \
                          SET_TIME, RESET_TIME, 0, 0)

#define PAUSE_DIAG                                                      \
    {                                                                   \
        auto diag_instance = senseAD::diag::DiagManager::GetInstance(); \
        diag_instance->Pause();                                         \
    }

#define RESUME_DIAG                                                     \
    {                                                                   \
        auto diag_instance = senseAD::diag::DiagManager::GetInstance(); \
        diag_instance->Resume();                                        \
    }

#define DIAG_ENABLE(DTC, MESSAGE)                                       \
    {                                                                   \
        auto diag_instance = senseAD::diag::DiagManager::GetInstance(); \
        diag_instance->enable(DTC, MESSAGE);                            \
    }

#define DIAG_DISABLE(DTC, MESSAGE)                                      \
    {                                                                   \
        auto diag_instance = senseAD::diag::DiagManager::GetInstance(); \
        diag_instance->disable(DTC, MESSAGE);                           \
    }

#define DIAG_REPORT_ABNORM(DTC, MESSAGE, ABNORMAL)                      \
    {                                                                   \
        auto diag_instance = senseAD::diag::DiagManager::GetInstance(); \
        diag_instance->reportFault(DTC, MESSAGE, ABNORMAL);             \
    }

#define DIAG_REPORT_DATA(DTC, MESSAGE) DIAG_REPORT_ABNORM(DTC, MESSAGE, false)

#define GET_DTC_EVENT(DTC) \
    senseAD::diag::DiagManager::GetInstance()->event_id(dtc);

}  // namespace diag
}  // namespace senseAD
