/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-11-29
 *****************************************************************************/
#include "lib/fault_manager/fault_manager.h"
#include "lib/io/file_util.h"
#include "onboard/onboard_lite/phm_comment_lite/phm_commenet.h"

namespace hozon {
namespace perception {
namespace common_onboard {
using hozon::perception::lib::FaultManager;
bool PhmComponent::InitFault() {
  std::string fm_config_file = "fm_config_file";
  GetFilePath(fm_config_file);
  HLOG_INFO << "[DEBUG_PHM] " << fm_config_file;
  // 初始化故障管理
  FaultManager::Instance()->Init(fm_config_file);
  // 注册故障上报接口
  FaultManager::Instance()->SetCallBackFaultReport(
      [this](const int32_t& faultid, const int32_t& objid,
             const int32_t& status, const int32_t& debounceCount,
             const int32_t& debounceTime, const base::DebounceType& debounce_type) {
        return FaultReport(faultid, objid, status, debounceCount, debounceTime, debounce_type);
      });
  // 注册故障恢复接口
  FaultManager::Instance()->SetCallBackReset([this]() { ResetFault(); });

  return true;
}

void PhmComponent::ResetFault() {
  if (!faultmap_.empty()) {
    for (const auto& it : faultmap_) {
      size_t pos = it.first.find('-');
      int16_t faultid = std::stoi(it.first.substr(0, pos));
      int16_t faultobj = std::stoi(it.first.substr(pos + 1));
      HLOG_WARN << "[DEBUG_PHM] MdcTransmitter::faultid " << faultid
                << " faultobj " << faultobj;
      SendFault_t cFault(faultid, faultobj, 0);
      phm_client_->ReportFault(cFault);
    }
    faultmap_.clear();
  }
}

void PhmComponent::ServiceAvailableCallback(const bool bResult) {
  HLOG_INFO << "[DEBUG_PHM] bResult: " << bResult;
  if (bResult) {
    HLOG_INFO << "[DEBUG_PHM] phm init succ: " << bResult;
  } else {
    HLOG_ERROR << "[DEBUG_PHM] phm init errr: " << bResult;
  }
}

void PhmComponent::FaultReceiveCallback(const ReceiveFault_t& fault) {
  HLOG_INFO << "[DEBUG_PHM] FaultReceiveCallback faultId: " << fault.faultId
            << " faultObj: " << fault.faultObj
            << " faultStatus: " << fault.faultStatus;
}

bool PhmComponent::FaultReport(const int32_t& faultid, const int32_t& objid,
                               const int32_t& status,
                               const int32_t& debounceCount,
                               const int32_t& debounceTime,
                               const base::DebounceType& debounce_type) {
  HLOG_DEBUG << "[DEBUG_PHM] faultid " << faultid << " objid " << objid
             << " status " << status;
  if (!phm_client_) {
    HLOG_ERROR << "[DEBUG_PHM] phm_client_ is nullptr ";
    return false;
  }

  std::string fault_name =
      std::to_string(faultid) + "-" + std::to_string(objid);

  auto it = faultmap_.find(fault_name);
  if (status == 1) {
    if (it == faultmap_.end()) {
      faultmap_.insert(std::make_pair(fault_name, status));
    }
    HLOG_WARN << "[DEBUG_CLUSTER_PHM_TEST] FaultManager ReportFault fault_name "
              << fault_name << " faultid " << faultid << " objid " << objid;
    SendFault_t cFault(faultid, objid, status);
    if (debounce_type == base::DebounceType::DEBOUNCE_TYPE_TIME) {
      cFault.faultDebounce.debounceType =
          hozon::netaos::phm::DebounceType::DEBOUNCE_TYPE_TIME;
      cFault.faultDebounce.debounceSetting.countDebounce.debounceTime =
          debounceTime;
    } else if (debounce_type == base::DebounceType::DEBOUNCE_TYPE_COUNT) {
      cFault.faultDebounce.debounceType =
          hozon::netaos::phm::DebounceType::DEBOUNCE_TYPE_COUNT;
      cFault.faultDebounce.debounceSetting.countDebounce.debounceCount =
          debounceCount;
      cFault.faultDebounce.debounceSetting.countDebounce.debounceTime =
          debounceTime;
    } else if (debounce_type == base::DebounceType::DEBOUNCE_TYPE_UNUSE) {
      cFault.faultDebounce.debounceType =
          hozon::netaos::phm::DebounceType::DEBOUNCE_TYPE_UNUSE;
    }
    phm_client_->ReportFault(cFault);
  } else if (0 == status) {
    SendFault_t cFault(faultid, objid, status);
    if (debounce_type == base::DebounceType::DEBOUNCE_TYPE_TIME) {
      cFault.faultDebounce.debounceType =
          hozon::netaos::phm::DebounceType::DEBOUNCE_TYPE_TIME;
      cFault.faultDebounce.debounceSetting.countDebounce.debounceTime =
          debounceTime;
    } else if (debounce_type == base::DebounceType::DEBOUNCE_TYPE_COUNT) {
      cFault.faultDebounce.debounceType =
          hozon::netaos::phm::DebounceType::DEBOUNCE_TYPE_COUNT;
      cFault.faultDebounce.debounceSetting.countDebounce.debounceCount =
          debounceCount;
      cFault.faultDebounce.debounceSetting.countDebounce.debounceTime =
          debounceTime;
    } else if (debounce_type == base::DebounceType::DEBOUNCE_TYPE_UNUSE) {
      cFault.faultDebounce.debounceType =
          hozon::netaos::phm::DebounceType::DEBOUNCE_TYPE_UNUSE;
    }
    phm_client_->ReportFault(cFault);
    if (it != faultmap_.end()) {
      HLOG_INFO << "[DEBUG_CLUSTER_PHM_TEST] FaultManager reset fault_name " << fault_name;
      faultmap_.erase(it);
    }
  }

  return true;
}

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
