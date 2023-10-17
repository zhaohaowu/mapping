/*
 * Copyright (c) Hozon Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  ehp
 */

#include "map_fusion/map_service/ehp/amap_diagnose.h"

#include <memory.h>

#include <cmath>
#include <ctime>
#include <iostream>

#include "amap/GHDDiagnosisDefines.h"
#include "common/configs/config_gflags.h"
#include "util/temp_log.h"

namespace hozon {
namespace mp {
namespace mf {
// 诊断处理 demo
DiagnosisImp::DiagnosisImp()
    : listener_(nullptr),
      diagnosis_state_(::hdmap::service::DIAGNOSIS_STATUS_OFF),
      diagnosis_type_(static_cast<::hdmap::service::DiagnosisType>(0)) {}

DiagnosisImp::~DiagnosisImp() { listener_ = nullptr; }

void DiagnosisImp::receiveDiagnosis(::hdmap::service::DiagnosisState state,
                                    ::hdmap::service::DiagnosisType type,
                                    const char* detailMessage) {
  if (state == ::hdmap::service::DIAGNOSIS_STATUS_ON) {
    diagnosis_state_ = state;
    diagnosis_type_ = type;
    if (detailMessage != nullptr) {
      detail_message_ = detailMessage;
    }
  } else {
    diagnosis_state_ = state;
    diagnosis_type_ = static_cast<::hdmap::service::DiagnosisType>(0);
    detail_message_.clear();
  }

  // 处理诊断消息
}

void DiagnosisImp::setDiagnosisListener(
    ::hdmap::service::IHDMapDiagnosisListener* diagnosisListener) {
  listener_ = diagnosisListener;
  // 测试代码,系统方主动调用清除所有诊断接口
  // if (listener_) {
  //   HLOG_ERROR << "diagnosisListener";
  //   listener_->onDiagnosisCleared();
  // }
}

const ::hdmap::service::DiagnosisState& DiagnosisImp::GetDiagnosisState() {
  return diagnosis_state_;
}

const ::hdmap::service::DiagnosisType& DiagnosisImp::GetDiagnosisType() {
  return diagnosis_type_;
}

const std::string& DiagnosisImp::GetDetailMessage() { return detail_message_; }
}  // namespace mf

}  // namespace mp
}  // namespace hozon
