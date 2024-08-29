/*
 * Copyright (c) Hozon Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  ehp
 */
#pragma once
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "amap/GHDMapDefines.h"
#include "amap/GHDMapService.h"
#include "common/time/clock.h"
#include "proto/localization/localization.pb.h"

namespace hozon {
namespace mp {
namespace mf {
class DiagnosisImp : public ::hdmap::service::IHDMapDiagnosisReceiver {
 public:
  DiagnosisImp();
  DiagnosisImp(const DiagnosisImp& ada) = delete;
  DiagnosisImp& operator=(const DiagnosisImp& ada) = delete;
  DiagnosisImp(DiagnosisImp&& ada) = delete;
  DiagnosisImp& operator=(const DiagnosisImp&& ada) = delete;
  ~DiagnosisImp() override;
  void receiveDiagnosis(::hdmap::service::DiagnosisState state,
                        ::hdmap::service::DiagnosisType type,
                        const char* detailMessage) override;
  void setDiagnosisListener(
      ::hdmap::service::IHDMapDiagnosisListener* diagnosisListener) override;
  const ::hdmap::service::DiagnosisState& GetDiagnosisState();
  const ::hdmap::service::DiagnosisType& GetDiagnosisType();
  const std::string& GetDetailMessage();

 private:
  ::hdmap::service::IHDMapDiagnosisListener* listener_;
  ::hdmap::service::DiagnosisState diagnosis_state_;
  ::hdmap::service::DiagnosisType diagnosis_type_;
  std::string detail_message_;
};

}  // namespace mf

}  // namespace mp
}  // namespace hozon
