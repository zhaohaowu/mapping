/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-09-09
 用于localmap的onboard板端对外接口实现。包含上游数据输入和对外数据发送。
 *****************************************************************************/

#pragma once

#include <memory>

#include "adf-lite/include/base.h"
#include "depend/nos/x86_2004/include/adf-lite/include/executor.h"
#include "depend/nos/x86_2004/include/adf/include/node_proto_register.h"
#include "modules/local_mapping/app/local_mapping.h"
namespace hozon {
namespace mp {
namespace lm {

using adf_lite_Bundle = hozon::netaos::adf_lite::Bundle;

class LocalMappingOnboard : public hozon::netaos::adf_lite::Executor {
 public:
  LocalMappingOnboard() = default;
  ~LocalMappingOnboard() override = default;

  int32_t AlgInit() override;
  void AlgRelease() override;

  // 接收running_mode通道，用于行泊切换
  int32_t OnRunningMode(adf_lite_Bundle* input);

  // 获取dr局部定位信息
  int32_t Onlocalization(adf_lite_Bundle* input);

  // 获取freespace信息
  int32_t OnFreeSpace(adf_lite_Bundle* input);

  // 获取Ins全局定位信息，仅用于可视化
  int32_t OnIns(adf_lite_Bundle* input);

  // 获取相机图像信息，仅用于可视化
  int32_t OnImage(adf_lite_Bundle* input);

  // 获取上游感知模型数据（包含障碍物、静态元素等）
  int32_t OnPerception(adf_lite_Bundle* input);
  // freespace数据映射到measurement_frame统一处理
  bool FillFreespaceData(
      const std::shared_ptr<MeasurementFrame>& measure_frame);

  // 发送车道线后处理结果给到第三方(如果需要)
  int32_t PublishLaneLine();

  // 发送建图结果给到下游模块
  int32_t PublishLocalMap();

 private:
  std::shared_ptr<LocalMapApp> app_ptr_ = nullptr;  // 整个localmap逻辑处理类

  double frame_proc_maxtime_ = 0.0;
  int frame_proc_num = 0;
  int frame_overtime_nums = 0;
};

REGISTER_ADF_CLASS(LocalMappingOnboard, LocalMappingOnboard);

}  // namespace lm
}  // namespace mp
}  // namespace hozon
