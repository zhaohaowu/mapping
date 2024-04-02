/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： global_lite.h
 *   author     ： taoshaoyuan
 *   date       ： 2024.01
 ******************************************************************************/

#pragma once

#include <adf-lite/include/base.h>
#include <depend/nos/x86_2004/include/adf-lite/include/executor.h>
#include <depend/nos/x86_2004/include/adf/include/node_proto_register.h>

#include <memory>
#include <mutex>
#include <string>

namespace hozon {
namespace mp {
namespace global_lite {

class GlobalLite : public hozon::netaos::adf_lite::Executor {
 public:
  GlobalLite() = default;
  ~GlobalLite() = default;

  int32_t AlgInit() override;
  void AlgRelease() override;

 private:
  int32_t OnImg(hozon::netaos::adf_lite::Bundle* input);
  const std::string kImgLiteTopic = "lm_image";
  const std::string kImgTrigger = "on_front_narrow_img";
  const std::string kImgRosTopic = "/front_narrow_img";
};

REGISTER_ADF_CLASS(GlobalLite, GlobalLite);

}  // namespace global_lite
}  // namespace mp
}  // namespace hozon
