/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： sample_bar.h
 *   author     ： taoshaoyuan
 *   date       ： 2023.01
 ******************************************************************************/

#pragma once

#include <memory>
#include <string>

#include "util/nodelink/core.h"
#include "util/nodelink/deploy.h"

namespace hozon {
namespace mp {

/*
 * SampleBar:
 *   - 发布topic为"Bar"的数据;
 *   - 订阅topic为"Foo"的数据.
 */
class SampleBar : public Node {
 public:
  SampleBar() = default;
  ~SampleBar() override = default;

  int Init(const std::string& config) override;

  int Start() override;

  void Stop() override;

 private:
  static void OnFoo(void* data, size_t size);

  void Proc();

  PublisherPtr pub_bar_ = nullptr;
  std::shared_ptr<std::thread> th_ = nullptr;
};

REGISTER(SampleBar);

}  // namespace mp
}  // namespace hozon
