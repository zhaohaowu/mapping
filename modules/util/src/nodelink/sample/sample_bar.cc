/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： sample_bar.cc
 *   author     ： taoshaoyuan
 *   date       ： 2023.01
 ******************************************************************************/

#include "modules/util/src/nodelink/sample/sample_bar.h"

#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "./sample_data.h"

namespace hozon {
namespace mp {

int SampleBar::Init(const std::string& config) {
  //! Parse config file if needed
  // unused config
  (void)(config);

  const std::string pub_topic = "Bar";
  pub_bar_ = Advertise(pub_topic);

  const std::string sub_topic = "Foo";
  auto callback = [](auto&& PH1, auto&& PH2) {
    OnFoo(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
  };
  Subscribe(sub_topic, callback);
  return 0;
}

int SampleBar::Start() {
  th_ = std::make_shared<std::thread>(&SampleBar::Proc, this);
  return 0;
}

void SampleBar::Stop() {
  if (th_ && th_->joinable()) {
    th_->join();
  }
}

void SampleBar::OnFoo(void* data, size_t size) {
  if (size != sizeof(DataFoo)) {
    std::cout << "Error: wrong size, maybe data is mismatched";
    return;
  }
  auto* foo = static_cast<DataFoo*>(data);
  std::cout << "OnFoo " << foo->foo << std::endl;
}

void SampleBar::Proc() {
  for (int i = 0; i < 10; ++i) {
    DataBar bar;
    bar.bar = i;
    pub_bar_->Pub(&bar, sizeof(bar));
  }
}

}  // namespace mp
}  // namespace hozon
