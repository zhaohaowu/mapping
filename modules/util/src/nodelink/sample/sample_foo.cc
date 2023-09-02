/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： sample_foo.cc
 *   author     ： taoshaoyuan
 *   date       ： 2023.01
 ******************************************************************************/

#include "sample_foo.h"

#include <yaml-cpp/yaml.h>

#include <iostream>

#include "sample_data.h"

namespace hozon {
namespace mp {

int SampleFoo::Init(const std::string& config) {
  //! Parse config file if needed
  YAML::Node root;
  try {
    root = YAML::LoadFile(config);
  } catch (YAML::ParserException& ex) {
    std::cout << "parse yaml " << config << " failed: " << ex.what();
    return -1;
  } catch (YAML::BadFile& ex) {
    std::cout << "load yaml " << config << " failed: " << ex.what();
    return -1;
  }
  int reserved = root["reserved"].as<int>();
  std::cout << "reserved: " << reserved << std::endl;

  const std::string pub_topic = "Foo";
  pub_foo_ = Advertise(pub_topic);

  const std::string sub_topic = "Bar";
  auto callback = std::bind(&SampleFoo::OnBar, this, std::placeholders::_1,
                            std::placeholders::_2);
  Subscribe(sub_topic, callback);

  return 0;
}

int SampleFoo::Start() {
  th_ = std::make_shared<std::thread>(&SampleFoo::Proc, this);
  return 0;
}

void SampleFoo::Stop() {
  if (th_ && th_->joinable()) {
    th_->join();
  }
}

void SampleFoo::OnBar(void* data, size_t size) {
  if (size != sizeof(DataBar)) {
    std::cout << "Error: wrong size, maybe data is mismatched";
    return;
  }
  auto* bar = (DataBar*)data;
  std::cout << "OnBar " << bar->bar << std::endl;
}

void SampleFoo::Proc() {
  for (int i = 0; i < 10; ++i) {
    DataFoo foo;
    foo.foo = i;
    pub_foo_->Pub(&foo, sizeof(foo));
  }
}

}  // namespace mp
}  // namespace hozon