/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： node_for_tests.cc
 *   author     ： taoshaoyuan
 *   date       ： 2023.01
 ******************************************************************************/

#include <yaml-cpp/yaml.h>

#include "util/nodelink/core.h"
#include "util/nodelink/deploy.h"

namespace hozon {
namespace mp {

using namespace std::chrono_literals; // NOLINT

/*
 * topic_from_foo: Foo
 * topic_from_bar: Bar
 * msg: foo
 * cnt: 100
 */

class IndFooNode : public Node {
 public:
  IndFooNode() = default;
  ~IndFooNode() override = default;

  int Init(const std::string& config) override {
    YAML::Node root;
    try {
      root = YAML::LoadFile(config);
    } catch (YAML::ParserException& ex) {
      return -1;
    } catch (YAML::BadFile& ex) {
      return -1;
    }

    topic_ = root["topic_from_foo"].as<std::string>();
    msg_ = root["msg"].as<std::string>();
    cnt_ = root["cnt"].as<int>();

    pub_ = Advertise(topic_);
    return 0;
  }

  int Start() override {
    th_ = std::make_shared<std::thread>(&IndFooNode::RunPub, this);
    return 0;
  }

  void Stop() override {
    if (th_ && th_->joinable()) {
      th_->join();
    }
  }

 private:
  void RunPub() {
    if (!pub_) return;

    for (int i = 0; i != cnt_; ++i) {
      pub_->Pub(reinterpret_cast<void*>(msg_.data()), msg_.size());

      std::this_thread::sleep_for(10ms);
    }
  }

  std::string topic_;
  std::string msg_;
  int cnt_ = 0;
  PublisherPtr pub_ = nullptr;
  std::shared_ptr<std::thread> th_ = nullptr;
};

REGISTER(IndFooNode);

class IndBarNode : public Node {
 public:
  IndBarNode() = default;
  ~IndBarNode() override = default;

  int Init(const std::string& config) override {
    YAML::Node root;
    try {
      root = YAML::LoadFile(config);
    } catch (YAML::ParserException& ex) {
      return -1;
    } catch (YAML::BadFile& ex) {
      return -1;
    }
    topic_sub_ = root["topic_from_foo"].as<std::string>();
    topic_pub_ = root["topic_from_bar"].as<std::string>();

    pub_ = Advertise(topic_pub_);

    auto cbk = std::bind(&IndBarNode::OnMsg, this, std::placeholders::_1,
                         std::placeholders::_2);
    Subscribe(topic_sub_, cbk);

    return 0;
  }

 private:
  void OnMsg(void* data, size_t size) {
    if (!pub_) return;

    pub_->Pub(data, size);
  }

  std::string topic_sub_;
  std::string topic_pub_;
  PublisherPtr pub_ = nullptr;
};

REGISTER(IndBarNode);

}  // namespace mp
}  // namespace hozon
