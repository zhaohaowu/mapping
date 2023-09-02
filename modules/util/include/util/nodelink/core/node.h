/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： node.h
 *   author     ： taoshaoyuan
 *   date       ： 2023.01
 ******************************************************************************/

#pragma once

#include <memory>
#include <set>
#include <string>
#include <vector>

#include "util/nodelink/core/publisher.h"
#include "util/nodelink/core/sub_worker.h"

namespace hozon {
namespace mp {

class SubWorker;
class PubWorker;

class Node {
 public:
  Node() = default;

  virtual ~Node() { Term(); }

  virtual int Init(const std::string& config) = 0;

  virtual int Start() { return 0; }

  virtual void Stop() {}

  int InitNode(const std::string& config);
  int StartNode();

  void Term();

  std::vector<std::string> SubAddrs();

  std::vector<std::string> PubAddrs();

  std::vector<std::string> SubTopics();

  std::vector<std::string> PubTopics();

 protected:
  PublisherPtr Advertise(const std::string& topic);

  void Subscribe(const std::string& topic, SubCallback cbk);

 protected:
  std::shared_ptr<SubWorker> sub_ = nullptr;
  std::shared_ptr<PubWorker> pub_ = nullptr;
  std::set<std::string> sub_topics_;
  std::set<std::string> pub_topics_;
  std::string class_name_;
};

}  // namespace mp
}  // namespace hozon