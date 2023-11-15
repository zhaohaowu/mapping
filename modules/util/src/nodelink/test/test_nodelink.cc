/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： test_nodelink.cc
 *   author     ： taoshaoyuan
 *   date       ： 2023.01
 ******************************************************************************/

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <zmq/zmq.h>

#include <fstream>

#include "util/nodelink/core.h"
#include "util/nodelink/deploy.h"

using namespace hozon::mp;             // NOLINT
using namespace std::chrono_literals;  // NOLINT

TEST(TestPubWorker, simple) {
  void* ctx = Context::Instance().Get();
  std::string inproc_addr = "inproc://TestPubWorker";
  std::string tcp_addr = "tcp://127.0.0.1:9100";

  std::vector<std::string> addrs = {inproc_addr, tcp_addr};

  auto pub_worker = PubWorker::Create(ctx, addrs);

  int ret = pub_worker->Init();
  EXPECT_EQ(ret, 0);

  std::vector<std::string> topics;
  std::vector<std::string> contents;
  for (int i = 0; i < 100; ++i) {
    std::string topic = std::string("Topic_") + std::to_string(i);
    topics.push_back(topic);
    std::string dt = std::string("Data_") + std::to_string(i);
    contents.push_back(dt);
  }

  auto pub_a = Publisher::Create("A", pub_worker);
  auto pub_b = Publisher::Create("B", pub_worker);

  std::vector<std::string> contents_a;
  std::vector<std::string> contents_b;
  for (int i = 0; i < 50; ++i) {
    std::string da = std::string("Data_A_") + std::to_string(i);
    std::string db = std::string("Data_B_") + std::to_string(i);
    contents_a.push_back(da);
    contents_b.push_back(db);
  }

  std::vector<std::string> expected_topics(topics);
  std::vector<std::string> expected_contents(contents);
  expected_topics.insert(expected_topics.end(), contents_a.size(), "A");
  expected_topics.insert(expected_topics.end(), contents_b.size(), "B");
  expected_contents.insert(expected_contents.end(), contents_a.begin(),
                           contents_a.end());
  expected_contents.insert(expected_contents.end(), contents_b.begin(),
                           contents_b.end());

  std::map<std::string, std::vector<std::string>> recv_topics;
  std::map<std::string, std::vector<std::string>> recv_contents;

  const std::string kStopAddr = "inproc://stop_test_pub_worker";
  const std::string kStopTopic = "Topic_stop_test_pub_worker";
  const std::string kStopContent = "";
  void* stop_skt = zmq_socket(ctx, ZMQ_PUB);
  ret = zmq_bind(stop_skt, kStopAddr.c_str());
  EXPECT_EQ(ret, 0);

  std::vector<std::shared_ptr<std::thread>> recv_th;
  for (const auto& addr : addrs) {
    auto th = std::make_shared<std::thread>([&] {
      void* sub_skt = zmq_socket(ctx, ZMQ_SUB);
      int ret = zmq_connect(sub_skt, addr.c_str());
      EXPECT_EQ(ret, 0);
      ret = zmq_connect(sub_skt, kStopAddr.c_str());
      EXPECT_EQ(ret, 0);
      ret = zmq_setsockopt(sub_skt, ZMQ_SUBSCRIBE, "", 0);
      EXPECT_EQ(ret, 0);

      while (true) {
        bool need_stop = false;
        zmq_msg_t recv_topic;
        zmq_msg_init(&recv_topic);
        ret = zmq_msg_recv(&recv_topic, sub_skt, 0);
        EXPECT_NE(ret, -1);

        int more = zmq_msg_more(&recv_topic);
        EXPECT_EQ(more, 1);

        void* data = zmq_msg_data(&recv_topic);
        size_t data_size = zmq_msg_size(&recv_topic);
        std::string topic((const char*)data, data_size);
        zmq_msg_close(&recv_topic);
        HLOG_INFO << "From " << addr << " recv topic: " << topic;
        if (topic == kStopTopic) {
          need_stop = true;
        }

        zmq_msg_t recv_content;
        zmq_msg_init(&recv_content);

        ret = zmq_msg_recv(&recv_content, sub_skt, 0);
        EXPECT_NE(ret, -1);

        more = zmq_msg_more(&recv_content);
        EXPECT_EQ(more, 0);

        data = zmq_msg_data(&recv_content);
        data_size = zmq_msg_size(&recv_content);
        std::string content((const char*)data, data_size);
        HLOG_INFO << "From " << addr << " recv content: " << content;
        zmq_msg_close(&recv_content);

        if (!need_stop) {
          recv_topics[addr].push_back(topic);
          recv_contents[addr].push_back(content);
        } else {
          break;
        }
      }
      zmq_close(sub_skt);
    });

    recv_th.push_back(th);
  }

  std::this_thread::sleep_for(1s);

  for (int i = 0; i < topics.size(); ++i) {
    const std::string& topic = topics[i];
    const std::string& content = contents[i];

    pub_worker->AddData(topic, reinterpret_cast<void*>(content.data()),
                        content.size());
  }

  for (const auto& ct : contents_a) {
    pub_a->Pub(reinterpret_cast<void*>(ct.c_str()), ct.size());
  }
  for (const auto& ct : contents_b) {
    pub_b->Pub(reinterpret_cast<void*>(ct.c_str()), ct.size());
  }

  std::this_thread::sleep_for(5s);

  ret =
      zmq_send(stop_skt, kStopTopic.c_str(), kStopTopic.length(), ZMQ_SNDMORE);
  EXPECT_NE(ret, -1);
  ret = zmq_send(stop_skt, kStopContent.c_str(), kStopContent.length(), 0);
  EXPECT_NE(ret, -1);

  for (const auto& th : recv_th) {
    if (th->joinable()) {
      th->join();
    }
  }

  for (const auto& addr : addrs) {
    const auto& recv_t = recv_topics[addr];
    const auto& recv_c = recv_contents[addr];

    EXPECT_EQ(recv_t.size(), expected_topics.size());
    EXPECT_EQ(recv_t, expected_topics);
    EXPECT_EQ(recv_c.size(), expected_contents.size());
    EXPECT_EQ(recv_c, expected_contents);
  }

  zmq_close(stop_skt);
  stop_skt = nullptr;
  pub_worker->Term();
}
#if 0
TEST(TestSubWorker, simple) {
  void* ctx = Context::Instance().Get();
  std::string inproc_addr = "inproc://TestSubWorker";
  std::string tcp_addr = "tcp://127.0.0.1:9100";

  std::vector<std::string> addrs = {
      inproc_addr,
      tcp_addr
  };

  auto sub_worker = SubWorker::Create(ctx, addrs);

  std::string topic_a = "A";
  std::string topic_b = "B";
  std::vector<std::string> contents_a;
  std::vector<std::string> contents_b;
  for (int i = 0; i < 100; ++i) {
    std::string da = "Data_" + topic_a + "_" + std::to_string(i);
    contents_a.push_back(da);
    std::string db = "Data_" + topic_b + "_" + std::to_string(i);
    contents_b.push_back(db);
  }
  std::vector<std::string> all_topics;
  std::vector<std::string> all_contents;
  for (const auto& it : contents_a) {
    all_contents.push_back(it);
    all_topics.push_back(topic_a);
  }
  for (const auto& it : contents_b) {
    all_contents.push_back(it);
    all_topics.push_back(topic_b);
  }

  std::vector<std::string> recv_topics;
  std::vector<std::string> recv_a_contents;
  sub_worker->Reg(topic_a, [&] (void* data, size_t size) {
    std::string content((const char*)data, size);
    HLOG_INFO << "recv topic " << topic_a << " " << content;
    recv_a_contents.push_back(content);
  });

  std::vector<std::string> recv_b_contents;
  sub_worker->Reg(topic_b, [&] (void* data, size_t size) {
    std::string content((const char*)data, size);
    HLOG_INFO << "recv topic " << topic_b << " " << content;
    recv_b_contents.push_back(content);
  });

  int ret = sub_worker->Init();
  EXPECT_EQ(ret, 0);

  //! 这里为什么也必须要等一下？否则tcp的消息会收不到.
  std::this_thread::sleep_for(100ms);

  std::vector<std::shared_ptr<std::thread>> pub_th;
  for (const auto& addr : addrs) {
    auto th = std::make_shared<std::thread>([&] {
      void* pub_skt = zmq_socket(ctx, ZMQ_PUB);
      int ret = zmq_bind(pub_skt, addr.c_str());
      EXPECT_EQ(ret, 0);
      //! zmq_bind()后socket处于mute state，此时pub的消息都会丢，因此
      //! 这里必须先等一等，等连接建立
      std::this_thread::sleep_for(100ms);

      for (int i = 0; i < all_contents.size(); ++i) {
        std::string topic = all_topics[i];
        // 加个addr前缀，用来在接收处区分
        std::string content = addr + all_contents[i];
        ret = zmq_send(pub_skt, topic.c_str(), topic.length(), ZMQ_SNDMORE);
        EXPECT_NE(ret, -1);

        ret = zmq_send(pub_skt, content.c_str(), content.length(), 0);
        EXPECT_NE(ret, -1);

        HLOG_INFO << "send " << topic << " " << content;
      }

//      std::this_thread::sleep_for(1s);
      zmq_close(pub_skt);
      HLOG_INFO << "close " << addr;
    });
    pub_th.push_back(th);
  }

  std::this_thread::sleep_for(3s);

  for (const auto& th : pub_th) {
    if (th->joinable()) {
      th->join();
    }
  }

  std::map<std::string, std::vector<std::string>> extracted_recv_a_contents;
  std::map<std::string, std::vector<std::string>> extracted_recv_b_contents;
  for (const auto& c : recv_a_contents) {
    for (const auto& addr : addrs) {
      if (c.rfind(addr, 0) == 0) {
        std::string remove_addr = c.substr(addr.length());
        extracted_recv_a_contents[addr].push_back(remove_addr);
      }
    }
  }
  for (const auto& c : recv_b_contents) {
    for (const auto& addr : addrs) {
      if (c.rfind(addr, 0) == 0) {
        std::string remove_addr = c.substr(addr.length());
        extracted_recv_b_contents[addr].push_back(remove_addr);
      }
    }
  }

  size_t extracted_size_a = 0;
  for (const auto& e : extracted_recv_a_contents) {
    extracted_size_a += e.second.size();
  }
  EXPECT_EQ(extracted_size_a, contents_a.size() * addrs.size());

  for (const auto& addr : addrs) {
    EXPECT_EQ(extracted_recv_a_contents[addr], contents_a);
  }

  size_t extracted_size_b = 0;
  for (const auto& e : extracted_recv_b_contents) {
    extracted_size_b += e.second.size();
  }
  EXPECT_EQ(extracted_size_b, contents_b.size() * addrs.size());

  for (const auto& addr : addrs) {
    EXPECT_EQ(extracted_recv_b_contents[addr], contents_b);
  }

  sub_worker->Term();
}
#endif
TEST(TestLinker, simple) {
  void* ctx = Context::Instance().Get();

  auto linker = Linker::Create(ctx);

  const std::string frontend_addr_inproc = "inproc://frontend_addr_inproc";
  const std::string frontend_addr_tcp = "tcp://127.0.0.1:9100";
  const std::string backend_addr_inproc = "inproc://backend_addr_inproc";
  const std::string backend_addr_tcp = "tcp://127.0.0.1:9101";

  linker->LinkFrontend(frontend_addr_inproc);
  linker->LinkFrontend(frontend_addr_tcp);
  linker->LinkBackend(backend_addr_inproc);
  linker->LinkBackend(backend_addr_tcp);

  linker->Start();
  std::this_thread::sleep_for(100ms);

  const std::string kStopAddr = "inproc://stop_test_pub_worker";
  const std::string kStopTopic = "Topic_stop_test_pub_worker";
  const std::string kStopContent = "";
  void* stop_skt = zmq_socket(ctx, ZMQ_PUB);
  int ret = zmq_bind(stop_skt, kStopAddr.c_str());
  EXPECT_EQ(ret, 0);

  std::vector<std::string> topics_a;
  std::vector<std::string> contents_a;
  std::vector<std::string> topics_b;
  std::vector<std::string> contents_b;

  for (int i = 0; i < 10; ++i) {
    topics_a.push_back("A");
    contents_a.push_back("A_" + std::to_string(i));
    topics_b.push_back("B");
    contents_b.push_back("B_" + std::to_string(i));
  }

  std::vector<std::string> all_topics;
  std::vector<std::string> all_contents;
  all_topics.insert(all_topics.end(), topics_a.begin(), topics_a.end());
  all_contents.insert(all_contents.end(), contents_a.begin(), contents_a.end());
  all_topics.insert(all_topics.end(), topics_b.begin(), topics_b.end());
  all_contents.insert(all_contents.end(), contents_b.begin(), contents_b.end());

  std::vector<std::shared_ptr<std::thread>> ths;

  auto frontend_th_inproc = std::make_shared<std::thread>([&] {
    std::vector<std::string> topics = topics_a;
    std::vector<std::string> contents = contents_a;
    const std::string& addr = frontend_addr_inproc;

    void* skt = zmq_socket(ctx, ZMQ_PUB);
    int ret = zmq_bind(skt, addr.c_str());
    EXPECT_EQ(ret, 0);

    std::this_thread::sleep_for(100ms);

    for (int i = 0; i < topics.size(); ++i) {
      std::string topic = topics[i];
      std::string content = contents[i];

      ret = zmq_send(skt, topic.c_str(), topic.length(), ZMQ_SNDMORE);
      EXPECT_NE(ret, -1);

      ret = zmq_send(skt, content.c_str(), content.length(), 0);
      EXPECT_NE(ret, -1);

      HLOG_INFO << "send inproc " << topic << " " << content;
    }

    zmq_close(skt);
  });
  ths.push_back(frontend_th_inproc);

  auto frontend_th_tcp = std::make_shared<std::thread>([&] {
    std::vector<std::string> topics = topics_b;
    std::vector<std::string> contents = contents_b;
    const std::string& addr = frontend_addr_tcp;

    void* skt = zmq_socket(ctx, ZMQ_PUB);
    int ret = zmq_bind(skt, addr.c_str());
    EXPECT_EQ(ret, 0);

    std::this_thread::sleep_for(100ms);

    for (int i = 0; i < topics.size(); ++i) {
      std::string topic = topics[i];
      std::string content = contents[i];

      ret = zmq_send(skt, topic.c_str(), topic.length(), ZMQ_SNDMORE);
      EXPECT_NE(ret, -1);

      ret = zmq_send(skt, content.c_str(), content.length(), 0);
      EXPECT_NE(ret, -1);

      HLOG_INFO << "send tcp " << topic << " " << content;
    }

    zmq_close(skt);
  });
  ths.push_back(frontend_th_tcp);

  auto backend_th_inproc = std::make_shared<std::thread>([&] {
    std::vector<std::string> topics = all_topics;
    const std::string& addr = backend_addr_inproc;

    void* skt = zmq_socket(ctx, ZMQ_SUB);
    int ret = zmq_connect(skt, kStopAddr.c_str());
    EXPECT_EQ(ret, 0);
    ret = zmq_connect(skt, addr.c_str());
    EXPECT_EQ(ret, 0);

    ret = zmq_setsockopt(skt, ZMQ_SUBSCRIBE, kStopTopic.c_str(),
                         kStopTopic.length());
    EXPECT_EQ(ret, 0);
    ret = zmq_setsockopt(skt, ZMQ_SUBSCRIBE, "A", 1);
    EXPECT_EQ(ret, 0);
    ret = zmq_setsockopt(skt, ZMQ_SUBSCRIBE, "B", 1);
    EXPECT_EQ(ret, 0);

    while (true) {
      bool need_stop = false;
      zmq_msg_t recv_topic;
      zmq_msg_init(&recv_topic);

      ret = zmq_msg_recv(&recv_topic, skt, 0);
      EXPECT_NE(ret, -1);

      void* data = zmq_msg_data(&recv_topic);
      size_t size = zmq_msg_size(&recv_topic);
      std::string topic((const char*)data, size);

      int more = zmq_msg_more(&recv_topic);
      EXPECT_EQ(more, 1);

      if (topic == kStopTopic) {
        need_stop = true;
      }

      zmq_msg_close(&recv_topic);

      zmq_msg_t recv_content;
      zmq_msg_init(&recv_content);

      ret = zmq_msg_recv(&recv_content, skt, 0);
      EXPECT_NE(ret, -1);

      data = zmq_msg_data(&recv_content);
      size = zmq_msg_size(&recv_content);
      std::string content((const char*)data, size);

      more = zmq_msg_more(&recv_content);
      EXPECT_EQ(more, 0);

      zmq_msg_close(&recv_content);

      HLOG_INFO << "From " << addr << " " << topic << " " << content;

      if (need_stop) {
        break;
      }
    }
    zmq_close(skt);
  });
  ths.push_back(backend_th_inproc);

  auto backend_th_tcp = std::make_shared<std::thread>([&] {
    std::vector<std::string> topics = topics_a;
    const std::string& addr = backend_addr_tcp;

    void* skt = zmq_socket(ctx, ZMQ_SUB);
    int ret = zmq_connect(skt, kStopAddr.c_str());
    EXPECT_EQ(ret, 0);
    ret = zmq_connect(skt, addr.c_str());
    EXPECT_EQ(ret, 0);

    ret = zmq_setsockopt(skt, ZMQ_SUBSCRIBE, kStopTopic.c_str(),
                         kStopTopic.length());
    EXPECT_EQ(ret, 0);
    ret = zmq_setsockopt(skt, ZMQ_SUBSCRIBE, "A", 1);
    EXPECT_EQ(ret, 0);
    //    ret = zmq_setsockopt(skt, ZMQ_SUBSCRIBE, "B", 1);
    //    EXPECT_EQ(ret, 0);

    while (true) {
      bool need_stop = false;
      zmq_msg_t recv_topic;
      zmq_msg_init(&recv_topic);

      ret = zmq_msg_recv(&recv_topic, skt, 0);
      EXPECT_NE(ret, -1);

      void* data = zmq_msg_data(&recv_topic);
      size_t size = zmq_msg_size(&recv_topic);
      std::string topic((const char*)data, size);

      int more = zmq_msg_more(&recv_topic);
      EXPECT_EQ(more, 1);

      if (topic == kStopTopic) {
        need_stop = true;
      }

      zmq_msg_close(&recv_topic);

      zmq_msg_t recv_content;
      zmq_msg_init(&recv_content);

      ret = zmq_msg_recv(&recv_content, skt, 0);
      EXPECT_NE(ret, -1);

      data = zmq_msg_data(&recv_content);
      size = zmq_msg_size(&recv_content);
      std::string content((const char*)data, size);

      more = zmq_msg_more(&recv_content);
      EXPECT_EQ(more, 0);

      zmq_msg_close(&recv_content);

      HLOG_INFO << "From " << addr << " " << topic << " " << content;

      if (need_stop) {
        break;
      }
    }
    zmq_close(skt);
  });
  ths.push_back(backend_th_tcp);

  std::this_thread::sleep_for(500ms);

  ret =
      zmq_send(stop_skt, kStopTopic.c_str(), kStopTopic.length(), ZMQ_SNDMORE);
  EXPECT_NE(ret, -1);
  ret = zmq_send(stop_skt, kStopContent.c_str(), kStopContent.length(), 0);
  EXPECT_NE(ret, -1);

  for (const auto& th : ths) {
    if (th->joinable()) {
      th->join();
    }
  }

  zmq_close(stop_skt);
  stop_skt = nullptr;
  linker->Term();
}

//! 部署流程
// auto node_a = std::make_shared<NodeA>();
// auto node_b = std::make_shared<NodeB>();
// node_a->Initialize(node_a_config); // Advertise() and Subscribe() in
// NodeA::Init() node_b->Initialize(node_b_config); // Advertise() and
// Subscribe() in NodeB::Init() auto linker = Linker::Create(ctx);
// linker->LinkNode(node_a);
// linker->LinkNode(node_b);
// linker->Start();
// node_a->Start();
// node_b->Start();
// ...
// node_a->Term();
// node_b->Term();
// linker->Term();

std::vector<std::string> sent_a;
std::vector<std::string> sent_b;
std::vector<std::string> recv_c;
std::vector<std::string> recv_d;
std::vector<std::string> sent_c;
std::vector<std::string> sent_d;
std::vector<std::string> recv_a;
std::vector<std::string> recv_b;

class NodeA : public Node {
 public:
  NodeA() = default;
  ~NodeA() override = default;

  int Init(const std::string& config) override {
    sent_a.clear();
    sent_b.clear();
    recv_c.clear();
    recv_d.clear();

    pub_a_ = Advertise("A");
    pub_b_ = Advertise("B");

    auto cbk_c = std::bind(&NodeA::OnC, this, std::placeholders::_1,
                           std::placeholders::_2);
    Subscribe("C", cbk_c);

    auto cbk_d = std::bind(&NodeA::OnD, this, std::placeholders::_1,
                           std::placeholders::_2);
    Subscribe("D", cbk_d);

    return 0;
  }

  int Start() override {
    th_ = std::make_shared<std::thread>(&NodeA::Loop, this);
    return 0;
  }

  void Stop() override {
    if (th_ && th_->joinable()) {
      th_->join();
    }
  }

 private:
  void OnC(void* data, size_t size) {
    std::string msg((const char*)data, size);
    std::cout << "OnC " << msg << std::endl;
    recv_c.push_back(msg);
  }

  void OnD(void* data, size_t size) {
    std::string msg((const char*)data, size);
    std::cout << "OnD " << msg << std::endl;
    recv_d.push_back(msg);
  }

  void Loop() {
    for (int i = 0; i < 10; ++i) {
      std::string data_a("Data_A_" + std::to_string(i));
      pub_a_->Pub(reinterpret_cast<void*>(data_a.c_str()), data_a.length());
      sent_a.push_back(data_a);

      std::string data_b("Data_B_" + std::to_string(i));
      pub_b_->Pub(reinterpret_cast<void*>(data_b.c_str()), data_b.length());
      sent_b.push_back(data_b);
    }
  }

 private:
  PublisherPtr pub_a_ = nullptr;
  PublisherPtr pub_b_ = nullptr;
  std::shared_ptr<std::thread> th_ = nullptr;
};

REGISTER(NodeA);

class NodeB : public Node {
 public:
  NodeB() = default;
  ~NodeB() override = default;

  int Init(const std::string& config) override {
    sent_c.clear();
    sent_d.clear();
    recv_a.clear();
    recv_b.clear();

    pub_c_ = Advertise("C");
    pub_d_ = Advertise("D");

    auto cbk_a = std::bind(&NodeB::OnA, this, std::placeholders::_1,
                           std::placeholders::_2);
    Subscribe("A", cbk_a);

    auto cbk_b = std::bind(&NodeB::OnB, this, std::placeholders::_1,
                           std::placeholders::_2);
    Subscribe("B", cbk_b);

    return 0;
  }

  int Start() override {
    th_ = std::make_shared<std::thread>(&NodeB::Loop, this);
    return 0;
  }

  void Stop() override {
    if (th_ && th_->joinable()) {
      th_->join();
    }
  }

 private:
  void OnA(void* data, size_t size) {
    std::string msg((const char*)data, size);
    std::cout << "OnA " << msg << std::endl;
    recv_a.push_back(msg);
  }

  void OnB(void* data, size_t size) {
    std::string msg((const char*)data, size);
    std::cout << "OnB " << msg << std::endl;
    recv_b.push_back(msg);
  }

  void Loop() {
    for (int i = 0; i < 10; ++i) {
      std::string data_c("Data_C_" + std::to_string(i));
      pub_c_->Pub(reinterpret_cast<void*>(data_c.c_str()), data_c.length());
      sent_c.push_back(data_c);

      std::string data_d("Data_D_" + std::to_string(i));
      pub_d_->Pub(reinterpret_cast<void*>(data_d.c_str()), data_d.length());
      sent_d.push_back(data_d);
    }
  }

 private:
  PublisherPtr pub_c_ = nullptr;
  PublisherPtr pub_d_ = nullptr;
  std::shared_ptr<std::thread> th_ = nullptr;
};

REGISTER(NodeB);

// TEST(TestNode, simple) {
//   void* ctx = Context::Instance().Get();

//   auto node_a = std::make_shared<NodeA>();
//   auto node_b = std::make_shared<NodeB>();
//   int ret = node_a->InitNode("");
//   EXPECT_EQ(ret, 0);
//   ret = node_b->InitNode("");
//   EXPECT_EQ(ret, 0);

//   auto linker = Linker::Create(ctx);
//   ret = linker->LinkNode(node_a);
//   EXPECT_EQ(ret, 0);
//   ret = linker->LinkNode(node_b);
//   EXPECT_EQ(ret, 0);
//   ret = linker->Start();
//   EXPECT_EQ(ret, 0);

//   ret = node_a->StartNode();
//   EXPECT_EQ(ret, 0);

//   ret = node_b->StartNode();
//   EXPECT_EQ(ret, 0);

//   std::this_thread::sleep_for(5s);

//   EXPECT_EQ(sent_a.size(), recv_a.size());
//   EXPECT_EQ(sent_b.size(), recv_b.size());
//   EXPECT_EQ(recv_c.size(), sent_c.size());
//   EXPECT_EQ(recv_d.size(), sent_d.size());

//   for (int i = 0; i < sent_a.size(); ++i) {
//     EXPECT_EQ(sent_a[i], recv_a[i]);
//   }

//   for (int i = 0; i < sent_b.size(); ++i) {
//     EXPECT_EQ(sent_b[i], recv_b[i]);
//   }

//   for (int i = 0; i < recv_c.size(); ++i) {
//     EXPECT_EQ(recv_c[i], sent_c[i]);
//   }

//   for (int i = 0; i < recv_d.size(); ++i) {
//     EXPECT_EQ(recv_d[i], sent_d[i]);
//   }

//   node_a->Term();

//   node_b->Term();

//   linker->Term();
// }

// TEST(TestFactory, simple) {
//  auto node = Factory::Instance().CreateNodeByName("FooNode");
//  node->InitNode("");
//}

/*
 * topic: Foo
 * msg: foo
 * cnt: 100
 */

namespace localization_test {

std::vector<std::string> g_pub_msgs;
std::vector<std::string> g_sub_msgs;

class FooNode : public Node {
 public:
  FooNode() = default;
  ~FooNode() override = default;

  int Init(const std::string& config) override {
    YAML::Node root;
    try {
      root = YAML::LoadFile(config);
    } catch (YAML::ParserException& ex) {
      return -1;
    } catch (YAML::BadFile& ex) {
      return -1;
    }
    topic_ = root["topic"].as<std::string>();
    msg_ = root["msg"].as<std::string>();
    cnt_ = root["cnt"].as<int>();

    pub_ = Advertise(topic_);
    return 0;
  }

  int Start() override {
    th_ = std::make_shared<std::thread>(&FooNode::RunPub, this);
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
      g_pub_msgs.push_back(msg_);
      std::this_thread::sleep_for(10ms);
    }
  }

  std::string topic_;
  std::string msg_;
  int cnt_ = 0;
  PublisherPtr pub_ = nullptr;
  std::shared_ptr<std::thread> th_ = nullptr;
};

REGISTER(FooNode);

class BarNode : public Node {
 public:
  BarNode() = default;
  ~BarNode() override = default;

  int Init(const std::string& config) override {
    YAML::Node root;
    try {
      root = YAML::LoadFile(config);
    } catch (YAML::ParserException& ex) {
      return -1;
    } catch (YAML::BadFile& ex) {
      return -1;
    }
    topic_ = root["topic"].as<std::string>();
    auto cbk = std::bind(&BarNode::OnMsg, this, std::placeholders::_1,
                         std::placeholders::_2);
    Subscribe(topic_, cbk);

    return 0;
  }

 private:
  void OnMsg(void* data, size_t size) {
    std::string msg((const char*)data, size);
    g_sub_msgs.push_back(msg);
  }

  std::string topic_;
};

REGISTER(BarNode);

}  // namespace localization_test

TEST(TestDeployer, no_load) {
  localization_test::g_pub_msgs.clear();
  localization_test::g_sub_msgs.clear();

  // create config yaml
  const std::string conf_path = "/tmp/test_deployer_no_load_config.yaml";
  const std::string topic = "Foo";
  const std::string msg = "foo";
  const int cnt = 100;

  std::ofstream conf_file;
  conf_file.open(conf_path);
  conf_file << "%YAML:1.0\n";
  conf_file << "topic: " << topic << std::endl;
  conf_file << "msg: " << msg << std::endl;
  conf_file << "cnt: " << cnt << std::endl;
  conf_file.close();

  // create deploy yaml
  const std::string deploy_path = "/tmp/test_deployer_no_load_deploy.yaml";
  std::ofstream deploy_file;
  deploy_file.open(deploy_path);
  deploy_file << "%YAML:1.0\n";
  deploy_file << "nodes:\n";
  deploy_file << "  - name: FooNode\n";
  deploy_file << "    conf: " << conf_path << std::endl;
  deploy_file << "  - name: BarNode\n";
  deploy_file << "    conf: " << conf_path << std::endl;
  deploy_file.close();

  auto deployer = Deployer::Create();
  int ret = deployer->Deploy(deploy_path, false);
  EXPECT_EQ(ret, 0);
  std::this_thread::sleep_for(2s);
  const std::string topo_path = "/tmp/test_deployer_no_load_topo.md";
  deployer->TopoToMd(topo_path);
  deployer->Term();

  std::vector<std::string> expected_msgs(cnt, msg);
  EXPECT_EQ(expected_msgs, localization_test::g_pub_msgs);
  EXPECT_EQ(expected_msgs, localization_test::g_sub_msgs);
}

TEST(TestDeployer, load) {
  // create config yaml
  const std::string conf_path = "/tmp/test_deployer_load_config.yaml";
  const std::string topic_from_foo = "Foo";
  const std::string topic_from_bar = "Bar";
  const std::string msg = "foo";
  const int cnt = 100;

  std::ofstream conf_file;
  conf_file.open(conf_path);
  conf_file << "%YAML:1.0\n";
  conf_file << "topic_from_foo: " << topic_from_foo << std::endl;
  conf_file << "topic_from_bar: " << topic_from_bar << std::endl;
  conf_file << "msg: " << msg << std::endl;
  conf_file << "cnt: " << cnt << std::endl;
  conf_file.close();

  // create deploy yaml
  const std::string deploy_path = "/tmp/test_deployer_load_deploy.yaml";
//! 这个宏在cmakelists里定义
#ifndef BUILD_DIR
#define BUILD_DIR "build"
#endif
  const std::string lib_path =
      std::string(BUILD_DIR) +
      "/modules/road_mapping/lib/liblocalization_node_for_tests.so";
  const std::string local_sub_addr = "ipc:///tmp/test_deployer_load_sub";
  std::ofstream deploy_file;
  deploy_file.open(deploy_path);
  deploy_file << "%YAML:1.0\n";
  deploy_file << "nodes:\n";
  deploy_file << "  - name: IndFooNode\n";
  deploy_file << "    conf: " << conf_path << std::endl;
  deploy_file << "    lib: " << lib_path << std::endl;
  deploy_file << "  - name: IndBarNode\n";
  deploy_file << "    conf: " << conf_path << std::endl;
  deploy_file << "    lib: " << lib_path << std::endl;
  deploy_file << "pub_to_external: [" << std::endl;
  deploy_file << "  " << local_sub_addr << std::endl;
  deploy_file << "]" << std::endl;
  deploy_file.close();

  // create local sub
  void* ctx = Context::Instance().Get();
  std::vector<std::string> sub_addr = {local_sub_addr};

  std::vector<std::string> local_sub_msgs;
  auto local_sub = SubWorker::Create(ctx, sub_addr);
  local_sub->Reg(topic_from_bar, [&](void* data, size_t size) {
    std::string msg((const char*)data, size);
    local_sub_msgs.push_back(msg);
  });

  int ret = local_sub->Init();
  EXPECT_EQ(ret, 0);
  local_sub->Start();

  auto deployer = Deployer::Create();
  ret = deployer->Deploy(deploy_path, true);
  EXPECT_EQ(ret, 0);
  std::this_thread::sleep_for(2s);
  const std::string topo_path = "/tmp/test_deployer_load_topo.md";
  deployer->TopoToMd(topo_path);
  deployer->Term();

  local_sub->Term();

  std::vector<std::string> expected_msgs(cnt, msg);
  EXPECT_EQ(local_sub_msgs, expected_msgs);
}
