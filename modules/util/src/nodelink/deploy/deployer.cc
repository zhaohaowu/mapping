/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： address.h
 *   author     ： taoshaoyuan
 *   date       ： 2023.01
 ******************************************************************************/

#include "util/nodelink/deploy/deployer.h"

#include <yaml-cpp/yaml.h>

#include <fstream>

#include "util/nodelink/core/context.h"
#include "util/nodelink/core/linker.h"
#include "util/nodelink/core/node.h"
#include "util/nodelink/deploy/factory.h"
#include "util/nodelink/deploy/loader.h"
#include "util/temp_log.h"

namespace hozon {
namespace mp {

int Deployer::Deploy(const std::string& deploy_file, bool dynamic_load) {
  if (Parse(deploy_file, deployer_config_) < 0) {
    HLOG_ERROR << "parse " << deploy_file << " failed";
    return -1;
  }

  if (dynamic_load) {
    std::vector<std::string> libs;
    for (const auto& n : deployer_config_.nodes) {
      if (!n.lib.empty()) {
        libs.push_back(n.lib);
      }
      //      else {
      //        HLOG_ERROR << "node " << n.name << " not specify lib path";
      //        return -1;
      //      }
    }

    if (InitLoaderAndLoadLibs(libs) < 0) {
      HLOG_ERROR << "InitLoaderAndLoadLibs failed";
      Term();
      return -1;
    }
  }

  if (InitNodes(deployer_config_.nodes) < 0) {
    HLOG_ERROR << "InitNodes failed";
    Term();
    return -1;
  }

  if (InitLinkerAndLinkNodes() < 0) {
    HLOG_ERROR << "InitLinkerAndLinkNodes failed";
    Term();
    return -1;
  }

  if (StartNodes() < 0) {
    HLOG_ERROR << "StartNodes failed";
    Term();
    return -1;
  }

  return 0;
}

void Deployer::Term() {
  for (auto& n : inited_nodes_) {
    if (n.second) {
      HLOG_INFO << "Term " << n.first;
      n.second->Term();
      n.second = nullptr;
    }
  }
  inited_nodes_.clear();

  if (linker_) {
    HLOG_INFO << "Term linker";
    linker_->Term();
    linker_ = nullptr;
  }

  if (loader_) {
    HLOG_INFO << "Term loader";
    loader_->Term();
    loader_ = nullptr;
  }
}

std::shared_ptr<Node> Deployer::GetNode(const std::string& name) {
  if (inited_nodes_.find(name) == inited_nodes_.end()) {
    return nullptr;
  }
  return inited_nodes_[name];
}

std::string Deployer::Topo() {
  struct NodePubSub {
    std::string name;
    std::vector<std::string> pubs;
    std::vector<std::string> subs;
  };

  std::vector<NodePubSub> nodes;
  for (const auto& n : inited_nodes_) {
    if (n.second) {
      auto pubs = n.second->PubTopics();
      auto subs = n.second->SubTopics();

      NodePubSub node = {.name = n.first, .pubs = pubs, .subs = subs};
      nodes.push_back(node);
    }
  }

  auto single_node_mermaid = [](NodePubSub node) {
    std::string output;
    for (const auto& pub : node.pubs) {
      output.append("\n    " + node.name + " --> " + pub + "([" + pub + "])");
    }

    for (const auto& sub : node.subs) {
      output.append("\n    " + sub + "([" + sub + "])" + " --> " + node.name);
    }

    // no pubs and subs, only display node name
    if (output.empty()) {
      output.append("\n    " + node.name);
    }

    return output;
  };

  std::string output;
  output.append("flowchart LR");
  for (const auto& n : nodes) {
    output.append(single_node_mermaid(n));
  }
  return output;
}

void Deployer::TopoToMd(const std::string& md_path) {
  std::string mermaid;
  mermaid.append("\n```mermaid\n");
  std::string topo = Topo();
  mermaid.append(topo);
  mermaid.append("\n```\n");

  std::ofstream md;
  md.open(md_path, std::ios::app);
  md << mermaid;
  md.close();
}

int Deployer::Parse(const std::string& deploy_file,
                    DeployerConfig& deployer_config) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(deploy_file);
  } catch (YAML::ParserException& ex) {
    HLOG_ERROR << "parse yaml " << deploy_file << " failed: " << ex.what();
    return -1;
  } catch (YAML::BadFile& ex) {
    HLOG_ERROR << "load yaml " << deploy_file << " failed: " << ex.what();
    return -1;
  }

  auto nodes =
      root["nodes"].as<std::vector<std::map<std::string, std::string>>>();
  if (nodes.empty()) {
    HLOG_ERROR << "nodes are empty";
    return -1;
  }

  std::set<std::string> already_exist;
  for (auto& node : nodes) {
    if (node.count("name") == 0 || node["name"].empty()) {
      HLOG_ERROR << "not assign `name` of node or `name` is empty";
      return -1;
    }

    std::string name = node["name"];

    std::string conf;
    if (node.count("conf") != 0) {
      conf = node["conf"];
    }
    std::string lib;
    if (node.count("lib") != 0) {
      lib = node["lib"];
    }

    if (already_exist.count(name) != 0) {
      HLOG_ERROR << "duplicated node " << name;
      return -1;
    }
    already_exist.insert(name);

    DeployerConfig::NodeInfo info = {.name = name, .conf = conf, .lib = lib};

    deployer_config.nodes.push_back(info);
  }

  if (root["sub_from_external"]) {
    deployer_config.external.sub_from_external =
        root["sub_from_external"].as<std::vector<std::string>>();
  }

  if (root["pub_to_external"]) {
    deployer_config.external.pub_to_external =
        root["pub_to_external"].as<std::vector<std::string>>();
  }

  return 0;
}

int Deployer::InitLoaderAndLoadLibs(const std::vector<std::string>& lib_paths) {
  loader_ = Loader::Create();
  if (loader_->Load(lib_paths) < 0) {
    HLOG_ERROR << "Load libs failed";
    return -1;
  }
  return 0;
}

int Deployer::InitNodes(
    const std::vector<DeployerConfig::NodeInfo>& node_infos) {
  for (const auto& info : node_infos) {
    const std::string name = info.name;
    const std::string conf = info.conf;

    HLOG_INFO << "Create node " << name;
    auto node = Factory::Instance().CreateNodeByName(name);
    if (!node) {
      HLOG_ERROR << "Create node " << name
                 << " failed, maybe name is wrong or not registered or not"
                    " specify library to load";
      return -1;
    }
    HLOG_INFO << "InitNode node " << name;
    if (node->InitNode(conf) < 0) {
      HLOG_ERROR << "InitNode node " << name << " failed";
      return -1;
    }
    inited_nodes_[name] = node;
  }
  return 0;
}

int Deployer::InitLinkerAndLinkNodes() {
  void* ctx = Context::Instance().Get();
  linker_ = Linker::Create(ctx);
  for (auto& n : inited_nodes_) {
    if (n.second) {
      HLOG_INFO << "LinkNode " << n.first;
      if (linker_->LinkNode(n.second) < 0) {
        HLOG_ERROR << "LinkNode " << n.first << " failed";
        return -1;
      }
    }
  }

  for (const auto& addr : deployer_config_.external.sub_from_external) {
    if (linker_->LinkFrontend(addr) < 0) {
      HLOG_ERROR << "Link sub external addr " << addr << " failed";
      return -1;
    } else {
      HLOG_INFO << "Link sub external addr " << addr;
    }
  }

  for (const auto& addr : deployer_config_.external.pub_to_external) {
    if (linker_->LinkBackend(addr) < 0) {
      HLOG_ERROR << "Link pub external addr " << addr << " failed";
      return -1;
    } else {
      HLOG_INFO << "Link pub external addr " << addr;
    }
  }

  if (linker_->Start() < 0) {
    HLOG_ERROR << "Linker Start failed";
    return -1;
  }
  return 0;
}

int Deployer::StartNodes() {
  for (auto& n : inited_nodes_) {
    if (n.second) {
      HLOG_INFO << "Start node " << n.first;
      if (n.second->StartNode() < 0) {
        HLOG_ERROR << "Start node " << n.first << " failed";
        return -1;
      }
    }
  }
  return 0;
}

}  // namespace mp
}  // namespace hozon