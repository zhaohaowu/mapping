/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： deployer.h
 *   author     ： taoshaoyuan
 *   date       ： 2023.01
 ******************************************************************************/

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace hozon {
namespace mp {

class Loader;
class Node;
class Linker;

struct DeployerConfig {
  struct NodeInfo {
    std::string name;
    std::string conf;
    std::string lib;
  };

  struct ExternalConfig {
    std::vector<std::string> sub_from_external;
    std::vector<std::string> pub_to_external;
  };

  std::vector<NodeInfo> nodes;
  ExternalConfig external;
};

class Deployer {
 private:
  using InitedNodes = std::map<std::string, std::shared_ptr<Node>>;

 public:
  Deployer() = default;

  ~Deployer() { Term(); }

  Deployer(const Deployer&) = delete;
  Deployer& operator=(const Deployer&) = delete;

  static std::shared_ptr<Deployer> Create() {
    return std::make_shared<Deployer>();
  }

  int Deploy(const std::string& deploy_file, bool dynamic_load = true);

  void Term();

  std::shared_ptr<Node> GetNode(const std::string& name);

  // return mermaid
  std::string Topo();

  // save mermaid to markdown file
  void TopoToMd(const std::string& md_path);

 private:
  int Parse(const std::string& deploy_file, DeployerConfig* deployer_config);

  bool CollectNodeInfo(
      const std::vector<std::map<std::string, std::string>>& nodes,
      std::vector<DeployerConfig::NodeInfo>* node_infos);

  int InitLoaderAndLoadLibs(const std::vector<std::string>& lib_paths);

  int InitNodes(const std::vector<DeployerConfig::NodeInfo>& node_infos);

  int InitLinkerAndLinkNodes();

  int StartNodes();

  bool LinkNode();
  bool LinkFrontend();
  bool LinkBackend();

 private:
  DeployerConfig deployer_config_;
  InitedNodes inited_nodes_;
  std::shared_ptr<Loader> loader_ = nullptr;
  std::shared_ptr<Linker> linker_ = nullptr;
};

using DeployerPtr = std::shared_ptr<Deployer>;

}  // namespace mp
}  // namespace hozon
