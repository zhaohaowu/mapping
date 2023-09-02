/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-08-31
 *****************************************************************************/
#include "modules/local_mapping/local_mapping.h"

namespace hozon {
namespace mp {
namespace lm {

void LMapApp::OnLocation(
    const std::shared_ptr<const adsfi_proto::hz_Adsfi::AlgLocation> &msg) {
  std::cout << "LMApp OnLocation" << std::endl;
}

void LMapApp::OnDr(
    const std::shared_ptr<const adsfi_proto::hz_Adsfi::AlgLocation> &msg) {
  std::cout << "LMApp OnDr" << std::endl;
}

void LMapApp::OnLaneLine(
    const std::shared_ptr<const adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray>
        &msg) {
  std::cout << "LMApp OnLaneLine" << std::endl;
}

void LMapApp::OnRoadEdge(
    const std::shared_ptr<const adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray>
        &msg) {
  std::cout << "LMApp OnRoadEdge" << std::endl;
}

bool LMapApp::FetchLocalMap(std::shared_ptr<LocalMap> local_map) {
  std::cout << "LMApp FetchLocalMap" << std::endl;
  return true;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
