/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-08-31
 *****************************************************************************/
#pragma once

#include <memory>

#include "cyber/component/component.h"
#include "depend/proto/localization/node_info.pb.h"
#include "modules/local_mapping/lib/local_mapping.h"

namespace hozon {
namespace mp {
namespace lm {
class LMapComponent : public apollo::cyber::Component<> {
 public:
  LMapComponent() = default;
  ~LMapComponent() override;

  /**
   * @brief cyber init
   *
   * @return `true` for success, `false` for failed
   */
  bool Init() override;

  /**
   * @brief receive location message
   *
   * @param msg : location message
   * @return `true` for receiveing and processing success, `false` for failed
   */
  bool OnLocation(
      const std::shared_ptr<const hozon::localization::Localization>& msg);

  /**
   * @brief receive dr message
   *
   * @param msg : dr message
   * @return `true` for receiveing and processing success, `false` for failed
   */
  bool OnDr(
      const std::shared_ptr<const hozon::dead_reckoning::DeadReckoning>& msg);

  /**
   * @brief receive ins message
   *
   * @param msg : ins message
   * @return `true` for receiveing and processing success, `false` for failed
   */
  bool OnIns(
      const std::shared_ptr<const hozon::localization::HafNodeInfo>& msg);

  /**
   * @brief receive laneline message
   *
   * @param msg : laneline message
   * @return `true` for receiveing and processing success, `false` for failed
   */
  bool OnLaneLine(
      const std::shared_ptr<const hozon::perception::TransportElement>& msg);

  /**
   * @brief receive roadedge message
   *
   * @param msg : roadedge message
   * @return `true` for receiveing and processing success, `false` for failed
   */
  bool OnRoadEdge(
      const std::shared_ptr<const hozon::perception::TransportElement>& msg);

  /**
   * @brief local map publish
   *
   * @return
   */
  void LocalMapPublish();

  /**
   * @brief local map location publish
   *
   * @return
   */
  void LocalMapLocationPublish();

 private:
  std::thread local_map_publish_thread_;
  std::thread local_map_location_publish_thread_;
  std::shared_ptr<LMapApp> lmap_;
  std::shared_ptr<apollo::cyber::Reader<hozon::localization::Localization>>
      location_listener_;
  std::shared_ptr<apollo::cyber::Reader<hozon::dead_reckoning::DeadReckoning>>
      dr_listener_;
  std::shared_ptr<apollo::cyber::Reader<hozon::localization::HafNodeInfo>>
      ins_listener_;
  std::shared_ptr<apollo::cyber::Reader<hozon::perception::TransportElement>>
      laneline_listener_;
  std::shared_ptr<apollo::cyber::Reader<hozon::perception::TransportElement>>
      roadedge_listener_;
  std::shared_ptr<apollo::cyber::Writer<hozon::mapping::LocalMap>>
      result_talker_;
  std::shared_ptr<apollo::cyber::Writer<hozon::localization::Localization>>
      result_location_talker_;
};

CYBER_REGISTER_COMPONENT(LMapComponent);

}  // namespace lm
}  // namespace mp
}  // namespace hozon
