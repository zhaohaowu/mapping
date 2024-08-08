#include "base/utils/log.h"
#include "depend/map/hdmap/hdmap.h"
#include "depend/proto/local_mapping/local_map.pb.h"
#include "depend/proto/map/map.pb.h"
#include "depend/proto/perception/perception_obstacle.pb.h"
#include "modules/map_fusion_02/base/element_base.h"
#include "modules/map_fusion_02/base/element_map.h"

namespace hozon {
namespace mp {
namespace mf {

class DataConvert {
 public:
  DataConvert() = default;
  ~DataConvert() = default;

  static bool LocalMap2ElmentMap(
      const std::shared_ptr<hozon::mapping::LocalMap>& msg,
      const std::shared_ptr<hozon::perception::PerceptionObstacles>& obj_msg,
      ElementMap::Ptr elem_map);

  static bool Localization2LocInfo(
      const std::shared_ptr<hozon::localization::Localization>& loc_msg,
      LocInfo::Ptr loc_info);
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon