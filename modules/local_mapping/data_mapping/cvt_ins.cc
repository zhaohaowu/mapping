/********************************************************
 * Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 * Author: HOZON
 *******************************************************/
#include "modules/local_mapping/data_mapping/data_mapping.h"

namespace hozon {
namespace mp {

namespace lm {
namespace data_mapping {

bool DataMapping::CvtPbIns2Ins(const NetaInsPtr& pb_ins,
                               std::shared_ptr<lm::InsData> ins_ptr) {
  ins_ptr->position.x() = pb_ins->pos_gcj02().x();
  ins_ptr->position.y() = pb_ins->pos_gcj02().y();
  ins_ptr->position.z() = pb_ins->pos_gcj02().z();
  ins_ptr->quaternion.w() = pb_ins->quaternion().w();
  ins_ptr->quaternion.x() = pb_ins->quaternion().x();
  ins_ptr->quaternion.y() = pb_ins->quaternion().y();
  ins_ptr->quaternion.z() = pb_ins->quaternion().z();
  ins_ptr->timestamp = pb_ins->header().data_stamp();

  Eigen::Matrix3d R = ins_ptr->quaternion.toRotationMatrix();
  Eigen::Translation3d translation = Eigen::Translation3d(
      ins_ptr->position.x(), ins_ptr->position.y(), ins_ptr->position.z());

  ins_ptr->pose = translation * R;

  return true;
}

}  // namespace data_mapping
}  // namespace lm
}  // namespace mp
}  //  namespace hozon
