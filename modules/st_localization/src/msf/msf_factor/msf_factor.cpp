/*
 * Copyright (C) 2023 by SenseTime Group Limited. All rights reserved.
 * Tang Zhuan <tangzhuan@senseauto.com>
 */

#include "msf/msf_factor/msf_factor.hpp"

#include "localization/data_type/base.hpp"
#include "msf/msf_factor/msf_factor_can/msf_factor_can_fins.hpp"
#include "msf/msf_factor/msf_factor_can/msf_factor_can_gnss.hpp"
#include "msf/msf_factor/msf_factor_can/msf_factor_can_smm.hpp"
#include "msf/msf_factor/msf_factor_imu/msf_factor_imu_gnss.hpp"
#include "msf/msf_factor/msf_factor_imu/msf_factor_imu_heading.hpp"
#include "msf/msf_factor/msf_factor_imu/msf_factor_imu_mockfull.hpp"
#include "msf/msf_factor/msf_factor_imu/msf_factor_imu_motion.hpp"
#include "msf/msf_factor/msf_factor_imu/msf_factor_imu_smm.hpp"

namespace senseAD {
namespace localization {
namespace msf {

std::unique_ptr<MSFFactor> MSFFactorFactory::CreateMSFFactor(
    LocatorType front_type, LocatorType back_type) {
  std::unique_ptr<MSFFactor> factor = nullptr;

  if (front_type == CAN) {
    switch (back_type) {
      case FINS:
        factor.reset(new MSFFactorCANFINS());
        break;
      case GNSS:
        factor.reset(new MSFFactorCANGNSS());
        break;
      case SMM:
        factor.reset(new MSFFactorCANSMM());
        break;
      default:
        LC_LERROR(LOCALIZATION)
            << "Failed to CreateMSFFactor from back_type: " << back_type;
        break;
    }
  } else if (front_type == IMU) {
    switch (back_type) {
      case GNSS:
        factor.reset(new MSFFactorIMUGNSS());
        break;
      case HEADING:
        factor.reset(new MSFFactorIMUHeading());
        break;
      case MOCK:
        factor.reset(new MSFFactorIMUMOCKFULL());
        break;
      case CAN:
        factor.reset(new MSFFactorIMUMOTION());
        break;
      case SMM:
        factor.reset(new MSFFactorIMUSMM());
        break;
      default:
        LC_LERROR(LOCALIZATION)
            << "Failed to CreateMSFFactor from back_type: " << back_type;
        break;
    }
  } else {
    LC_LERROR(LOCALIZATION)
        << "Failed to CreateMSFFactor from front_type: " << front_type;
  }

  return factor;
}

}  // namespace msf
}  // namespace localization
}  // namespace senseAD
