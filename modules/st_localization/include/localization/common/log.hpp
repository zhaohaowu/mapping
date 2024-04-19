/*
 * Copyright (C) 2023 by SenseTime Group Limited. All rights reserved.
 * Tang Zhuan <tangzhuan@senseauto.com>
 */

#pragma once

#include "ad_log/ad_log.hpp"

namespace senseAD {
namespace localization {

#define LC_LTRACE(tag) AD_LTRACE(tag)
#define LC_LDEBUG(tag) AD_LDEBUG(tag)
#define LC_LINFO(tag) AD_LINFO(tag)
#define LC_LERROR(tag) AD_LERROR(tag)
#define LC_LWARN(tag) AD_LWARN(tag)
#define LC_LFATAL(tag) AD_LFATAL(tag)

#define LC_LTRACE_EVERY(tag, n) AD_LTRACE_EVERY(tag, n)
#define LC_LDEBUG_EVERY(tag, n) AD_LDEBUG_EVERY(tag, n)
#define LC_LINFO_EVERY(tag, n) AD_LINFO_EVERY(tag, n)
#define LC_LERROR_EVERY(tag, n) AD_LERROR_EVERY(tag, n)
#define LC_LWARN_EVERY(tag, n) AD_LWARN_EVERY(tag, n)
#define LC_LFATAL_EVERY(tag, n) AD_LFATAL_EVERY(tag, n)

#define LC_LTRACE_EVERY_SEC(tag, n) AD_LTRACE_EVERY_SEC(tag, n)
#define LC_LDEBUG_EVERY_SEC(tag, n) AD_LDEBUG_EVERY_SEC(tag, n)
#define LC_LINFO_EVERY_SEC(tag, n) AD_LINFO_EVERY_SEC(tag, n)
#define LC_LERROR_EVERY_SEC(tag, n) AD_LERROR_EVERY_SEC(tag, n)
#define LC_LWARN_EVERY_SEC(tag, n) AD_LWARN_EVERY_SEC(tag, n)

}  // namespace localization
}  // namespace senseAD
