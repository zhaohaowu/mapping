/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： mapping_log.h
 *   author     ： taoshaoyuan
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#ifdef NOT_USE_PCP_BASE_LOG
#include <glog/logging.h>
#ifndef FIXED
#define FIXED std::fixed
#endif
#ifndef SETPRECISION
#define SETPRECISION(num) std::setprecision(num)
#endif
#define HLOG_DEBUG VLOG(4)
#define HLOG_INFO LOG(INFO)
#define HLOG_WARN LOG(WARNING)
#define HLOG_ERROR LOG(ERROR)
#define HLOG_FATAL LOG(FATAL)
#else
#include <perception-base/base/utils/log.h>
#endif
