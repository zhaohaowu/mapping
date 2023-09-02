/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： temp_log.h
 *   author     ： taoshaoyuan
 *   date       ： 2023.09
 ******************************************************************************/

//! 临时使用，后期统一使用perception-base里的log.h

#pragma once
#include <iomanip>

#ifdef USE_PLATFORM_MDC
#include <adsf/node/node_log.h>
#else
#include <glog/logging.h>
#endif

#ifdef USE_PLATFORM_MDC

#ifndef FIXED
#define FIXED ara::log::Fixed
#endif

#ifndef SETPRECISION
#define SETPRECISION(num) ara::log::Setprecision(num)
#endif

#define HLOG_DEBUG NODE_LOG_DEBUG
#define HLOG_INFO NODE_LOG_INFO
#define HLOG_WARN NODE_LOG_WARN
#define HLOG_ERROR NODE_LOG_ERROR
#define HLOG_FATAL NODE_LOG_FATAL

#else  // USE_PLATFORM_MDC

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

#endif  // USE_PLATFORM_MDC
