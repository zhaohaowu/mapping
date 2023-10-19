#!/usr/bin/env bash

TOP_DIR="$(builtin cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd -P)"

export HZ_UT_BIN="mapping_ut"
export PROJECT_LIB_ROOT=${TOP_DIR}/../../

export LD_LIBRARY_PATH=${TOP_DIR}/../../lib/:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${TOP_DIR}/../../lib/:$LD_LIBRARY_PATH

export LD_LIBRARY_PATH=${TOP_DIR}/../../../../depend/third_party/x86/boost/lib:$LD_LIBRARY_PATH
# export LD_LIBRARY_PATH=${TOP_DIR}/../../../../depend/third_party/x86/glog/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${TOP_DIR}/../../../../depend/third_party/x86/gtest/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${TOP_DIR}/../../../../depend/third_party/x86/gflags/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${TOP_DIR}/../../../../depend/third_party/x86/cuda/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${TOP_DIR}/../../../../depend/nos/x86_2004/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${TOP_DIR}/../../../../depend/perception-base/release/x86/lib:$LD_LIBRARY_PATH

export PROJECT_CONFIG_ROOT=${TOP_DIR}/../production/

${TOP_DIR}/../${HZ_UT_BIN}