#!/usr/bin/env bash

TOP_DIR="$(builtin cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd -P)"
echo ${TOP_DIR}

export WORKSPACE=${TOP_DIR}/../..
echo $WORKSPACE

#  compile so
export LD_LIBRARY_PATH=${TOP_DIR}/../../lib/:$LD_LIBRARY_PATH

# third so
export LD_LIBRARY_PATH=${TOP_DIR}/../../../../depend/nos/x86_2004/lib/:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${TOP_DIR}/../../../../depend/third_party/x86/boost/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${TOP_DIR}/../../../../depend/third_party/x86/glog/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${TOP_DIR}/../../../../depend/third_party/x86/gtest/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${TOP_DIR}/../../../../depend/third_party/x86/gflags/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${TOP_DIR}/../../../../depend/third_party/x86/cuda/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${TOP_DIR}/../../../../depend/third_party/x86/opencv/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${TOP_DIR}/../../../../depend/third_party/perception-base/release/x86/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${TOP_DIR}/../../../../depend/perception-lib/release/x86/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${TOP_DIR}/../../../../depend/perception-common-onboard/release/x86/lib:$LD_LIBRARY_PATH

YAML_CONFIG_ROOT=${TOP_DIR}/../../../../onboard/onboard_lite/production/conf
ADF_LITE_BIN_DIR=${TOP_DIR}/../../../../depend/nos/x86_2004/bin
TOP_CONFIG_YAML_PATH=${YAML_CONFIG_ROOT}/top_config.yaml

# source ${ADF_LITE_BIN_DIR}/../scripts/env_setup.sh
export HZ_SET_LOG_LEVEL=IGNORE.IGNORE:kTrace

${ADF_LITE_BIN_DIR}/adf-lite-process  ${TOP_CONFIG_YAML_PATH}