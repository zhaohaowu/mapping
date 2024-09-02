#!/usr/bin/env bash

CURR_DIR="$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
echo ${CURR_DIR}

OUTPUT_ROOT="$(builtin cd ${CURR_DIR}/.. && pwd -P)"
echo ${OUTPUT_ROOT}

export PERCEPTION_ROOT_PATH=${CURR_DIR}/../runtime_service/mapping/

WORKSPACE="$(builtin cd ${OUTPUT_ROOT}/../.. && pwd -P)"
echo ${WORKSPACE}
source ${WORKSPACE}/depend/nos/x86_2004/scripts/env_setup.sh

export LD_LIBRARY_PATH=${OUTPUT_ROOT}/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${OUTPUT_ROOT}/../lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${OUTPUT_ROOT}/runtime_service/mapping/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${OUTPUT_ROOT}/../../depend/nos/x86_2004/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${OUTPUT_ROOT}/../../depend/third_party/x86_2004/yaml-cpp/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${OUTPUT_ROOT}/../../depend/third_party/x86_2004/glog/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${OUTPUT_ROOT}/../../depend/third_party/x86_2004/gflags/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${OUTPUT_ROOT}/../../depend/third_party/x86_2004/protobuf/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${OUTPUT_ROOT}/../../depend/third_party/x86_2004/ceres/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${OUTPUT_ROOT}/../../depend/third_party/x86_2004/boost/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${OUTPUT_ROOT}/../../depend/third_party/x86_2004/opencv/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${OUTPUT_ROOT}/../../depend/third_party/x86_2004/pcl/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${OUTPUT_ROOT}/../../depend/third_party/x86_2004/absl/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${OUTPUT_ROOT}/../../depend/third_party/x86_2004/zmq/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${OUTPUT_ROOT}/../../depend/third_party/x86_2004/gtest/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${OUTPUT_ROOT}/../../depend/third_party/x86_2004/amap/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${OUTPUT_ROOT}/../../depend/third_party/x86_2004/zipper/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${OUTPUT_ROOT}/../../depend/third_party/x86_2004/jsoncpp/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${OUTPUT_ROOT}/../../depend/third_party/x86_2004/hzavpmap/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${OUTPUT_ROOT}/../../depend/third_party/x86_2004/fastrtps/lib:$LD_LIBRARY_PATH


export ADFLITE_ROOT_PATH=${OUTPUT_ROOT}

PROCESS_EXE=${OUTPUT_ROOT}/runtime_service/mapping/bin/mapping
YAML_CONFIG_ROOT=${OUTPUT_ROOT}/runtime_service/mapping/conf/lite
TOP_CONFIG_YAML_PATH=${YAML_CONFIG_ROOT}/top_config.yaml

${PROCESS_EXE} ${TOP_CONFIG_YAML_PATH}
