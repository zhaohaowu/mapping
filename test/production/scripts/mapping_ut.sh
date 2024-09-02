#!/usr/bin/env bash

CURR_DIR="$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
echo ${CURR_DIR}

WORK_ROOT="$(builtin cd ${CURR_DIR}/.. && pwd -P)"
echo ${WORK_ROOT}

export PERCEPTION_ROOT_PATH=${WORK_ROOT}/../runtime_service/mapping/
export ADFLITE_ROOT_PATH=${WORK_ROOT}/../

export LD_LIBRARY_PATH=${WORK_ROOT}/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${WORK_ROOT}/../lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${WORK_ROOT}/../runtime_service/mapping/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${WORK_ROOT}/../../../depend/third_party/x86_2004/gtest/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${WORK_ROOT}/../../../depend/third_party/x86_2004/yaml-cpp/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${WORK_ROOT}/../../../depend/third_party/x86_2004/glog/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${WORK_ROOT}/../../../depend/third_party/x86_2004/gflags/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${WORK_ROOT}/../../../depend/third_party/x86_2004/protobuf/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${WORK_ROOT}/../../../depend/third_party/x86_2004/ceres/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${WORK_ROOT}/../../../depend/third_party/x86_2004/boost/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${WORK_ROOT}/../../../depend/third_party/x86_2004/opencv/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${WORK_ROOT}/../../../depend/third_party/x86_2004/pcl/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${WORK_ROOT}/../../../depend/third_party/x86_2004/absl/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${WORK_ROOT}/../../../depend/third_party/x86_2004/zmq/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${WORK_ROOT}/../../../depend/third_party/x86_2004/gtest/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${WORK_ROOT}/../../../depend/third_party/x86_2004/amap/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${WORK_ROOT}/../../../depend/third_party/x86_2004/zipper/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${WORK_ROOT}/../../../depend/third_party/x86_2004/jsoncpp/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${WORK_ROOT}/../../../depend/third_party/x86_2004/hzavpmap/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${WORK_ROOT}/../../../depend/third_party/x86_2004/fastrtps/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${WORK_ROOT}/../../../depend/third_party/x86_2004/cuda/targets/x86_64-linux/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${WORK_ROOT}/../../../depend/nos/x86_2004/lib:$LD_LIBRARY_PATH



chmod +x ${WORK_ROOT}/bin/mapping_ut
${WORK_ROOT}/bin/mapping_ut

