#!/usr/bin/env bash

CURR_DIR="$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
echo ${CURR_DIR}

OUTPUT_ROOT="$(builtin cd ${CURR_DIR}/.. && pwd -P)"
echo ${OUTPUT_ROOT}

export PERCEPTION_ROOT_PATH=${CURR_DIR}/../runtime_service/mapping/

WORKSPACE="$(builtin cd ${OUTPUT_ROOT}/../.. && pwd -P)"
echo ${WORKSPACE}

# third so
export LD_LIBRARY_PATH=${OUTPUT_ROOT}/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${OUTPUT_ROOT}/../lib:$LD_LIBRARY_PATH
# mapping so
export LD_LIBRARY_PATH=${OUTPUT_ROOT}/runtime_service/mapping/lib:$LD_LIBRARY_PATH

export ADFLITE_ROOT_PATH=${OUTPUT_ROOT}

PROCESS_EXE=${OUTPUT_ROOT}/runtime_service/mapping/bin/mapping
YAML_CONFIG_ROOT=${OUTPUT_ROOT}/runtime_service/mapping/conf/lite
TOP_CONFIG_YAML_PATH=${YAML_CONFIG_ROOT}/top_config_evaluating.yaml

# export HZ_SET_LOG_LEVEL=IGNORE.IGNORE:kOff

${PROCESS_EXE} ${TOP_CONFIG_YAML_PATH}
