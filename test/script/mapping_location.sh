#!/usr/bin/env bash

CURR_DIR="$(builtin cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd -P)"
echo ${CURR_DIR}

OUTPUT_ROOT="$(builtin cd ${CURR_DIR}/../.. && pwd -P)"
echo ${OUTPUT_ROOT}

WORKSPACE="$(builtin cd ${OUTPUT_ROOT}/../.. && pwd -P)"
echo ${WORKSPACE}

# third so
export LD_LIBRARY_PATH=${WORKSPACE}/depend/nos/x86_2004/lib/:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${OUTPUT_ROOT}/lib:$LD_LIBRARY_PATH

export ADFLITE_ROOT_PATH=${OUTPUT_ROOT}

ADF_LITE_BIN_DIR=${OUTPUT_ROOT}/bin
YAML_CONFIG_ROOT=${OUTPUT_ROOT}/production/conf
TOP_CONFIG_YAML_PATH=${YAML_CONFIG_ROOT}/top_config.yaml

export HZ_SET_LOG_LEVEL=IGNORE.IGNORE:kTrace

chmod +x ${ADF_LITE_BIN_DIR}/adf-lite-process
${ADF_LITE_BIN_DIR}/adf-lite-process ${TOP_CONFIG_YAML_PATH}
