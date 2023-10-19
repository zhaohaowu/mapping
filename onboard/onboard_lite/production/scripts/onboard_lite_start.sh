#!/usr/bin/env bash

TOP_DIR="$(builtin cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd -P)"
echo ${TOP_DIR}

OUTPUT_ROOT="$(builtin cd ${TOP_DIR}/../../  && pwd -P)"
echo ${OUTPUT_ROOT}

#  compile so
export LD_LIBRARY_PATH=/app/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${OUTPUT_ROOT}/lib/:$LD_LIBRARY_PATH

export ADFLITE_ROOT_PATH=${OUTPUT_ROOT}

ADF_LITE_BIN_DIR=${OUTPUT_ROOT}/bin
YAML_CONFIG_ROOT=${OUTPUT_ROOT}/production/conf
TOP_CONFIG_YAML_PATH=${YAML_CONFIG_ROOT}/top_config.yaml

# source ${ADF_LITE_BIN_DIR}/../scripts/env_setup.sh
export HZ_SET_LOG_LEVEL=IGNORE.IGNORE:kTrace
chmod -R 777 ${ADF_LITE_BIN_DIR}/adf-lite-process
${ADF_LITE_BIN_DIR}/adf-lite-process  ${TOP_CONFIG_YAML_PATH}