#!/usr/bin/env bash

TOP_DIR="$(builtin cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd -P)"
echo ${TOP_DIR}
#  compile so
export LD_LIBRARY_PATH=${TOP_DIR}/../../lib/:$LD_LIBRARY_PATH

YAML_CONFIG_ROOT=${TOP_DIR}/../../production/conf
ADF_LITE_BIN_DIR=${TOP_DIR}/../../bin
TOP_CONFIG_YAML_PATH=${YAML_CONFIG_ROOT}/top_config.yaml

# source ${ADF_LITE_BIN_DIR}/../scripts/env_setup.sh
export HZ_SET_LOG_LEVEL=IGNORE.IGNORE:kTrace

chmod -R 777 ${ADF_LITE_BIN_DIR}/adf-lite-process
${ADF_LITE_BIN_DIR}/adf-lite-process  ${TOP_CONFIG_YAML_PATH}