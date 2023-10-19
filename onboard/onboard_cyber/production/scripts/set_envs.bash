#!/usr/bin/env bash

CURR_DIR="$(builtin cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd -P)"
OUTPUT_ROOT="$(builtin cd ${CURR_DIR}/.. && pwd -P)"
export LD_LIBRARY_PATH=${OUTPUT_ROOT}/lib:$LD_LIBRARY_PATH
