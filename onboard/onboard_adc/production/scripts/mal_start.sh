#!/usr/bin/env bash

TOP_DIR="$(builtin cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd -P)"
echo ${TOP_DIR}
FLAG_FILE_NAME=hz_mapping.flag

export DEBUG_MAPPING_WORK_ROOT=${TOP_DIR}/../../mal_mdc

#  compile so
export LD_LIBRARY_PATH=${TOP_DIR}/lib/:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${TOP_DIR}/runtime_service/mapping/lib:$LD_LIBRARY_PATH

export LD_LIBRARY_PATH=/opt/platform/mdc_platform/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/opt/usr/third_party:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/opt/usr/app/1/gea/lib/:$LD_LIBRARY_PATH
# export LD_LIBRARY_PATH=../lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${TOP_DIR}/../../lib:$LD_LIBRARY_PATH

export CM_CONFIG_FILE_PATH=${DEBUG_MAPPING_WORK_ROOT}/runtime_service/mapping/etc/hz_mappingProcess/
export RT_DDS_URI=${DEBUG_MAPPING_WORK_ROOT}/runtime_service/mapping/etc/hz_mappingProcess/dds.xml

mapping_bin="${DEBUG_MAPPING_WORK_ROOT}/runtime_service/mapping/bin/hz_mapping"
FLAG_FILE_PATH="${DEBUG_MAPPING_WORK_ROOT}/runtime_service/mapping/conf/${FLAG_FILE_NAME}"

chmod +x ${mapping_bin}
pmupload ${mapping_bin} --flagfile ${FLAG_FILE_PATH} --allocGroup=default_dm
