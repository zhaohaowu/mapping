#!/usr/bin/env bash
set -e
set -o pipefail

# 解析workspace
WORKSPACE=$(builtin cd $(dirname ${BASH_SOURCE[0]}); pwd)
cd $WORKSPACE
# 设置python解释器
if [ -z "$PYTHON_BIN" ]; then
  PYTHON_BIN=$(which python3 || which python || true)
fi

export LD_LIBRARY_PATH=${WORKSPACE}/depend/third_party/x86/protobuf/lib:$LD_LIBRARY_PATH
export PYTHONPATH=$WORKSPACE
if [ -d "${WORKSPACE}/release" ]; then
  rm -r "${WORKSPACE}/release"
fi
$PYTHON_BIN tools/compile.py $@ --workspace $WORKSPACE

if [ ! -d "${WORKSPACE}/release/lib/" ]; then
  mkdir -p "${WORKSPACE}/release/lib/"
fi

lib_folder="${WORKSPACE}/release/mal_orin/lib/"
mv $lib_folder/libeuropa_common.so ${WORKSPACE}/release/lib/
mv $lib_folder/libeuropa_hdmap.so ${WORKSPACE}/release/lib/
mv $lib_folder/libperception-base.so ${WORKSPACE}/release/lib/
mv $lib_folder/libperception-lib.so ${WORKSPACE}/release/lib/

WITH_MAL_PLUGIN_FLAG=$(cat plugin_env.txt)
if [ "${WITH_MAL_PLUGIN_FLAG}" = "true" ]; then
  cd ${WORKSPACE}/depend/mapping_plugin
  mapping_plugin_workspace=$(pwd)

  # 过滤掉--plugin参数
  filtered_params=""
  for param in "$@"; do
    if [ "$param" != "--plugin" ]; then
      filtered_params="$filtered_params $param"
    fi
  done

  bash build.sh $filtered_params
  src_folder="$mapping_plugin_workspace/release/mal_plugin_orin/runtime_service/mapping_plugin/"
  dest_folder="${WORKSPACE}/release/mal_orin/runtime_service/"
  cp $src_folder $dest_folder -rf
  rm $dest_folder/mapping_plugin/lib/* -rf
  cp $src_folder/../../lib/libmapping_europa_common.so $dest_folder/mapping_plugin/lib/
  cp $src_folder/../../lib/libmapping_europa_hdmap.so $dest_folder/mapping_plugin/lib/
  cp $src_folder/../../scripts/* $dest_folder/../scripts/
  rm ${WORKSPACE}/plugin_env.txt -rf
fi
