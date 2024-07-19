#!/usr/bin/env bash

CURR_DIR="$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
echo ${CURR_DIR}

OUTPUT_ROOT="$(builtin cd ${CURR_DIR}/.. && pwd -P)"
echo ${OUTPUT_ROOT}

export PERCEPTION_ROOT_PATH=${CURR_DIR}/../runtime_service/mapping/

WORKSPACE="$(builtin cd ${OUTPUT_ROOT}/../.. && pwd -P)"
echo ${WORKSPACE}

# export HZ_SET_LOG_LEVEL=IGNORE.IGNORE:kOff

function start_mapping() {
  # third so
  export LD_LIBRARY_PATH=${OUTPUT_ROOT}/lib:$LD_LIBRARY_PATH
  export LD_LIBRARY_PATH=${OUTPUT_ROOT}/../lib:$LD_LIBRARY_PATH
  # mapping so
  export LD_LIBRARY_PATH=${OUTPUT_ROOT}/runtime_service/mapping/lib:$LD_LIBRARY_PATH
  export LD_LIBRARY_PATH=${OUTPUT_ROOT}/../../depend/nos/x86_2004/lib:$LD_LIBRARY_PATH

  full_path="${OUTPUT_ROOT}/../../asan/asan.log"
  if [ ! -f "$full_path" ]; then
    mkdir -p "$(dirname "$full_path")"
    touch "$full_path"
    echo "File created: $full_path"
  else
    echo "File already exists: $full_path"
  fi

  export ASAN_OPTIONS="check_initialization_order=1:detect_stack_use_after_return=1:detect_invalid_pointer_pairs=1:detect_leaks=1:detect_container_overflow=1:detect_odr_violation=1:allocator_may_return_null=1:detect_deadlocks=1:halt_on_error=0:log_path=${full_path}"

  export ADFLITE_ROOT_PATH=${OUTPUT_ROOT}

  chmod 755 ${OUTPUT_ROOT}/runtime_service/mapping/bin/mapping
  PROCESS_EXE=${OUTPUT_ROOT}/runtime_service/mapping/bin/mapping
  YAML_CONFIG_ROOT=${OUTPUT_ROOT}/runtime_service/mapping/conf/lite
  TOP_CONFIG_YAML_PATH=${YAML_CONFIG_ROOT}/top_config.yaml
  source ${WORKSPACE}/depend/nos/x86_2004/scripts/env_setup.sh
  LD_PRELOAD=/usr/lib/gcc/x86_64-linux-gnu/9/libasan.so ${PROCESS_EXE} ${TOP_CONFIG_YAML_PATH} &
}

function kill_mapping() {
  mapping_pid=$(ps -aux |grep mapping |grep /bin/mapping |grep -v grep |awk '{print $2}')
  echo $mapping_pid
  kill $mapping_pid
}

function asan_pack() {
  cd ${WORKSPACE}
  tar -czvf asan.tar.gz asan/*
}

function bag_play() {
  source ${WORKSPACE}/depend/nos/x86_2004/scripts/env_setup.sh
  ls ${WORKSPACE}/orin_data/
  for file in "${WORKSPACE}/orin_data/"*.mcap; do
    if [ -f "$file" ]; then
        echo "Processing file: $file"
        ls $file
        bag play $file -k /localization/fusionmap /localization/location /localization/local_map /localization/pe /localization/deadreckoning -f
    fi
  done
}

function main() {
    start_mapping
    sleep 10
    bag_play
    kill_mapping
    sleep 50
    asan_pack
    return 0
}

main "$@"
