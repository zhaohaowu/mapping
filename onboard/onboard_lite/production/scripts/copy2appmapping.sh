#!/usr/bin/env bash

TOP_DIR="$(builtin cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd -P)"

function copy() {
  pushd ${TOP_DIR}/../runtime_service/
  cp -rf mapping /app/runtime_service/
  chown nvidia:nvidia -R /app/runtime_service/mapping
  chmod 755 -R /app/runtime_service/mapping
  popd

  cp -rf ${TOP_DIR}/../lib/* /app/lib/
  chown nvidia:nvidia -R /app/lib/
  chmod 755 -R /app/lib/

  cp -rf ${TOP_DIR}/../conf/* /app/conf
  chown nvidia:nvidia -R /app/conf/mapping
  chmod 755 -R /app/conf/mapping

  cp -rf ${TOP_DIR}/../data/* /app/data

  cp -rf ${TOP_DIR}/../scripts/* /app/scripts
  mapping_file="/app/runtime_service/mapping/bin/mapping"
  if [ -f "$mapping_file" ]; then
      cur_mapping_bin_md5=$(md5sum "$mapping_file")
      echo "cur runtime_service/mapping/bin/mapping md5: $cur_mapping_bin_md5"
      echo "succ deploy mapping!!!"
  else
      echo "Error: File '$mapping_file' does not exist."
      echo "fail deploy mapping!!!"
  fi
}
function remove() {
  find /app/runtime_service/mapping -mindepth 1 -maxdepth 1 ! -name etc -exec rm -rf {} +
  rm -rf /app/conf/mapping/*
}

function backup() {
  pushd /opt/usr/
  popd
}

function run() {
  echo "run run_mode: $RUN_MODE"
  # mount -o remount,rw /opt/app/
  if [[ $RUN_MODE = "cp" ]]; then
    remove
    copy
  elif [[ $RUN_MODE = "rm" ]]; then
    remove
  elif [[ $RUN_MODE = "bk" ]]; then
    backup
  fi
  sync
}

function usage() {
    echo "
Usage:   $0 [mode]

Example: $0 [cp|rm|bk]
"
}

function main() {
    if [ $# -lt 1 ]; then
        usage && return 1
    fi
    mapping_file="/app/runtime_service/mapping/bin/mapping"
    if [ -f "$mapping_file" ]; then
        cur_mapping_bin_md5=$(md5sum "$mapping_file")
        echo "cur runtime_service/mapping/bin/mapping md5: $cur_mapping_bin_md5"
    fi
    if [ $# -ge 1 ]; then
        case "$1" in
        "cp") RUN_MODE="cp" ;;
        "rm") RUN_MODE="rm" ;;
        "bk") RUN_MODE="bk" ;;
        *) usage && return 0 ;;
        esac
    fi
    run
    [ $? -ne 0 ] && return 1
    return 0
}
main "$@"
exit $?