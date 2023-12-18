#!/usr/bin/env bash

TOP_DIR="$(builtin cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd -P)"

function copy() {
  pushd ${TOP_DIR}/../runtime_service/
  cp -rf * /app/runtime_service/
  chmod 755 -R /app/runtime_service/mapping
  popd

  cp -rf ${TOP_DIR}/../conf/* /app/conf

  cp -rf ${TOP_DIR}/../data/* /app/data

  cp -rf ${TOP_DIR}/../scripts/* /app/scripts
}
function remove() {
  rm -rf /app/runtime_service/mapping
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
    RUN_MODE="cp"
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