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

function check_cp_status() {
  pushd ${TOP_DIR}/../
  folder_array=('./conf' './lib' './runtime_service/mapping' './scripts')
  for folder in ${folder_array[@]} ;do
        #echo "*************** check start : " $folder " **************" 
        file_array=$(find $folder -type f)
        for file in $file_array;do
                cmp_file $file
        done
        #echo "*************** check end : " $folder  "**************" 
done

if [[ $cp_failed_num -gt 0 ]]; then
    echo "!!!!!!!!!!!!!*****************copy failed***************!!!!!!!!!!!!!"
    echo "!!!!!!!!!!!!! file cp failed num :"$cp_failed_num
    echo "!!!!!!!!!!!!!*****************copy failed***************!!!!!!!!!!!!!"
else
    echo "!!!!!!!!!!!!!*****************copy success***************!!!!!!!!!!!!!"
    echo "!!!!!!!!!!!!!*****************copy success***************!!!!!!!!!!!!!"
    echo "!!!!!!!!!!!!!*****************copy success***************!!!!!!!!!!!!!"
fi

  popd
}

function cmp_file() {
local file=$1
md5_todo=$(md5sum ${TOP_DIR}/../$file | awk '{print $1}')
md5_done=$(md5sum /app/$file|awk '{print $1}')
if [ "$md5_todo" != "$md5_done" ]; then
  md5sum $file
  md5sum /app/$file
  if [[ $file == *"libglobalproto.so"* ]]; then
    echo "-------------------------------------------------------------------"
    echo "!!! libglobalproto.so can't copy, has already droped, don't care!!!"
    echo "-------------------------------------------------------------------"
  else
    echo "!!!!!!!!!!!!!*****************copy failed***************!!!!!!!!!!!!!"
	((cp_failed_num++))
    echo "!!!!!!!!!!!!!*****************copy failed***************!!!!!!!!!!!!!"
  fi
#else
#    echo $file "cp success."
fi
}

function run() {
  echo "run run_mode: $RUN_MODE"
  # mount -o remount,rw /opt/app/
  if [[ $RUN_MODE = "cp" ]]; then
    remove
    copy
    check_cp_status
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
cp_failed_num=0
main "$@"
exit $?