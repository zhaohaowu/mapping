#!/system/bin/sh

REPO_DIR="$(cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd -P)"
BUILD_DIR=${REPO_DIR}/build
COMPILE_JSON=${BUILD_DIR}/compile_commands.json
SKIP_FILE=${REPO_DIR}/.skipfiles
REPORTS_DIR=${BUILD_DIR}/reports
REPORTS_HTML_DIR=${BUILD_DIR}/reports_html
REPORTS_JSON=${BUILD_DIR}/reports_json

function help_install_clang_tidy() {
  echo "==============="
  echo "Install clang-tidy:"
  echo "1. Download pre-built clang+llvm package from url below:"
  echo "  https://github.com/llvm/llvm-project/releases/download/llvmorg-16.0.0/clang+llvm-16.0.0-x86_64-linux-gnu-ubuntu-18.04.tar.xz"
  echo "2. Retrieve downloaded package to any directory (e.g. /opt/clang+llvm-16.0.0-x86_64-linux-gnu-ubuntu-18.04);"
  echo "3. Add package's bin directory to environment PATH in ~/.bashrc, e.g.:"
  echo "  export PATH=/opt/clang+llvm-16.0.0-x86_64-linux-gnu-ubuntu-18.04/bin:\$PATH"
  echo ""
}

function help_install_code_checker() {
  echo "==============="
  echo "Install CodeChecker:"
  echo "1. Using command below to install:"
  echo "  pip3 install codechecker"
  echo "2. After installing, the CodeChecker executable file is located in ~/.local/bin, now add ~/.local/bin directory to environment PATH in ~/.bashrc, e.g.:"
  echo "  export PATH=\$HOME/.local/bin:\$PATH"
  echo ""
}

function check_env() {
  if ! command -v clang-tidy &> /dev/null; then
    echo "Error: clang-tidy not found"
    help_install_clang_tidy
    exit 1
  fi

  if ! command -v CodeChecker &> /dev/null; then
    echo "Error: CodeChecker not found"
    help_install_code_checker
    exit 1
  fi

  if [ ! -f ${COMPILE_JSON} ]; then
    echo "Error: ${COMPILE_JSON} not exist, please compile whole project first"
    exit 1
  fi
}

function analyze() {
  cd ${REPO_DIR}
  CodeChecker analyze ${COMPILE_JSON} -o ${REPORTS_DIR} --analyzers clang-tidy --analyzer-config 'clang-tidy:take-config-from-directory=true' -i ${SKIP_FILE}
}

function parse() {
  cd ${REPO_DIR}
  CodeChecker parse ${REPORTS_DIR} -e json -o ${REPORTS_JSON}
  CodeChecker parse ${REPORTS_DIR} -e html -o ${REPORTS_HTML_DIR}
}

function stat_dirs() {
  ret=$(python3 <<END
#!/usr/bin/python3
import json
import os
from codechecker_report_converter.report import Report

REPO_DIR = '${REPO_DIR}'
REPORTS_JSON = '${REPORTS_JSON}'

STAT_DIRS = [
  'modules/dr',
  'modules/local_mapping',
  'modules/location',
  'modules/map_fusion',
  'modules/util',
]

with open(REPORTS_JSON, 'r') as f:
  report = json.load(f)

total_num = len(report['reports'])
abs_stat_dirs = [os.path.join(REPO_DIR, d) for d in STAT_DIRS]
stat_nums = [0] * len(STAT_DIRS)

for rep in report['reports']:
  origin_path = rep['file']['original_path']
  for i in range(0, len(STAT_DIRS)):
    if origin_path.startswith(abs_stat_dirs[i]):
      stat_nums[i] += 1

other_num = total_num - sum(stat_nums)

stat_str = '----== Statistics by directories ==----\n'
for (stat_dir, stat_num) in zip(STAT_DIRS, stat_nums):
  stat_str += stat_dir + ': ' + str(stat_num) + '\n'
stat_str += 'OTHERS: ' + str(other_num) + '\n'
stat_str += 'TOTAL: ' + str(total_num) + '\n'
stat_str += '----==============================----\n'
print(stat_str)
END
)
  echo "${ret}"
}

function main() {
  check_env
  analyze
  parse
  stat_dirs
}

main "$@"
