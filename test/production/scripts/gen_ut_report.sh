#!/usr/bin/bash
TOP_DIR="$(builtin cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd -P)"

# 运行测试过程
bash ${TOP_DIR}/mapping_ut.sh

work_root=${TOP_DIR}/../../../..

# 收集覆盖率信息
lcov --capture --directory ${work_root} --output-file mapping_ut.info --path .

filter_files=('*/depend/*' '*/usr/*' '*/9/*' '*/test/*' '*/proto/*' '*/wheel_odom*' '*/*.pb.h')

# 过滤掉不关心的文件或目录
lcov --remove mapping_ut.info "${filter_files[@]}" --output-file mapping_ut.info

# 生成文本报告
lcov --list mapping_ut.info > mal_ut_coverage.txt

# 生成 HTML 报告
genhtml mapping_ut.info --output-directory mal_ut_report

# 显示报告路径
echo "打开浏览器并访问: $(pwd)/mal_ut_report/index.html"

rm -rf "mapping_ut.info"
