'''
Company: Copyright (c) Hozon Technologies Co., Ltd. 2019-2023. All rights reserved.
Author: Nihongjie
Date: 2023-09-13 11:32:49
LastEditors: Hozon
LastEditTime: 2023-10-17 20:02:26
Description: 
FilePath: /nos_tool/nos_convert_tool.py
'''
import os
import argparse
import subprocess
import time


def bash_rtfbag_convert(rtfbag_path, nos_path):
    if not os.path.exists(rtfbag_path):
        print("no {}".format(rtfbag_path))
        return
    print(rtfbag_path)
    mcap_path = os.path.splitext(rtfbag_path)[0] + "_mcap"
    rtfbag_to_mcap = "cd {} && source scripts/env_setup.sh && bag convert -i {} --input-file-type rtfbag --input-data-version 0228-0324 -o {} ".format(
        nos_path, rtfbag_path, mcap_path)
    rtfbag_to_mcap = "\"bash -c '{}'\"".format(rtfbag_to_mcap)
    res = "gnome-terminal --window -e " + rtfbag_to_mcap + " &"
    ret = subprocess.Popen(res,
                           shell=True,
                           stdout=subprocess.PIPE,
                           stderr=subprocess.PIPE,
                           encoding="utf-8")
    count = 0
    while True:
        cmd = "ps aux|grep bag|grep " + mcap_path
        p = subprocess.Popen(cmd,
                             shell=True,
                             stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT)
        flag = False
        for line in p.stdout.readlines():
            line = str(line)
            if line.find("bag convert -i") >= 0:
                flag = True
                break
        if not flag:
            break
        count += 1
        print("Deal with rtfbag to mcap, count {}".format(count))
        time.sleep(5)

    cyber_path = os.path.splitext(rtfbag_path)[0] + "_cyber"
    res = "gnome-terminal --window -e "
    count = 0
    for p in os.listdir(mcap_path):
        if not p.endswith("mcap"):
            continue
        mcap_file_path = os.path.join(mcap_path, p)
        print("Deal with {}".format(mcap_file_path))
        mcap_to_cyber = "cd {} && source scripts/env_setup.sh && bag convert -i {} --input-file-type mcap --input-data-version 0228-0324 -o {} --out-file-type cyber".format(
            nos_path, mcap_file_path, cyber_path)
        mcap_to_cyber = "\"bash -c '{}'\"".format(mcap_to_cyber)
        if count == 0:
            res += mcap_to_cyber
        else:
            res += " --tab -e " + mcap_to_cyber
        count += 1
    ret = os.popen(res)
    print("Finish converting")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-r',
                        "--rtfbag",
                        type=str,
                        default="",
                        help=("rtfbag path"),
                        required=True)
    parser.add_argument('-n',
                        "--nos",
                        type=str,
                        default="",
                        help=("nos_bag tool"),
                        required=True)
    FLAGS = parser.parse_args()
    bash_rtfbag_convert(FLAGS.rtfbag, FLAGS.nos)
