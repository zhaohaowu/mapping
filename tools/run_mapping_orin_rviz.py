'''
Company: Copyright (c) 2023 Neta Authors. All Rights Reserved
Author: Nihongjie
Date: 2023-09-05 11:32:48
LastEditors: Hozon
LastEditTime: 2023-10-31 18:23:23
Description: 
mapping_commit:0c49b265065679b4c706129eb978074b951f0649
'''
import os
import sys

if __name__ == '__main__':
    kill_cmd = "pid_id=$(ps aux | grep localization_all.launch | awk -F ' ' '{print $2}') \
                && kill -9 $pid_id \
                && echo $pid        \
                && pid_id=$(ps aux | grep ehp | awk -F ' ' '{print $2}') \
                && kill -9 $pid_id;exec bash"

    os.system("gnome-terminal --window -e \"bash -c '{}'\"".format(kill_cmd))

    mapping_path = "/data/workspace/mapping"
    mcap_path = "/data/orin_data/mcap/20231101-010944_2.mcap"
    rviz_mapping_path = "/data/workspace/fusion-localization"

    cmd = "cd {}/release/mal_x86/scripts \
                && bash mal_start.sh;exec bash".format(
        mapping_path)
    cmd = "\"bash -c '{}'\"".format(cmd)
    mapping_cmd = cmd

    cmd = "cd {}/depend/nos/nos_x86_2004 \
                && source scripts/env_setup.sh \
                && bag play {} -s 1300 -f;exec bash".format(
        mapping_path, mcap_path)
    cmd = "\"bash -c '{}'\"".format(cmd)
    mcap_play_cmd = cmd

    cmd = "cd {}/depend/nos/nos_x86_2004 \
                && source scripts/env_setup.sh \
                && topic monitor -a;exec bash".format(
        mapping_path, mcap_path)
    cmd = "\"bash -c '{}'\"".format(cmd)
    mcap_monitor_cmd = cmd

    cmd = "cd {}/build \
                && source devel/setup.bash \
                && cd {} \
                && source set_envs.sh \
                && cd {}/tools/rviz_bridge/launch \
                && roslaunch rviz_bridge rviz_bridge.launch abs_rviz_conf:=/home/ouyanghailin/Documents/mm_adc_rviz_oyhl.rviz;exec bash".format(
        rviz_mapping_path, rviz_mapping_path, rviz_mapping_path)
    cmd = "\"bash -c '{}'\"".format(cmd)
    rviz_cmd = cmd

    # res = "gnome-terminal --window -e " + rviz_cmd
    res = "gnome-terminal --window -e " + mapping_cmd \
        + " --tab -e " + mcap_play_cmd \
        + " --tab -e " + mcap_monitor_cmd \
        + " --tab -e " + rviz_cmd
    os.system(res)
