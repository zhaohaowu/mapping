cmake_minimum_required(VERSION 3.12.1)

project(rviz_msgs)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    std_msgs
    tf
    tf2_ros
    rviz
)

# 需要先添加msg/common, 后添加其它目录, 否则会报common里定义的基础类型找不到
add_message_files(
  DIRECTORY msg/common
)
add_message_files(
  DIRECTORY msg/location
)
add_message_files(
  DIRECTORY msg/dead_reckoning
)
add_message_files(
  DIRECTORY msg/soc
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
    CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf tf2_ros message_generation
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
