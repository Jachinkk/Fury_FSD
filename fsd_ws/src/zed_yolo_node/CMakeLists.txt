cmake_minimum_required(VERSION 2.8.3)
project(zed_yolo_node)

# 设置 C++ 标准（如果项目中有 C++ 代码）
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 ROS1 依赖
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

# 声明 catkin 包（关键修复：添加这行）
catkin_package()

# 安装 Python 脚本到 bin 目录（确保 scripts/zed_yolo.py 存在）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/zed_yolo.py")
  catkin_install_python(PROGRAMS
    scripts/zed_yolo.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
  message(WARNING "Python script scripts/zed_yolo.py not found!")
endif()

# 安装 launch 文件（确保 launch 目录存在）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )
endif()

# 安装消息/服务/动作（如有需要）
# install(FILES msg/MyMessage.msg
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
# )

