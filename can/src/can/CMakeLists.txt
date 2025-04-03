cmake_minimum_required(VERSION 3.5)
project(can)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 查找ROS依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /home/fury/fsd  # controlcan.h路径
)

# 定义可执行文件
add_executable(can_tx_node src/can_tx_node.cpp)

# 强制指定输出路径（关键修复）
set_target_properties(can_tx_node PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}"
  INSTALL_RPATH "$ORIGIN;/home/fury/fsd"
  BUILD_WITH_INSTALL_RPATH TRUE
  SKIP_BUILD_RPATH FALSE
)

# 链接库
target_link_libraries(can_tx_node
  ${catkin_LIBRARIES}
  /home/fury/fsd/libcontrolcan.so
)

# 安装规则
install(TARGETS can_tx_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装launch文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# 声明catkin包
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)
