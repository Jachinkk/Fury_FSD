cmake_minimum_required(VERSION 3.0.2)
project(guiji)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找ROS依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

# 查找第三方依赖
find_package(Eigen3 REQUIRED)
find_package(CasADi REQUIRED HINTS /usr/local/lib/cmake/casadi)  # 显式指定路径
find_package(SFML COMPONENTS graphics window system REQUIRED)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CasADi_INCLUDE_DIRS}
  ${SFML_INCLUDE_DIRS}
)

# 链接目录
link_directories(/usr/local/lib)

# 定义可执行文件
add_executable(guiji 
  src/guiji.cpp
  # 添加其他源文件...
)

# 链接库（注意顺序）
target_link_libraries(guiji
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  casadi # 命名空间方式更安全
  sfml-graphics
  sfml-window
  sfml-system
)

# 设置安装规则
install(TARGETS guiji
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 声明catkin包（关键！）
catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES guiji
  CATKIN_DEPENDS roscpp std_msgs
)

# 强制指定输出路径
set_target_properties(guiji PROPERTIES
  OUTPUT_NAME guiji
  RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}
)

# 确保安装规则
install(TARGETS guiji
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
