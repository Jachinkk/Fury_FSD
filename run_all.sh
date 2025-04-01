# 载入 ROS2 环境变量
#source /opt/ros/foxy/setup.bash
export LD_LIBRARY_PATH=/usr/local/cuda-11.6/targets/sbsa-linux/lib:$LD_LIBRARY_PATH


# 切换到工作目录（根据实际情况修改路径）
cd ~/zd/zidong

# 清理上一次的编译文件
rm -rf build install log

# 编译指定的包
#colcon build --packages-select shijue guiji can zed_yolo_node
#colcon build --packages-select can

colcon build --packages-select zed_yolo_node
# 可以在这里继续添加其他你常用的命令
source install/setup.bash


# ros2 run shijue shijue_app

# ros2 run guiji guiji_app

source install/setup.bash

#ros2 launch guiji my_launch.py
#ros2 run can can_tx_node
ros2 run zed_yolo_node zed_yolo.py

