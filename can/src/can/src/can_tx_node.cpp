#include <ros/ros.h>
#include "controlcan.h"  // 注意：不要重复定义 VCI_USBCAN2，使用库内定义
#include <chrono>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <iostream>
#include <std_msgs/Float64MultiArray.h>
#include <vector>

#define M_PI 3.14159265358979323846
using namespace std;

int jinzhi(double radian)
{
    if(radian > 0.5233) radian = 0.5233;
    if(radian < -0.5233) radian = -0.5233;

    int result = (radian + 0.5233) / 0.5233 / 2 * 100;
    return result;
}

class CanTxNode
{
public:
  CanTxNode() : device_index_(0)
  {
    ros::NodeHandle nh;
    
    // 订阅控制信息
    kongzhi_subscription_ = nh.subscribe("kongzhi_topic", 10, &CanTxNode::kongzhiCallback, this);

    // 初始化超时检测变量
    last_msg_time_ = ros::Time::now();
    // 定时器回调，周期1秒，检测是否超过3秒未收到消息
    timeout_timer_ = nh.createTimer(ros::Duration(1.0), &CanTxNode::timeoutCallback, this);

    // 查找 CAN 设备
    int num = VCI_FindUsbDevice2(boardInfos_);
    if (num <= 0)
    {
      ROS_ERROR("未找到 CAN 设备");
      throw std::runtime_error("未找到 CAN 设备");
    }
    ROS_INFO("找到 %d 个 CAN 设备", num);

    // 打开设备
    if (VCI_OpenDevice(VCI_USBCAN2, device_index_, 0) != 1)
    {
      ROS_ERROR("打开 CAN 设备失败");
      throw std::runtime_error("打开 CAN 设备失败");
    }
    ROS_INFO("成功打开 CAN 设备");

    // 读取板卡信息（可选）
    if (VCI_ReadBoardInfo(VCI_USBCAN2, device_index_, &boardInfo_) != 1)
    {
      ROS_WARN("读取板卡信息失败");
    }
    else
    {
      ROS_INFO("板卡序列号: %s", boardInfo_.str_Serial_Num);
    }

    // 设置 CAN 初始化参数
    VCI_INIT_CONFIG config;
    config.AccCode  = 0;
    config.AccMask  = 0xFFFFFFFF;
    config.Filter   = 1;      // 接收所有帧（发送时无影响）
    config.Timing0  = 0x00;    // 波特率参数（例如 125Kbps）
    config.Timing1  = 0x1C;
    config.Mode     = 0;      // 正常模式

    if (VCI_InitCAN(VCI_USBCAN2, device_index_, 0, &config) != 1)
    {
      ROS_ERROR("初始化 CAN 通道 0 失败");
      throw std::runtime_error("初始化 CAN 通道 0 失败");
    }
    if (VCI_StartCAN(VCI_USBCAN2, device_index_, 0) != 1)
    {
      ROS_ERROR("启动 CAN 通道 0 失败");
      throw std::runtime_error("启动 CAN 通道 0 失败");
    }
    ROS_INFO("CAN 通道 0 启动成功，开始发送 CAN 帧");
  }

private:
  // 当收到订阅消息时更新上次消息时间，并发送 CAN 帧
  void kongzhiCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    if (msg->data.size() > 2)
    {
      std::cout << "控制信息错误" << std::endl;
    }
    else
    {
      kongzhi_v = msg->data[0];
      kongzhi_steer = msg->data[1];
      std::cout << "控制信息接受成功" << std::endl;
      
      // 更新上次消息时间
      last_msg_time_ = ros::Time::now();

      send_can_message();
    }
  }

  // 定时器回调函数，每秒检查是否超过3秒未收到消息
  void timeoutCallback(const ros::TimerEvent&)
  {
    ros::Duration diff = ros::Time::now() - last_msg_time_;
    if (diff.toSec() > 1.5)
    {
      ROS_WARN("超过3秒未收到消息,触发超时函数");
      handle_timeout();
      // 为了防止反复触发，可以考虑更新last_msg_time_或者采取其他措施
      last_msg_time_ = ros::Time::now();  // 或者根据实际需求修改
    }
  }

  // 超时后触发的函数
  void handle_timeout()
  {
    // 在此添加超时后需要执行的动作
    ROS_INFO("执行超时处理函数");
    VCI_CAN_OBJ sendObj;
    memset(&sendObj, 0, sizeof(sendObj));

    sendObj.ID = 0x0417;
    sendObj.SendType = 1;       // 正常发送
    sendObj.RemoteFlag = 0;     // 数据帧
    sendObj.ExternFlag = 0;     // 标准帧
    sendObj.DataLen = 8;        // 数据长度
    
    sendObj.Data[0] = 9;        // 示例数据
    sendObj.Data[1] = 0;        // 1 是接管，0 是不接管
    sendObj.Data[2] = 27;       // 0 是低速，1 是高速

    // 第三位与第四位表示方向盘转角
    
    
    sendObj.Data[3] = 50;
    sendObj.Data[4] = 0; 

    // 第5位与第6位是电机扭矩
    sendObj.Data[5] = 50; 
    sendObj.Data[6] = 0; 

    sendObj.Data[7] = 0; 

    int ret = VCI_Transmit(VCI_USBCAN2, device_index_, 0, &sendObj, 1);
    if (ret != 1)
    {
      ROS_ERROR("000000CAN 帧0000");
    }
    else
    {
      ROS_INFO("0000111CAN 111, ID: 0x%08X", sendObj.ID);
    }
    std::cout << VCI_USBCAN2 << std::endl;
    // 例如：发送一个特定的 CAN 帧、重置变量、报警等
  }
  
  // 发送 CAN 帧的函数
  void send_can_message()
  {
    VCI_CAN_OBJ sendObj;
    memset(&sendObj, 0, sizeof(sendObj));

    sendObj.ID = 0x0417;
    sendObj.SendType = 1;       // 正常发送
    sendObj.RemoteFlag = 0;     // 数据帧
    sendObj.ExternFlag = 0;     // 标准帧
    sendObj.DataLen = 8;        // 数据长度
    
    sendObj.Data[0] = 9;        // 示例数据
    sendObj.Data[1] = 0;        // 1 是接管，0 是不接管
    sendObj.Data[2] = 27;       // 0 是低速，1 是高速

    // 第三位与第四位表示方向盘转角
    int steer = jinzhi(-kongzhi_steer);
    std::cout << "steer: " << steer << std::endl;
    sendObj.Data[3] = steer;
    sendObj.Data[4] = 0; 


    
    // 第5位与第6位是电机扭矩

    cout<<"c=kongzhi_v"<<kongzhi_v<<endl;
    if(kongzhi_v==1)
    {
      sendObj.Data[5] = 80; 
    }
    else
    {
      sendObj.Data[5] = 50; 
    }
   
    sendObj.Data[6] = 0; 

    sendObj.Data[7] = 0; 

    int ret = VCI_Transmit(VCI_USBCAN2, device_index_, 0, &sendObj, 1);
    if (ret != 1)
    {
      ROS_ERROR("1111CAN 0000");
    }
    else
    {
      ROS_INFO("1111CAN1111 ID: 0x%08X", sendObj.ID);
    }
    std::cout << VCI_USBCAN2 << std::endl;
  }

  ros::Subscriber kongzhi_subscription_;
  ros::Timer timeout_timer_;
  int device_index_;
  
  VCI_BOARD_INFO boardInfo_;
  VCI_BOARD_INFO boardInfos_[50];
  double kongzhi_v;
  double kongzhi_steer;
  // 保存上次接收到消息的时间
  ros::Time last_msg_time_;
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "can_tx_node");
  try {
    CanTxNode node;
    ros::spin();
  } catch (const std::exception & e) {
    ROS_ERROR("Exception: %s", e.what());
  }
  return 0;
}
