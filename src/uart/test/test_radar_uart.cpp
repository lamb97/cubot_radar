//
// Created by zhangtianyi on 2023/4/20.
//
#include <iostream>
#include "serial_port_param.h"
#include "serial_port.h"
#include "uart.h"

#include "robot_command.h"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "detect_armors_info.h"
#include "detector_param.h"
#include "cubot_radar/CarsMsg.h"
#include "camera_radar_matrix_param.h"
#include "coordinate_transformer.h"
u_char data[13];

int16_t target_robot_ID;
float target_position_x;
float target_position_y;
float target_position_z;
// 回调函数，处理接收到的车辆信息
void carsCallback(const cubot_radar::CarsMsg::ConstPtr& msg)
{
    // 输出接收到的车辆信息
    ROS_INFO("Received car information. Number of cars: %d", msg->carNum);

    // 遍历每辆车并输出其世界坐标
    for (int i = 0; i < msg->carNum; ++i)
    {
        ROS_INFO("Car %d: Class: %d, World Coordinates: (%f, %f, %f)",
                 i+1, msg->carsInfo[i].carClass,
                 msg->carsInfo[i].carWorldX,
                 msg->carsInfo[i].carWorldY,
                 msg->carsInfo[i].carWorldZ);
        target_robot_ID = (int16_t)msg->carsInfo[i].carClass;
        target_position_x = (float)msg->carsInfo[i].carWorldX;
        target_position_y = (float)msg->carsInfo[i].carWorldY;
        target_position_z = (float)msg->carsInfo[i].carWorldZ;
    }
}


int main(int argc, char** argv) {
    // 创建串口
    SerialPort serialPort;

    // 加载串口参数
    SerialPortParam serialPortParam;
    std::string yameFile = "/home/zhangtianyi/test_ws/src/cubot_radar/config/param/serial_port_param.yaml";
    if (!SerialPortParam::LoadFromYamlFile(yameFile, &serialPortParam)) {
        return -1;
    }

    // 设置串口参数
    if (!serialPort.SetParam(serialPortParam)) {
        return -1;
    }

    // 初始化ROS节点
    ros::init(argc, argv, "uart_subscriber");

    // 创建ros句柄
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("cars", 10, carsCallback);

    // 设置循环频率
    ros::Rate loopRate(5);

    // 初始化串口
    if (!serialPort.Init()) {
        return -1;
    }

    // 打开串口
    if (!serialPort.Open()) {
        return -1;
    }

    // 初始化控制指令
    RobotCommand command;

    // 在ROS节点仍在运行时执行循环
    while (ros::ok()) {
        // ros消息回调处理函数
        ros::spinOnce();

        //给ID X Y Z赋值
        command.ID = target_robot_ID;
        command.X = target_position_x;
        command.Y = target_position_y;
        command.Z = target_position_z;
//        红方只发送5-9
        if (command.ID > 4) {
            if (command.X > 0.0 && command.Y > 0.0 && command.Z > 0.0) {
                // 生成控制指令数据帧
                unsigned int commandFrameSize = RobotCommand::GetFrameSize();
                unsigned char commandFrame[commandFrameSize];
                command.EncapsulateToFrame(commandFrame);

                // 将指令数据帧发送给下位机
                serialPort.Write(commandFrameSize, commandFrame);

                // ros消息回调处理函数
                ros::spinOnce();
                loopRate.sleep();
            }

        }
        //蓝方
        if (command.ID < 5) {
            if (command.X > 0.0 && command.Y > 0.0 && command.Z > 0.0) {
                // 生成控制指令数据帧
                unsigned int commandFrameSize = RobotCommand::GetFrameSize();
                unsigned char commandFrame[commandFrameSize];
                command.EncapsulateToFrame(commandFrame);

                // 将指令数据帧发送给下位机
                serialPort.Write(commandFrameSize, commandFrame);
            }
        }

        // 控制循环频率
        loopRate.sleep();
    }

    // 关闭串口
    serialPort.Close();

    // 释放串口资源
    serialPort.Release();

    return 0;
}
