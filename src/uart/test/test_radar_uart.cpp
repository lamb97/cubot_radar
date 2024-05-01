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

// 串口数据接收缓冲区
//unsigned char receivedBuffer[1000];
u_char data[13];

int16_t target_robot_ID;
float target_position_x;
float target_position_y;
float target_position_z;

//ros数据回调函数
void detectCallback(const sensor_msgs::PointCloud2::ConstPtr &rosPCLMsg, const cubot_radar::CarsMsg::ConstPtr &carsMsg)
{
    int receiveFrequency = 0;
    cubot_radar::CarsMsg carsMsgs;
    // 接收到话题消息，进入回调函数
    receiveFrequency ++;
    ROS_INFO("Received the fuseMsg %i times", receiveFrequency);

    // 将sensor_msgs/Image格式转换为OpenCV格式
    boost::shared_ptr<cv_bridge::CvImage> getImage = cv_bridge::toCvCopy(carsMsg->image, "bgr8");
    cv::Mat subImage = getImage->image;

    CameraRadarMatrix matrixParam;
    std::string yamlFile = "/home/zhangtianyi/test_ws/src/cubot_radar/config/param/camera_radar_matrix_param.yaml";
    CameraRadarMatrixParam::LoadFromYamlFile(yamlFile, &matrixParam);

    cv::Mat cameraMatrix = matrixParam.CameraMatrix;
    cv::Mat internalMatrix = matrixParam.InternalMatrix;
    cv::Mat distCoeffs = matrixParam.DistortionVector;
    cv::Mat externalMatrix = matrixParam.ExternalMatrix;

    // 将rosPCL数据转换为PCL点云数据
    pcl::PCLPointCloud2 PCLCloud;
    pcl_conversions::toPCL(*rosPCLMsg, PCLCloud);

    // 将PCL点云数据转换为有点坐标和强度信息的PCLXYZI数据
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);  //点云指针对象
    pcl::fromPCLPointCloud2(PCLCloud, *cloud);

    // 使用相机的内参和畸变系数修正图片
    cv::Mat remap;
    cv::Mat map1;
    cv::Mat map2;
    remap = subImage.clone();
    cv::Size imageSize = subImage.size();
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                                cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, nullptr),
                                imageSize, CV_16SC2, map1, map2);
    // 畸变校正
    cv::remap(subImage, remap, map1, map2, cv::INTER_LINEAR);

    auto pointNum = (int)cloud->points.size();

    cv::Mat pointsData(4, pointNum, CV_64F);

    for (int i = 0; i < pointNum; ++i) {

        float x = cloud->points[i].x;
        float y = cloud->points[i].y;
        float z = cloud->points[i].z;

        pointsData.at<double>(0, i) = x;
        pointsData.at<double>(1, i) = y;
        pointsData.at<double>(2, i) = z;
        pointsData.at<double>(3, i) = 1;
    }

    // 将点云世界坐标转为像素坐标
    std::vector<cv::Point2i> pixelCoordinates;
    CoordinateTransformer::WorldToPixel(internalMatrix, externalMatrix, pointsData, &pixelCoordinates);


    // 给点云赋BGR值
    std::vector<cv::Point3i> BGRValue;
    CoordinateTransformer::ColoredPointCloud(pixelCoordinates, subImage, &BGRValue);

    for (int i = 0; i < pointNum; ++i){

        // 忽视无效的点
        if (pointsData.at<double>(0, i) == 0 && pointsData.at<double>(1, i) == 0 && pointsData.at<double>(2, i) == 0) {
            continue;
        }
        // 忽视BGR值都为0的点
        if (BGRValue[i].x == 0 && BGRValue[i].y == 0 && BGRValue[i].z == 0) {
            continue;
        }
        // 给点云赋予XYZ的值
        cloud->points[i].x = (float)pointsData.at<double>(0, i);
        cloud->points[i].y = (float)pointsData.at<double>(1, i);
        cloud->points[i].z = (float)pointsData.at<double>(2, i);

        // 给点云赋予BGR值（给点云赋值的操作需要一个循环里进行，才能完成点的对应）
        cloud->points[i].r = BGRValue[i].z;
        cloud->points[i].g = BGRValue[i].y;
        cloud->points[i].b = BGRValue[i].x;
    }


    // 存放单帧图像车辆类别和世界坐标的容器
    std::vector<std::pair<int, cv::Point3d>> carsWorldInfo;

    carsMsgs.image = carsMsg->image;
    carsMsgs.carNum = carsMsg->carNum;
    carsMsgs.carsInfo.resize(carsMsg->carNum);


    ros::Rate loopRate(5);
    for(int i = 0; i < carsMsg->carNum; i++){
        // 单个车辆的类别和世界坐标
        std::pair<int, cv::Point3d> carWorldInfo;

        // 车辆图像中心点坐标
        cv::Point2i carPixelCoordination((int)carsMsg->carsInfo[i].carX, (int)carsMsg->carsInfo[i].carY);

        // 单个车辆的类别
        carWorldInfo.first = carsMsg->carsInfo[i].carClass;

        std::cout << "(" <<  carWorldInfo.first << ")" <<std::endl;

        // 计算单个车辆的世界坐标
        carWorldInfo.second = CoordinateTransformer::CarWorldCoordination(carPixelCoordination,
                                                                          pointsData,
                                                                          pixelCoordinates,
                                                                          5);

        //carsMsgs.carsInfo[i].armorNum = carsMsg->carsInfo[i].armorNum;
        carsMsgs.carsInfo[i].carClass = carsMsg->carsInfo[i].carClass;
        carsMsgs.carsInfo[i].carWorldX = carWorldInfo.second.x;
        carsMsgs.carsInfo[i].carWorldY = carWorldInfo.second.y;
        carsMsgs.carsInfo[i].carWorldZ = carWorldInfo.second.z;

        std::cout << "(" << carsMsgs.carsInfo[i].carWorldX << "," << carsMsgs.carsInfo[i].carWorldY << "," << carsMsgs.carsInfo[i].carWorldZ << ")" << std::endl;

        carsWorldInfo.emplace_back(carWorldInfo);
    }

    ROS_INFO("Finish all the process %i times", receiveFrequency);

    ROS_INFO("Car: %i", carsMsgs.carNum);

    loopRate.sleep();

   if(!carsWorldInfo.empty())
   {
       ROS_INFO("Car: %i", carsWorldInfo[0].first);
       ROS_INFO("Car in: %f %f %f", carsWorldInfo[0].second.x, carsWorldInfo[0].second.y, carsWorldInfo[0].second.z);

//    auto car = (int16_t)carsWorldInfo[0].first;
//    robot_command.target_position_x = (int16_t)carsWorldInfo[0].second.x;
//    auto position_decimalX = (int16_t)(carsWorldInfo[0].second.x/1*100)/1;
//    auto position_integerY = (int16_t)carsWorldInfo[0].second.x/1;
//    auto position_decimalY = (int16_t)(carsWorldInfo[0].second.x/1*100)/1;
//    auto position_integerZ = (int16_t)carsWorldInfo[0].second.x/1;
//    auto position_decimalZ = (int16_t)(carsWorldInfo[0].second.x/1*100)/1;
       target_robot_ID = (int16_t)carsWorldInfo[0].first;
       target_position_x = (float)carsWorldInfo[0].second.x;
       target_position_y = (float)carsWorldInfo[0].second.y;
       target_position_z =  (float)carsWorldInfo[0].second.z;
   }

}


// //串口数据接收回调函数
//void HandleDataReceived(const unsigned int &bytesToRead, void* userData)
//{
//    // 读取一行数据
//    auto *serialPort = (SerialPort *)userData;
//    unsigned char size = serialPort->ReadLine(receivedBuffer);
//    receivedBuffer[size] = '\0';
//
//    // 记录接收到的时间戳
//    std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
//    uint64_t timestamp = now.time_since_epoch().count();
//
//    // 显示读取到的数据
//    std::cout << "[" << timestamp << "] - data = " << (char*)receivedBuffer << std::endl;
//}

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

    // ros节点初始化
    ros::init(argc, argv, "uart_subscriber");

    // 创建ros句柄
    ros::NodeHandle nh;
    //ros::Subscriber livoxSub2 = nh.subscribe();
    // 定义一个接受点云话题信息的消息过滤器
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointCloudSub(nh, "livox/lidar", 5);

    // 定义一个接受自定义车辆话题信息的消息过滤器
    message_filters::Subscriber<cubot_radar::CarsMsg> carsMsgSub(nh, "carsInfo", 5);

    // 设置循环频率
    ros::Rate loopRate(5);

    // 定义消息同步器的机制为大致时间同步.
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, cubot_radar::CarsMsg> MySyncPolicy;

    // 定义时间同步器的消息队列大小和接收话题
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(80), pointCloudSub, carsMsgSub);



    // 注册回调函数
    sync.registerCallback(boost::bind(&detectCallback, _1, _2));



    // 注册串口的数据接收回调函数
    //serialPort.RegistDataReceivedHandler(HandleDataReceived, &serialPort);

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

    while (ros::ok()) {
        //给ID X Y Z赋值
        command.ID = target_robot_ID;
        command.X = target_position_x;
        command.Y = target_position_y;
        command.Z = target_position_z;

////        红方只发送5-9
//        if (command.ID > 4) {
//            if (command.X > 0.0 && command.Y > 0.0 && command.Z > 0.0) {
//                // 生成控制指令数据帧
//                unsigned int commandFrameSize = RobotCommand::GetFrameSize();
//                unsigned char commandFrame[commandFrameSize];
//                command.EncapsulateToFrame(commandFrame);
//
//                // 将指令数据帧发送给下位机
//                serialPort.Write(commandFrameSize, commandFrame);
//
//                // ros消息回调处理函数
//                ros::spinOnce();
//                loopRate.sleep();
//            }
//
//        }

        //蓝方只发送0-4
        if (command.ID < 5) {
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

        // 关闭串口
        serialPort.Close();

        // 释放串口资源
        serialPort.Release();

        //ros消息回调处理函数
        //ros::spin();
        return 0;
    }
}
