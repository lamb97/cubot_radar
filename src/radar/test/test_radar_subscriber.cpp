#include <iostream>
#include <opencv2/opencv.hpp>
#include "spdlog/spdlog.h"
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <cubot_radar/CustomMsg.h>
#include "camera_radar_matrix_param.h"
#include "delta_cv.h"
#include "coordinate_transformer.h"

int receiveFrequency = 0;

pcl::visualization::CloudViewer viewer("Cloud Viewer");

// 用于订阅rosPCL数据的回调函数
void rosPCLMsgCallback(const sensor_msgs::PointCloud2::ConstPtr &rosPCLMsg);

// 用于订阅CustomMsg数据的回调函数
void CustomMsgCallback(const cubot_radar::CustomMsg::ConstPtr &radarMsg);

int main(int argc, char **argv) {
    ros::init(argc, argv, "livox_lidar_subsriber");

    //创建ros句柄
    ros::NodeHandle n;


    ros::Subscriber livoxSub1 = n.subscribe("livox/lidar", 10, CustomMsgCallback);
    //ros::Subscriber livoxSub2 = n.subscribe("livox/lidar", 10, rosPCLMsgCallback);

    // ros::spin()将会进入循环,一直调用回调函数,每次调用1000个数据。
    // 循环等待回调函数(若在循环内接收话题消息则使用ros::spinOnce)
    ros::spin();
    return 0;
}

// 用于订阅rosPCL数据的回调函数
void rosPCLMsgCallback(const sensor_msgs::PointCloud2::ConstPtr &rosPCLMsg) {
    // 接收到话题消息，进入回调函数
    receiveFrequency ++;
    ROS_INFO("Received the CustomMsg %i times", receiveFrequency); // receiveFrequency为计数器

    //创建ros句柄
    ros::NodeHandle nh;

    // 声明发布消息类型

    ros::Publisher pubRosPCL = nh.advertise<sensor_msgs::PointCloud2>("color_lidar", 10);
    ros::Rate loopRate(10);  // 发布频率为10赫兹

    CameraRadarMatrix matrixParam;
    std::string yamlFile = "/home/zhangtianyi/test_ws/src/cubot_radar/config/param/camera_radar_matrix_param.yaml";
    CameraRadarMatrixParam::LoadFromYamlFile(yamlFile, &matrixParam);

    cv::Mat cameraMatrix = matrixParam.CameraMatrix;
    cv::Mat internalMatrix = matrixParam.InternalMatrix;
    cv::Mat distCoeffs = matrixParam.DistortionVector;  //畸变校正矩阵
    cv::Mat externalMatrix = matrixParam.ExternalMatrix;

    ROS_INFO("Finished loading matrix param");

    cv::Mat rawImage;
    rawImage = cv::imread("/home/zhangtianyi/MVviewer/pictures黄/6.bmp");
    if(rawImage.empty())
    {
        spdlog::error("rawImage does not exist !");
        return;
    }

    // 使用相机的内参和畸变系数修正图片
    cv::Mat remapImage, map1, map2;
    remapImage = rawImage.clone();
    cv::Size imageSize = rawImage.size();
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                                cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, nullptr),
                                imageSize, CV_16SC2, map1, map2);
    // 畸变校正
    cv::remap(rawImage, remapImage, map1, map2, cv::INTER_LINEAR);

    ROS_INFO("Finished remapping image");

    // 将rosPCL数据转换为PCL点云数据
    pcl::PCLPointCloud2 PCLCloud;
    pcl_conversions::toPCL(*rosPCLMsg, PCLCloud);

    // 将PCL点云数据转换为有点坐标和强度信息的PCLXYZI数据
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);  //点云指针对象
    pcl::fromPCLPointCloud2(PCLCloud, *cloud);

    cloud->is_dense = false;
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->points.resize(cloud->width);

    int pointNum = (int)cloud->points.size();

    cv::Mat pointsData(4, pointNum, CV_64F);

    for(auto& point : cloud->points)
    {
        std::cout << "point.x1: " << point.x << std::endl;
        std::cout << "point.y1: " << point.y << std::endl;
        std::cout << "point.z1: " << point.z << std::endl;
//      std::cout << "point.intensity1: " << point.intensity << std::endl;
    }

    // 将点云中所有点载入矩阵
    for (int i = 0; i < pointNum; ++i) {
        pointsData.at<double>(0, i) = cloud->points[i].x;
        pointsData.at<double>(1, i) = cloud->points[i].y;
        pointsData.at<double>(2, i) = cloud->points[i].z;
        pointsData.at<double>(3, i) = 1;
    }

    ROS_INFO("Start to transform coordination");
    // 将点云世界坐标转为像素坐标
    std::vector<cv::Point2i> pixelCoordinates;
    CoordinateTransformer::WorldToPixel(internalMatrix, externalMatrix, pointsData, &pixelCoordinates);

    ROS_INFO("Finished transforming coordination");

    // 给点云赋BGR值
    std::vector<cv::Point3i> BGRValue;
    CoordinateTransformer::ColoredPointCloud(pixelCoordinates, rawImage, &BGRValue);

    ROS_INFO("Finished storing BGR value for point cloud");

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

    ROS_INFO("Finished coloring point cloud");
    // 显示接收到的点云数据
    viewer.showCloud(cloud);

    // 将处理后的点云以ros消息发布出去，便于在rviz上查看
    ROS_INFO("Start to publish the point cloud");
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "livox_frame"; //坐标系
    pubRosPCL.publish(output);
    loopRate.sleep();

    ROS_INFO("Finished all the process %i times\n", receiveFrequency);
}

// 用于订阅CustomMsg数据的回调函数
void CustomMsgCallback(const cubot_radar::CustomMsg::ConstPtr &radarMsg) {
    // 接收到话题消息，进入回调函数
    receiveFrequency ++;
    ROS_INFO("Received the CustomMsg %i times", receiveFrequency);

    //创建ros句柄
    ros::NodeHandle nh;

    // 声明发布消息类型
    ros::Publisher pubRosPCL = nh.advertise<sensor_msgs::PointCloud2>("color_lidar", 10);
    ros::Rate loopRate(10);  // 发布频率为10赫兹

    CameraRadarMatrix matrixParam;
    std::string yamlFile = "/home/zhnagtianyi/test_ws/src/cubot_radar/config/param/camera_radar_matrix_param.yaml";
    CameraRadarMatrixParam::LoadFromYamlFile(yamlFile, &matrixParam);

    cv::Mat cameraMatrix = matrixParam.CameraMatrix;
    cv::Mat internalMatrix = matrixParam.InternalMatrix;
    cv::Mat distCoeffs = matrixParam.DistortionVector;
    cv::Mat externalMatrix = matrixParam.ExternalMatrix;

    cv::Mat rawImage;
    rawImage = cv::imread("/home/zhangtianyi/MVviewer/picture/6.png");
    if(rawImage.empty())
    {
        spdlog::error("rawImage does not exist !");
        return;
    }

    cv::Mat map1, map2;
    cv::Size imageSize = rawImage.size();

    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                                cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, nullptr),
                                imageSize, CV_16SC2, map1, map2);

    cv::remap(rawImage, rawImage, map1, map2, cv::INTER_LINEAR);  // 畸变校正

    // 存放像素值的容器初始化
    std::vector<std::vector<int>> pixelValue;
    // 将修正后的图片像素值载入容器
    DeltaCV::getPixelValue(rawImage, &pixelValue);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);  //点云指针对象
    cloud->is_dense = false;
    cloud->width = radarMsg->points.size();
    cloud->height = 1;
    cloud->points.resize(cloud->width);

    int pointNum = (int)cloud->points.size();

    cv::Mat pointsData(4, pointNum, CV_64F);

    // 将点云中所有点载入矩阵
    for (int i = 0; i < pointNum; ++i) {

        float x = radarMsg->points[i].x;
        float y = radarMsg->points[i].y;
        float z = radarMsg->points[i].z;

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
    CoordinateTransformer::ColoredPointCloud(pixelCoordinates, rawImage, &BGRValue);

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
    // 显示接收到的点云数据
    viewer.showCloud(cloud);

    // 将处理后的点云以ros消息发布出去，便于在rviz上查看
    ROS_INFO("Start to publish the point cloud");
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "livox_frame"; //坐标系
    pubRosPCL.publish(output);
    loopRate.sleep();

    ROS_INFO("Finish all the process %i times", receiveFrequency);
}
