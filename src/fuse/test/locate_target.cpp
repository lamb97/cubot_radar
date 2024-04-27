//
// Created by godzhu on 2021/12/22.
//

#include <iostream>
#include <ros/ros.h>
#include <rosbag/bag.h>
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
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "camera_radar_matrix_param.h"
#include "coordinate_transformer.h"
#include "detector_param.h"
#include "point_cloud_filter.h"


//int receiveFrequency = 0;
//
// pcl::visualization::CloudViewer viewer("Cloud Viewer");
//
//void callback(const sensor_msgs::Image::ConstPtr& cameraMsg, const sensor_msgs::PointCloud2::ConstPtr &rosPCLMsg);

int main(int argc, char** argv)
{
    // 创建显示窗口
    cv::namedWindow("subImage", cv::WINDOW_NORMAL);
    // ros节点初始化
    ros::init(argc, argv, "locate_target");//multi_sensor_subscriber

    // 创建ros句柄
    ros::NodeHandle nh;

    ros::Publisher timeSent = nh.advertise<cubot_radar::CarsMsg>("carsInfo", 1);
//    // 定义一个接受图像话题信息的消息过滤器
//    message_filters::Subscriber<sensor_msgs::Image> imageSub(nh, "camera/image", 10);//2
//
//    // 定义一个接受点云话题信息的消息过滤器
//    message_filters::Subscriber<sensor_msgs::PointCloud2> pointCloudSub(nh, "livox/lidar", 10);//2
//
//    // 定义消息同步器的机制为大致时间同步
//    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
//
//    // 定义时间同步器的消息队列大小和接收话题
//    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(800), imageSub, pointCloudSub);//80
//
//    // 注册回调函数
//    sync.registerCallback(boost::bind(&callback, _1, _2));

    // ros消息回调处理函数
    ros::spin();

    return 0;
}
//
//void callback(const sensor_msgs::Image::ConstPtr& cameraMsg, const sensor_msgs::PointCloud2::ConstPtr &rosPCLMsg)
//{
//    // 接收到话题消息，进入回调函数
//    receiveFrequency ++;
//    ROS_INFO("Received the CustomMsg %i times", receiveFrequency);
//
//    //创建ros句柄
//    ros::NodeHandle nh;
//
//    // 声明发布消息类型
//    ros::Publisher pubRosPCL = nh.advertise<sensor_msgs::PointCloud2>("color_lidar", 10);
//    ros::Rate loopRate(10);  // 发布频率为10赫兹
//
//    CameraRadarMatrix matrixParam;
//    std::string yamlFile = "/home/zhangtianyi/test_ws/src/cubot_radar/config/param/camera_radar_matrix_param.yaml";
//    CameraRadarMatrixParam::LoadFromYamlFile(yamlFile, &matrixParam);
//
//    cv::Mat cameraMatrix = matrixParam.CameraMatrix;
//    cv::Mat internalMatrix = matrixParam.InternalMatrix;
//    cv::Mat distCoeffs = matrixParam.DistortionVector;
//    cv::Mat externalMatrix = matrixParam.ExternalMatrix;
//
//    // 将rosPCL数据转换为PCL点云数据
//    pcl::PCLPointCloud2 PCLCloud;
//    pcl_conversions::toPCL(*rosPCLMsg, PCLCloud);
//
//    // 将PCL点云数据转换为有点坐标和强度信息的PCLXYZI数据  ---->>  应该是PCLXYZRGB?
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);  //点云指针对象
//    pcl::fromPCLPointCloud2(PCLCloud, *cloud);
//
//
//    cv::Mat subImage = cv_bridge::toCvShare(cameraMsg, "bgr8")->image;
//
//    // 使用相机的内参和畸变系数修正图片
//    cv::Mat remapImage, map1, map2;
//    remapImage = subImage.clone();
//    cv::Size imageSize = subImage.size();
//    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
//                                cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, nullptr),
//                                imageSize, CV_16SC2, map1, map2);
//    // 畸变校正
//    cv::remap(subImage, remapImage, map1, map2, cv::INTER_LINEAR);
//
//    //获取了点云中点的数量，并将其赋值给pointNum变量
//    int pointNum = (int)cloud->points.size();
//
//    //创建了一个 4 * pointNum 大小的CV_64F类型（双精度浮点型）的矩阵pointsData，用于存储点云的xyz坐标和齐次坐标。
//    cv::Mat pointsData(4, pointNum, CV_64F);
//
//    //遍历每个点，将点云中的xyz坐标存储在pointsData矩阵的相应位置,同时将齐次坐标的最后一行设置为1。
//    for (int i = 0; i < pointNum; ++i) {
//
//        float x = cloud->points[i].x;
//        float y = cloud->points[i].y;
//        float z = cloud->points[i].z;
//
//        pointsData.at<double>(0, i) = x;
//        pointsData.at<double>(1, i) = y;
//        pointsData.at<double>(2, i) = z;
//        pointsData.at<double>(3, i) = 1;
//    }
//
//    // 将点云世界坐标转为像素坐标
//    std::vector<cv::Point2i> pixelCoordinates;
//    CoordinateTransformer::WorldToPixel(internalMatrix, externalMatrix, pointsData, &pixelCoordinates);
//
//    // 给点云赋BGR值
//    std::vector<cv::Point3i> BGRValue;
//    CoordinateTransformer::ColoredPointCloud(pixelCoordinates, subImage, &BGRValue);
//
//    for (int i = 0; i < pointNum; ++i){
//
//        // 忽视无效的点
//        if (pointsData.at<double>(0, i) == 0 && pointsData.at<double>(1, i) == 0 && pointsData.at<double>(2, i) == 0) {
//            continue;
//        }
//        // 忽视BGR值都为0的点
//        if (BGRValue[i].x == 0 && BGRValue[i].y == 0 && BGRValue[i].z == 0) {
//            continue;
//        }
//        // 给点云赋予XYZ的值
//        cloud->points[i].x = (float)pointsData.at<double>(0, i);
//        cloud->points[i].y = (float)pointsData.at<double>(1, i);
//        cloud->points[i].z = (float)pointsData.at<double>(2, i);
//
//        // 给点云赋予BGR值（给点云赋值的操作需要一个循环里进行，才能完成点的对应）
//        cloud->points[i].r = BGRValue[i].z;
//        cloud->points[i].g = BGRValue[i].y;
//        cloud->points[i].b = BGRValue[i].x;
//    }
//
//
//
//    // 将处理后的点云以ros消息发布出去，便于在rviz上查看
//    ROS_INFO("Start to publish the point cloud");
//    sensor_msgs::PointCloud2 output;
//    pcl::toROSMsg(*cloud, output);
//    output.header.frame_id = "livox_frame"; //坐标系
//    pubRosPCL.publish(output);
//    loopRate.sleep();
//
//    ROS_INFO("Finish all the process %i times", receiveFrequency);
//
//    // 显示接收到的点云数据
//    viewer.showCloud(cloud);
//
//    // 显示接受到的图像数据
//    cv::imshow("subImage", subImage);
//
//    cv::waitKey(30);
//}
