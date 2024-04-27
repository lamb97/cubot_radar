//
// Created by zhangtianyi on 2024/1/3.
//

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
#include "livox_radar_subscriber.h"
#include "point_cloud_filter.h"

int receiveFrequency = 0;

pcl::visualization::CloudViewer viewer("Cloud Viewer");

// 用于订阅CustomMsg数据的回调函数
void CustomMsgCall(const cubot_radar::CustomMsg::ConstPtr &radarMsg){
    // 接收到话题消息，进入回调函数
    receiveFrequency ++;
    ROS_INFO("Received the CustomMsg %i times", receiveFrequency);

    //创建ros句柄
    ros::NodeHandle nhp;

    // 声明发布消息类型
    ros::Publisher pubRosPCL = nhp.advertise<sensor_msgs::PointCloud2>("color_lidar", 10);
    ros::Rate loopRate(10);  // 发布频率为10赫兹

    CameraRadarMatrix matrixParam;
    std::string yamlFile = "/home/zhnagtianyi/test_ws/src/cubot_radar/config/param/camera_radar_matrix_param.yaml";
    CameraRadarMatrixParam::LoadFromYamlFile(yamlFile, &matrixParam);

    cv::Mat cameraMatrix = matrixParam.CameraMatrix;
    cv::Mat internalMatrix = matrixParam.InternalMatrix;
    cv::Mat distCoeffs = matrixParam.DistortionVector;
    cv::Mat externalMatrix = matrixParam.ExternalMatrix;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    LivoxRadarSubscriber::msgConvert(radarMsg, cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud1(new pcl::PointCloud<pcl::PointXYZI>);               //体素滤波器的输出点云
    PointCloudFilter::VoxelFilter(cloud, filteredCloud1, 0.05f);
    pcl::PointCloud<pcl::PointXYZI>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZI>);                  //地面滤波器的输出点云
    PointCloudFilter::GroundFilter(filteredCloud1,groundCloud,4.f,0.5f,0.5f);

    pcl::PointCloud<pcl::PointXYZI>::Ptr gatheringCloud(new pcl::PointCloud<pcl::PointXYZI>);               //聚类的点云
    LivoxRadarSubscriber livoxRadarSubscriber;
    livoxRadarSubscriber.clusterDBSCAN(groundCloud, gatheringCloud, 0.3, 10);

//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);               //着色的点云
//    livoxRadarSubscriber.pointColoring(gatheringCloud, color_cloud, internalMatrix, externalMatrix, rawImage);
//    发送出去到融合节点再着色

    // 显示接收到的点云数据
    viewer.showCloud(groundCloud);

    // 将处理后的点云以ros消息发布出去，便于在rviz上查看
    ROS_INFO("Start to publish the point cloud");
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*groundCloud, output);
    output.header.frame_id = "livox_frame"; //坐标系
    pubRosPCL.publish(output);
    loopRate.sleep();

    ROS_INFO("Finish all the process %i times", receiveFrequency);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "livox_lidar_subsriber");

    //创建ros句柄
    ros::NodeHandle nhs;

    ros::Subscriber livoxSub = nhs.subscribe("livox/lidar", 10, CustomMsgCall);

    // ros::spin()将会进入循环,一直调用回调函数,每次调用1000个数据。
    // 循环等待回调函数(若在循环内接收话题消息则使用ros::spinOnce)
    ros::spin();
    return 0;
}

