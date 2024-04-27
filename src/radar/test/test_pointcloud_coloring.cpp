//
// Created by godzhu on 2021/12/4.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include "spdlog/spdlog.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include "camera_radar_matrix_param.h"
#include "delta_cv.h"

// 将雷达扫描得到的点由世界坐标系转换为像素坐标系
void WorldToPixel(const cv::Mat &internalMatrix, const cv::Mat &externalMatrix, const cv::Mat &pointDatas,
                  std::vector<cv::Point2f> *UV) {
    // 相机世界坐标系转像素坐标系矩阵运算关系
    cv::Mat result = internalMatrix * externalMatrix * pointDatas;
    cv::Point2f singlePoint;
    for (int i = 0; i < result.cols; i++) {
        auto u = (float) result.at<double>(0, i);
        auto v = (float) result.at<double>(1, i);
        auto depth = (float) result.at<double>(2, i);
        singlePoint.x = u / depth;
        singlePoint.y = v / depth;
        UV->push_back(singlePoint);
    }
}

// 对世界坐标点转换得到的像素坐标点赋值
void ColoredPointCloud(const cv::Mat &internalMatrix,
                   const cv::Mat &externalMatrix,
                   const cv::Mat &pointDatas,
                   int row, int col,
//                   const std::vector<std::vector<int>> &pixelValue,
                   const cv::Mat& rawImage,
                   std::vector<cv::Point3i> *RGB) {

    std::vector<cv::Point2f> UV;
    WorldToPixel(internalMatrix, externalMatrix, pointDatas, &UV);  // 世界坐标转像素坐标

    cv::Point3i singleRGB;
    for (auto &i : UV) {
        int u = int(i.x);
        int v = int(i.y);
        int32_t index = v * col + u;
        if (u < col && u > 0 && v < row && v > 0) {
//        if (index > 0 && index < row * col) {
            singleRGB.x = rawImage.at<cv::Vec3b>(v, u)[0];
            singleRGB.y = rawImage.at<cv::Vec3b>(v, u)[1];
            singleRGB.z = rawImage.at<cv::Vec3b>(v, u)[2];
//            singleRGB.x = pixelValue[index][0];
//            singleRGB.y = pixelValue[index][1];
//            singleRGB.z = pixelValue[index][2];
            RGB->push_back(singleRGB);
        } else {
            singleRGB.x = 0;
            singleRGB.y = 0;
            singleRGB.z = 0;
            RGB->push_back(singleRGB);
        }
    }

}

int main(int argc, char **argv) {
    CameraRadarMatrix matrixParam;
    std::string yamlFile = "/home/zhangtianyi/test_ws/src/cubot_radar/config/param/camera_radar_matrix_param.yaml";
    CameraRadarMatrixParam::LoadFromYamlFile(yamlFile, &matrixParam);

    cv::Mat cameraMatrix = matrixParam.CameraMatrix;
    cv::Mat internalMatrix = matrixParam.InternalMatrix;
    cv::Mat distCoeffs = matrixParam.DistortionVector;
    cv::Mat externalMatrix = matrixParam.ExternalMatrix;

    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    cv::namedWindow("rawImage", cv::WINDOW_NORMAL);
    cv::Mat rawImage;
    rawImage = cv::imread("/home/zhangtianyi/test_ws/src/cubot_radar/data/data1/picture/6.bmp");
    if (rawImage.empty()) {
        spdlog::error("rawImage does not exist !");
        return -1;
    }

//    cv::Mat map1, map2;
//    cv::Size imageSize = rawImage.size();
//
//    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
//                                cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, nullptr),
//                                imageSize, CV_16SC2, map1, map2);
//
//    cv::remap(rawImage, rawImage, map1, map2, cv::INTER_LINEAR);  // 畸变校正
//
//    cv::imwrite("/home/godzhu/6_remap.bmp", rawImage);

    // 修正后图片的高和宽像素值数量
    int row = rawImage.rows;
    int col = rawImage.cols;

    std::vector<cv::Mat> pixelVector;
//     存放像素值的容器初始化
    std::vector<std::vector<int>> pixelValue;
    DeltaCV::getPixelValue(rawImage, &pixelValue);

    std::string cloudFilename("/home/zhangtianyi/test_ws/src/cubot_radar/data/data1/pcd/calibration_scene.pcd");

//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);  //点云指针对象
    typedef pcl::PointXYZI PointType;
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);

    if (pcl::io::loadPCDFile(cloudFilename, *cloud)) {
        spdlog::error("Cannot open file {0} !", cloudFilename);
        return -1;
    }

    cloud->is_dense = false;
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->points.resize(cloud->width);

    auto pointNum = (int) cloud->points.size();

    cv::Mat pointData(4, pointNum, CV_64F);

    for (int i = 0; i < pointNum; ++i) {

        float x = cloud->points[i].x;
        float y = cloud->points[i].y;
        float z = cloud->points[i].z;

        pointData.at<double>(0, i) = x;
        pointData.at<double>(1, i) = y;
        pointData.at<double>(2, i) = z;
        pointData.at<double>(3, i) = 1;
    }

    // 给点云赋RGB值
    std::vector<cv::Point3i> RGB;
    ColoredPointCloud(internalMatrix, externalMatrix, pointData, row, col, rawImage, &RGB);

    for (int i = 0; i < pointNum; ++i) {

        // 忽视无效的点
        if (pointData.at<double>(0, i) == 0 && pointData.at<double>(1, i) == 0 && pointData.at<double>(2, i) == 0) {
            continue;
        }
        // 忽视RGB值都为0的点
        if (RGB[i].x == 0 && RGB[i].y == 0 && RGB[i].z == 0) {
            continue;
        }
        // 给点云赋予XYZ的值
        cloud->points[i].x = (float) pointData.at<double>(0, i);
        cloud->points[i].y = (float) pointData.at<double>(1, i);
        cloud->points[i].z = (float) pointData.at<double>(2, i);
/*
        // 给点云赋予RGB值（给点云赋值的操作需要一个循环里进行，才能完成点的对应）
        cloud->points[i].r = RGB[i].z;
        cloud->points[i].g = RGB[i].y;
        cloud->points[i].b = RGB[i].x;*/
    }
    // 显示接收到的点云数据
    viewer.showCloud(cloud);
    cv::imshow("rawImage", rawImage);
    cv::waitKey(1000000);
    return 0;

}




















//
//
//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/registration/icp.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl_ros/point_cloud.h>
//#include <ros/ros.h>
//#include <tf2_ros/transform_broadcaster.h>
//#include <tf2_ros/transform_listener.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <geometry_msgs/TransformStamped.h>
//#include <Eigen/Dense>
//
//typedef pcl::PointXYZ PointT;
//typedef pcl::PointCloud<PointT> PointCloud;
//bool start_flag = 1;
//void doubleLocation();
//
//void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input_cloud_msg) {
//
//    // 确定是不是第一次、是不是飘了，否则不启动
//    // if() start_flag = flag; else if() start_flag = flag;
//
//    if(start_flag) {
//        // 将ROS消息类型的点云转换为PCL类型
//        PointCloud::Ptr input_cloud(new PointCloud);
//        pcl::fromROSMsg(*input_cloud_msg, *input_cloud);
//
//        // 读取地图点云
//        PointCloud::Ptr map_cloud(new PointCloud);
//        pcl::io::loadPCDFile<PointT>("/home/lu/map.pcd", *map_cloud);
//
//        // 配准算法
//        pcl::IterativeClosestPoint<PointT, PointT> icp;
//        icp.setInputSource(input_cloud);
//        icp.setInputTarget(map_cloud);
//        PointCloud::Ptr registered(new PointCloud);
//        icp.align(*registered);
//
//        // 获取最终的变换矩阵
//        Eigen::Matrix4f final_transform = icp.getFinalTransformation();
//
//        // 将 Eigen::Matrix4f 转换为 geometry_msgs::TransformStamped 消息
//        geometry_msgs::TransformStamped transformStamped;
//        transformStamped.header.stamp = ros::Time::now();
//        transformStamped.header.frame_id = "map_new";  // 地图坐标系
//        transformStamped.child_frame_id = "laser_new"; // 激光雷达坐标系
//
//        transformStamped.transform.translation.x = final_transform(0, 3);
//        transformStamped.transform.translation.y = final_transform(1, 3);
//        transformStamped.transform.translation.z = final_transform(2, 3);
//
//        Eigen::Matrix3f rotation = final_transform.block<3, 3>(0, 0);
//        Eigen::Quaternionf q(rotation);
//        transformStamped.transform.rotation.x = q.x();
//        transformStamped.transform.rotation.y = q.y();
//        transformStamped.transform.rotation.z = q.z();
//        transformStamped.transform.rotation.w = q.w();
//
//        // 创建 TransformBroadcaster
//        tf2_ros::TransformBroadcaster tfBroadcaster;
//        // 发布 TransformStamped 消息到话题
//        tfBroadcaster.sendTransform(transformStamped);
//
//        doubleLocation(); // 调用重定位函数
//    }
//}
//
//int main(int argc, char** argv) {
//    ros::init(argc, argv, "loc");
//    ros::NodeHandle nh;
//    // 订阅点云话题
//    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 1, pointcloudCallback);
//    ros::spin();
//    return 0;
//}
//
//void doubleLocation() {
//    // 假设地图到激光雷达的变换矩阵 T_map_to_laser
//    Eigen::Matrix4f T_map_to_laser;
//    // 示例数据
//    T_map_to_laser <<  0.9987, -0.0012, 0.0510, 1.23,
//            0.0011, 0.9999, -0.0100, -0.32,
//            -0.0510, 0.0100, 0.9987, 0.45,
//            0, 0, 0, 1;
//
//    // 从变换矩阵中获取激光雷达在本地坐标系中的位置和姿态
//    Eigen::Vector3f position_before = T_map_to_laser.block<3, 1>(0, 3);
//    Eigen::Matrix3f rotation_before = T_map_to_laser.block<3, 3>(0, 0);
//
//    std::cout << "更新前激光雷达在本地坐标系中的位置：" << std::endl << position_before << std::endl;
//    std::cout << "更新前激光雷达在本地坐标系中的姿态：" << std::endl << rotation_before << std::endl;
//
////    T_map_to_laser = T_map_to_laser * icp.getFinalTransformation();
//
//    // 创建一个TransformListener对象
//    tf2_ros::Buffer tf_buffer;
//    tf2_ros::TransformListener tf_listener(tf_buffer);
//    // 定义要查询的父子坐标系
//    std::string parent_frame = "map_new";
//    std::string child_frame = "laser_new";
//    geometry_msgs::TransformStamped transformStamped = tf_buffer.lookupTransform(parent_frame, child_frame, ros::Time(0));
//    // 将TransformStamped类型的变量转换为Eigen类型的变换矩阵
//    Eigen::Affine3f transformEigen;
//    tf2::fromMsg(transformStamped.transform, transformEigen);// 消息对象转换成Affine3f，Affine3f是4*4的
//    Eigen::Matrix4f transformMatrix = transformEigen.matrix(); // 转换成Matrix4f
//    T_map_to_laser = T_map_to_laser * transformMatrix; // 更新新的雷达相对于世界坐标系的姿态位置
//
//    // 重定位出新的T_map_to_laser发出去
//    // 将 Eigen::Matrix4f 转换为 geometry_msgs::TransformStamped 消息
//    geometry_msgs::TransformStamped transformStamped1;
//    transformStamped1.header.stamp = ros::Time::now();
//    transformStamped1.header.frame_id = "map";  // 地图坐标系
//    transformStamped1.child_frame_id = "laser"; // 激光雷达坐标系
//
//    transformStamped1.transform.translation.x = T_map_to_laser(0, 3);
//    transformStamped1.transform.translation.y = T_map_to_laser(1, 3);
//    transformStamped1.transform.translation.z = T_map_to_laser(2, 3);
//
//    Eigen::Matrix3f rotation = T_map_to_laser.block<3, 3>(0, 0);
//    Eigen::Quaternionf q(rotation);
//    transformStamped1.transform.rotation.x = q.x();
//    transformStamped1.transform.rotation.y = q.y();
//    transformStamped1.transform.rotation.z = q.z();
//    transformStamped1.transform.rotation.w = q.w();
//
//    // 创建 TransformBroadcaster
//    tf2_ros::TransformBroadcaster tfBroadcaster1;
//    // 发布 TransformStamped 消息到话题
//    tfBroadcaster1.sendTransform(transformStamped1);
//
//    // 从变换矩阵中获取激光雷达在本地坐标系中的位置和姿态
//    Eigen::Vector3f position_after = T_map_to_laser.block<3, 1>(0, 3);
//    Eigen::Matrix3f rotation_after = T_map_to_laser.block<3, 3>(0, 0);
//
//    std::cout << "更新后激光雷达在本地坐标系中的位置：" << std::endl << position_after << std::endl;
//    std::cout << "更新后激光雷达在本地坐标系中的姿态：" << std::endl << rotation_after << std::endl;
//}