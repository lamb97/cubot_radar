//
// Created by godzhu on 2021/12/22.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include "spdlog/spdlog.h"
#include <rosbag/bag.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include "camera_radar_matrix_param.h"
#include "coordinate_transformer.h"

int main(int argc, char **argv) {
    // 加载参数
    CameraRadarMatrix matrixParam;
    std::string yamlFile = "/home/zhangtianyi/test_ws/src/cubot_radar/config/param/camera_radar_matrix_param.yaml";
    CameraRadarMatrixParam::LoadFromYamlFile(yamlFile, &matrixParam);

    cv::Mat cameraMatrix = matrixParam.CameraMatrix;
    cv::Mat internalMatrix = matrixParam.InternalMatrix;
    cv::Mat distCoeffs = matrixParam.DistortionVector;
    cv::Mat externalMatrix = matrixParam.ExternalMatrix;

    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    cv::namedWindow("depthImage", cv::WINDOW_NORMAL);
    cv::Mat rawImage;
    rawImage = cv::imread("/home/zhangtianyi/test_ws/src/cubot_radar/data/data1/picture/6.bmp");
    if (rawImage.empty()) {
        spdlog::error("rawImage does not exist !");
        return -1;
    }

    cv::Mat emptyImage(rawImage.rows, rawImage.cols, CV_8U);

    std::string cloudFilename("/home/zhangtianyi/test_ws/src/cubot_radar/data/data1/pcd/calibration_scene.pcd");

//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);  //点云指针对象
    typedef pcl::PointXYZ PointType;
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

    std::vector<float> depthValue;
    cv::Mat pointData(4, pointNum, CV_64F);

    for (int i = 0; i < pointNum; ++i) {

        float x = cloud->points[i].x;
        float y = cloud->points[i].y;
        float z = cloud->points[i].z;
        pointData.at<double>(0, i) = x;
        pointData.at<double>(1, i) = y;
        pointData.at<double>(2, i) = z;
        pointData.at<double>(3, i) = 1;

        std::cout << "point.x1: " << x << std::endl;
        std::cout << "point.y1: " << y << std::endl;
        std::cout << "point.z1: " << z << std::endl;

    }

    // 点云世界坐标转换为像素坐标
    std::vector<cv::Point2i> pixelCoordinates;
    CoordinateTransformer::WorldToPixel(internalMatrix, externalMatrix, pointData, &pixelCoordinates);

    // 将雷达扫描的点云投射到图像上
    CoordinateTransformer::DrawDepthMap(pixelCoordinates, cloud, rawImage);

    // 显示接收到的点云数据
    viewer.showCloud(cloud);
    cv::imshow("depthImage", rawImage);

    cv::waitKey(1000000);
    return 0;

}
