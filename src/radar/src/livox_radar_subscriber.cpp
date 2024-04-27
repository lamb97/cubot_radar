//
// Created by godzhu on 11/16/21.
//

#include "livox_radar_subscriber.h"

LivoxRadarSubscriber::LivoxRadarSubscriber()
{
    spdlog::info("LivoxRadarSubscriber was initialized successful");
}

// 将PCL点云数据转换为有点坐标和强度信息的PCLXYZI数据
void LivoxRadarSubscriber::msgConvert(const cubot_radar::CustomMsg::ConstPtr &msg,
                                      pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {

    cloud->width = msg->points.size();
    cloud->height = 1;
    cloud->resize(cloud->width);

    for (size_t i = 0; i < msg->points.size(); i++) {
        pcl::PointXYZI pointXyzi;
        pointXyzi.x = msg->points[i].x;
        pointXyzi.y = msg->points[i].y;
        pointXyzi.z = msg->points[i].z;
        pointXyzi.intensity = msg->points[i].reflectivity;
        cloud->push_back(pointXyzi);
    }
}

// 从指定点根据邻域半径和最小点数进行点的聚类
void LivoxRadarSubscriber::setArrivalPointsAndCluster(int idx,
                                                      int clusterId,
                                                      const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                                      double epsilon,
                                                      int minPoints,
                                                      pcl::KdTreeFLANN<pcl::PointXYZI> &kdtree) {
    std::vector<int> arrivalPoints;

    // 使用 kd 树进行近邻搜索
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    if (kdtree.radiusSearch(cloud->points[idx], epsilon, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
        for (int i : pointIdxRadiusSearch) {
            if (i == idx)
                continue;

            arrivalPoints.push_back(i);
        }
    }

    if (arrivalPoints.size() >= minPoints) {
        for (int point : arrivalPoints) {
            setArrivalPointsAndCluster(point, clusterId, cloud, epsilon, minPoints, kdtree);
        }
    }
}

// 对点云中的每个点进行遍历并调用setArrivalPointsAndCluster
void LivoxRadarSubscriber::clusterDBSCAN(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                         pcl::PointCloud<pcl::PointXYZI>::Ptr &clusteredCloud,
                                         double epsilon,
                                         int minPoints) {
    clusteredCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    clusteredCloud->header = cloud->header;

    // 初始化距离矩阵
    distanceMatrix.resize(cloud->points.size(), std::vector<double>(cloud->points.size(), -1.0));

    // 创建 kd 树
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud);

    std::vector<bool> isKey(cloud->points.size(), false);
    std::vector<int> clusterId(cloud->points.size(), -1);

    int clusterIdCounter = 0;

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        if (!isKey[i] && clusterId[i] == -1) {
            int currentClusterId = clusterIdCounter++;
            clusterId[i] = currentClusterId;
            setArrivalPointsAndCluster(static_cast<int>(i), currentClusterId, cloud, epsilon, minPoints, kdtree);
        }
    }
}

void LivoxRadarSubscriber::pointColoring(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr &color_cloud,
                                         cv::Mat &internalMatrix,
                                         cv::Mat &externalMatrix,
                                         cv::Mat &rawImage) {
    // 将点云中所有点载入矩阵
    cv::Mat pointsData(4, cloud->size(), CV_64F);
    int pointNum = static_cast<int>(cloud->points.size());
    for (int i = 0; i < pointNum; ++i) {
        pointsData.at<double>(0, i) = cloud->points[i].x;
        pointsData.at<double>(1, i) = cloud->points[i].y;
        pointsData.at<double>(2, i) = cloud->points[i].z;
        pointsData.at<double>(3, i) = 1;
    }

    // 将点云世界坐标转为像素坐标
    std::vector<cv::Point2i> pixelCoordinates;
    CoordinateTransformer::WorldToPixel(internalMatrix, externalMatrix, pointsData, &pixelCoordinates);

    // 给点云赋BGR值
    std::vector<cv::Point3i> BGRValue;
    CoordinateTransformer::ColoredPointCloud(pixelCoordinates, rawImage, &BGRValue);

    // 给点云赋予XYZ和BGR的值
    for (int i = 0; i < pointNum; ++i) {
        if (!(pointsData.at<double>(0, i) == 0 && pointsData.at<double>(1, i) == 0 && pointsData.at<double>(2, i) == 0) &&
            !(BGRValue[i].x == 0 && BGRValue[i].y == 0 && BGRValue[i].z == 0)) {
            color_cloud->points[i].x = static_cast<float>(pointsData.at<double>(0, i));
            color_cloud->points[i].y = static_cast<float>(pointsData.at<double>(1, i));
            color_cloud->points[i].z = static_cast<float>(pointsData.at<double>(2, i));

            color_cloud->points[i].r = BGRValue[i].z;
            color_cloud->points[i].g = BGRValue[i].y;
            color_cloud->points[i].b = BGRValue[i].x;
        }
    }
}



