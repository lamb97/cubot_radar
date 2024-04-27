#ifndef SRC_LIVOX_RADAR_SUBSCRIBER_H
#define SRC_LIVOX_RADAR_SUBSCRIBER_H

#include <iostream>
#include "spdlog/spdlog.h"
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <pcl_ros/point_cloud.h>
#include <cubot_radar/CustomMsg.h>
#include <cubot_radar/CustomPoint.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <pcl/common/geometry.h>
#include "coordinate_transformer.h"



class LivoxRadarSubscriber {
private:
    std::vector<std::vector<double>> distanceMatrix;    ///< 添加缓存距离矩阵

     /**
     * @brief                       从指定点根据邻域半径和最小点数进行点的聚类
     * @param[in] cloud             输入点云
     * @param[in] idx               起始点的索引，表示算法从哪个点开始进行聚类
     * @param[in] clusterId         当前聚类的标识符，用于标记属于同一聚类的点
     * @param[in] epsilon           DBSCAN聚类中的邻域半径
     * @param[in] minPoints         DBSCAN聚类中的最小点数要求
     * @param[in] kdtree            KD树数据结构，用于加速近邻搜索
     */
    void setArrivalPointsAndCluster(int idx,
                                    int clusterId,
                                    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                    double epsilon,
                                    int minPoints,
                                    pcl::KdTreeFLANN<pcl::PointXYZI> &kdtree);

public:

    // 构造函数
    LivoxRadarSubscriber();

    // 析构函数
    ~LivoxRadarSubscriber() = default;

    /**
    * @brief                         将PCL点云数据转换为有点坐标和强度信息的PCLXYZI数据
    * @param[in] msg                 输入点云
    * @param[out] cloud              输出点云
    */
    static void msgConvert(const cubot_radar::CustomMsg::ConstPtr &msg,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);

    /**
    * @brief                       对点云中的每个点进行遍历并调用setArrivalPointsAndCluster
    * @param[in] cloud             输入点云
    * @param[out] clusteredCloud   输出点云
    * @param[in] epsilon           DBSCAN聚类中的邻域半径
    * @param[in] minPoints         DBSCAN聚类中的最小点数要求
    */
    void clusterDBSCAN(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr &clusteredCloud,
                       double epsilon,
                       int minPoints);

    /**
    * @brief                       对点云着色
    * @param[in] cloud             输入XYZI点云
    * @param[out] color_cloud      输出XYZRGB点云
    * @param[in] internalMatrix    相机内参
    * @param[in] externalMatrix    相机外参
    * @param[in] rawImage          相机图像
    */
    void pointColoring (const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &color_cloud,
                        cv::Mat &internalMatrix,
                        cv::Mat &externalMatrix,
                        cv::Mat &rawImage);
};

#endif //SRC_LIVOX_RADAR_SUBSCRIBER_H