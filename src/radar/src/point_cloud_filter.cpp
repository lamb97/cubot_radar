
//
// Created by godzhu on 2022/1/14.
//

#include "point_cloud_filter.h"

void PointCloudFilter::VoxelFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud,
                                   pcl::PointCloud<pcl::PointXYZI>::Ptr &voxelCloud,
                                   float leafSize)
{
    // 创建点云体素滤波器
    pcl::VoxelGrid<pcl::PointXYZI> voxelGrid;
    voxelGrid.setInputCloud(inputCloud);

    // 设置滤波器参数
    voxelGrid.setLeafSize(leafSize, leafSize, leafSize);

    // 输出滤波后点云
    voxelGrid.filter(*voxelCloud);
}

void PointCloudFilter::GroundFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud,
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr &groundCloud,
                                    float maxDistance,
                                    float cellSize,
                                    float slope)
{
    pcl::PointIndicesPtr ground(new pcl::PointIndices);

    // 创建点云形态学滤波器
    pcl::ApproximateProgressiveMorphologicalFilter<pcl::PointXYZI> apmf;
    apmf.setInputCloud(inputCloud);

    // 设置滤波器参数
    apmf.setMaxWindowSize(20);
    apmf.setMaxDistance(maxDistance);
    apmf.setInitialDistance(0.15f);
    apmf.setCellSize(cellSize);
    apmf.setSlope(slope);
    apmf.setBase(2.0f);
    apmf.setExponential(true);

    apmf.extract(ground->indices);

    // 创建过滤对象并提取地面回波
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(inputCloud);
    extract.setIndices(ground);
    extract.filter(*groundCloud);
}

