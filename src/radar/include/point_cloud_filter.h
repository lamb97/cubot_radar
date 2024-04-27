//
// Created by godzhu on 2022/1/14.
//

#ifndef SRC_POINT_CLOUD_FILTER_H
#define SRC_POINT_CLOUD_FILTER_H

#include <iostream>
#include <fstream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>


class PointCloudFilter
{
public:

    // 构造函数
    PointCloudFilter() = default;

    // 析构函数
    ~PointCloudFilter() = default;

    /**
     * @brief 点云体素滤波器
     * @param[in] inputCloud   输入点云
     * @param[out] voxelCloud  输出点云
     * @param[in] leafSize     设置滤波时创建的体素体积为leafSize立方厘米的立方体
     */
    static void VoxelFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr &voxelCloud,
                            float leafSize);

    /**
     * @brief 点云形态学滤波器
     * @param[in] inputCloud     输入点云
     * @param[out] groundCloud   输出点云
     * @param[in] maxDistance    最大距离限制
     * @param[in] cellSize       细胞大小限制
     * @param[in] slope          点间坡度限制
     */
    static void GroundFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr &groundCloud,
                             float maxDistance,
                             float cellSize,
                             float slope);

};

#endif //SRC_POINT_CLOUD_FILTER_H
