//
// Created by godzhu on 2021/12/7.
//

#ifndef SRC_COORDINATE_TRANSFORMATION_H
#define SRC_COORDINATE_TRANSFORMATION_H

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include "spdlog/spdlog.h"

#include <opencv2/core.hpp>

class CoordinateTransformer {
public:

    /**
     * @brief 将雷达扫描得到的点由世界坐标系转换为像素坐标系
     * @param[in]  internalMatrix   相机的内参矩阵(3*3)
     * @param[in]  externalMatrix   相机雷达之间的外参矩阵(3*4)
     * @param[in]  pointsDatas      存放点云世界坐标的矩阵(4*pointsNum)
     * @param[out] pixelCoordinates 存放像素坐标的容器
     */
    static void WorldToPixel(const cv::Mat &internalMatrix,
                             const cv::Mat &externalMatrix,
                             const cv::Mat &pointsDatas,
                             std::vector <cv::Point2i> *pixelCoordinates) ;

    /**
     * @brief 对世界坐标点转换得到的像素坐标点赋值
     * @param[in]  pixelCoordinates  存放像素坐标的容器
     * @param[in]  rawImage          OpenCV格式的图像
     * @param[out] BGRValue          存放点云图像素值的容器
     */
    static void ColoredPointCloud(const std::vector<cv::Point2i> &pixelCoordinates,
                                  const cv::Mat &rawImage,
                                  std::vector<cv::Point3i> *BGRValue) ;


    static void pointColoring(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr &color_cloud,
                                              cv::Mat &internalMatrix,
                                              cv::Mat &externalMatrix,
                                              cv::Mat &rawImage) ;



    /**
     * @brief 将雷达扫描的点云投射到图像上
     * @param[in] pixelCoordinates 存放像素坐标的容器
     * @param[in] rawImage         OpenCV格式的图像
     * @param[in] cloud            PCL格式的点云
     */
    static void DrawDepthMap(const std::vector<cv::Point2i> &pixelCoordinates,
                             const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                             const cv::Mat &rawImage) ;

    /**
     * @brief 得到车辆图像中心点坐标对应的世界坐标
     * @param[in] carPixelCoordination 车辆图像中心点坐标
     * @param[in] pointsData           雷达扫描所有点的世界坐标
     * @param[in] pixelCoordinates     雷达扫描所有点投射到相机图像的图像坐标
     * @param[in] serchRange           搜索车辆图像中心点周围雷达图像点的半径
     * @return 车辆图像中心点坐标对应的世界坐标
     */
    static cv::Point3d CarWorldCoordination(const cv::Point2i &carPixelCoordination,
                                            const cv::Mat &pointsData,
                                            const std::vector<cv::Point2i> &pixelCoordinates,
                                            int searchRange);

//    static cv::Point3d toWorldPoint(const cv::Point2i &carPixelCoordination,const cv::Mat &pointsData);

//    static cv::Point3d Car2WorldCoordination(const cv::Point2i &carPixelCoordination,
//                                                             const cv::Mat &pointsData,
//                                                             const std::vector<cv::Point2i> &pixelCoordinates,
//                                                             const cv::Mat &externalMatrix,        //相机雷达外参 4*4
//                                                             const cv::Mat &CameraMatrix,          //相机内参 3*3 无奇次坐标
//                                                             const cv:: Mat &rotation,              //相机外参旋转矩阵
//                                                             const cv::Mat &translation,           //相机外参平移矩阵
//                                                             int searchRange,
//                                                             double zValueTolerance);

    static cv::Mat average_cameraPoint;


};

#endif //SRC_COORDINATE_TRANSFORMATION_H