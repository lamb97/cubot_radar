//
// Created by godzhu on 2021/12/2.
//

#ifndef SRC_CAMERA_RADAR_MATRIX_H
#define SRC_CAMERA_RADAR_MATRIX_H

#include <opencv2/opencv.hpp>

class CameraRadarMatrix
{
public:
    cv::Mat CameraMatrix;                              ///< 相机的内参矩阵(3x3)
    cv::Mat InternalMatrix;                            ///< 相机的内参矩阵(3x4)
    cv::Mat ExternalMatrix;                            ///< 相机与雷达的外参矩阵(4x4)
    cv::Mat DistortionVector;                          ///< 相机的畸变校正向量

    /**
    * @brief 构造函数
    */
    CameraRadarMatrix();

    /**
     * @brief 析构函数
     */
    ~CameraRadarMatrix() = default;

};

#endif //SRC_CAMERA_RADAR_MATRIX_H