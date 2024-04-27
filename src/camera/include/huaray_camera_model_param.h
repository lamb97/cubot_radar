//
// Created by plutoli on 2022/4/5.
//

#ifndef CUBOT_BRAIN_HUARAY_CAMERA_MODEL_PARAM_H
#define CUBOT_BRAIN_HUARAY_CAMERA_MODEL_PARAM_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

/**
 * @brief 华睿相机的模型参数；包括内参矩阵、外参矩阵、畸变系数向量
 * @note 相机模型原理资料如下：\n
 *       https://blog.csdn.net/weixin_39928787/article/details/111117833 \n
 *       https://blog.csdn.net/weixin_43206570/article/details/84797361 \n
 *       https://blog.csdn.net/reasonyuanrobot/article/details/86614381 \n
 */
class HuarayCameraModelParam
{
public:
    cv::Mat CvInternalMatrix;                               ///< 华睿相机OpenCV格式的内参矩阵
    cv::Mat CvExternalMatrix;                               ///< 华睿相机OpenCV格式的外参矩阵
    cv::Mat CvDistortionVector;                             ///< 华睿相机OpenCV格式的畸变系数向量
    Eigen::Matrix3d EigenInternalMatrix;                    ///< 华睿相机Eigen格式的内参矩阵
    Eigen::Isometry3d EigenExternalMatrix;                  ///< 华睿相机Eigen格式的外参矩阵
    Eigen::Matrix<double, 5, 1> EigenDistortionVector;       ///< 华睿相机Eigen格式的畸变系数向量

    /**
    * @brief 构造函数
    */
    HuarayCameraModelParam();

    /**
     * @brief 析构函数
     */
    ~HuarayCameraModelParam() = default;
};

#endif //CUBOT_BRAIN_HUARAY_CAMERA_MODEL_PARAM_H