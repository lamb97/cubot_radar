//
// Created by godzhu on 2021/12/2.
//

#ifndef SRC_CAMERA_RADAR_MATRIX_PARAM_H
#define SRC_CAMERA_RADAR_MATRIX_PARAM_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "spdlog/spdlog.h"
#include "log_message.h"
#include "camera_radar_matrix.h"

/**
 * @brief 相机和雷达矩阵参数
 */
class CameraRadarMatrixParam
{
public:
    /**
     * @brief 构造函数
     */
    CameraRadarMatrixParam() = default;

    /**
     * @brief 析构函数
     */
    ~CameraRadarMatrixParam() = default;

    /**
     * @brief 从YAML配置文件中加载相机和雷达矩阵参数
     * @param[in]  yamlFileName           相机和雷达矩阵参数配置文件名称
     * @param[out] cameraRadarMatrixParam 相机和雷达矩阵参数
     * @return 参数加载结果\n
     *         -<em>false</em> 参数加载失败\n
     *         -<em>true</em>  参数加载成功\n
     */
    static bool LoadFromYamlFile(const std::string &yamlFileName, CameraRadarMatrix *matrixParam);
};

#endif //SRC_CAMERA_RADAR_MATRIX_PARAM_H
