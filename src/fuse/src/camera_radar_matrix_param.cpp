//
// Created by godzhu on 2021/12/2.
//

#include "camera_radar_matrix_param.h"

bool CameraRadarMatrixParam::LoadFromYamlFile(const std::string &yamlFileName,
                                              CameraRadarMatrix *matrixParam) {

    // 判断YAML配置文件是否存在
    if (access(yamlFileName.c_str(), F_OK) == -1) {
        spdlog::error("CameraRadarMatrixParam was loaded failure because yaml file is not exist");
        return false;
    }

    // 判断YAML配置文件是否可读
    if (access(yamlFileName.c_str(), R_OK) == -1) {
        spdlog::error("CameraRadarMatrixParam was loaded failure because yaml file can not be read");
        return false;
    }

    // 创建并打开文件存储器
    cv::FileStorage fileStorage;
    if (!fileStorage.open(yamlFileName, cv::FileStorage::READ))
    {
        spdlog::error("CameraRadarMatrixParam was loaded failure because yaml file can not be opened");
        return false;
    }
    // 读取CameraMatrix参数
    if ((!fileStorage["CameraMatrix"].isNone()) && (fileStorage["CameraMatrix"].isMap())) {
        fileStorage["CameraMatrix"] >> matrixParam->CameraMatrix;
    } else {
        spdlog::error("CameraRadarMatrixParam's CameraMatrix was loaded failure");
    }

    // 读取DistortionVector参数
    if ((!fileStorage["DistortionVector"].isNone()) && (fileStorage["DistortionVector"].isMap())) {
        fileStorage["DistortionVector"] >> matrixParam->DistortionVector;
    } else {
        spdlog::error("CameraRadarMatrixParam's DistortionVector was loaded failure");
    }

    // 读取InternalMatrix参数
    if ((!fileStorage["InternalMatrix"].isNone()) && (fileStorage["InternalMatrix"].isMap())) {
        fileStorage["InternalMatrix"] >> matrixParam->InternalMatrix;
    } else {
        spdlog::error("CameraRadarMatrixParam's InternalMatrix was loaded failure");
    }

    // 读取ExternalMatrix参数
    if ((!fileStorage["ExternalMatrix"].isNone()) && (fileStorage["ExternalMatrix"].isMap())) {
        fileStorage["ExternalMatrix"] >> matrixParam->ExternalMatrix;
    } else {
        spdlog::error("CameraRadarMatrixParam's ExternalMatrix was loaded failure");
    }
    return true;
}