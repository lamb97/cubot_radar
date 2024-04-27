//
// Created by godzhu on 2021/12/8.
//

#include "detector_param.h"

DetectorParam::DetectorParam() :
YOLOv5ConfThres(0.0),
YOLOv5IouThres(0.0),
YOLOv5CarModelPath(),
YOLOv5ArmorModelPath(),
YOLOv5ArmorNamePath()
{
}

DetectorParam::~DetectorParam()
{
    spdlog::info("Detector was initialized.");
}

bool DetectorParam::LoadDetectorParam(const std::string& yamlFileName) {
    // 判断YAML配置文件是否存在
    if(access(yamlFileName.c_str(), F_OK) == -1)
    {
        spdlog::error("DetectorParam was loaded failure because yaml file is not exist");
        return false;
    }

    // 判断YAML配置文件是否可读
    if(access(yamlFileName.c_str(), R_OK) == -1)
    {
        spdlog::error("DetectorParam was loaded failure because yaml file can not be read");
        return false;
    }

    // 创建并打开文件存储器
    cv::FileStorage fileStorage;
    if(!fileStorage.open(yamlFileName, cv::FileStorage::READ))
    {
        spdlog::error("DetectorParam was loaded failure because yaml file can not be opened");
        return false;
    }

    // 读取YOLOv5ConfThres参数
    if((!fileStorage["YOLOv5ConfThres"].isNone()))
    {
        fileStorage["YOLOv5ConfThres"] >> YOLOv5ConfThres;
    } else {
        spdlog::error("DetectorParam's YOLOv5ConfThres was loaded failure");
    }

    // 读取YOLOv5IouThres参数
    if((!fileStorage["YOLOv5IouThres"].isNone()))
    {
        fileStorage["YOLOv5IouThres"] >> YOLOv5IouThres;
    } else {
        spdlog::error("DetectorParam's YOLOv5IouThres was loaded failure");
    }

    // 读取YOLOv5IouThres参数
    if((!fileStorage["YOLOv5IouThres"].isNone()))
    {
        fileStorage["YOLOv5IouThres"] >> YOLOv5IouThres;
    } else {
        spdlog::error("DetectorParam's YOLOv5IouThres was loaded failure");
    }

    // 读取YOLOv5CarModelPath参数
    if((!fileStorage["YOLOv5CarModelPath"].isNone()) && (fileStorage["YOLOv5CarModelPath"].isString()))
    {
        fileStorage["YOLOv5CarModelPath"] >> YOLOv5CarModelPath;
    } else {
        spdlog::error("DetectorParam's YOLOv5Modelpath was loaded failure");
    }

    // 读取YOLOv5ArmorModelPath参数
    if((!fileStorage["YOLOv5ArmorModelPath"].isNone()) && (fileStorage["YOLOv5ArmorModelPath"].isString()))
    {
        fileStorage["YOLOv5ArmorModelPath"] >> YOLOv5ArmorModelPath;
    } else {
        spdlog::error("DetectorParam's YOLOv5ArmorModelPath was loaded failure");
    }


//    // 读取YOLOv5ArmorNamePath参数
//    if((!fileStorage["YOLOv5ArmorNamePath"].isNone()) && (fileStorage["YOLOv5ArmorNamePath"].isString()))
//    {
//        fileStorage["YOLOv5ArmorNamePath"] >> YOLOv5ArmorNamePath;
//    } else {
//        spdlog::error("DetectorParam's YOLOv5ArmorNamePath was loaded failure");
//    }

    return true;
}