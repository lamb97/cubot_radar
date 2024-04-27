//
// Created by plutoli on 2022/4/5.
//

#ifndef CUBOT_BRAIN_HUARAY_CAMERA_PARAM_H
#define CUBOT_BRAIN_HUARAY_CAMERA_PARAM_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "easy_logger.h"
#include "huaray_camera_runtime_param.h"
#include "huaray_camera_model_param.h"
#include "huaray_camera_hardware_param.h"

/**
 * @brief 华睿相机参数
 */
class HuarayCameraParam
{
public:
    std::string Key;                                          ///< 华睿相机的标识符
    HuarayCameraRuntimeParam RuntimeParam;                    ///< 华睿相机的运行时参数
    HuarayCameraModelParam ModelParam;                        ///< 华睿相机的模型参数
    std::vector<HuarayCameraHardwareParam> HardwareParams;    ///< 华睿相机的硬件参数集合

    /**
     * @brief 构造函数
     */
    HuarayCameraParam();

    /**
     * @brief 析构函数
     */
    ~HuarayCameraParam() = default;

    /**
     * @brief 从yaml配置文件中加载华睿相机参数
     * @param[in]  yamlFileName         华睿相机参数配置文件名
     * @param[out] huarayCameraParam    华睿相机参数
     * @return 加载结果\n
     *         -<em>false</em> 加载失败\n
     *         -<em>true</em> 加载成功\n
     */
    static bool LoadFromYamlFile(const std::string &yamlFileName, HuarayCameraParam *huarayCameraParam);
};

#endif //CUBOT_BRAIN_HUARAY_CAMERA_PARAM_H