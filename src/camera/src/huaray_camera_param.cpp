//
// Created by plutoli on 2021/6/30.
//

#include "huaray_camera_param.h"

// 构造函数
HuarayCameraParam::HuarayCameraParam():
        Key(),
        RuntimeParam(),
        ModelParam(),
        HardwareParams()
{
}

// 从YAML配置文件中加载华睿相机参数
bool HuarayCameraParam::LoadFromYamlFile(const std::string &yamlFileName, HuarayCameraParam *huarayCameraParam)
{
    // 记录日志信息
    spdlog::info(LOG_BEGIN);

    // 判断YAML配置文件是否存在
    if (access(yamlFileName.c_str(), F_OK) == -1)
    {
        spdlog::error("HuarayCameraParam was loaded failure because yaml file is not existed");
        spdlog::info(LOG_END);
        return false;
    }

    // 判断YAML配置文件是否可读
    if (access(yamlFileName.c_str(), R_OK) == -1)
    {
        spdlog::error("HuarayCameraParam was loaded failure because yaml file can not be read");
        spdlog::info(LOG_END);
        return false;
    }

    // 创建并打开文件存储器
    cv::FileStorage fileStorage;
    if (!fileStorage.open(yamlFileName, cv::FileStorage::READ))
    {
        spdlog::error("HuarayCameraParam was loaded failure because yaml file can not be opened");
        spdlog::info(LOG_END);
        return false;
    }

    // 读取Key参数
    if ((!fileStorage["Key"].isNone()) && (fileStorage["Key"].isString()))
    {
        huarayCameraParam->Key = static_cast<std::string>(fileStorage["Key"]);
        spdlog::info("[" + huarayCameraParam->Key + "] -  HuarayCameraParam's Key was loaded successful");
    }
    else
    {
        spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraParam's Key was loaded failure");
    }

    // 读取相机的运行参数
    cv::FileNode runtimeParamNode = fileStorage["RuntimeParam"];
    if (!runtimeParamNode.empty())
    {
        // 读取IsOnline参数
        if ((!runtimeParamNode["IsOnline"].isNone()) && (runtimeParamNode["IsOnline"].isInt()))
        {
            huarayCameraParam->RuntimeParam.IsOnline = static_cast<int>(runtimeParamNode["IsOnline"]);
            spdlog::info("[" + huarayCameraParam->Key + "] - HuarayCameraRuntimeParam's IsOnline was loaded successful");
        }
        else
        {
            spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraRuntimeParam's IsOnline was loaded failure");
        }

        // 读取IsRecordVideo参数
        if ((!runtimeParamNode["IsRecordVideo"].isNone()) && (runtimeParamNode["IsRecordVideo"].isInt()))
        {
            huarayCameraParam->RuntimeParam.IsRecordVideo = static_cast<int>(runtimeParamNode["IsRecordVideo"]);
            spdlog::info("[" + huarayCameraParam->Key + "] - HuarayCameraRuntimeParam's IsRecordVideo was loaded successful");
        }
        else
        {
            spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraRuntimeParam's IsRecordVideo was loaded failure");
        }

        // 读取RecordVideoQuality参数
        if ((!runtimeParamNode["RecordVideoQuality"].isNone()) && (runtimeParamNode["RecordVideoQuality"].isInt()))
        {
            huarayCameraParam->RuntimeParam.RecordVideoQuality = static_cast<int>(runtimeParamNode["RecordVideoQuality"]);
            spdlog::info("[" + huarayCameraParam->Key + "] - HuarayCameraRuntimeParam's RecordVideoQuality was loaded successful");
        }
        else
        {
            spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraRuntimeParam's RecordVideoQuality was loaded failure");
        }

        // 读取OfflineVideoName参数
        if ((!runtimeParamNode["OfflineVideoName"].isNone()) && (runtimeParamNode["OfflineVideoName"].isString()))
        {
            huarayCameraParam->RuntimeParam.OfflineVideoName = static_cast<std::string>(runtimeParamNode["OfflineVideoName"]);
            spdlog::info("[" + huarayCameraParam->Key + "] - HuarayCameraRuntimeParam's OfflineVideoName was loaded successful");
        }
        else
        {
            spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraRuntimeParam's OfflineVideoName was loaded failure");
        }

        // 读取RecordVideoPath参数
        if ((!runtimeParamNode["RecordVideoPath"].isNone()) && (runtimeParamNode["RecordVideoPath"].isString()))
        {
            huarayCameraParam->RuntimeParam.RecordVideoPath = static_cast<std::string>(runtimeParamNode["RecordVideoPath"]);
            spdlog::info("[" + huarayCameraParam->Key + "] - HuarayCameraRuntimeParam's RecordVideoPath was loaded successful");
        }
        else
        {
            spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraRuntimeParam's RecordVideoPath was loaded failure");
        }
    }
    else
    {
        spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraRuntimeParam was loaded failure because it is empty");
    }

    // 读取相机的模型参数
    cv::FileNode modelParamNode = fileStorage["ModelParam"];
    if (!modelParamNode.empty())
    {
        // 读取CvInternalMatrix参数
        if ((!modelParamNode["CvInternalMatrix"].isNone()) && (modelParamNode["CvInternalMatrix"].isMap()))
        {
            modelParamNode["CvInternalMatrix"] >> huarayCameraParam->ModelParam.CvInternalMatrix;
            cv::cv2eigen(huarayCameraParam->ModelParam.CvInternalMatrix, huarayCameraParam->ModelParam.EigenInternalMatrix);
            spdlog::info("[" + huarayCameraParam->Key + "] - HuarayCameraModelParam's CvInternalMatrix was loaded successful");
        }
        else
        {
            spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraModelParam's CvInternalMatrix was loaded failure");
        }

        // 读取CvExternalMatrix参数
        if ((!modelParamNode["CvExternalMatrix"].isNone()) && (modelParamNode["CvExternalMatrix"].isMap()))
        {
            modelParamNode["CvExternalMatrix"] >> huarayCameraParam->ModelParam.CvExternalMatrix;
            cv::cv2eigen(huarayCameraParam->ModelParam.CvExternalMatrix, huarayCameraParam->ModelParam.EigenExternalMatrix.matrix());
            spdlog::info("[" + huarayCameraParam->Key + "] - HuarayCameraModelParam's CvExternalMatrix was loaded successful");
        }
        else
        {
            spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraModelParam's CvExternalMatrix was loaded failure");
        }

        // 读取CvDistortionVector参数
        if ((!modelParamNode["CvDistortionVector"].isNone()) && (modelParamNode["CvDistortionVector"].isMap()))
        {
            modelParamNode["CvDistortionVector"] >> huarayCameraParam->ModelParam.CvDistortionVector;
            cv::cv2eigen(huarayCameraParam->ModelParam.CvDistortionVector, huarayCameraParam->ModelParam.EigenDistortionVector);
            spdlog::info("[" + huarayCameraParam->Key + "] - HuarayCameraModelParam's CvDistortionVector was loaded successful");
        }
        else
        {
            spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraModelParam's CvDistortionVector was loaded failure");
        }
    }

    // 读取相机的硬件参数
    cv::FileNode hardwareParamsNode = fileStorage["HardwareParams"];
    if (!hardwareParamsNode.empty())
    {
        unsigned int index = 0;
        cv::FileNodeIterator iterator = hardwareParamsNode.begin();
        while (iterator != hardwareParamsNode.end())
        {
            // 创建硬件参数
            HuarayCameraHardwareParam hardwareParam;

            // 读取Scene参数
            if ((!(*iterator)["Scene"].isNone()) && ((*iterator)["Scene"].isInt()))
            {
                EHardwareParamScene scene;
                if (HuarayCameraHardwareParam::ConvertToHardwareParamScene(static_cast<int>((*iterator)["Scene"]), &scene))
                {
                    hardwareParam.Scene = scene;
                    spdlog::info("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s Scene was loaded successful", index);
                }
                else
                {
                    spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s Scene was converted failure", index);
                }
            }
            else
            {
                spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s Scene was loaded failure", index);
            }

            // 读取FrameRate参数
            if ((!(*iterator)["FrameRate"].isNone()) && ((*iterator)["FrameRate"].isReal()))
            {
                hardwareParam.FrameRate = static_cast<double>((*iterator)["FrameRate"]);
                spdlog::info("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s FrameRate was loaded successful", index);
            }
            else
            {
                spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s FrameRate was loaded failure", index);
            }

            // 读取IsExposureAuto参数
            if ((!(*iterator)["IsExposureAuto"].isNone()) && ((*iterator)["IsExposureAuto"].isInt()))
            {
                hardwareParam.IsExposureAuto = static_cast<int>((*iterator)["IsExposureAuto"]);
                spdlog::info("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s IsExposureAuto was loaded successful", index);
            }
            else
            {
                spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s IsExposureAuto was loaded failure", index);
            }

            // 读取ExposureTime参数
            if ((!(*iterator)["ExposureTime"].isNone()) && ((*iterator)["ExposureTime"].isReal()))
            {
                hardwareParam.ExposureTime = static_cast<double>((*iterator)["ExposureTime"]);
                spdlog::info("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s ExposureTime was loaded successful", index);
            }
            else
            {
                spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s ExposureTime was loaded failure", index);
            }

            // 读取GainRaw参数
            if ((!(*iterator)["GainRaw"].isNone()) && ((*iterator)["GainRaw"].isReal()))
            {
                hardwareParam.GainRaw = static_cast<double>((*iterator)["GainRaw"]);
                spdlog::info("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s GainRaw was loaded successful", index);
            }
            else
            {
                spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s GainRaw was loaded failure", index);
            }

            // 读取Gamma参数
            if ((!(*iterator)["Gamma"].isNone()) && ((*iterator)["Gamma"].isReal()))
            {
                hardwareParam.Gamma = static_cast<double>((*iterator)["Gamma"]);
                spdlog::info("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s Gamma was loaded successful", index);
            }
            else
            {
                spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s Gamma was loaded failure", index);
            }

            // 读取IsBalanceWhiteAuto参数
            if ((!(*iterator)["IsWhiteBalanceAuto"].isNone()) && ((*iterator)["IsWhiteBalanceAuto"].isInt()))
            {
                hardwareParam.IsWhiteBalanceAuto = static_cast<int>((*iterator)["IsWhiteBalanceAuto"]);
                spdlog::info("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s IsWhiteBalanceAuto was loaded successful", index);
            }
            else
            {
                spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s IsWhiteBalanceAuto was loaded failure", index);
            }

            // 读取BalanceRatio_R参数
            if ((!(*iterator)["BalanceRatio_R"].isNone()) && ((*iterator)["BalanceRatio_R"].isReal()))
            {
                hardwareParam.BalanceRatio_R = static_cast<double>((*iterator)["BalanceRatio_R"]);
                spdlog::info("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s BalanceRatio_R was loaded successful", index);
            }
            else
            {
                spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s BalanceRatio_R was loaded failure", index);
            }

            // 读取BalanceRatio_G参数
            if ((!(*iterator)["BalanceRatio_G"].isNone()) && ((*iterator)["BalanceRatio_G"].isReal()))
            {
                hardwareParam.BalanceRatio_G = static_cast<double>((*iterator)["BalanceRatio_G"]);
                spdlog::info("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s BalanceRatio_G was loaded successful", index);
            }
            else
            {
                spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s BalanceRatio_G was loaded failure", index);
            }

            // 读取BalanceRatio_B参数
            if ((!(*iterator)["BalanceRatio_B"].isNone()) && ((*iterator)["BalanceRatio_B"].isReal()))
            {
                hardwareParam.BalanceRatio_B = static_cast<double>((*iterator)["BalanceRatio_B"]);
                spdlog::info("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s BalanceRatio_B was loaded successful", index);
            }
            else
            {
                spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s BalanceRatio_B was loaded failure", index);
            }

            // 读取Width参数
            if ((!(*iterator)["Width"].isNone()) && ((*iterator)["Width"].isInt()))
            {
                hardwareParam.Width = static_cast<int>((*iterator)["Width"]);
                spdlog::info("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s Width was loaded successful", index);
            }
            else
            {
                spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s Width was loaded failure", index);
            }

            // 读取Height参数
            if ((!(*iterator)["Height"].isNone()) && ((*iterator)["Height"].isInt()))
            {
                hardwareParam.Height = static_cast<int>((*iterator)["Height"]);
                spdlog::info("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s Height was loaded successful", index);
            }
            else
            {
                spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s Height was loaded failure", index);
            }

            // 读取Offset_X参数
            if ((!(*iterator)["Offset_X"].isNone()) && ((*iterator)["Offset_X"].isInt()))
            {
                hardwareParam.Offset_X = static_cast<int>((*iterator)["Offset_X"]);
                spdlog::info("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s Offset_X was loaded successful", index);
            }
            else
            {
                spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s Offset_X was loaded failure", index);
            }

            // 读取Offset_Y参数
            if ((!(*iterator)["Offset_Y"].isNone()) && ((*iterator)["Offset_Y"].isInt()))
            {
                hardwareParam.Offset_Y = static_cast<int>((*iterator)["Offset_Y"]);
                spdlog::info("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s Offset_Y was loaded successful", index);
            }
            else
            {
                spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s Offset_Y was loaded failure", index);
            }

            // 读取Brightness参数
            if ((!(*iterator)["Brightness"].isNone()) && ((*iterator)["Brightness"].isInt()))
            {
                hardwareParam.Brightness = static_cast<int>((*iterator)["Brightness"]);
                spdlog::info("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s Brightness was loaded successful", index);
            }
            else
            {
                spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s Brightness was loaded failure", index);
            }

            // 读取IsSelected参数
            if ((!(*iterator)["IsSelected"].isNone()) && ((*iterator)["IsSelected"].isInt()))
            {
                hardwareParam.IsSelected = static_cast<int>((*iterator)["IsSelected"]);
                spdlog::info("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s IsSelected was loaded successful", index);
            }
            else
            {
                spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams[{}]'s IsSelected was loaded failure", index);
            }

            // 保存硬件参数
            huarayCameraParam->HardwareParams.emplace_back(hardwareParam);

            // 硬件参数索引和迭代器累加
            index++;
            iterator++;
        }
    }
    else
    {
        spdlog::error("[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams was loaded failure because it is empty");
    }

    // 关闭文件存储器
    fileStorage.release();
    // 记录日志信息
    spdlog::info("[" + huarayCameraParam->Key + "] - HuarayCameraParam was loaded successful");
    spdlog::info(LOG_END);

    // 返回加载结果
    return true;
}