//
// Created by plutoli on 2022/4/5.
//

#include "huaray_camera_hardware_param.h"

// ******************************  HuarayCameraHardwareParam类的公有函数  ******************************

// 构造函数

// 构造函数
HuarayCameraHardwareParam::HuarayCameraHardwareParam():
        Scene(EHardwareParamScene::Fight),
        FrameRate(200.0),
        IsExposureAuto(true),
        ExposureTime(10.0),
        GainRaw(1.0),
        Gamma(0.0),
        IsWhiteBalanceAuto(true),
        BalanceRatio_R(0.0),
        BalanceRatio_G(0.0),
        BalanceRatio_B(0.0),
        Width(800),
        Height(600),
        Offset_X(0),
        Offset_Y(0),
        Brightness(20),
        IsSelected(false)
{
}

// 转换适配场景
bool HuarayCameraHardwareParam::ConvertToHardwareParamScene(const int &input, EHardwareParamScene *output)
{
    // 初始化转换结果
    bool result = false;

    // 转换适配场景
    switch (input)
    {
        case 1:
            *output = EHardwareParamScene::Fight;
            result = true;
            break;

        case 2:
            *output = EHardwareParamScene::Buff;
            result = true;
            break;

        default:
            break;
    }

    // 返回转换结果
    return result;
}