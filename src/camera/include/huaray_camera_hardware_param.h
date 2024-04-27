//
// Created by plutoli on 2022/4/5.
//

#ifndef CUBOT_BRAIN_HUARAY_CAMERA_HARDWARE_PARAM_H
#define CUBOT_BRAIN_HUARAY_CAMERA_HARDWARE_PARAM_H

#include <string>
#include <vector>
#include "system_configurator.h"

/**
 * @brief 硬件参数适配的场景
 */
enum class EHardwareParamScene
{
    Fight = 1,     ///< 战斗场景
    Buff = 2       ///< Buff场景
};

/**
 * @brief 华睿相机的硬件参数
 */
class HuarayCameraHardwareParam
{
public:
    EHardwareParamScene Scene;  ///< 相机硬件参数适配的场景

    double FrameRate;           ///< 相机帧率，必须在最小帧率和最大帧率的范围内
    bool IsExposureAuto;        ///< 是否自动曝光，默认为true
    double ExposureTime;        ///< 曝光时间，必须在最小曝光时间和最大曝光时间的范围内

    double GainRaw;             ///< 相机的增益值，必须在最小增益值和最大增益值的范围内
    double Gamma;               ///< 相机的伽马值，必须在最小伽马值和最大伽马值的范围内
    bool IsWhiteBalanceAuto;    ///< 是否自动白平衡，默认为true
    double BalanceRatio_R;      ///< 红色平衡率，必须在最小平衡率和最大平衡率的范围内
    double BalanceRatio_G;      ///< 绿色平衡率，必须在最小平衡率和最大平衡率的范围内
    double BalanceRatio_B;      ///< 蓝色平衡率，必须在最小平衡率和最大平衡率的范围内

    int64_t Width;              ///< 图像宽度，必须在最小宽度和最大宽度的范围内
    int64_t Height;             ///< 图像高度，必须在最小高度和最大高度的范围内
    int64_t Offset_X;           ///< 图像X偏移，必须在最小偏移和最大偏移的范围内
    int64_t Offset_Y;           ///< 图像Y偏移，必须在最小偏移和最大偏移的范围内

    int64_t Brightness;         ///< 相机的亮度，必须在最大亮度和最小亮度的范围内

    bool IsSelected;            ///< 相机硬件参数的选中状态

    /**
     * @brief 构造函数
     */
    HuarayCameraHardwareParam();

    /**
     * @brief 析构函数
     */
    ~HuarayCameraHardwareParam() = default;

    /**
   * @brief 转换适配场景
   * @param[in]   input   输入的硬件参数适配场景数值
   * @param[out]  output  转换得到的硬件参数适配场景
   * @return 适配场景转换结果\n
   *         -<em>false</em> 适配场景转换失败\n
   *         -<em>true</em> 适配场景转换成功\n
   * @note 适配场景的取值为1/2，输入数据不在此范围内，则转换失败\n
   *         -<em>1</em> Fight\n
   *         -<em>2</em> Buff\n
   */
    static bool ConvertToHardwareParamScene(const int &input, EHardwareParamScene *output);
};

#endif //CUBOT_BRAIN_HUARAY_CAMERA_HARDWARE_PARAM_H