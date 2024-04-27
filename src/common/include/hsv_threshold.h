//
// Created by plutoli on 2021/8/17.
//

#ifndef CUBOT_BRAIN_HSV_THRESHOLD_H
#define CUBOT_BRAIN_HSV_THRESHOLD_H

/**
 * @brief 图像的HSV三通道阈值
 */
class HSVThreshold
{
public:
    unsigned char HueLower;          ///< 色相阈值下限
    unsigned char HueUpper;          ///< 色相阈值上限
    unsigned char SaturationLower;   ///< 饱和度阈值下限
    unsigned char SaturationUpper;   ///< 饱和度阈值上限
    unsigned char ValueLower;        ///< 亮度阈值下限
    unsigned char ValueUpper;        ///< 亮度阈值上限

    /**
     * @brief 构造函数
     */
    HSVThreshold();

    /**
     * @brief 析构函数
     */
    ~HSVThreshold() = default;
};

#endif //CUBOT_BRAIN_HSV_THRESHOLD_H