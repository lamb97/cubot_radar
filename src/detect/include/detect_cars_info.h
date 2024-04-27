//
// Created by godzhu on 2021/12/24.
//

#ifndef SRC_DETECT_CARS_INFO_H
#define SRC_DETECT_CARS_INFO_H

#include <string>
#include <vector>
#include "detect_armors_info.h"

class DetectCarInfo
{
public:
    // 构造函数
    DetectCarInfo();

    // 析构函数
    ~DetectCarInfo() = default;

    u_int8_t carClass;                                      // 车辆的类别：0~4--红1~红5，5~9--蓝1~蓝5
    int carX;                                               // 车辆的中心点x轴图像坐标
    int carY;                                               // 车辆的中心点y轴图像坐标
    int carWidth;                                           // 车辆的图像宽度
    int carHeight;                                          // 车辆的图像高度
    std::vector<DetectArmorsInfo> detectArmorsInfo;         // 车辆内装甲板信息
};

#endif //SRC_DETECT_CARS_INFO_H
