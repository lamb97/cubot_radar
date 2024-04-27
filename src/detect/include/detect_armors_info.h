//
// Created by godzhu on 2021/12/24.
//

#ifndef SRC_DETECT_ARMORS_INFO_H
#define SRC_DETECT_ARMORS_INFO_H

#include <string>

class DetectArmorsInfo
{
public:
    // 构造函数
    DetectArmorsInfo();
    
    // 析构函数
    ~DetectArmorsInfo() = default;

    u_int8_t armorClass;           // 装甲板的类别：0~4--红1~红5，5~9--蓝1~蓝5
    int armorX;                    // 装甲板的中心点x轴图像坐标
    int armorY;                    // 装甲板的中心点y轴图像坐标
    int armorWidth;                // 装甲板的图像宽度
    int armorHeight;               // 装甲板的图像高度
    
};

#endif //SRC_DETECT_ARMORS_INFO_H
