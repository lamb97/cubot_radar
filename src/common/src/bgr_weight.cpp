//
// Created by plutoli on 2021/8/12.
//

#include "bgr_weight.h"

// 构造函数
BGRWeight::BGRWeight():
    Blue(1.0),
    Green(0.0),
    Red(0.0)
{
}

// 判断BGR三通道权值是否有效
bool BGRWeight::IsValid() const
{
    // 初始化判断结果
    bool result = false;

    // 判断权值的有效性
    if ((Blue >= 0.0) && (Green >= 0.0) && (Red >= 0.0) && ((Blue + Green + Red) <= 1.0))
    {
        result = true;
    }

    // 返回判断结果
    return result;
}