//
// Created by zhangtianyi on 2023/4/25.
//

#include "robot_command.h"
#include <iostream>
// // 构造函数
// RobotCommand::RobotCommand():
//         ID(1.0),
//         X(0.0),
//         Y(0.0),
//         Z(0.0)
// {
// }

// // 获取控制指令打包之后的数据帧字节长度
// unsigned int RobotCommand::GetFrameSize()
// {
//     // 初始化打包之后的字节总长度
//     unsigned int totalSize = 0;

//     // 数据帧的帧头占用1个字节
//     unsigned int headerSize = 1;
//     totalSize += headerSize;

//     // 数据帧的类型占用1个字节
//     unsigned int typeSize = 1;
//     totalSize += typeSize;

//     // 目标机器人ID占用1个字节
//     unsigned int idSize = 1;
//     totalSize += idSize;

//     // 目标机器人的X轴坐标占用3个字节
//     unsigned int XSize = 3;
//     totalSize += XSize;

//     // 目标机器人的Y轴坐标占用3个字节
//     unsigned int YSize = 3;
//     totalSize += YSize;

//     // 目标机器人的Z轴坐标占用3个字节
//     unsigned int ZSize = 3;
//     totalSize += ZSize;

//     // 数据帧的帧尾占用1个字节
//     unsigned int tailSize = 1;
//     totalSize += tailSize;

//     // 返回数据长度
//     return totalSize;
// }

// // 将控制指令封装成数据帧
// void RobotCommand::EncapsulateToFrame(unsigned char *frame) const
// {
//     // 初始化字节索引
//     unsigned int index = 0;

//     // 写入数据帧的帧头
//     *(frame + index) = 0xAA;
//     index += 1;

//     // 写入数据帧的类型
//     *(frame + index) = 0x01;
//     index += 1;

//     // 写入目标机器人ID
//     *(frame + index) = ID;
//     index += 1;

//     if (X  < 0 )
//     {
//         *(frame + index) = 0x00;
//         index += 1;
//     }
//     else
//     {
//         *(frame + index) = 0x01;
//         index += 1;
//     }


//     // 提取目标机器人的X轴坐标的整数部分和小数部分
//     double XInteger;
//     double XFraction = std::modf(X, &XInteger);

//     // 写入目标机器人的X轴坐标的整数部分
//     double XIntegerAbs = std::abs(XInteger);
//     auto XIntegerValue = static_cast<unsigned char>(XIntegerAbs);
//     *(frame + index) = XIntegerValue;
//     index += 1;


//     // 写入目标机器人的X轴坐标的小数部分
//     double XFractionAbs = std::round(std::abs(XFraction * 100.0));
//     auto XFractionValue = static_cast<unsigned char>(XFractionAbs);
//     *(frame + index) = XFractionValue;
//     index += 1;

//     if (Y < 0 )
//     {
//         *(frame + index) = 0x00;
//     }
//     else
//     {
//         *(frame + index) = 0x01;
//     }
//     index += 1;
//     // 提取目标机器人的Y轴坐标的整数部分和小数部分
//     double YInteger;
//     double YFraction = std::modf(Y, &YInteger);

//     // 写入目标机器人的Y轴坐标的整数部分
//     double YIntegerAbs =  std::abs(YInteger);
//     auto YIntegerValue = static_cast<unsigned char>(YIntegerAbs);
//     *(frame + index) = YIntegerValue;
//     index += 1;

//     // 写入目标机器人的Y轴坐标的小数部分
//     double YFractionAbs = std::round(std::abs(YFraction * 100.0));
//     auto YFractionValue = static_cast<unsigned char>(YFractionAbs);
//     *(frame + index) =  YFractionValue;
//     index += 1;

//     if (Z < 0 )
//     {
//         *(frame + index) = 0x00;
//     }
//     else
//     {
//         *(frame + index) = 0x01;
//     }
//     index += 1;

//     // 提取目标机器人的Z轴坐标的整数部分和小数部分
//     double ZInteger;
//     double ZFraction = std::modf(Z, &ZInteger);

//     // 写入目标机器人的Z轴坐标的整数部分
//     double ZIntegerAbs = std::abs(ZInteger);
//     auto ZIntegerValue = static_cast<unsigned char>(ZIntegerAbs);
//     *(frame + index) = ZIntegerValue;
//     index += 1;

//     // 写入目标机器人的Z轴坐标的小数部分
//     double ZFractionAbs = std::round(std::abs(ZFraction * 100.0));;
//     auto ZFractionValue = static_cast<unsigned char>(ZFractionAbs);
//     *(frame + index) =  ZFractionValue;
//     index += 1;

//     // 写入数据帧的帧尾
//     *(frame + index) = 0xDD;
// }
// // 构造函数
// RobotCommand::RobotCommand():
//         ID(1.0),
//         X(0.0),
//         Y(0.0),
//         Z(0.0)
// {
// }

// // 获取控制指令打包之后的数据帧字节长度
// unsigned int RobotCommand::GetFrameSize()
// {
//     // 初始化打包之后的字节总长度
//     unsigned int totalSize = 0;

//     // 数据帧的帧头占用1个字节
//     unsigned int headerSize = 1;
//     totalSize += headerSize;

//     // 数据帧的类型占用1个字节
//     unsigned int typeSize = 1;
//     totalSize += typeSize;

//     // 目标机器人ID占用1个字节
//     unsigned int idSize = 1;
//     totalSize += idSize;

//     // 目标机器人的X轴坐标占用3个字节
//     unsigned int XSize = 3;
//     totalSize += XSize;

//     // 目标机器人的Y轴坐标占用3个字节
//     unsigned int YSize = 3;
//     totalSize += YSize;

//     // 目标机器人的Z轴坐标占用3个字节
//     unsigned int ZSize = 3;
//     totalSize += ZSize;

//     // 数据帧的帧尾占用1个字节
//     unsigned int tailSize = 1;
//     totalSize += tailSize;

//     // 返回数据长度
//     return totalSize;
// }

// 将控制指令封装成数据帧
void RobotCommand::EncapsulateToFrame(unsigned char *frame) const
{
    // 初始化字节索引
    unsigned int index = 0;

    // 写入数据帧的帧头
    *(frame + index) = 0xAA;
    index += 1;

    // 写入数据帧的类型
    *(frame + index) = 0x01;
    index += 1;

    // 写入目标机器人ID
    *(frame + index) = ID;
    index += 1;

    if (X  < 0 )
    {
        *(frame + index) = 0x00;
        index += 1;
    }
    else
    {
        *(frame + index) = 0x01;
        index += 1;
    }


    // 提取目标机器人的X轴坐标的整数部分和小数部分
    double XInteger;
    double XFraction = std::modf(X, &XInteger);

    // 写入目标机器人的X轴坐标的整数部分
    double XIntegerAbs = std::abs(XInteger);
    auto XIntegerValue = static_cast<unsigned char>(XIntegerAbs);
    *(frame + index) = XIntegerValue;
    index += 1;


    // 写入目标机器人的X轴坐标的小数部分
    double XFractionAbs = std::round(std::abs(XFraction * 100.0));
    auto XFractionValue = static_cast<unsigned char>(XFractionAbs);
    *(frame + index) = XFractionValue;
    index += 1;

    if (Y < 0 )
    {
        *(frame + index) = 0x00;
    }
    else
    {
        *(frame + index) = 0x01;
    }
    index += 1;
    // 提取目标机器人的Y轴坐标的整数部分和小数部分
    double YInteger;
    double YFraction = std::modf(Y, &YInteger);

    // 写入目标机器人的Y轴坐标的整数部分
    double YIntegerAbs =  std::abs(YInteger);
    auto YIntegerValue = static_cast<unsigned char>(YIntegerAbs);
    *(frame + index) = YIntegerValue;
    index += 1;

    // 写入目标机器人的Y轴坐标的小数部分
    double YFractionAbs = std::round(std::abs(YFraction * 100.0));
    auto YFractionValue = static_cast<unsigned char>(YFractionAbs);
    *(frame + index) =  YFractionValue;
    index += 1;

    if (Z < 0 )
    {
        *(frame + index) = 0x00;
    }
    else
    {
        *(frame + index) = 0x01;
    }
    index += 1;

    // 提取目标机器人的Z轴坐标的整数部分和小数部分
    double ZInteger;
    double ZFraction = std::modf(Z, &ZInteger);

    // 写入目标机器人的Z轴坐标的整数部分
    double ZIntegerAbs = std::abs(ZInteger);
    auto ZIntegerValue = static_cast<unsigned char>(ZIntegerAbs);
    *(frame + index) = ZIntegerValue;
    index += 1;

    // 写入目标机器人的Z轴坐标的小数部分
    double ZFractionAbs = std::round(std::abs(ZFraction * 100.0));;
    auto ZFractionValue = static_cast<unsigned char>(ZFractionAbs);
    *(frame + index) =  ZFractionValue;
    index += 1;

    // 写入数据帧的帧尾
    *(frame + index) = Get;
}
