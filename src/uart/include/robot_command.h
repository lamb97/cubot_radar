//
// Created by zhangtianyi on 2023/4/25.
//

#ifndef ROS_WS_ROBOT_COMMAND_H
#define ROS_WS_ROBOT_COMMAND_H

#include <cmath>


/**
 * @brief 机器人大脑的控制指令
 */
class RobotCommand
{
public:
    unsigned char ID;    ///< 目标机器人ID
    float X;             ///< 目标机器人的X轴坐标
    float Y;             ///< 目标机器人的Y轴坐标
    float Z;             ///< 目标机器人的Z轴坐标


    /**
    * @brief 构造函数
    */
    RobotCommand();

    /**
     * @brief 析构函数
     */
    ~RobotCommand() = default;

    /**
     * @brief 获取控制指令打包之后的数据帧字节长度
     * @return 控制指令打包之后的数据帧字节长度
     */
    static unsigned int GetFrameSize();

    /**
     * @brief 将控制指令封装成数据帧
     * @param[out] frame 存储数据帧的字节数组
     */
    void EncapsulateToFrame(unsigned char *frame) const;
};



#endif //ROS_WS_ROBOT_COMMAND_H
