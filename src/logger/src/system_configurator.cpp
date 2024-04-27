//
// Created by plutoli on 2022/2/4.
//

#include "system_configurator.h"

// ******************************  SystemConfigurator类的公有函数  ******************************

// 转换机器人大脑内核工作模式
bool SystemConfigurator::ConvertToWorkMode(const int &input, EWorkMode *output)
{
    // 初始化转换结果
    bool result = true;

    // 转换工作模式
    switch (input)
    {
        case 0:
            *output = EWorkMode::Manual;
            break;

        case 1:
            *output = EWorkMode::ShootOne;
            break;

        case 2:
            *output = EWorkMode::ShootTwo;
            break;

        case 3:
            *output = EWorkMode::ShootThree;
            break;

        case 4:
            *output = EWorkMode::ShootFour;
            break;

        case 5:
            *output = EWorkMode::ShootFive;
            break;

        case 6:
            *output = EWorkMode::ShootSentry;
            break;

        case 7:
            *output = EWorkMode::ShootOutpost;
            break;

        case 8:
            *output = EWorkMode::ShootBase;
            break;

        case 9:
            *output = EWorkMode::AutomaticShoot;
            break;

        case 10:
            *output = EWorkMode::CurvedFireOutpost;
            break;

        case 11:
            *output = EWorkMode::CurvedFireBase;
            break;

        case 12:
            *output = EWorkMode::ShootSmallBuff;
            break;

        case 13:
            *output = EWorkMode::ShootLargeBuff;
            break;

        default:
            result = false;
            break;
    }

    // 返回转换结果
    return result;
}

// 转换机器人大脑内核的补偿多项式阶数
bool SystemConfigurator::ConvertToPolynomialOrder(const int &input, EPolynomialOrder *output)
{
    // 初始化转换结果
    bool result = true;

    // 转换目标解算器的补偿多项式阶数
    switch (input)
    {
        case 1:
            *output = EPolynomialOrder::One;
            break;

        case 2:
            *output = EPolynomialOrder::Two;
            break;

        case 3:
            *output = EPolynomialOrder::Three;
            break;

        case 4:
            *output = EPolynomialOrder::Four;
            break;

        case 5:
            *output = EPolynomialOrder::Five;
            break;

        case 6:
            *output = EPolynomialOrder::Six;
            break;

        case 7:
            *output = EPolynomialOrder::Seven;
            break;

        case 8:
            *output = EPolynomialOrder::Eight;
            break;

        case 9:
            *output = EPolynomialOrder::Nine;
            break;

        default:
            result = false;
            break;
    }

    // 返回转换结果
    return result;
}

// 转换机器人本体的子弹速度
bool SystemConfigurator::ConvertToBulletVelocity(const int &input, EBulletVelocity *output)
{
    // 初始化转换结果
    bool result = true;

    // 转换子弹速度
    switch (input)
    {
        case 10:
            *output = EBulletVelocity::MPS_10;
            break;

        case 11:
            *output = EBulletVelocity::MPS_11;
            break;

        case 12:
            *output = EBulletVelocity::MPS_12;
            break;

        case 13:
            *output = EBulletVelocity::MPS_13;
            break;

        case 14:
            *output = EBulletVelocity::MPS_14;
            break;

        case 15:
            *output = EBulletVelocity::MPS_15;
            break;

        case 16:
            *output = EBulletVelocity::MPS_16;
            break;

        case 17:
            *output = EBulletVelocity::MPS_17;
            break;

        case 18:
            *output = EBulletVelocity::MPS_18;
            break;

        case 19:
            *output = EBulletVelocity::MPS_19;
            break;

        case 20:
            *output = EBulletVelocity::MPS_20;
            break;

        case 21:
            *output = EBulletVelocity::MPS_21;
            break;

        case 22:
            *output = EBulletVelocity::MPS_22;
            break;

        case 23:
            *output = EBulletVelocity::MPS_23;
            break;

        case 24:
            *output = EBulletVelocity::MPS_24;
            break;

        case 25:
            *output = EBulletVelocity::MPS_25;
            break;

        case 26:
            *output = EBulletVelocity::MPS_26;
            break;

        case 27:
            *output = EBulletVelocity::MPS_27;
            break;

        case 28:
            *output = EBulletVelocity::MPS_28;
            break;

        case 29:
            *output = EBulletVelocity::MPS_29;
            break;

        case 30:
            *output = EBulletVelocity::MPS_30;
            break;

        default:
            result = false;
            break;
    }

    // 返回转换结果
    return result;
}

// 转换机器人本体的请求类型
bool SystemConfigurator::ConvertToRequestType(const int &input, ERequestType *output)
{
    // 初始化转换结果
    bool result = true;

    // 转换请求类型
    switch (input)
    {
        case 1:
            *output = ERequestType::HeartBeat;
            break;

        case 2:
            *output = ERequestType::SwitchWorkMode;
            break;

        case 3:
            *output = ERequestType::SwitchBulletVelocity;
            break;

        case 4:
            *output = ERequestType::SaveLog;
            break;

        case 5:
            *output = ERequestType::RebootApplication;
            break;

        case 6:
            *output = ERequestType::RebootComputer;
            break;
        case 7:
            *output = ERequestType::PredictorCommond;
            break;

        default:
            result = false;
            break;
    }

    // 返回转换结果
    return result;
}