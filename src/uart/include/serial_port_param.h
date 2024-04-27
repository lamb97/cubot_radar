//
// Created by plutoli on 2021/7/19.
//

#ifndef CUBOT_BRAIN_SERIAL_PORT_PARAM_H
#define CUBOT_BRAIN_SERIAL_PORT_PARAM_H

#include <string>
#include <termios.h>
#include <opencv2/opencv.hpp>
#include "spdlog/spdlog.h"
#include "log_message.h"
#include "easy_logger.h"

/**
 * @brief 波特率枚举类型
 */
enum class EBaudRate
{
    BR0 = B0,
    BR50 = B50,
    BR75 = B75,
    BR110 = B110,
    BR134 = B134,
    BR150 = B150,
    BR200 = B200,
    BR300 = B300,
    BR600 = B600,
    BR1200 = B1200,
    BR1800 = B1800,
    BR2400 = B2400,
    BR4800 = B4800,
    BR9600 = B9600,
    BR19200 = B19200,
    BR38400 = B38400,
    BR57600 = B57600,
    BR115200 = B115200,
    BR230400 = B230400,
    BR460800 = B460800,
    BR500000 = B500000,
    BR576000 = B576000,
    BR921600 = B921600,
    BR1000000 = B1000000,
    BR1152000 = B1152000,
    BR1500000 = B1500000,
    BR2000000 = B2000000,
    BR2500000 = B2500000,
    BR3000000 = B3000000,
    BR3500000 = B3500000,
    BR4000000 = B4000000
};

/**
 * @brief 数据位枚举类型
 */
enum class EDataBits
{
    Five = 5,       ///< 5位数据位
    Six = 6,        ///< 6位数据位
    Seven = 7,      ///< 7位数据位
    Eight = 8       ///< 8位数据位
};

/**
 * @brief 停止位枚举类型
 */
enum class EStopBits
{
    One = 1,        ///< 1位停止位
    Two = 2         ///< 2位停止位
};

/**
 * @brief 校验位枚举类型
 */
enum class EParity
{
    None = 0,       ///< 无校验
    Odd = 1,        ///< 奇校验
    Even = 2,       ///< 偶校验
    Mark = 3,       ///< 使用Mark
    Space = 4       ///< 使用Space
};

/**
 * @brief 流控枚举类型
 */
enum class EFlowControl
{
    None = 0,       ///< 无流控
    Software = 1,   ///< 软件流控
    Hardware = 2    ///< 硬件流控
};

/**
 * @brief 串口参数
 */
class SerialPortParam
{
public:
    std::string Key;                        ///< 串口标识字符串
    std::string Name;                       ///< 串口驱动文件名
    EBaudRate BaudRate;                     ///< 串口波特率
    EDataBits DataBits;                     ///< 串口数据位
    EStopBits StopBits;                     ///< 串口停止位
    EParity Parity;                         ///< 串口校验位
    EFlowControl FlowControl;               ///< 串口流控
    unsigned int ReadBufferSize;            ///< 串口读取缓冲区长度
    unsigned int WriteBufferSize;           ///< 串口写入缓冲区长度
    std::vector<unsigned char> Separators;  ///< 串口数据帧分隔符数组

    /**
    * @brief 构造函数
    */
    SerialPortParam();

    /**
     * @brief 析构函数
     */
    ~SerialPortParam() = default;

    /**
     * @brief 转换波特率
     * @param[in]   input   输入的波特率数值
     * @param[out]  output  转换得到的波特率
     * @return 波特率转换结果\n
     *         -<em>false</em> 转换失败\n
     *         -<em>true</em> 转换成功\n
     * @note 如果输入的数值不是标准波特率，则转换失败\n
     */
    static bool ConvertToBaudRate(const int &input, EBaudRate *output);

    /**
     * @brief 转换数据位
     * @param[in]   input   输入的数据位数值
     * @param[out]  output  转换得到的数据位
     * @return 数据位转换结果\n
     *         -<em>false</em> 转换失败\n
     *         -<em>true</em> 转换成功\n
     * @note 数据位的长度为5/6/7/8，输入数据不在此范围内，则转换失败\n
     */
    static bool ConvertToDataBits(const int &input, EDataBits *output);

    /**
     * @brief 转换停止位
     * @param[in]   input   输入的停止位数值
     * @param[out]  output  转换得到的停止位
     * @return 停止位转换结果\n
     *         -<em>false</em> 转换失败\n
     *         -<em>true</em> 转换成功\n
     * @note 停止位的长度为1/2，输入数据不在此范围内，则转换失败\n
     */
    static bool ConvertToStopBits(const int &input, EStopBits *output);

    /**
     * @brief 转换校验位
     * @param[in]   input   输入的校验位数值
     * @param[out]  output  转换得到的校验位
     * @return 校验位转换结果\n
     *         -<em>false</em> 转换失败\n
     *         -<em>true</em> 转换成功\n
     * @note 校验位的取值为0/1/2/3/4，输入数据不在此范围内，则转换失败\n
     *         -<em>0</em> Non，无校验\n
     *         -<em>1</em> Odd，奇校验\n
     *         -<em>2</em> Even，偶校验\n
     *         -<em>3</em> Mark，使用Mark\n
     *         -<em>4</em> Space，使用Space\n
     */
    static bool ConvertToParity(const int &input, EParity *output);

    /**
     * @brief 转换流控
     * @param[in]   input   输入的流控数值
     * @param[out]  output  转换得到的流控
     * @return 流控转换结果\n
     *         -<em>false</em> 转换失败\n
     *         -<em>true</em> 转换成功\n
     * @note 流控的取值为0/1/2，输入数据不在此范围内，则转换失败\n
     *         -<em>0</em> Non，无流控\n
     *         -<em>1</em> Software，软件流控\n
     *         -<em>2</em> Hardware，硬件流控\n
     */
    static bool ConvertToFlowControl(const int &input, EFlowControl *output);

    /**
     * @brief 从yaml配置文件中加载串口参数
     * @param[in]  yamlFileName     串口参数配置文件名称
     * @param[out] serialPortParam  串口参数
     * @return 串口参数加载结果\n
     *         -<em>false</em> 加载失败\n
     *         -<em>true</em> 加载成功\n
     */
    static bool LoadFromYamlFile(const std::string &yamlFileName, SerialPortParam *serialPortParam);
};

#endif //CUBOT_BRAIN_SERIAL_PORT_PARAM_H