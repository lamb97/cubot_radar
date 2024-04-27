//
// Created by zhangtianyi on 2022/3/27.
//

#ifndef SRC_SERIAL_PORT_H
#define SRC_SERIAL_PORT_H


#include <unistd.h>
#include <fcntl.h>
#include <atomic>
#include <termios.h>
#include <thread>
#include <sys/ioctl.h>
#include <condition_variable>
#include "log_message.h"
#include "easy_logger.h"
#include "serial_port_param.h"

/**
 * @brief 串口数据接收回调函数
 * @param bytesToRead 接收到的字节数
 * @param userData 回调函数关联的用户数据
 * @note 参考网址：https://www.cnblogs.com/smartlife/articles/12519130.html
 */
typedef void (*DataReceivedHandler)(const unsigned int &bytesToRead, void* userData);

/**
 * @brief 通用串口类
 * @note 使用步骤如下：\n
 *       Step1：实例化SerialPort\n
 *       Step2：自行构造SerialPortParam或从Yaml文件中读取SerialPortParam\n
 *       Step3：调用SetParam()函数设置串口参数\n
 *       Step4：调用RegistDataReceivedHandler()函数注册数据接收回调函数\n
 *       Step5：调用Init()函数初始化串口\n
 *       Step6：调用Open()函数打开串口\n
 *       Step7：调用Read()/ReadLine()函数读取数据\n
 *       Step8：调用Write()函数下写数据\n
 *       Step9：调用Close()函数关闭串口\n
 *       Step10：调用Release()函数释放串口资源\n
 *       Step11：调用UnregistDataReceivedHandler()函数解除注册数据接收回调函数\n
 */




class SerialPort
{
public:
    /**
    * @brief 构造函数
    */
    SerialPort();

    /**
     * @brief 析构函数
     */
    ~SerialPort();

    /**
    * @brief 拷贝构造函数
    * @param[in] serialPort 拷贝对象
    * @note 禁用拷贝构造函数
    */
    SerialPort(const SerialPort &serialPort) = delete;

    /**
     * @brief 拷贝赋值运算符
     * @param[in] serialPort 拷贝对象
     * @return 拷贝赋值结果
     * @note 禁用拷贝赋值运算符
     */
    SerialPort& operator=(const SerialPort &serialPort) = delete;

    /**
     * @brief 移动构造函数
     * @param[in] serialPort 移动对象
     * @note 禁用移动构造函数
     */
    SerialPort(SerialPort &&serialPort) = delete;

    /**
     * @brief 移动赋值运算符
     * @param[in] serialPort 移动对象
     * @return 移动赋值结果
     * @note 禁用移动赋值运算符
     */
    SerialPort& operator=(SerialPort &&serialPort) = delete;

    /**
     * @brief 获取串口配置参数
     * @return 串口配置参数
     */
    SerialPortParam GetParam();

    /**
     * @brief 设置串口配置参数
     * @param[in] param 串口配置参数
     * @return 串口配置参数设置结果\n
     *         -<em>false</em> 设置失败\n
     *         -<em>true</em> 设置成功\n
     * @note 串口初始化之后，不能设置串口配置参数
     */
    bool SetParam(const SerialPortParam &param);

    /**
     * @brief 获取串口的初始化状态
     * @return 串口的初始化状态\n
     *         -<em>false</em> 尚未初始化\n
     *         -<em>true</em> 已经初始化\n
     */
    bool IsInitialized();

    /**
     * @brief 获取串口打开状态
     * @return 串口的打开状态\n
     *         -<em>false</em> 尚未打开\n
     *         -<em>true</em> 已经打开\n
     */
    bool IsOpened();

    /**
     * @brief 获取串口的初始化时间戳
     * @return 串口的初始化时间戳
     */
    uint64_t GetInitTimestamp();

    /**
     * @brief 获取串口的打开时间戳
     * @return 串口的打开时间戳
     */
    uint64_t GetOpenTimestamp();

    /**
     * @brief 初始化串口
     * @return 串口初始化结果\n
     *         -<em>false</em> 初始化失败\n
     *         -<em>true</em> 初始化成功\n
     * @note Release()和Init()配套使用；
     */
    bool Init();

    /**
     * @brief 释放串口资源
     * @return 串口资源释放结果\
     *         -<em>false</em> 资源释放失败\n
     *         -<em>true</em> 资源释放成功\n
     * @note Release()和Init()配套使用；相机必须关闭之后才能执行释放操作。
     */
    bool Release();

    /**
     * @brief 打开串口
     * @return 串口打开结果\n
     *         -<em>false</em> 打开失败\n
     *         -<em>true</em> 打开成功\n
     */
    bool Open();

    /**
     * @brief 关闭串口
     * @return 串口关闭结果\n
     *         -<em>false</em> 关闭失败\n
     *         -<em>true</em> 关闭成功\n
     */
    bool Close();

    /**
     * @brief 获取串口缓冲区中可读取的数据长度(以字节为单位)
     * @return 可读取的数据长度(以字节为单位)
     */
    unsigned int GetBytesToRead();

    /**
     * @brief 获取串口缓冲区中已写入的数据长度(以字节为单位)
     * @return 已写入的数据长度(以字节为单位)
     */
    unsigned int GetBytesToWrite();

    /**
     * @brief 从串口缓冲区中读取数据
     * @param[in] size    要读取的数据长度
     * @param[out] data   要读取的数据存储地址
     * @return 实际读取到的数据长度(以字节为单位)
     */
    unsigned int Read(const unsigned int &size, unsigned char *data);

    /**
     * @brief 根据串口配置参数给定的分隔符数组，从串口缓冲区中读取一行数据
     * @param[out] data 要读取的数据存储地址
     * @return 实际读取到的数据长度(以字节为单位)
     */
    unsigned int ReadLine(unsigned char *data);

    /**
     * @brief 向串口缓冲区中写入数据
     * @param[in] size 要写入的数据长度
     * @param[in] data 要写入的数据存储地址
     */
    void Write(const unsigned int &size, const unsigned char *data);

    /**
     * @brief 注册串口数据接收回调函数
     * @param[in] handler 串口数据接收回调函数
     * @param[in] userData 回调函数关联的用户数据
     * @return 串口数据接收回调函数注册结果\
     *         -<em>false</em> 注册失败\n
     *         -<em>true</em> 注册成功\n
     * @note 如果串口已经打开，数据接收回调函数将注册失败。
     */
    bool RegistDataReceivedHandler(DataReceivedHandler handler, void *userData);

    /**
     * @brief 解除注册串口数据接收回调函数
     * @return 串口数据接收回调函数解除注册结果\
     *         -<em>false</em> 解除注册失败\n
     *         -<em>true</em> 解除注册成功\n
     * @note 如果串口已经打开，数据接收回调函数将解除注册失败。
     */
    bool UnregistDataReceivedHandler();



private:
    SerialPortParam param_;                             ///< 串口配置参数
    std::atomic<bool> isInitialized_;                   ///< 串口的初始化状态
    std::atomic<bool> isOpened_;                        ///< 串口的打开状态
    std::atomic<int> fileDescriptor_;                   ///< 串口的文件描述符
    std::atomic<uint64_t> initTimestamp_;               ///< 串口的初始化时间戳
    std::atomic<uint64_t> openTimestamp_;               ///< 串口的打开时间戳
    std::mutex operateMutex_;                           ///< 串口的操作互斥锁
    unsigned char *readBuffer_;                         ///< 串口的读取缓冲区
    unsigned char *writeBuffer_;                        ///< 串口的写入缓冲区
    unsigned int bytesToRead_;                          ///< 串口的读取缓冲区的字节数
    unsigned int bytesToWrite_;                         ///< 串口的写入缓冲区的字节数
    std::mutex readMutex_;                              ///< 串口的读取互斥锁
    std::mutex writeMutex_;                             ///< 串口的写入互斥锁
    std::atomic<bool> readSwitch_;                      ///< 串口的读取开关
    std::atomic<bool> writeSwitch_;                     ///< 串口的写入开关
    std::thread readFromDriverThread_;                  ///< 串口的驱动程序数据读取线程
    std::thread writeToDriverThread_;                   ///< 串口的驱动程序数据写入线程
    std::condition_variable writeConditionVariable_;    ///< 串口的驱动程序数据写入线程条件变量
    DataReceivedHandler dataReceivedHandler_;           ///< 串口的数据接收回调函数
    void *dataReceivedUserData_;                        ///< 串口的数据接收用户数据

    /**
     * @brief 从串口驱动程序中读取数据
     */
    void ReadFromDriver();

    /**
     * @brief 向串口驱动程序中写入数据
     */
    void WriteToDriver();
};


#endif //SRC_SERIAL_PORT_H
