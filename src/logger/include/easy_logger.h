//
// Created by plutoli on 2021/12/30.
//

#ifndef CUBOT_BRAIN_EASY_LOGGER_H
#define CUBOT_BRAIN_EASY_LOGGER_H

#include <string>
#include <sstream>
#include <iomanip>
#include <iostream>
#include "easy_logger_param.h"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"

// 定义日志起始字符串
#ifndef LOG_BEGIN
#define LOG_BEGIN "********************     " + static_cast<std::string>(__PRETTY_FUNCTION__) + " Begin     ********************"
#endif

// 定义日志结束字符串
#ifndef LOG_END
#define LOG_END "********************     " + static_cast<std::string>(__PRETTY_FUNCTION__) + " End     ********************"
#endif

/**
 * @brief 日志类型
 */
enum class ELogType
{
    Info = 0,
    Warn = 1,
    Error = 2
};

/**
 * @brief 基于spdlog封装的日志记录器模块
 */
class EasyLogger
{
public:
    /**
     * @brief 析构函数
     */
    ~EasyLogger();

    /**
     * @brief 获取日志记录器的单例引用
     * @return 日志记录器的单例引用
     * @note 在此不返回日志记录器的指针，避免用户使用delete指令导致对象被提前销毁
     */
    static EasyLogger& GetSingleInstance();

    /**
     * @brief 获取日志记录器参数
     * @return 日志记录器参数
     */
    EasyLoggerParam GetParam() const;

    /**
     * @brief 设置日志记录器参数
     * @param[in] param 日志记录器参数
     * @return 日志记录器参数设置结果\n
     *         -<em>false</em> 设置失败\n
     *         -<em>true</em> 设置成功\n
     */
    bool SetParam(const EasyLoggerParam &param);

    /**
     * @brief 获取日志记录器的初始化状态
     * @return 日志记录器的初始化状态\n
     *         -<em>false</em> 尚未初始化\n
     *         -<em>true</em> 已经初始化\n
     */
    bool IsInitialized() const;

    /**
     * @brief 初始化日志记录器
     * @return 日志记录器初始化结果\n
     *         -<em>false</em> 初始化失败\n
     *         -<em>true</em> 初始化成功\n
     */
    bool Init();

    /**
     * @brief 释放日志记录器的资源
     * @return 日志记录器资源释放结果\
     *         -<em>false</em> 资源释放失败\n
     *         -<em>true</em> 资源释放成功\n
     * @note Release()和Init()配套使用；
     */
    bool Release();

    /**
     * @brief 保存日志信息
     * @param[in] type 日志信息类型
     * @param[in] log 日志信息
     * @return 日志信息保存结果\n
     *         -<em>false</em> 保存失败\n
     *         -<em>true</em> 保存成功\n
     * @note 如果日志记录器没有初始化，则保存失败
     */
    bool Save(const ELogType &type, const std::string &log);

private:
    /**
     * @brief 日志记录器的初始化状态
     */
    std::atomic<bool> isInitialized_;

    /**
     * @brief 日志记录器参数
     */
    EasyLoggerParam param_;

    /**
     * @brief spdlog格式的日志记录器
     */
    std::shared_ptr<spdlog::logger> logger_;

    /**
     * @brief 构造函数
     * @note 将默认构造函数设置为private，阻止外部调用实例化Logger类
     */
    EasyLogger();
};

#endif //CUBOT_BRAIN_EASY_LOGGER_H