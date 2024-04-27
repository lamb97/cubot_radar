//
// Created by plutoli on 2021/12/30.
//

#ifndef CUBOT_BRAIN_EASY_LOGGER_PARAM_H
#define CUBOT_BRAIN_EASY_LOGGER_PARAM_H

#include <string>
#include <unistd.h>

/**
 * @brief 日志记录器参数
 */
class EasyLoggerParam
{
public:
    /**
     * @brief 日志文件存储路径，以"/"结尾
     * @note 日志文件存储路径默认为启动目录下的logs文件夹
     */
    std::string LogPath;

    /**
     * @brief 日志文件名的前缀
     * @note 日志文件名的前缀默认为log，不能包含非法字符
     */
    std::string LogPrefix;

    /**
     * @brief 日志信息刷新周期，单位：秒
     */
    uint64_t FlushPeriod;

    /**
    * @brief 构造函数
    */
    EasyLoggerParam();

    /**
     * @brief 析构函数
     */
    ~EasyLoggerParam() = default;
};

#endif //CUBOT_BRAIN_EASY_LOGGER_PARAM_H