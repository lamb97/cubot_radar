//
// Created by plutoli on 2021/12/30.
//

#include "easy_logger_param.h"

// ******************************  EasyLoggerParam类的公有函数  ******************************

// 构造函数
EasyLoggerParam::EasyLoggerParam():
    LogPath(),
    LogPrefix("log"),
    FlushPeriod(5)
{
    // 日志存储路径默认为启动目录下的log文件夹
    LogPath = ::getcwd(nullptr, 0);
    LogPath += "/logs/";
}