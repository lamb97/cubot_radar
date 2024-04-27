//
// Created by plutoli on 2021/12/30.
//

#include "easy_logger.h"

// ******************************  EasyLogger类的公有函数  ******************************

// 析构函数
EasyLogger::~EasyLogger()
{
    if (isInitialized_)
    {
        Release();
    }
}

// 获取日志记录器的单例引用
EasyLogger& EasyLogger::GetSingleInstance()
{
    static EasyLogger logger;
    return logger;
}

// 获取日志记录器参数
EasyLoggerParam EasyLogger::GetParam() const
{
    return param_;
}

// 设置日志记录器参数
bool EasyLogger::SetParam(const EasyLoggerParam &param)
{
    // 初始化设置结果
    bool result = false;

    // 保存日志记录器参数
    if (!isInitialized_)
    {
        param_ = param;
        result = true;
    }

    // 返回设置结果
    return result;
}

// 获取日志记录器的初始化状态
bool EasyLogger::IsInitialized() const
{
    return isInitialized_;
}

// 初始化日志记录器
bool EasyLogger::Init()
{
    // 判断日志记录器是否已经初始化
    if (isInitialized_)
    {
        std::cout << "EasyLogger can not be initialized repeatedly" << std::endl;
        return false;
    }

    // 判断日志文件存储路径是否合法
    if (::access(param_.LogPath.c_str(), R_OK) == -1)
    {
        if (::mkdir(param_.LogPath.c_str(), 0777) == -1)
        {
            std::cout << "LogPath does not exist, and was created failure" << std::endl;
            return false;
        }
    }

    // 获取当前日期时间字符串，使用日期时间字符串创建日志文件名
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&now_time);
    std::stringstream stream;
    stream << std::put_time(&tm, "%Y-%m-%d_%H:%M:%S");
    std::string dateTimeString = stream.str();
    std::string logFile = param_.LogPath + param_.LogPrefix + "_" + dateTimeString + ".txt";

    // 创建spdlog格式的日志记录器
    auto consoleSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    auto fileSink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(logFile, true);
    spdlog::sinks_init_list sinkList = {fileSink, consoleSink};
    logger_ = std::make_shared<spdlog::logger>(spdlog::logger("EasyLogger", sinkList.begin(), sinkList.end()));

    // 注册spdlog格式的日志记录器
    spdlog::register_logger(logger_);
    spdlog::set_default_logger(logger_);

    // 设置日志记录器刷新周期，确保日志信息不会丢失
    spdlog::flush_every(std::chrono::seconds(param_.FlushPeriod));

    // 设置初始化状态
    isInitialized_ = true;

    // 返回初始化结果
    return true;
}

// 释放日志记录器的资源
bool EasyLogger::Release()
{
    // 初始化释放结果
    bool result = false;

    // 重置初始化标志
    if (isInitialized_)
    {
        // 强制刷新日志记录器，保存日志信息
        logger_->flush();

        // 重置初始化标志
        isInitialized_ = false;
        result = true;
    }

    // 返回释放结果
    return result;
}

// 保存日志信息
bool EasyLogger::Save(const ELogType &type, const std::string &log)
{
    // 初始化保存结果
    bool result = false;

    // 保存日志信息
    if (isInitialized_)
    {
        switch (type)
        {
            case ELogType::Info:
                logger_->info(log);
                break;

            case ELogType::Warn:
                logger_->warn(log);
                break;

            case ELogType::Error:
                logger_->error(log);
                break;

            default:
                break;
        }

        result = true;
    }

    // 返回保存结果
    return result;
}

// ******************************  EasyLogger类的私有函数  ******************************

// 构造函数
EasyLogger::EasyLogger():
    isInitialized_(false),
    param_(),
    logger_(nullptr)
{
}