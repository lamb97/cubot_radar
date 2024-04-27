//
// Created by plutoli on 2021/7/20.
//

#ifndef CUBOT_BRAIN_LOG_MESSAGE_H
#define CUBOT_BRAIN_LOG_MESSAGE_H

// 定义日志起始字符串
#ifndef LOG_BEGIN
#define LOG_BEGIN "###############     " + static_cast<std::string>(__FUNCTION__) + "() Begin     ###############"
#endif

// 定义日志结束字符串
#ifndef LOG_END
#define LOG_END "###############     " + static_cast<std::string>(__FUNCTION__) + "() End       ###############"
#endif

#endif //CUBOT_BRAIN_LOG_MESSAGE_H
