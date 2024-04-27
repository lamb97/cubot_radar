find_package(Threads REQUIRED)

# 生成logger库
add_library(logger SHARED
            src/easy_logger.cpp
            src/easy_logger_param.cpp
            src/system_configurator.cpp)
target_link_libraries(logger
                      Threads::Threads)


