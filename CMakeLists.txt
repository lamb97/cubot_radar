cmake_minimum_required(VERSION 3.0)
project(cubot_radar)

# 设置C++版本
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOUIC_SEARCH_PATHS ./UI)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/Qt/5.15.2/gcc_64/lib/cmake")
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(QT_VERSION 5)
set(REQUIRED_LIBS Core Gui Widgets)
set(REQUIRED_LIBS_QUALIFIED Qt5::Core Qt5::Gui Qt5::Widgets)
find_package(Qt${QT_VERSION} COMPONENTS ${REQUIRED_LIBS} REQUIRED)


message(STATUS "QtVersion: "  ${Qt5Widgets_VERSION_STRING})

# 设置华睿相机的库文件
set(HUARAY_LIBS

        libavcodec.so
        libavcodec.so.58
        libavfilter.so
        libavfilter.so.7
        libavformat.so
        libavformat.so.58
        libavutil.so
        libavutil.so.56
        libImageConvert.so
        liblog4cpp.so
        liblog4cpp.so.5
        libMVSDK.so
        libMVSDK.so.2.1.0.194984
        libMVSDKGuiQt.so
        libMVSDKGuiQt.so.1
        libpostproc.so
        libpostproc.so.55
        libswresample.so
        libswresample.so.3
        libswscale.so
        libswscale.so.5
        libVideoRender.so

        libGCBase_gcc421_v3_0.so
        libGenApi_gcc421_v3_0.so
        liblog4cpp_gcc421_v3_0.so
        libLog_gcc421_v3_0.so
        libMathParser_gcc421_v3_0.so
        libNodeMapData_gcc421_v3_0.so
        libXmlParser_gcc421_v3_0.so)

# 初始化头文件包含目录
include_directories(/home/zhangtianyi/document/radar_ws/devel/include
                     ${PROJECT_SOURCE_DIR}/include
                    /opt/HuarayTech/MVviewer/include)

# 初始化库文件链接目录
link_directories(/opt/HuarayTech/MVviewer/lib
                 /opt/HuarayTech/MVviewer/lib/GenICam/bin/Linux64_x64)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             rospy
             std_msgs
             geometry_msgs
             sensor_msgs
             message_generation
             message_filters
             rosbag
             pcl_ros
             image_transport
             cv_bridge)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
include_directories(${tf2_INCLUDE_DIRS} ${tf2_ros_INCLUDE_DIRS})


set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/home/zhangtianyi/libtorch-cxx11-abi-shared-with-deps-1.10.0+cu113/libtorch/share/cmake")
find_package(Torch REQUIRED)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(${PCL_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/src/detect/include
                    ${PROJECT_SOURCE_DIR}/src/fuse/include
                    ${PROJECT_SOURCE_DIR}/src/uart/include
                    ${PROJECT_SOURCE_DIR}/src/UI/include
                    ${PROJECT_SOURCE_DIR}/src/logger/include
                    ${PROJECT_SOURCE_DIR}/src/camera/include
                    ${PROJECT_SOURCE_DIR}/src/common/include
                    ${PROJECT_SOURCE_DIR}/src/radar/include
                   )

message(STATUS "Opencv lib")
message(STATUS "        version:${OpenCV_VERSION}")
message(STATUS "        libdir:${OpenCV_LIBS}")
message(STATUS "        includedir:${OpenCV_INCLUDE_DIRS}")

# 生成自定义消息在 'msg' 文件夹
add_message_files(FILES
                  CustomPoint.msg
                  CustomMsg.msg
                  ArmorsInformation.msg
                  CarsInformation.msg
                  CarsMsg.msg)

# 添加生成消息和服务的依赖
generate_messages(DEPENDENCIES
                  std_msgs
                  sensor_msgs
                  geometry_msgs)

catkin_package(CATKIN_DEPENDS
               message_runtime)


# 初始化系统子模块
add_subdirectory(src/logger)
add_subdirectory(src/camera)
add_subdirectory(src/common)
add_subdirectory(src/radar)
add_subdirectory(src/detect)
add_subdirectory(src/fuse)
add_subdirectory(src/uart)
add_subdirectory(src/UI)
add_subdirectory(src/application)



