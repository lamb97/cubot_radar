//
// Created by plutoli on 2022/4/5.
//

#include "huaray_camera.h"

// ******************************  HuarayCamera类的公有函数  ******************************

// 构造函数
HuarayCamera::HuarayCamera():
    param_(),
    isInitialized_(false),
    isOpened_(false),
    isNormal_(false),
    initTimestamp_(0),
    openTimestamp_(0),
    cameraHandle_(nullptr),
    videoCapture_(),
    videoWriter_(),
    frameWidth_(1280),
    frameHeight_(1024),
    operateMutex_(),
    data_(),
    dataMutex_(),
    updateDataSwitch_(false),
    updateDataThread_(),
    recordVideoSwitch_(false),
    recordVideoThread_()
{
    ResetData();
}

// 析构函数
HuarayCamera::~HuarayCamera()
{
    // 关闭华睿相机
    if (IsOpened())
    {
        Close();
    }

    // 释放华睿相机系统资源
    if (IsInitialized())
    {
        Release();
    }
}

// 获取华睿相机参数
HuarayCameraParam HuarayCamera::GetParam()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return param_;
}

// 设置华睿相机参数
bool HuarayCamera::SetParam(const HuarayCameraParam &param)
{
    // 锁定华睿相机的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;

    // 记录日志信息
    log = LOG_BEGIN;
    spdlog::info(log);

    // 判断相机是否已经初始化
    if (isInitialized_)
    {
        log = "[" + param_.Key + "] - HuarayCameraParam was set failure because Camera has been initialized";
        spdlog::error(log);
        log = LOG_END;
        spdlog::info(log);
        return false;
    }

    // 记录相机参数
    param_ = param;

    // 记录日志信息
    log = "[" + param_.Key + "] - HuarayCameraParam was set successful";
    spdlog::info(log);
    log = LOG_END;
    spdlog::info(log);

    // 返回设置结果
    return true;
}

// 获取华睿相机的初始化状态
bool HuarayCamera::IsInitialized()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return isInitialized_;
}

// 获取华睿相机的打开状态
bool HuarayCamera::IsOpened()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return isOpened_;
}

// 获取华睿相机的工作状态
bool HuarayCamera::IsNormal()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return isNormal_;
}

// 获取华睿相机的初始化时间戳
uint64_t HuarayCamera::GetInitTimestamp()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return initTimestamp_;
}

// 获取华睿相机的打开时间戳
uint64_t HuarayCamera::GetOpenTimestamp()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return openTimestamp_;
}

// 初始化华睿相机
bool HuarayCamera::Init()
{
    // 锁定华睿相机的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;

    // 记录日志信息
    log = LOG_BEGIN;
    spdlog::info(log);

    // 判断相机是否已经初始化
    if (isInitialized_)
    {
        log = "[" + param_.Key + "] - Camera can not be initialized repeatedly";
        spdlog::warn(log);
        log = LOG_END;
        spdlog::info(log);
        return false;
    }

    // 根据相机的运行状态进行初始化
    if (param_.RuntimeParam.IsOnline)
    {
        // 如果相机需要录像，判断录像文件存储路径是否合法
        if (param_.RuntimeParam.IsRecordVideo)
        {
            // 判断录像文件存储路径是否存在
            if (::access(param_.RuntimeParam.RecordVideoPath.c_str(), R_OK) == -1)
            {
                if (::mkdir(param_.RuntimeParam.RecordVideoPath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)
                {
                    log = "[" + param_.Key + "] - Camera was initialized failure because RuntimeParam's RecordVideoPath is invalid";
                    spdlog::error(log);
                    log = LOG_END;
                    spdlog::info(log);
                    return false;
                }
            }
        }

        // 通过华睿相机的SDK搜索所有设备
        IMV_DeviceList deviceList;
        if (IMV_EnumDevices(&deviceList, interfaceTypeAll) == IMV_OK)
        {
            log = "[" + param_.Key + "] - SDK found devices: " + std::to_string(deviceList.nDevNum);
            spdlog::info(log);
        }
        else
        {
            log = "[" + param_.Key + "] - Camera was initialized failure because SDK can not search for devices";
            spdlog::error(log);
            log = LOG_END;
            spdlog::info(log);
            return false;
        }

        // 搜索与相机标识字符串匹配的设备索引
        int deviceIndex = -1;
        for (int i = 0; i < deviceList.nDevNum; ++i)
        {
            std::string cameraKey = deviceList.pDevInfo[i].cameraKey;
            if (cameraKey == param_.Key)
            {
                deviceIndex = i;
                break;
            }
        }

        // 判断是否搜索成功
        if (deviceIndex == -1)
        {
            log = "[" + param_.Key + "] - Camera was initialized failure because there is no matched device";
            spdlog::error(log);
            log = LOG_END;
            spdlog::info(log);
            return false;
        }

        // 根据设备索引创建相机句柄
        if (IMV_CreateHandle(&cameraHandle_, modeByIndex, (void*)&deviceIndex) == IMV_OK)
        {
            log = "[" + param_.Key + "] - Camera's handle was created successful";
            spdlog::info(log);
        }
        else
        {
            log = "[" + param_.Key + "] - Camera was initialized failure because handle can not be created";
            spdlog::error(log);
            log = LOG_END;
            spdlog::info(log);
            return false;
        }
    }
    else
    {
        // 判断离线视频文件是否存在
        if (::access(param_.RuntimeParam.OfflineVideoName.c_str(), F_OK) == -1)
        {
            log = "[" + param_.Key + "] - Camera was initialized failure because RuntimeParam's OfflineVideo does not exist";
            spdlog::error(log);
            log = LOG_END;
            spdlog::info(log);
            return false;
        }

        // 判断离线视频文件是否可读
        if (::access(param_.RuntimeParam.OfflineVideoName.c_str(), R_OK) == -1)
        {
            log = "[" + param_.Key + "] - Camera was initialized failure because RuntimeParam's OfflineVideo can not be read";
            spdlog::error(log);
            log = LOG_END;
            spdlog::info(log);
            return false;
        }
    }

    // 设置初始化时间戳
    std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
    initTimestamp_ = now.time_since_epoch().count();

    // 设置初始化状态
    isInitialized_ = true;

    // 记录日志信息
    log = "[" + param_.Key + "] - Camera was initialized successful";
    spdlog::info(log);
    log = LOG_END;
    spdlog::info(log);

    // 返回初始化结果
    return true;
}

// 释放华睿相机系统资源
bool HuarayCamera::Release()
{
    // 锁定华睿相机的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;

    // 记录日志信息
    log = LOG_BEGIN;
    spdlog::info(log);

    // 判断相机是否已经打开
    if (isOpened_)
    {
        log = "[" + param_.Key + "] - Camera was released failure because it has been opened";
        spdlog::error(log);
        log = LOG_END;
        spdlog::info(log);
        return false;
    }

    // 判断相机是否初始化完毕
    if (!isInitialized_)
    {
        log = "[" + param_.Key + "] - Camera can not be released repeatedly";
        spdlog::warn(log);
        log = LOG_END;
        spdlog::info(log);
        return false;
    }

    // 释放华睿相机系统资源
    if (param_.RuntimeParam.IsOnline)
    {
        // 销毁相机句柄
        if (IMV_DestroyHandle(cameraHandle_) == IMV_OK)
        {
            log = "[" + param_.Key + "] - Camera's handle was distroyed successful";
            spdlog::info(log);
        }
        else
        {
            log = "[" + param_.Key + "] - Camera's handle was distroyed failure";
            spdlog::error(log);
        }

        // 重置相机句柄
        cameraHandle_ = nullptr;
    }

    // 重置初始化时间戳
    initTimestamp_ = 0;

    // 重置初始化状态
    isInitialized_ = false;

    // 记录日志信息
    log = "[" + param_.Key + "] - Camera was released successful";
    spdlog::info(log);
    log = LOG_END;
    spdlog::info(log);

    // 返回释放结果
    return true;
}

// 打开华睿相机
bool HuarayCamera::Open()
{
    // 锁定华睿相机的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    

    // 记录日志信息
    log = LOG_BEGIN;
    spdlog::info(log);

    // 判断相机是否初始化完毕
    if (!isInitialized_)
    {
        log = "[" + param_.Key + "] - Camera was opened failure because it has not been initialized";
        spdlog::error(log);
        log = LOG_END;
        spdlog::info(log);
        return false;
    }

    // 判断相机是否已经打开
    if (isOpened_)
    {
        log = "[" + param_.Key + "] - Camera can not be opened repeatedly";
        spdlog::warn(log);
        log = LOG_END;
        spdlog::info(log);
        return false;
    }

    // 根据华睿相机的运行状态打开相机
    if (param_.RuntimeParam.IsOnline)
    {
        // 连接相机句柄
        if (IMV_Open(cameraHandle_) == IMV_OK)
        {
            log = "[" + param_.Key + "] - Camera's handle was connected successful";
            spdlog::info(log);
        }
        else
        {
            log = "[" + param_.Key + "] - Camera was opened failure because it's handle can not be connected";
            spdlog::error(log);
            log = LOG_END;
            spdlog::info(log);
            return false;
        }

        // 注册相机连接状态改变事件回调函数
        if (IMV_SubscribeConnectArg(cameraHandle_, HandleConnectChangedEvent, (void*)this) == IMV_OK)
        {
            log = "[" + param_.Key + "] - Camera's ConnectChangedEvent was subscribed successful";
            spdlog::info(log);
        }
        else
        {
            IMV_Close(cameraHandle_);
            log = "[" + param_.Key + "] - Camera was opened failure because it's ConnectChangedEvent can not be subscribed";
            spdlog::error(log);
            log = LOG_END;
            spdlog::info(log);
            return false;
        }

        // 将相机当前硬件参数写入设备
        for (unsigned int i = 0; i < param_.HardwareParams.size(); ++i)
        {
            if (param_.HardwareParams[i].IsSelected)
            {
                WriteHardwareParamToDevice(param_.HardwareParams[i]);
                break;
            }
        }

        // 读取相机的图像宽度
        int64_t width = 0;
        if (IMV_GetIntFeatureValue(cameraHandle_, "Width", &width) != IMV_OK)
        {
            log = "[" + param_.Key + "] - Camera was opened failure because Width feature can not be read";
            spdlog::error(log);
            log = LOG_END;
            spdlog::info(log);
            return false;
        }

        // 读取相机的图像高度
        int64_t height = 0;
        if (IMV_GetIntFeatureValue(cameraHandle_, "Height", &height) != IMV_OK)
        {
            log = "[" + param_.Key + "] - Camera was opened failure because Height feature can not be read";
            spdlog::error(log);
            log = LOG_END;
            spdlog::info(log);
            return false;
        }

        // 更新相机数据帧的宽度和高度
        frameWidth_ = static_cast<int>(width);
        frameHeight_ = static_cast<int>(height);

        // 如果需要录像，则打开相机视频录制器
        if (param_.RuntimeParam.IsRecordVideo)
        {
            // 获取当前日期时间字符串
            std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
            std::time_t now_time = std::chrono::system_clock::to_time_t(now);
            std::tm tm = *std::localtime(&now_time);
            std::stringstream stream;
            stream << std::put_time(&tm, "%Y-%m-%d_%H:%M:%S");
            std::string dateTimeString = stream.str();

            // 创建录像文件名称和录像文件的绝对路径
            std::string recordFileName = "[" + param_.Key + "] - RawVideo_" + dateTimeString + ".avi";
            std::string fullPath = param_.RuntimeParam.RecordVideoPath + recordFileName;



            // 打开数据帧录制器
            if (videoWriter_.open(fullPath,
                                  cv::VideoWriter::fourcc('M','P','4','2'),
                                  param_.RuntimeParam.RecordVideoFps,
                                  cv::Size(frameWidth_, frameHeight_),
                                  true))
            {
                log = "[" + param_.Key + "] - Camera was started recording successful";
                spdlog::info(log);
            }
            else
            {
                log = "[" + param_.Key + "] - Camera was opened failure because it can not be started recording";
                spdlog::error(log);
                log = LOG_END;
                spdlog::info(log);
                return false;
            }
        }

        // 开始拉流
        if (IMV_StartGrabbingEx(cameraHandle_, 0, grabStrartegyLatestImage) == IMV_OK)
        {
            log = "[" + param_.Key + "] - Camera was started grabbing successful";
            spdlog::info(log);
        }
        else
        {
            log = "[" + param_.Key + "] - Camera was opened failure because it can not be started grabbing";
            spdlog::error(log);
            log = LOG_END;
            spdlog::info(log);
            return false;
        }

        // 重置相机数据统计信息
        if (IMV_ResetStatisticsInfo(cameraHandle_) == IMV_OK)
        {
            log = "[" + param_.Key + "] - Camera's statistics was reset successful";
            spdlog::info(log);
        }
        else
        {
            log = "[" + param_.Key + "] - Camera's statistics was reset failure";
            spdlog::error(log);
        }
    }
    else
    {
        // 打开视频捕捉器
        if(videoCapture_.open(param_.RuntimeParam.OfflineVideoName))
        {
            // 读取离线视频文件数据帧的高度和宽度
            double width = videoCapture_.get(cv::CAP_PROP_FRAME_WIDTH);
            double height = videoCapture_.get(cv::CAP_PROP_FRAME_HEIGHT);

            // 更新相机数据帧的宽度和高度
            frameWidth_ = static_cast<int>(width);
            frameHeight_ = static_cast<int>(height);

            // 记录日志信息
            log = "[" + param_.Key + "] - OfflineVideo has been opened successful";
            spdlog::info(log);
        }
        else
        {
            log = "[" + param_.Key + "] - Camera was opened failure because OfflineVideo can't be opened";
            spdlog::error(log);
            log = LOG_END;
            spdlog::info(log);
            return false;
        }
    }

    // 启动相机缓存数据帧更新线程
    updateDataSwitch_ = true;
    updateDataThread_ = std::thread(&HuarayCamera::UpdateData, this);
    log = "[" + param_.Key + "] - Camera's UpdateDataThread was started successful";
    spdlog::info(log);

    // 启动相机缓存数据帧录制线程
    if (param_.RuntimeParam.IsOnline && param_.RuntimeParam.IsRecordVideo)
    {
        recordVideoSwitch_ = true;
        recordVideoThread_ = std::thread(&HuarayCamera::RecordVideo, this);
        log = "[" + param_.Key + "] - Camera's RecordVideoThread was started successful";
        spdlog::info(log);
    }

    // 设置打开时间戳
    std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
    openTimestamp_ = now.time_since_epoch().count();

    // 设置打开状态
    isOpened_ = true;

    // 设置工作状态
    isNormal_ = true;

    // 记录日志信息
    log = "[" + param_.Key + "] - Camera was opened successful";
    spdlog::info(log);
    log = LOG_END;
    spdlog::info(log);

    // 返回打开结果
    return true;
}

// 关闭华睿相机
bool HuarayCamera::Close()
{
    // 锁定华睿相机的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    

    // 记录日志信息
    log = LOG_BEGIN;
    spdlog::info(log);

    // 判断相机是否已经打开
    if (!isOpened_)
    {
        log = "[" + param_.Key + "] - Camera can not be closed repeatedly";
        spdlog::warn(log);
        log = LOG_END;
        spdlog::info(log);
        return false;
    }

    // 停止相机缓存数据帧更新线程
    updateDataSwitch_ = false;
    if (updateDataThread_.joinable())
    {
        updateDataThread_.join();
    }
    log = "[" + param_.Key + "] - Camera's UpdateDataThread was stopped successful";
    spdlog::info(log);

    // 判断相机是否为在线模式
    if (param_.RuntimeParam.IsOnline)
    {
        // 停止相机缓存数据帧录制线程
        if (param_.RuntimeParam.IsRecordVideo)
        {
            recordVideoSwitch_ = false;
            if (recordVideoThread_.joinable())
            {
                recordVideoThread_.join();
            }
            log = "[" + param_.Key + "] - Camera's RecordVideoThread was stopped successful";
            spdlog::info(log);
        }

        // 释放数据帧录制器
        if (videoWriter_.isOpened())
        {
            videoWriter_.release();
        }

        // 停止拉流
        if (IMV_StopGrabbing(cameraHandle_) == IMV_OK)
        {
            log = "[" + param_.Key + "] - Camera was stoped grabbing successful";
            spdlog::info(log);
        }
        else
        {
            log = "[" + param_.Key + "] - Camera was stoped grabbing failure";
            spdlog::error(log);
        }

        // 关闭相机句柄
        if (IMV_IsOpen(cameraHandle_))
        {
            if (IMV_Close(cameraHandle_) == IMV_OK)
            {
                log = "[" + param_.Key + "] - Camera's handle was disconnected successful";
                spdlog::info(log);
            }
            else
            {
                log = "[" + param_.Key + "] - Camera's handle was disconnected failure";
                spdlog::error(log);
            }
        }
    }
    else
    {
        // 释放视频捕捉器
        if (videoCapture_.isOpened())
        {
            videoCapture_.release();
        }
    }

    // 重置打开时间戳
    openTimestamp_ = 0;

    // 重置打开状态
    isOpened_ = false;

    // 重置工作状态
    isNormal_ = false;

    // 记录日志信息
    log = "[" + param_.Key + "] - Camera was closed successful";
    spdlog::info(log);
    log = LOG_END;
    spdlog::info(log);

    // 返回关闭结果
    return true;
}

// 切换华睿相机硬件参数
bool HuarayCamera::SwitchHardwareParam(const unsigned int &hardwareParamIndex)
{
    // 锁定华睿相机的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    

    // 记录日志信息
    log = LOG_BEGIN;
    spdlog::info(log);

    // 判断相机是否处于在线运行状态
    if (!param_.RuntimeParam.IsOnline)
    {
        log = "[" + param_.Key + "] - HuarayCameraHardwareParam was switched failure because camera is not in Online mode";
        spdlog::error(log);
        log = LOG_END;
        spdlog::info(log);
        return false;
    }

    // 判断相机是否已经打开
    if (!isOpened_)
    {
        log = "[" + param_.Key + "] - HuarayCameraHardwareParam was switched failure because camera has not been opened";
        spdlog::error(log);
        log = LOG_END;
        spdlog::info(log);
        return false;
    }

    // 更新相机硬件参数属性，将硬件参数写入设备
    if (hardwareParamIndex < param_.HardwareParams.size())
    {
        // 重置所有相机硬件参数的选择状态
        for (unsigned int i = 0; i < param_.HardwareParams.size(); ++i)
        {
            param_.HardwareParams[i].IsSelected = false;
        }

        // 设置要切换的硬件参数的选择状态
        param_.HardwareParams[hardwareParamIndex].IsSelected = true;

        // 将硬件参数写入设备
        WriteHardwareParamToDevice(param_.HardwareParams[hardwareParamIndex]);
    }
    else
    {
        log = "[" + param_.Key + "] - HuarayCameraHardwareParam was switched failure because hardwareParamIndex is invalid";
        spdlog::error(log);
        log = LOG_END;
        spdlog::info(log);
        return false;
    }

    // 记录日志信息
    log = "[" + param_.Key + "] - HuarayCameraHardwareParam was switched successful";
    spdlog::info(log);
    log = LOG_END;
    spdlog::info(log);

    // 返回切换结果
    return true;
}

// 重置缓存的华睿相机数据帧
void HuarayCamera::ResetData()
{
    // 锁定相机数据缓冲区
    std::unique_lock<std::shared_mutex> uniqueLock(dataMutex_);

    // 重置相机数据帧的索引
    data_.Index = 0;

    // 重置相机数据帧的时间戳
    std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
    data_.Timestamp = now.time_since_epoch().count();

    // 重置相机数据帧的图像
    data_.Image = cv::Mat(frameHeight_,
                          frameWidth_,
                          CV_8UC3,
                          cv::Scalar(0, 0, 255));
    cv::putText(data_.Image,
                "The cached data has been reset",
                cv::Point(frameWidth_ / 2 - 200, frameHeight_ / 2),
                cv::FONT_HERSHEY_SIMPLEX,
                1.0,
                cv::Scalar(0, 0, 0),
                2);
}

// 获取缓存的华睿相机数据帧
void HuarayCamera::GetData(HuarayCameraData *data)
{
    std::shared_lock<std::shared_mutex> sharedLock(dataMutex_);
    data->Index = data_.Index;
    data->Timestamp = data_.Timestamp;
    data_.Image.copyTo(data->Image);
}

// 获取当前华睿相机状态帧
bool HuarayCamera::GetStatus(HuarayCameraStatus *status)
{
    // 锁定华睿相机的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    

    // 判断相机是否处于在线状态
    if (!param_.RuntimeParam.IsOnline)
    {
        log = "[" + param_.Key + "] - HuarayCameraStatus was got failure because camera is not in Online mode";
        spdlog::error(log);
        return false;
    }

    // 判断相机是否打开
    if (!isOpened_)
    {
        log = "[" + param_.Key + "] - HuarayCameraStatus was got failure because camera has not been opened";
        spdlog::error(log);
        return false;
    }

    // 获取相机的标识符、连接状态和工作状态
    status->Key = param_.Key;
    status->IsConnected = IMV_IsOpen(cameraHandle_);
    status->IsWorking = IMV_IsGrabbing(cameraHandle_);

    // 获取统计信息
    IMV_StreamStatisticsInfo statisticsInfo;
    if (IMV_GetStatisticsInfo(cameraHandle_, &statisticsInfo) == IMV_OK)
    {
        switch (statisticsInfo.nCameraType)
        {
            case typeGigeCamera:
            {
                status->ErrorFrame = statisticsInfo.gigeStatisticsInfo.imageError;
                status->LostPacketFrame = statisticsInfo.gigeStatisticsInfo.lostPacketBlock;
                status->TotalFrame = statisticsInfo.gigeStatisticsInfo.imageReceived;
                status->BandWidth = statisticsInfo.gigeStatisticsInfo.bandwidth;
                status->FPS = statisticsInfo.gigeStatisticsInfo.fps;
                break;
            }

            case typeU3vCamera:
            {
                status->ErrorFrame = statisticsInfo.u3vStatisticsInfo.imageError;
                status->LostPacketFrame = statisticsInfo.u3vStatisticsInfo.lostPacketBlock;
                status->TotalFrame = statisticsInfo.u3vStatisticsInfo.imageReceived;
                status->BandWidth = statisticsInfo.u3vStatisticsInfo.bandwidth;
                status->FPS = statisticsInfo.u3vStatisticsInfo.fps;
                break;
            }

            case typePCIeCamera:
            {
                status->ErrorFrame = statisticsInfo.pcieStatisticsInfo.imageError;
                status->LostPacketFrame = statisticsInfo.pcieStatisticsInfo.lostPacketBlock;
                status->TotalFrame = statisticsInfo.pcieStatisticsInfo.imageReceived;
                status->BandWidth = statisticsInfo.pcieStatisticsInfo.bandwidth;
                status->FPS = statisticsInfo.pcieStatisticsInfo.fps;
            }

            default:
                break;
        }
    }
    else
    {
        log = "[" + param_.Key + "] - HuarayCameraStatus was got failure because statistics information can not be read";
        spdlog::error(log);
        return false;
    }

    // 返回获取结果
    return true;
}

// 将华睿相机的硬件参数写入设备
void HuarayCamera::WriteHardwareParamToDevice(const HuarayCameraHardwareParam &hardwareParam)
{
    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    

    // 写入IsBalanceWhiteAuto
    if (hardwareParam.IsWhiteBalanceAuto)
    {
        if (IMV_SetEnumFeatureValue(cameraHandle_,
                                    "BalanceWhiteAuto",
                                    2) == IMV_OK)
        {
            log = "[" + param_.Key + "] - HardwareParam's IsBalanceWhiteAuto was writen successful";
            spdlog::info(log);
        }
        else
        {
            log = "[" + param_.Key + "] - HardwareParam's IsBalanceWhiteAuto was writen failure";
            spdlog::error(log);
        }
    }
    else
    {
        if (IMV_SetEnumFeatureValue(cameraHandle_,
                                    "BalanceWhiteAuto",
                                    0) == IMV_OK)
        {
            log = "[" + param_.Key + "] - HardwareParam's IsBalanceWhiteAuto was writen successful";
            spdlog::info(log);
        }
        else
        {
            log = "[" + param_.Key + "] - HardwareParam's IsBalanceWhiteAuto was writen failure";
            spdlog::error(log);
        }
    }

    // 写入IsExposureAuto和ExposureTime
    if (hardwareParam.IsExposureAuto)
    {
        // 写入IsExposureAuto
        if (IMV_SetEnumFeatureValue(cameraHandle_,
                                    "ExposureAuto",
                                    2) == IMV_OK)
        {
            log = "[" + param_.Key + "] - HardwareParam's IsExposureAuto was writen successful";
            spdlog::info(log);
        }
        else
        {
            log = "[" + param_.Key + "] - HardwareParam's IsExposureAuto was writen failure";
            spdlog::error(log);
        }
    }
    else
    {
        // 写入IsExposureAuto
        if (IMV_SetEnumFeatureValue(cameraHandle_,
                                    "ExposureAuto",
                                    0) == IMV_OK)
        {
            log = "[" + param_.Key + "] - HardwareParam's IsExposureAuto was writen successful";
            spdlog::info(log);
        }
        else
        {
            log = "[" + param_.Key + "] - HardwareParam's IsExposureAuto was writen failure";
            spdlog::error(log);
        }

        // 写入ExposureTime
        if (IMV_SetDoubleFeatureValue(cameraHandle_,
                                      "ExposureTime",
                                      hardwareParam.ExposureTime) == IMV_OK)
        {
            log = "[" + param_.Key + "] - HardwareParam's ExposureTime was writen successful";
            spdlog::info(log);
        }
        else
        {
            log = "[" + param_.Key + "] - HardwareParam's ExposureTime was writen failure";
            spdlog::error(log);
        }
    }

    // 写入GainRaw
    if (IMV_SetDoubleFeatureValue(cameraHandle_,
                                  "GainRaw",
                                  hardwareParam.GainRaw) == IMV_OK)
    {
        log = "[" + param_.Key + "] - HardwareParam's GainRaw was writen successful";
        spdlog::info(log);
    }
    else
    {
        log = "[" + param_.Key + "] - HardwareParam's GainRaw was writen failure";
        spdlog::error(log);
    }

    // 写入Gamma
    if (IMV_SetDoubleFeatureValue(cameraHandle_,
                                  "Gamma",
                                  hardwareParam.Gamma) == IMV_OK)
    {
        log = "[" + param_.Key + "] - HardwareParam's Gamma was writen successful";
        spdlog::info(log);
    }
    else
    {
        log = "[" + param_.Key + "] - HardwareParam's Gamma was writen failure";
        spdlog::error(log);
    }

    // 写入Brightness
    if (IMV_SetIntFeatureValue(cameraHandle_,
                               "Brightness",
                               hardwareParam.Brightness) == IMV_OK)
    {
        log = "[" + param_.Key + "] - HardwareParam's Brightness was writen successful";
        spdlog::info(log);
    }
    else
    {
        log = "[" + param_.Key + "] - HardwareParam's Brightness was writen failure";
        spdlog::error(log);
    }
}

// ******************************  HuarayCamera类的私有函数  ******************************


// 更新缓存的华睿相机数据帧
void HuarayCamera::UpdateData()
{
    // 修改线程名称
    std::string threadName = "update_camera_data";
    prctl(PR_SET_NAME, threadName.c_str());

    // 设置线程绑定的CPU内核
    // 参考网址：https://antrn.blog.csdn.net/article/details/114263105?spm=1001.2014.3001.5502
    //         https://blog.csdn.net/qq_34440148/article/details/121603698?spm=1001.2014.3001.5502
    //         https://blog.csdn.net/liaoxiangui/article/details/7905612
    //         https://blog.csdn.net/zxc024000/article/details/79438061
    //         https://www.bbsmax.com/A/q4zVKp9XJK/
    int coreNumber = get_nprocs();
    int coreIndex = param_.RuntimeParam.UpdateDataCpuCore;
    if ((coreIndex >= 0) && (coreIndex < coreNumber))
    {
        cpu_set_t mask;
        CPU_ZERO(&mask);
        CPU_SET(coreIndex, &mask);
        sched_setaffinity(0, sizeof(mask), &mask);
    }

    // 更新缓存的华睿相机数据帧
    if (param_.RuntimeParam.IsOnline)
    {
        UpdateDataFromOnlineDevice();
    }
    else
    {
        UpdateDataFromOfflineVideo();
    }
}

// 通过离线视频更新缓存的华睿相机数据帧
void HuarayCamera::UpdateDataFromOfflineVideo()
{
    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    

    // 读取离线视频文件的帧数
    int frameCount = static_cast<int>(videoCapture_.get(cv::CAP_PROP_FRAME_COUNT));

    // 读取离线视频文件的帧率，计算相邻数据帧之间的时间延迟(微秒)
    double fps = videoCapture_.get(cv::CAP_PROP_FPS);
    auto frameDelay = static_cast<uint64_t>(1000000000.0 / fps);

    // 循环读取离线视频图像
    cv::Mat image;
    while (updateDataSwitch_)
    {
        // 获取起始时间戳
        std::chrono::time_point<std::chrono::steady_clock> beginTime = std::chrono::steady_clock::now();
        uint64_t beginTimestamp = beginTime.time_since_epoch().count();

        // 读取离线视频数据帧
        if (!videoCapture_.read(image))
        {
            break;
        }

        // 获取当前时间戳
        std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
        uint64_t timestamp = now.time_since_epoch().count();

        // 更新缓存的华睿相机数据帧
        dataMutex_.lock();
        data_.Index++;
        data_.Timestamp = timestamp;
        image.copyTo(data_.Image);
        dataMutex_.unlock();

        // 获取当前帧图像的位置，判断当前帧图像是否为最后一帧图像
        // TODO 对于某些离线视频文件，使用capture.get(cv::CAP_PROP_FRAME_COUNT)函数读取到的frameCount为0；\n
        //      无法通过framePosition判断是否播放到最后一帧图像，必须重新打开离线视频文件。
        int framePosition = static_cast<int>(videoCapture_.get(cv::CAP_PROP_POS_FRAMES));
        if ((frameCount > 0) && (framePosition == frameCount))
        {
            // 重置缓存的华睿相机数据帧
            ResetData();

            // 重置当前帧图像的位置
            if (videoCapture_.set(cv::CAP_PROP_POS_FRAMES, 0))
            {
                // 记录日志信息
                log = "[" + param_.Key + "] - OfflineVideo's image position has been reset successful";
                spdlog::info(log);
            }
            else
            {
                // 记录日志信息
                log = "[" + param_.Key + "] - OfflineVideo's image position has been reset failure";
                spdlog::error(log);
            }
        }

        // 获取截止时间戳
        std::chrono::time_point<std::chrono::steady_clock> endTime = std::chrono::steady_clock::now();
        uint64_t endTimestamp = endTime.time_since_epoch().count();

        // 线程延时，保证离线视频图像读取速度正常
        uint64_t timeSpan = endTimestamp - beginTimestamp;
        if (frameDelay > timeSpan)
        {
            std::this_thread::sleep_for(std::chrono::nanoseconds(frameDelay - timeSpan));
        }
    }

    // 重置缓存的华睿相机数据帧
    ResetData();

    // 记录日志信息
    if (updateDataSwitch_)
    {
        log = "[" + param_.Key + "] - OfflineVideo playback was completed because it's invalid FrameCount property";
        spdlog::error(log);
    }
    else
    {
        log = "[" + param_.Key + "] - OfflineVideo playback was completed gracefully";
        spdlog::info(log);
    }
}

// 通过在线设备更新缓存的华睿相机数据帧
void HuarayCamera::UpdateDataFromOnlineDevice()
{
    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    

    // 初始化原始数据帧
    IMV_Frame frame;

    // 初始化异常状态计数器
    unsigned int abnormalCounter = 0;

    // 循环读取并转换原始数据帧
    while (updateDataSwitch_)
    {
        // 获取原始数据帧
        if (IMV_GetFrame(cameraHandle_, &frame, 100) != IMV_OK)
        {
            // 判断并修改相机的工作状态
            // 注意：在实际使用时，可以根据实际需要修改判断条件
            abnormalCounter++;
            if (abnormalCounter >= 500)
            {
                isNormal_ = false;
            }

            // 记录日志信息
            log = "[" + param_.Key + "] - `Raw` frame was read failure";
            spdlog::error(log);

            continue;
        }

        // 重置异常状态计数器
        abnormalCounter = 0;

        // 初始化BGR24图像的数据长度和数据缓冲区
        int bgr24Size = static_cast<int>(frame.frameInfo.width * frame.frameInfo.height * 3);
        unsigned char bgr24Buffer[bgr24Size];

        // 初始化转换参数
        IMV_PixelConvertParam pixelConvertParam;
        pixelConvertParam.nWidth = frame.frameInfo.width;
        pixelConvertParam.nHeight = frame.frameInfo.height;
        pixelConvertParam.ePixelFormat = frame.frameInfo.pixelFormat;
        pixelConvertParam.pSrcData = frame.pData;
        pixelConvertParam.nSrcDataLen = frame.frameInfo.size;
        pixelConvertParam.nPaddingX = frame.frameInfo.paddingX;
        pixelConvertParam.nPaddingY = frame.frameInfo.paddingY;
        pixelConvertParam.eBayerDemosaic = demosaicNearestNeighbor;
        pixelConvertParam.eDstPixelFormat = gvspPixelBGR8;
        pixelConvertParam.pDstBuf = bgr24Buffer;
        pixelConvertParam.nDstBufSize = bgr24Size;

        // 转换原始数据帧的像素格式
        if (IMV_PixelConvert(cameraHandle_, &pixelConvertParam) == IMV_OK)
        {
            // 创建BGR24图像
            int height = static_cast<int>(frame.frameInfo.height);
            int width = static_cast<int>(frame.frameInfo.width);
            cv::Mat bgr24Image(height, width, CV_8UC3, bgr24Buffer, width * 3);

            // 更新缓存的相机数据帧
            dataMutex_.lock();
            data_.Index++;
            data_.Timestamp = frame.frameInfo.timeStamp;
            bgr24Image.copyTo(data_.Image);
            dataMutex_.unlock();

            // 释放原始数据帧
            IMV_ReleaseFrame(cameraHandle_, &frame);
        }
        else
        {
            log = "[" + param_.Key + "] - Raw frame was converted failure";
            spdlog::error(log);
        }
    }

    // 重置缓存的华睿相机数据帧
    ResetData();

    // 记录日志信息
    log = "[" + param_.Key + "] - OnlineVideo reading was completed gracefully";
    spdlog::info(log);
}

// 录制缓存的华睿相机数据帧
void HuarayCamera::RecordVideo()
{
    // 修改线程名称
    std::string threadName = "record_camera_data";
    prctl(PR_SET_NAME, threadName.c_str());

    // 设置线程绑定的CPU内核
    // 参考网址：https://antrn.blog.csdn.net/article/details/114263105?spm=1001.2014.3001.5502
    //         https://blog.csdn.net/qq_34440148/article/details/121603698?spm=1001.2014.3001.5502
    //         https://blog.csdn.net/liaoxiangui/article/details/7905612
    //         https://blog.csdn.net/zxc024000/article/details/79438061
    //         https://www.bbsmax.com/A/q4zVKp9XJK/
    int coreNumber = get_nprocs();
    int coreIndex = param_.RuntimeParam.RecordVideoCpuCore;
    if ((coreIndex >= 0) && (coreIndex < coreNumber))
    {
        cpu_set_t mask;
        CPU_ZERO(&mask);
        CPU_SET(coreIndex, &mask);
        sched_setaffinity(0, sizeof(mask), &mask);
    }

    // 初始化临时图像
    cv::Mat image;

    // 计算采样周期；单位：纳秒
    auto samplePeriod = static_cast<uint64_t>(1000000000.0 / param_.RuntimeParam.RecordVideoFps);

    // 录制相机数据帧
    while (recordVideoSwitch_)
    {
        // 获取起始时间戳
        std::chrono::time_point<std::chrono::steady_clock> beginTime = std::chrono::steady_clock::now();
        uint64_t beginTimestamp = beginTime.time_since_epoch().count();

        // 复制相机数据帧中的图像
        dataMutex_.lock();
        data_.Image.copyTo(image);
        dataMutex_.unlock();

        // 判断图像参数是否与视频录制器匹配
        if ((image.cols != frameWidth_) || (image.rows != frameHeight_) || (image.type() != CV_8UC3))
        {
            continue;
        }

        // 保存相机数据帧中的图像
        videoWriter_.write(image);

        // 获取截止时间戳
        std::chrono::time_point<std::chrono::steady_clock> endTime = std::chrono::steady_clock::now();
        uint64_t endTimestamp = endTime.time_since_epoch().count();

        // 计算相机数据帧的保存时间
        uint64_t writeSpan = endTimestamp - beginTimestamp;

        // 线程延时，确保录制视频的帧率正常
        if (samplePeriod > writeSpan)
        {
            std::this_thread::sleep_for(std::chrono::nanoseconds(samplePeriod - writeSpan));
        }
    }
}

// ******************************  HuarayCamera类的事件回调函数  ******************************

// 相机连接状态改变事件回调函数
void HandleConnectChangedEvent(const IMV_SConnectArg* pConnectArg, void* pUser)
{
    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    

    // 获取相机对象指针和相机标识字符串
    auto camera = (HuarayCamera *)pUser;
    std::string cameraKey = camera->GetParam().Key;

    // 判断相机连接状态
    if (pConnectArg->event == offLine)
    {
        // 记录日志信息
        log = "[" + cameraKey + "] - Camera was disconnected for some reason......";
        spdlog::warn(log);

        // 关闭相机
        camera->Close();
    }
    else
    {
        // 记录日志信息
        log = "[" + cameraKey + "] - Camera was connected for some reason......";
        spdlog::warn(log);

        // 打开相机
        camera->Open();
    }
}
