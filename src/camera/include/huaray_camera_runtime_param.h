//
// Created by plutoli on 2022/4/5.
//

#ifndef CUBOT_BRAIN_HUARAY_CAMERA_RUNTIME_PARAM_H
#define CUBOT_BRAIN_HUARAY_CAMERA_RUNTIME_PARAM_H

#include <string>

/**
 * @brief 华睿相机的运行时参数
 */
class HuarayCameraRuntimeParam
{
public:
    bool IsOnline;                      ///< 是否在线运行
    bool IsRecordVideo;                 ///< 是否录制视频
    unsigned int RecordVideoQuality;    ///< 录制视频的质量；取值范围为[1,100]，默认为50
    int UpdateDataCpuCore;              ///< 更新数据帧任务的CPU内核编号；默认为-1
    int RecordVideoCpuCore;             ///< 录制视频流任务的CPU内核编号；默认为-1
    double RecordVideoFps;              ///< 录制视频的帧率；默认为20.0
    std::string OfflineVideoName;       ///< 离线视频文件名称
    std::string RecordVideoPath;        ///< 录像文件存储路径

    /**
     * @brief 构造函数
     */
    HuarayCameraRuntimeParam();

    /**
     * @brief 析构函数
     */
    ~HuarayCameraRuntimeParam() = default;
};

#endif //CUBOT_BRAIN_HUARAY_CAMERA_RUNTIME_PARAM_H