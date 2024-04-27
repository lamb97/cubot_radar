//
// Created by plutoli on 2022/4/5.
//

#include "huaray_camera_runtime_param.h"

// ******************************  HuarayCameraRuntimeParam类的公有函数  ******************************

// 构造函数
HuarayCameraRuntimeParam::HuarayCameraRuntimeParam():
    IsOnline(true),
    IsRecordVideo(false),
    RecordVideoQuality(50),
    UpdateDataCpuCore(-1),
    RecordVideoCpuCore(-1),
    RecordVideoFps(20.0),
    OfflineVideoName(),
    RecordVideoPath()
{
}