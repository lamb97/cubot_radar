//
// Created by plutoli on 2021/8/14.
//

#include <string>
#include <cstring>
#include "huaray_camera_status.h"

int main(int argc, char *argv[])
{
    // 序列化华睿相机状态帧
    HuarayCameraStatus status1;
    status1.Key = "HuarayCameraStatus";
    status1.IsConnected = false;
    status1.ErrorFrame = 10;
    status1.LostPacketFrame = 15;
    status1.TotalFrame = 20;
    status1.BandWidth = 4.9;
    status1.FPS = 9.4;
    unsigned int statusSize = status1.GetSize();
    unsigned char byteArray[statusSize];
    status1.Serialize(byteArray);

    // 加载华睿相机状态帧
    HuarayCameraStatus status2 = HuarayCameraStatus::LoadFromBytes(byteArray);

    return 0;
}
