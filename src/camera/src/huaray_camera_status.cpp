//
// Created by plutoli on 2022/4/5.
//

#include "huaray_camera_status.h"

// ******************************  HuarayCameraStatus类的公有函数  ******************************

// 构造函数
HuarayCameraStatus::HuarayCameraStatus():
    Key(),
    IsConnected(false),
    IsWorking(false),
    ErrorFrame(0),
    LostPacketFrame(0),
    TotalFrame(0),
    BandWidth(0.0),
    FPS(0.0)
{
}

// 获取状态帧序列化之后的字节长度
unsigned int HuarayCameraStatus::GetSize() const
{
    // 初始化序列化之后的字节总长度
    unsigned int totalSize = 0;

    // 计算状态帧中的每一个数据长度
    unsigned int keyLength = Key.length();
    unsigned int keyLengthSize = sizeof(keyLength);
    totalSize += keyLengthSize;
    totalSize += keyLength;
    unsigned int isConnectedSize = sizeof(IsConnected);
    totalSize += isConnectedSize;
    unsigned int errorFrameSize = sizeof(ErrorFrame);
    totalSize += errorFrameSize;
    unsigned int lostPacketFrameSize = sizeof(LostPacketFrame);
    totalSize += lostPacketFrameSize;
    unsigned int totalFrameSize = sizeof(TotalFrame);
    totalSize += totalFrameSize;
    unsigned int bandWidthSize = sizeof(BandWidth);
    totalSize += bandWidthSize;
    unsigned int fpsSize = sizeof(FPS);
    totalSize += fpsSize;

    // 返回数据长度
    return totalSize;
}

// 序列化华睿相机状态帧
void HuarayCameraStatus::Serialize(unsigned char *byteArray) const
{
    // 初始化字节索引
    unsigned int index = 0;

    // 序列化keyLength
    unsigned int keyLength = Key.length();
    unsigned int keyLengthSize = sizeof(keyLength);
    memcpy(byteArray + index, &keyLength, keyLengthSize);
    index += keyLengthSize;

    // 序列化Key
    memcpy(byteArray + index, Key.c_str(), keyLength);
    index += keyLength;

    // 序列化IsConnected
    unsigned int isConnectedSize = sizeof(IsConnected);
    memcpy(byteArray + index, &IsConnected, isConnectedSize);
    index += isConnectedSize;

    // 序列化ErrorFrame
    unsigned int errorFrameSize = sizeof(ErrorFrame);
    memcpy(byteArray + index, &ErrorFrame, errorFrameSize);
    index += errorFrameSize;

    // 序列化LostPacketFrame
    unsigned int lostPacketFrameSize = sizeof(LostPacketFrame);
    memcpy(byteArray + index, &LostPacketFrame, lostPacketFrameSize);
    index += lostPacketFrameSize;

    // 序列化TotalFrame
    unsigned int totalFrameSize = sizeof(TotalFrame);
    memcpy(byteArray + index, &TotalFrame, totalFrameSize);
    index += totalFrameSize;

    // 序列化BandWidth
    unsigned int bandWidthSize = sizeof(BandWidth);
    memcpy(byteArray + index, &BandWidth, bandWidthSize);
    index += bandWidthSize;

    // 序列化FPS
    unsigned int fpsSize = sizeof(FPS);
    memcpy(byteArray + index, &FPS, fpsSize);
}

// 从字节数组中加载华睿相机状态帧
HuarayCameraStatus HuarayCameraStatus::LoadFromBytes(const unsigned char *byteArray)
{
    // 初始化字节索引
    unsigned int index = 0;

    // 加载keyLength
    unsigned int keyLength = 0;
    unsigned int keyLengthSize = sizeof(keyLength);
    memcpy(&keyLength, byteArray + index, keyLengthSize);
    index += keyLengthSize;

    // 加载key
    std::string key(keyLength, 0);
    memcpy((unsigned char *)key.c_str(), byteArray + index, keyLength);
    index += keyLength;

    // 加载isConnected
    bool isConnected = false;
    unsigned int isConnectedSize = sizeof(isConnected);
    memcpy(&isConnected, byteArray + index, isConnectedSize);
    index += isConnectedSize;

    // 加载errorFrame
    unsigned int errorFrame = 0;
    unsigned int errorFrameSize = sizeof(errorFrame);
    memcpy(&errorFrame, byteArray + index, errorFrameSize);
    index += errorFrameSize;

    // 加载lostPacketFrame
    unsigned int lostPacketFrame = 0;
    unsigned int lostPacketFrameSize = sizeof(lostPacketFrame);
    memcpy(&lostPacketFrame, byteArray + index, lostPacketFrameSize);
    index += lostPacketFrameSize;

    // 加载totalFrame
    unsigned int totalFrame = 0;
    unsigned int totalFrameSize = sizeof(totalFrame);
    memcpy(&totalFrame, byteArray + index, totalFrameSize);
    index += totalFrameSize;

    // 加载bandWidth
    double bandWidth = 0.0;
    unsigned int bandWidthSize = sizeof(bandWidth);
    memcpy(&bandWidth, byteArray + index, bandWidthSize);
    index += bandWidthSize;

    // 加载fps
    double fps = 0.0;
    unsigned int fpsSize = sizeof(fps);
    memcpy(&fps, byteArray + index, fpsSize);
    index += fpsSize;

    // 初始化华睿相机状态帧
    HuarayCameraStatus huarayCameraStatus;
    huarayCameraStatus.Key = key;
    huarayCameraStatus.IsConnected = isConnected;
    huarayCameraStatus.ErrorFrame = errorFrame;
    huarayCameraStatus.LostPacketFrame = lostPacketFrame;
    huarayCameraStatus.TotalFrame = totalFrame;
    huarayCameraStatus.BandWidth = bandWidth;
    huarayCameraStatus.FPS = fps;

    // 返回华睿相机状态帧
    return huarayCameraStatus;
}