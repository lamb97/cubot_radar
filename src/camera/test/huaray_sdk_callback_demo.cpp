
/// \file
/// \~chinese
/// \brief 断线重连范例
/// \example main.cpp
/// \~english
/// \brief resume connect sample
/// \example main.cpp


// *******本Demo为简单演示SDK的使用，没有附加修改相机IP的代码，
// 在运行之前，请使用相机客户端修改相机IP地址的网段与主机的网段一致*********
//********This demo is a simple demonstration of the use of the SDK. There is no additional code to modify the camera IP.
//Before running, please use the camera client to modify the network segment of the camera IP address to be consistent with that of the host.*********

#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include "IMVApi.h"
#include "IMVDefines.h"


using namespace Huaray::GenICam;
using namespace Huaray::Infra;

#ifdef _WIN32
#define SLEEP(a)	Sleep(a);
#else
#define SLEEP(a)	usleep(a * 1000);
#endif

ICameraPtr cameraSptr;//相机对象 		eng//camera object
IStreamSourcePtr streamPtr;//流对象 	eng://stream object
IEventSubscribePtr eventPtr;//事件对象	eng://event object

bool isMainThreadExit = false;
char pUser[10] = "Camera";

static void onDeviceLinkNotify(const SConnectArg& conArg, void* pUser);
static void onGetFrame(const CFrame &frame, const void* pUser);
static void onStreamEvent(const SStreamArg& streamArg, void* pUser);
static void onChannelCallback(const SMsgChannelArg& msgChannelArg, void* pUser);

// 尝试重新连接相机
// try reconnecting camera
static void TryToResumeConnect(void)
{
    if (NULL == cameraSptr.get())
    {
        printf("cameraSptr is null.\n");
        return;
    }

    // 断开相机连接
    // disconnect cammera
    if (cameraSptr->disConnect())
    {
        printf("*******  disConnect success  *******\n");
    }
    else
    {
        printf("*******  disConnect failed  *******\n");
    }

    while (1)
    {
        // 退出程序时，不必连接相机
        // When you exit the program, you do not have to connect the camera
        if(isMainThreadExit)
        {
            printf("Main thread has been exited\n");
            return;
        }

        // 重新连接相机
        // reconnecting camera
        if (!cameraSptr->connect())
        {
            printf("*******  Retry connect fail  *******\n");
            SLEEP(500);
            continue;
        }
        else
        {
            printf("*******  Retry connect success  *******\n");
            break;
        }
    }

    // 重新创建流对象
    // Recreate stream object
    streamPtr.reset();
    streamPtr = CSystem::getInstance().createStreamSource(cameraSptr);
    if (NULL == streamPtr.get())
    {
        printf("create stream obj  fail.\n");
        return;
    }

    // 注册回调
    // register callback functiohn
    if (streamPtr->attachGrabbingEx(onGetFrame, (void*)pUser))
    {
        printf("*******  attach stream callback success  *******\n");
    }
    else
    {
        printf("*******  attach stream callback fail  *******\n");
    }

    // 开始拉流
    // start grabbing
    if (!streamPtr->startGrabbing())
    {
        printf("*******  start grab fail  *******\n");
        return;
    }

    printf("*******  start grab success  *******\n");

    return;
}

// 取流回调函数
// get stream callback function
static void onGetFrame(const CFrame &frame, const void* pUser)
{
    // 判断帧的有效性
    // Judge the validity of frame
    if (!frame.valid())
    {
        printf("frame is invalid!\n");
    }
    else
    {
        printf("pUser:%s  blockId = %lu.\n", (char*)pUser, frame.getBlockId());
    }

    return;
}

static void onDeviceLinkNotify(const SConnectArg& conArg, void* pUser)
{
    // 如果只针对断线时的操作，只要接收到offLine时处理即可
    // if it is only for the operation during disconnection, it can only be processed when receiving camera offline
    if (SConnectArg::offLine == conArg.m_event)
    {
        printf("*******  camera is offline  *******\n");

        if (NULL == streamPtr.get())
        {
            printf("streamPtr is null.\n");
            return;
        }

        // 注销回调
        // unregister callback function
        if (streamPtr->detachGrabbingEx(onGetFrame, (void*)pUser))
        {
            printf("*******  detach stream callback success  *******\n");
        }
        else
        {
            printf("*******  detach stream callback fail  *******\n");
        }

        // 停止拉流
        // stop grabbing
        if (streamPtr->stopGrabbing())
        {
            printf("*******  stop grab success  *******\n");
        }
        else
        {
            printf("*******  stop grab fail  *******\n");
        }
    }
    else if (SConnectArg::onLine == conArg.m_event)
    {
        printf("*******  camera is online  *******\n");

        TryToResumeConnect();
    }

    return;
}

// 相机流事件通知回调函数
// Camera stream event notification callback function
static void onStreamEvent(const SStreamArg& streamArg, void* pUser)
{
    if (streamEventStreamChannelError == streamArg.eStreamEventStatus)
    {
        printf("pUser:%s  EventStreamChannelError!\n", (char*)pUser);

        if (NULL == streamPtr.get())
        {
            printf("streamPtr is null.\n");
            return;
        }

        // 停止拉流
        // stop grabbing
        if (!streamPtr->stopGrabbing())
        {
            printf("StreamChannelError: stopGrabbing fail.\n");
            return;
        }

        // 判断相机是否断开
        // judge whether camera offline
        if (NULL == cameraSptr.get())
        {
            printf("cameraSptr is null.\n");
            return;
        }

        if (!cameraSptr->isConnected())
        {
            printf("camera is disconnected.\n");
            return;
        }

        // 重新开始拉流
        // restart grabbing
        if (!streamPtr->startGrabbing())
        {
            printf("StreamChannelError: startGrabbing fail.\n");
            return;
        }
    }
    else if (streamEventLostPacket == streamArg.eStreamEventStatus)
    {
        printf("pUser:%s  EventLostPacket!\n", (char*)pUser);
    }
    else if (streamEventNormal == streamArg.eStreamEventStatus)
    {
        // do nothing
    }

    return;
}

// 消息通知回调函数
// message notification callback function
static void onChannelCallback(const SMsgChannelArg& msgChannelArg, void* pUser)
{
    if (msgChannelArg.eventID == MSG_EVENT_ID_EXPOSURE_END)
    {
        printf("pUser:%s  Exposure End!\n", (char*)pUser);
    }
    else if (msgChannelArg.eventID == MSG_EVENT_ID_FRAME_TRIGGER)
    {
        printf("pUser:%s  Frame Trigger!\n", (char*)pUser);
    }
    else if (msgChannelArg.eventID == MSG_EVENT_ID_FRAME_START)
    {
        printf("pUser:%s  Frame Open!\n", (char*)pUser);
    }
    else
    {
        printf("eventID %d pUser:%s\n", msgChannelArg.eventID, (char*)pUser);
    }

    return;
}
// 显示设备信息
// dispaly camera information
static uint32_t selectDevice(uint32_t cameraCnt)
{
    uint32_t cameraIndex = 0;
    // 提示用户选择
    // imform user to select
    printf("Please intput the camera index:");
    while (1)
    {
        scanf("%d", &cameraIndex);
        setbuf(stdin, NULL);
        // 判断用户选择合法性
        // judge the validity of user selection
        cameraIndex -= 1;// 显示的Index是从1开始 eg:cameraIndex starts at 1
        if ((cameraIndex < 0) || (cameraIndex >= cameraCnt))
        {
            printf("Invaild!\n");
        }
        else
        {
            break;
        }
    }
    return cameraIndex;
}

static void displayDeviceInfo(TVector<ICameraPtr>& vCameraPtrList)
{
    ICameraPtr cameraSptr;
    // 打印Title行
    // print title
    printf("Idx Type Vendor     Model      S/N             DeviceUserID    IP Address    \n");
    printf("------------------------------------------------------------------------------\n");
    for (uint32_t cameraIndex = 0; cameraIndex < vCameraPtrList.size(); cameraIndex++)
    {
        cameraSptr = vCameraPtrList[cameraIndex];
        // Idx 设备列表的相机索引 最大表示字数：3
        // index of  camera camera list,display in 3 characters
        printf("%-3d", cameraIndex + 1);

        // Type 相机的设备类型（GigE，U3V，CL，PCIe）
        // Device type of camera
        switch (cameraSptr->getType())
        {
            case ICamera::typeGige:
                printf(" GigE");
                break;
            case ICamera::typeU3v:
                printf(" U3V ");
                break;
            case ICamera::typeCL:
                printf(" CL  ");
                break;
            case ICamera::typePCIe:
                printf(" PCIe");
                break;
            default:
                printf("     ");
                break;
        }

        // VendorName 制造商信息 最大表示字数：10
        // VendorName manufacturer information  display in 10 characters
        const char* vendorName = cameraSptr->getVendorName();
        char vendorNameCat[11];
        if (strlen(vendorName) > 10)
        {
            strncpy(vendorNameCat, vendorName, 7);
            vendorNameCat[7] = '\0';
            strcat(vendorNameCat, "...");
            printf(" %-10.10s", vendorNameCat);
        }
        else
        {
            printf(" %-10.10s", vendorName);
        }

        // ModeName 相机的型号信息 最大表示字数：10
        // ModeName camera model information  display in 10 characters
        printf(" %-10.10s", cameraSptr->getModelName());

        // Serial Number 相机的序列号 最大表示字数：15
        // camera uart Number  display in 15 characters
        printf(" %-15.15s", cameraSptr->getSerialNumber());

        // deviceUserID 自定义用户ID 最大表示字数：15
        // camera user id, display in 15 characters
        const char* deviceUserID = cameraSptr->getName();
        char deviceUserIDCat[16];
        if (strlen(deviceUserID) > 15)
        {
            strncpy(deviceUserIDCat, deviceUserID, 12);
            deviceUserIDCat[12] = '\0';
            strcat(deviceUserIDCat, "...");
            printf(" %-15.15s", deviceUserIDCat);
        }
        else
        {
            memcpy(deviceUserIDCat, deviceUserID, sizeof(deviceUserIDCat));
            printf(" %-15.15s", deviceUserIDCat);
        }

        // IPAddress GigE相机时获取IP地址
        // IP address of GigE camera
        IGigECameraPtr gigeCameraPtr = IGigECamera::getInstance(cameraSptr);
        if (NULL != gigeCameraPtr.get())
        {
            CString ip = gigeCameraPtr->getIpAddress();
            printf(" %s", ip.c_str());
        }
        printf("\n");

    }
}

int32_t main(void)
{
    CSystem &systemObj = CSystem::getInstance();
    TVector<ICameraPtr> vCameraPtrList;

    if (!systemObj.discovery(vCameraPtrList))
    {
        printf("discovery camera fail.\n");
        return 0;
    }

    if (0 == vCameraPtrList.size())
    {
        printf("no camera camera find.\n");
        return 0;
    }
    // 打印相机基本信息（序号,类型,制造商信息,型号,序列号,用户自定义ID,IP地址）
    // print camera info (index,Type,vendor name, model,uart number,DeviceUserID,IP Address)
    displayDeviceInfo(vCameraPtrList);
    uint32_t cameraIndex = selectDevice((uint32_t)vCameraPtrList.size());
    cameraSptr = vCameraPtrList[cameraIndex];

    // 连接设备
    // connect camera
    if (!cameraSptr->connect())
    {
        printf("connect cameral fail.\n");
        return 0;
    }

    // 创建事件对象
    // recreate event object
    eventPtr = systemObj.createEventSubscribe(cameraSptr);
    if (NULL == eventPtr.get())
    {
        printf("create event obj  fail.\n");
        return 0;
    }

    // 注册相机断线通知
    // register camera disconnection notice
    if (!eventPtr->subscribeConnectArgsEx(ConnectArgProcEx(&onDeviceLinkNotify), (void*)pUser))
    {
        printf("subscribe connect event fail.\n");
        return 0;
    }

    // 注册相机流事件通知
    // register camera stream evet notice
    if (!eventPtr->subscribeStreamArgEx(StreamArgProcEx(&onStreamEvent), (void*)pUser))
    {
        printf("subscribe stream event fail.\n");
        return 0;
    }

    // 注册消息通道事件回调函数
    // register message channel event callback function
    if (!eventPtr->subscribeMsgChannelEx(MsgChannelProcEx(&onChannelCallback), (void*)pUser))
    {
        printf("subscribe message channel fail.\n");
        return 0;
    }

    // 创建流对象
    // create stream object
    streamPtr = systemObj.createStreamSource(cameraSptr);
    if (NULL == streamPtr.get())
    {
        printf("create stream obj  fail.\n");
        return 0;
    }

    // 注册取流回调函数
    // Register get frame callback function
    if (!streamPtr->attachGrabbingEx(onGetFrame, (void*)pUser))
    {
        printf("attch Grabbing fail.\n");
        return 0;
    }

    // 开始拉流
    // start grabbing
    if (!streamPtr->startGrabbing())
    {
        printf("StartGrabbing  fail.\n");
        return 0;
    }

    // 主线程等待状态
    // main thread wait status
    getchar();

    // 反注册相机流事件通知
    // unregistration camera streamEvent notice
    if (!eventPtr->unsubscribeStreamArgEx(StreamArgProcEx(&onStreamEvent), (void*)pUser))
    {
        printf("unsubscribe stream event fail.\n");
        return 0;
    }

    // 反注册消息通道事件回调函数
    // unregister message channel event callback function
    if (!eventPtr->unsubscribeMsgChannelEx(MsgChannelProcEx(&onChannelCallback), (void*)pUser))
    {
        printf("unsubscribe message channel fail.\n");
        return 0;
    }

    // 反注册断线通知
    // unregistration disconnection notice
    if (!eventPtr->unsubscribeConnectArgsEx(ConnectArgProcEx(&onDeviceLinkNotify), (void*)pUser))
    {
        printf("unsubscribe connect event fail.\n");
        return 0;
    }

    // 反注册取流回调函数
    // unregister get frame callback function
    if (!streamPtr->detachGrabbingEx(onGetFrame, (void*)pUser))
    {
        printf("detch Grabbing fail.\n");
    }

    // 停止相机拉流
    // stop camera grabbing
    if (!streamPtr->stopGrabbing())
    {
        printf("stop Grabbing fail.\n");
        return 0;
    }

    // 断开设备
    // disconnect camera
    if (!cameraSptr->disConnect())
    {
        printf("disConnect cameral fail.\n");
        return 0;
    }

    isMainThreadExit = true;

    return 1;
}
*/
