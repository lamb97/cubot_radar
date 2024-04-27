//
// Created by plutoli on 2021/8/14.
//

#include <string>
#include <cstring>
#include "huaray_camera.h"

int main(int argc, char *argv[])
{
    // 读取华睿相机参数
    HuarayCameraParam param;
    std::string yamlFile = "/home/zhangtianyi/test_ws/src/cubot_radar/config/param/huaray_camera_param.yaml";
    HuarayCameraParam::LoadFromYamlFile(yamlFile, &param);

    // 初始化华睿相机
    HuarayCamera camera;
    camera.SetParam(param);
    camera.Init();
//    camera.WriteHardwareParamToDevice(param.HardwareParams[0]);
    camera.Open();

    // 创建显示窗体
    cv::namedWindow(camera.GetParam().Key, cv::WINDOW_NORMAL);

    // 播放相机视频
    while (camera.IsOpened())
    {
        //std::cout<<"?(.,.)?"<<std::endl;
        HuarayCameraData data;
        camera.GetData(&data);
        cv::imshow(camera.GetParam().Key, data.Image);
        cv::waitKey(10);
    }

    // 关闭相机
    camera.Close();
    // 清理相机资源
    camera.Release();

    return 0;
}
