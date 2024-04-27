#include <iostream>
#include <unistd.h>
#include <vector>
#include "huaray_camera_param.h"
#include "huaray_camera_runtime_param.h"
#include "huaray_camera_hardware_param.h"
#include "huaray_camera_model_param.h"

int main(int argc, char *argv[])
{
    HuarayCameraParam param;
    HuarayCameraParam::LoadFromYamlFile("/home/zhangtianyi/test_ws/src/cubot_radar/config/param/huaray_camera_param.yaml",  &param);

    return 0;
}
