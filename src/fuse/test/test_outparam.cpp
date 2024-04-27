//
// Created by zhangtianyi on 2023/12/20.
//
#include "huaray_camera.h"
#include "outparam.h"
#include <ros/ros.h>
#include <iterator>



int main(int argc, char *argv[]){
    ros::init(argc, argv, "set_params_node");
    ros::NodeHandle nh;

    CameraCalibration calibrator(0, 0);
    calibrator.initializeCameraParameters();
    calibrator.displayInstructions();

    // 创建相机
    HuarayCamera camera;
    // 读取华睿相机参数
    HuarayCameraParam param;
    std::string yamlFile = "/home/zhangtianyi/document/radar_ws/src/cubot_radar/config/param/huaray_camera_param.yaml";
    HuarayCameraParam::LoadFromYamlFile(yamlFile, &param);

    // 设置相机参数
    camera.SetParam(param);
    // 初始化华睿相机
    camera.Init();
    // camera.WriteHardwareParamToDevice(param.HardwareParams[0]);
    // 打开相机
    camera.Open();
    HuarayCameraData data;

    while (1) {
        camera.GetData(&data);
        calibrator.runCalibration(data.Image);

        if (cv::waitKey(10) == 27) {
            // ESC键退出
            std::cout << "标定结束" << std::endl;
            break;
            // 关闭摄像头或释放资源
            cv::destroyAllWindows();
        }
    }

    //关闭相机
    camera.Close();
    camera.Release();

    calibrator.performPNPCalibration();
    calibrator.sentCalibrationparam();

    return 0;
}