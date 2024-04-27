//
// Created by godzhu on 11/19/21.
//
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <std_msgs/Header.h>
#include "huaray_camera.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;			

    image_transport::ImageTransport imageTransport(nh);

    image_transport::Publisher imagePublisher = imageTransport.advertise("camera/image", 1);

    std_msgs::Header header;
//    header.frame_id = "livox_frame";

    ros::Rate loopRate(5);

    cv::Mat imagePub;

    // 读取华睿相机参数
    HuarayCameraParam param;
    std::string yamlFile = "/home/zhangtianyi/test_ws/src/cubot_radar/config/param/huaray_camera_param.yaml";
    HuarayCameraParam::LoadFromYamlFile(yamlFile, &param);

    // 初始化华睿相机
    HuarayCamera camera;
    camera.SetParam(param);
    camera.Init();
    camera.WriteHardwareParamToDevice(param.HardwareParams[0]);
    camera.Open();

    // 创建显示窗体
    cv::namedWindow(camera.GetParam().Key, cv::WINDOW_NORMAL);

    while (camera.IsOpened() && nh.ok()){
        HuarayCameraData data;
        camera.GetData(&data);
        imagePub = data.Image;
        header.stamp = ros::Time::now();
        sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(header, "bgr8", imagePub).toImageMsg();
        imagePublisher.publish(imageMsg);
        loopRate.sleep();
    }

//    // 打开视频
//    cv::VideoCapture capture("/home/godzhu/Documents/cubot_brain6/data/video/2021-09-05/blue2.avi");
//    if (!capture.isOpened()) {
//        spdlog::error("Video cannot be opened");
//        return -1;
//    }
//
//    // 播放相机视频
//    while (capture.read(imagePub) && nh.ok()) {
//
//        sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(header, "bgr8", imagePub).toImageMsg();
//        cv::imshow("imagePub", imagePub);
//        imagePublisher.publish(imageMsg);
//        loopRate.sleep();
//    }

    // 关闭相机
    camera.Close();

    // 清理相机资源
    camera.Release();

    return 0;
}
