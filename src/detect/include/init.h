//
// Created by zhangtianyi on 2023/3/24.
//

#ifndef ROS_WS_INIT_H
#define ROS_WS_INIT_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Eigen>
#include <opencv2/calib3d.hpp>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui_c.h>
#include "detect_yolo.h"

namespace guard_init
{
    extern cv::Mat cv_camera_matrix;
    extern cv::Mat cv_dist_coeffs;
    extern cv::Mat cv_Rtag2camera;
    extern cv::Mat cv_Ttag2camera;

    extern float yolov5_conf_thres;
    extern float yolov5_iou_thres;
    extern std::string yolov5_detect_path;
    extern std::string yolov5_detect_2_path;   // 第二次检测
    extern std::string position_mlp_path_blue;
    extern std::string position_mlp_path_red;
    extern std::string yolov5_name_path;
    extern std::string yolov5_name_2_path;  // 第二个类别文件
    extern std::string backgroud_path;
    extern std::string video_source_path;
    extern torch::DeviceType device_type;

    extern cv::Point2f enemy_red_pixel_pts_right[4];             // 像素坐标
    extern cv::Point2f enemy_red_pixel_pts_left[4];              // 像素坐标
    extern cv::Point2f enemy_blue_pixel_pts_right[4];            // 像素坐标
    extern cv::Point2f enemy_blue_pixel_pts_left[4];             // 像素坐标
    extern int enemy_color;
    extern float limit_fuse;

    bool isInside(Rect rect1, Rect rect2);
    void loadModelParam(bool is_gpu);
    void yolov5Init(YOLOv5Detector detector, std::vector<std::string> &class_names);
    void yolov5Init_2(YOLOv5Detector_2 detector_2, std::vector<std::string> &class_names_2);
}




#endif //ROS_WS_INIT_H