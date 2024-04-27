//
// Created by zhangtianyi on 2023/12/19.
//

#ifndef SRC_POSEESTIMATION_H
#define SRC_POSEESTIMATION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include <iostream>
#include "camera_radar_matrix_param.h"
#include "huaray_camera_param.h"

#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

/**
 * @brief 位姿估计
 */
class CameraCalibration {
public:
    static cv::Mat rotation;                                   ///< 旋转向量
    static cv::Mat translation;                                ///< 平移向量

private:
    int enemy;                                          ///<红方蓝方
    int camera_type;                                    ///<左右相机
    int confirm;                                        ///<表示是否确认了标定点的选择
    cv::Mat frame;                                      ///<存储当前帧
    std::vector<cv::Point3f> ops;                       ///<存储四个标定三维点的容器
    std::vector<cv::Point2f> imagePoints;               ///<存储四个标定二维点的容器
    cv::Rect zoomedRegion;                              ///<声明放大区域变量
    cv::Mat internalMatrix;                             ///< 相机内参数矩阵
    cv::Mat distCoeffs;                                 ///< 畸变系数
    static bool leftButtonPressed;                      ///< 标志位，用于跟踪鼠标左键是否被按下


public:

    /**
    * @brief 构造函数
    */
    CameraCalibration(int enemy, int camera_type);

    /**
    * @brief 提示标哪些点
    */
    void displayInstructions();

    /**
    * @brief 给定相机参数及三维坐标点
    */
    void initializeCameraParameters();

    /**
    * @brief 进行位姿估计
    */
    void runCalibration(cv::Mat& image);

    /**
    * @brief PNP求解
    */
    void performPNPCalibration();

    /**
    * @brief PNP求解的位姿发送到tf树
    */
    void sentCalibrationparam();

private:

    /**
    * @brief 鼠标回调函数，用来标点
    * @param[flags]  flags         未用到
    */
    static void mouseCallback(int event, int x, int y, int flags, void* userdata);

    /**
    * @brief 鼠标回调函数触发的根据光标悬浮点更新放大界面的区域
    * @param[in]  x,y    鼠标光标的xy坐标
    */

    void updateZoomedRegion(int x, int y);
    /**
    * @brief 鼠标回调函数触发的显示当前画面和放大画面
    */
    void drawCalibrationPoints(cv::Mat frame, int x, int y);

    /**
    * @brief 鼠标回调函数触发的记录当前光标坐标值
    * @param[in]  x,y    鼠标光标的xy坐标
    */
    void recordClick(int x, int y);
};



#endif //SRC_POSEESTIMATION_H
