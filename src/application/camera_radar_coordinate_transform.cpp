//
// Created by godzhu on 2021/11/26.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include "camera_radar_matrix_param.h"

// 图像坐标系下的2D点集合
std::vector<cv::Point2f> points2d();

// 雷达坐标系下的3D世界坐标
std::vector<cv::Point3f> points3d();

int main() {
    // Read points
    std::vector<cv::Point2f> pixel_Points = points2d();
    std::vector<cv::Point3f> worlpoints3d = points3d();

    CameraRadarMatrix matrixParam;
    std::string yamlFile = "/home/zhangtianyi/test_ws/src/cubot_radar/config/param/camera_radar_matrix_param.yaml";
    CameraRadarMatrixParam::LoadFromYamlFile(yamlFile, &matrixParam);

    cv::Mat cameraMatrix = matrixParam.CameraMatrix;
    cv::Mat distCoeffs = matrixParam.DistortionVector;

    cv::Mat rvec(3, 1, cv::DataType<double>::type);
    cv::Mat tvec(3, 1, cv::DataType<double>::type);

    // 计算相机在雷达坐标系下的旋转平移矩阵
    cv::solvePnP(worlpoints3d,
                 pixel_Points,
                 cameraMatrix,
                 distCoeffs,
                 rvec,
                 tvec,
                 false,
                 cv::SOLVEPNP_ITERATIVE);

    std::cout << "rvec: " << rvec << std::endl;
    std::cout << "tvec: " << tvec << std::endl;

    cv::Mat rotationMatrix(3, 3, cv::DataType<double>::type);

    cv::Rodrigues(rvec, rotationMatrix);

    std::cout << "rotationMatrix: " << rotationMatrix << std::endl;
    
    // 根据所给的3D坐标和已知的几何变换来求解投影后的2D坐标
    std::vector<cv::Point2f> projectedPoints;
    cv::projectPoints(worlpoints3d, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);

    // 显示反投影求解2D坐标
    for (unsigned int i = 0; i < projectedPoints.size(); ++i) {
        std::cout << "Image point: " << pixel_Points[i] << " Projected to " << projectedPoints[i] << std::endl;
    }

    return 0;
}

// 图像坐标系下的2D点集合
std::vector<cv::Point2f> points2d() {
    std::vector<cv::Point2f> lightPixelPoints;

    // light3的九个角点
    lightPixelPoints.emplace_back(87, 214);
    lightPixelPoints.emplace_back(76, 679);
    lightPixelPoints.emplace_back(473, 327);
    lightPixelPoints.emplace_back(650, 331);
    lightPixelPoints.emplace_back(654, 516);
    lightPixelPoints.emplace_back(468, 517);
    lightPixelPoints.emplace_back(783, 185);
    lightPixelPoints.emplace_back(1020, 188);
    lightPixelPoints.emplace_back(1018, 589);
    lightPixelPoints.emplace_back(774, 582);
    lightPixelPoints.emplace_back(396, 46);
    lightPixelPoints.emplace_back(544, 64);
    lightPixelPoints.emplace_back(687, 79);
    lightPixelPoints.emplace_back(650, 126);
    lightPixelPoints.emplace_back(522, 116);
    lightPixelPoints.emplace_back(386, 98);

    return lightPixelPoints;
}


// 雷达坐标系下的3D世界坐标
std::vector<cv::Point3f> points3d() {
    std::vector<cv::Point3f> lightWorldPoints;

    // 以雷达坐标系为世界坐标系
    lightWorldPoints.emplace_back(7.378, 3.327, 1.482);
    lightWorldPoints.emplace_back(7.206, 3.296, -1.191);
    lightWorldPoints.emplace_back(5.042, 0.752, 0.577);
    lightWorldPoints.emplace_back(5.214, 0.049, 0.606);
    lightWorldPoints.emplace_back(5.166, 0.018, -0.164);
    lightWorldPoints.emplace_back(5.004, 0.756, -0.172);
    lightWorldPoints.emplace_back(4.918, -0.42, 1.105);
    lightWorldPoints.emplace_back(4.843, -1.303, 1.114);
    lightWorldPoints.emplace_back(4.675, -1.273, -0.384);
    lightWorldPoints.emplace_back(4.731, -0.41, -0.391);
    lightWorldPoints.emplace_back(5.096, 1.075, 1.685);
    lightWorldPoints.emplace_back(5.268, 0.507, 1.677);
    lightWorldPoints.emplace_back(5.431, -0.065, 1.67);
    lightWorldPoints.emplace_back(5.988, 0.074, 1.637);
    lightWorldPoints.emplace_back(5.815, 0.651, 1.64);
    lightWorldPoints.emplace_back(5.672, 1.227, 1.64);

    return lightWorldPoints;
}
