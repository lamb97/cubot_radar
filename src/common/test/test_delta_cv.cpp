//
// Created by godzhu on 2021/12/28.
//

#include "delta_cv.h"

int main() {
    // 读取原始图像
    cv::Mat image = imread("/home/zhangtianyi/图片/5.png", cv::IMREAD_COLOR);
    if (image.empty()) {
        std::cout << "Error. Could not read picture." << std::endl;
        return -1;
    }
    cv::cvtColor(image, image, cv::COLOR_BGR2HSV);

    // 初始化参数
    BGRWeight weights;
    weights.Blue = 0.3;
    weights.Green = 0.3;
    weights.Red = 0.4;
    cv::Mat grayScaleImage;
    grayScaleImage = cv::Mat(image.rows, image.cols, CV_8UC1);

    HSVThreshold hsvThreshold;
    hsvThreshold.HueLower = 0;
    hsvThreshold.HueUpper = 10;
    hsvThreshold.SaturationLower = 43;
    hsvThreshold.SaturationUpper = 255;
    hsvThreshold.ValueLower = 46;
    hsvThreshold.ValueUpper = 255;

    // 计算运算时间
    auto begin = std::chrono::steady_clock::now();
    for (unsigned int i = 0; i < 1000; i++)
    {
//        DeltaCV::WeightedBinarize(image, weights, 120, &grayScaleImage);
//        DeltaCV::FastWeidhtedBinarize(image, weights, 120, &grayScaleImage);
//        DeltaCV::InRange(image, hsvThreshold, &grayScaleImage);
        DeltaCV::FastInRange(image, hsvThreshold, &grayScaleImage);
    }
    auto end = std::chrono::steady_clock::now();
    auto delay = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
    std::cout << "delay = " << delay.count() << " ms" << std::endl;
    cv::imshow("grayImage", grayScaleImage);
//    cv::imwrite("/home/godzhu/1.jpg", grayScaleImage);
    cv::waitKey(10000);
    return 0;
}