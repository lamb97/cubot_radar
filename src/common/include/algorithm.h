#ifndef GENERAL_H
#define GENERAL_H

#include <opencv2/opencv.hpp>

float f_min(float x, float y);

float f_max(float x, float y);

void makeRectSafe(cv::Rect &rect, cv::Mat &src);

cv::Rect rectCenterScale(cv::Rect rect, cv::Size size);

cv::Rect reMapRect(cv::Rect &rect, int blocksizeW, int blocksizeH);

void hsv_to_bgr(int h, int s, int v, int &b, int &g, int &r);

#endif