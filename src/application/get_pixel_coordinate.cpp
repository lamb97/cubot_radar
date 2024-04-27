//
// Created by godzhu on 2021/11/26.
//

#include <iostream>
#include <opencv2/opencv.hpp>

cv::Mat src;
bool down = false;
cv::Point prept = cv::Point(-1, -1);
cv::Point curpt = cv::Point(-1, -1);
void on_mouse(int event, int x, int y, int flags, void* ustc)
{
    if (event == cv::EVENT_LBUTTONDOWN)    //右键按下
    {
        prept = cv::Point(x, y);
        std::cout << x << " " << y << std::endl;
        down = true;
    }
    else if (event == cv::EVENT_LBUTTONUP)     //右键放开
        down = false;

    if (down == true && event == cv::EVENT_MOUSEMOVE)    //右键按下且鼠标移动
    {
        curpt = cv::Point(x, y);
        line(src, prept, curpt, cv::Scalar(255, 0, 0), 5);
        cv::waitKey(5);        //可以解决画图时卡顿的问题
        imshow("src", src);
        prept = curpt;
    }
}

int main()
{
    src = cv::imread("/home/zhangtianyi/test_ws/src/cubot_radar/data/data1/picture/6.bmp", 1);

    cv::namedWindow("src",cv::WINDOW_NORMAL);
    cv::setMouseCallback("src", on_mouse, 0);//关键内置函数

    imshow("src", src);
    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}



