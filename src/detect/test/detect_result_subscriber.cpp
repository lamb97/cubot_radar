//
// Created by godzhu on 2021/12/24.
//

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/Image.h>
#include "detect_armors_info.h"
#include "detector_param.h"
#include "cubot_radar/CarsMsg.h"

int receiveFrequency = 0;

void detectCallback(const cubot_radar::CarsMsg::ConstPtr &carsMsg) {
    // 接收到话题消息，进入回调函数
    receiveFrequency++;
    ROS_INFO("Received the DetectMsg %i times", receiveFrequency);

    // 将sensor_msgs/Image格式转换为OpenCV格式
    boost::shared_ptr<cv_bridge::CvImage> getImage = cv_bridge::toCvCopy(carsMsg->image, "bgr8");
    cv::Mat subImage = getImage->image;

    for (int i = 0; i < carsMsg->carNum; i++) {
        cv::Rect carBox((int)carsMsg->carsInfo[i].carX,
                        (int)carsMsg->carsInfo[i].carY,
                        (int)carsMsg->carsInfo[i].carWidth,
                        (int)carsMsg->carsInfo[i].carHeight);
        // 添加车辆类别显示信息
        cv::putText(subImage, std::to_string(carsMsg->carsInfo[i].carClass),
                    cv::Point(carBox.tl().x, carBox.tl().y - 5),
                    cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 255, 255), 1);

        for (int j = 0; j < carsMsg->carsInfo[i].armorNum; j++) {
            cv::Rect armorBox((int)carsMsg->carsInfo[i].armorsInfo[j].armorX,
                              (int)carsMsg->carsInfo[i].armorsInfo[j].armorY,
                              (int)carsMsg->carsInfo[i].armorsInfo[j].armorWidth,
                              (int)carsMsg->carsInfo[i].armorsInfo[j].armorHeight);

            // 标签为0~5对应红色装甲板
            if (carsMsg->carsInfo[i].armorsInfo[j].armorClass >= 0 &&
                carsMsg->carsInfo[i].armorsInfo[j].armorClass < 5) {
                // 画框显示装甲板区域
                cv::rectangle(subImage, armorBox, cv::Scalar(0, 0, 255), 2);

                // 画框显示车辆区域
                cv::rectangle(subImage, carBox, cv::Scalar(0, 0, 255), 2);
            }

            // 标签为5~9对应蓝色装甲板
            if (carsMsg->carsInfo[i].armorsInfo[j].armorClass >= 5 &&
                carsMsg->carsInfo[i].armorsInfo[j].armorClass < 10) {
                // 画框显示装甲板区域
                cv::rectangle(subImage, armorBox, cv::Scalar(255, 0, 0), 2);

                // 画框显示车辆区域
                cv::rectangle(subImage, carBox, cv::Scalar(255, 0, 0), 2);
            }
        }

    }

    ROS_INFO("Finish all the process %i times", receiveFrequency);

    cv::imshow("subImage", subImage);
    cv::waitKey(10);
}

int main(int argc, char **argv) {
    // 创建显示窗口
    cv::namedWindow("subImage", cv::WINDOW_NORMAL);
    cv::waitKey(10);
    // ros节点初始化
    ros::init(argc, argv, "detect_result_subscriber");

    // 创建ros句柄
    ros::NodeHandle nh;

    ros::Subscriber livoxSub2 = nh.subscribe("carsInfo_two", 10, detectCallback);

    // ros消息回调处理函数
    ros::spin();
    return 0;
}
