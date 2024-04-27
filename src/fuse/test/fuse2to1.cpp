//
// Created by zhangtianyi on 2023/12/26.
//
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "detect_armors_info.h"
#include "detector_param.h"
#include "cubot_radar/CarsMsg.h"
#include "coordinate_transformer.h"

int receiveFrequency = 0;
ros::Publisher carsMsg2to1Publisher;

void detectCallback(const cubot_radar::CarsMsg::ConstPtr &carsMsg1, const cubot_radar::CarsMsg::ConstPtr &carsMsg2)
{
    cubot_radar::CarsMsg combinedCarsMsg;
    // 接收到话题消息，进入回调函数
    receiveFrequency ++;

    if (carsMsg1->carNum != 0 && carsMsg2->carNum != 0) {
        // 如果 carsMsg1 和 carsMsg2 都有效，将它们的检测结果融合
        combinedCarsMsg = *carsMsg1;
        // 遍历 carsMsg2，将不在 carsMsg1 中的车辆信息添加到 combinedCarsMsg
        for (int j = 0; j < carsMsg2->carNum; j++) {
            bool found = false;
            // 检查 carsMsg2 中的车辆是否在 carsMsg1 中已存在
            for (int i = 0; i < carsMsg1->carNum; i++) {
                if (carsMsg1->carsInfo[i].carClass == carsMsg2->carsInfo[j].carClass) {
                    // 如果找到相同类型的车辆，更新其坐标信息
                    combinedCarsMsg.carsInfo[i].carWorldX = (carsMsg1->carsInfo[i].carWorldX + carsMsg2->carsInfo[i].carWorldX) / 2;
                    combinedCarsMsg.carsInfo[i].carWorldY = (carsMsg1->carsInfo[i].carWorldY + carsMsg2->carsInfo[i].carWorldY) / 2;
                    combinedCarsMsg.carsInfo[i].carWorldZ = (carsMsg1->carsInfo[i].carWorldZ + carsMsg2->carsInfo[i].carWorldZ) / 2;
                    found = true;
                    break;
                }
            }
            // 如果 carsMsg2 中的车辆在 carsMsg1 中不存在，将其添加到 combinedCarsMsg 中
            if (!found) {
                combinedCarsMsg.carsInfo.push_back(carsMsg2->carsInfo[j]);
            }
        }
        // 更新车辆数量
        combinedCarsMsg.carNum = static_cast<int>(combinedCarsMsg.carsInfo.size());
    } else if (carsMsg1->carNum != 0 && carsMsg2->carNum == 0) {
        // 如果只有 carsMsg1 有效，则使用 carsMsg1 的检测结果
        combinedCarsMsg = *carsMsg1;
    } else if (carsMsg1->carNum == 0 && carsMsg2->carNum != 0) {
        // 如果只有 carsMsg2 有效，则使用 carsMsg2 的检测结果
        combinedCarsMsg = *carsMsg2;
    } else {
        // 如果都无效，则使用 carsMsg1 的检测结果，反正是无效的
        combinedCarsMsg = *carsMsg1;
    }

    //打印输出车辆类别与坐标
    for (int i = 0; i < combinedCarsMsg.carNum; ++i) {
        std::cout << "(" << combinedCarsMsg.carsInfo[i].carClass << ")" << std::endl;
        std::cout << "(" << combinedCarsMsg.carsInfo[i].carWorldX << "," << combinedCarsMsg.carsInfo[i].carWorldY << "," << combinedCarsMsg.carsInfo[i].carWorldZ << ")" << std::endl;
    }

    ROS_INFO("Finish all the process %i times", receiveFrequency);

    // 发布合并后的结果
    carsMsg2to1Publisher.publish(combinedCarsMsg);
}



int main(int argc, char** argv)
{

    // 创建显示窗口
    cv::namedWindow("subImage", cv::WINDOW_NORMAL);

    //Referee_Transmit_Car_Location(0x0301,data);

    // ros节点初始化
    ros::init(argc, argv, "detect_result_subscriber");

    // 创建ros句柄
    ros::NodeHandle nh;

    // 定义一个接受点云话题信息的消息过滤器
    message_filters::Subscriber<cubot_radar::CarsMsg> carsMsgSub(nh, "cars2_1", 2);

    // 定义一个接受自定义车辆话题信息的消息过滤器
    message_filters::Subscriber<cubot_radar::CarsMsg> carsMsgSub2(nh, "cars2_2", 2);

    // 定义消息同步器的机制为大致时间同步
    typedef message_filters::sync_policies::ApproximateTime<cubot_radar::CarsMsg, cubot_radar::CarsMsg> MySyncPolicy;

    // 定义时间同步器的消息队列大小和接收话题
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(80), carsMsgSub2, carsMsgSub);

    carsMsg2to1Publisher = nh.advertise<cubot_radar::CarsMsg>("cars2to1", 1);
    // 注册回调函数
    sync.registerCallback(boost::bind(&detectCallback, _1, _2));

    // ros消息回调处理函数
    ros::spin();

    return 0;
}
