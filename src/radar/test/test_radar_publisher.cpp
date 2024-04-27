//
// Created by godzhu on 11/18/21.
//
#include <iostream>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <pcl_ros/point_cloud.h>
#include <cubot_radar/CustomMsg.h>
#include <cubot_radar/CustomPoint.h>

ros::Publisher pub;

void customPointCallback(const cubot_radar::CustomMsg::ConstPtr &msg) {
    std::cout << "1" << std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);//点云指针对象
    sensor_msgs::PointCloud2 output;
    cloud->width = msg->points.size();
    cloud->height = 1;
    cloud->resize(cloud->width);

    //创建ros句柄
    ros::NodeHandle nh;
    // 声明发布消息类型
    ros::Publisher pubRosPCL = nh.advertise<sensor_msgs::PointCloud2>("color_lidar", 10);
    ros::Rate loopRate(10);  // 发布频率为10赫兹

    for (size_t i = 0; i < cloud->points.size(); i++) {
        cloud->points[i].x = msg->points[i].x;
        cloud->points[i].y = msg->points[i].y;
        cloud->points[i].z = msg->points[i].z;
        cloud->points[i].intensity = msg->points[i].reflectivity;
    }

    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "livox_frame"; //坐标系
    pub.publish(output);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "livox_lidar_publisher");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;

    //创建一个Subscriber 订阅名为/speed_remote的topic
    ros::Subscriber speed_remote_sub = n.subscribe("livox/lidar", 10, customPointCallback);

    //循环等待回调函数
    ros::spin();
    return 0;
}