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
#include "camera_radar_matrix_param.h"
#include "coordinate_transformer.h"
#include "point_cloud_filter.h"

#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>
#include<opencv2/core/eigen.hpp>

#include <iterator>


int receiveFrequency = 0;
//u_char data[13];
ros::Publisher carsMsgPublisher;
pcl::visualization::CloudViewer viewer("Cloud Viewer");
cv::Mat rotation3_3 = cv::Mat(3, 3, CV_64F);
cv::Mat translation = cv::Mat(3, 1, CV_64F);
// 上一帧的z值，int对应不同的车辆
static std::vector<std::pair<int, double>> prevZValues = {
        {0, 0.0},
        {1, 0.0},
        {2, 0.0},
        {3, 0.0},
        {4, 0.0},
        {5, 0.0},
        {6, 0.0},
        {7, 0.0},
        {8, 0.0},
        {9, 0.0}
};

void detectCallback(const sensor_msgs::PointCloud2::ConstPtr &rosPCLMsg, const cubot_radar::CarsMsg::ConstPtr &carsMsg)
{
//    exit(1);
    cubot_radar::CarsMsg carsMsgs;
    // 接收到话题消息，进入回调函数
    receiveFrequency ++;
    ROS_INFO("Received the fuseMsg %i times", receiveFrequency);

    // 将sensor_msgs/Image格式转换为OpenCV格式
    boost::shared_ptr<cv_bridge::CvImage> getImage = cv_bridge::toCvCopy(carsMsg->image, "bgr8");
    cv::Mat subImage = getImage->image;

    CameraRadarMatrix matrixParam;
    std::string yamlFile = "/home/zhangtianyi/test_ws/src/cubot_radar/config/param/camera_radar_matrix_param.yaml";
    CameraRadarMatrixParam::LoadFromYamlFile(yamlFile, &matrixParam);

    cv::Mat cameraMatrix = matrixParam.CameraMatrix;
    cv::Mat internalMatrix = matrixParam.InternalMatrix;
    cv::Mat distCoeffs = matrixParam.DistortionVector;
    cv::Mat externalMatrix = matrixParam.ExternalMatrix;

    // 将rosPCL数据转换为PCL点云数据
    pcl::PCLPointCloud2 PCLCloud;
    pcl_conversions::toPCL(*rosPCLMsg, PCLCloud);

    // 将PCL点云数据转换为有点坐标和强度信息的PCLXYZI数据
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);  //点云指针对象
    pcl::fromPCLPointCloud2(PCLCloud, *cloud);

    // 使用相机的内参和畸变系数修正图片
    cv::Mat remap;
    cv::Mat map1;
    cv::Mat map2;
    remap = subImage.clone();
    cv::Size imageSize = subImage.size();
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                                cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, nullptr),
                                imageSize, CV_16SC2, map1, map2);
    // 畸变校正
    cv::remap(subImage, remap, map1, map2, cv::INTER_LINEAR);

    auto pointNum = (int)cloud->points.size();

    cv::Mat pointsData(4, pointNum, CV_64F);

    for (int i = 0; i < pointNum; ++i) {
        float x = cloud->points[i].x;
        float y = cloud->points[i].y;
        float z = cloud->points[i].z;

        pointsData.at<double>(0, i) = x;
        pointsData.at<double>(1, i) = y;
        pointsData.at<double>(2, i) = z;
        pointsData.at<double>(3, i) = 1;
    }

    // 将点云世界坐标转为像素坐标
    std::vector<cv::Point2i> pixelCoordinates;
    CoordinateTransformer::WorldToPixel(internalMatrix,
                                        externalMatrix,
                                        pointsData,
                                        &pixelCoordinates);

    // 给点云赋BGR值
    std::vector<cv::Point3i> BGRValue;
    CoordinateTransformer::ColoredPointCloud(pixelCoordinates, subImage, &BGRValue);

    for (int i = 0; i < pointNum; ++i) {
        // 忽视无效的点
        if (pointsData.at<double>(0, i) == 0 && pointsData.at<double>(1, i) == 0 && pointsData.at<double>(2, i) == 0) {
            continue;
        }
        // 忽视BGR值都为0的点
        if (BGRValue[i].x == 0 && BGRValue[i].y == 0 && BGRValue[i].z == 0) {
            continue;
        }
        // 给点云赋予XYZ的值
        cloud->points[i].x = (float)pointsData.at<double>(0, i);
        cloud->points[i].y = (float)pointsData.at<double>(1, i);
        cloud->points[i].z = (float)pointsData.at<double>(2, i);

        // 给点云赋予BGR值（给点云赋值的操作需要一个循环里进行，才能完成点的对应）
        cloud->points[i].r = BGRValue[i].z;
        cloud->points[i].g = BGRValue[i].y;
        cloud->points[i].b = BGRValue[i].x;
    }

    // 存放单帧图像车辆类别和世界坐标的容器
    std::vector<std::pair<int, cv::Point3d>> carsWorldInfo;

    carsMsgs.image = carsMsg->image;
    carsMsgs.carNum = carsMsg->carNum;
    carsMsgs.carsInfo.resize(carsMsg->carNum);


    ros::Rate loopRate(5);

    for(int i = 0; i < carsMsg->carNum; i++) {
        // 单个车辆的类别和世界坐标
        std::pair<int, cv::Point3d> carWorldInfo;

        // 车辆图像中心点坐标
        cv::Point2i carPixelCoordination((int)carsMsg->carsInfo[i].carX, (int)carsMsg->carsInfo[i].carY);

        // 单个车辆的类别
        carWorldInfo.first = carsMsg->carsInfo[i].carClass;

        std::cout << "(" <<  carWorldInfo.first << ")" <<std::endl;


        // 计算单个车辆的世界坐标
        carWorldInfo.second = CoordinateTransformer::CarWorldCoordination(carPixelCoordination,
                                                                           pointsData,
                                                                           pixelCoordinates,
                                                                           232);

        //carsMsgs.carsInfo[i].armorNum = carsMsg->carsInfo[i].armorNum;
        carsMsgs.carsInfo[i].carClass = carsMsg->carsInfo[i].carClass;
        carsMsgs.carsInfo[i].carWorldX = carWorldInfo.second.x;
        carsMsgs.carsInfo[i].carWorldY = carWorldInfo.second.y;
        carsMsgs.carsInfo[i].carWorldZ = carWorldInfo.second.z;

        std::cout << "(" << carsMsgs.carsInfo[i].carWorldX << "," << carsMsgs.carsInfo[i].carWorldY << "," << carsMsgs.carsInfo[i].carWorldZ << ")" << std::endl;

        carsWorldInfo.emplace_back(carWorldInfo);
    }

    ROS_INFO("Finish all the process %i times", receiveFrequency);

    // 显示接收到的点云数据
    viewer.showCloud(cloud);

    ROS_INFO("Car: %i", carsMsgs.carNum);
    carsMsgPublisher.publish(carsMsgs);

    loopRate.sleep();
}


int main(int argc, char** argv) {
    // ros节点初始化
    ros::init(argc, argv, "detect_result_subscriber");
    // 创建ros句柄
    ros::NodeHandle nh;
    // 创建显示窗口
    cv::namedWindow("subImage", cv::WINDOW_NORMAL);
    int index = 0;
while(index == 0){
    //创建一个转换监听器
    tf::TransformListener tf(ros::Duration(10));
    //创建一个变换关系存储量
    tf::StampedTransform transform;

    tf.waitForTransform("world", "camera", ros::Time(0), ros::Duration(5));

    tf.lookupTransform ("world", "camera",ros::Time(0), transform);
    // 提取平移向量
    tf::Vector3 translation1 = transform.getOrigin();
    // 提取旋转四元数
    tf::Quaternion rotation1 = transform.getRotation();

    Eigen::Translation3f translation_matrix(translation1.x(), translation1.y(), translation1.z());
    Eigen::Quaternionf rotation_matrix(rotation1.w(), rotation1.x(), rotation1.y(), rotation1.z());
    Eigen::Matrix4f transform_matrix = (translation_matrix * rotation_matrix).matrix();

    translation.at<double>(0,0) = transform_matrix(0,3);
    translation.at<double>(1,0) = transform_matrix(1,3);
    translation.at<double>(2,0) = transform_matrix(2,3);
    Eigen::Matrix3f eigenBlock = transform_matrix.block<3, 3>(0, 0);//Eigen::Matrix4f的block方法，从0,0检索三行三列
    cv::eigen2cv(eigenBlock, rotation3_3);
    // 输出平移向量、平移向量
    std::cout << "Translation vector:\n" << translation << std::endl;
    std::cout << "Rotation matrix:\n" << rotation3_3 << std::endl;
    index++;
}


    // 定义一个接受点云话题信息的消息过滤器
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointCloudSub(nh, "livox/lidar", 10);

    // 定义一个接受自定义车辆话题信息的消息过滤器
    message_filters::Subscriber<cubot_radar::CarsMsg> carsMsgSub(nh, "carsInfo", 10);

    // 定义一个接受自定义车辆话题信息的消息过滤器
//    message_filters::Subscriber<cubot_radar::CarsMsg> cars2MsgSub(nh, "carsInfo_two", 2);
    // 定义消息同步器的机制为大致时间同步
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, cubot_radar::CarsMsg> MySyncPolicy;

    // 定义时间同步器的消息队列大小和接收话题
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(80), pointCloudSub, carsMsgSub);
    // 定义时间同步器的消息队列大小和接收话题
//    message_filters::Synchronizer<MySyncPolicy> sync2(MySyncPolicy(80), pointCloudSub, cars2MsgSub);

    carsMsgPublisher = nh.advertise<cubot_radar::CarsMsg>("cars", 1);
    // 注册回调函数
    sync.registerCallback(boost::bind(&detectCallback, _1, _2));

    // ros消息回调处理函数
    ros::spin();

    return 0;
}
