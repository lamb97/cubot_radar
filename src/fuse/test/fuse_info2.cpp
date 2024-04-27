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
#include "camera_radar_matrix_param.h"
#include "coordinate_transformer.h"



int receiveFrequency = 0;
//u_char data[13];
ros::Publisher carsMsg2_1Publisher;
ros::Publisher carsMsg2_2Publisher;
pcl::visualization::CloudViewer viewer("Cloud Viewer");
cv::Mat rotation2_1;
cv::Mat translation2_1;
cv::Mat rotation2_2;
cv::Mat translation2_2;
// 上一帧的z值，int对应不同的车辆
static std::vector<std::pair<int, double>> prevZValues1 = {
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
static std::vector<std::pair<int, double>> prevZValues2 = {
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
    cubot_radar::CarsMsg carsMsgs1;
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

    for (int i = 0; i < pointNum; ++i){

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

    carsMsgs1.image = carsMsg->image;
    carsMsgs1.carNum = carsMsg->carNum;
    carsMsgs1.carsInfo.resize(carsMsg->carNum);


    ros::Rate loopRate(5);
    for(int i = 0; i < carsMsg->carNum; i++){
        // 单个车辆的类别和世界坐标
        std::pair<int, cv::Point3d> carWorldInfo;

        // 车辆图像中心点坐标
        cv::Point2i carPixelCoordination((int)carsMsg->carsInfo[i].carX, (int)carsMsg->carsInfo[i].carY);

        // 单个车辆的类别
        carWorldInfo.first = carsMsg->carsInfo[i].carClass;

        //std::cout << "(" <<  carWorldInfo.first << ")" <<std::endl;

        // 计算单个车辆的世界坐标
        carWorldInfo.second = CoordinateTransformer::Car2WorldCoordination(carPixelCoordination,
                                                                          pointsData,
                                                                          pixelCoordinates,
                                                                          externalMatrix,
                                                                          cameraMatrix,
                                                                          rotation2_1,
                                                                          translation2_1,
                                                                          5,
                                                                          0.1);

        //z值突变，若前后两帧z值超过一定阈值，更新后一帧的z为前一帧的z，z值用世界坐标系的
        for (auto& entry : prevZValues1) {
            if (entry.first == carWorldInfo.first) {
                // 检查是否是第一帧，如果是，直接更新prevZValues
                if (receiveFrequency == 1) {
                    entry.second = carWorldInfo.second.z;
                } else{
                    double zThreshold = 1.0;    // 阈值，用于判断是否进行z值更新
                    double zDifference = std::abs(carWorldInfo.second.z - entry.second);
                    if (zThreshold < zDifference) {
                        carWorldInfo.second.z = entry.second;
                        //z值更新带来的世界坐标系转化
                        //舍弃相机坐标系的奇次坐标
                        cv::Mat carCameraPoint =  (cv::Mat_<double>(3, 1) <<
                                                                          CoordinateTransformer::average_cameraPoint.at<double>(0),
                                CoordinateTransformer::average_cameraPoint.at<double>(1),
                                CoordinateTransformer::average_cameraPoint.at<double>(2));
                        double ratio = (entry.second - carCameraPoint.at<double>(2)) /
                                       (carWorldInfo.second.z - carCameraPoint.at<double>(2));
                        //世界坐标系从cv::point3d变到cv::Mat
                        cv::Mat carWorldMat = (cv::Mat_<double>(3, 1) << carWorldInfo.second.x, carWorldInfo.second.y, carWorldInfo.second.z);
                        //上交的修正公式
                        cv::Mat carWorldMat_modify = ratio * (carWorldMat-carCameraPoint) + carCameraPoint;
                        cv::Point3d carWorld3d_modify{carWorldMat_modify.at<double>(0), carWorldMat_modify.at<double>(1), carWorldMat_modify.at<double>(2)};
                        carWorldInfo.second = carWorld3d_modify;
                    }
                    // 更新prevZValues为当前帧的z值
                    entry.second = carWorldInfo.second.z;
                }
                break;
            }
        }

        //carsMsgs1.carsInfo[i].armorNum = carsMsg->carsInfo[i].armorNum;
        carsMsgs1.carsInfo[i].carClass = carsMsg->carsInfo[i].carClass;
        carsMsgs1.carsInfo[i].carWorldX = carWorldInfo.second.x;
        carsMsgs1.carsInfo[i].carWorldY = carWorldInfo.second.y;
        carsMsgs1.carsInfo[i].carWorldZ = carWorldInfo.second.z;

        //std::cout << "(" << carsMsgs1.carsInfo[i].carWorldX << "," << carsMsgs1.carsInfo[i].carWorldY << "," << carsMsgs1.carsInfo[i].carWorldZ << ")" << std::endl;

        carsWorldInfo.emplace_back(carWorldInfo);
    }

    ROS_INFO("Finish all the process %i times", receiveFrequency);

    // 显示接收到的点云数据
    viewer.showCloud(cloud);


    ROS_INFO("Car: %i", carsMsgs1.carNum);
    carsMsg2_1Publisher.publish(carsMsgs1);

    loopRate.sleep();

}

void detectCallback2(const sensor_msgs::PointCloud2::ConstPtr &rosPCLMsg, const cubot_radar::CarsMsg::ConstPtr &carsMsg)
{
    cubot_radar::CarsMsg carsMsgs2;
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

    for (int i = 0; i < pointNum; ++i){

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

    carsMsgs2.image = carsMsg->image;
    carsMsgs2.carNum = carsMsg->carNum;
    carsMsgs2.carsInfo.resize(carsMsg->carNum);


    ros::Rate loopRate(5);
    for(int i = 0; i < carsMsg->carNum; i++){
        // 单个车辆的类别和世界坐标
        std::pair<int, cv::Point3d> carWorldInfo;

        // 车辆图像中心点坐标
        cv::Point2i carPixelCoordination((int)carsMsg->carsInfo[i].carX, (int)carsMsg->carsInfo[i].carY);

        // 单个车辆的类别
        carWorldInfo.first = carsMsg->carsInfo[i].carClass;

        //std::cout << "(" <<  carWorldInfo.first << ")" <<std::endl;

        // 计算单个车辆的世界坐标
        carWorldInfo.second = CoordinateTransformer::Car2WorldCoordination(carPixelCoordination,
                                                                          pointsData,
                                                                          pixelCoordinates,
                                                                          externalMatrix,
                                                                          cameraMatrix,
                                                                          rotation2_2,
                                                                          translation2_2,
                                                                          5,
                                                                          0.1);


        //z值突变，若前后两帧z值超过一定阈值，更新后一帧的z为前一帧的z，z值用世界坐标系的
        for (auto& entry : prevZValues2) {
            if (entry.first == carWorldInfo.first) {
                // 检查是否是第一帧，如果是，直接更新prevZValues
                if (receiveFrequency == 1) {
                    entry.second = carWorldInfo.second.z;
                } else{
                    double zThreshold = 1.0;    // 阈值，用于判断是否进行z值更新
                    double zDifference = std::abs(carWorldInfo.second.z - entry.second);
                    if (zThreshold < zDifference) {
                        carWorldInfo.second.z = entry.second;
                        //z值更新带来的世界坐标系转化
                        //舍弃相机坐标系的奇次坐标
                        cv::Mat carCameraPoint =  (cv::Mat_<double>(3, 1) <<
                                                                          CoordinateTransformer::average_cameraPoint.at<double>(0),
                                CoordinateTransformer::average_cameraPoint.at<double>(1),
                                CoordinateTransformer::average_cameraPoint.at<double>(2));
                        double ratio = (entry.second - carCameraPoint.at<double>(2)) /
                                       (carWorldInfo.second.z - carCameraPoint.at<double>(2));
                        //世界坐标系从cv::point3d变到cv::Mat
                        cv::Mat carWorldMat = (cv::Mat_<double>(3, 1) << carWorldInfo.second.x, carWorldInfo.second.y, carWorldInfo.second.z);
                        //上交的修正公式
                        cv::Mat carWorldMat_modify = ratio * (carWorldMat-carCameraPoint) + carCameraPoint;
                        cv::Point3d carWorld3d_modify{carWorldMat_modify.at<double>(0), carWorldMat_modify.at<double>(1), carWorldMat_modify.at<double>(2)};
                        carWorldInfo.second = carWorld3d_modify;
                    }
                    // 更新prevZValues为当前帧的z值
                    entry.second = carWorldInfo.second.z;
                }
                break;
            }
        }


        //carsMsgs2.carsInfo[i].armorNum = carsMsg->carsInfo[i].armorNum;
        carsMsgs2.carsInfo[i].carClass = carsMsg->carsInfo[i].carClass;
        carsMsgs2.carsInfo[i].carWorldX = carWorldInfo.second.x;
        carsMsgs2.carsInfo[i].carWorldY = carWorldInfo.second.y;
        carsMsgs2.carsInfo[i].carWorldZ = carWorldInfo.second.z;

        //std::cout << "(" << carsMsgs2.carsInfo[i].carWorldX << "," << carsMsgs2.carsInfo[i].carWorldY << "," << carsMsgs2.carsInfo[i].carWorldZ << ")" << std::endl;

        carsWorldInfo.emplace_back(carWorldInfo);
    }

    ROS_INFO("Finish all the process %i times", receiveFrequency);

    // 显示接收到的点云数据
    viewer.showCloud(cloud);

    ROS_INFO("Car: %i", carsMsgs2.carNum);
    carsMsg2_2Publisher.publish(carsMsgs2);

    loopRate.sleep();

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

    // 从ROS参数服务器获取字符串形式的向量
    std::string rotationStr1, translationStr1;
    if (!nh.getParam("/rotation1", rotationStr1)) {
        ROS_ERROR("Failed to retrieve /rotation parameter.");
        return 1;
    }
    if (!nh.getParam("/translation1", translationStr1)) {
        ROS_ERROR("Failed to retrieve /translation parameter.");
        return 1;
    }
    // 将获取到的字符串分割为字符串数组
    std::istringstream rotationStream1(rotationStr1);
    std::vector<std::string> rotationTokens1(std::istream_iterator<std::string>{rotationStream1}, std::istream_iterator<std::string>());
    std::istringstream translationStream1(translationStr1);
    std::vector<std::string> translationTokens1(std::istream_iterator<std::string>{translationStream1}, std::istream_iterator<std::string>());
    // 将字符串数组转换为 std::vector<float>
    std::vector<float> rotationVec1, translationVec1;
    for (const auto& token : rotationTokens1) {
        rotationVec1.push_back(std::stof(token));
    }
    for (const auto& token : translationTokens1) {
        translationVec1.push_back(std::stof(token));
    }
    // 将获取到的 std::vector<float> 数据转换为 cv::Mat
    cv::Mat rotationMat1(rotationVec1, true);
    cv::Mat translationMat1(translationVec1, true);
    rotation2_1 = rotationMat1;
    translation2_1 = translationMat1;

    std::string rotationStr2, translationStr2;
    if (!nh.getParam("/rotation2", rotationStr2)) {
        ROS_ERROR("Failed to retrieve /rotation parameter.");
        return 1;
    }
    if (!nh.getParam("/·translation2", translationStr2)) {
        ROS_ERROR("Failed to retrieve /translation parameter.");
        return 1;
    }
    // 将获取到的字符串分割为字符串数组
    std::istringstream rotationStream2(rotationStr2);
    std::vector<std::string> rotationTokens2(std::istream_iterator<std::string>{rotationStream2}, std::istream_iterator<std::string>());
    std::istringstream translationStream2(translationStr2);
    std::vector<std::string> translationTokens2(std::istream_iterator<std::string>{translationStream2}, std::istream_iterator<std::string>());
    // 将字符串数组转换为 std::vector<float>
    std::vector<float> rotationVec2, translationVec2;
    for (const auto& token : rotationTokens2) {
        rotationVec2.push_back(std::stof(token));
    }
    for (const auto& token : translationTokens2) {
        translationVec2.push_back(std::stof(token));
    }
    // 将获取到的 std::vector<float> 数据转换为 cv::Mat
    cv::Mat rotationMat2(rotationVec2, true);
    cv::Mat translationMat2(translationVec2, true);
    rotation2_2 = rotationMat2;
    translation2_2 = translationMat2;

    // 定义一个接受点云话题信息的消息过滤器
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointCloudSub(nh, "livox/lidar", 2);

    // 定义一个接受自定义车辆话题信息的消息过滤器
    message_filters::Subscriber<cubot_radar::CarsMsg> carsMsgSub(nh, "carsInfo", 2);

    // 定义一个接受自定义车辆话题信息的消息过滤器
    message_filters::Subscriber<cubot_radar::CarsMsg> cars2MsgSub(nh, "carsInfo_two", 2);
    // 定义消息同步器的机制为大致时间同步
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, cubot_radar::CarsMsg> MySyncPolicy;

    // 定义时间同步器的消息队列大小和接收话题
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(80), pointCloudSub, carsMsgSub);
    // 定义时间同步器的消息队列大小和接收话题
    message_filters::Synchronizer<MySyncPolicy> sync2(MySyncPolicy(80), pointCloudSub, cars2MsgSub);

    carsMsg2_1Publisher = nh.advertise<cubot_radar::CarsMsg>("cars2_1", 1);
    carsMsg2_2Publisher = nh.advertise<cubot_radar::CarsMsg>("cars2_2", 1);

    // 注册回调函数
    sync.registerCallback (boost::bind(&detectCallback, _1, _2));
    sync2.registerCallback(boost::bind(&detectCallback2, _1, _2));

    // ros消息回调处理函数
    ros::spin();

    return 0;
}