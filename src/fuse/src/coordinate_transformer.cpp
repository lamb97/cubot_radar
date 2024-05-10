//
// Created by godzhu on 2021/12/7.
//

#include "coordinate_transformer.h"

// 将雷达扫描得到的点由世界坐标系转换为像素坐标系
void CoordinateTransformer::WorldToPixel(const cv::Mat &internalMatrix,
                                         const cv::Mat &externalMatrix,
                                         const cv::Mat &pointsDatas,
                                         std::vector<cv::Point2i> *pixelCoordinates)
{
    // 将雷达扫描得到的点由世界坐标系转换为像素坐标系
    cv::Mat result = internalMatrix * externalMatrix * pointsDatas;
    cv::Point2i singlePoint;
    for(int i = 0; i < result.cols; i++) {
        auto n_x = (float)result.at<double>(0, i) / (float)result.at<double>(2, i);
        auto n_y = (float)result.at<double>(1, i) / (float)result.at<double>(2, i);
        auto n_z = 1;
        cv::Mat normalizePoint = 
        cv::Mat internalMatrix1;
        internalMatrix.convertTo(internalmatrix1,CV_32F , 1,0)
        cv::Mat resultEnd = internalMatrix * normalizePoint
        singlePoint.x = resultEnd.at<double>(0,0);
        singlePoint.y = resultEnd.at<double>(1,0);
        pixelCoordinates->push_back(singlePoint);
    }
}

// 对世界坐标点转换得到的像素坐标点赋值
void CoordinateTransformer::ColoredPointCloud(const std::vector<cv::Point2i> &pixelCoordinates,
                                              const cv::Mat &rawImage,
                                              std::vector<cv::Point3i> *BGRValue)
{
    // 将点云世界坐标转到像素坐标下的点赋予BGR值
    cv::Point3i singleBGR;
    int row = rawImage.rows;
    int col = rawImage.cols;
    for(auto & i : pixelCoordinates){
        int u = int(i.x);
        int v = int(i.y);
        // 给在像素坐标范围内的点赋BGR值
        if (u < col && u > 0 && v < row && v > 0) {
            singleBGR.x = rawImage.at<cv::Vec3b>(v, u)[0];
            singleBGR.y = rawImage.at<cv::Vec3b>(v, u)[1];
            singleBGR.z = rawImage.at<cv::Vec3b>(v, u)[2];
            BGRValue->push_back(singleBGR);
        }
        // 确保与世界点一一对应，不在像素范围内的点也赋值
        else
        {
            singleBGR.x = 0;
            singleBGR.y = 0;
            singleBGR.z = 0;
            BGRValue->push_back(singleBGR);
        }
    }
}


//void CoordinateTransformer::pointColoring(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
//                                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr &color_cloud,
//                                         cv::Mat &internalMatrix,
//                                         cv::Mat &externalMatrix,
//                                         cv::Mat &rawImage) {
//    // 将点云中所有点载入矩阵
//    cv::Mat pointsData(4, cloud->size(), CV_64F);
//    int pointNum = static_cast<int>(cloud->points.size());
//    for (int i = 0; i < pointNum; ++i) {
//        pointsData.at<double>(0, i) = cloud->points[i].x;
//        pointsData.at<double>(1, i) = cloud->points[i].y;
//        pointsData.at<double>(2, i) = cloud->points[i].z;
//        pointsData.at<double>(3, i) = 1;
//    }
//
//    // 将点云世界坐标转为像素坐标
//    std::vector<cv::Point2i> pixelCoordinates;
//    CoordinateTransformer::WorldToPixel(internalMatrix, externalMatrix, pointsData,rotation, &pixelCoordinates);
//
//    // 给点云赋BGR值
//    std::vector<cv::Point3i> BGRValue;
//    CoordinateTransformer::ColoredPointCloud(pixelCoordinates, rawImage, &BGRValue);
//
//    // 给点云赋予XYZ和BGR的值
//    for (int i = 0; i < pointNum; ++i) {
//        if (!(pointsData.at<double>(0, i) == 0 && pointsData.at<double>(1, i) == 0 && pointsData.at<double>(2, i) == 0) &&
//            !(BGRValue[i].x == 0 && BGRValue[i].y == 0 && BGRValue[i].z == 0)) {
//            color_cloud->points[i].x = static_cast<float>(pointsData.at<double>(0, i));
//            color_cloud->points[i].y = static_cast<float>(pointsData.at<double>(1, i));
//            color_cloud->points[i].z = static_cast<float>(pointsData.at<double>(2, i));
//
//            color_cloud->points[i].r = BGRValue[i].z;
//            color_cloud->points[i].g = BGRValue[i].y;
//            color_cloud->points[i].b = BGRValue[i].x;
//        }
//    }
//}

// 将雷达扫描的点云投射到图像上
void CoordinateTransformer::DrawDepthMap(const std::vector<cv::Point2i> &pixelCoordinates,
                                         const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                         const cv::Mat &rawImage) {
    // 获取点云XYZ坐标的最大最小值
    pcl::PointXYZ minPoint, maxPoint;
    pcl::getMinMax3D(*cloud, minPoint, maxPoint);
    // 点云最大深度
    float maxDepth = maxPoint.x;

    auto pointNum = (int)cloud->points.size();

    std::vector<float> depthValue;

    for (int i = 0; i < pointNum; ++i) {
        // 将点云深度归一化
        depthValue.emplace_back(cloud->points[i].x / maxDepth);
    }

    // 将转换过来的像素坐标显示在原始图片上
    for (int j = 0; j < pixelCoordinates.size(); j++) {
        int u = int(pixelCoordinates[j].x);
        int v = int(pixelCoordinates[j].y);
        // 滤除掉不在图像上的点
        if (u < rawImage.cols && u > 0 && v < rawImage.rows && v > 0){
            cv::Point2i UV(u, v);
            // 深度越大的像素B值越大，深度越小的像素R值越大
            auto B = (unsigned char)(255 * sqrt(depthValue[j]));
            auto R = (unsigned char)(255 * (1 - sqrt(depthValue[j])));
            // 绘制实心点
            cv::circle(rawImage, UV, 1, cv::Scalar(B, 0, R), -1);
        }
    }
}

// 得到车辆图像中心点坐标对应的世界坐标
cv::Point3d CoordinateTransformer::CarWorldCoordination(const cv::Point2i &carPixelCoordination,
                                                        const cv::Mat &pointsData,
                                                        const std::vector<cv::Point2i> &pixelCoordinates,
                                                        int searchRange)
{
    // 存放像素距离的容器
    std::vector<double> allDistance;
    // 存放对应像素点索引的容器
    std::vector<int> distanceIndex;

    // 搜索车辆像素点附近的雷达像素点,计算像素距离
    for (int i = 0; i < pixelCoordinates.size(); i++) {
        if (pow(pixelCoordinates[i].x - carPixelCoordination.x, 2) +
            pow(pixelCoordinates[i].y - carPixelCoordination.y, 2) < pow(searchRange, 2)) {

            // 计算距离
            double distance = sqrt(pow(pixelCoordinates[i].x - carPixelCoordination.x, 2) +
                                   pow(pixelCoordinates[i].y - carPixelCoordination.y, 2));
            // 对应的索引
            int index = i;

            // 距离和索引载入容器
            allDistance.emplace_back(distance);
            distanceIndex.emplace_back(index);
        }
    }

    // 得到相距最小距离点的索引
    auto minDistance = std::min_element(std::begin(allDistance), std::end(allDistance));
    auto minDistanceIndex = (int)std::distance(std::begin(allDistance), minDistance);

    if(!distanceIndex.empty())
    {
        int realMinIndex = distanceIndex[minDistanceIndex];
        // 得到离车辆图像中心点坐标最近的雷达像素点对应的世界坐标
        double carWorldX = pointsData.at<double>(0, realMinIndex);
        double carWorldY = pointsData.at<double>(1, realMinIndex);
        double carWorldZ = pointsData.at<double>(2, realMinIndex);

        // 返回世界坐标结果
        return {carWorldX, carWorldY, carWorldZ};
    }
    else
        return {0, 0, 0};

//    // 清除存放像素距离容器的所有数据
//    allDistance.clear();
//    // 清除存放对应像素点索引容器的所有数据
//    distanceIndex.clear();

}

//cv::Point3d CoordinateTransformer::toWorldPoint(const cv::Point2i &carPixelCoordination,const cv::Mat &pointsData)
//{
//    cv::Point3d worldPoint(1.3,1.2,1.5);
//    return worldPoint;
//}


cv::Mat CoordinateTransformer::average_cameraPoint    = cv::Mat(4, 1, CV_64F);
