//
// Created by zhangtianyi on 2023/12/19.
//

#include "outparam.h"

    bool CameraCalibration::leftButtonPressed = false; // 初始化静态成员变量，只有静态成员变量可以这样初始化，其他的要在构造函数内初始化

    cv::Mat CameraCalibration::rotation    = cv::Mat(3, 1, CV_32F);
    cv::Mat CameraCalibration::translation = cv::Mat(3, 1, CV_32F);


CameraCalibration::CameraCalibration(int enemy, int camera_type)
    : enemy(enemy), camera_type(camera_type), confirm(false) {}

    void CameraCalibration::mouseCallback(int event, int x, int y, int flags, void *userdata) {
        CameraCalibration *calibrator = static_cast<CameraCalibration *>(userdata);
        if (event == cv::EVENT_MOUSEMOVE && !calibrator->leftButtonPressed) {
            // 处理鼠标悬浮事件
            calibrator->updateZoomedRegion(x, y);
            calibrator->drawCalibrationPoints(calibrator->frame, x, y);
        } else if (event == cv::EVENT_LBUTTONDOWN) {
            // 处理鼠标左键按下事件
            calibrator->leftButtonPressed = true;
            calibrator->recordClick(x, y);
        } else if (event == cv::EVENT_LBUTTONUP) {
            // 处理鼠标左键释放事件
            calibrator->leftButtonPressed = false;
            // 停止放大显示
            cv::destroyWindow("Zoomed In");
        }
    }

    void CameraCalibration::updateZoomedRegion(int x, int y) {
        // 根据鼠标悬浮点更新放大界面的区域
        int zoomSize = 100;  // 放大窗口的尺寸
        int halfZoomSize = zoomSize / 2;
        int startX = std::max(0, static_cast<int>(x) - halfZoomSize);
        int startY = std::max(0, static_cast<int>(y) - halfZoomSize);
        int endX = std::min(frame.cols, startX + zoomSize);
        int endY = std::min(frame.rows, startY + zoomSize);

        zoomedRegion = cv::Rect(startX, startY, endX - startX, endY - startY);
    }

    void CameraCalibration::drawCalibrationPoints(cv::Mat frame, int x, int y) {
        int zoomSize = 100;
        // 显示当前画面
        cv::imshow("Calibration Image", frame);

        // 创建一个用于显示放大界面的窗口
        cv::namedWindow("Zoomed In", cv::WINDOW_NORMAL);

        // 创建一个临时图像，将放大区域复制到临时图像上
        cv::Mat zoomedInImage = frame(zoomedRegion).clone();

        // 在放大的窗口中绘制鼠标光标的位置
        int crosshairSize = 10;
        cv::line(zoomedInImage, cv::Point(x - zoomedRegion.x - crosshairSize / 2, y - zoomedRegion.y),
                 cv::Point(x - zoomedRegion.x + crosshairSize / 2, y - zoomedRegion.y), cv::Scalar(0, 255, 0), 1);
        cv::line(zoomedInImage, cv::Point(x - zoomedRegion.x, y - zoomedRegion.y - crosshairSize / 2),
                 cv::Point(x - zoomedRegion.x, y - zoomedRegion.y + crosshairSize / 2), cv::Scalar(0, 255, 0), 1);

        // 显示放大界面
        cv::imshow("Zoomed In", zoomedInImage);
        cv::resizeWindow("Zoomed In", zoomSize, zoomSize);
    }

    void CameraCalibration::recordClick(int x, int y) {
    // 将点击点的坐标记录到标定点容器中
    imagePoints.emplace_back(x, y);
    // 输出提示信息
    std::cout << "Recorded click at (" << x << ", " << y << ")" << std::endl;
    }

    void CameraCalibration::initializeCameraParameters() {
        // 根据标定类型获取相机内参和畸变参数
        HuarayCameraParam huarayCameraParam;
        if (camera_type == 0){
            std::string yamlFile = "/home/zhangtianyi/test_ws/src/cubot_radar/config/param/huaray_camera_param.yaml";
            HuarayCameraParam::LoadFromYamlFile(yamlFile, &huarayCameraParam);
        }
        else{
            std::string yamlFile = "/home/zhangtianyi/test_ws/src/cubot_radar/config/param/huaray_camera_param2.yaml";
            HuarayCameraParam::LoadFromYamlFile(yamlFile, &huarayCameraParam);
        }
        internalMatrix = huarayCameraParam.ModelParam.CvInternalMatrix;
        distCoeffs = huarayCameraParam.ModelParam.CvDistortionVector;

        // 初始化ops
        if (enemy == 0) { // enemy is red
            if (camera_type == 0) {  // right camera
                ops.emplace_back(2901, 1331, 1727);
                ops.emplace_back(1165, 1463, 1302);
                ops.emplace_back(9147, 5401, 8271);
                ops.emplace_back(1657, 1540, 9147);
//                std::vector<cv::Point3f> points{ cv::Point3f(-300, -300, 0),
//                                                 cv::Point3f(-300, 300, 0),
//                                                 cv::Point3f(300, 300, 0),
//                                                 cv::Point3f(300, -300, 0) };
            } else { // left camera
                std::vector<cv::Point3f> points{ cv::Point3f(1, 1, 1),
                                                 cv::Point3f(0, 1, 1),
                                                 cv::Point3f(1, 0, 1),
                                                 cv::Point3f(1, 1, 0) };
//                ops.push_back(points);
            }
        }
        else { // enemy is blue
            if (camera_type == 0) {  // right camera
                std::vector<cv::Point3f> points{ cv::Point3f(1, 1, 1),
                                                 cv::Point3f(0, 1, 1),
                                                 cv::Point3f(1, 0, 1),
                                                 cv::Point3f(1, 1, 0) };
//                ops.push_back(points);
            } else { // left camera
                std::vector<cv::Point3f> points{ cv::Point3f(1, 1, 1),
                                                 cv::Point3f(0, 1, 1),
                                                 cv::Point3f(1, 0, 1),
                                                 cv::Point3f(1, 1, 0) };
//                ops.push_back(points);
            }
        }
    }



    void CameraCalibration::displayInstructions() {
        // 根据标定类型和敌方颜色显示标定信息
        if (enemy == 0) { // enemy is red
            if (camera_type == 0) {  // right camera
                std::cout << "请依次点击: red_base, blue_outpost, b_rt, b_lt" << std::endl;
            } else { // left camera
            std::cout << "请依次点击: red_outpost, red_base, b_rt, b_lt" << std::endl;
            }
        }
        else { // enemy is blue
            if (camera_type == 0) {  // right camera
                std::cout << "请依次点击: blue_base, red_outpost, r_rt, r_lt" << std::endl;
            } else { // left camera
                std::cout << "请依次点击: blue_outpost, blue_base, r_rt, r_lt" << std::endl;
            }
        }
    }

    void CameraCalibration::performPNPCalibration(){
    // 执行PNP求解
    std::cout<<"pnp"<<std::endl;
//    for (const auto& outerVec : ops) {
//        for (const auto& point :: outerVec) {
//            std::cout << "ops:" << "x: " << point.x << ", y: " << point.y << ", z: " << point.z << std::endl;
//        }
//    }
    for (const auto& point : imagePoints) {
        std::cout << "imagePoints:" << "x: " << point.x << ", y: " << point.y << std::endl;
    }
    std::cout << "internalMatrix: " << internalMatrix << std::endl;
    std::cout << "distCoeffs: "     << distCoeffs     << std::endl;

    cv::solvePnP(ops, imagePoints, internalMatrix, distCoeffs, rotation, translation, true, 2);

    cv::Mat rotation3_3 = cv::Mat(3, 3, CV_32F);
    cv::Rodrigues(rotation,rotation3_3);

    // 结果返回
    std::cout << "Translation Vector: " << translation << std::endl;
    std::cout << "Rotation Vector: " << rotation << std::endl;
    std::cout << "Rotation Matrix: " << rotation3_3 << std::endl;
    }

void CameraCalibration::runCalibration(cv::Mat& image) {

        if (image.empty()) {
            std::cerr << "Error: No image." << std::endl;
            return;
        }
        frame = image;
        cv::namedWindow("Calibration Image");
        cv::setMouseCallback("Calibration Image", mouseCallback, this);

        cv::imshow("Calibration Image", image);


//        // 关闭摄像头或释放资源
//        cv::destroyAllWindows();
}

void CameraCalibration::sentCalibrationparam() {
    int ss = 1;
    //pnp出的旋转向量转换成旋转矩阵
    cv::Mat rotation3_3;
    cv::Rodrigues(rotation, rotation3_3);
    //cv::mat转换成eigne::Matrix3f
    Eigen::Matrix3f eigen_rotation;
    cv::cv2eigen(rotation3_3, eigen_rotation);
    // 将 旋转平移 转换为 geometry_msgs::TransformStamped 消息
    geometry_msgs::TransformStamped transformStamped;
    // 创建 TransformBroadcaster
    tf2_ros::TransformBroadcaster tfBroadcaster;

    //tf树的父子坐标系一般代表将子坐标系中的点转换到父坐标系中的变换关系
    transformStamped.header.frame_id = "world";  // 世界坐标系
    transformStamped.child_frame_id = "camera"; // 相机坐标系

    transformStamped.transform.translation.x = translation.at<float>(0, 0);
    transformStamped.transform.translation.y = translation.at<float>(1, 0);
    transformStamped.transform.translation.z = translation.at<float>(2, 0);

    Eigen::Quaternionf q(eigen_rotation);
    transformStamped.transform.rotation.w = q.w();
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();

    while (ss<100000) {
        transformStamped.header.stamp = ros::Time::now();
        // 发布 TransformStamped 消息到话题
        tfBroadcaster.sendTransform(transformStamped);
    }
}







