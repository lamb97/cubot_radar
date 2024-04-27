#include <ros/ros.h>
#include <rosbag/bag.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>
#include "huaray_camera.h"
#include "detect_cars_info.h"
#include "detect_armors_info.h"
#include "detector_param.h"
#include "detect_yolo.h"
#include "cubot_radar/CarsMsg.h"

ros::Publisher carsMsgPublisher1;
ros::Publisher carsMsgPublisher2;

void ControlBody1(HuarayCamera *camera,
                  ros::NodeHandle *nh)
{
    // 初始化cubot_radar::CarsMsg消息类型
    cubot_radar::CarsMsg carsMsg;

    // 初始化目标检测器参数
    DetectorParam detectorParam;
    std::string yamlfilePath = "/home/zhangtianyi/test_ws/src/cubot_radar/config/param/YOLOv5_detect.yaml";
    // 加载目标检测器参数
    detectorParam.LoadDetectorParam(yamlfilePath);

    // 初始化目标检测器并加载YOLOv5模型
    YOLOv5Detector carDetector(detectorParam.YOLOv5CarModelPath, torch::kCUDA);
    YOLOv5Detector armorDetector(detectorParam.YOLOv5ArmorModelPath, torch::kCUDA);

    // 声明原图片和车的区域图片
    cv::Mat rawImage;
    cv::Mat carImage;

    // 初始化车辆对象
    DetectCarInfo detectCarInfo;
    // 存放图像中车辆信息的容器
    std::vector<DetectCarInfo> detectCarsInfo;

    // 使用目标检测器得到结果
    std::vector<Detection> carResult;
    std::vector<Detection> armorResult;

    // 设置循环频率
    ros::Rate loopRate(15);

    while (camera->IsOpened() && nh->ok()){
        HuarayCameraData data;
        camera->GetData(&data);
        rawImage = data.Image;
        spdlog::info("----------New Frame----------");
        auto carStart = std::chrono::high_resolution_clock::now();

        // 对图片车辆目标进行预测
        carResult = carDetector.CarDetect(rawImage,
                                          detectorParam.YOLOv5ConfThres,
                                          detectorParam.YOLOv5IouThres);
        // 计算车辆检测耗时
        auto carEnd = std::chrono::high_resolution_clock::now();
        auto carDuration = std::chrono::duration_cast<std::chrono::milliseconds>(carEnd - carStart);
        spdlog::info("Car1 process takes : {0} ms", carDuration.count());

        for (uint8_t i = 0; i < (uint8_t) carResult.size(); i++)
        {
            if ((carResult[i].bbox.width > 0) && (carResult[i].bbox.height > 0) &&
                (carResult[i].bbox.width / carResult[i].bbox.height < 20) &&
                (carResult[i].bbox.height / carResult[i].bbox.width < 20))
            {

                // 提取车辆区域
                carImage = rawImage(carResult[i].bbox);

                auto armorStart = std::chrono::high_resolution_clock::now();

                // 对图片装甲板目标进行预测
                armorResult = armorDetector.ArmorDetect(carImage,
                                                        detectorParam.YOLOv5ConfThres,
                                                        detectorParam.YOLOv5IouThres);

                auto armorEnd = std::chrono::high_resolution_clock::now();
                // 计算装甲板检测耗时
                auto armorDuration = std::chrono::duration_cast<std::chrono::milliseconds>(armorEnd - armorStart);
                spdlog::info("Armor1 process takes : {0} ms", armorDuration.count());

                // 将一辆车上检测到的所有装甲板按照分数由大到小排序
                std::vector<Detection> sortArmorResult;
                sortArmorResult = YOLOv5Detector::SortResult(armorResult);

                detectCarInfo = YOLOv5Detector::CarInfo(carResult, sortArmorResult, i);

                detectCarsInfo.emplace_back(detectCarInfo);
            }
        }

        // 一帧图像里车辆的数量
        auto carNum = (int) detectCarsInfo.size();
        ROS_INFO("Car1: %i", carNum);
        // 给cubot_radar::CarsMsg话题载入一帧图像中的车辆及装甲板信息
        carsMsg.carNum = carNum;

        // 初始化车辆信息的容器
        carsMsg.carsInfo.resize(carNum);

        for (int p = 0; p < carNum; ++p) {
            // 车辆的编号
            carsMsg.carsInfo[p].carClass = detectCarsInfo[p].carClass;
            // 车辆的中心点x坐标
            carsMsg.carsInfo[p].carX = detectCarsInfo[p].carX;
            // 车辆的中心点y坐标
            carsMsg.carsInfo[p].carY = detectCarsInfo[p].carY;
            // 车辆的宽度
            carsMsg.carsInfo[p].carWidth = detectCarsInfo[p].carWidth;
            // 车辆的高度
            carsMsg.carsInfo[p].carHeight = detectCarsInfo[p].carHeight;

            // 一辆车里装甲板的数量
            auto armorNum = (int) detectCarsInfo[p].detectArmorsInfo.size();

            // 一个车辆图片里装甲板的个数
            carsMsg.carsInfo[p].armorNum = armorNum;

            // 初始化装甲板信息的容器
            carsMsg.carsInfo[p].armorsInfo.resize(armorNum);

            for (int q = 0; q < armorNum; ++q) {
                // 装甲板的编号
                carsMsg.carsInfo[p].armorsInfo[q].armorClass = detectCarsInfo[p].detectArmorsInfo[q].armorClass;
                // 装甲板的中心点x坐标
                carsMsg.carsInfo[p].armorsInfo[q].armorX = detectCarsInfo[p].detectArmorsInfo[q].armorX;
                // 装甲板的中心点y坐标
                carsMsg.carsInfo[p].armorsInfo[q].armorY = detectCarsInfo[p].detectArmorsInfo[q].armorY;
                // 装甲板的宽度
                carsMsg.carsInfo[p].armorsInfo[q].armorWidth = detectCarsInfo[p].detectArmorsInfo[q].armorWidth;
                // 装甲板的高度2
                carsMsg.carsInfo[p].armorsInfo[q].armorHeight = detectCarsInfo[p].detectArmorsInfo[q].armorHeight;
            }
        }

        // 清除车辆信息容器数据
        detectCarsInfo.clear();

        // 设置车辆消息发布时间与ros时间同步
        carsMsg.header.stamp = ros::Time::now();

        // 将OpenCV格式的图像转换为ros图像发布格式
        sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rawImage).toImageMsg();
        carsMsg.image = *imageMsg;

        // 判断是否有图像数据
        if (!carsMsg.image.data.empty())
            // 发布车辆信息
            carsMsgPublisher1.publish(carsMsg);

        loopRate.sleep();
    }
}

void ControlBody2(HuarayCamera *camera,
                  ros::NodeHandle *nh)
{
    // 初始化cubot_radar::CarsMsg消息类型
    cubot_radar::CarsMsg carsMsg;

    // 初始化目标检测器参数
    DetectorParam detectorParam;
    std::string yamlfilePath = "/home/zhangtianyi/test_ws/src/cubot_radar/config/param/YOLOv5_detect.yaml";
    // 加载目标检测器参数
    detectorParam.LoadDetectorParam(yamlfilePath);

    // 初始化目标检测器并加载YOLOv5模型
    YOLOv5Detector carDetector(detectorParam.YOLOv5CarModelPath, torch::kCUDA);
    YOLOv5Detector armorDetector(detectorParam.YOLOv5ArmorModelPath, torch::kCUDA);

    // 声明原图片和车的区域图片
    cv::Mat rawImage;
    cv::Mat carImage;

    // 初始化车辆对象
    DetectCarInfo detectCarInfo;
    // 存放图像中车辆信息的容器
    std::vector<DetectCarInfo> detectCarsInfo;

    // 使用目标检测器得到结果
    std::vector<Detection> carResult;
    std::vector<Detection> armorResult;

    // 设置循环频率
    ros::Rate loopRate(15);

    while (camera->IsOpened() && nh->ok()){
        HuarayCameraData data;
        camera->GetData(&data);
        rawImage = data.Image;
        spdlog::info("----------New ----------");
        auto carStart = std::chrono::high_resolution_clock::now();

        // 对图片车辆目标进行预测
        carResult = carDetector.CarDetect(rawImage,
                                          detectorParam.YOLOv5ConfThres,
                                          detectorParam.YOLOv5IouThres);

        auto carEnd = std::chrono::high_resolution_clock::now();
        // 计算车辆检测耗时
        auto carDuration = std::chrono::duration_cast<std::chrono::milliseconds>(carEnd - carStart);
        spdlog::info("Car2 process takes : {0} ms", carDuration.count());

        for (uint8_t i = 0; i < (uint8_t) carResult.size(); i++)
        {
            if ((carResult[i].bbox.width > 0) && (carResult[i].bbox.height > 0) &&
                (carResult[i].bbox.width / carResult[i].bbox.height < 20) &&
                (carResult[i].bbox.height / carResult[i].bbox.width < 20))
            {

                // 提取车辆区域
                carImage = rawImage(carResult[i].bbox);

                auto armorStart = std::chrono::high_resolution_clock::now();

                // 对图片装甲板目标进行预测
                armorResult = armorDetector.ArmorDetect(carImage,
                                                        detectorParam.YOLOv5ConfThres,
                                                        detectorParam.YOLOv5IouThres);

                auto armorEnd = std::chrono::high_resolution_clock::now();
                // 计算装甲板检测耗时
                auto armorDuration = std::chrono::duration_cast<std::chrono::milliseconds>(armorEnd - armorStart);
                spdlog::info("Armor2 process takes : {0} ms", armorDuration.count());

                // 将一辆车上检测到的所有装甲板按照分数由大到小排序
                std::vector<Detection> sortArmorResult;
                sortArmorResult = YOLOv5Detector::SortResult(armorResult);

                detectCarInfo = YOLOv5Detector::CarInfo(carResult, sortArmorResult, i);

                detectCarsInfo.emplace_back(detectCarInfo);
            }
        }

        // 一帧图像里车辆的数量
        auto carNum = (int) detectCarsInfo.size();
        ROS_INFO("Car2: %i", carNum);
        // 给cubot_radar::CarsMsg话题载入一帧图像中的车辆及装甲板信息
        carsMsg.carNum = carNum;

        // 初始化车辆信息的容器
        carsMsg.carsInfo.resize(carNum);

        for (int p = 0; p < carNum; ++p)
        {
            // 车辆的编号
            carsMsg.carsInfo[p].carClass = detectCarsInfo[p].carClass;
            // 车辆的中心点x坐标
            carsMsg.carsInfo[p].carX = detectCarsInfo[p].carX;
            // 车辆的中心点y坐标
            carsMsg.carsInfo[p].carY = detectCarsInfo[p].carY;
            // 车辆的宽度
            carsMsg.carsInfo[p].carWidth = detectCarsInfo[p].carWidth;
            // 车辆的高度
            carsMsg.carsInfo[p].carHeight = detectCarsInfo[p].carHeight;

            // 一辆车里装甲板的数量
            auto armorNum = (int) detectCarsInfo[p].detectArmorsInfo.size();

            // 一个车辆图片里装甲板的个数
            carsMsg.carsInfo[p].armorNum = armorNum;

            // 初始化装甲板信息的容器
            carsMsg.carsInfo[p].armorsInfo.resize(armorNum);

            for (int q = 0; q < armorNum; ++q)
            {
                // 装甲板的编号
                carsMsg.carsInfo[p].armorsInfo[q].armorClass = detectCarsInfo[p].detectArmorsInfo[q].armorClass;
                // 装甲板的中心点x坐标
                carsMsg.carsInfo[p].armorsInfo[q].armorX = detectCarsInfo[p].detectArmorsInfo[q].armorX;
                // 装甲板的中心点y坐标
                carsMsg.carsInfo[p].armorsInfo[q].armorY = detectCarsInfo[p].detectArmorsInfo[q].armorY;
                // 装甲板的宽度
                carsMsg.carsInfo[p].armorsInfo[q].armorWidth = detectCarsInfo[p].detectArmorsInfo[q].armorWidth;
                // 装甲板的高度
                carsMsg.carsInfo[p].armorsInfo[q].armorHeight = detectCarsInfo[p].detectArmorsInfo[q].armorHeight;
            }
        }


//        cv::Mat siteMap = cv::imread("/home/godzhu/radar_ws/src/cubot_radar/data/picture/RM场地.png");
//        cv::imshow("siteMap", siteMap);
//        cv::waitKey(1);

        // 清除车辆信息容器数据
        detectCarsInfo.clear();

        // 设置车辆消息发布时间与ros时间同步
        carsMsg.header.stamp = ros::Time::now();

        // 将OpenCV格式的图像转换为ros图像发布格式
        sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rawImage).toImageMsg();
        carsMsg.image = *imageMsg;

        // 判断是否有图像数据
        if (!carsMsg.image.data.empty())
            // 发布车辆信息
            carsMsgPublisher2.publish(carsMsg);

        loopRate.sleep();
    }
}

int main(int argc, char *argv[]) {
    //初始化"image_publisher_one"结点
    ros::init(argc, argv, "image_publisher_one");
    // 创建ros句柄
    ros::NodeHandle nh;

    // ros自定义话题的发布器(话题名carsInfo,队列长度为1)
    carsMsgPublisher1 = nh.advertise<cubot_radar::CarsMsg>("carsInfo", 1);
    carsMsgPublisher2 = nh.advertise<cubot_radar::CarsMsg>("carsInfo_two", 1);

    cv::namedWindow("rawImage", cv::WINDOW_NORMAL);


/*
    // 初始化目标检测器参数
    DetectorParam detectorParam;
    std::string yamlfilePath = "/home/zhangtianyi/test_ws/src/cubot_radar/config/param/YOLOv5_detect.yaml";
    // 加载目标检测器参数
    detectorParam.LoadDetectorParam(yamlfilePath);
   // 读取视频数据
   cv::VideoCapture capture(detectorParam.VideoPath);
   if (!capture.isOpened()) {
       spdlog::error("Video can not be opened because {0} was error!", detectorParam.VideoPath);
        return -1;
    }
*/

    //  读取相机1
    cv::Mat imagePub1;
    HuarayCameraParam param1;
    std::string yamlFile1 = "/home/zhangtianyi/test_ws/src/cubot_radar/config/param/huaray_camera_param.yaml";
    HuarayCameraParam::LoadFromYamlFile(yamlFile1, &param1);

    //  读取相机2
    cv::Mat imagePub2;
    HuarayCameraParam param2;
    std::string yamlFile2 = "/home/zhangtianyi/test_ws/src/cubot_radar/config/param/huaray_camera_param2.yaml";
    HuarayCameraParam::LoadFromYamlFile(yamlFile2, &param2);

    // 初始化华睿相机1
    HuarayCamera camera1;
    camera1.SetParam(param1);
    camera1.Init();
    camera1.WriteHardwareParamToDevice(param1.HardwareParams[0]);
    camera1.Open();

    // 初始化华睿相机2
    HuarayCamera camera2;
    camera2.SetParam(param2);
    camera2.Init();
    camera2.WriteHardwareParamToDevice(param2.HardwareParams[0]);
    camera2.Open();

    //创建新线程controlThread1时，代码ControlBody1将自动在这个新线程中执行
    std::thread controlThread1 = std::thread(ControlBody1,
                                             &camera1,
                                             &nh);

    std::thread controlThread2 = std::thread(ControlBody2,
                                             &camera2,
                                             &nh);

    // 等待机器人本体控制线程1结束
    if (controlThread1.joinable()) {
        controlThread1.join();
    }

    // 等待机器人本体控制线程2结束
    if (controlThread2.joinable()) {
        controlThread2.join();
    }

    return 0;
}
















