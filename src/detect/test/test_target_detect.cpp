//
// Created byo godzhu 2021/12/19.
//

#include "detector_param.h"
#include "detect_yolo.h"
#include "huaray_camera.h"

int main(int argc, char *argv[]) {
    // 初始化目标检测器参数
    DetectorParam detectorParam;
    std::string yamlfilePath = "/home/zhangtianyi/document/radar_ws/src/cubot_radar/config/param/YOLOv5_detect.yaml";
    // 加载目标检测器参数
    detectorParam.LoadDetectorParam(yamlfilePath);

    // 初始化目标检测器并加载YOLOv5模型
    YOLOv5Detector carDetector;
    YOLOv5Detector armorDetector;

    carDetector.YOLOv5Load(detectorParam.YOLOv5CarModelPath);
    armorDetector.YOLOv5Load(detectorParam.YOLOv5ArmorModelPath);

    // 加载装甲板目标检测的类别
    std::string armorClassName;
    std::vector<std::string> armorClassNames = {"BG", "B1", "B2","B3", "B4", "B5","BO", "BBS","BBB", "RG","R1",
                                                "R2","R3", "R4", "R5","RO", "RBs", "RBb","NG", "N1", "N2",
                                                "N3", "N4", "N5","NO", "NBS", "NBB","PG","P1", "P2", "P3","P4",
                                                "P5", "PO","PBS", "PBB"};

    //armorClassNames = YOLOv5Detector::LoadNames(detectorParam.YOLOv5ArmorNamePath);

    // 声明原图片和车的区域图
    cv::Mat rawImage;
    cv::Mat carImage;

    // 使用目标检测器得到结果
    std::vector<Detection> carResult;
    std::vector<Detection> armorResult;

    // 读取相机
    cv::Mat imagePub;
    HuarayCameraParam param;
    std::string yamlFile = "/home/zhangtianyi/document/radar_ws/src/cubot_radar/config/param/huaray_camera_param.yaml";
    HuarayCameraParam::LoadFromYamlFile(yamlFile, &param);

    // 初始化华睿相机
    HuarayCamera camera;
    camera.SetParam(param);
    camera.Init();
    camera.WriteHardwareParamToDevice(param.HardwareParams[0]);
    camera.Open();

    while (camera.IsOpened()){
        HuarayCameraData data;
        camera.GetData(&data);
        rawImage = data.Image;

        spdlog::info("----------New Frame----------");
        auto carStart = std::chrono::high_resolution_clock::now();

        // 对图片车辆目标进行预测
        carResult = carDetector.CarDetect(rawImage, detectorParam.YOLOv5ConfThres, detectorParam.YOLOv5IouThres);

        auto carEnd = std::chrono::high_resolution_clock::now();
        // 计算车辆检测耗时
        auto carDuration = std::chrono::duration_cast<std::chrono::milliseconds>(carEnd - carStart);
        spdlog::info("Car process takes : {0} ms", carDuration.count());

        // 存放图像中车的类别和中心点坐标的容器
        std::vector<std::pair<int, cv::Point2i>> carsPixelCoordination;
        std::pair<int, cv::Point2i> carPixelCoordination;

        for (auto &i : carResult) {
            if ((i.bbox.width > 0) && (i.bbox.height > 0) && (i.bbox.width / i.bbox.height < 20) &&
                                      (i.bbox.height / i.bbox.width < 20)) {
                // 画框显示所有车
                cv::rectangle(rawImage, i.bbox, cv::Scalar(255, 255, 255), 2);

                // 提取车辆区域
                carImage = rawImage(i.bbox);

                auto armorStart = std::chrono::high_resolution_clock::now();

                // 对图片装甲板目标进行预测
                armorResult = armorDetector.ArmorDetect(carImage, detectorParam.YOLOv5ConfThres,
                                                        detectorParam.YOLOv5IouThres);

                auto armorEnd = std::chrono::high_resolution_clock::now();
                // 计算装甲板检测耗时
                auto armorDuration = std::chrono::duration_cast<std::chrono::milliseconds>(armorEnd - armorStart);
                spdlog::info("Armor process takes : {0} ms", armorDuration.count());

                std::vector<Detection> sortArmorResult;
                sortArmorResult = YOLOv5Detector::SortResult(armorResult);

                // 将一帧图像中车的类别和中心坐标载入容器
                if (!sortArmorResult.empty()) {
                    carPixelCoordination.first = sortArmorResult[0].classIndex;
                    carPixelCoordination.second.x = i.bbox.x;
                    carPixelCoordination.second.y = i.bbox.y;
                    carsPixelCoordination.emplace_back(carPixelCoordination);

                    std::cout<<"  //////////"<<carPixelCoordination.first<<std::endl;

                    // 根据得分从大到小排序的装甲板确定车的类别
                    armorClassName = armorClassNames[sortArmorResult[0].classIndex];

                    // 添加车辆类别显示信息
                    cv::putText(rawImage, armorClassName,
                                cv::Point(i.bbox.tl().x, i.bbox.tl().y - 5),
                                cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 255, 255), 1);
                }

                for (auto &j : sortArmorResult) {
                    if ((j.bbox.width > 0) && (j.bbox.height > 0) && (j.bbox.width / j.bbox.height < 20) &&
                                              (j.bbox.height / j.bbox.width < 20)) {

                        // 将在车区域内的装甲板框还原到原图
                        cv::Rect armorBox(j.bbox.x + i.bbox.x,
                                          j.bbox.y + i.bbox.y,
                                          j.bbox.width,
                                          j.bbox.height);

                        // 标签为1~5对应蓝色装甲板
                        if (j.classIndex >= 1 && j.classIndex < 6) {
                            // 画框显示装甲板区域
                            cv::rectangle(rawImage, armorBox, cv::Scalar(255, 0, 0), 2);

                            // 画框显示车辆区域
                            cv::rectangle(rawImage, i.bbox, cv::Scalar(255, 0, 0), 2);
                        }

                        // 标签为11~15对应红色装甲板
                        if (j.classIndex >= 10 && j.classIndex < 15) {
                            // 画框显示装甲板区域
                            cv::rectangle(rawImage, armorBox, cv::Scalar(0, 0, 255), 2);

                            // 画框显示车辆区域
                            cv::rectangle(rawImage, i.bbox, cv::Scalar(0, 0, 255), 2);
                        }
                    }
                }
            }
        }

        cv::namedWindow("rawImage", 0);//
        cv::resizeWindow("rawImage",1200, 1000);//
        cv::imshow("rawImage", rawImage);
        cv::waitKey(1);
    }

    return 0;
}
