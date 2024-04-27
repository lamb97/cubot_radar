//
// Created by godzhu on 2021/12/8.
//

#include "detector_param.h"
#include "detect_yolo.h"

int main(int argc, char *argv[]) {
    cv::namedWindow("rawImage", cv::WINDOW_NORMAL);
    // 初始化目标检测器参数
    DetectorParam detectorParam;
    std::string yamlfilePath = "/home/zhangtianyi/test_ws/src/cubot_radar/config/param/YOLOv5_detect.yaml";
    // 加载目标检测器参数
    detectorParam.LoadDetectorParam(yamlfilePath);

    // 初始化目标检测器并加载YOLOv5模型
    YOLOv5Detector detector("/home/zhangtianyi/test_ws/src/cubot_radar/config/model/YOLOv5_radar.pt", torch::kCUDA);

    // 加载目标检测的类别
    std::string className;
    std::vector<std::string> classNames;
    classNames = detector.LoadNames(detectorParam.YOLOv5NamePath);

    // 读取视频数据
    cv::Mat rawImage;
    cv::VideoCapture capture(detectorParam.VideoPath);
    if (!capture.isOpened()) {
        spdlog::error("Video can not be opened because {0} was error!", detectorParam.VideoPath);
        return -1;
    }

    // 对视频数据进行处理
    while (capture.read(rawImage)) {
        // 使用目标检测器得到结果
        std::vector<Detection> result;
        spdlog::info("----------New Frame----------");
        auto start = std::chrono::high_resolution_clock::now();
        // 对图片目标进行预测
        result = detector.CarDetect(rawImage, detectorParam.YOLOv5ConfThres, detectorParam.YOLOv5IouThres);
        auto end = std::chrono::high_resolution_clock::now();
        // 计算耗时
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        spdlog::info("process takes : {0} ms", duration.count());

        for (int i = 0; i < result.size(); i++) {
            if (result[i].classIndex == 0) {
                // 画框显示所有车
                cv::rectangle(rawImage, result[i].bbox, cv::Scalar(255, 255, 255), 2);
                for (int j = 0; j < result.size(); j++) {
                    switch (result[j].classIndex) {
                        // 标签为1对应红1装甲板
                        case 1: {
                            className = classNames[1];

                            // 画框显示装甲板区域
                            cv::rectangle(rawImage, result[j].bbox, cv::Scalar(0, 0, 255), 2);

                            // 判断装甲板是否在车内
                            if (result[j].bbox == (result[j].bbox & result[i].bbox)){
                                // 画框显示车辆区域
                                cv::rectangle(rawImage, result[i].bbox, cv::Scalar(0, 0, 255), 2);

                                // 添加车辆类别显示信息
                                cv::putText(rawImage, className, cv::Point(result[i].bbox.tl().x, result[i].bbox.tl().y - 5),
                                            cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 255, 255), 1);
                            }

                            break;
                        }
                        // 标签为2对应红2装甲板
                        case 2: {
                            className = classNames[2];

                            // 画框显示装甲板区域
                            cv::rectangle(rawImage, result[j].bbox, cv::Scalar(0, 0, 255), 2);

                            // 判断装甲板是否在车内
                            if (result[j].bbox == (result[j].bbox & result[i].bbox)){
                                // 画框显示车辆区域
                                cv::rectangle(rawImage, result[i].bbox, cv::Scalar(0, 0, 255), 2);

                                // 添加车辆类别显示信息
                                cv::putText(rawImage, className, cv::Point(result[i].bbox.tl().x, result[i].bbox.tl().y - 5),
                                            cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 255, 255), 1);
                            }

                            break;
                        }
                        // 标签为3对应蓝1装甲板
                        case 3: {
                            className = classNames[3];

                            // 画框显示装甲板区域
                            cv::rectangle(rawImage, result[j].bbox, cv::Scalar(255, 0, 0), 2);

                            // 判断装甲板是否在车内
                            if (result[j].bbox == (result[j].bbox & result[i].bbox)){
                                // 画框显示车辆区域
                                cv::rectangle(rawImage, result[i].bbox, cv::Scalar(255, 0, 0), 2);

                                // 添加车辆类别显示信息
                                cv::putText(rawImage, className, cv::Point(result[i].bbox.tl().x, result[i].bbox.tl().y - 5),
                                            cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 255, 255), 1);
                            }

                            break;
                        }
                        // 标签为4对应蓝2装甲板
                        case 4: {
                            className = classNames[4];

                            // 画框显示装甲板区域
                            cv::rectangle(rawImage, result[j].bbox, cv::Scalar(255, 0, 0), 2);

                            // 判断装甲板是否在车内
                            if (result[j].bbox == (result[j].bbox & result[i].bbox)){
                                // 画框显示车辆区域
                                cv::rectangle(rawImage, result[i].bbox, cv::Scalar(255, 0, 0), 2);

                                // 添加车辆类别显示信息
                                cv::putText(rawImage, className, cv::Point(result[i].bbox.tl().x, result[i].bbox.tl().y - 5),
                                            cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 255, 255), 1);
                            }

                            break;
                        }
                        default:
                            break;
                    }
                }
            }

        }
        cv::namedWindow("rawImage", 0);//
        cv::resizeWindow("rawImage",700, 500);//
        cv::imshow("rawImage", rawImage);
        cv::waitKey(0);
    }

    return 0;
}
