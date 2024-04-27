#ifndef SRC_DETECT_YOLO_H
#define SRC_DETECT_YOLO_H

#include <memory>

//#include <torch/script.h>
//#include <torch/torch.h>
//
//#include <c10/cuda/CUDAStream.h>
//#include <ATen/cuda/CUDAEvent.h>

#include <iostream>
#include <opencv2/opencv.hpp>

#include "utils.h"
#include "spdlog/spdlog.h"
#include "detect_cars_info.h"
#include "detect_armors_info.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
//#include <cuda_provider_factory.h>
#include <include1/onnxruntime_cxx_api.h>
#include <opencv2/videoio.hpp>
#include <chrono>


class YOLOv5Detector {
public:
    /**
     * @brief 构造函数
     */
    YOLOv5Detector();

    /**
     * @brief 析构函数
     */
    ~YOLOv5Detector()= default;

    /***
     * @brief 加载网络模型
     * @param YOLOv5ModelPath  - YOLOv5的模型路径
     */
    void YOLOv5Load(const std::string& YOLOv5ModelPath);

    /***
     * @brief car的推理模块
     * @param[in] rawImage 输入图片
     * @param[in] YOLOv5ConfThres 置信度阈值
     * @param[in] YOLOv5IouThres IoU Iou区域置信度阈值
     * @return 检测结果 - bounding box, score, index
     */
    std::vector<Detection> CarDetect(const cv::Mat& rawImage, float YOLOv5ConfThres, float YOLOv5IouThres);

    /***
     * @brief 推理模块
     * @param[in] carImage 输入图片
     * @param[in] YOLOv5ConfThres 置信度阈值
     * @param[in] YOLOv5IouThres IoU Iou区域置信度阈值
     * @return 检测结果 - bounding box, score, index
     */
    std::vector<Detection> ArmorDetect(const cv::Mat& carImage, float YOLOv5ConfThres, float YOLOv5IouThres);

    /**
     * @brief 将检测类别加载入容器
     * @param[in] YOLOv5NamePath 参数文件位置
     * @return 存放检测类别的容器
     */
    static std::vector<std::string> LoadNames(const std::string& YOLOv5NamePath);

/*    *//**
     * @brief 根据检测结果的得分大小对结果进行排序
     * @param[in] result 原存放结果的容器
     * @return 排序好后存放结果的容器
     */
    static std::vector<Detection> SortResult(std::vector<Detection>& result);

    /**
     * @brief 将检测到的车辆及车辆中装甲板信息存储到容器
     * @param carResult         检测的车辆结果
     * @param sortArmorResult   检测的装甲板结果
     * @return 车辆及车辆中装甲板信息
     */
    static DetectCarInfo CarInfo(const std::vector<Detection>& carResult,
                                 const std::vector<Detection>& sortArmorResult,
                                 const uint8_t& i);

private:


    /**
     * @brief 归一化
     * @param src - image
     * @param dst - input_image_
     */
    void normalize_(cv::Mat img);

    /**
     * @brief 非极大值抑制
     * @param src - input_boxes
     * @param src - YOLOv5IouThres
     * @param dst - input_boxes
     */
    void nms(std::vector<BoxInfo>& input_boxes, float YOLOv5IouThres);


private:
    std::vector<float> input_image_;
    int inpWidth;
    int inpHeight;
    int nout;
    int num_proposal;


    Ort::Env env = Ort::Env(ORT_LOGGING_LEVEL_ERROR, "YOLOface");
    Ort::Session *ort_session = nullptr;
    Ort::SessionOptions sessionOptions = Ort::SessionOptions();
    //    OrtStatus* status = OrtSessionOptionsAppendExecutionProvider_CUDA(sessionOptions, 0);
    std::vector<char*> input_names;
    std::vector<char*> output_names;
    std::vector<std::vector<int64_t>> input_node_dims; // >=1 outputs
    std::vector<std::vector<int64_t>> output_node_dims; // >=1 outputs
};

#endif //SRC_DETECT_YOLO_H
