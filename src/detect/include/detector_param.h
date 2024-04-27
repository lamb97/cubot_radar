//
// Created by godzhu on 2021/12/8.
//

#ifndef POSITION_PREDICT_PY_INIT_H
#define POSITION_PREDICT_PY_INIT_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "spdlog/spdlog.h"

class DetectorParam
{
public:

    // 构造函数
    DetectorParam();

    // 析构函数
    ~DetectorParam();

    float YOLOv5ConfThres;            // YOLOv5的置信度阈值
    float YOLOv5IouThres;             // YOLOv5的iou区域阈值
    std::string YOLOv5CarModelPath;   // YOLOv5的检测car模型路径
    std::string YOLOv5ArmorModelPath; // YOLOv5的检测装甲板的模型路径
    std::string YOLOv5ArmorNamePath;  // YOLOv5的识别装甲板类别路径

    /**
     * @brief 导入目标检测器的参数
     * @param[in] yamlFileName 参数文件的路径
     * @return 参数加载结果\n
     *         -<em>false</em> 参数加载失败
     *         -<em>false</em> 参数加载成功
     */
    bool LoadDetectorParam(const std::string& yamlFileName);
};

#endif //POSITION_PREDICT_PY_INIT_H
