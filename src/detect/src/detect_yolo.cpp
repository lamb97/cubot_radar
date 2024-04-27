



#include "detect_yolo.h"



YOLOv5Detector::YOLOv5Detector(){}

// 加载网络模型
void YOLOv5Detector::YOLOv5Load(const std::string &YOLOv5ModelPath) {
    sessionOptions.SetGraphOptimizationLevel(ORT_ENABLE_BASIC);
    ort_session = new Ort::Session(env, YOLOv5ModelPath.c_str(), sessionOptions);//加载模型
    size_t numInputNodes = ort_session->GetInputCount();// 获取输入和输出节点的数量
    size_t numOutputNodes = ort_session->GetOutputCount();
    Ort::AllocatorWithDefaultOptions allocator;

    for (int i = 0; i < numInputNodes; i++)
    {
        input_names.push_back(ort_session->GetInputName(i, allocator));
        Ort::TypeInfo input_type_info = ort_session->GetInputTypeInfo(i);
        auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
        auto input_dims = input_tensor_info.GetShape();
        input_node_dims.push_back(input_dims);
    }
    for (int i = 0; i < numOutputNodes; i++)
    {
        output_names.push_back(ort_session->GetOutputName(i, allocator));
        Ort::TypeInfo output_type_info = ort_session->GetOutputTypeInfo(i);
        auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
        auto output_dims = output_tensor_info.GetShape();
        output_node_dims.push_back(output_dims);
    }
    this->inpHeight = input_node_dims[0][2];
    this->inpWidth  = input_node_dims[0][3];
}

std::vector<Detection> YOLOv5Detector::CarDetect(const cv::Mat &rawImage, float YOLOv5ConfThres, float YOLOv5IouThres) {
    cv::Mat dstimg;
//    std::cout<<this->inpWidth<< this->inpHeight<<endl;
    resize(rawImage, dstimg, cv::Size(this->inpWidth, this->inpHeight));
    this->normalize_(dstimg);
    std::array<int64_t, 4> input_shape_{ 1, 3, this->inpHeight, this->inpWidth };

    auto allocator_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
    Ort::Value input_tensor_ = Ort::Value::CreateTensor<float>(allocator_info, input_image_.data(), input_image_.size(), input_shape_.data(), input_shape_.size());


    std::vector<Ort::Value> ort_outputs = ort_session->Run(Ort::RunOptions{ nullptr }, &input_names[0], &input_tensor_, 1, output_names.data(), output_names.size());   // ��ʼ����
    std::vector<BoxInfo> generate_boxes;
    std::vector<Detection> car_boxes;

    Ort::Value &predictions = ort_outputs.at(0);
    auto pred_dims = predictions.GetTensorTypeAndShapeInfo().GetShape();
    num_proposal = pred_dims.at(1);
    nout = pred_dims.at(2);

    float ratioh = (float)rawImage.rows / this->inpHeight, ratiow = (float)rawImage.cols / this->inpWidth;
    int n = 0, k = 0; ///cx,cy,w,h,box_score, class_score, x1,y1,score1, ...., x5,y5,score5
    const float* pdata = predictions.GetTensorMutableData<float>();
    for (n = 0; n < this->num_proposal; n++) {
        float box_score = pdata[4];
        if (box_score > YOLOv5ConfThres) {
            float class_socre = box_score * pdata[5];
            if (class_socre > YOLOv5ConfThres) {
                float cx = pdata[0] * ratiow;    ///cx
                float cy = pdata[1] * ratioh;    ///cy
                float  w = pdata[2] * ratiow;    ///w
                float  h = pdata[3] * ratioh;    ///h

                float xmin = cx - 0.5 * w;
                float ymin = cy - 0.5 * h;
                float xmax = cx + 0.5 * w;
                float ymax = cy + 0.5 * h;

                generate_boxes.push_back(BoxInfo{ xmin, ymin, xmax, ymax, class_socre, 0});

            }
        }
        pdata += nout;
    }

    nms(generate_boxes, YOLOv5IouThres);

    for (size_t n = 0; n < generate_boxes.size(); n++) {
        int xmin = int(generate_boxes[n].x1);
        int ymin = int(generate_boxes[n].y1);
        int xmax = int(generate_boxes[n].x2);
        int ymax = int(generate_boxes[n].y2);

        // 保证 xmin、ymin、xmax、ymax 在图像边界内
        xmin = std::max(0, std::min(xmin, rawImage.cols - 1));
        ymin = std::max(0, std::min(ymin, rawImage.rows - 1));
        xmax = std::max(0, std::min(xmax, rawImage.cols - 1));
        ymax = std::max(0, std::min(ymax, rawImage.rows - 1));

        //rectangle(rawImage, cv::Point(xmin, ymin), cv::Point(int(generate_boxes[n].x2), int(generate_boxes[n].y2)), cv::Scalar(0, 0, 255), 2);
        //std::string label = cv::format("%.2f", generate_boxes[n].score/10);
        //label = "Car";
        //putText(rawImage, label, cv::Point(xmin, ymin - 5), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 255, 0), 1);

        cv::Rect roi_rect(xmin, ymin, xmax - xmin, ymax - ymin);

        car_boxes.push_back(Detection{roi_rect, generate_boxes[n].score, 0});
    }

    return car_boxes;
}

std::vector<Detection> YOLOv5Detector::ArmorDetect(const cv::Mat &carImage, float YOLOv5ConfThres, float YOLOv5IouThres)  {
    cv::Mat dstimg;
    resize(carImage, dstimg, cv::Size(this->inpWidth, this->inpHeight));
    this->normalize_(dstimg);
    std::array<int64_t, 4> input_shape_{ 1, 3, this->inpHeight, this->inpWidth };

    auto allocator_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
    Ort::Value input_tensor_ = Ort::Value::CreateTensor<float>(allocator_info, input_image_.data(), input_image_.size(), input_shape_.data(), input_shape_.size());

    std::vector<Ort::Value> ort_outputs = ort_session->Run(Ort::RunOptions{ nullptr }, &input_names[0], &input_tensor_, 1, output_names.data(), output_names.size());   // ��ʼ����
    std::vector<BoxInfo> generate_boxes;
    std::vector<Detection> armor_boxes;

    Ort::Value &predictions = ort_outputs.at(0);
    auto pred_dims = predictions.GetTensorTypeAndShapeInfo().GetShape();
    num_proposal = pred_dims.at(1);
    nout = pred_dims.at(2);

/*    std::vector<std::string> class_labels = {"BG", "B1", "B2","B3", "B4", "B5","BO", "BBS","BBB", "RG","R1",
                                             "R2","R3", "R4", "R5","RO", "RBs", "RBb","NG", "N1", "N2",
                                             "N3", "N4", "N5","NO", "NBS", "NBB","PG","P1", "P2", "P3","P4",
                                             "P5", "PO","PBS", "PBB"};*/

    float ratioh = (float)carImage.rows / this->inpHeight, ratiow = (float)carImage.cols / this->inpWidth;
    int n = 0, k = 0; ///cx,cy,w,h,box_score, class_score, x1,y1,score1, ...., x5,y5,score5
    const float* pdata = predictions.GetTensorMutableData<float>();

/*    float classScore = FLT_MIN;
    int ClassIndex = -1;
    for(unsigned int k = 0; k < 36; k++) {
        if (pdata[13 + k] > classScore) {
            ClassIndex = k;
            classScore = pdata[13 + k];
        }
    }*/

    for (n = 0; n < this->num_proposal; n++) {

        float classScore = FLT_MIN;
        int ClassIndex = -1;
        for(unsigned int k = 0; k < 36; k++) {
            if (pdata[13 + k] > classScore) {
                ClassIndex = k;
                classScore = pdata[13 + k];
            }
        }

        float box_score = pdata[4];
        if (box_score > YOLOv5ConfThres) {
            float class_socre = box_score * pdata[5];
            if (class_socre > YOLOv5ConfThres) {
                float cx = pdata[0] * ratiow;    ///cx
                float cy = pdata[1] * ratioh;    ///cy
                float  w = pdata[2] * ratiow;    ///w
                float  h = pdata[3] * ratioh;    ///h

                float xmin = cx - 0.5 * w;
                float ymin = cy - 0.5 * h;
                float xmax = cx + 0.5 * w;
                float ymax = cy + 0.5 * h;

                generate_boxes.push_back(BoxInfo{ xmin, ymin, xmax, ymax, class_socre, ClassIndex});
            }
        }
        pdata += nout;
    }

    nms(generate_boxes,YOLOv5IouThres);

    for (size_t n = 0; n < generate_boxes.size(); n++) {
        int xmin = int(generate_boxes[n].x1);
        int ymin = int(generate_boxes[n].y1);
        int xmax = int(generate_boxes[n].x2);
        int ymax = int(generate_boxes[n].y2);

        // 保证 xmin、ymin、xmax、ymax 在图像边界内
        xmin = std::max(0, std::min(xmin, carImage.cols - 1));
        ymin = std::max(0, std::min(ymin, carImage.rows - 1));
        xmax = std::max(0, std::min(xmax, carImage.cols - 1));
        ymax = std::max(0, std::min(ymax, carImage.rows - 1));

        //rectangle(carImage, cv::Point(xmin, ymin), cv::Point(int(generate_boxes[n].x2), int(generate_boxes[n].y2)), cv::Scalar(0, 0, 255), 1);

        //std::string label1 = cv::format("%.2f", classScore*10);
        //std::string label2 =  class_labels[generate_boxes[n].classIndex];
        //putText(carImage, label1, cv::Point(xmin, ymin - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 0.1);
        //putText(carImage, label2, cv::Point(xmin, ymin + 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 0.1);

        cv::Rect roi_rect(xmin, ymin, xmax - xmin, ymax - ymin);

        armor_boxes.push_back(Detection{roi_rect, generate_boxes[n].score, generate_boxes[n].classIndex});

    }
    return armor_boxes;
}

// 将检测类别加载入容器
std::vector<std::string> YOLOv5Detector::LoadNames(const std::string &YOLOv5NamePath)
{
    // 加载目标检测类别
    std::vector<std::string> classNames;
    std::ifstream infile(YOLOv5NamePath);
    if (infile.is_open()) {
        std::string line;
        while (getline(infile, line)) {
            classNames.emplace_back(line);
        }
        infile.close();
    } else {
        spdlog::error("Error loading the class names");
    }

    return classNames;
}

// 根据检测结果的得分大小对结果进行排序
std::vector<Detection> YOLOv5Detector::SortResult(std::vector<Detection> &result)
{
    // 新建容器将结果的得分载入
    std::vector<float> armorScore;
    for (auto &i : result)
    {
        armorScore.emplace_back(i.score);
    }

    // 根据结果的得分大小对结果进行排序
    std::vector<Detection> sortArmorResult;
    while (!result.empty())
    {
        auto maxValue = std::max_element(armorScore.begin(), armorScore.end());
        auto maxValueIndex = std::distance(armorScore.begin(), maxValue);
        sortArmorResult.emplace_back(result[maxValueIndex]);
        result.erase(result.begin() + maxValueIndex);
    }
    return sortArmorResult;
}

DetectCarInfo YOLOv5Detector::CarInfo(const std::vector<Detection> &carResult,
                                                     const std::vector<Detection> &sortArmorResult,
                                                     const uint8_t &i)
{
    // 初始化车辆对象
    DetectCarInfo detectCarInfo;

    // 初始化装甲板对象
    DetectArmorsInfo detectArmorInfo;

    // 将一帧图像中单个车辆信息载入车辆对象
    if (!sortArmorResult.empty()) {
        // 车辆编号为检测分数最高的装甲板编号
        detectCarInfo.carClass = sortArmorResult[0].classIndex;
        // 车辆中心点的x轴坐标为检测车辆矩形框中心点x轴坐标
        detectCarInfo.carX = carResult[i].bbox.x;
        // 车辆中心点的y轴坐标为检测车辆矩形框中心点y轴坐标
        detectCarInfo.carY = carResult[i].bbox.y;
        // 车辆的宽度为检测车辆矩形框的宽度
        detectCarInfo.carWidth = carResult[i].bbox.width;
        // 车辆的高度为检测车辆矩形框的高度
        detectCarInfo.carHeight = carResult[i].bbox.height;
    }

    for (auto const &j : sortArmorResult)
    {
        if ((j.bbox.width > 0) && (j.bbox.height > 0) && (j.bbox.width / j.bbox.height < 20) &&
            (j.bbox.height / j.bbox.width < 20)) {

            // 装甲板编号为检测装甲板编号
            detectArmorInfo.armorClass = j.classIndex;
            // 装甲板中心点的x轴坐标为检测装甲板矩形框中心点x轴坐标
            detectArmorInfo.armorX = j.bbox.x + carResult[i].bbox.x;
            // 装甲板中心点的y轴坐标为检测装甲板矩形框中心点y轴坐标
            detectArmorInfo.armorY = j.bbox.y + carResult[i].bbox.y;
            // 装甲板的宽度为检测装甲板矩形框的宽度
            detectArmorInfo.armorWidth = j.bbox.width;
            // 装甲板的高度为检测装甲板矩形框的高度
            detectArmorInfo.armorHeight = j.bbox.height;
            // 装甲板信息载入车辆对象的装甲板容器
            detectCarInfo.detectArmorsInfo.emplace_back(detectArmorInfo);

        }
    }

    return detectCarInfo;
}


////////////////////////////////////////////   private   //////////////////////////////////////////////////////


void YOLOv5Detector::normalize_(cv::Mat img) {
    //    img.convertTo(img, CV_32F);
    int row = img.rows;
    int col = img.cols;
    this->input_image_.resize(row * col * img.channels());
    for (int c = 0; c < 3; c++) {
        for (int i = 0; i < row; i++) {
            for (int j = 0; j < col; j++) {
                float pix = img.ptr<uchar>(i)[j * 3 + 2 - c];
                this->input_image_[c * row * col + i * col + j] = pix / 255.0;
            }
        }
    }
}

void YOLOv5Detector::nms(std::vector<BoxInfo>& input_boxes, float YOLOv5IouThres) {
    sort(input_boxes.begin(), input_boxes.end(), [](BoxInfo a, BoxInfo b) { return a.score > b.score; });
    std::vector<float> vArea(input_boxes.size());
    for (int i = 0; i < int(input_boxes.size()); ++i) {
        vArea[i] = (input_boxes.at(i).x2 - input_boxes.at(i).x1 + 1)
                   * (input_boxes.at(i).y2 - input_boxes.at(i).y1 + 1);
    }

    std::vector<bool> isSuppressed(input_boxes.size(), false);
    for (int i = 0; i < int(input_boxes.size()); ++i) {
        if (isSuppressed[i]) { continue; }
        for (int j = i + 1; j < int(input_boxes.size()); ++j) {
            if (isSuppressed[j]) { continue; }
            float xx1 = (std::max)(input_boxes[i].x1, input_boxes[j].x1);
            float yy1 = (std::max)(input_boxes[i].y1, input_boxes[j].y1);
            float xx2 = (std::min)(input_boxes[i].x2, input_boxes[j].x2);
            float yy2 = (std::min)(input_boxes[i].y2, input_boxes[j].y2);

            float w = (std::max)(float(0), xx2 - xx1 + 1);
            float h = (std::max)(float(0), yy2 - yy1 + 1);
            float inter = w * h;
            float ovr = inter / (vArea[i] + vArea[j] - inter);

            if (ovr >= YOLOv5IouThres) {
                isSuppressed[j] = true;
            }
        }
    }
    // return post_nms;
    int idx_t = 0;
    input_boxes.erase(remove_if(input_boxes.begin(), input_boxes.end(), [&idx_t, &isSuppressed](const BoxInfo& f) { return isSuppressed[idx_t++]; }), input_boxes.end());
}

/*std::vector<Detection> YOLOv5Detector::ScaleCoordinates(const at::TensorAccessor<float, 2> &data,
                                                        float padWidth, float padHeight, float scale,
                                                        const cv::Size &imageShape)
{
    auto clip = [](float n, float lower, float upper)
    {
        return std::max(lower, std::min(n, upper));
    };

    std::vector<Detection> detections;
    for (int i = 0; i < data.size(0); i++)
    {
        Detection detection;
        float x1 = (data[i][Det::tl_x] - padWidth) / scale;  // x padding
        float y1 = (data[i][Det::tl_y] - padHeight) / scale;  // y padding
        float x2 = (data[i][Det::br_x] - padWidth) / scale;  // x padding
        float y2 = (data[i][Det::br_y] - padHeight) / scale;  // y padding
//        std::cout << " x1: " << x1 << std::endl;
        x1 = clip(x1, 0, imageShape.width);
        y1 = clip(y1, 0, imageShape.height);
        x2 = clip(x2, 0, imageShape.width);
        y2 = clip(y2, 0, imageShape.height);
//        std::cout <<" x1_: " << x1 << std::endl;
        detection.bbox = cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2));
        detection.score = data[i][Det::score];
        detection.classIndex = data[i][Det::classIndex];
        detections.emplace_back(detection);
    }
    return detections;
}*/
