#ifndef SRC_UTILS_H
#define SRC_UTILS_H

enum Det {
    tl_x = 0,
    tl_y = 1,
    br_x = 2,
    br_y = 3,
    score = 4,
    classIndex = 5
};

struct Detection {
    cv::Rect bbox;
    float score;
    int classIndex;
};

//新的结果
typedef struct BoxInfo
{
    //左上右下位置坐标
    float x1;
    float y1;
    float x2;
    float y2;
    float score;
    int classIndex;
} BoxInfo;


/*typedef struct DetectBox {
    DetectBox(float x1=0, float y1=0, float x2=0, float y2=0,
              float confidence=0, float classID=-1, float trackID=-1) {
        this->x1 = x1;
        this->y1 = y1;
        this->x2 = x2;
        this->y2 = y2;
        this->confidence = confidence;
        this->classID = classID;
        this->trackID = trackID;
    }
    float x1, y1, x2, y2;
    float confidence;
    float classID;
    float trackID;
} DetectBox;*/


#endif //SRC_UTILS_H