//
// Created by plutoli on 2021/8/14.
//

#include <string>
#include <cstring>
#include "huaray_camera_data.h"

int main(int argc, char *argv[])
{
    // 读取图片
    cv::Mat image = imread("/home/zhangtianyi/图片/5.png",cv::IMREAD_COLOR);
    if (image.empty())
    {
        std::cout << "Error. Could not read picture." << std::endl;
        return -1;
    }

    cv::imshow("image1", image);
    cv::waitKey(0);

    // 序列化华睿相机数据帧
    HuarayCameraData data1;
    data1.Image = image;
    data1.Timestamp = 12345;
    unsigned int dataSize = data1.GetSize();
    unsigned char byteArray[dataSize];
    data1.Serialize(byteArray);

    // 加载华睿相机数据帧
    HuarayCameraData data2 = HuarayCameraData::LoadFromBytes(byteArray);
    cv::imshow("image2", data2.Image);
    cv::waitKey(0);

    return 0;
}
