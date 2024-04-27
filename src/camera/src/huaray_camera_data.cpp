//
// Created by plutoli on 2022/4/5.
//

#include "huaray_camera_data.h"

// ******************************  HuarayCameraData类的公有函数  ******************************

// 构造函数
HuarayCameraData::HuarayCameraData():
    Timestamp(0),
    Image()
{
}
// 获取华睿相机数据帧序列化之后的字节长度
unsigned int HuarayCameraData::GetSize() const
{
    // 初始化序列化之后的字节总长度
    unsigned int totalSize = 0;

    // 计算数据帧中每个数据的长度
    unsigned int timestampSize = sizeof(Timestamp);
    totalSize += timestampSize;
    unsigned int rowsSize = sizeof(Image.rows);
    totalSize += rowsSize;
    unsigned int colsSize = sizeof(Image.cols);
    totalSize += colsSize;
    unsigned int typeSize = sizeof(Image.type());
    totalSize += typeSize;
    unsigned int imageSize = Image.total() * Image.elemSize();
    totalSize += imageSize;

    // 返回数据长度
    return totalSize;
}

// 序列化华睿相机数据帧
void HuarayCameraData::Serialize(unsigned char *byteArray) const
{
    // 初始化字节索引
    unsigned int index = 0;

    // 序列化Timestamp
    unsigned int timestampSize = sizeof(Timestamp);
    memcpy(byteArray + index, &Timestamp, timestampSize);
    index += timestampSize;

    // 序列化Image.rows
    int rows = Image.rows;
    unsigned int rowsSize = sizeof(rows);
    memcpy(byteArray + index, &rows, rowsSize);
    index += rowsSize;

    // 序列化Image.cols
    int cols = Image.cols;
    unsigned int colsSize = sizeof(cols);
    memcpy(byteArray + index, &cols, colsSize);
    index += colsSize;

    // 序列化Image.type()
    int type = Image.type();
    unsigned int typeSize = sizeof(type);
    memcpy(byteArray + index, &type, typeSize);
    index += typeSize;

    // 序列化Image
    unsigned int imageSize = Image.total() * Image.elemSize();
    memcpy(byteArray + index, Image.data, imageSize);
}

// 加载相机数据帧
HuarayCameraData HuarayCameraData::LoadFromBytes(const unsigned char *byteArray)
{
    // 初始化字节索引
    size_t index = 0;

    // 加载时间戳
    uint64_t timestamp = 0;
    unsigned int timestampSize = sizeof(timestamp);
    memcpy(&timestamp, byteArray + index, timestampSize);
    index += timestampSize;

    // 加载图像的行数
    int rows = 0;
    unsigned int rowsSize = sizeof(rows);
    memcpy(&rows, byteArray + index, rowsSize);
    index += rowsSize;

    // 加载图像的列数
    int cols = 0;
    unsigned int colsSize = sizeof(cols);
    memcpy(&cols, byteArray + index, colsSize);
    index += colsSize;

    // 加载图像的类型
    int type = 0;
    unsigned int typeSize = sizeof(type);
    memcpy(&type, byteArray + index, typeSize);
    index += typeSize;

    // 加载图像数据
    cv::Mat image(rows, cols, type);
    unsigned int imageSize = image.total() * image.elemSize();
    memcpy(image.data, byteArray + index, imageSize);

    // 初始化华睿图像数据帧
    HuarayCameraData huarayCameraData;
    huarayCameraData.Image = image;
    huarayCameraData.Timestamp = timestamp;

    // 返回华睿图像数据帧
    return huarayCameraData;
}