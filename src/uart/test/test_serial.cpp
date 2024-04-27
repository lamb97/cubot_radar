////
//// Created by zhangtianyi on 202/4/2.
////
//#include <iostream>
//#include <queue>
//#include <memory>
//#include <qt5/QtGui/qopengles2ext.h>
//#include "uart/serial_port_param.h"
//#include "uart/serial_port.h"
//
//
//unsigned int GetdataSize()
//{
//    //初始化打包之后的字节总长度
//    unsigned int totalSize = 0;
//
//    //数据帧的帧头占用1个字节
//    unsigned int headerSize = 1;
//    totalSize +=headerSize;
//
//    //数据帧的ID占用一个字节
//    unsigned int idSize = 2;
//    totalSize += idSize;
//
//    //Pitch轴的偏转角度占用2个字节
//    unsigned int pitchSize = 2;
//    totalSize +=pitchSize;
//
//    //击打目标点到枪口的距离占用1个字节
//    unsigned int distanceSize = 1;
//    totalSize +=distanceSize;
//
//    //数据帧的帧尾占用1个字节
//    unsigned int tailSize = 1;
//    totalSize += tailSize;
//
//    //返回数据长度
//    return totalSize;
//
//}
//unsigned int commandFrameSize = GetdataSize();
////unsigned char commandFrame[commandFrameSize];
//
//
////将数据帧发送给串口
//void TransmitFrame(const unsigned int &size, const unsigned char *frame)
//{
//    SerialPort.write(size, frame);
//}
//
//
//int main(int, char* [])
//{
//    SerialPortParam serialPortParam;
//    std::string yameFile = "config/sentry/2021-09-05/param/serial_port_param.yaml";
//    SerialPortParam::LoadFromYamlFile(yameFile, &serialPortParam);
//
//    SerialPort serialPort;
//    serialPort.SetParam(serialPortParam);
//    serialPort.Init();
//    serialPort.Open();
//
//
//    unsigned char buffer[100];
//    memset(buffer, 0, 100);
//    unsigned int count = 0;
//
//    while(true)
//    {
//        serialPort.Write(23,(const unsigned char *)data.c_str())
//    }
//}
