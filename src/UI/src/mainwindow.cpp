// You may need to build the project (run Qt uic code generator) to get "ui_mainwindow.h" resolved
#include "mainwindow.h"
#include "ui_mainwindow.h"


mainwindow::mainwindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::mainwindow)
{
    ui->setupUi(this);

    //timer = new QTimer(this);
//    connect(ui->pushButton_Show, SIGNAL(clicked()), this, SLOT(on_pushButton_Show_clicked()));
    connect(ui->pushButton_Begin, SIGNAL(clicked()), this, SLOT(on_pushButton_Begin_clicked()));
    //connect(&qnode, SIGNAL(imageSignal(cv::Mat)), this, SLOT(showImage(cv::Mat)));
    connect(ui->pushButton_Cancel, SIGNAL(clicked()), this, SLOT(on_pushButton_Cancel_clicked()));
    connect(ui->pushButton_Exit, SIGNAL(clicked()), this, SLOT(close()));
}

mainwindow::~mainwindow()
{
    delete ui;
}

//void mainwindow::readImage()
//{
//    if(capture.isOpened())
//    {
//        capture >> srcImage;
//        if(!srcImage.empty())
//        {
//            // 图像的BGR格式转换为RGB格式
//            cvtColor(srcImage, srcImage, COLOR_BGR2RGB);
//            cv::resize(srcImage, srcImage, Size(640, 512));
//            // 将抓取到的帧，转换为QImage格式
//            QImage image((const uchar*)srcImage.data, srcImage.cols, srcImage.rows, QImage::Format_RGB888);
//            ui->label_video->setPixmap(QPixmap::fromImage(image));          // 将图像显示到label上
//            ui->label_video->resize( ui->label_video->pixmap()->size());   // 将label控件resize到图像的尺寸
//        }
//    }
//
////    if(camera.IsOpened())
////    {
////        HuarayCameraData data;
////        camera.GetData(&data);
////        srcImage = data.Image;
////
////        cvtColor(srcImage, dstImage, COLOR_BGR2RGB);  // BGR格式图像转换为RGB格式
////        cv::resize(dstImage, dstImage, Size(640, 512));
////        // 将抓取到的帧，转换为QImage格式
////        QImage image((const uchar*)dstImage.data, dstImage.cols, dstImage.rows, QImage::Format_RGB888);
////        ui->label_video->setPixmap(QPixmap::fromImage(image));    // 将图片显示到label上
////        ui->label_video->resize(ui->label_video->pixmap()->size());    // 将label控件resize到frame的尺寸
////        ui->label_video->show();
////    }
//}
//
//void mainwindow::on_pushButton_Begin_clicked()
//{
//    capture.open("/media/zy/Data/radar_ws/src/cubot_radar/data/video/2021-06-02/test.avi");
//    timer->start(25); // 开始计时，每隔25ms更新一次，超时则发出timeout()信号
//
////    // 读取华睿相机参数
////    std::string yamlFile = "/src/cubot_radar/config/param/huaray_camera_param.yaml";
////    // 初始化华睿相机
////    HuarayCameraParam::LoadFromYamlFile(yamlFile, &param);
////    camera.SetParam(param);
////    camera.Init();
////    camera.WriteHardwareParamToDevice(param.HardwareParams[0]);
////    camera.Open();
////    timer->start(25);  // 开始计时,每25ms更新一次
//}
// ***************************-----------------************************************************

// 订阅图像回调函数
void mainwindow::imageCallback(const cubot_radar::CarsMsg::ConstPtr & carsMsg) {
    boost::shared_ptr<cv_bridge::CvImage> getImage = cv_bridge::toCvCopy(carsMsg->image, "bgr8");
    srcImage = getImage->image;
    //Q_EMIT imageSignal(srcImage);
//    for (int i = 0; i < carsMsg->carNum; i++) {
//        cv::Rect carBox((int) carsMsg->carsInfo[i].carX,
//                        (int) carsMsg->carsInfo[i].carY,
//                        (int) carsMsg->carsInfo[i].carWidth,
//                        (int) carsMsg->carsInfo[i].carHeight);
//        // 添加车辆类别显示信息
//        cv::putText(srcImage, std::to_string(carsMsg->carsInfo[i].carClass),
//                    cv::Point(carBox.tl().x, carBox.tl().y - 5),
//                    cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 255, 255), 1);

//        for (int j = 0; j < carsMsg->carsInfo[i].armorNum; j++) {
//            cv::Rect armorBox((int) carsMsg->carsInfo[i].armorsInfo[j].armorX,
//                              (int) carsMsg->carsInfo[i].armorsInfo[j].armorY,
//                              (int) carsMsg->carsInfo[i].armorsInfo[j].armorWidth,
//                              (int) carsMsg->carsInfo[i].armorsInfo[j].armorHeight);
//
//            // 标签为0~5对应红色装甲板
//            if (carsMsg->carsInfo[i].armorsInfo[j].armorClass >= 0 &&
//                carsMsg->carsInfo[i].armorsInfo[j].armorClass < 5) {
//                // 画框显示装甲板区域
//               // cv::rectangle(srcImage, armorBox, cv::Scalar(0, 0, 255), 2);
//
//                // 画框显示车辆区域
//                cv::rectangle(srcImage, carBox, cv::Scalar(0, 0, 255), 2);
//            }

//           //  标签为5~9对应蓝色装甲板
//            if (carsMsg->carsInfo[i].armorsInfo[j].armorClass >= 5 &&
//                carsMsg->carsInfo[i].armorsInfo[j].armorClass < 10) {
//                // 画框显示装甲板区域
//                cv::rectangle(srcImage, armorBox, cv::Scalar(255, 0, 0), 2);
//
//                 //画框显示车辆区域
//                cv::rectangle(srcImage, carBox, cv::Scalar(255, 0, 0), 2);
//            }
//        }
//
//    }
    display(srcImage);




//    picture = cv::imread("/home/zhangtianyi/test_ws/src/cubot_radar/data/data1/picture/map.jpg");
//    //cv::flip(picture, picture, -1); //旋转180°
//
//// 定义轮廓区域1
//    std::vector<cv::Point2i> contour1;
//    contour1.emplace_back(cv::Point2i(24,190));
//    contour1.emplace_back(cv::Point2i(87, 190));
//    contour1.emplace_back(cv::Point2i(87, 284));
//    contour1.emplace_back(cv::Point2i(24, 284));
//    std::vector<std::vector<cv::Point2i>> contours1;
//    contours1.push_back(contour1);
//    // 定义轮廓区域2
//    std::vector<cv::Point2i> contour2;
//    contour2.emplace_back(cv::Point2i(54,89));
//    contour2.emplace_back(cv::Point2i(105, 89));
//    contour2.emplace_back(cv::Point2i(105, 122));
//    contour2.emplace_back(cv::Point2i(54, 158));
//    std::vector<std::vector<cv::Point2i>> contours2;
//    contours2.push_back(contour2);
//    // 定义轮廓区域3
//    std::vector<cv::Point2i> contour3;
//    contour3.emplace_back(cv::Point2i(1,10));
//    contour3.emplace_back(cv::Point2i(200, 10));
//    contour3.emplace_back(cv::Point2i(200, 56));
//    contour3.emplace_back(cv::Point2i(1, 56));
//    std::vector<std::vector<cv::Point2i>> contours3;
//    contours3.push_back(contour3);
//    // 定义轮廓区域4
//    std::vector<cv::Point2i> contour4;
//    contour4.emplace_back(cv::Point2i(198,89));
//    contour4.emplace_back(cv::Point2i(268, 89));
//    contour4.emplace_back(cv::Point2i(268, 170));
//    contour4.emplace_back(cv::Point2i(198, 124));
//    std::vector<std::vector<cv::Point2i>> contours4;
//    contours4.push_back(contour4);
    // 绘制多边形
//    cv::polylines(picture, contours1, true, Scalar(0, 255, 0), 2, 8);
//    cv::polylines(picture, contours2, true, Scalar(0, 255, 0), 2, 8);
//    cv::polylines(picture, contours3, true, Scalar(0, 255, 0), 2, 8);
//    cv::polylines(picture, contours4, true, Scalar(0, 255, 0), 2, 8);
//    // BGR格式图片转换为RGB格式图片
//    //cv::cvtColor(picture, picture, COLOR_BGR2RGB);
//    // 转换成QImage
//    QImage pic = QImage((const unsigned char *) (picture.data), picture.cols, picture.rows, QImage::Format_RGB888);
//    // 自适应控件大小并显示图片
//    ui->label_picture->setFixedSize(300, 560);
//    ui->label_picture->setPixmap(QPixmap::fromImage(pic.scaled(ui->label_picture->size(), Qt::KeepAspectRatio)));
//    //cout<< "picture size: " << ui->label_picture->width() << " " << ui->label_picture->height() <<endl;
//    cv::waitKey(1);



    // 将rosPCL数据转换为PCL点云数据
//    pcl::PCLPointCloud2 PCLCloud;
//    pcl_conversions::toPCL(*rosPCLMsg, PCLCloud);
    //std::vector<cv::Point3f> targetPoints;

    // 将PCL点云数据转换为有点坐标的PCLXYZ数据
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::fromPCLPointCloud2(PCLCloud, *cloud);

//    for(auto& point : cloud->points)
//    {
//        cv::Point3f target_3d = cv::Point3f(point.x, point.y, point.z);
//        targetPoints.emplace_back(target_3d);
//    }

//    for(int i=0; i<targetPoints.size(); i++)
//    {
//        cout<< "target: " << targetPoints[i] <<endl;
//    }

}


// 订阅图像回调函数
void mainwindow::image2Callback(const cubot_radar::CarsMsg::ConstPtr & carsMsg) {
    boost::shared_ptr<cv_bridge::CvImage> getImage = cv_bridge::toCvCopy(carsMsg->image, "bgr8");
    srcImage = getImage->image;
    //Q_EMIT imageSignal(srcImage);
//    for (int i = 0; i < carsMsg->carNum; i++) {
//        cv::Rect carBox((int) carsMsg->carsInfo[i].carX,
//                        (int) carsMsg->carsInfo[i].carY,
//                        (int) carsMsg->carsInfo[i].carWidth,
//                        (int) carsMsg->carsInfo[i].carHeight);
//        // 添加车辆类别显示信息
//        cv::putText(srcImage, std::to_string(carsMsg->carsInfo[i].carClass),
//                    cv::Point(carBox.tl().x, carBox.tl().y - 5),
//                    cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 255, 255), 1);
//
//        for (int j = 0; j < carsMsg->carsInfo[i].armorNum; j++) {
//            cv::Rect armorBox((int) carsMsg->carsInfo[i].armorsInfo[j].armorX,
//                              (int) carsMsg->carsInfo[i].armorsInfo[j].armorY,
//                              (int) carsMsg->carsInfo[i].armorsInfo[j].armorWidth,
//                              (int) carsMsg->carsInfo[i].armorsInfo[j].armorHeight);
//
//            // 标签为0~5对应红色装甲板
//            if (carsMsg->carsInfo[i].armorsInfo[j].armorClass >= 0 &&
//                carsMsg->carsInfo[i].armorsInfo[j].armorClass < 5) {
//                // 画框显示装甲板区域
//                //cv::rectangle(srcImage, armorBox, cv::Scalar(0, 0, 255), 2);
//
//                // 画框显示车辆区域
//                cv::rectangle(srcImage, carBox, cv::Scalar(0, 0, 255), 2);
//            }

//            // 标签为5~9对应蓝色装甲板
//            if (carsMsg->carsInfo[i].armorsInfo[j].armorClass >= 5 &&
//                carsMsg->carsInfo[i].armorsInfo[j].armorClass < 10) {
//                // 画框显示装甲板区域
//               cv::rectangle(srcImage, armorBox, cv::Scalar(255, 0, 0), 2);
//
//                // 画框显示车辆区域
//                cv::rectangle(srcImage, carBox, cv::Scalar(255, 0, 0), 2);
//            }
//        }
//
//    }
    display2(srcImage);

//        picture = cv::imread("/home/zhangtianyi/test_ws/src/cubot_radar/data/data1/picture/map.jpg");
//        //cv::flip(picture, picture, -1); //旋转180°
//
//
//        // BGR格式图片转换为RGB格式图片
//        //cv::cvtColor(picture, picture, COLOR_BGR2RGB);
//        // 转换成QImage
//        QImage pic = QImage((const unsigned char*)(picture.data), picture.cols, picture.rows, QImage::Format_RGB888);
//        // 自适应控件大小并显示图片
//        ui->label_picture->setFixedSize(300,560);
//        ui->label_picture->setPixmap(QPixmap::fromImage(pic.scaled(ui->label_picture->size(),Qt::KeepAspectRatio)));
//        //cout<< "picture size: " << ui->label_picture->width() << " " << ui->label_picture->height() <<endl;
//        cv::waitKey(1);



    // 将rosPCL数据转换为PCL点云数据
//    pcl::PCLPointCloud2 PCLCloud;
//    pcl_conversions::toPCL(*rosPCLMsg, PCLCloud);
    //std::vector<cv::Point3f> targetPoints;

    // 将PCL点云数据转换为有点坐标的PCLXYZ数据
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::fromPCLPointCloud2(PCLCloud, *cloud);

//    for(auto& point : cloud->points)
//    {
//        cv::Point3f target_3d = cv::Point3f(point.x, point.y, point.z);
//        targetPoints.emplace_back(target_3d);
//    }

//    for(int i=0; i<targetPoints.size(); i++)
//    {
//        cout<< "target: " << targetPoints[i] <<endl;
//    }
}

// 订阅图像回调函数
void mainwindow::image3Callback(const cubot_radar::CarsMsg::ConstPtr & carsMsg) {
    boost::shared_ptr<cv_bridge::CvImage> getImage = cv_bridge::toCvCopy(carsMsg->image, "bgr8");
    srcImage = getImage->image;
    //Q_EMIT imageSignal(srcImage);
//    for (int i = 0; i < carsMsg->carNum; i++) {
//        cv::Rect carBox((int) carsMsg->carsInfo[i].carX,
//                        (int) carsMsg->carsInfo[i].carY,
//                        (int) carsMsg->carsInfo[i].carWidth,
//                        (int) carsMsg->carsInfo[i].carHeight);
//        // 添加车辆类别显示信息
//        cv::putText(srcImage, std::to_string(carsMsg->carsInfo[i].carClass),
//                    cv::Point(carBox.tl().x, carBox.tl().y - 5),
//                    cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 255, 255), 1);
//
//        for (int j = 0; j < carsMsg->carsInfo[i].armorNum; j++) {
//            cv::Rect armorBox((int) carsMsg->carsInfo[i].armorsInfo[j].armorX,
//                              (int) carsMsg->carsInfo[i].armorsInfo[j].armorY,
//                              (int) carsMsg->carsInfo[i].armorsInfo[j].armorWidth,
//                              (int) carsMsg->carsInfo[i].armorsInfo[j].armorHeight);
//
//            // 标签为0~5对应红色装甲板
//            if (carsMsg->carsInfo[i].armorsInfo[j].armorClass >= 0 &&
//                carsMsg->carsInfo[i].armorsInfo[j].armorClass < 5) {
//                // 画框显示装甲板区域
//                //cv::rectangle(srcImage, armorBox, cv::Scalar(0, 0, 255), 2);
//
//                // 画框显示车辆区域
//                cv::rectangle(srcImage, carBox, cv::Scalar(0, 0, 255), 2);
//            }

//            // 标签为5~9对应蓝色装甲板
//            if (carsMsg->carsInfo[i].armorsInfo[j].armorClass >= 5 &&
//                carsMsg->carsInfo[i].armorsInfo[j].armorClass < 10) {
//                // 画框显示装甲板区域
//               cv::rectangle(srcImage, armorBox, cv::Scalar(255, 0, 0), 2);
//
//                // 画框显示车辆区域
//                cv::rectangle(srcImage, carBox, cv::Scalar(255, 0, 0), 2);
//            }
//        }
//
//    }
    display3(srcImage);

//        picture = cv::imread("/home/zhangtianyi/test_ws/src/cubot_radar/data/data1/picture/map.jpg");
//        //cv::flip(picture, picture, -1); //旋转180°
//
//
//        // BGR格式图片转换为RGB格式图片
//        //cv::cvtColor(picture, picture, COLOR_BGR2RGB);
//        // 转换成QImage
//        QImage pic = QImage((const unsigned char*)(picture.data), picture.cols, picture.rows, QImage::Format_RGB888);
//        // 自适应控件大小并显示图片
//        ui->label_picture->setFixedSize(300,560);
//        ui->label_picture->setPixmap(QPixmap::fromImage(pic.scaled(ui->label_picture->size(),Qt::KeepAspectRatio)));
//        //cout<< "picture size: " << ui->label_picture->width() << " " << ui->label_picture->height() <<endl;
//        cv::waitKey(1);



    // 将rosPCL数据转换为PCL点云数据
//    pcl::PCLPointCloud2 PCLCloud;
//    pcl_conversions::toPCL(*rosPCLMsg, PCLCloud);
    //std::vector<cv::Point3f> targetPoints;

    // 将PCL点云数据转换为有点坐标的PCLXYZ数据
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::fromPCLPointCloud2(PCLCloud, *cloud);

//    for(auto& point : cloud->points)
//    {
//        cv::Point3f target_3d = cv::Point3f(point.x, point.y, point.z);
//        targetPoints.emplace_back(target_3d);
//    }

//    for(int i=0; i<targetPoints.size(); i++)
//    {
//        cout<< "target: " << targetPoints[i] <<endl;
//    }
}


// 显示图像
void mainwindow::display(cv::Mat dstIamge)
{
    // BGR格式图像转换为RGB格式
    cv::cvtColor(dstIamge, dstIamge, COLOR_BGR2RGB);
    cv::resize(dstIamge, dstIamge, Size(640, 512));
    // 将抓取到的帧转换为QImage格式
    QImage qimage((const uchar*)dstIamge.data, dstIamge.cols, dstIamge.rows, QImage::Format_RGB888);
    qimage_mutex_.lock();
    qimage_ = qimage.copy();
    // 将图像显示到label上
    ui->label_video->setPixmap(QPixmap::fromImage(qimage));


    // 将label控件resize到图像尺寸
    //ui->label_video->resize(ui->label_video->pixmap()->size());
    ui->label_video->setPixmap(QPixmap::fromImage(qimage.scaled(ui->label_video->size(),Qt::KeepAspectRatio)));
    //cout<< "video size " << ui->label_video->width() << " " << ui->label_video->height() <<endl;
    qimage_mutex_.unlock();
    //cv::imshow("subImage", srcImage);
    cv::waitKey(1);
}


// 显示图像
void mainwindow::display2(cv::Mat dstIamge)
{
    // BGR格式图像转换为RGB格式

    cv::cvtColor(dstIamge, dstIamge, COLOR_BGR2RGB);
    cv::resize(dstIamge, dstIamge, Size(640, 512));
    // 将抓取到的帧转换为QImage格式
    QImage qimage((const uchar*)dstIamge.data, dstIamge.cols, dstIamge.rows, QImage::Format_RGB888);
    qimage_mutex_.lock();
    qimage_ = qimage.copy();
    // 将图像显示到label上
    ui->label_video2->setPixmap(QPixmap::fromImage(qimage));
    // 将label控件resize到图像尺寸
    //ui->label_video->resize(ui->label_video->pixmap()->size());
    ui->label_video2->setPixmap(QPixmap::fromImage(qimage.scaled(ui->label_video2->size(),Qt::KeepAspectRatio)));
   // cout<< "video size " << ui->label_video->width() << " " << ui->label_video->height() <<endl;
    qimage_mutex_.unlock();
    //cv::imshow("subImage", srcImage);
    cv::waitKey(1);
}

void mainwindow::display3(cv::Mat dstIamge)
{
    // BGR格式图像转换为RGB格式
    cv::cvtColor(dstIamge, dstIamge, COLOR_BGR2RGB);
    cv::resize(dstIamge, dstIamge, Size(360, 288));
    // 将抓取到的帧转换为QImage格式
    QImage qimage((const uchar*)dstIamge.data, dstIamge.cols, dstIamge.rows, QImage::Format_RGB888);
    qimage_mutex_.lock();
    qimage_ = qimage.copy();
    // 将图像显示到label上
    ui->label_video3->setPixmap(QPixmap::fromImage(qimage));
    // 将label控件resize到图像尺寸
    //ui->label_video->resize(ui->label_video->pixmap()->size());
    ui->label_video3->setPixmap(QPixmap::fromImage(qimage.scaled(ui->label_video3->size(),Qt::KeepAspectRatio)));
    // cout<< "video size " << ui->label_video->width() << " " << ui->label_video->height() <<endl;
    qimage_mutex_.unlock();
    //cv::imshow("subImage", srcImage);
    cv::waitKey(1);
}



//目标点
//void mainwindow::Callback(const cubot_radar::CarsMsg::ConstPtr &carsMsg)
//{
//
//    // 定义轮廓区域1
//    std::vector<cv::Point2i> contour1;
//    contour1.emplace_back(cv::Point2i(24,190));
//    contour1.emplace_back(cv::Point2i(87, 190));
//    contour1.emplace_back(cv::Point2i(87, 284));
//    contour1.emplace_back(cv::Point2i(24, 284));
//    std::vector<std::vector<cv::Point2i>> contours1;
//    contours1.push_back(contour1);
//    // 定义轮廓区域2
//    std::vector<cv::Point2i> contour2;
//    contour2.emplace_back(cv::Point2i(54,89));
//    contour2.emplace_back(cv::Point2i(105, 89));
//    contour2.emplace_back(cv::Point2i(105, 122));
//    contour2.emplace_back(cv::Point2i(54, 158));
//    std::vector<std::vector<cv::Point2i>> contours2;
//    contours2.push_back(contour2);
//    // 定义轮廓区域3
//    std::vector<cv::Point2i> contour3;
//    contour3.emplace_back(cv::Point2i(1,10));
//    contour3.emplace_back(cv::Point2i(200, 10));
//    contour3.emplace_back(cv::Point2i(200, 56));
//    contour3.emplace_back(cv::Point2i(1, 56));
//    std::vector<std::vector<cv::Point2i>> contours3;
//    contours3.push_back(contour3);
//    // 定义轮廓区域4
//    std::vector<cv::Point2i> contour4;
//    contour4.emplace_back(cv::Point2i(198,89));
//    contour4.emplace_back(cv::Point2i(268, 89));
//    contour4.emplace_back(cv::Point2i(268, 170));
//    contour4.emplace_back(cv::Point2i(198, 124));
//    std::vector<std::vector<cv::Point2i>> contours4;
//    contours4.push_back(contour4);
//    // 绘制多边形
//    cv::polylines(picture, contours1, true, Scalar(0, 255, 0), 2, 8);
//    cv::polylines(picture, contours2, true, Scalar(0, 255, 0), 2, 8);
//    cv::polylines(picture, contours3, true, Scalar(0, 255, 0), 2, 8);
//    cv::polylines(picture, contours4, true, Scalar(0, 255, 0), 2, 8);
//
//    for(int i = 0; i<carsMsg->carNum; i++)
//    {
//        cv::Point2i target_2d = cv::Point2i(abs(carsMsg->carsInfo[i].carWorldY * 20), carsMsg->carsInfo[i].carWorldX * 20);
//        cout<< "target: " << target_2d <<endl;
//
//        //判断目标点是否在多边形内
//        if(cv::pointPolygonTest(contour1, target_2d, false) == 1)
//        {
//            cv::fillPoly(picture, contours1, Scalar(255, 0, 0));
//            for(int i = 0; i < contours1.size(); i++)
//            {
//                cv::circle(picture, target_2d, 10, Scalar(0,0,255), -1, 16, 0);
//                cv::putText(picture, to_string(5), Point(target_2d.x - 5, target_2d.y + 5),
//                            1, 1, Scalar(255, 255, 255), 1);
//            }
//        }
//        else if(cv::pointPolygonTest(contour2, target_2d, false) == 1)
//        {
//            cv::fillPoly(picture, contours2, Scalar(255, 0, 0));
//            for(int i = 0; i < contours2.size(); i++)
//            {
//                cv::circle(picture, target_2d, 10, Scalar(0,0,255), -1, 16 ,0);
//                cv::putText(picture, to_string(5), Point(target_2d.x - 5, target_2d.y + 5),
//                            1, 1, Scalar(255, 255, 255), 1);
//            }
//        }
//        else if(cv::pointPolygonTest(contour3, target_2d, false) == 1)
//        {
//            cv::fillPoly(picture, contours3, Scalar(255, 0, 0));
//            for(int i = 0; i < contours3.size(); i++)
//            {
//                //cv::RotatedRect rotateRect = cv::minAreaRect(contours3[i]);
//                //polyCenter = cv::Point2f(rotateRect.center.x, rotateRect.center.y);
//                cv::circle(picture, target_2d, 10, Scalar(0,0,255), -1, 16 ,0);
//                cv::putText(picture, to_string(5), Point(target_2d.x - 5, target_2d.y + 5),
//                            1, 1, Scalar(255, 255, 255), 1);
//            }
//        }
//        else if(cv::pointPolygonTest(contour4, target_2d, false) == 1)
//        {
//            cv::fillPoly(picture, contours4, Scalar(255, 0, 0));
//            for(int i = 0; i < contours4.size(); i++)
//            {
//                cv::circle(picture, target_2d, 10, Scalar(0,0,255), -1, 16 ,0);
//                cv::putText(picture, to_string(5), Point(target_2d.x - 5, target_2d.y + 5),
//                            1, 1, Scalar(255, 255, 255), 1);
//            }
//        }
//        else
//        {
//            cv::circle(picture, target_2d, 10, Scalar(0,0,255), -1, 16 ,0);
//            cv::putText(picture, to_string(5), Point(target_2d.x - 5, target_2d.y + 5),
//                        1, 1, Scalar(255, 255, 255), 1);
//        }
//
//        // BGR格式图片转换为RGB格式图片
//        //cv::cvtColor(picture, picture, COLOR_BGR2RGB);
//        // 转换成QImage
//        QImage pic = QImage((const unsigned char*)(picture.data), picture.cols, picture.rows, QImage::Format_RGB888);
//        // 自适应控件大小并显示图片
//        ui->label_picture->setFixedSize(300,560);
//        ui->label_picture->setPixmap(QPixmap::fromImage(pic.scaled(ui->label_picture->size(),Qt::KeepAspectRatio)));
//        //cout<< "picture size: " << ui->label_picture->width() << " " << ui->label_picture->height() <<endl;
//        cv::waitKey(1);
//    }
//
//    targetPoints.clear();
//    cout<< "---------------------------------" <<endl;
//
//}

// 订阅图像
void mainwindow::init()
{
    ros::init(init_argc, init_argv, "image_subscriber");
    ros::NodeHandle nh;
    sub = nh.subscribe("carsInfo", 100, &mainwindow::imageCallback, this);
    pua = nh.subscribe("carsInfo_two",100,&mainwindow::image2Callback,this);
    pid = nh.subscribe("carsInfo_three",100,&mainwindow::image3Callback,this);
    ros::spin();
}

// 开始按钮
void mainwindow::on_pushButton_Begin_clicked()
{
//    dialog = new Dialog(this);
//    dialog->setWindowState(Qt::WindowMaximized);
//    dialog->setModal(false);
//    dialog->show();
//    dialog->init();
    init();
    //qnode.init();
}

 // 显示图像 qnode
//void mainwindow::showImage(cv::Mat image)
//{
//    // 图像的BGR格式转换为RGB格式
//    cv::cvtColor(image, image, COLOR_BGR2RGB);
//    cv::resize(image, image, Size(640, 512));
//    // 将抓取到的帧转换为QImage格式
//    QImage qimage((const uchar*)image.data, image.cols, image.rows, QImage::Format_RGB888);
//    qimage_mutex_.lock();
//    qimage_ = qimage.copy();
//    // 将图像显示到label上
//    ui->label_video->setPixmap(QPixmap::fromImage(qimage));
//    // 将label控件resize到图像尺寸
//    ui->label_video->resize(ui->label_video->pixmap()->size());
//    qimage_mutex_.unlock();
//}

// 取消按钮
void mainwindow::on_pushButton_Cancel_clicked()
{
    sub.shutdown();   // 取消订阅
    //timer->stop();  // 停止读取数据
    //capture.release();  // 释放内存

}

// 开始加载图片
//void mainwindow::on_pushButton_Show_clicked()
//{
//    pointsSub();
//}

// 订阅坐标点回调函数
//void mainwindow::PointsMsgCallback(const sensor_msgs::PointCloud2::ConstPtr &rosPCLMsg)
//{
//    picture = cv::imread("/media/zy/Data/radar_ws/src/cubot_radar/data/picture/map.jpg");
//    //cv::flip(picture, picture, -1); //旋转180°
//
//    // 将rosPCL数据转换为PCL点云数据
//    pcl::PCLPointCloud2 PCLCloud;
//    pcl_conversions::toPCL(*rosPCLMsg, PCLCloud);
//    //std::vector<cv::Point3f> targetPoints;
//
//    // 将PCL点云数据转换为有点坐标的PCLXYZ数据
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::fromPCLPointCloud2(PCLCloud, *cloud);
//
//    for(auto& point : cloud->points)
//    {
//        cv::Point3f target_3d = cv::Point3f(point.x, point.y, point.z);
//        targetPoints.emplace_back(target_3d);
//    }
//
////    for(int i=0; i<targetPoints.size(); i++)
////    {
////        cout<< "target: " << targetPoints[i] <<endl;
////    }
//
//    for(int i=0; i<targetPoints.size(); i++)
//    {
//        cv::Point2i target_2d = cv::Point2i(targetPoints[i].x * 50, abs(targetPoints[i].y * 50));
//        cout<< "target: " << target_2d <<endl;
//
//        // 定义轮廓区域1
//        std::vector<cv::Point2i> contour1;
//        contour1.emplace_back(cv::Point2i(24,190));
//        contour1.emplace_back(cv::Point2i(87, 190));
//        contour1.emplace_back(cv::Point2i(87, 284));
//        contour1.emplace_back(cv::Point2i(24, 284));
//        std::vector<std::vector<cv::Point2i>> contours1;
//        contours1.push_back(contour1);
//        // 定义轮廓区域2
//        std::vector<cv::Point2i> contour2;
//        contour2.emplace_back(cv::Point2i(54,89));
//        contour2.emplace_back(cv::Point2i(105, 89));
//        contour2.emplace_back(cv::Point2i(105, 122));
//        contour2.emplace_back(cv::Point2i(54, 158));
//        std::vector<std::vector<cv::Point2i>> contours2;
//        contours2.push_back(contour2);
//        // 定义轮廓区域3
//        std::vector<cv::Point2i> contour3;
//        contour3.emplace_back(cv::Point2i(1,10));
//        contour3.emplace_back(cv::Point2i(200, 10));
//        contour3.emplace_back(cv::Point2i(200, 56));
//        contour3.emplace_back(cv::Point2i(1, 56));
//        std::vector<std::vector<cv::Point2i>> contours3;
//        contours3.push_back(contour3);
//        // 定义轮廓区域4
//        std::vector<cv::Point2i> contour4;
//        contour4.emplace_back(cv::Point2i(198,89));
//        contour4.emplace_back(cv::Point2i(268, 89));
//        contour4.emplace_back(cv::Point2i(268, 170));
//        contour4.emplace_back(cv::Point2i(198, 124));
//        std::vector<std::vector<cv::Point2i>> contours4;
//        contours4.push_back(contour4);
//        // 绘制多边形
//        cv::polylines(picture, contours1, true, Scalar(0, 255, 0), 2, 8);
//        cv::polylines(picture, contours2, true, Scalar(0, 255, 0), 2, 8);
//        cv::polylines(picture, contours3, true, Scalar(0, 255, 0), 2, 8);
//        cv::polylines(picture, contours4, true, Scalar(0, 255, 0), 2, 8);
//        //判断目标点是否在多边形内
//        if(cv::pointPolygonTest(contour1, target_2d, false) == 1)
//        {
//            cv::fillPoly(picture, contours1, Scalar(255, 0, 0));
//            for(int i = 0; i < contours1.size(); i++)
//            {
//                cv::circle(picture, target_2d, 10, Scalar(0,0,255), -1, 16, 0);
//                cv::putText(picture, to_string(5), Point(target_2d.x - 5, target_2d.y + 5),
//                            1, 1, Scalar(255, 255, 255), 1);
//            }
//        }
//        else if(cv::pointPolygonTest(contour2, target_2d, false) == 1)
//        {
//            cv::fillPoly(picture, contours2, Scalar(255, 0, 0));
//            for(int i = 0; i < contours2.size(); i++)
//            {
//                cv::circle(picture, target_2d, 10, Scalar(0,0,255), -1, 16 ,0);
//                cv::putText(picture, to_string(5), Point(target_2d.x - 5, target_2d.y + 5),
//                            1, 1, Scalar(255, 255, 255), 1);
//            }
//        }
//        else if(cv::pointPolygonTest(contour3, target_2d, false) == 1)
//        {
//            cv::fillPoly(picture, contours3, Scalar(255, 0, 0));
//            for(int i = 0; i < contours3.size(); i++)
//            {
//                //cv::RotatedRect rotateRect = cv::minAreaRect(contours3[i]);
//                //polyCenter = cv::Point2f(rotateRect.center.x, rotateRect.center.y);
//                cv::circle(picture, target_2d, 10, Scalar(0,0,255), -1, 16 ,0);
//                cv::putText(picture, to_string(5), Point(target_2d.x - 5, target_2d.y + 5),
//                            1, 1, Scalar(255, 255, 255), 1);
//            }
//        }
//        else if(cv::pointPolygonTest(contour4, target_2d, false) == 1)
//        {
//            cv::fillPoly(picture, contours4, Scalar(255, 0, 0));
//            for(int i = 0; i < contours4.size(); i++)
//            {
//                cv::circle(picture, target_2d, 10, Scalar(0,0,255), -1, 16 ,0);
//                cv::putText(picture, to_string(5), Point(target_2d.x - 5, target_2d.y + 5),
//                            1, 1, Scalar(255, 255, 255), 1);
//            }
//        }
//        else
//        {
//            cv::circle(picture, target_2d, 10, Scalar(0,0,255), -1, 16 ,0);
//            cv::putText(picture, to_string(5), Point(target_2d.x - 5, target_2d.y + 5),
//                        1, 1, Scalar(255, 255, 255), 1);
//        }
//
//        // BGR格式图片转换为RGB格式图片
//        //cv::cvtColor(picture, picture, COLOR_BGR2RGB);
//        // 转换成QImage
//        QImage pic = QImage((const unsigned char*)(picture.data), picture.cols, picture.rows, QImage::Format_RGB888);
//        // 自适应控件大小并显示图片
//        ui->label_picture->setFixedSize(300,560);
//        ui->label_picture->setPixmap(QPixmap::fromImage(pic.scaled(ui->label_picture->size(),Qt::KeepAspectRatio)));
//        //cout<< "picture size: " << ui->label_picture->width() << " " << ui->label_picture->height() <<endl;
//        cv::waitKey(1);
//    }
//
//    targetPoints.clear();
//    cout<< "---------------------------------" <<endl;
//}

// 订阅坐标点
//void mainwindow::pointsSub()
//{
////    ros::init(init_argc, init_argv, "points_subsriber");
////    ros::NodeHandle n;
////    ros::Subscriber points_sub = n.subscribe("points_output", 10, &mainwindow::PointsMsgCallback, this);
////    ros::spin();
//}

