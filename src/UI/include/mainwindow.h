#ifndef SRC_MAINWINDOW_H
#define SRC_MAINWINDOW_H

#include <QMainWindow>
#include <QWidget>
#include <QPaintEvent>
#include <QTimer>
#include <QPainter>
#include <QPixmap>
#include <QLabel>
#include <QImage>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "huaray_camera.h"
#include "cubot_radar/CarsMsg.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <QMutex>
#include <QThread>
//#include "qnode.h"
//#include "dialog.h"

using namespace std;
using namespace cv;

QT_BEGIN_NAMESPACE
namespace Ui
{
    class QNode;
    class mainwindow;
}
QT_END_NAMESPACE

class mainwindow : public QMainWindow
{
Q_OBJECT

public:
    explicit mainwindow(QWidget *parent = nullptr);

    ~mainwindow() override;

    void imageCallback(const cubot_radar::CarsMsg::ConstPtr &carsMsg);
    void image2Callback(const cubot_radar::CarsMsg::ConstPtr &carsMsg);
    void image3Callback(const cubot_radar::CarsMsg::ConstPtr &carsMsg);
   // void Callback(const cubot_radar::CarsMsg::ConstPtr &carsMsg);
    //void PointsMsgCallback(const sensor_msgs::PointCloud2::ConstPtr &rosPCLMsg);

public slots:
    //void showImage(cv::Mat);

    void init();
    void display(cv::Mat dstImage);
    void display2(cv::Mat dstImage);
    void display3(cv::Mat dstImage);
//    void pointsSub();

//Q_SIGNALS:
//    void imageSignal(cv::Mat);

private slots:
    void on_pushButton_Begin_clicked();
//    void on_pushButton_Show_clicked();
    void on_pushButton_Cancel_clicked();

private:
    //QTimer *timer;
    //cv::VideoCapture capture;
    HuarayCamera camera;
    HuarayCameraParam param;
    cv::Mat picture;

    cv::Mat srcImage;
    int init_argc{};
    char **init_argv{};
    ros::Subscriber sub;
    ros::Subscriber pua;
    ros::Subscriber pid;
    QImage qimage_;
    mutable QMutex qimage_mutex_;

    std::vector<cv::Point3f> targetPoints;  //订阅的目标点

    Ui::mainwindow *ui;
//    QNode qnode;
//    Dialog *dialog; //添加私有成员，为一个dialog(窗口)的指针
};


#endif //SRC_MAINWINDOW_H
