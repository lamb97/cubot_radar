//
// Created by godzhu on 2021/12/29.
//

#include <QApplication>
#include <QPushButton>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    // 显示UI界面
    QApplication a(argc, argv);
    mainwindow mainWindow;
    mainWindow.show();

    return QApplication::exec();
}