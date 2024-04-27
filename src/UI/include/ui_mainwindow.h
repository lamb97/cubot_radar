/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_mainwindow
{
public:
    QWidget *centralwidget;
    QVBoxLayout *verticalLayout_3;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_2;
    QHBoxLayout *horizontalLayout_3;
    QVBoxLayout *verticalLayout;
    QLabel *label_video3;
    QLabel *label_video;
    QLabel *label_video2;
    QHBoxLayout *horizontalLayout;
    QSpacerItem *horizontalSpacer;
    QPushButton *pushButton_Begin;
    QSpacerItem *horizontalSpacer_3;
    QPushButton *pushButton_Cancel;
    QSpacerItem *horizontalSpacer_5;
    QPushButton *pushButton_Exit;
    QSpacerItem *horizontalSpacer_6;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *mainwindow)
    {
        if (mainwindow->objectName().isEmpty())
            mainwindow->setObjectName(QString::fromUtf8("mainwindow"));
        mainwindow->resize(1001, 659);
        centralwidget = new QWidget(mainwindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        verticalLayout_3 = new QVBoxLayout(centralwidget);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(0);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(0);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(0);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label_video3 = new QLabel(centralwidget);
        label_video3->setObjectName(QString::fromUtf8("label_video3"));
        label_video3->setEnabled(true);
        label_video3->setMaximumSize(QSize(324, 16777215));

        verticalLayout->addWidget(label_video3);


        horizontalLayout_3->addLayout(verticalLayout);

        label_video = new QLabel(centralwidget);
        label_video->setObjectName(QString::fromUtf8("label_video"));
        label_video->setEnabled(true);

        horizontalLayout_3->addWidget(label_video);

        label_video2 = new QLabel(centralwidget);
        label_video2->setObjectName(QString::fromUtf8("label_video2"));

        horizontalLayout_3->addWidget(label_video2);


        horizontalLayout_2->addLayout(horizontalLayout_3);


        verticalLayout_2->addLayout(horizontalLayout_2);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(0);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalSpacer = new QSpacerItem(48, 28, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        pushButton_Begin = new QPushButton(centralwidget);
        pushButton_Begin->setObjectName(QString::fromUtf8("pushButton_Begin"));

        horizontalLayout->addWidget(pushButton_Begin);

        horizontalSpacer_3 = new QSpacerItem(48, 28, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_3);

        pushButton_Cancel = new QPushButton(centralwidget);
        pushButton_Cancel->setObjectName(QString::fromUtf8("pushButton_Cancel"));

        horizontalLayout->addWidget(pushButton_Cancel);

        horizontalSpacer_5 = new QSpacerItem(48, 28, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_5);

        pushButton_Exit = new QPushButton(centralwidget);
        pushButton_Exit->setObjectName(QString::fromUtf8("pushButton_Exit"));

        horizontalLayout->addWidget(pushButton_Exit);

        horizontalSpacer_6 = new QSpacerItem(48, 28, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_6);


        verticalLayout_2->addLayout(horizontalLayout);

        verticalLayout_2->setStretch(0, 560);
        verticalLayout_2->setStretch(1, 31);

        verticalLayout_3->addLayout(verticalLayout_2);

        mainwindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(mainwindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1001, 27));
        mainwindow->setMenuBar(menubar);
        statusbar = new QStatusBar(mainwindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        mainwindow->setStatusBar(statusbar);

        retranslateUi(mainwindow);

        QMetaObject::connectSlotsByName(mainwindow);
    } // setupUi

    void retranslateUi(QMainWindow *mainwindow)
    {
        mainwindow->setWindowTitle(QCoreApplication::translate("mainwindow", "MainWindow", nullptr));
        label_video3->setText(QString());
        label_video->setText(QString());
        label_video2->setText(QString());
        pushButton_Begin->setText(QCoreApplication::translate("mainwindow", "\350\247\206\351\242\221", nullptr));
        pushButton_Cancel->setText(QCoreApplication::translate("mainwindow", "\345\217\226\346\266\210", nullptr));
        pushButton_Exit->setText(QCoreApplication::translate("mainwindow", "\351\200\200\345\207\272", nullptr));
    } // retranslateUi

};

namespace Ui {
    class mainwindow: public Ui_mainwindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
