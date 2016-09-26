#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QEvent>
#include <QApplication>
#include <QDebug>
#include <vector>
#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include <stdio.h>
#include <stdlib.h>

using namespace ros;

#define ROS_MASTER_IP "http://172.16.49."
#define ROS_MASTER_PORT ":11311"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int robot_id,bool real_robot, QWidget *parent = 0);
    ~MainWindow();

private slots:

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
