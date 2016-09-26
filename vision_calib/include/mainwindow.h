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

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "minho_team_ros/imgRequest.h"
using namespace ros;
using minho_team_ros::imgRequest;
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
   void display_image(const sensor_msgs::ImageConstPtr& msg);
private:
    Ui::MainWindow *ui;
    int robot_id_;
    bool requesting_on;
    
    ros::Publisher imgreq_pub_;
    image_transport::ImageTransport *it_;
    image_transport::Subscriber image_sub_;
    ros::NodeHandle *_node_;
    ros::AsyncSpinner *spinner;
};

#endif // MAINWINDOW_H
