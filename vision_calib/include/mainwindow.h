#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QImage>
#include <QPixmap>
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
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "minho_team_ros/imgRequest.h"
using namespace ros;
using namespace cv;
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
   void setup();
   void display_image(const sensor_msgs::ImageConstPtr& msg);
   void addImageToScene();
   void on_bt_grab_clicked();
   void on_bt_stop_clicked();
private:
   Ui::MainWindow *ui;
   int robot_id_;
   bool requesting_on;

   ros::Publisher imgreq_pub_;
   image_transport::ImageTransport *it_;
   image_transport::Subscriber image_sub_;
   ros::NodeHandle *_node_;
   QGraphicsScene *scene_;
   imgRequest msg_;
   QImage image_;
signals:
   void addNewImage();   
    
};

#endif // MAINWINDOW_H
