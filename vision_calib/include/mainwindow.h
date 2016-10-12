#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QTimer>
#include <QImage>
#include <QPixmap>
#include <QEvent>
#include <QApplication>
#include <QDebug>
#include <vector>
#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "minho_team_ros/imgRequest.h"
#include "minho_team_ros/requestOmniVisionConf.h"
#include "imagecalibrator.h"
#include <QKeyEvent>
#include <QMessageBox>

using namespace ros;
using namespace cv;
using minho_team_ros::imgRequest;
using minho_team_ros::requestOmniVisionConf;
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
   void keyPressEvent(QKeyEvent *event);
   void display_image(const sensor_msgs::ImageConstPtr& msg);
   void addImageToScene();
   void applyBinary();
   void interactWithUser();
   //BUTTONS
   void on_bt_grab_clicked();
   void on_bt_stop_clicked();
   void on_bt_setdist_clicked();
   void on_bt_setlut_clicked();
   void on_bt_setimg_clicked();
   void on_bt_screenshot_clicked();
   void on_check_draw_clicked(bool state);
   //SLIDEBARS
   void on_h_min_valueChanged(int value);
   void on_h_max_valueChanged(int value);
   void on_s_min_valueChanged(int value);
   void on_s_max_valueChanged(int value);
   void on_v_min_valueChanged(int value);
   void on_v_max_valueChanged(int value);
   //SPINBOXES
   void on_spin_tilt_valueChanged(int value);
   void on_spin_cx_valueChanged(int value);
   void on_spin_cy_valueChanged(int value);
   //COMBOBOXES
   void on_combo_label_currentIndexChanged(int index);
   void on_combo_aqtype_currentIndexChanged(int index);
   void loadValuesOnTrackbars(minho_team_ros::label labelconf);
   void loadMirrorValues(minho_team_ros::mirrorConfig mirrorConf);
   void loadImageValues(minho_team_ros::imageConfig imageConf);
private:
   Ui::MainWindow *ui;
   int robot_id_;
   bool calibration_mode,draw_mode;
   QGraphicsScene *scene_;
   ImageCalibrator *img_calib_;
   Mat temp;
   QTimer *img_calib_timer, *interaction_timer;
   //ROS
   ros::Publisher imgreq_pub_;
   ros::Publisher mirror_pub_;
   ros::Publisher vision_pub_;
   ros::Publisher image_pub_;
   ros::ServiceClient omniVisionConf;
   image_transport::ImageTransport *it_;
   image_transport::Subscriber image_sub_;
   ros::NodeHandle *_node_;
   imgRequest msg_;
   QImage image_;

signals:
   void addNewImage();   
    
};

#endif // MAINWINDOW_H
