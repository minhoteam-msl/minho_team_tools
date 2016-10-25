#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QEvent>
#include <QApplication>
#include <QtQuick/QQuickView>
#include <QtQuick/QQuickItem>
#include <QDebug>
#include <vector>
#include "ros/ros.h"
#include "minho_team_ros/robotInfo.h"
#include "minho_team_ros/controlInfo.h"
#include "minho_team_ros/hardwareInfo.h"
#include "minho_team_ros/teleop.h"
#include <iostream>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include <stdio.h>
#include <stdlib.h>

using namespace ros;
using minho_team_ros::robotInfo; //Namespace for robot information msg - PUBLISHING
using minho_team_ros::controlInfo; //Namespace for control information msg - SUBSCRIBING
using minho_team_ros::teleop; //Namespace for teleop information msg - SUBSCRIBING
using minho_team_ros::hardwareInfo; //Namespace for hardware information msg - SUBSCRIBING

#define ROS_MASTER_IP "http://172.16.49."
#define ROS_MASTER_PORT ":11311"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
   /// \brief class constructor that connects signals and slots and creates ROS publishers and
   /// subscribers. Given robot_id and real_robot defines the names of the topics
   /// \params [in] : robot_id -> id [0-6] of the robot to be used
   ///                real_robot -> defines if a real robot is being used or a simulated one
   explicit MainWindow(int robot_id,bool real_robot, QWidget *parent = 0);
   /// \brief class destructor, that sends teleop off message on close
   ~MainWindow();

private slots:
   /// \brief button slot function to turn on teleop for defined robot. Sends a teleop 
   /// message over ROS enabling teleop
   void on_pushButton_clicked();
   /// \brief button slot function to turn off teleop for defined robot. Sends a teleop 
   /// message over ROS disabling teleop
   void on_pushButton_2_clicked();
   /// \brief horizontal_bar slot function to change maximum linear velocity
   /// \params [in] : value -> new value to be applied to maximum linear velocity
   void on_hs_lin_valueChanged(int value);
   /// \brief horizontal_bar slot function to change maximum angular velocity
   /// \params [in] : value -> new value to be applied to maximum angular velocity
   void on_hs_ang_valueChanged(int value);
   /// \brief periodical function called by _update_ QTimer to compute new velocities
   /// to be apllied and send them to the robot (real or sim) over ROS using
   /// controlInfo message
   void onUpdate();
   /// \brief function to sum or subtract value to the directional velocities
   /// to easily compute the resultant velocity and direction (euler) to send to 
   /// the robot. Also computes the resultant angular velocity to be applied
   void updateThrusts();
   /// \brief callback function called when key is pressed. It allows to implement
   /// commands by keys instead of mouse, which is more intuitive
   /// \params [in] : event -> key event detected where info about pressed key is given
   void keyPressEvent(QKeyEvent *event);
   /// \brief callback function called when key is released. It allows to implement
   /// commands by keys instead of mouse, which is more intuitive
   /// \params [in] : event -> key event detected where info about released key is given
   void keyReleaseEvent(QKeyEvent *event);
   /// \brief ROS callback to receive robotInfo message containing primary world 
   /// state of the current robot.
   /// \params [in] : msg -> message contaning robotInfo message data
   void robotInfoCallback(const minho_team_ros::robotInfo::ConstPtr& msg);
   /// \brief ROS callback to receive hardwareInfo message containing primary hardware 
   /// information of the robot. Mainly, it is interesting to display the battery levels.
   /// \params [in] : msg -> message contaning hardwareInfo message data
   void hardwareInfoCallback(const minho_team_ros::hardwareInfo::ConstPtr& msg);
   /// \brief event to drive all controls to zero when GUI is deselected by the user
   bool event( QEvent * pEvent ){
      if ( pEvent->type() == QEvent::WindowDeactivate ){
         for(int i = 0;i<7;i++) { thrust_activation_[i] = false; thrust_[i] = 0; }
      }
      return QMainWindow::event( pEvent );
	}
private:
   /// \brief pointer to GUI
   Ui::MainWindow *ui;
   /// \brief defines whether teleop is activated or not
   bool teleop_activated_;
   /// \brief update timer to update thrusts and send data
   QTimer *_update_;
   /// \brief defined robot id 
   int robot_id_;
   /// \brief maximum linear and angular velocites values
   int max_lin_, max_ang_;
   /// \brief defines wether there was a kick request. defines 
   /// wether the request is a kick or a pass
   bool kick_request_, is_pass_;
   /// \brief defines the state of the dribblers (running or not)
   bool dribblers_state_;
   /// \brief vector to hold current directional thrusts
   std::vector<float> thrust_;
   /// \brief vector to enable or disable a certain directional thrust
   std::vector<bool> thrust_activation_;
   /// \brief controlInfo and teleop ROS publishers
   ros::Publisher control_pub_, teleop_pub_;
   /// \brief robotInfo ROS subscriber
   ros::Subscriber robot_sub_;
   /// \brief hardwareInfo ROS subscriber
   ros::Subscriber hardware_sub_;
   /// \brief ROS node handle pointer
   ros::NodeHandle *_node_;
   /// \brief ROS AsyncSpinner pointer 
   ros::AsyncSpinner *spinner;
};

#endif // MAINWINDOW_H
