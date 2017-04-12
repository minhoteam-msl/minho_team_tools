#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "ros/ros.h"
#include "minho_team_ros/aiInfo.h"
#include "minho_team_ros/controlConfig.h"
#include "minho_team_ros/requestControlConfig.h"
#include <QTimer>

using minho_team_ros::aiInfo;
using minho_team_ros::controlConfig;
using minho_team_ros::requestControlConfig;

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
    void on_max_lin_vel_valueChanged(int value);
    void on_max_ang_vel_valueChanged(int value);
    void on_bt_voronoi_seg_clicked();
    void on_bt_intersect_seg_clicked();
    void on_bt_dijk_path_clicked();
    void on_bt_dijk_path_obst_circle_clicked();
    void on_bt_smooth_path_clicked();
    void on_bt_smooth_path_obst_circle_clicked();
    void on_bt_path_clicked();
    void on_bt_path_interpolation_clicked();
    void on_bt_obstacles_circle_clicked();
    void on_spin_Kp_rot_valueChanged(double value);
    void on_spin_Ki_rot_valueChanged(double value);
    void on_spin_Kd_rot_valueChanged(double value);
    void on_spin_Kp_lin_valueChanged(double value);
    void on_spin_Ki_lin_valueChanged(double value);
    void on_spin_Kd_lin_valueChanged(double value);
    void on_targ_x_valueChanged(double value);
    void on_targ_y_valueChanged(double value);
    void on_targ_theta_valueChanged(double value);
    void on_targ_k_x_valueChanged(double value);
    void on_targ_k_y_valueChanged(double value);
    void on_action_0_clicked(bool state);
    void on_action_50_clicked(bool state);
    void on_targ_kstr_valueChanged(int value);
    void on_kick_type_chuto_clicked(bool state);
    void on_kick_type_passe_clicked(bool state);
    void on_spin_accel_valueChanged(double value);
    void on_spin_decel_valueChanged(double value);
    void on_bt_send_parameters_clicked();
    void sendInfo();
private:
    Ui::MainWindow *ui;
    aiInfo ai;
    controlConfig ctrl_config;
    /// \brief requestControlConfig ROS Service Client
    ros::ServiceClient controlConfSv;
    ros::Publisher ai_pub;
    ros::Publisher cconfig_pub;
    /// \brief ROS Node
    ros::NodeHandle *node_;
    QTimer *send_timer;
};

#endif // MAINWINDOW_H
