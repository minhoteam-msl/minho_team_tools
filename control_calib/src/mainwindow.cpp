#include "mainwindow.h"
#include "ui_control_calib.h"

MainWindow::MainWindow(int robot_id, bool real_robot, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    on_max_lin_valueChanged(ui->max_lin->value());
    on_max_rot_valueChanged(ui->max_rot->value());
    ui->action_0->setChecked(true);

    // Setup of ROS
    QString asd = "Control_calib";
    asd.append(QString::number(robot_id));
    asd.append("_");
    asd.append(QString::number(std::time(0)));
    std::stringstream ai_topic;
    std::stringstream ccalib_topic;
    std::stringstream ccalib_sv_topic;
    if(!real_robot){
      // Setup ROS Node and pusblishers/subscribers in SIMULATOR
      ai_topic << "minho_gazebo_robot" << std::to_string(robot_id);
      ccalib_topic << "minho_gazebo_robot" << std::to_string(robot_id);
      ccalib_sv_topic << "minho_gazebo_robot" << std::to_string(robot_id);
    } else {
      // Setup ROS Node and pusblishers/subscribers in REAL ROBOT
      if(robot_id>0){
         // Setup custom master
         QString robot_ip = QString(ROS_MASTER_IP)+QString::number(robot_id)
                            +QString(ROS_MASTER_PORT);
         ROS_WARN("ROS_MASTER_URI: '%s'",robot_ip.toStdString().c_str());
         setenv("ROS_MASTER_URI",robot_ip.toStdString().c_str(),1);
      } else ROS_WARN("ROS_MASTER_URI is localhost");
    }

    ai_topic << "/aiInfo";
    ccalib_topic << "/controlConfig";
    ccalib_sv_topic << "/requestControlConfig";
      
    //Initialize ROS
    int argc = 0;
    ros::init(argc, NULL, asd.toStdString().c_str(),ros::init_options::NoSigintHandler);
    node_ = new ros::NodeHandle();

    ai_pub = node_->advertise<aiInfo>(ai_topic.str().c_str(),100);
    cconfig_pub = node_->advertise<controlConfig>(ccalib_topic.str().c_str(),100);
    controlConfSv = node_->serviceClient<minho_team_ros::requestControlConfig>(ccalib_sv_topic.str().c_str());
    //Request Current Configuration
    requestControlConfig srv; 
    if(controlConfSv.call(srv)){
      ROS_INFO("Retrieved configuration from target robot.");
      ui->spin_p->setValue(srv.response.config.P);
      ui->spin_i->setValue(srv.response.config.I);
      ui->spin_d->setValue(srv.response.config.D);
      ui->max_lin->setValue(srv.response.config.max_linear_velocity);
      ui->max_rot->setValue(srv.response.config.max_angular_velocity);
    } else ROS_ERROR("Failed to retrieve configuration from target robot.");

    send_timer = new QTimer();
    connect(send_timer,SIGNAL(timeout()),this,SLOT(sendInfo()));
    send_timer->start(60);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_max_lin_valueChanged(int value)
{
    ui->lb_maxlin->setText(QString::number(value));
    ctrl_config.max_linear_velocity = value;
}

void MainWindow::on_max_rot_valueChanged(int value)
{
    ui->lb_maxrot->setText(QString::number(value));
    ctrl_config.max_angular_velocity = value;
}

void MainWindow::on_bt_path_clicked()
{
    ctrl_config.send_path = !ctrl_config.send_path;
}

void MainWindow::on_bt_voronoi_clicked()
{
    ctrl_config.send_voronoi = !ctrl_config.send_voronoi;
}

void MainWindow::on_spin_p_valueChanged(double value)
{
    ctrl_config.P = value;
}

void MainWindow::on_spin_i_valueChanged(double value)
{
    ctrl_config.I = value;
}

void MainWindow::on_spin_d_valueChanged(double value)
{
    ctrl_config.D = value;
}

void MainWindow::on_targ_x_valueChanged(double value)
{
    ai.target_pose.x = value;
}

void MainWindow::on_targ_y_valueChanged(double value)
{
    ai.target_pose.y = value;
}

void MainWindow::on_targ_theta_valueChanged(double value)
{
    ai.target_pose.z = value;
}

void MainWindow::on_targ_k_x_valueChanged(double value)
{
    ai.target_kick.x = value;
}

void MainWindow::on_targ_k_y_valueChanged(double value)
{
    ai.target_kick.y = value;
}

void MainWindow::on_action_0_clicked(bool state)
{
    if(state) ai.action = 0;
}

void MainWindow::on_action_1_clicked(bool state)
{
    if(state) ai.action = 1;
}

void MainWindow::on_action_2_clicked(bool state)
{
    if(state) ai.action = 2;
}

void MainWindow::on_action_3_clicked(bool state)
{
    if(state) ai.action = 3;
}

void MainWindow::on_targ_kstr_valueChanged(int value)
{
    ai.target_kick_strength = value;
}

void MainWindow::on_kick_type_chuto_clicked(bool state)
{
    if(state) ai.target_kick_is_pass = false;
}
void MainWindow::on_kick_type_passe_clicked(bool state)
{
    if(state) ai.target_kick_is_pass = true;
}

void MainWindow::sendInfo()
{
    ai_pub.publish(ai);
    cconfig_pub.publish(ctrl_config);
}

