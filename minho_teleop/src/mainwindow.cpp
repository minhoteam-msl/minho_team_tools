#include "mainwindow.h"
#include "ui_teleop.h"

/// \brief class constructor that connects signals and slots and creates ROS publishers and
/// subscribers. Given robot_id and real_robot defines the names of the topics
/// \param robot_id - id [0-6] of the robot to be used
/// \param real_robot - defines if a real robot is being used or a simulated one
MainWindow::MainWindow(int robot_id, bool real_robot, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
   ui->setupUi(this);
   teleop_activated_ = false;
   robot_id_ = robot_id;
   _update_ = new QTimer();
   dribblers_state_ = false;
   kick_request_ = is_pass_ = false;
   thrust_.resize(7);
   thrust_activation_.resize(7);
   connect(_update_,SIGNAL(timeout()),this,SLOT(onUpdate()));
   connect(_update_,SIGNAL(timeout()),this,SLOT(updateThrusts()));
   ui->hs_lin->setValue(30);
   ui->hs_ang->setValue(30);
   ui->lb_robot_name->setStyleSheet("QLabel { color : red; }");
   ui->centralWidget->setFocusPolicy(Qt::StrongFocus);
   ui->lb_robot_name->setText(QString("Robot ")+QString::number(robot_id_));
   QFont font = ui->lb_pose->font();
   font.setPointSize(6);
   font.setBold(true);
   ui->lb_pose->setFont(font);
   
   // Setup of ROS
   QString asd = "Teleop";
   asd.append(QString::number(robot_id));
   std::stringstream control_topic;
   std::stringstream teleop_topic;
   std::stringstream robot_topic;
   std::stringstream hardware_topic;
   std::stringstream resetimu_service_topic;
   std::stringstream reloc_service_topic;
   std::stringstream kick_service_topic;
   
   
   if(real_robot){
      // Setup ROS Node and pusblishers/subscribers in SIMULATOR
      control_topic << "/controlInfo";
      teleop_topic << "/teleop";
      robot_topic << "/robotInfo";
      hardware_topic << "/hardwareInfo";
      resetimu_service_topic << "/requestResetIMU";
      reloc_service_topic << "/requestReloc";
      kick_service_topic << "/requestKick";
      
      if(robot_id>0){
         // Setup custom master
         QString robot_ip = QString(ROS_MASTER_IP)+QString::number(robot_id)
                            +QString(ROS_MASTER_PORT);
         ROS_WARN("ROS_MASTER_URI: '%s'",robot_ip.toStdString().c_str());
         setenv("ROS_MASTER_URI",robot_ip.toStdString().c_str(),1);
      } else ROS_WARN("ROS_MASTER_URI is localhost");
      
   } else {
      // Setup ROS Node and pusblishers/subscribers in REAL ROBOT
      control_topic << "minho_gazebo_robot" << std::to_string(robot_id) << "/controlInfo";
      teleop_topic << "minho_gazebo_robot" << std::to_string(robot_id) << "/teleop";
      robot_topic << "minho_gazebo_robot" << std::to_string(robot_id) << "/robotInfo";
      hardware_topic << "minho_gazebo_robot" << std::to_string(robot_id) << "/hardwareInfo";
      resetimu_service_topic << "minho_gazebo_robot" << std::to_string(robot_id) << "/requestResetIMU"; 
      reloc_service_topic << "minho_gazebo_robot" << std::to_string(robot_id) << "/requestReloc"; 
      kick_service_topic << "minho_gazebo_robot" << std::to_string(robot_id) << "/requestKick"; 
   }
   

   //Initialize ROS
   int argc = 0;
   ros::init(argc, NULL, asd.toStdString().c_str(),ros::init_options::NoSigintHandler);
   _node_ = new ros::NodeHandle();
   //Initialize controlInfo publisher
   control_pub_ = _node_->advertise<controlInfo>(control_topic.str().c_str(),1);
   //Initialize teleop publisher
   teleop_pub_ = _node_->advertise<teleop>(teleop_topic.str().c_str(),100);
   //Initialize robotlInfo subscriber
   robot_sub_ = _node_->subscribe(robot_topic.str().c_str(), 100, &MainWindow::robotInfoCallback, this);
   //Initialize hardwareInfo subscriber
   hardware_sub_ = _node_->subscribe(hardware_topic.str().c_str(), 100, &MainWindow::hardwareInfoCallback, this);
   
   resetIMUService = _node_->serviceClient<minho_team_ros::requestResetIMU>(resetimu_service_topic.str().c_str());
   requestRelocService = _node_->serviceClient<minho_team_ros::requestReloc>(reloc_service_topic.str().c_str());
   requestKickService = _node_->serviceClient<minho_team_ros::requestKick>(kick_service_topic.str().c_str());
   
   //Initialize spinner
   spinner = new ros::AsyncSpinner(2);
   spinner->start();

   _update_->start(50); // update teleop data to robot
}

/// \brief class destructor, that sends teleop off message on close
MainWindow::~MainWindow()
{
    on_pushButton_2_clicked();
    ros::shutdown();    
    delete ui;
}

/// \brief button slot function to turn on teleop for defined robot. Sends a teleop 
/// message over ROS enabling teleop
void MainWindow::on_pushButton_clicked()
{
    teleop_activated_ = true;
    ui->lb_robot_name->setStyleSheet("QLabel { color : green; }");
    kick_request_ = false;
    //Notify teleop state change
    minho_team_ros::teleop msg;
    msg.set_teleop = true;
    if(teleop_pub_) teleop_pub_.publish(msg);
}

/// \brief button slot function to turn off teleop for defined robot. Sends a teleop 
/// message over ROS disabling teleop
void MainWindow::on_pushButton_2_clicked()
{
    teleop_activated_ = false;
    ui->lb_robot_name->setStyleSheet("QLabel { color : red; }");
    //Notify teleop state change
    minho_team_ros::teleop msg;
    msg.set_teleop = false;
    if(teleop_pub_) teleop_pub_.publish(msg);
}

/// \brief horizontal_bar slot function to change maximum linear velocity
/// \param value - new value to be applied to maximum linear velocity
void MainWindow::on_hs_lin_valueChanged(int value)
{
    ui->lb_lin->setText(QString::number(value));
    max_lin_ = value;
}
/// \brief horizontal_bar slot function to change maximum angular velocity
/// \param value - new value to be applied to maximum angular velocity
void MainWindow::on_hs_ang_valueChanged(int value)
{
    ui->lb_ang->setText(QString::number(value));
    max_ang_ = value;
}

/// \brief periodical function called by _update_ QTimer to compute new velocities
/// to be apllied and send them to the robot (real or sim) over ROS using
/// controlInfo message
void MainWindow::onUpdate()
{
    if(teleop_activated_){
        minho_team_ros::controlInfo msg;
        // publish data over ROS
        double sumY = thrust_[0]-thrust_[2];
        double sumX = thrust_[3]-thrust_[1];
        int velocity = sqrt(sumX*sumX+sumY*sumY);
        if(velocity>max_lin_) velocity = max_lin_;
        double direction = atan2(sumY,sumX)*(180.0/M_PI)-90;
        while(direction>360) direction-=360;
        while(direction<0) direction+=360;
        double angSum = thrust_[5]-thrust_[6];

        if(kick_request_){
            // send kick
            requestKick srv;
            srv.request.kick_strength = thrust_[4];
            if(is_pass_) srv.request.kick_is_pass = is_pass_;
            thrust_[4] = 0;
            kick_request_ = is_pass_ = false;
            requestKickService.call(srv);
        }
        
        // publish stuff
        msg.linear_velocity = velocity;
        msg.movement_direction = (int)direction;
        msg.angular_velocity = (int)angSum;
        msg.is_teleop = true;
        msg.dribbler_on = dribblers_state_;
        if(control_pub_) control_pub_.publish(msg);
    }
}
/// \brief function to sum or subtract value to the directional velocities
/// to easily compute the resultant velocity and direction (euler) to send to 
/// the robot. Also computes the resultant angular velocity to be applied
void MainWindow::updateThrusts()
{
    if(teleop_activated_){
        if(thrust_activation_[0]) { thrust_[0] += 3; if(thrust_[0]>max_lin_) thrust_[0]=max_lin_; }
        else { thrust_[0] -= 20; if(thrust_[0]<0) thrust_[0]=0; }
        if(thrust_activation_[1]) { thrust_[1] += 3; if(thrust_[1]>max_lin_) thrust_[1]=max_lin_; }
        else { thrust_[1] -= 20; if(thrust_[1]<0) thrust_[1]=0; }
        if(thrust_activation_[2]) { thrust_[2] += 3; if(thrust_[2]>max_lin_) thrust_[2]=max_lin_; }
        else { thrust_[2] -= 20; if(thrust_[2]<0) thrust_[2]=0; }
        if(thrust_activation_[3]) { thrust_[3] += 3; if(thrust_[3]>max_lin_) thrust_[3]=max_lin_; }
        else { thrust_[3] -= 20; if(thrust_[3]<0) thrust_[3]=0; }
        if(thrust_activation_[4]) { thrust_[4] += 5; if(thrust_[4]>100) thrust_[4]=100; }
        if(thrust_activation_[5]) { thrust_[5] += 1; if(thrust_[5]>max_ang_) thrust_[5]=max_ang_; }
        else { thrust_[5] -= 20; if(thrust_[5]<0) thrust_[5]=0; }
        if(thrust_activation_[6]) { thrust_[6] += 1; if(thrust_[6]>max_ang_) thrust_[6]=max_ang_; }
        else { thrust_[6] -= 20; if(thrust_[6]<0) thrust_[6]=0; }
    }
}
/// \brief callback function called when key is pressed. It allows to implement
/// commands by keys instead of mouse, which is more intuitive
/// \param event - key event detected where info about pressed key is given
void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if(!event->isAutoRepeat()){
        switch(event->key()){
            case 'W':{  //front
                thrust_activation_[0] = true;
                break;
            }
            case 'A':{  //left
                thrust_activation_[1] = true;
                break;
            }
            case 'S':{  //back
                thrust_activation_[2] = true;
                break;
            }
            case 'D':{  //right
                thrust_activation_[3] = true;
                break;
            }
            case 'Q':{  //turn left
                thrust_activation_[5] = true;
                break;
            }
            case 'E':{  //turn right
                thrust_activation_[6] = true;
                break;
            }
            case 'P':{  //pass
                thrust_activation_[4] = true;
                is_pass_ = true;
                break;
            }
            case 'K':{  //kick
                thrust_activation_[4] = true;
                is_pass_ = false;
                break;
            }
            case 'L':{  //dribblers on/off
                dribblers_state_ = !dribblers_state_;
                break;
            }
            case ' ':{  //dribblers on/off
                for(int i = 0;i<7;i++) { thrust_activation_[i] = false; thrust_[i] = 0; }
                break;
            }
            case 'T':{  //turn on/off
                if(teleop_activated_){
                    on_pushButton_2_clicked();
                } else {
                    on_pushButton_clicked();
                }
                break;
            }
            case Qt::Key_Escape:{
              on_pushButton_2_clicked();
              ros::shutdown(); 
              exit(1);
              break;
            }
        }
    }
}
/// \brief callback function called when key is released. It allows to implement
/// commands by keys instead of mouse, which is more intuitive
/// \param event - key event detected where info about released key is given
void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    if(!event->isAutoRepeat()){
        switch(event->key()){
            case 'W':{  //front
                thrust_activation_[0] = false;
                break;
            }
            case 'A':{  //left
                thrust_activation_[1] = false;
                break;
            }
            case 'S':{  //back
                thrust_activation_[2] = false;
                break;
            }
            case 'D':{  //right
                thrust_activation_[3] = false;
                break;
            }
            case 'Q':{  //turn left
                thrust_activation_[5] = false;
                break;
            }
            case 'E':{  //turn right
                thrust_activation_[6] = false;
                break;
            }
            case 'P':{  //pass
                thrust_activation_[4] = false;
                kick_request_ = true;
                break;
            }
            case 'K':{  //kick
                thrust_activation_[4] = false;
                kick_request_ = true;
                break;
            }
        }
    }
}

/// \brief ROS callback to receive robotInfo message containing primary world 
/// state of the current robot.
/// \param msg - message contaning robotInfo message data
void MainWindow::robotInfoCallback(const minho_team_ros::robotInfo::ConstPtr& msg)
{
    QString info = QString("[")+QString::number(msg->robot_pose.x,'f',2) + 
                   QString(",")+QString::number(msg->robot_pose.y,'f',2) +
                   QString(",")+QString::number(msg->robot_pose.z,'f',1) +
                   QString("º] : HasBall->") + QString::number(msg->has_ball);
                   
   ui->lb_pose->setText(info);
}

/// \brief ROS callback to receive hardwareInfo message containing primary hardware 
/// information of the robot. Mainly, it is interesting to display the battery levels.
/// \param msg - message contaning hardwareInfo message data
void MainWindow::hardwareInfoCallback(const minho_team_ros::hardwareInfo::ConstPtr& msg)
{
   QString info = QString("PC:")+QString::number(msg->battery_pc,'f',2) + 
                   QString(" | CAM: ")+QString::number(msg->battery_camera,'f',2) +
                   QString(" | MAIN: ")+QString::number(msg->battery_main,'f',2);
                   
   ui->lb_bats->setText(info); 
}

/// \brief button slot function to reset the IMU geo-0º-reference
void MainWindow::on_bt_resetimu_clicked()
{
   requestResetIMU srv;
   resetIMUService.call(srv);
}

/// \brief button slot function initialize the reloc process in the target robot
void MainWindow::on_bt_reloc_clicked()
{
   requestReloc srv;
   requestRelocService.call(srv);
}
