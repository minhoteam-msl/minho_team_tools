#include "mainwindow.h"
#include "ui_vision_calib.h"

MainWindow::MainWindow(int robot_id, bool real_robot, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
   ui->setupUi(this);
   setup();
   
   robot_id_ = robot_id;
   requesting_on = false;
   ui->lb_robot_name->setText(QString("Robot ")+QString::number(robot_id_));
   connect(this,SIGNAL(addNewImage()),this,SLOT(addImageToScene()));
   ui->graphicsView->setHorizontalScrollBarPolicy ( Qt::ScrollBarAlwaysOff );
	ui->graphicsView->setVerticalScrollBarPolicy ( Qt::ScrollBarAlwaysOff );
	
   // Setup of ROS
   QString asd = "Vision_calib";
   asd.append(QString::number(robot_id));
   std::stringstream imreq_topic;
   std::stringstream imgtrans_topic;
   if(real_robot){
      // Setup ROS Node and pusblishers/subscribers in SIMULATOR
      imreq_topic << "/imgRequest";
      imgtrans_topic << "/camera";
      
      if(robot_id>0){
         // Setup custom master
         QString robot_ip = QString(ROS_MASTER_IP)+QString::number(robot_id)
                            +QString(ROS_MASTER_PORT);
         ROS_WARN("ROS_MASTER_URI: '%s'",robot_ip.toStdString().c_str());
         setenv("ROS_MASTER_URI",robot_ip.toStdString().c_str(),1);
      } else ROS_WARN("ROS_MASTER_URI is localhost");
      
   } else {
      // Setup ROS Node and pusblishers/subscribers in REAL ROBOT
      imreq_topic << "minho_gazebo_robot" << std::to_string(robot_id) << "/imgRequest";
      imgtrans_topic << "minho_gazebo_robot" << std::to_string(robot_id) << "/camera";
   }
   

   //Initialize ROS
   int argc = 0;
   ros::init(argc, NULL, asd.toStdString().c_str(),ros::init_options::NoSigintHandler);
   _node_ = new ros::NodeHandle();
   it_ = new image_transport::ImageTransport(*_node_);
   imgreq_pub_ = _node_->advertise<imgRequest>(imreq_topic.str().c_str(),100);
   //Initialize image_transport subscriber
   image_sub_ = it_->subscribe(imgtrans_topic.str().c_str(),1,&MainWindow::display_image,this);
}

MainWindow::~MainWindow()
{  
   ros::shutdown();
   delete ui;
}

void MainWindow::setup()
{
   scene_ = new QGraphicsScene();
   ui->graphicsView->setScene(scene_); 
}
   
void MainWindow::display_image(const sensor_msgs::ImageConstPtr& msg)
{
   cv_bridge::CvImagePtr recv_img = cv_bridge::toCvCopy(msg,"bgr8");
   Mat temp = recv_img->image;
   image_ =  QImage( temp.data,
                 temp.cols, temp.rows,
                 static_cast<int>(temp.step),
                 QImage::Format_RGB888 );

   emit addNewImage();
}

void MainWindow::on_bt_grab_clicked()
{
   bool multiple_send_request = ui->radio_multiple->isChecked();
   msg_.is_multiple = multiple_send_request;
   msg_.frequency = ui->spin_framerate->value();
   msg_.type = (int)pow(2,ui->combo_aqtype->currentIndex());
   imgreq_pub_.publish(msg_);
}

void MainWindow::on_bt_stop_clicked()
{
   msg_.is_multiple = false;
   msg_.frequency = ui->spin_framerate->value();
   msg_.type = (int)pow(2,ui->combo_aqtype->currentIndex());
   imgreq_pub_.publish(msg_);
}

void MainWindow::addImageToScene()
{
   scene_->clear();
   scene_->addPixmap(QPixmap::fromImage(image_));
}
