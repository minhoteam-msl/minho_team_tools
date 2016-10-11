#include "mainwindow.h"
#include "ui_vision_calib.h"

MainWindow::MainWindow(int robot_id, bool real_robot, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
   ui->setupUi(this);
   setup();
   
   robot_id_ = robot_id;
   calibration_mode = false;
   img_calib_timer = new QTimer();
   img_calib_ = new ImageCalibrator();
   ui->lb_robot_name->setStyleSheet("QLabel { color : red; }");
   ui->lb_robot_name->setText(QString("Calibration Mode for Robot ")+QString::number(robot_id_));
   connect(this,SIGNAL(addNewImage()),this,SLOT(addImageToScene()));
   ui->graphicsView->setHorizontalScrollBarPolicy ( Qt::ScrollBarAlwaysOff );
	ui->graphicsView->setVerticalScrollBarPolicy ( Qt::ScrollBarAlwaysOff );
	connect(img_calib_timer,SIGNAL(timeout()),this,SLOT(applyBinary()));
	
   // Setup of ROS
   QString asd = "Vision_calib";
   asd.append(QString::number(robot_id));
   std::stringstream imreq_topic;
   std::stringstream imgtrans_topic;
   std::stringstream mirror_topic;
   std::stringstream vision_topic;
   
   if(real_robot){
      // Setup ROS Node and pusblishers/subscribers in SIMULATOR
      imreq_topic << "/imgRequest";
      imgtrans_topic << "/camera";
      mirror_topic << "/mirrorConfig";
      vision_topic << "/visionHSVConfig";
      
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
      mirror_topic << "minho_gazebo_robot" << std::to_string(robot_id) << "/mirrorConfig";
      vision_topic << "minho_gazebo_robot" << std::to_string(robot_id) << "/visionHSVConfig";
   }
   

   //Initialize ROS
   int argc = 0;
   ros::init(argc, NULL, asd.toStdString().c_str(),ros::init_options::NoSigintHandler);
   _node_ = new ros::NodeHandle();
   it_ = new image_transport::ImageTransport(*_node_);
   imgreq_pub_ = _node_->advertise<imgRequest>(imreq_topic.str().c_str(),100);
   mirror_pub_ = _node_->advertise<mirrorConfig>(mirror_topic.str().c_str(),100);
   vision_pub_ = _node_->advertise<visionHSVConfig>(vision_topic.str().c_str(),100);
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
   temp = Mat(480,480,CV_8UC3,Scalar(0,0,0));
}
   
   
void MainWindow::keyPressEvent(QKeyEvent *event)
{
   switch(event->key()){
      // Mode change
      case Qt::Key_R:{ //change mode to raw
         ui->combo_aqtype->setCurrentIndex(0);
         break;
      }
      case Qt::Key_W:{ //change mode to world
         ui->combo_aqtype->setCurrentIndex(2);
         break;
      }
      case Qt::Key_S:{ //change mode to segmented
         ui->combo_aqtype->setCurrentIndex(1);
         break;
      }
      case Qt::Key_M:{ //change mode to map
         ui->combo_aqtype->setCurrentIndex(3);
         break;
      }
      case Qt::Key_F:{ //feed - multiple images
         ui->radio_multiple->setChecked(true);
         on_bt_grab_clicked();
         break;
      }
      case Qt::Key_I:{ //increase fps
         ui->spin_framerate->setValue(ui->spin_framerate->value()+1);
         on_bt_grab_clicked();
         break;
      }
      case Qt::Key_D:{ //decrease fps
         ui->spin_framerate->setValue(ui->spin_framerate->value()-1);
         on_bt_grab_clicked();
         break;
      }
      case Qt::Key_G:{ //grab single image
         // Single image
         ui->radio_single->setChecked(true);
         on_bt_grab_clicked();
         break;
      }
      case Qt::Key_T:{ //turn class mode on or off
         if(!calibration_mode){
            calibration_mode = true;
            ui->lb_robot_name->setStyleSheet("QLabel { color : green; }");
            img_calib_timer->start(30);
         } else {
            calibration_mode = false;
            ui->lb_robot_name->setStyleSheet("QLabel { color : red; }");
            img_calib_timer->stop();
         }
         break;
      }
   }
}
void MainWindow::display_image(const sensor_msgs::ImageConstPtr& msg)
{
   cv_bridge::CvImagePtr recv_img = cv_bridge::toCvCopy(msg,"bgr8");
   temp = recv_img->image;
   image_ =  QImage( temp.data,
                 temp.cols, temp.rows,
                 static_cast<int>(temp.step),
                 QImage::Format_RGB888 );

   emit addNewImage();
}

void MainWindow::addImageToScene()
{
   if(!calibration_mode){
      scene_->clear();
      scene_->addPixmap(QPixmap::fromImage(image_));
   }
}

void MainWindow::applyBinary()
{
   LABEL_t label = static_cast<LABEL_t>(ui->combo_label->currentIndex());
   //Process binary image
   Mat binary = temp.clone();
   img_calib_->getBinary(&binary,img_calib_->getLabelConfiguration(label));
   //Display Image
   image_ =  QImage( binary.data,
                     binary.cols, binary.rows,
                     static_cast<int>(binary.step),
                     QImage::Format_RGB888 ); 
   scene_->clear();
   scene_->addPixmap(QPixmap::fromImage(image_));
}

label MainWindow::initLabelConfig(LABEL_t _label,labelConfiguration label_conf)
{
   label msg;
   range h,s,v;
   
   h.min = label_conf.lb_calib[H][MIN]; h.max= label_conf.lb_calib[H][MAX];
   s.min = label_conf.lb_calib[S][MIN]; s.max= label_conf.lb_calib[S][MAX];
   v.min = label_conf.lb_calib[V][MIN]; v.max= label_conf.lb_calib[V][MAX];
   
   msg.classifier = (int)pow(2,(float)_label);
   msg.H = h; msg.S = s; msg.V = v;
   return msg;
}
// BUTTONS
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

void MainWindow::on_bt_setdist_clicked()
{
   QStringList pixValues = ui->line_pixdist->text().split(",");
   vector<short unsigned int> values; values.clear();
   int targetValues = ui->spin_maxdist->value()/ui->spin_step->value();
   bool bad_configuration = false;
   int bad_conf_type = 0;
   if(targetValues!=pixValues.size()) { 
      bad_configuration = true; bad_conf_type = 1; 
   } else {
      for(unsigned int val=0;val<pixValues.size();val++)
         values.push_back(pixValues[val].toInt());
         
      unsigned int i = 0;
      while( ((i+1)<values.size()) && !bad_configuration){
         if(values[i]>values[i+1]){
            bad_configuration = true; bad_conf_type = 2;
         }
         i++;  
      }
   }
   
   if(bad_configuration){
      ROS_ERROR("Bad mirror configuration (%d)",bad_conf_type);
      QString err;
      if(bad_conf_type==1) err = "Wrong number of distance values.\n"
      +QString::number(targetValues)+" arguments needed but "
      +QString::number(pixValues.size())+" were provided";
      else err = "Wrong sequence of distance values.";
      QMessageBox::critical(NULL, QObject::tr("Bad Distances Configuration"),
         QObject::tr(err.toStdString().c_str()));
   }else {
      ROS_INFO("Correct mirror configuration sent!");
      mirrorConfig msg;
      msg.max_distance = ui->spin_maxdist->value();
      msg.step = ui->spin_step->value();
      msg.pixel_distances = values;
      mirror_pub_.publish(msg);
   }
}

void MainWindow::on_bt_setlut_clicked()
{
   labelConfiguration field_conf = img_calib_->getLabelConfiguration(FIELD);
   labelConfiguration line_conf = img_calib_->getLabelConfiguration(LINE);
   labelConfiguration ball_conf = img_calib_->getLabelConfiguration(BALL);
   labelConfiguration obstacle_conf = img_calib_->getLabelConfiguration(OBSTACLE);
   
   visionHSVConfig msg;
   msg.field = initLabelConfig(FIELD,field_conf);
   msg.line = initLabelConfig(LINE,line_conf);
   msg.ball = initLabelConfig(BALL,ball_conf);
   msg.obstacle = initLabelConfig(OBSTACLE,obstacle_conf);
   vision_pub_.publish(msg);
   ROS_INFO("Correct vision configuration sent!");
}
//SLIDEBARS
void MainWindow::on_h_min_valueChanged(int value)
{
   ui->lb_hmin->setText(QString::number(value));
   img_calib_->updateCurrentConfiguration(static_cast<LABEL_t>(ui->combo_label->currentIndex())
                                          ,H
                                          ,MIN
                                          ,value);
   
}

void MainWindow::on_h_max_valueChanged(int value)
{
   ui->lb_hmax->setText(QString::number(value));
   img_calib_->updateCurrentConfiguration(static_cast<LABEL_t>(ui->combo_label->currentIndex())
                                          ,H
                                          ,MAX
                                          ,value);
}

void MainWindow::on_s_min_valueChanged(int value)
{
   ui->lb_smin->setText(QString::number(value));
   img_calib_->updateCurrentConfiguration(static_cast<LABEL_t>(ui->combo_label->currentIndex())
                                          ,S
                                          ,MIN
                                          ,value);
   
}

void MainWindow::on_s_max_valueChanged(int value)
{
   ui->lb_smax->setText(QString::number(value));
   img_calib_->updateCurrentConfiguration(static_cast<LABEL_t>(ui->combo_label->currentIndex())
                                          ,S
                                          ,MAX
                                          ,value);
}

void MainWindow::on_v_min_valueChanged(int value)
{
   ui->lb_vmin->setText(QString::number(value));
   img_calib_->updateCurrentConfiguration(static_cast<LABEL_t>(ui->combo_label->currentIndex())
                                          ,V
                                          ,MIN
                                          ,value);
}

void MainWindow::on_v_max_valueChanged(int value)
{
   ui->lb_vmax->setText(QString::number(value));
   img_calib_->updateCurrentConfiguration(static_cast<LABEL_t>(ui->combo_label->currentIndex())
                                          ,V
                                          ,MAX
                                          ,value);
}
//COMBOBOXES
void MainWindow::on_combo_label_currentIndexChanged(int index)
{
   LABEL_t label = static_cast<LABEL_t>(index);
   loadValuesOnTrackbars(img_calib_->getLabelConfiguration(label));
   
}
void MainWindow::loadValuesOnTrackbars(labelConfiguration labelconf)
{
   ui->h_min->setValue(labelconf.lb_calib[H][MIN]);
   ui->h_max->setValue(labelconf.lb_calib[H][MAX]);
   ui->s_min->setValue(labelconf.lb_calib[S][MIN]);
   ui->s_max->setValue(labelconf.lb_calib[S][MAX]);
   ui->v_min->setValue(labelconf.lb_calib[V][MIN]);
   ui->v_max->setValue(labelconf.lb_calib[V][MAX]);
}

