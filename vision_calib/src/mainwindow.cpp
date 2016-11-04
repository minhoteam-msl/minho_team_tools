#include "mainwindow.h"
#include "ui_vision_calib.h"

/// \brief class constructor. Takes care of initializing GUI,
/// connect several signals/slots and to create and configure
/// all ROS publishers and subscribers.
/// \param robot_id - id [0-6] of the robot to use
/// \param real_robot - defines wether its a real or sim robot
MainWindow::MainWindow(int robot_id, bool real_robot, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
   ui->setupUi(this);
   setup();
   
   robot_id_ = robot_id;
   calibration_mode = draw_mode = false;
   img_calib_timer = new QTimer();
   interaction_timer = new QTimer();
   img_calib_ = new ImageCalibrator();
   ui->lb_robot_name->setStyleSheet("QLabel { color : red; }");
   ui->lb_robot_name->setText(QString("Calibration Mode for Robot ")+QString::number(robot_id_));
   connect(this,SIGNAL(addNewImage()),this,SLOT(addImageToScene()));
   ui->graphicsView->setHorizontalScrollBarPolicy ( Qt::ScrollBarAlwaysOff );
	ui->graphicsView->setVerticalScrollBarPolicy ( Qt::ScrollBarAlwaysOff );
	connect(img_calib_timer,SIGNAL(timeout()),this,SLOT(applyBinary()));
	connect(interaction_timer,SIGNAL(timeout()),this,SLOT(interactWithUser()));
   
   // Setup of ROS
   QString asd = "Vision_calib";
   asd.append(QString::number(robot_id));
   asd.append("_");
   asd.append(QString::number(std::time(0)));
   std::stringstream imreq_topic;
   std::stringstream imgtrans_topic;
   std::stringstream mirror_topic;
   std::stringstream vision_topic;
   std::stringstream image_topic;
   
   if(!real_robot){
      // Setup ROS Node and pusblishers/subscribers in SIMULATOR
      imreq_topic << "minho_gazebo_robot" << std::to_string(robot_id);
      imgtrans_topic << "minho_gazebo_robot" << std::to_string(robot_id);
      mirror_topic << "minho_gazebo_robot" << std::to_string(robot_id);
      vision_topic << "minho_gazebo_robot" << std::to_string(robot_id);
      image_topic << "minho_gazebo_robot" << std::to_string(robot_id);
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
   
   imreq_topic << "/imgRequest";
   imgtrans_topic << "/camera/image";
   mirror_topic << "/mirrorConfig";
   vision_topic << "/visionHSVConfig";
   image_topic << "/imageConfig";
      

   //Initialize ROS
   int argc = 0;
   ros::init(argc, NULL, asd.toStdString().c_str(),ros::init_options::NoSigintHandler);
   _node_ = new ros::NodeHandle();
   it_ = new image_transport::ImageTransport(*_node_);
   mirror_pub_ = _node_->advertise<mirrorConfig>(mirror_topic.str().c_str(),100);
   vision_pub_ = _node_->advertise<visionHSVConfig>(vision_topic.str().c_str(),100);
   image_pub_ = _node_->advertise<imageConfig>(image_topic.str().c_str(),100);
   
   omniVisionConf = _node_->serviceClient<minho_team_ros::requestOmniVisionConf>("requestOmniVisionConf");
   imgRequest = _node_->serviceClient<minho_team_ros::requestImage>("requestImage");
   //Initialize image_transport subscriber
   image_transport::TransportHints hints("compressed", ros::TransportHints());
   image_sub_ = it_->subscribe(imgtrans_topic.str().c_str(),1,&MainWindow::imageCallback,this,hints);
   
   
   //Request Current Configuration
   requestOmniVisionConf srv; 
   srv.request.request_node_name = asd.toStdString();
   if(omniVisionConf.call(srv)){
      ROS_INFO("Retrieved configuration from target robot.");
      img_calib_->mirrorConfigFromMsg(srv.response.mirrorConf);
      img_calib_->lutConfigFromMsg(srv.response.visionConf);
      img_calib_->imageConfigFromMsg(srv.response.imageConf);
      loadValuesOnTrackbars(img_calib_->getLabelConfiguration(static_cast<LABEL_t>(ui->combo_label->currentIndex())));
      loadMirrorValues(srv.response.mirrorConf);
      loadImageValues(srv.response.imageConf);
      
   } else ROS_ERROR("Failed to retrieve configuration from target robot.");
   
   interaction_timer->start(50);
}

/// \brief - class destructor. Shuts down ROS node
MainWindow::~MainWindow()
{  
   ros::shutdown();
   delete ui;
}

/// \brief function to setup the graphicsscene and create the 
/// image holding matrix
void MainWindow::setup()
{
   scene_ = new QGraphicsScene();
   ui->graphicsView->setScene(scene_); 
   temp = Mat(480,480,CV_8UC3,Scalar(0,0,0));
}
   
/// \brief function to detect key press events. This key presses 
/// detection allow to implement several actions without mouse usage
/// \param event - QKeyEvent holding the data about the key press event   
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
         calibration_mode = false;
         ui->lb_robot_name->setStyleSheet("QLabel { color : red; }");
         img_calib_timer->stop();
         interaction_timer->start(50);
         break;
      }
      case Qt::Key_S:{ //change mode to segmented
         ui->combo_aqtype->setCurrentIndex(1);
         calibration_mode = false;
         ui->lb_robot_name->setStyleSheet("QLabel { color : red; }");
         img_calib_timer->stop();
         interaction_timer->start(50);
         break;
      }
      case Qt::Key_M:{ //change mode to map
         ui->combo_aqtype->setCurrentIndex(3);
         calibration_mode = false;
         ui->lb_robot_name->setStyleSheet("QLabel { color : red; }");
         img_calib_timer->stop();
         interaction_timer->start(50);
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
            interaction_timer->stop();
         } else {
            calibration_mode = false;
            ui->lb_robot_name->setStyleSheet("QLabel { color : red; }");
            img_calib_timer->stop();
            interaction_timer->start(50);
         }
         ui->combo_aqtype->setCurrentIndex(0); // Raw mode when
         on_bt_grab_clicked();
         break;
      }
   }
}

/// \brief callback function that receives published images and converts them 
/// into Opencv(Mat) format using cv_bridge and to Qt's QImage to be displayed.
/// Since its called by ROS (another thread), emits a signal (addNewImage) so
/// the GUI's thread can take care of putting the image in GraphicsView.
/// \param msg - sensor_msgs::Image data, holding information about
/// the image sent.
void MainWindow::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
   cv_bridge::CvImagePtr recv_img = cv_bridge::toCvCopy(msg,"bgr8");
   temp = recv_img->image.clone();
   image_ =  QImage( recv_img->image.data,
                 recv_img->image.cols, recv_img->image.rows,
                 static_cast<int>(recv_img->image.step),
                 QImage::Format_RGB888).rgbSwapped();
   emit addNewImage();
}

/// \brief slot function that consumes addNewImage signal. Only displays the 
/// image if both calibration modes are disabled (calibration_mode and draw_mode)
void MainWindow::addImageToScene()
{
   if(!calibration_mode && !draw_mode){
      scene_->clear();
      scene_->addPixmap(QPixmap::fromImage(image_));
   }
}

/// \brief function to apply binary HSV masking to the selected label, with the
/// defined parameters for that label, in GUI's trackbars. The conversion is done
/// based on the image present in temp matrix
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
                     QImage::Format_RGB888);
   scene_->clear();
   scene_->addPixmap(QPixmap::fromImage(image_));
}

/// \brief function to interact with the user and display info/images. If in draw
/// mode it displays a calibration crosshair and, if in inspection mode, displays
/// the mapped distance for the selected (mouse hovering) pixel
void MainWindow::interactWithUser()
{
   if(draw_mode){
      if(ui->check_draw->isChecked()){
      //Draw cross
         Mat draw = temp.clone();
         img_calib_->drawCenter(&draw);
         image_ =  QImage(draw.data,
                 draw.cols, draw.rows,
                 static_cast<int>(draw.step),
                 QImage::Format_RGB888).rgbSwapped();
         scene_->clear();
         scene_->addPixmap(QPixmap::fromImage(image_));
      }  
   } 

   QPointF relativeOrigin = ui->graphicsView->mapToScene(ui->graphicsView->mapFromGlobal(QCursor::pos()));
   if(relativeOrigin.x()>=0 && relativeOrigin.x()<480 && relativeOrigin.y()>=0 && relativeOrigin.y()<480){
      ui->lb_pxcoords->setText(QString("(")+QString::number(relativeOrigin.x())+QString(",")+QString::number(relativeOrigin.y())+QString(") px"));
      Point2d real_coords = img_calib_->worldMapping(Point(relativeOrigin.x(),relativeOrigin.y()));
      ui->lb_realcoords->setText(QString::number(real_coords.x,'f',2)+QString(" m , ")+QString::number(real_coords.y,'f',2)+QString("º"));
   }
}

// BUTTONS
/// \brief slot function for click action of bt_grab. Function to send a request
/// for images based on selected options of type and frequency
void MainWindow::on_bt_grab_clicked()
{
   bool multiple_send_request = ui->radio_multiple->isChecked();
   minho_team_ros::requestImage srv;
   srv.request.is_multiple = multiple_send_request;
   srv.request.frequency = ui->spin_framerate->value();
   srv.request.type = (int)pow(2,ui->combo_aqtype->currentIndex());
   imgRequest.call(srv);
}

/// \brief slot function for click action of bt_stop. Function to stop image 
/// send. It send a single image requst
void MainWindow::on_bt_stop_clicked()
{
   minho_team_ros::requestImage srv;
   srv.request.is_multiple = false;
   srv.request.frequency = ui->spin_framerate->value();
   srv.request.type = (int)pow(2,ui->combo_aqtype->currentIndex());
   imgRequest.call(srv);
}

/// \brief slot function for click action of bt_setdist. Checks validaty and
/// sends new configuration for mirror's pixel-meter mapping
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

/// \brief slot function for click action of bt_setlut. Sends new configuration
/// for vision's HSV look up table
void MainWindow::on_bt_setlut_clicked()
{
   vision_pub_.publish(img_calib_->getLutConfiguration());
   ROS_INFO("Correct vision configuration sent!");
}

/// \brief slot function for click action of bt_setimg. Sends new configuraiton
/// for image parameters
void MainWindow::on_bt_setimg_clicked()
{
   image_pub_.publish(img_calib_->getImageConfiguration());
   ROS_INFO("Correct image configuration sent!");
}

/// \brief slot function for click action of bt_screenshot. Takes and saves a
/// screenshot in {ros_project_folder}/screenshots
void MainWindow::on_bt_screenshot_clicked()
{
   QString path = QString(getenv("HOME"))+QString("/catkin_ws/src/minho_team_tools/vision_calib/screenshots/");
   QString file = "Screenshot_";
   file.append(QString::number(std::time(0))); 
   file.append(".png");
   path+=file;
   ROS_INFO("Saved screenshot to %s",path.toStdString().c_str());
   imwrite(path.toStdString().c_str(),temp);
}

/// \brief slot function for click action of check_draw. Enbles or disables 
/// draw mode
/// \param state - boolean value to define if draw mode is on or off
void MainWindow::on_check_draw_clicked(bool state)
{
   draw_mode = state;
}

//SLIDEBARS
/// \brief slot function for hue min trackbar. Sets a new value for the selected
/// label
/// \param value - new value to be applied to label's hue min
void MainWindow::on_h_min_valueChanged(int value)
{
   ui->lb_hmin->setText(QString::number(value));
   img_calib_->updateCurrentConfiguration(static_cast<LABEL_t>(ui->combo_label->currentIndex())
                                          ,H
                                          ,MIN
                                          ,value);
   
}

/// \brief slot function for hue max trackbar. Sets a new value for the selected
/// label
/// \param value - new value to be applied to label's hue max
void MainWindow::on_h_max_valueChanged(int value)
{
   ui->lb_hmax->setText(QString::number(value));
   img_calib_->updateCurrentConfiguration(static_cast<LABEL_t>(ui->combo_label->currentIndex())
                                          ,H
                                          ,MAX
                                          ,value);
}

/// \brief slot function for sat min trackbar. Sets a new value for the selected
/// label
/// \param value - new value to be applied to label's sat min
void MainWindow::on_s_min_valueChanged(int value)
{
   ui->lb_smin->setText(QString::number(value));
   img_calib_->updateCurrentConfiguration(static_cast<LABEL_t>(ui->combo_label->currentIndex())
                                          ,S
                                          ,MIN
                                          ,value);
   
}

/// \brief slot function for sat max trackbar. Sets a new value for the selected
/// label
/// \param value - new value to be applied to label's sat max
void MainWindow::on_s_max_valueChanged(int value)
{
   ui->lb_smax->setText(QString::number(value));
   img_calib_->updateCurrentConfiguration(static_cast<LABEL_t>(ui->combo_label->currentIndex())
                                          ,S
                                          ,MAX
                                          ,value);
}

/// \brief slot function for value min trackbar. Sets a new value for the selected
/// label
/// \param value - new value to be applied to label's value min
void MainWindow::on_v_min_valueChanged(int value)
{
   ui->lb_vmin->setText(QString::number(value));
   img_calib_->updateCurrentConfiguration(static_cast<LABEL_t>(ui->combo_label->currentIndex())
                                          ,V
                                          ,MIN
                                          ,value);
}

/// \brief slot function for value max trackbar. Sets a new value for the selected
/// label
/// \param value - new value to be applied to label's value max
void MainWindow::on_v_max_valueChanged(int value)
{
   ui->lb_vmax->setText(QString::number(value));
   img_calib_->updateCurrentConfiguration(static_cast<LABEL_t>(ui->combo_label->currentIndex())
                                          ,V
                                          ,MAX
                                          ,value);
}

//SPINBOXES
/// \brief slot function for tilt spinbox. Sets a new value for the image 
/// tilt
/// \param value - new value to be applied image's tiltilt
void MainWindow::on_spin_tilt_valueChanged(int value)
{
   imageConfig msg;
   msg.center_x = ui->spin_cx->value();
   msg.center_y = ui->spin_cy->value();
   msg.tilt = value;
   img_calib_->imageConfigFromMsg(msg);
}

/// \brief slot function for center x spinbox. Sets a new value for the image 
/// tilt
/// \param value - new value to be applied image's center x
void MainWindow::on_spin_cx_valueChanged(int value)
{
   imageConfig msg;
   msg.center_x = value;
   msg.center_y = ui->spin_cy->value();
   msg.tilt = ui->spin_tilt->value();
   img_calib_->imageConfigFromMsg(msg);
}

/// \brief slot function for center y spinbox. Sets a new value for the image 
/// tilt
/// \param value - new value to be applied image's center y
void MainWindow::on_spin_cy_valueChanged(int value)
{
   imageConfig msg;
   msg.center_x = ui->spin_cx->value();
   msg.center_y = value;
   msg.tilt = ui->spin_tilt->value();
   img_calib_->imageConfigFromMsg(msg);
}
   
//COMBOBOXES
/// \brief slot function for label's combobox. Sets the current label that
/// we will be performing the configurations
/// \param index - new label index to be used
void MainWindow::on_combo_label_currentIndexChanged(int index)
{
   LABEL_t label = static_cast<LABEL_t>(index);
   loadValuesOnTrackbars(img_calib_->getLabelConfiguration(label));
   
}

/// \brief slot function for acquisition type's combobox. Sets the current 
/// acquisition type to be used in image requests
/// \param index - new image acquisition type to be used
void MainWindow::on_combo_aqtype_currentIndexChanged(int index)
{
   on_bt_grab_clicked();   
}

//LOAD FUNCTIONS
/// \brief function to load values of a label configuration onto GUI's 
/// trackbars
/// \param labelconf - label message containing label configuration
void MainWindow::loadValuesOnTrackbars(minho_team_ros::label labelconf)
{
   ui->h_min->setValue(labelconf.H.min);
   ui->h_max->setValue(labelconf.H.max);
   ui->s_min->setValue(labelconf.S.min);
   ui->s_max->setValue(labelconf.S.max);
   ui->v_min->setValue(labelconf.V.min);
   ui->v_max->setValue(labelconf.V.max);
}

/// \brief function to load values of a mirror configuration onto GUI's 
/// spinboxes and line edit
/// \param mirrorConf - mirrorConfig message to be used
void MainWindow::loadMirrorValues(minho_team_ros::mirrorConfig mirrorConf)
{
   //mirrorConfig
   ui->spin_step->setValue(mirrorConf.step);
   ui->spin_maxdist->setValue(mirrorConf.max_distance);
   QString distances = "";
   for(unsigned int i=0;i<mirrorConf.pixel_distances.size();i++)
      distances+=QString::number(mirrorConf.pixel_distances[i])
               + QString(",");
   distances.remove(distances.size()-1,1);
   ui->line_pixdist->setText(distances);
}

/// \brief function to load values of a image configuration onto GUI's 
/// spinboxes
/// \param imageConf - imageConfig message to be used
void MainWindow::loadImageValues(minho_team_ros::imageConfig imageConf)
{
   //mirrorConfig
   ui->spin_tilt->setValue(imageConf.tilt);
   ui->spin_cx->setValue(imageConf.center_x);
   ui->spin_cy->setValue(imageConf.center_y);
}

