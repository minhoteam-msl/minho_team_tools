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

   //INITIALIZATIONS
   robot_id_ = robot_id;
   calibration_mode = draw_mode = false;
   ui->lb_robot_name->setStyleSheet("QLabel { color : red; }");
   ui->lb_robot_name->setText(QString("Calibration Mode for Robot ")+QString::number(robot_id_));
   ui->lb_robot_name_2=ui->lb_robot_name;
   ui->graphicsView->setHorizontalScrollBarPolicy ( Qt::ScrollBarAlwaysOff );
   ui->graphicsView->setVerticalScrollBarPolicy ( Qt::ScrollBarAlwaysOff );
   ui->tabWidget->setCurrentIndex(0);
   on_radio_property_clicked();
   
   //TIMERS
   img_calib_timer = new QTimer();
   interaction_timer = new QTimer();
   
   //IMAGECALIBRATOR
   img_calib_ = new ImageCalibrator();
   
   //SIGNAL
   connect(this,SIGNAL(addNewImage()),this,SLOT(addImageToScene()));
   connect(this,SIGNAL(addError(int)),this,SLOT(drawrealError(int)));
   connect(img_calib_timer,SIGNAL(timeout()),this,SLOT(applyBinary()));
   connect(interaction_timer,SIGNAL(timeout()),this,SLOT(interactWithUser()));
   connect(ui->tabWidget, SIGNAL(currentChanged(int)), this, SLOT(tabSelected()));
   
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
   std::stringstream property_topic;
   std::stringstream pid_topic;
   std::stringstream error_topic;
   std::stringstream roi_topic;

   if(!real_robot){
      // Setup ROS Node and pusblishers/subscribers in SIMULATOR
      imreq_topic << "minho_gazebo_robot" << std::to_string(robot_id);
      imgtrans_topic << "minho_gazebo_robot" << std::to_string(robot_id);
      mirror_topic << "minho_gazebo_robot" << std::to_string(robot_id);
      vision_topic << "minho_gazebo_robot" << std::to_string(robot_id);
      image_topic << "minho_gazebo_robot" << std::to_string(robot_id);
      property_topic << "minho_gazebo_robot" << std::to_string(robot_id);
      pid_topic << "minho_gazebo_robot" << std::to_string(robot_id);
      error_topic << "minho_gazebo_robot" << std::to_string(robot_id);
      roi_topic << "minho_gazebo_robot" << std::to_string(robot_id);
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
   property_topic << "/cameraProperty";
   pid_topic << "/PID";
   error_topic << "/ControlerError";
   roi_topic << "/ROI";
      

   //Initialize ROS
   int argc = 0;
   ros::init(argc, NULL, asd.toStdString().c_str(),ros::init_options::NoSigintHandler);
   _node_ = new ros::NodeHandle();
   it_ = new image_transport::ImageTransport(*_node_);
   
   // Creating Pusblishers
   mirror_pub_ = _node_->advertise<mirrorConfig>(mirror_topic.str().c_str(),100);
   vision_pub_ = _node_->advertise<visionHSVConfig>(vision_topic.str().c_str(),100);
   image_pub_ = _node_->advertise<imageConfig>(image_topic.str().c_str(),100);
   property_pub_ = _node_->advertise<cameraProperty>(property_topic.str().c_str(),100);
   pid_pub_ = _node_->advertise<PID>(pid_topic.str().c_str(),100);
   roi_pub_ = _node_->advertise<ROI>(roi_topic.str().c_str(),10);
   
   // Initializing service Clients
   pidConf = _node_->serviceClient<minho_team_ros::requestCamPID>("requestCamPID");
   propertyConf = _node_->serviceClient<minho_team_ros::requestCamProperty>("requestCamProperty");
   omniVisionConf = _node_->serviceClient<minho_team_ros::requestOmniVisionConf>("requestOmniVisionConf");
   imgRequest = _node_->serviceClient<minho_team_ros::requestImage>("requestImage");
   roiConf= _node_->serviceClient<minho_team_ros::requestROI>("requestROI");
   
   //Initialize image_transport and error subscriber
   image_transport::TransportHints hints("compressed", ros::TransportHints());
   image_sub_ = it_->subscribe(imgtrans_topic.str().c_str(),1,&MainWindow::imageCallback,this,hints);
   error_sub_ = _node_->subscribe(error_topic.str().c_str(),1,&MainWindow::drawCallback,this);   
   
   //Request Current Property Configuration
   loadPropertiesValues(0);
   set_ROIs();

   //Request Current Mirror, LUT, IMAGE Configurations
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
   scene_2= new QGraphicsScene();
   ui->graphicsView->setScene(scene_);
   ui->graphicsView_2->setScene(scene_2);
   calib=false;
   configura_plot();
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
   if(ui->tabWidget->currentIndex()==0){
	   if(!calibration_mode && !draw_mode){
		  scene_->clear();
		  scene_->addPixmap(QPixmap::fromImage(image_));
	   }
   }
   else {
	    scene_2->clear();
	    QPainter p(&image_);
	    p.setPen(Qt::red);
	    p.drawRect(ui->spinBox_x_w->value(),ui->spinBox_y_w->value(),ui->spinBox_d_w->value(),ui->spinBox_d_w->value());
	    p.setPen(Qt::blue);
	    p.drawRect(ui->spinBox_x_b->value(),ui->spinBox_y_b->value(),ui->spinBox_d_b->value(),ui->spinBox_d_b->value());
	    scene_2->addPixmap(QPixmap::fromImage(image_));
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
   
   if(ui->tabWidget->currentIndex()==1){
	   QPointF relativeOrigin = ui->graphicsView_2->mapToScene(ui->graphicsView_2->mapFromGlobal(QCursor::pos()));
	   if(relativeOrigin.x()>=0 && relativeOrigin.x()<480 && relativeOrigin.y()>=0 && relativeOrigin.y()<480){
		  ui->lb_pxcoords_2->setText(QString("(")+QString::number(relativeOrigin.x())+QString(",")+QString::number(relativeOrigin.y())+QString(") px"));
		  Point2d real_coords = img_calib_->worldMapping(Point(relativeOrigin.x(),relativeOrigin.y()));
		  ui->lb_realcoords_2->setText(QString::number(real_coords.x,'f',2)+QString(" m , ")+QString::number(real_coords.y,'f',2)+QString("º"));
	   	}

	}
      imageConfig imgconf = img_calib_->getImageConfiguration();
      ui->lb_distpx->setText(QString::number(sqrt((relativeOrigin.x()-imgconf.center_x)*(relativeOrigin.x()-imgconf.center_x)
      +(relativeOrigin.y()-imgconf.center_y)*(relativeOrigin.y()-imgconf.center_y))));
      ui->lb_distpx_2->setText(ui->lb_distpx->text());
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
   srv.request.type = ui->combo_aqtype->currentIndex();
   imgRequest.call(srv);
   
   this->centralWidget()->setFocus();
}

/// \brief slot function for click action of bt_stop. Function to stop image 
/// send. It send a single image requst
void MainWindow::on_bt_stop_clicked()
{
   minho_team_ros::requestImage srv;
   srv.request.is_multiple = false;
   srv.request.frequency = ui->spin_framerate->value();
   srv.request.type = ui->combo_aqtype->currentIndex();
   imgRequest.call(srv);
   
   this->centralWidget()->setFocus();
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
   
   this->centralWidget()->setFocus();
}

/// \brief slot function for click action of bt_setlut. Sends new configuration
/// for vision's HSV look up table
void MainWindow::on_bt_setlut_clicked()
{
   vision_pub_.publish(img_calib_->getLutConfiguration());
   ROS_INFO("Correct vision configuration sent!");
   
   this->centralWidget()->setFocus();
}

/// \brief slot function for click action of bt_setimg. Sends new configuraiton
/// for image parameters
void MainWindow::on_bt_setimg_clicked()
{
   image_pub_.publish(img_calib_->getImageConfiguration());
   ROS_INFO("Correct image configuration sent!");
   
   this->centralWidget()->setFocus();
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
   
   this->centralWidget()->setFocus();
}

/// \brief slot function for click action of bt_screenshot. Takes and saves a
/// screenshot in {ros_project_folder}/screenshots
void MainWindow::on_bt_screenshot_2_clicked()
{
   QString path = QString(getenv("HOME"))+QString("/catkin_ws/src/minho_team_tools/vision_calib/screenshots/");
   QString file = "Screenshot_CamP_";
   file.append(QString::number(std::time(0))); 
   file.append(".png");
   path+=file;
   ROS_INFO("Saved screenshot to %s",path.toStdString().c_str());
   imwrite(path.toStdString().c_str(),temp);
   
   this->centralWidget()->setFocus();
}

/// \brief slot function for click action of bt_calib. This enables or disables
/// the calibration (LUT) of the received image
void MainWindow::on_bt_calib_clicked()
{
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
   
   this->centralWidget()->setFocus();
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
   
   this->centralWidget()->setFocus();
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
   
   this->centralWidget()->setFocus();
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
   
   this->centralWidget()->setFocus();
}
   
//COMBOBOXES
/// \brief slot function for label's combobox. Sets the current label that
/// we will be performing the configurations
/// \param index - new label index to be used
void MainWindow::on_combo_label_currentIndexChanged(int index)
{
   LABEL_t label = static_cast<LABEL_t>(index);
   loadValuesOnTrackbars(img_calib_->getLabelConfiguration(label));
   
   this->centralWidget()->setFocus();
   
}

/// \brief slot function for acquisition type's combobox. Sets the current 
/// acquisition type to be used in image requests
/// \param index - new image acquisition type to be used
void MainWindow::on_combo_aqtype_currentIndexChanged(int index)
{
   on_bt_grab_clicked(); 
   int id = ui->combo_aqtype->currentIndex();
   if(id>0){
      calibration_mode = false;
      ui->lb_robot_name->setStyleSheet("QLabel { color : red; }");
      img_calib_timer->stop();
      interaction_timer->start(50);
   }  
   
   this->centralWidget()->setFocus();
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


/*---------------------------------FUNÇOES DO PLOT REAL TIME-------------------------------------*/
/*----------------------------------------------------------------------------------------------*/
//FUNCTION THAT CONFIGURES PLOT
void MainWindow::configura_plot()
{

    ui->error_plot->addGraph();
    ui->error_plot->graph(0)->setPen(QPen(Qt::red));
    ui->error_plot->graph(0)->setAntialiasedFill(false);
    ui->error_plot->yAxis->setLabel("Error");
    ui->error_plot->xAxis->setLabel("Tempo (s)");

     ui->error_plot->xAxis->setTickLabelType(QCPAxis::ltDateTime);
     ui->error_plot->xAxis->setDateTimeFormat("mm:ss");
     ui->error_plot->xAxis->setAutoTickStep(true);
     ui->error_plot->xAxis->setTickStep(1);
     ui->error_plot->axisRect()->setupFullAxesBox();

     // make left and bottom axes transfer their ranges to right and top axes:
     connect(ui->error_plot->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->error_plot->xAxis2, SLOT(setRange(QCPRange)));
     connect(ui->error_plot->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->error_plot->yAxis2, SLOT(setRange(QCPRange)));
}


void MainWindow::drawrealError(int num)
{
	
    static int key_x=0;
	
    ui->error_plot->graph(0)->addData(key_x, num);
    // remove data of lines that's outside visible range:
    ui->error_plot->graph(0)->removeDataBefore(key_x-1000);
    // rescale value (vertical) axis to fit the current data:
    ui->error_plot->graph(0)->rescaleValueAxis();
    ui->error_plot->xAxis->setRange(key_x+1, 1000, Qt::AlignRight);
    ui->error_plot->replot();
    key_x+=2;
}
/*----------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------*/

/// \brief slot function for properties type's combobox. Sets the current 
/// property to be calibrated
/// \param index - new property type to be used
void MainWindow::on_combo_properties_currentIndexChanged(int index)
{
	loadPropertiesValues(index);
	configura_plot();
   	this->centralWidget()->setFocus();
}

/// \brief slot function for click action of bt_campid. Sends
/// the property PID Controler values
void MainWindow::on_bt_campid_clicked()
{
    PID msg;
    msg.property_id = ui->combo_properties->currentIndex();
    if(msg.property_id==6){
	   msg.property_id = 5;
	   msg.blue=true;
    }
    else msg.blue=false;
    msg.p =(float) (ui->Kp_trackbar->value()/temp_k_div);
    msg.d =(float) (ui->Kd_trackbar->value()/temp_k_div);
    msg.i =(float) (ui->Ki_trackbar->value()/temp_k_div);
    pid_pub_.publish(msg);
    
    ui->label_kp_curr_value->setText(QString::number(ui->Kp_trackbar->value()/temp_k_div));
    ui->label_kd_curr_value->setText(QString::number(ui->Kd_trackbar->value()/temp_k_div));
    ui->label_ki_curr_value->setText(QString::number(ui->Ki_trackbar->value()/temp_k_div));
	
}
/// \brief slot function for click action of bt_auto_cal. Sends
/// signal to omniCamera to start the auto-calibration process
void MainWindow::on_bt_auto_cal_clicked()
{
	calib=!calib;
	if(calib) ui->bt_auto_cal->setText("Calibrating");

	else ui->bt_auto_cal->setText("Calibrate");
	
	PID msg;
	msg.calibrate=true;
	pid_pub_.publish(msg);
	
}

/// \brief slot function for click action of bt_campid. Sends
/// signal to reset PID controler of Properties of omniCamera
void MainWindow::on_bt_reset_clicked()
{
	cameraProperty msg;
	msg.reset=true;
	property_pub_.publish(msg);
}

/// \brief slot function for click action of bt_camprop. Sends
/// the property value
void MainWindow::on_bt_camprop_clicked()
{
   cameraProperty msg;
   msg.property_id = ui->combo_properties->currentIndex();
   if(msg.property_id==6){
	   msg.property_id = 5;
	   msg.blue=true;
   }
   else msg.blue=false;
   
   msg.value_a =(float) (ui->prop_trackbar->value()/temp_prop_div);
   property_pub_.publish(msg);
}

/// \brief slot function for check action of radio_PID. Enables and disable
/// some trackbars
void MainWindow::on_radio_PID_clicked()
{
	ui->prop_trackbar->setEnabled(false);
	ui->bt_camprop->setEnabled(false);
	ui->bt_campid->setEnabled(true);
	ui->Kp_trackbar->setEnabled(true);
	ui->Kd_trackbar->setEnabled(true);
	ui->Ki_trackbar->setEnabled(true);
}

/// \brief slot function for check action of radio_property. Enables and disable
/// some trackbars
void MainWindow::on_radio_property_clicked()
{
	ui->prop_trackbar->setEnabled(true);
	ui->bt_camprop->setEnabled(true);
	ui->bt_campid->setEnabled(false);
	ui->Kp_trackbar->setEnabled(false);
	ui->Kd_trackbar->setEnabled(false);
	ui->Ki_trackbar->setEnabled(false);
}

/// \brief slot function for tab change. Enables or disables 
/// components necessary for each tab.
void MainWindow::tabSelected()
{
	if(ui->tabWidget->currentIndex()==0){
		   interaction_timer->start(50);
		   on_bt_grab_clicked();
		}
	if(ui->tabWidget->currentIndex()==1){
		interaction_timer->stop();
		interaction_timer->start(50);
		img_calib_timer->stop();
		configura_plot();
		calibrate_activ();
		}
}

/// \brief function for video activate when tabs change
/// from 0 to 1
void MainWindow::calibrate_activ()
{
	minho_team_ros::requestImage srv;
	srv.request.is_multiple = true;
	srv.request.frequency = 20;
	srv.request.type = 0;
	imgRequest.call(srv);
}

/// \brief function to read properties values from pusblishers
/// \param num - represents index of property
void MainWindow::loadPropertiesValues(int num)
{
   // Camera Properties Configuration
   cameraProperty msg;
   PID msg_pid;
   if(num==6){
	   num=5;
	   msg.blue=true;
	   msg_pid.blue=true;
	   }
   msg.property_id=num;
   requestCamProperty srv_prop;
   srv_prop.request.property=msg;
   srv_prop.request.is_set=false;

   // Properties Gain Values COnfiguration
   requestCamPID srv_pid;
   msg_pid.property_id=num;
   srv_pid.request.parameter=msg_pid;
   srv_pid.request.is_set=false;
   if(!propertyConf.call(srv_prop))ROS_ERROR("Failed to retrieve property values from target robot."); 
   else if(!pidConf.call(srv_pid))ROS_ERROR("Failed to retrieve PID values from target Property.");
		else set_track_ranges(srv_prop.response.property,srv_pid.response.parameterS);
}

/// \brief function to set trackbar ranges of Kp,Ki,Kd,Prop
/// \param prop - new cameraProperty that contains values to be loaded to trackbar of property
/// \param pid - new PID that contains values to be loaded to trackbars of PID
void MainWindow::set_track_ranges(cameraProperty prop,PID pid)
{
	// BRIGHTNESS
	if(prop.property_id==0){
		temp_prop_div=1000;
		ui->prop_trackbar->setRange(5469,29688);
		
		ui->Kp_trackbar->setRange(0,4844);
		ui->Ki_trackbar->setRange(0,4844);
		ui->Kd_trackbar->setRange(0,4844);
		temp_k_div=100;
		}
	// GAIN
	if(prop.property_id==1){
		ui->prop_trackbar->setRange(0,239991);
		temp_prop_div=1000;
		
		ui->Kp_trackbar->setRange(0,38);
		ui->Ki_trackbar->setRange(0,38);
		ui->Kd_trackbar->setRange(0,38);
		temp_k_div=100;
		}
	// SHUTTER(0.046->32.754)
	if(prop.property_id==2){
		ui->prop_trackbar->setRange(46,32754);
		temp_prop_div=1000;
		
		ui->Kp_trackbar->setRange(0,1600);
		ui->Ki_trackbar->setRange(0,1600);
		ui->Kd_trackbar->setRange(0,1600);
		temp_k_div=10000;
		}
	// GAMMA	
	if(prop.property_id==3){
		ui->prop_trackbar->setRange(500,3999);
		temp_prop_div=1000;
		
		ui->Kp_trackbar->setRange(0,544);//(0.0277*10000)*2->primeira aproximação
		ui->Ki_trackbar->setRange(0,1000);
		ui->Kd_trackbar->setRange(0,1000);
		temp_k_div=10000;
		}
	// SATURATION	
	if(prop.property_id==4){
		ui->prop_trackbar->setRange(0,399902);
		temp_prop_div=1000;
		
		ui->Kp_trackbar->setRange(0,636);
		ui->Ki_trackbar->setRange(0,636);
		ui->Kd_trackbar->setRange(0,636);
		temp_k_div=100;
		}
	// WHITE_BALANCE	
	if(prop.property_id==5){
		ui->prop_trackbar->setRange(0,1023);
		temp_prop_div=1;

		ui->Kp_trackbar->setRange(0,1600);
		ui->Ki_trackbar->setRange(0,1600);
		ui->Kd_trackbar->setRange(0,1600);
		temp_k_div=100;
		}

    ui->Kp_trackbar->setValue(pid.p*temp_k_div);
    ui->Kp_value->setText(QString::number(pid.p,'f',3));
    
    ui->Kd_trackbar->setValue(pid.d*temp_k_div);
    ui->Kd_value->setText(QString::number(pid.d,'f',3));
    
    ui->Ki_trackbar->setValue(pid.i*temp_k_div);
    ui->Ki_value->setText(QString::number(pid.i,'f',3));
    
    ui->prop_trackbar->setValue((prop.value_a*temp_prop_div));
    ui->Prop_value->setText(QString::number(prop.value_a,'f',3));
    
    ui->label_kp_curr_value->setText(QString::number(pid.p));
    ui->label_kd_curr_value->setText(QString::number(pid.d));
    ui->label_ki_curr_value->setText(QString::number(pid.i));
}

/// \brief slot function for Kp trackbar. Sets a new value for the selected
/// label
/// \param value - new value to be applied to label's value
void MainWindow::on_Kp_trackbar_valueChanged(int value)
{
	ui->Kp_value->setText(QString::number(value/temp_k_div,'f',3));
}

/// \brief slot function for Kd trackbar. Sets a new value for the selected
/// label
/// \param value - new value to be applied to label's value
void MainWindow::on_Kd_trackbar_valueChanged(int value)
{
	ui->Kd_value->setText(QString::number(value/temp_k_div,'f',3));
}

/// \brief slot function for Ki trackbar. Sets a new value for the selected
/// label
/// \param value - new value to be applied to label's value
void MainWindow::on_Ki_trackbar_valueChanged(int value)
{
	ui->Ki_value->setText(QString::number(value/temp_k_div,'f',3));
}

/// \brief slot function for property value trackbar. Sets a new value for the selected
/// label
/// \param value - new value to be applied to label's value
void MainWindow::on_prop_trackbar_valueChanged(int value)
{
	ui->Prop_value->setText(QString::number(value/temp_prop_div,'f',3));
}

/// \brief callback function that receives published error messages
/// Since its called by ROS (another thread), emits a signal (addError) so
/// the GUI's thread can take care of other messages.
/// \param msg - error_msg::error data, holds information about the error
/// of the selected property.
void MainWindow::drawCallback(minho_team_ros::ControlerError msg)
{
	if(msg.property_id>=0 && msg.property_id<=6){
		if(calib){
			emit addError(msg.erro);
			ui->prop_trackbar->setValue((msg.value*temp_prop_div));
    			ui->Prop_value->setText(QString::number(msg.value,'f',3));
		}
	}
	else{
		on_combo_properties_currentIndexChanged(msg.property_id);
		ROS_ERROR("ID TO SET ERROR GOTTA A WRONG VALUE");
	}
}

void MainWindow::set_ROIs()
{
	
	requestROI srv;
	srv.request.is_set=false;
	
	if(!roiConf.call(srv))ROS_ERROR("Failed to retrieve ROI values from target robot."); 
	else{
			ROS_INFO("White ROI setted");
			ui->spinBox_x_w->setValue(srv.response.white.x);
			ui->spinBox_y_w->setValue(srv.response.white.y);
			ui->spinBox_d_w->setValue(srv.response.white.d);
			
			ROS_INFO("Black ROI setted");
			ui->spinBox_x_b->setValue(srv.response.black.x);
			ui->spinBox_y_b->setValue(srv.response.black.y);
			ui->spinBox_d_b->setValue(srv.response.black.d);
	}
}

void MainWindow::on_bt_ROIS_w_clicked()
{
	ROI msg;
	msg.x=ui->spinBox_x_w->value();
	msg.y=ui->spinBox_y_w->value();
	msg.d=ui->spinBox_d_w->value();
	msg.white=true;
	roi_pub_.publish(msg);
}

void MainWindow::on_bt_ROIS_b_clicked()
{
	ROI msg;
	msg.x=ui->spinBox_x_b->value();
	msg.y=ui->spinBox_y_b->value();
	msg.d=ui->spinBox_d_b->value();
	msg.white=false;
	roi_pub_.publish(msg);

}
