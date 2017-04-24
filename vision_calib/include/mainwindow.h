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
#include "minho_team_ros/requestOmniVisionConf.h"
#include "minho_team_ros/requestImage.h"
#include "minho_team_ros/requestCamProperty.h"
#include "minho_team_ros/requestCamPID.h"
#include "minho_team_ros/requestROI.h"
#include "minho_team_ros/ControlerError.h"
#include "minho_team_ros/ROI.h"
#include "minho_team_ros/worldConfig.h"
#include "minho_team_ros/position.h"
#include "imagecalibrator.h"
#include <QKeyEvent>
#include <QMessageBox>
#include "RLE.h"
#include "ScanLines.h"


using namespace ros;
using namespace cv;
using minho_team_ros::requestImage;
using minho_team_ros::requestOmniVisionConf;
using minho_team_ros::requestCamProperty;
using minho_team_ros::requestCamPID;
using minho_team_ros::requestROI;
using minho_team_ros::position;
using minho_team_ros::cameraProperty;
using minho_team_ros::PID;
using minho_team_ros::ControlerError;
using minho_team_ros::ROI;
using minho_team_ros::worldConfig;

#define ROS_MASTER_IP "http://172.16.49."
#define ROS_MASTER_PORT ":11311"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
   /// \brief class constructor. Takes care of initializing GUI,
   /// connect several signals/slots and to create and configure
   /// all ROS publishers and subscribers.
   /// \param robot_id - id [0-6] of the robot to use
   /// \param real_robot - defines wether its a real or sim robot
    explicit MainWindow(int robot_id,bool real_robot, QWidget *parent = 0);
    
    /// \brief class destructor. Shuts down ROS node
    ~MainWindow();
private slots:
   /// \brief function to setup the graphicsscene and create the 
   /// image holding matrix
   void setup();
   
   /// \brief function to detect key press events. This key presses 
   /// detection allow to implement several actions without mouse usage
   /// \param event - QKeyEvent holding the data about the key press event  
   void keyPressEvent(QKeyEvent *event);
   
   /// \brief function that allows to add a point to the mask contour with
   /// mouse lbutton and remove the last one with mouse rbutton
   void mousePressEvent(QMouseEvent *event);
   
   /// \brief callback function that receives published images and converts them 
   /// into Opencv(Mat) format using cv_bridge and to Qt's QImage to be displayed.
   /// Since its called by ROS (another thread), emits a signal (addNewImage) so
   /// the GUI's thread can take care of putting the image in GraphicsView.
   /// \param msg - sensor_msgs::Image data, holding information about
   /// the image sent.
   void imageCallback(const sensor_msgs::ImageConstPtr& msg);
   
   /// \brief slot function that consumes addNewImage signal. Only displays the 
   /// image if both calibration modes are disabled (calibration_mode and draw_mode)
   void addImageToScene();
   
   /// \brief function to apply binary HSV masking to the selected label, with the
   /// defined parameters for that label, in GUI's trackbars. The conversion is done
   /// based on the image present in temp matrix
   void applyBinary(); 
   
   /// \brief function to interact with the user and display info/images. If in draw
   /// mode it displays a calibration crosshair and, if in inspection mode, displays
   /// the mapped distance for the selected (mouse hovering) pixel
   void interactWithUser();
   
   //BUTTONS
   /// \brief slot function for click action of bt_grab. Function to send a request
   /// for images based on selected options of type and frequency
   void on_bt_grab_clicked();
   
   /// \brief slot function for click action of bt_stop. Function to stop image 
   /// send. It send a single image requst
   void on_bt_stop_clicked();
   
   /// \brief slot function for click action of bt_setdist. Checks validaty and
   /// sends new configuration for mirror's pixel-meter mapping
   void on_bt_setdist_clicked();
   
   /// \brief slot function for click action of bt_setlut. Sends new configuration
   /// for vision's HSV look up table
   void on_bt_setlut_clicked();
   
   /// \brief slot function for click action of bt_setimg. Sends new configuraiton
   /// for image parameters
   void on_bt_setimg_clicked();
   
   /// \brief slot function for click action of bt_screenshot. Takes and saves a
   /// screenshot in {ros_project_folder}/screenshots
   void on_bt_screenshot_clicked();
   
   /// \brief slot of function for click action of bt_calib. This enables or disables
   /// the calibration (LUT) of the received image
   void on_bt_calib_clicked();
   
   /// \brief slot function for click action of check_draw. Enbles or disables 
   /// draw mode
   /// \param state - boolean value to define if draw mode is on or off
   void on_check_draw_clicked(bool state);
   
   //SLIDEBARS
   /// \brief slot function for hue min trackbar. Sets a new value for the selected
   /// label
   /// \param value - new value to be applied to label's hue min
   void on_h_min_valueChanged(int value);
   
   /// \brief slot function for hue max trackbar. Sets a new value for the selected
   /// label
   /// \param value - new value to be applied to label's hue max
   void on_h_max_valueChanged(int value);
   
   /// \brief slot function for sat min trackbar. Sets a new value for the selected
   /// label
   /// \param value - new value to be applied to label's sat min
   void on_s_min_valueChanged(int value);
   
   /// \brief slot function for sat max trackbar. Sets a new value for the selected
   /// label
   /// \param value - new value to be applied to label's sat max
   void on_s_max_valueChanged(int value);
   
   /// \brief slot function for value min trackbar. Sets a new value for the selected
   /// label
   /// \param value - new value to be applied to label's value min
   void on_v_min_valueChanged(int value);
   
   /// \brief slot function for value max trackbar. Sets a new value for the selected
   /// label
   /// \param value - new value to be applied to label's value max
   void on_v_max_valueChanged(int value);
   
   //SPINBOXES
   /// \brief slot function for tilt spinbox. Sets a new value for the image 
   /// tilt
   /// \param value - new value to be applied image's tilt
   void on_spin_tilt_valueChanged(int value);
   
   /// \brief slot function for center x spinbox. Sets a new value for the image 
   /// tilt
   /// \param value - new value to be applied image's center x
   void on_spin_cx_valueChanged(int value);
   
   /// \brief slot function for center y spinbox. Sets a new value for the image 
   /// tilt
   /// \param value - new value to be applied image's center y
   void on_spin_cy_valueChanged(int value);
   
   //COMBOBOXES
   /// \brief slot function for label's combobox. Sets the current label that
   /// we will be performing the configurations
   /// \param index - new label index to be used
   void on_combo_label_currentIndexChanged(int index);
   
   /// \brief slot function for acquisition type's combobox. Sets the current 
   /// acquisition type to be used in image requests
   /// \param index - new image acquisition type to be used
   void on_combo_aqtype_currentIndexChanged(int index);
   
   //LOAD FUNCTIONS
   /// \brief function to load values of a label configuration onto GUI's 
   /// trackbars
   /// \param labelconf - label message containing label configuration
   void loadValuesOnTrackbars(minho_team_ros::label labelconf);
   
   /// \brief function to load values of a mirror configuration onto GUI's 
   /// spinboxes and line edit
   /// \param mirrorConf - mirrorConfig message to be used
   void loadMirrorValues(minho_team_ros::mirrorConfig mirrorConf);
   
   /// \brief function to load values of a image configuration onto GUI's 
   /// spinboxes
   /// \param imageConf - imageConfig message to be used
   void loadImageValues(minho_team_ros::imageConfig imageConf);
   
   /// \brief slot function for click action of bt_screenshot. Takes and saves a
   /// screenshot in {ros_project_folder}/screenshots
   void on_bt_screenshot_2_clicked();

   /*----------------------------------------------------------------------------------------------*/

   /// \brief slot function for tab change. Enables or disables 
   /// components necessary for each tab.
   void tabSelected();

   /// \brief slot function for click action of bt_campid. Sends
   /// the property PID Controler values
   void on_bt_campid_clicked();

   /// \brief slot function for click action of bt_camprop. Sends
   /// the property value
   void on_bt_camprop_clicked();

   /// \brief function for video activate when tabs change
   /// from 0 to 1
   void calibrate_activ();

   /// \brief slot function for properties type's combobox. Sets the current 
   /// property to be calibrated
   /// \param index - new property type to be used
   void on_combo_properties_currentIndexChanged(int index);

   /// \brief slot function for check action of radio_PID. Enables and disable
   /// some trackbars
   void on_radio_PID_clicked();

   /// \brief slot function for check action of radio_property. Enables and disable
   /// some trackbars
   void on_radio_property_clicked();

   /// \brief slot function for Kp trackbar. Sets a new value for the selected
   /// label
   /// \param value - new value to be applied to label's value
   void on_Kp_trackbar_valueChanged(int value);

   /// \brief slot function for Kd trackbar. Sets a new value for the selected
   /// label
   /// \param value - new value to be applied to label's value
   void on_Kd_trackbar_valueChanged(int value);

   /// \brief slot function for Ki trackbar. Sets a new value for the selected
   /// label
   /// \param value - new value to be applied to label's value
   void on_Ki_trackbar_valueChanged(int value);

   /// \brief slot function for property value trackbar. Sets a new value for the selected
   /// label
   /// \param value - new value to be applied to label's value
   void on_prop_trackbar_valueChanged(int value);

   /// \brief callback function that receives published error messages
   /// Since its called by ROS (another thread), emits a signal (addError) so
   /// the GUI's thread can take care of other messages.
   /// \param msg - error_msg::error data, holds information about the error
   /// of the selected property.
   void addValCallback(minho_team_ros::ControlerError msg);

   /// \brief function to set trackbar ranges of Kp,Ki,Kd,Prop
   /// \param prop - new cameraProperty that contains values to be loaded to trackbar of property
   /// \param pid - new PID that contains values to be loaded to trackbars of PID
   void set_track_ranges(cameraProperty prop,PID pid);

   /// \brief function to read properties values from pusblishers
   /// \param num - represents index of property
   void loadPropertiesValues(int num);

   /// \brief slot function for click action of bt_auto_cal. Sends
   /// signal to omniCamera to start the auto-calibration process
   void on_bt_auto_cal_clicked();

   /// \brief slot function for click action of bt_campid. Sends
   /// signal to reset PID controler of Properties of omniCamera
   void on_bt_reset_clicked();

   void on_bt_ROIS_w_clicked();
   void on_bt_ROIS_b_clicked();
   void set_ROIs();
   void on_bt_grab_2_clicked();
   void on_bt_stop_2_clicked();
   void on_bt_screenshot_3_clicked();
   void on_combo_label_2_currentIndexChanged(int index);
   void on_combo_aqtype_2_currentIndexChanged(int index);
   void on_bt_worldpar_clicked();
   void on_spin_framerate_3_valueChanged(double value);
   void on_spinBox_valueChanged(int value);
   void on_spin_framerate_5_valueChanged(int value);
   void on_spin_framerate_6_valueChanged(int value);
   void on_spin_framerate_7_valueChanged(int value);
   void on_spin_framerate_8_valueChanged(int value);
   void on_bt_RLE_clicked();
   void on_bt_lumi_clicked();
   void on_bt_sat_clicked();
   void on_sat_trackbar_valueChanged(int value);
   void on_lumi_trackbar_valueChanged(int value);
   void LoadTargetsCal();
   void loadKalmanValues();
   void on_bt_kalman_clicked();
   void on_Qxy_valueChanged(int value);
   void on_Qz_valueChanged(int value);
   void on_Rxy_valueChanged(int value);
   void on_Rz_valueChanged(int value);
   void InitializeLinesDetector();
   void on_check_draw_2_clicked(bool state);
   void on_check_draw_3_clicked(bool state);
   void on_filter_percentage_trackbar_valueChanged(int value);
   void on_check_draw_3_stateChanged(int value);
   void on_bt_setlengstr_clicked();
   void on_scanline_trackbar_valueChanged(int value);

private:
   /// \brief pointer to MainWindow's GUI 
   Ui::MainWindow *ui;
   /// \brief robot's id that we are configuring
   int robot_id_;
   /// \brief dividers to establish values correspondency
   float temp_prop_div,temp_k_div;
   /// \brief boolean variables to define current working mode
   bool calibration_mode,draw_mode,draw_scan,draw_rle;
   /// \brief parent scene in QGraphicsView to draw on
   QGraphicsScene *scene_,*scene_2, *scene_3;
   /// \brief ImageCalibrator object, to do image operations and data
   /// (calibrations) storage
   ImageCalibrator *img_calib_;
   /// \brief Opencv image container to work binary conversions
   Mat temp;
   /// \brief QTimers to do binary and draw actions and error plot
   QTimer *img_calib_timer, *interaction_timer;
   //ROS
   /// \brief mirrorConfig ROS Publisher
   ros::Publisher mirror_pub_;
   /// \brief visionHSVConfig ROS Publisher
   ros::Publisher vision_pub_;
   /// \brief imageConfig ROS Publisher
   ros::Publisher image_pub_;
   /// \brief cameraProperty ROS publisher
   ros::Publisher property_pub_;
   /// \brief PID ROS publisher
   ros::Publisher pid_pub_;
   /// \brief requestOmniVisionConf ROS Service Client
   ros::ServiceClient omniVisionConf;
   /// \brief requestImage ROS Service Client
   ros::ServiceClient imgRequest;
   /// \brief requestCamProperty ROS Service Client
   ros::ServiceClient propertyConf;
   /// \brief requestCamPID ROS Service Client
   ros::ServiceClient pidConf;
   /// \brief image_transport child node
   image_transport::ImageTransport *it_;
   /// \brief sensor_msgs::Image ROS Subscriver
   image_transport::Subscriber image_sub_;
   /// \brief error_msgs::Error ROS Subscriver
   ros::Subscriber error_sub_;
   /// \brief ROS Node
   ros::NodeHandle *_node_;
   
   ros::Publisher roi_pub_;
   ros::Publisher world_pub_;
   ros::ServiceClient roiConf;
   /// \brief QImage buffer to display in QGraphicsView
   QImage image_;

   bool calib;

   worldConfig worldConfBall, worldConfObs, worldConfRLE, kalmanConf;
   ScanLines linesRad;
   RLE rleLinesRad;
   Mat idxImage;

   vector<Point> maskContourPoints;

   QString dist_pix,line_pix;
   int scan_size;
signals:
   /// \brief signal to make the new image be drawn in GUI
   void addNewImage();
    
};

#endif // MAINWINDOW_H
