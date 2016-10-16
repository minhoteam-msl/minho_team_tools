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
   /// \brief - class constructor. Takes care of initializing GUI,
   /// connect several signals/slots and to create and configure
   /// all ROS publishers and subscribers.
   /// \params [in] - robot_id -> id [0-6] of the robot to use
   ///              - real_robot -> defines wether its a real or sim robot
    explicit MainWindow(int robot_id,bool real_robot, QWidget *parent = 0);
    /// \brief - class destructor. Shuts down ROS node
    ~MainWindow();
private slots:
   /// \brief function to setup the graphicsscene and create the 
   /// image holding matrix
   void setup();
   /// \brief function to detect key press events. This key presses 
   /// detection allow to implement several actions without mouse usage
   /// \params [in] : event -> QKeyEvent holding the data about the key press event  
   void keyPressEvent(QKeyEvent *event);
   /// \brief callback function that receives published images and converts them 
   /// into Opencv(Mat) format using cv_bridge and to Qt's QImage to be displayed.
   /// Since its called by ROS (another thread), emits a signal (addNewImage) so
   /// the GUI's thread can take care of putting the image in GraphicsView.
   /// \params [in] : msg -> sensor_msgs::Image data, holding information about
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
   /// \brief slot function for click action of check_draw. Enbles or disables 
   /// draw mode
   /// \params [in] : state -> boolean value to define if draw mode is on or off
   void on_check_draw_clicked(bool state);
   
   //SLIDEBARS
   /// \brief slot function for hue min trackbar. Sets a new value for the selected
   /// label
   /// \params [in] : value -> new value to be applied to label's hue min
   void on_h_min_valueChanged(int value);
   /// \brief slot function for hue max trackbar. Sets a new value for the selected
   /// label
   /// \params [in] : value -> new value to be applied to label's hue max
   void on_h_max_valueChanged(int value);
   /// \brief slot function for sat min trackbar. Sets a new value for the selected
   /// label
   /// \params [in] : value -> new value to be applied to label's sat min
   void on_s_min_valueChanged(int value);
   /// \brief slot function for sat max trackbar. Sets a new value for the selected
   /// label
   /// \params [in] : value -> new value to be applied to label's sat max
   void on_s_max_valueChanged(int value);
   /// \brief slot function for value min trackbar. Sets a new value for the selected
   /// label
   /// \params [in] : value -> new value to be applied to label's value min
   void on_v_min_valueChanged(int value);
   /// \brief slot function for value max trackbar. Sets a new value for the selected
   /// label
   /// \params [in] : value -> new value to be applied to label's value max
   void on_v_max_valueChanged(int value);
   
   //SPINBOXES
   /// \brief slot function for tilt spinbox. Sets a new value for the image 
   /// tilt
   /// \params [in] : value -> new value to be applied image's tilt
   void on_spin_tilt_valueChanged(int value);
   /// \brief slot function for center x spinbox. Sets a new value for the image 
   /// tilt
   /// \params [in] : value -> new value to be applied image's center x
   void on_spin_cx_valueChanged(int value);
   /// \brief slot function for center y spinbox. Sets a new value for the image 
   /// tilt
   /// \params [in] : value -> new value to be applied image's center y
   void on_spin_cy_valueChanged(int value);
   
   //COMBOBOXES
   /// \brief slot function for label's combobox. Sets the current label that
   /// we will be performing the configurations
   /// \params [in] : index -> new label index to be used
   void on_combo_label_currentIndexChanged(int index);
   /// \brief slot function for acquisition type's combobox. Sets the current 
   /// acquisition type to be used in image requests
   /// \params [in] : index -> new image acquisition type to be used
   void on_combo_aqtype_currentIndexChanged(int index);
   
   //LOAD FUNCTIONS
   /// \brief function to load values of a label configuration onto GUI's 
   /// trackbars
   /// \params [in] : labelconf -> label message containing label configuration
   void loadValuesOnTrackbars(minho_team_ros::label labelconf);
   /// \brief function to load values of a mirror configuration onto GUI's 
   /// spinboxes and line edit
   /// \params [in] : mirrorConf -> mirrorConfig message to be used
   void loadMirrorValues(minho_team_ros::mirrorConfig mirrorConf);
   /// \brief function to load values of a image configuration onto GUI's 
   /// spinboxes
   /// \params [in] : imageConf -> imageConfig message to be used
   void loadImageValues(minho_team_ros::imageConfig imageConf);
private:
   /// \brief pointer to MainWindow's GUI 
   Ui::MainWindow *ui;
   /// \brief robot's id that we are configuring
   int robot_id_;
   /// \brief boolean variables to define current working mode
   bool calibration_mode,draw_mode;
   /// \brief parent scene in QGraphicsView to draw on
   QGraphicsScene *scene_;
   /// \brief ImageCalibrator object, to do image operations and data
   /// (calibrations) storage
   ImageCalibrator *img_calib_;
   /// \brief Opencv image container to work binary conversions
   Mat temp;
   /// \brief QTimers to do binary and draw actions
   QTimer *img_calib_timer, *interaction_timer;
   //ROS
   /// \brief imgRequest ROS Publisher
   ros::Publisher imgreq_pub_;
   /// \brief mirrorConfig ROS Publisher
   ros::Publisher mirror_pub_;
   /// \brief visionHSVConfig ROS Publisher
   ros::Publisher vision_pub_;
   /// \brief imageConfig ROS Publisher
   ros::Publisher image_pub_;
   /// \brief requestOmniVisionConf ROS Service Client
   ros::ServiceClient omniVisionConf;
   /// \brief image_transport child node
   image_transport::ImageTransport *it_;
   /// \brief sensor_msgs::Image ROS Subscriver
   image_transport::Subscriber image_sub_;
   /// \brief ROS Node
   ros::NodeHandle *_node_;
   /// \brief imgRequest message
   imgRequest msg_;
   /// \brief QImage buffer to display in QGraphicsView
   QImage image_;

signals:
   void addNewImage();   
    
};

#endif // MAINWINDOW_H
