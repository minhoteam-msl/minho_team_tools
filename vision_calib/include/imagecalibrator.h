#ifndef IMAGECALIBRATOR_H
#define IMAGECALIBRATOR_H

#include <QFile>
#include <QString>
#include <QTextStream>
#include <QStringList>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "minho_team_ros/mirrorConfig.h"
#include "minho_team_ros/range.h"
#include "minho_team_ros/label.h"
#include "minho_team_ros/visionHSVConfig.h"
#include "minho_team_ros/imageConfig.h"
#include "types.h"
#include <iostream>
#define TO_RAD (M_PI/180.0)

using namespace std;
using namespace cv;
using minho_team_ros::mirrorConfig;
using minho_team_ros::range;
using minho_team_ros::label;
using minho_team_ros::visionHSVConfig;
using minho_team_ros::imageConfig;

class ImageCalibrator
{
public:
   /// \brief class constructor. Calls variablesInitializtion function
   ImageCalibrator();
   /// \brief function to initialize data. Creates Mats and morphing elements
   void variablesInitialization();
   /// \brief function to convert an rgb pixel into hsv in 0-180/0-255/0-255 range.
   /// \params [in] : in -> rgb struct containing rgb values to be converted
   /// \params [out] : hsv struct containing hsv values resultant from the conversion
   hsv rgbtohsv(rgb in);
   /// \brief function to return binary representation of an image given a certain
   /// hsv configuration (range).
   /// \params [in] : in -> pointer to target image (in and out image container)
   ///                labelconf -> hsv configuration for a certain label
   void getBinary(Mat *in, minho_team_ros::label labelconf);
   /// \brief function to update a certain range value in lutconfig message.
   /// \params [in] : label -> label to be changed
   ///                component -> component of label (h,s or v) to be changed
   ///                range -> range of component to be changed (min or max)
   ///                value -> value to be applied to range
   void updateCurrentConfiguration(LABEL_t label, COMPONENT_t component, RANGE_t range, int value);
   /// \brief function to get current label configuration
   /// \params [in] : label -> label's id to be get
   /// \params [out] : label -> label's configuration
   minho_team_ros::label getLabelConfiguration(LABEL_t label);
   /// \brief function to get current visionHSVConfig configuration
   /// \params [out] : visionHSVConfig -> current visionHSVConfig configuration
   minho_team_ros::visionHSVConfig getLutConfiguration();
   /// \brief function to get current imageConfig configuration
   /// \params [out] : imageConfig -> current imageConfig configuration
   minho_team_ros::imageConfig getImageConfiguration();
   /// \brief applies a configuration for lutconfig giver a visionHSVConfig message
   /// \params [in] : msg -> visionHSVConfig message to be used 
   void lutConfigFromMsg(visionHSVConfig msg);
   /// \brief applies a configuration for mirrorConf giver a mirrorConfig message
   /// \params [in] : msg -> mirrorConfig message to be used 
   void mirrorConfigFromMsg(mirrorConfig msg);
   /// \brief applies a configuration for imageConf giver a imageConfig message
   /// \params [in] : msg -> imageConfig message to be used    
   void imageConfigFromMsg(imageConfig msg);
   /// \brief function to draw a center point and a crosshair given the current
   /// imageConf configuration message
   /// \params [in] : image -> target image container where things will be drawn
   void drawCenter(Mat *image);
private:
   /* Vision Variables */
   Mat element;
   Mat processed,buffer;
   visionHSVConfig lutconfig;
   mirrorConfig mirrorConf;
   imageConfig imageConf;
};

#endif // IMAGECALIBRATOR_H
