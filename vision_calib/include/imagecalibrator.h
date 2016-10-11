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
#include "types.h"
#include <iostream>

using namespace std;
using namespace cv;
using minho_team_ros::mirrorConfig;
using minho_team_ros::range;
using minho_team_ros::label;
using minho_team_ros::visionHSVConfig;

class ImageCalibrator
{
public:
    ImageCalibrator();
    void variablesInitialization();
    void getBinary(Mat *in, minho_team_ros::label labelconf); // Returns thresholded HSV image
    hsv rgbtohsv(rgb in); // Converts rgb to hsv values' ranges
    /* Other Variables and Lut Variables*/
    void updateCurrentConfiguration(LABEL_t label, COMPONENT_t component, RANGE_t range, int value);
    void lutConfigFromMsg(visionHSVConfig msg);
    minho_team_ros::label getLabelConfiguration(LABEL_t label);
    minho_team_ros::visionHSVConfig getLutConfiguration();
    /* Vision Variables */
    Mat element;
    Mat processed,buffer;
    visionHSVConfig lutconfig;
    mirrorConfig mirrorConf;
};

#endif // IMAGECALIBRATOR_H
