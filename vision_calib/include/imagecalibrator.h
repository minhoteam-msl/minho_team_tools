#ifndef IMAGECALIBRATOR_H
#define IMAGECALIBRATOR_H

#include <QFile>
#include <QString>
#include <QTextStream>
#include <QStringList>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "types.h"
#include <iostream>

using namespace std;
using namespace cv;
class ImageCalibrator
{
public:
    ImageCalibrator();
    void variablesInitialization();
    void getBinary(Mat *in, labelConfiguration labelconf); // Returns thresholded HSV image
    hsv rgbtohsv(rgb in); // Converts rgb to hsv values' ranges
    /* Other Variables and Lut Variables*/
    void updateCurrentConfiguration(LABEL_t label, COMPONENT_t component, RANGE_t range, int value);
    labelConfiguration getLabelConfiguration(LABEL_t label);
    /* Vision Variables */
    Mat element;
    Mat processed,buffer;
    lutConfiguration lutconfig;
    Mat pix;
};

#endif // IMAGECALIBRATOR_H
